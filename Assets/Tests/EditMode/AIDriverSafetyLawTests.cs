using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Two safety laws in <see cref="AIDriverController"/> that were both wrong in a way no
/// other test could see, because in each case the code produced a perfectly legal number.
///
/// <para>
/// <b>S-A, the overtake clamp.</b> The branch that eases the traffic clamp blended it back
/// toward the unblocked racing-line speed and then guarded the result with
/// <c>Mathf.Min(eased, targetSpeedKmh)</c>. <c>Mathf.Lerp</c> for t in [0,1] never leaves
/// the interval it was given, and in traffic the racing-line speed is always above the
/// caution speed — so the guard was a no-op and the branch did exactly what the guard was
/// written to prevent: it raised the clamp, by up to
/// <c>overtakeTrafficSlowdownScale</c> of the way back to full racing speed, into the back
/// of the car being overtaken.
/// </para>
///
/// <para>
/// <b>S-B, the emergency brake.</b> The trigger distance was
/// <c>minGap + headway * v + margin * v</c> capped at the sensor's reach: linear in speed
/// and capped, so above about 40 km/h it returned the cap. Since <c>FrontDistance</c> is a
/// box-cast travel distance it can never exceed the reach, so the distance half of the
/// trigger was true on every blocked tick. A car running alongside a rival at 250 km/h was
/// told to brake at 0.85, and any follower closing on its leader at all tripped it.
/// </para>
///
/// <para>
/// Both are modelled here with a real <see cref="AIPerceptionSensor"/> driven by real
/// physics casts, mirroring the rig in <c>F1FoundationPlayModeTests</c> but simplified for
/// EditMode: no ground plane, no wheels, no drivetrain, and a high sensor stride so the
/// forced tick these tests set up is not overwritten by the throttle inside
/// <c>Simulate</c>.
/// </para>
/// </summary>
public class AIDriverSafetyLawTests
{
    /// <summary>Tolerance for km/h and metres. Well under any authored margin.</summary>
    private const float Epsilon = 0.01f;

    private readonly List<GameObject> _spawned = new List<GameObject>();
    private AIDifficultyProfile _difficulty;
    private AIPerceptionSensor _sensor;

    /// <summary>Readings taken straight after each <c>Simulate</c>, for the assertions.</summary>
    private float sensorLeaderSpeed;
    private float sensorClosingSpeed;
    private float sensorFrontDistance;

    [TearDown]
    public void TearDown()
    {
        for (int i = 0; i < _spawned.Count; i++)
        {
            if (_spawned[i] != null)
                Object.DestroyImmediate(_spawned[i]);
        }

        _spawned.Clear();

        if (_difficulty != null)
            Object.DestroyImmediate(_difficulty);
        _difficulty = null;
    }

    // ---------------------------------------------------------------------
    // S-A — an overtake eases the clamp, but only toward the car ahead.
    // ---------------------------------------------------------------------

    [Test]
    public void OvertakingNeverCommandsMoreThanTheLeaderPlusTheClosingAllowance()
    {
        AIDriverController driver;
        VehiclePhysicsCoordinator coordinator;
        Rigidbody rb;
        CreateRig(out driver, out _, out coordinator, out rb);

        // A leader 15 m up the road doing 36 km/h, and this car at 250 km/h — the case from
        // the defect report. On a Hard car the unblocked target is far above both.
        CreateLeader("Slow Leader", new Vector3(0f, 0f, 17.8f), 36f);
        SetVelocity(rb, Vector3.forward * (250f / 3.6f));

        // First without the overtake decision, to establish what the full caution gives.
        driver.overtakeTrigger = 0.99f;
        Step(coordinator, driver);
        Assert.IsFalse(driver.IsOvertaking, "The baseline run is not supposed to be overtaking.");
        float withoutOvertake = driver.LastSpeedTargetKmh;
        float lineTarget = driver.LastTargetSpeedKmh;
        Assert.Greater(lineTarget, withoutOvertake + 50f,
            "The scenario has to actually have traffic to clamp against, otherwise it " +
            "measures nothing. Line target " + lineTarget + " km/h, clamped to " +
            withoutOvertake + " km/h.");

        // Then with it. Same car, same leader, same distance.
        driver.overtakeTrigger = 0.2f;
        Step(coordinator, driver);
        Assert.IsTrue(driver.IsOvertaking,
            "A Hard driver with a clear lane beside a car in front has to decide to pass.");
        float overtaking = driver.LastSpeedTargetKmh;

        float leaderKmh = sensorLeaderSpeed;
        Assert.LessOrEqual(overtaking, leaderKmh + driver.followMaxClosingKmh + Epsilon,
            "A car that has committed to a pass must not be allowed to close on the car it " +
            "is passing faster than the following law's own closing allowance. Leader " +
            leaderKmh.ToString("F1") + " km/h, allowance " +
            driver.followMaxClosingKmh.ToString("F1") + " km/h, so the ceiling is " +
            (leaderKmh + driver.followMaxClosingKmh).ToString("F1") +
            " km/h but the overtaker commanded " + overtaking.ToString("F1") + " km/h. " +
            "The old branch blended toward the unblocked racing-line speed here, which is " +
            "the speed of a road with nothing on it.");

        Assert.LessOrEqual(overtaking, lineTarget + Epsilon,
            "The clamp result must still be held to the line's target speed.");

        // The ease has to survive the fix. Guarding the branch so it can never raise the
        // clamp would satisfy the bound above by making the overtake change the lane and
        // nothing else — which is the earlier bug, wearing the other hat.
        Assert.Greater(overtaking, withoutOvertake + Epsilon,
            "Deciding to pass must still ease the traffic clamp. It went from " +
            withoutOvertake.ToString("F1") + " km/h to " + overtaking.ToString("F1") +
            " km/h; a bound that lands back on the un-overtaken speed has removed the " +
            "ease instead of bounding it.");
    }

    [Test]
    public void OvertakingWithAnUnknownLeaderSpeedLeavesTheCautionWhereTheFollowLawPutIt()
    {
        AIDriverController driver;
        VehiclePhysicsCoordinator coordinator;
        Rigidbody rb;
        CreateRig(out driver, out _, out coordinator, out rb);

        // A leader that is stopped dead. The sensor can see it and knows it is a body, so the
        // following law runs — but its speed reads zero, and a leader whose speed is unknown
        // must not authorise closing any faster.
        CreateLeader("Stationary Leader", new Vector3(0f, 0f, 17.8f), 0f);
        SetVelocity(rb, Vector3.forward * (250f / 3.6f));

        driver.overtakeTrigger = 0.99f;
        Step(coordinator, driver);
        float withoutOvertake = driver.LastSpeedTargetKmh;

        driver.overtakeTrigger = 0.2f;
        Step(coordinator, driver);
        float overtaking = driver.LastSpeedTargetKmh;

        Assert.LessOrEqual(overtaking, withoutOvertake + Epsilon,
            "With no leader-speed reading there is nothing to bound an eased clamp against, " +
            "so the overtake must not raise it at all. The caution went from " +
            withoutOvertake.ToString("F1") + " km/h to " + overtaking.ToString("F1") +
            " km/h, which is the same defect as raising it toward the racing line — the " +
            "speed just happens to have come from the wrong bound.");
    }

    [Test]
    public void SideOverlapContactStillOverridesTheFollowClampAndTheLineSpeedStillCaps()
    {
        AIDriverController driver;
        VehiclePhysicsCoordinator coordinator;
        Rigidbody rb;
        CreateRig(out driver, out _, out coordinator, out rb);

        // A leader 15 m ahead doing 36 km/h, which on its own holds this car to about
        // 22 km/h, and a rival already alongside at contact range.
        CreateLeader("Overlap Leader", new Vector3(0f, 0f, 17.8f), 36f);
        CreateLeader("Contact Beside", new Vector3(2.6f, 0f, 0f), 36f);
        SetVelocity(rb, Vector3.forward * (250f / 3.6f));
        driver.overlapEscapeMinSpeedKmh = 40f;

        // Overtake disabled: the side-overlap escape cancels any overtake anyway, and
        // this half of the test is about the floor that runs after it.
        driver.overtakeTrigger = 0.99f;

        Step(coordinator, driver);
        // LastFollowTargetKmh is the follow law's own answer, recorded before the overlap
        // floor runs. LastSpeedTargetKmh is already the floor's, so asserting against it
        // would just be asserting the floor against itself.
        Assert.Less(driver.LastFollowTargetKmh + Epsilon, driver.overlapEscapeMinSpeedKmh,
            "The scenario needs the follow clamp to be tighter than the overlap floor, or " +
            "the floor proves nothing. Follow clamp was " +
            driver.LastFollowTargetKmh.ToString("F1") + " km/h against a " +
            driver.overlapEscapeMinSpeedKmh.ToString("F1") + " km/h floor.");
        Assert.GreaterOrEqual(driver.LastSpeedTargetKmh,
            driver.overlapEscapeMinSpeedKmh - Epsilon,
            "A car already touching a rival cannot be following it — the clamp collapses to " +
            "a standstill and neither car can restart. The floor must lift it to " +
            driver.overlapEscapeMinSpeedKmh.ToString("F1") + " km/h; it is at " +
            driver.LastSpeedTargetKmh.ToString("F1") + " km/h.");

        // The other half: the floor is a floor, not a grant. Drop the line's own speed below
        // it and the target must come back down to the line speed rather than sitting at 40.
        SetAllLineSpeeds(driver.racingLine, 30f);
        Step(coordinator, driver);

        Assert.LessOrEqual(driver.LastSpeedTargetKmh, driver.LastTargetSpeedKmh + Epsilon,
            "The speed target must still be held to the racing line's own target. Line " +
            driver.LastTargetSpeedKmh.ToString("F1") + " km/h, target " +
            driver.LastSpeedTargetKmh.ToString("F1") + " km/h.");
        Assert.Less(driver.LastSpeedTargetKmh, driver.overlapEscapeMinSpeedKmh,
            "With the line itself slower than the overlap floor, the line wins. Target is " +
            driver.LastSpeedTargetKmh.ToString("F1") + " km/h.");
    }

    // ---------------------------------------------------------------------
    // S-B — the emergency brake is a hazard test at every speed.
    // ---------------------------------------------------------------------

    [Test]
    public void EmergencyTriggerDistanceGrowsWithSpeedAndNeverFlattens()
    {
        AIDriverController driver;
        CreateRig(out driver, out _, out _, out _);

        float previous = EmergencyDistanceMetres(driver, 0f);
        for (float kmh = 5f; kmh <= 300f; kmh += 5f)
        {
            float distance = EmergencyDistanceMetres(driver, kmh);
            Assert.GreaterOrEqual(distance, previous - Epsilon,
                "The trigger distance must not shrink as the car speeds up. At " +
                kmh.ToString("F1") + " km/h it was " + distance.ToString("F2") +
                " m, below the " + previous.ToString("F2") + " m it needed at " +
                (kmh - 5f).ToString("F1") + " km/h.");
            previous = distance;
        }

        float atRacing = EmergencyDistanceMetres(driver, 40f);
        float atTopSpeed = EmergencyDistanceMetres(driver, 300f);

        // The old law was min(6 + 2.3 * v, reach): it saturated at the sensor's 32 m above
        // about 40 km/h and returned 32 for everything from there to 300. This is the one
        // assertion that separates "grows with speed" from "is a constant".
        Assert.Greater(atTopSpeed, atRacing + 1f,
            "Above racing speed the trigger must keep opening out. It was " +
            atRacing.ToString("F2") + " m at 40 km/h and " + atTopSpeed.ToString("F2") +
            " m at 300 km/h — the trigger is a constant there, so it is not a hazard test, " +
            "it is a presence test.");
    }

    [Test]
    public void EmergencyBrakeDoesNotFireForASlowClosingRateAtRacingSpeed()
    {
        AIDriverController driver;
        VehiclePhysicsCoordinator coordinator;
        Rigidbody rb;
        CreateRig(out driver, out _, out coordinator, out rb);

        // A leader 30 m up the road at 244 km/h while this car runs at 250 — a closing rate
        // of 6 km/h. The gap lasts about 17 seconds. That is a queue, and the following law
        // is the right tool for it; a 0.85 brake application here is a car stopping dead on
        // a straight because it can see a rival.
        Rigidbody leader = CreateLeader("Fast Leader", new Vector3(0f, 0f, 32f), 244f);
        SetVelocity(rb, Vector3.forward * (250f / 3.6f));

        Step(coordinator, driver);
        Assert.Less(sensorClosingSpeed, driver.emergencyClosingKmh + 2f,
            "The scenario is only meaningful if the closing rate is small. Closing " +
            sensorClosingSpeed.ToString("F1") + " km/h.");

        Assert.AreNotEqual(AISpeedClampReason.EmergencyBrake, driver.LastSpeedClampReason,
            "Closing at " + sensorClosingSpeed.ToString("F1") + " km/h with " +
            driver.LastEmergencyDistance.ToString("F1") +
            " m of trigger distance, the emergency brake applied anyway. A follower that is " +
            "closing on its leader at all must not be treated as about to hit it, or every " +
            "convoy brakes to a halt.");

        // And again at the old flat threshold's exact boundary, 4 km/h of closing.
        SetVelocity(leader, Vector3.forward * (246f / 3.6f));
        Step(coordinator, driver);
        Assert.AreNotEqual(AISpeedClampReason.EmergencyBrake, driver.LastSpeedClampReason,
            "Same at the boundary closing rate of " + driver.emergencyClosingKmh.ToString("F1") +
            " km/h: closing " + sensorClosingSpeed.ToString("F1") + " km/h.");
    }

    [Test]
    public void EmergencyBrakeStillFiresAtContactRangeEvenWithAlmostNoClosingRate()
    {
        AIDriverController driver;
        VehiclePhysicsCoordinator coordinator;
        Rigidbody rb;
        CreateRig(out driver, out _, out coordinator, out rb);

        // The other half of the requirement. A car matching its leader's speed 6 m behind is
        // in exactly the same position as one closing at 40 km/h into it — the brake that
        // helps is the same brake — so the trigger must not be a closing-rate test alone.
        Rigidbody leader = CreateLeader("Leader At Contact Range", new Vector3(0f, 0f, 7.6f), 247f);
        SetVelocity(rb, Vector3.forward * (250f / 3.6f));

        Step(coordinator, driver);
        Assert.LessOrEqual(sensorFrontDistance, driver.emergencyMinGap + Epsilon,
            "The scenario needs the leader inside the minimum gap, or contact range proves " +
            "nothing. Measured " + sensorFrontDistance.ToString("F2") + " m against a " +
            driver.emergencyMinGap.ToString("F1") + " m minimum.");
        Assert.Less(sensorClosingSpeed, driver.emergencyClosingKmh,
            "The scenario needs the closing rate to be below the deadband. Closing " +
            sensorClosingSpeed.ToString("F1") + " km/h.");

        Assert.AreEqual(AISpeedClampReason.EmergencyBrake, driver.LastSpeedClampReason,
            "A car 6 m ahead at 250 km/h is a contact whatever the closing rate is, and the " +
            "emergency brake must still say so. Closing only " +
            sensorClosingSpeed.ToString("F1") + " km/h.");

        // And with the leader stopped dead — the largest closing rate there is.
        SetVelocity(leader, Vector3.zero);
        Step(coordinator, driver);
        Assert.AreEqual(AISpeedClampReason.EmergencyBrake, driver.LastSpeedClampReason,
            "A stationary car 6 m ahead at 250 km/h is the clearest emergency there is.");
    }

    // ---------------------------------------------------------------------
    // Rig
    // ---------------------------------------------------------------------

    private GameObject CreateRig(
        out AIDriverController driver,
        out AIRacingLine line,
        out VehiclePhysicsCoordinator coordinator,
        out Rigidbody rb)
    {
        GameObject root = new GameObject("AI Safety Law Rig");
        _spawned.Add(root);

        GameObject lineObject = new GameObject("Safety Test Racing Line");
        lineObject.transform.SetParent(root.transform);
        line = lineObject.AddComponent<AIRacingLine>();
        line.loop = false;
        for (int i = 0; i < 4; i++)
            CreateWaypoint(lineObject.transform, "WP_0" + i, new Vector3(0f, 0f, i * 28f), 240f);
        line.RefreshWaypoints();

        GameObject car = new GameObject("AI Safety Test Car");
        car.transform.SetParent(root.transform);
        car.transform.position = Vector3.zero;
        rb = car.AddComponent<Rigidbody>();
        rb.useGravity = false;
        car.AddComponent<BoxCollider>().size = new Vector3(2f, 1f, 4f);
        coordinator = car.AddComponent<VehiclePhysicsCoordinator>();
        _sensor = car.AddComponent<AIPerceptionSensor>();
        driver = car.AddComponent<AIDriverController>();

        coordinator.applyProfileOnAwake = false;
        coordinator.rb = rb;
        coordinator.UseExternalInput = true;

        // 32 m is the AI prefab's own reach, and it is the number the old trigger saturated
        // against, so the rig keeps it rather than flattering the fix with a longer sensor.
        _sensor.forwardDistance = 32f;
        _sensor.sideDistance = 6f;
        // A long stride and a long minimum interval so the Tick inside Simulate is a no-op
        // and cannot overwrite the reading these tests set up deliberately.
        _sensor.fixedFrameStride = 12;
        _sensor.minimumUpdateInterval = 0.25f;

        driver.coordinator = coordinator;
        driver.racingLine = line;
        driver.perception = _sensor;
        driver.baseLookaheadDistance = 18f;
        driver.lookaheadPerKmh = 0f;
        driver.laneVariationStrength = 0f; // determinism
        driver.overtakeTrafficSlowdownScale = 0.4f; // the prefab's value
        driver.stuckDetectionSeconds = 30f; // keep recovery out of these scenarios

        _difficulty = AIDifficultyProfile.CreateRuntimeProfile(AIDifficultyPreset.Hard);
        driver.difficultyProfile = _difficulty;

        coordinator.RefreshState();
        return root;
    }

    private Rigidbody CreateLeader(string name, Vector3 position, float speedKmh)
    {
        GameObject leader = GameObject.CreatePrimitive(PrimitiveType.Cube);
        leader.name = name;
        // Root level on purpose: the sensor filters out colliders that are part of the car
        // it belongs to, and a leader parented under the rig would be filtered as self.
        leader.transform.position = position;
        leader.transform.localScale = new Vector3(2f, 1f, 4f);
        Rigidbody body = leader.AddComponent<Rigidbody>();
        body.useGravity = false;
        SetVelocity(body, Vector3.forward * (speedKmh / 3.6f));
        _spawned.Add(leader);
        return body;
    }

    private AIRacingWaypoint CreateWaypoint(Transform parent, string name, Vector3 position, float targetSpeedKmh)
    {
        GameObject waypointObject = new GameObject(name);
        waypointObject.transform.SetParent(parent);
        waypointObject.transform.position = position;
        AIRacingWaypoint waypoint = waypointObject.AddComponent<AIRacingWaypoint>();
        waypoint.targetSpeedKmh = targetSpeedKmh;
        waypoint.laneWidth = 7f;
        waypoint.overtakeWidth = 4f;
        return waypoint;
    }

    private void SetAllLineSpeeds(AIRacingLine line, float targetSpeedKmh)
    {
        for (int i = 0; i < line.Count; i++)
            line.GetWaypoint(i).targetSpeedKmh = targetSpeedKmh;
        line.RefreshWaypoints();
    }

    private void Step(VehiclePhysicsCoordinator coordinator, AIDriverController driver)
    {
        Physics.SyncTransforms();
        _sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();

        sensorLeaderSpeed = _sensor.LeaderSpeedKmh;
        sensorClosingSpeed = _sensor.ClosingSpeedKmh;
        sensorFrontDistance = _sensor.FrontDistance;
    }

    private static void SetVelocity(Rigidbody body, Vector3 velocity)
    {
#if UNITY_6000_0_OR_NEWER
        body.linearVelocity = velocity;
#else
        body.velocity = velocity;
#endif
    }

    /// <summary>
    /// Reads the private trigger distance. It is private, and the alternative — asserting
    /// only on the brake it produces — cannot see saturation: a constant trigger and a
    /// growing one both fire on a car 3 m away.
    /// </summary>
    private static float EmergencyDistanceMetres(AIDriverController driver, float speedKmh)
    {
        MethodInfo method = typeof(AIDriverController).GetMethod(
            "GetEmergencyDistanceMetres",
            BindingFlags.Instance | BindingFlags.NonPublic);

        Assert.IsNotNull(method,
            "AIDriverController no longer has a private GetEmergencyDistanceMetres(float). " +
            "If it was renamed these tests need updating with it.");

        return (float)method.Invoke(driver, new object[] { speedKmh });
    }
}