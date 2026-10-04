using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Tests for the planner that decides the speed an AI car may carry right now.
///
/// <para>
/// <see cref="AISpeedProfileTests"/> covers the arithmetic in isolation. These cover the part
/// that could not be tested there, because it needs a car and a line: that the horizon really is
/// long enough to see a corner, that it really is the minimum over that horizon rather than the
/// next waypoint, and that the braking budget is derived from the force clamp the simulation
/// actually enforces rather than from an assumed figure.
/// </para>
///
/// <para>
/// The defect these exist to prevent: the controller targeted the speed of the single waypoint
/// ahead — about 11 m on the baked Track_01 line. A planner that merely re-reads that waypoint,
/// or whose horizon is shorter than the braking distance, passes every test that does not
/// measure the horizon length against the car's real speed.
/// </para>
/// </summary>
public class AILongitudinalPlannerTests
{
    private const float KmhToMs = 1f / 3.6f;

    /// <summary>A figure the measured budget can plausibly land near on this physics.</summary>
    private const float BrakingDecel = 25f;

    private static float Ms(float kmh) => kmh * KmhToMs;

    /// <summary>Mean segment spacing on the baked Track_01 line, metres. Measured, not assumed:
    /// the plan records it as 11.17-11.33 m, and a horizon sized in waypoints depends on it.</summary>
    private const float MeanSegmentMetres = 11.33f;

    /// <summary>
    /// The speed a car may carry at the origin if a single constraint of
    /// <paramref name="limitKmh"/> sits <paramref name="distanceMetres"/> ahead, given
    /// <see cref="BrakingDecel"/>. This is the solver's own relation, restated here so a test can
    /// say what a fixture's geometry permits without depending on the implementation it checks.
    /// </summary>
    private static float ReachableFrom(float limitKmh, float distanceMetres)
    {
        float limitMs = Ms(limitKmh);
        return Mathf.Sqrt(limitMs * limitMs + 2f * BrakingDecel * distanceMetres) / KmhToMs;
    }

    private readonly System.Collections.Generic.List<GameObject> _spawned = new();

    [TearDown]
    public void TearDown()
    {
        foreach (GameObject go in _spawned)
        {
            if (go != null)
                Object.DestroyImmediate(go);
        }

        _spawned.Clear();
    }

    // --------------------------------------------------------------- the horizon

    [Test]
    public void TheHorizonIsLongEnoughToSeeACornerTheCarCannotYetBeBrakingFor()
    {
        // The measured situation on Track_01: 240 km/h, and a 90 km/h corner far enough ahead
        // that the plan must already be pulling the target down.
        //
        // The check is deliberately on the HORIZON rather than on the returned speed, because a
        // planner that returns the right number by looking at one waypoint would pass a speed
        // assertion and still be the original bug. What matters is that the plan LOOKED at the
        // corner at all.
        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        AIRacingLine line = BuildLine(out GameObject root, straightSpeedKmh: 240f,
            cornerIndex: 8, cornerIndex2: 9, cornerSpeedKmh: 90f, spacing: 11.2f);

        planner.SolvePlannedSpeedKmh(
            line, 0, Vector3.zero, 240f, BrakingDecel,
            1f, 1f, 0f,
            minimumHorizonMetres: 45f, maximumWaypoints: 24, fallbackKmh: 240f);

        Assert.Greater(planner.LastHorizonMetres, 100f,
            $"At 240 km/h with a {BrakingDecel} m/s^2 budget the braking distance is " +
            $"{240f / 3.6f * 240f / 3.6f / (2f * BrakingDecel):F0} m, so the horizon must cover " +
            $"more than 100 m to see a corner in time. The planner only looked " +
            $"{planner.LastHorizonMetres:F1} m, which cannot brake for anything on this line.");

        Object.DestroyImmediate(root);
    }

    [Test]
    public void ASlowCarGetsAShortHorizonRatherThanWalkingTheWholeTrack()
    {
        // The horizon is a braking distance, not a waypoint count. If it were a count, a car
        // crawling at 40 km/h would still scan the full 24-waypoint budget and pay for it every
        // tick at 100 Hz.
        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        AIRacingLine line = BuildLine(out GameObject root, straightSpeedKmh: 240f,
            cornerIndex: 20, cornerIndex2: 21, cornerSpeedKmh: 90f, spacing: 11.2f);

        planner.SolvePlannedSpeedKmh(
            line, 0, Vector3.zero, 40f, BrakingDecel,
            1f, 1f, 0f,
            minimumHorizonMetres: 45f, maximumWaypoints: 24, fallbackKmh: 240f);

        // 40 km/h is 11.1 m/s: a braking distance of ~2.5 m, well under the 45 m floor, so the
        // floor governs and the horizon stays short even though a corner sits 20 waypoints out.
        Assert.AreEqual(45f, planner.LastHorizonMetres, 0.5f,
            "A slow car's horizon should be governed by the minimum, not by how far down the " +
            "line the next corner happens to be.");

        Assert.Less(planner.LastHorizonWaypointCount, 12,
            $"A 45 m horizon over 11.2 m segments is about four waypoints. The planner scanned " +
            $"{planner.LastHorizonWaypointCount}, which means the horizon is not being applied.");

        Object.DestroyImmediate(root);
    }

    // ------------------------------------------------- the minimum, not the next waypoint

    [Test]
    public void ASlowCornerFurtherAlongBeatsAFastWaypointImmediatelyAhead()
    {
        // This is the whole defect in one fixture. The next waypoint is 11 m away at 240 km/h and
        // permits far more than 240; the corner is 90 m away at 90 km/h and permits much less.
        // A planner that reads only the next waypoint returns 240 here. A planner that takes the
        // minimum returns something well below it.
        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        AIRacingLine line = BuildLine(out GameObject root, straightSpeedKmh: 240f,
            cornerIndex: 4, cornerIndex2: 5, cornerSpeedKmh: 90f, spacing: 11.2f);

        float allowed = planner.SolvePlannedSpeedKmh(
            line, 0, Vector3.zero, 240f, BrakingDecel,
            1f, 1f, 0f,
            minimumHorizonMetres: 45f, maximumWaypoints: 24, fallbackKmh: 240f);

        float fromNextWaypointOnly = ReachableFrom(240f, 11.2f);
        float fromCornerOnly = ReachableFrom(90f, 4f * 11.2f);

        Assert.Less(fromCornerOnly, fromNextWaypointOnly,
            "Precondition: the corner must actually govern this approach, or the test measures " +
            "nothing. Over 45 m at 25 m/s^2 a 90 km/h corner permits " +
            fromCornerOnly.ToString("F0") + " km/h against the straight's " +
            fromNextWaypointOnly.ToString("F0") + " km/h.");

        Assert.AreEqual(fromCornerOnly, allowed, 1f,
            "The answer must be the MINIMUM over the horizon. The corner alone permits " +
            fromCornerOnly.ToString("F0") + " km/h and the next waypoint alone permits " +
            fromNextWaypointOnly.ToString("F0") + " km/h, so a planner taking the minimum " +
            "returns the former. It returned " + allowed.ToString("F0") +
            " km/h, which is near the single-waypoint figure, so it is still reading one " +
            "waypoint rather than the horizon. The corner at 45 m governs this approach.");

        Object.DestroyImmediate(root);
    }

    [Test]
    public void TheAllowedSpeedNeverExceedsTheTightestCornerInsideTheHorizon()
    {
        // Whatever the geometry, the answer may not be faster than the slowest corner the plan
        // actually looked at — that is the invariant the backward pass exists to hold.
        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        AIRacingLine line = BuildLine(out GameObject root, straightSpeedKmh: 300f,
            cornerIndex: 3, cornerIndex2: 4, cornerSpeedKmh: 85f, spacing: 11.2f);

        float allowed = planner.SolvePlannedSpeedKmh(
            line, 0, Vector3.zero, 300f, BrakingDecel,
            1f, 1f, 0f,
            minimumHorizonMetres: 45f, maximumWaypoints: 24, fallbackKmh: 300f);

        float cornerPermits = ReachableFrom(85f, 3f * 11.2f);

        Assert.AreEqual(cornerPermits, allowed, 1f,
            "The plan saw an 85 km/h corner " + (3f * 11.2f).ToString("F0") + " m away (" +
            planner.LastHorizonWaypointCount + " waypoints, " +
            planner.LastHorizonMetres.ToString("F0") + " m horizon), which permits " +
            cornerPermits.ToString("F0") + " km/h. It returned " + allowed.ToString("F0") +
            " km/h. The allowed speed is the minimum over the horizon; anything above the " +
            "tightest constraint in it means a nearer waypoint is masking a slower one behind it.");

        Assert.Less(allowed, ReachableFrom(300f, 11.2f),
            "Precondition: the corner must bind harder than the straight's own first waypoint, " +
            "otherwise this test would pass even for a planner that ignored the horizon.");

        Object.DestroyImmediate(root);
    }

    [Test]
    public void ABrakeBudgetOfZeroStillHonoursTheCornerSpeedItCanSee()
    {
        // With no braking capability the formula degenerates to the corner speed itself, which
        // is the correct answer: a car that cannot slow can only travel as fast as the slowest
        // thing ahead of it allows. It must not return infinity or the straight-line speed.
        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        AIRacingLine line = BuildLine(out GameObject root, straightSpeedKmh: 240f,
            cornerIndex: 4, cornerIndex2: 5, cornerSpeedKmh: 70f, spacing: 11.2f);

        float allowed = planner.SolvePlannedSpeedKmh(
            line, 0, Vector3.zero, 240f, 0f,
            1f, 1f, 0f,
            minimumHorizonMetres: 45f, maximumWaypoints: 24, fallbackKmh: 240f);

        // The budget is floored at 0.5 m/s^2 rather than allowed to reach exactly zero, which is
        // what keeps the horizon arithmetic finite; the corner therefore permits very slightly
        // more than its own speed over the distance to reach it. Anything near the straight's
        // 240 km/h means the zero was read as unbounded acceleration instead of as an inability
        // to shed speed.
        Assert.Less(allowed, 90f,
            $"A car that cannot brake may only carry roughly the corner speed. It was allowed " +
            $"{allowed:F0} km/h, so the zero budget is being read as unbounded acceleration " +
            "rather than as no ability to shed speed.");

        Assert.GreaterOrEqual(allowed, 70f,
            $"The answer must not fall BELOW the corner speed either — the corner is a limit, " +
            $"not a target to undershoot. It allowed {allowed:F0} km/h against a 70 km/h corner.");

        Object.DestroyImmediate(root);
    }

    // --------------------------------------------------------- the measured budget

    [Test]
    public void ACarWithNoGroundedWheelsFallsBackRatherThanBelievingZeroGrip()
    {
        // Airborne, or a spawn frame before the suspension has taken up the car's weight: the
        // measured total load is ~0, and planning against that would hand back an enormous
        // allowed speed. The fallback is a known-plausible figure, not the measurement.
        GameObject root = new GameObject("Budget Fallback Rig");
        _spawned.Add(root);

        GameObject car = new GameObject("Car");
        car.transform.SetParent(root.transform);
        Rigidbody rb = car.AddComponent<Rigidbody>();
        rb.useGravity = false;
        VehiclePhysicsCoordinator coordinator = car.AddComponent<VehiclePhysicsCoordinator>();
        coordinator.applyProfileOnAwake = false;
        coordinator.rb = rb;

        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        float measured = planner.MeasureBrakeDecelMs2(coordinator, 18f);

        Assert.AreEqual(18f, measured, 0.001f,
            "With no grounded wheels the clamp cannot be evaluated, so the authored fallback " +
            "must be used. Measuring it anyway would plan against a near-zero budget and " +
            "return an unbounded allowed speed on the frame the car leaves the ground.");
    }

    [Test]
    public void TheBudgetIsTheSumOfThePerWheelClampDividedByMass()
    {
        // The measurement mirrors the clamp in DrivetrainBrakeSystem.ApplyForceAtWheel:
        //   force <= NormalForce * tireForceLimitMultiplier   (per grounded wheel)
        // so the deceleration ceiling is the sum of those clamps over mass. This pins the
        // formula rather than a value, because a value would rot the moment the car's mass or
        // the multiplier changed.
        GameObject root = new GameObject("Budget Formula Rig");
        _spawned.Add(root);

        GameObject car = new GameObject("Car");
        car.transform.SetParent(root.transform);
        Rigidbody rb = car.AddComponent<Rigidbody>();
        rb.useGravity = false;
        rb.mass = 1000f;

        VehiclePhysicsCoordinator coordinator = car.AddComponent<VehiclePhysicsCoordinator>();
        coordinator.applyProfileOnAwake = false;
        coordinator.rb = rb;
        coordinator.drivetrain = car.AddComponent<DrivetrainBrakeSystem>();
        coordinator.drivetrain.tireForceLimitMultiplier = 1.6f;

        // Four grounded wheels carrying 2000 N each is 8000 N of load. At 1.6x that is 12,800 N
        // of permitted brake force, which over 1000 kg is 12.8 m/s^2.
        RaycastWheel[] wheels = new RaycastWheel[4];
        for (int i = 0; i < 4; i++)
        {
            GameObject wheelObject = new GameObject($"Wheel{i}");
            wheelObject.transform.SetParent(car.transform);
            wheels[i] = wheelObject.AddComponent<RaycastWheel>();
            wheels[i].NormalForce = 2000f;
            wheels[i].IsGrounded = true;
        }

        coordinator.wheels = wheels;

        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        float measured = planner.MeasureBrakeDecelMs2(coordinator, 18f);

        Assert.AreEqual(12.8f, measured, 0.01f,
            "The budget must be the sum over grounded wheels of NormalForce x " +
            "tireForceLimitMultiplier, divided by mass — the exact force the drivetrain will " +
            "permit. Got " + measured.ToString("F2") + " m/s^2.");
    }

    [Test]
    public void AnUngroundedWheelDoesNotContributeToTheBudget()
    {
        // Three wheels on the ground is not the same car as four. Summing every wheel
        // regardless would credit the plan with grip the car does not have — the whole point of
        // measuring rather than assuming.
        GameObject root = new GameObject("Budget Grounded Rig");
        _spawned.Add(root);

        GameObject car = new GameObject("Car");
        car.transform.SetParent(root.transform);
        Rigidbody rb = car.AddComponent<Rigidbody>();
        rb.useGravity = false;
        rb.mass = 1000f;

        VehiclePhysicsCoordinator coordinator = car.AddComponent<VehiclePhysicsCoordinator>();
        coordinator.applyProfileOnAwake = false;
        coordinator.rb = rb;
        coordinator.drivetrain = car.AddComponent<DrivetrainBrakeSystem>();
        coordinator.drivetrain.tireForceLimitMultiplier = 1.6f;

        RaycastWheel[] wheels = new RaycastWheel[4];
        for (int i = 0; i < 4; i++)
        {
            GameObject wheelObject = new GameObject($"Wheel{i}");
            wheelObject.transform.SetParent(car.transform);
            wheels[i] = wheelObject.AddComponent<RaycastWheel>();
            wheels[i].NormalForce = 2000f;
            wheels[i].IsGrounded = i < 3;
        }

        coordinator.wheels = wheels;

        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        float measured = planner.MeasureBrakeDecelMs2(coordinator, 18f);

        Assert.AreEqual(9.6f, measured, 0.01f,
            "Only three grounded wheels may contribute: 3 x 2000 N x 1.6 / 1000 kg = 9.6 m/s^2. " +
            "Got " + measured.ToString("F2") + " m/s^2, so the budget is counting a wheel that " +
            "is in the air and would plan braking the car cannot perform.");
    }

    [Test]
    public void TheWaypointCapNeverTruncatesTheHorizonOnTheRealLine()
    {
        // The cap on the per-tick walk is a performance guard, not a horizon length. If it binds,
        // the planner silently stops looking partway down the braking zone and permits a corner
        // speed it cannot brake for -- the single-waypoint defect in a smaller dose, and one that
        // only appears at speed.
        //
        // The figure that matters: at the top of this car's range the budget this physics
        // produces asks for ~22 waypoints, so a cap near 24 has almost no headroom, and a WET
        // line lowers the budget and lengthens the horizon at exactly the same time.
        AIRacingLine line = OpenRealLine();
        AILongitudinalPlanner planner = new AILongitudinalPlanner();

        // Deliberately pessimistic. A budget this low is roughly half what the physics produces on a
        // dry track, and it is the case that matters: a weaker budget means a LONGER horizon,
        // so a cap sized only for dry running would truncate the walk exactly when the car
        // most needs to see the corner. This is the budget the cap is documented against.
        const float degradedBudget = 8f;
        const int topSpeedKmh = 282;

        float horizonMetres = Mathf.Max(
            Mathf.Pow(topSpeedKmh / 3.6f, 2f) / (2f * degradedBudget) * 1.25f, 45f);

        int waypointsNeeded = Mathf.CeilToInt(horizonMetres / MeanSegmentMetres);
        int cap = AIDriverProfile.CreateRuntimeProfile().maxBrakingHorizonWaypoints;

        Assert.GreaterOrEqual(cap, waypointsNeeded,
            $"At {topSpeedKmh} km/h with a degraded {degradedBudget} m/s^2 budget the horizon " +
            $"is {horizonMetres:F0} m, which is {waypointsNeeded} waypoints at this line's " +
            $"{MeanSegmentMetres:F2} m spacing. The cap is {cap}, so the walk would stop " +
            $"{waypointsNeeded - cap} waypoints short and miss a corner it needed to brake " +
            "for. Raise maxBrakingHorizonWaypoints rather than accepting a shorter horizon.");

        // And the same must hold at the budget this physics actually produces at rest.
        float dryBudget = 1000f * 9.81f * 1.6f / 1000f;   // mg over four wheels, at the 1.6 clamp
        float dryHorizon = Mathf.Max(Mathf.Pow(topSpeedKmh / 3.6f, 2f) / (2f * dryBudget) * 1.25f, 45f);
        int dryNeeded = Mathf.CeilToInt(dryHorizon / MeanSegmentMetres);

        Assert.GreaterOrEqual(cap, dryNeeded,
            $"Even at the dry budget of {dryBudget:F1} m/s^2 the top speed needs {dryNeeded} " +
            $"waypoints and the cap is {cap}.");
    }

    // ------------------------------------------------------------------ learning

    [Test]
    public void LearningIsIgnoredUnlessTheBrakeWasActuallyWorkedHard()
    {
        // A light brake sheds speed for reasons that have nothing to do how much brake the car
        // has — rolling resistance, aero. Learning from one would drag the budget down to
        // whatever the softest useful deceleration is, and the plan would then brake far too
        // early everywhere.
        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        planner.Reset();
        planner.RecordBrakeCommand(0.3f);
        planner.LearnFromLastStep(200f, 199.8f, 35f, 0.01f);

        Assert.IsFalse(planner.HasLearnedBrakeCapability,
            "A 0.3 brake input is not evidence of what the car can do. Learning from it would " +
            "slowly ratchet the braking budget toward the speed at which a soft brake happens " +
            "to scrub off, which is not a capability.");
    }

    [Test]
    public void AHardBrakeApplicationTeachesThePlannerWhatTheCarCanDo()
    {
        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        planner.Reset();
        planner.RecordBrakeCommand(1f);

        // 200 -> 199.2 km/h over 0.01 s is (0.8/3.6)/0.01 = 22.2 m/s^2 of observed deceleration.
        planner.LearnFromLastStep(200f, 199.2f, 35f, 0.01f);

        Assert.IsTrue(planner.HasLearnedBrakeCapability,
            "A full brake application that visibly shed speed is exactly the evidence the " +
            "learning term exists to use.");

        Assert.AreEqual(22.2f, planner.LastLearnedBrakeDecelMs2, 0.1f,
            "The learned capability should be the deceleration actually observed — here " +
            "(0.8 km/h / 3.6) / 0.01 s = 22.2 m/s^2.");
    }

    [Test]
    public void AnObservationBelowTheMeasurementSpeedIsDiscarded()
    {
        // Near a standstill the reading is dominated by drivetrain spool-up and by the tyre model
        // handing over to its damping branch. It is not evidence about braking.
        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        planner.Reset();
        planner.RecordBrakeCommand(1f);
        planner.LearnFromLastStep(12f, 11.9f, 35f, 0.01f);

        Assert.IsFalse(planner.HasLearnedBrakeCapability,
            "A step that started at 0 km/h — below the 35 km/h measurement floor — must not " +
            "teach the planner anything, or every launch would poison the budget.");
    }

    [Test]
    public void TheLearnedBudgetCanOnlyReduceWhatTheCarClaimsItCanDo()
    {
        // The measured ceiling is a limit the simulation enforces. Observation may only ever
        // bring the plan DOWN from it: a wet line or a locked wheel means the car delivers less
        // than its clamp allows, and planning for the clamp would ask for braking that never
        // arrives. The opposite direction — observation raising the budget above the clamp —
        // would be planning for a capability the physics will refuse.
        GameObject root = new GameObject("Budget Ceiling Rig");
        _spawned.Add(root);

        GameObject car = new GameObject("Car");
        car.transform.SetParent(root.transform);
        Rigidbody rb = car.AddComponent<Rigidbody>();
        rb.useGravity = false;
        rb.mass = 1000f;

        VehiclePhysicsCoordinator coordinator = car.AddComponent<VehiclePhysicsCoordinator>();
        coordinator.applyProfileOnAwake = false;
        coordinator.rb = rb;
        coordinator.drivetrain = car.AddComponent<DrivetrainBrakeSystem>();
        coordinator.drivetrain.tireForceLimitMultiplier = 1.6f;

        RaycastWheel[] wheels = new RaycastWheel[4];
        for (int i = 0; i < 4; i++)
        {
            GameObject wheelObject = new GameObject($"Wheel{i}");
            wheelObject.transform.SetParent(car.transform);
            wheels[i] = wheelObject.AddComponent<RaycastWheel>();
            wheels[i].NormalForce = 2000f;
            wheels[i].IsGrounded = true;
        }

        coordinator.wheels = wheels;

        AILongitudinalPlanner planner = new AILongitudinalPlanner();
        planner.Reset();

        float measured = planner.MeasureBrakeDecelMs2(coordinator, 18f);
        Assert.AreEqual(12.8f, measured, 0.01f, "Precondition: the clamp implies 12.8 m/s^2.");

        // Driven exactly as production drives it, because the two halves are interleaved: Tick
        // measures and learns, the pedal is recorded once the whole input law has run, and the
        // NEXT Tick is what learns from it.
        planner.RecordBrakeCommand(1f);
        planner.Tick(coordinator, 200f, 18f, 35f, 0.01f);
        planner.RecordBrakeCommand(1f);
        planner.Tick(coordinator, 199f, 18f, 35f, 0.01f);

        Assert.IsTrue(planner.HasLearnedBrakeCapability,
            "Precondition: the step should have produced a usable observation. A full brake " +
            "application from 200 to 199 km/h is exactly the evidence the learning term uses.");

        Assert.Greater(planner.LastLearnedBrakeDecelMs2, measured,
            "Precondition: this fixture only makes sense if the observation claims MORE than " +
            "the clamp allows, so that the ceiling can be shown to win. The observation was " +
            planner.LastLearnedBrakeDecelMs2.ToString("F1") + " m/s^2 against a measured " +
            measured.ToString("F1") + " m/s^2.");

        float budget = planner.LastBrakeDecelMs2;

        Assert.LessOrEqual(budget, measured + 0.001f,
            $"The measured ceiling is {measured:F1} m/s^2 and the observation claimed more. " +
            $"The planner used {budget:F1} m/s^2. An observation must never raise the budget " +
            "above the force clamp the simulation enforces, or the plan will ask for braking " +
            "that is silently discarded.");
    }

    // -------------------------------------------------------------------- fixtures

    /// <summary>
    /// A straight racing line of mostly-uniform speed with one slow pair of waypoints, which is
    /// the shape that exposes a single-waypoint reader: the fast waypoint immediately ahead
    /// always permits more than the slow corner further on.
    /// </summary>
    /// <summary>
    /// Opens the real baked Track_01 line. Needed by the cap test, because a horizon sized in
    /// waypoints depends on this line's spacing and no synthetic fixture can stand in for it.
    /// </summary>
    private AIRacingLine OpenRealLine()
    {
        UnityEditor.SceneManagement.EditorSceneManager.OpenScene(
            "Assets/Scenes/Track_01.unity", UnityEditor.SceneManagement.OpenSceneMode.Single);

        AIRacingLine line = Object.FindAnyObjectByType<AIRacingLine>();
        Assert.IsNotNull(line,
            "Track_01 must carry a baked AIRacingLine, or the horizon cannot be sized against " +
            "the real track's geometry.");
        Assert.Greater(line.Count, 100,
            "The real baked line is 300 waypoints; found " + line.Count + ".");
        return line;
    }

    private AIRacingLine BuildLine(
        out GameObject root,
        float straightSpeedKmh,
        int cornerIndex,
        int cornerIndex2,
        float cornerSpeedKmh,
        float spacing,
        int waypointCount = 24)
    {
        root = new GameObject("Planner Test Line");
        _spawned.Add(root);

        AIRacingLine line = root.AddComponent<AIRacingLine>();
        line.loop = false;

        for (int i = 0; i < waypointCount; i++)
        {
            GameObject waypointObject = new GameObject($"WP_{i:00}");
            waypointObject.transform.SetParent(root.transform);
            waypointObject.transform.position = new Vector3(0f, 0f, i * spacing);

            AIRacingWaypoint waypoint = waypointObject.AddComponent<AIRacingWaypoint>();
            waypoint.targetSpeedKmh = (i == cornerIndex || i == cornerIndex2)
                ? cornerSpeedKmh
                : straightSpeedKmh;
            waypoint.laneWidth = 16f;
            waypoint.brakingCaution = 0f;
        }

        // Awake and OnValidate do not run for a plain MonoBehaviour built from a test, so the
        // waypoint list is populated explicitly rather than by the editor.
        line.RefreshWaypoints();
        Assert.AreEqual(waypointCount, line.Count, "The fixture's racing line is empty.");
        return line;
    }
}