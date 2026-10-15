using UnityEngine;

/// <summary>
/// Decides the speed an AI car may carry <i>right now</i>, and turns that decision into pedals.
///
/// <para>
/// The defect this exists to fix: the controller used to take the target speed of the single
/// waypoint immediately ahead — about 11 m on the baked Track_01 line. Braking from 240 km/h to
/// a 90 km/h corner needs roughly 120 m on this physics, so by the time the corner was visible the
/// car was already inside the braking zone and the only available response was to saturate the
/// brake and oscillate. That is the mechanism behind the "no erratic braking" acceptance
/// criterion for Phase 3.3.
/// </para>
///
/// <para>
/// The arithmetic is <see cref="AISpeedProfile"/>, kept as pure functions so it can be tested
/// without a car. This class is the part that cannot be: it walks the real racing line, measures
/// what the car can actually do, and holds the running learning state.
/// </para>
///
/// <para>
/// A note on what "measured" means here. The tyre model has no longitudinal half —
/// <c>RaycastWheel</c> hard-zeroes longitudinal slip (<c>LocalSlipVector.y</c>), so
/// <c>TractionSystem.LongGripCoeff</c> is identically zero on all four wheels and there is no
/// wheelspin signal to read. What the simulation will physically honour is the per-wheel clamp in
/// <c>DrivetrainBrakeSystem</c>: <c>force &lt;= NormalForce * tireForceLimitMultiplier</c>. The
/// budget here is therefore derived from that same clamp rather than from a tyre model that does
/// not exist, and is then trimmed by the brake efficiency the advanced brake system reports and
/// by deceleration actually observed on the track.
/// </para>
/// </summary>
public class AILongitudinalPlanner
{
    private const float KmhToMs = 1f / 3.6f;

    /// <summary>
    /// Only a brake application that is genuinely near full tells us anything about capability.
    /// A light brake sheds speed for reasons that have nothing to do with how much brake the car
    /// has, so learning from one would drive the estimate — and therefore the whole plan — down
    /// to whatever the softest useful deceleration happens to be.
    /// </summary>
    private const float LearnBrakeThreshold = 0.8f;

    /// <summary>
    /// How fast the learned capability tracks new observations. Low, because a single sample is
    /// evidence rather than proof: on a dry track this should settle at the measured ceiling
    /// rather than chasing every kerb, and on a low-grip line it should still converge in a
    /// couple of corners.
    /// </summary>
    private const float LearnRate = 0.02f;

    /// <summary>
    /// Below this much total grounded load the car is not carrying its own weight and the clamp
    /// is meaningless — airborne, or mid-spawn before the suspension has settled. Planning
    /// against a near-zero budget would produce an enormous allowed speed, so these frames fall
    /// back to the authored figure instead.
    /// </summary>
    private const float MinimumGroundedNormalForce = 1f;

    private float[] _targetSpeedsKmh = new float[16];
    private float[] _distancesMetres = new float[16];

    private bool _hasLearned;
    private float _learnedBrakeDecelMs2;
    private float _speedKmhLastTick;
    private float _brakeInputLastTick;

    /// <summary>Budget this tick used to plan against, in m/s^2.</summary>
    public float LastBrakeDecelMs2 { get; private set; }

    /// <summary>Ceiling derived from the drivetrain's own per-wheel clamp, before learning. m/s^2.</summary>
    public float LastMeasuredBrakeDecelMs2 { get; private set; }

    /// <summary>Learned deceleration, or the measured value until a valid sample arrives. m/s^2.</summary>
    public float LastLearnedBrakeDecelMs2 => _hasLearned ? _learnedBrakeDecelMs2 : LastMeasuredBrakeDecelMs2;

    /// <summary>Speed the plan permits the car to carry at its current position. km/h.</summary>
    public float LastPlannedSpeedKmh { get; private set; }

    /// <summary>How far ahead the plan looked this tick. Metres.</summary>
    public float LastHorizonMetres { get; private set; }

    /// <summary>How many waypoints the plan considered this tick.</summary>
    public int LastHorizonWaypointCount { get; private set; }

    /// <summary>True once at least one usable braking sample has been observed.</summary>
    public bool HasLearnedBrakeCapability => _hasLearned;

    /// <summary>
    /// Forgets the learned capability. The measured ceiling is unaffected — it is re-derived
    /// from the car every tick — but a fresh car should not inherit another car's history.
    /// </summary>
    public void Reset()
    {
        _hasLearned = false;
        _learnedBrakeDecelMs2 = 0f;
        _speedKmhLastTick = 0f;
        _brakeInputLastTick = 0f;
        LastBrakeDecelMs2 = 0f;
        LastMeasuredBrakeDecelMs2 = 0f;
        LastPlannedSpeedKmh = 0f;
        LastHorizonMetres = 0f;
        LastHorizonWaypointCount = 0;
    }

    /// <summary>
    /// Measures what the car can do this tick and folds in the previous step's observation.
    ///
    /// <para>
    /// The measurement mirrors the clamp in <c>DrivetrainBrakeSystem.ApplyForceAtWheel</c>:
    /// the sum, over grounded wheels, of <c>NormalForce * tireForceLimitMultiplier</c>, divided
    /// by mass. That is the deceleration the simulation will actually permit, so it is the right
    /// ceiling to plan against. It is scaled by the brake efficiency the advanced brake system
    /// reports, because lockup reduces delivered pressure and a locked wheel does not stop the
    /// car any faster.
    /// </para>
    ///
    /// <para>
    /// Downforce deliberately does not appear here. It raises the ceiling on how much suspension
    /// force a wheel may produce, so a fast car does measure a higher budget — but only through
    /// the compression it causes, not as a term added to the load.
    /// </para>
    /// </summary>
    /// <param name="coordinator">The car. May be null, in which case the fallback is returned.</param>
    /// <param name="speedKmh">Speed read this tick, km/h.</param>
    /// <param name="previousBrakeInput">Brake commanded last tick, 0..1. The force it commanded
    /// was applied during the step that produced <paramref name="speedKmh"/>.</param>
    /// <param name="fallbackMs2">Budget to use when the car cannot be measured — a car with no
    /// grounded wheels, or no drivetrain at all.</param>
    /// <param name="minimumSpeedKmhForMeasurement">Observations below this are dominated by
    /// drivetrain spool and are not evidence of braking capability.</param>
    /// <param name="stepSeconds">Physics step length. The project runs at 100 Hz, not Unity's
    /// default 50 Hz, so this must not be assumed.</param>
    public float Tick(
        VehiclePhysicsCoordinator coordinator,
        float speedKmh,
        float fallbackMs2,
        float minimumSpeedKmhForMeasurement,
        float stepSeconds)
    {
        LastMeasuredBrakeDecelMs2 = MeasureBrakeDecelMs2(coordinator, fallbackMs2);

        // Last tick's brake force was applied during the step that produced this tick's speed,
        // so the pair of them is exactly one step of braking.
        LearnFromLastStep(_speedKmhLastTick, speedKmh, minimumSpeedKmhForMeasurement, stepSeconds);

        // The learned term is a floor on capability, never a ceiling. A car that has shown it
        // cannot achieve more than this — a wet line, a locked wheel — must not be planned for
        // at the dry-track figure, or the plan will ask for braking the car cannot deliver.
        LastBrakeDecelMs2 = _hasLearned
            ? Mathf.Min(LastMeasuredBrakeDecelMs2, _learnedBrakeDecelMs2)
            : LastMeasuredBrakeDecelMs2;

        _speedKmhLastTick = speedKmh;
        _brakeInputLastTick = 0f;
        return LastBrakeDecelMs2;
    }

    /// <summary>
    /// Records what the brake commanded on the previous step actually achieved.
    ///
    /// <para>
    /// The two speeds are passed in rather than held privately, because "the speed before the
    /// step" is exactly the quantity the measurement is defined against: by the end of a long
    /// braking run the car may already be too slow to measure anything from. A caller driving
    /// the planner a step at a time — a test, or anything else not going through
    /// <see cref="Tick"/> — can only supply it if it is asked for it.
    /// </para>
    /// </summary>
    /// <param name="speedKmhBeforeStep">Speed at the start of the measured step, km/h.</param>
    /// <param name="speedKmhAfterStep">Speed at the end of it, km/h.</param>
    /// <param name="minimumSpeedKmhForMeasurement">Below this, the reading is not evidence.</param>
    /// <param name="stepSeconds">Length of the measured step.</param>
    public void LearnFromLastStep(
        float speedKmhBeforeStep,
        float speedKmhAfterStep,
        float minimumSpeedKmhForMeasurement,
        float stepSeconds)
    {
        // MeasureBrakeDecel is handed 0 so its answer is unambiguous: it rejects by returning its
        // input, and a sample it accepts is clamped to at least 0.5. Passing the running estimate
        // instead would make "rejected" and "accepted, and the sample happened to equal the
        // estimate" indistinguishable.
        float observed = AISpeedProfile.MeasureBrakeDecel(
            0f,
            speedKmhBeforeStep,
            speedKmhAfterStep,
            stepSeconds,
            minimumSpeedKmhForMeasurement);

        if (_brakeInputLastTick >= LearnBrakeThreshold && observed > 0f)
        {
            _learnedBrakeDecelMs2 = _hasLearned
                ? Mathf.Lerp(_learnedBrakeDecelMs2, observed, LearnRate)
                : observed;
            _hasLearned = true;
        }

        _brakeInputLastTick = 0f;
    }

    /// <summary>
    /// Records the brake this tick is commanding, so the next tick can learn from what it
    /// achieved. Kept separate from <see cref="Tick"/> because the pedal is only known once the
    /// whole input law has run.
    /// </summary>
    public void RecordBrakeCommand(float brakeInput)
    {
        _brakeInputLastTick = Mathf.Clamp01(brakeInput);
    }

    /// <summary>
    /// Deceleration ceiling implied by the car's own drivetrain clamp, in m/s^2.
    /// </summary>
    public float MeasureBrakeDecelMs2(VehiclePhysicsCoordinator coordinator, float fallbackMs2)
    {
        float floor = Mathf.Max(0.5f, fallbackMs2);
        if (coordinator == null || coordinator.rb == null || coordinator.wheels == null)
            return floor;

        float mass = coordinator.rb.mass;
        if (mass <= 0.01f)
            return floor;

        float multiplier = 1f;
        if (coordinator.drivetrain != null)
            multiplier = coordinator.drivetrain.tireForceLimitMultiplier;

        float totalNormal = 0f;
        for (int i = 0; i < coordinator.wheels.Length; i++)
        {
            RaycastWheel wheel = coordinator.wheels[i];
            if (wheel == null || !wheel.IsGrounded)
                continue;

            totalNormal += wheel.NormalForce;
        }

        // Airborne, or a spawn frame before the suspension has taken up its own weight. Planning
        // against a near-zero budget would hand back an enormous allowed speed, so these frames
        // keep the authored figure rather than believing the measurement.
        if (totalNormal < MinimumGroundedNormalForce)
            return floor;

        float decel = totalNormal * multiplier / mass;

        if (coordinator.advancedBraking != null)
            decel *= Mathf.Clamp01(coordinator.advancedBraking.BrakeEfficiency);

        return Mathf.Max(0.5f, decel);
    }

    /// <summary>
    /// The speed the car may carry at its current position given every corner within its braking
    /// horizon, and the minimum over that horizon rather than the next waypoint alone.
    /// </summary>
    /// <param name="line">The racing line to follow.</param>
    /// <param name="segmentIndex">Segment the car currently occupies.</param>
    /// <param name="position">Car position, world space.</param>
    /// <param name="speedKmh">Current speed, km/h — sets the horizon length.</param>
    /// <param name="brakeDecelMs2">Budget from <see cref="Tick"/>, m/s^2.</param>
    /// <param name="speedMultiplier">Difficulty pace multiplier.</param>
    /// <param name="cornerConfidence">Difficulty corner confidence.</param>
    /// <param name="cautionStrength">Interpolated corner caution for this driver's competitiveness.</param>
    /// <param name="minimumHorizonMetres">Floor on the horizon, so a slow car still sees a corner.</param>
    /// <param name="maximumWaypoints">Cap on horizon entries, bounding the per-tick walk.</param>
    /// <param name="fallbackKmh">Value to return when the line yields no usable constraint.</param>
    public float SolvePlannedSpeedKmh(
        AIRacingLine line,
        int segmentIndex,
        Vector3 position,
        float speedKmh,
        float brakeDecelMs2,
        float speedMultiplier,
        float cornerConfidence,
        float cautionStrength,
        float minimumHorizonMetres,
        int maximumWaypoints,
        float fallbackKmh)
    {
        LastPlannedSpeedKmh = fallbackKmh;
        LastHorizonWaypointCount = 0;

        if (line == null || line.Count < 2 || maximumWaypoints < 1)
            return fallbackKmh;

        float budget = Mathf.Max(0.5f, brakeDecelMs2);
        float speedMs = Mathf.Max(0f, speedKmh * KmhToMs);

        // The horizon is a braking distance, not a waypoint count. At 250 km/h with the budget this
        // physics actually produces — about 15.7 m/s^2 at rest, from 1000 kg over four wheels at
        // the 1.6 per-wheel clamp — that is ~190 m, or seventeen waypoints. A car in a hairpin
        // needs barely one. Fixing the count instead would make one of those two badly wrong, and
        // it is why the single waypoint the controller used to read was so far short: 11 m is
        // less than a seventh of the 122 m that 240-to-90 actually takes at this budget.
        //
        // The safety factor covers the two things the pure distance ignores: the drivetrain needs
        // roughly 130-170 ms to reach full brake, and the sensor that would warn about a car
        // ahead is sampled on its own cadence rather than every tick.
        float horizon = speedMs * speedMs / (2f * budget) * 1.25f;
        horizon = Mathf.Max(horizon, minimumHorizonMetres);
        LastHorizonMetres = horizon;

        EnsureCapacity(maximumWaypoints);

        int index = line.WrapIndex(segmentIndex);
        Vector3 onLine = line.GetClosestPointOnSegment(index, position);

        // Distance still to run along the current segment before the first waypoint ahead. The
        // car's projection can sit anywhere on it, including past the far end when it is between
        // segments, so this is clamped rather than assumed.
        float remainingOnSegment = SegmentLength(line, index)
            - Mathf.Clamp(Vector3.Distance(position, onLine), 0f, SegmentLength(line, index));

        int count = 0;
        int maximum = line.loop
            ? maximumWaypoints
            : Mathf.Min(maximumWaypoints, Mathf.Max(0, line.Count - segmentIndex));

        // Distance from the car, not segment length. These are different numbers and confusing
        // them is the whole original defect in a different costume: a corner 90 m ahead must be
        // recorded as being 90 m away, or the solver grants it the speed a car would have after
        // braking over one segment and never slows for it.
        float travelled = 0f;

        for (int step = 0; step < maximum; step++)
        {
            travelled += Mathf.Max(0f, remainingOnSegment);

            int nextIndex = line.WrapIndex(index + 1);
            AIRacingWaypoint waypoint = line.GetWaypoint(nextIndex);
            if (waypoint != null)
            {
                float curvature = line.CalculateCurvature01(nextIndex, 1);
                float cornerScale = Mathf.Lerp(1f, cornerConfidence, curvature);
                float caution = Mathf.Lerp(1f, 1f - waypoint.brakingCaution * cautionStrength, curvature);

                _targetSpeedsKmh[count] = waypoint.targetSpeedKmh * speedMultiplier * cornerScale * caution;
                _distancesMetres[count] = travelled;
                count++;
            }

            LastHorizonWaypointCount = count;

            if (travelled >= horizon)
                break;

            index = nextIndex;
            remainingOnSegment = SegmentLength(line, index);

            // A line that does not loop runs out. Anything past the last waypoint is not a
            // constraint, and wrapping would invent one.
            if (!line.loop && index >= line.Count - 1)
                break;
        }

        if (count == 0)
            return fallbackKmh;

        float allowed = AISpeedProfile.SolveAllowedSpeedKmh(_targetSpeedsKmh, _distancesMetres, count, budget);
        if (float.IsNaN(allowed) || float.IsInfinity(allowed))
            return fallbackKmh;

        LastPlannedSpeedKmh = Mathf.Max(0f, allowed);
        return LastPlannedSpeedKmh;
    }

    private static float SegmentLength(AIRacingLine line, int index)
    {
        if (!line.loop && index >= line.Count - 1)
            return 0f;

        return Vector3.Distance(line.GetPosition(index), line.GetPosition(index + 1));
    }

    private void EnsureCapacity(int required)
    {
        if (_targetSpeedsKmh.Length >= required)
            return;

        int size = Mathf.NextPowerOfTwo(Mathf.Max(required, _targetSpeedsKmh.Length * 2));
        _targetSpeedsKmh = new float[size];
        _distancesMetres = new float[size];
    }
}