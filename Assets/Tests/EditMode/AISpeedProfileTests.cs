using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Tests for the braking planner's arithmetic.
///
/// <para>
/// These are the tests the project did not have. Every existing AI behaviour test builds a
/// synthetic racing line and drives the controller; none of them checks that the car can
/// actually see a corner in time to slow for it. That gap is what let the AI target the speed
/// of the single waypoint in front of it — about 11 m — when braking from 240 km/h to 90 km/h
/// needs roughly 42 m. The car could not see the corner until it was already in the braking
/// zone, so the only response left was to saturate the brake and oscillate.
/// </para>
///
/// <para>
/// This solver is pure arithmetic over plain arrays, so it is tested directly here rather than
/// inferred from a race.
/// </para>
/// </summary>
public class AISpeedProfileTests
{
    private const float KmhToMs = 1f / 3.6f;
    private const float BrakingDecel = 25f;   // m/s^2, a realistic F1 figure for this physics

    private static float Ms(float kmh) => kmh * KmhToMs;

    // ---------------------------------------------------------------- point solver

    [Test]
    public void NoConstraintsImposeNoLimit()
    {
        Assert.AreEqual(float.PositiveInfinity,
            AISpeedProfile.SolveAllowedSpeedKmh(new float[0], new float[0], BrakingDecel));

        Assert.AreEqual(float.PositiveInfinity,
            AISpeedProfile.SolveAllowedSpeedKmh(null, null, BrakingDecel));
    }

    [Test]
    public void WithNoBrakingCapabilityTheCarCanOnlyCarryTheCornerSpeed()
    {
        // A car that cannot slow has exactly one legal choice: be travelling no faster than the
        // slowest thing it is about to reach.
        float allowed = AISpeedProfile.SolveAllowedSpeedKmh(
            new[] { 240f, 90f }, new[] { 5f, 500f }, 0f);

        Assert.AreEqual(90f, allowed, 0.01f);
    }

    [Test]
    public void ASingleCornerIsReachedAtTheRightSpeed()
    {
        // 90 km/h = 25 m/s, 100 m out, 25 m/s^2 of braking:
        //   v^2 <= 25^2 + 2*25*100 = 5625  ->  v = 75 m/s = 270 km/h
        float allowed = AISpeedProfile.SolveAllowedSpeedKmh(
            new[] { 90f }, new[] { 100f }, BrakingDecel);

        Assert.AreEqual(270f, allowed, 0.5f);
    }

    [Test]
    public void TheNearestConstraintWinsOnlyWhenItIsTheTightest()
    {
        // A distant tight corner beats a near loose one: 300 km/h ten metres away permits
        // 310.6 km/h, while a 90 km/h corner sixty metres away permits only 216.8. The distant
        // corner is the binding constraint, so the pair must return the distant one's answer
        // and not the near one's.
        //
        // The near limit has to be high and the far distance short for this to mean anything.
        // Braking at 25 m/s^2 barely changes speed over a few metres, so a near corner is
        // almost never the tight one: 240 km/h at 10 m still permits 253 km/h, which is well
        // above what a 90 km/h corner 300 m away allows, and a fixture like that would be
        // testing the near constraint while claiming to test the far one.
        float allowed = AISpeedProfile.SolveAllowedSpeedKmh(
            new[] { 300f, 90f }, new[] { 10f, 60f }, BrakingDecel);

        Assert.AreEqual(
            AISpeedProfile.SolveAllowedSpeedKmh(new[] { 90f }, new[] { 60f }, BrakingDecel),
            allowed, 0.01f,
            "The tight distant corner should be the binding constraint.");
    }

    [Test]
    public void AddingAConstraintCanNeverRaiseTheAllowedSpeed()
    {
        // The property that makes the planner safe: more information never lets the car go faster.
        //
        // The constraints have to ACCUMULATE for that to be a statement about this solver. An
        // earlier version of this test solved a fresh two-constraint problem each pass and
        // asserted the sequence of answers was non-increasing, which is a claim about the
        // generated fixture rather than about the solver — the constraint from the previous
        // pass was thrown away, so a later pass was free to return a higher number and the
        // assertion was measuring nothing. Here each pass appends to the same horizon, so the
        // answer is genuinely a minimum over a growing set and can only fall.
        var corners = new System.Collections.Generic.List<float>();
        var distances = new System.Collections.Generic.List<float>();
        float previous = float.PositiveInfinity;

        for (int i = 0; i < 40; i++)
        {
            float corner = 60f + (i * 37f % 180f);
            float distance = 5f + (i * 23f % 400f);
            corners.Add(corner);
            distances.Add(distance);

            float allowed = AISpeedProfile.SolveAllowedSpeedKmh(
                corners.ToArray(), distances.ToArray(), BrakingDecel);

            Assert.LessOrEqual(allowed, previous + 0.001f,
                $"Adding the constraint {corner} km/h at {distance} m raised the allowed speed " +
                $"to {allowed} from {previous}. More constraints must only ever tighten the plan.");
            previous = allowed;
        }
    }

    [Test]
    public void ANegativeBrakingCapabilityIsTreatedAsNoneRatherThanAsThrust()
    {
        // A signed error in the estimate must not make the solver think the car accelerates.
        float allowed = AISpeedProfile.SolveAllowedSpeedKmh(
            new[] { 90f }, new[] { 100f }, -30f);

        Assert.AreEqual(90f, allowed, 0.01f);
    }

    // ------------------------------------------------- the defect this exists to fix

    [Test]
    public void ACarAt240MustAlreadyBeSlowingFortyMetresFromTheCorner()
    {
        // The measured situation on Track_01: a car carrying 240 km/h with a 90 km/h corner
        // about 42 m ahead. The AI used to target the speed of the waypoint 11 m in front of
        // it, so it saw 90 km/h too late and braked at full force for the last third of a
        // corner.
        float allowed = AISpeedProfile.SolveAllowedSpeedKmh(
            new[] { 90f }, new[] { 42f }, BrakingDecel);

        Assert.Less(allowed, 240f,
            "At 42 m from a 90 km/h corner the car cannot still be carrying 240 km/h. " +
            "If this assertion fails the planner cannot brake in time and the AI will " +
            "bang-bang the brake into every corner.");
    }

    [Test]
    public void ThePlanLowersTheTargetSmoothlyOnApproachRatherThanInOneStep()
    {
        // This is what removes the chatter: the commanded speed falls continuously as the corner
        // gets closer, so the brake has somewhere to ramp instead of snapping between extremes.
        float[] distances = { 60f, 50f, 42f, 35f, 30f, 20f, 11.2f, 5f, 0f };

        float previous = float.PositiveInfinity;
        foreach (float d in distances)
        {
            float allowed = AISpeedProfile.SolveAllowedSpeedKmh(
                new[] { 90f }, new[] { d }, BrakingDecel);

            Assert.LessOrEqual(allowed, previous + 0.001f,
                $"The plan rose as the corner got closer ({d} m -> {allowed} km/h).");

            // Never above what physics allows, and never below the corner it is heading for.
            float ceiling = Mathf.Sqrt(Ms(90f) * Ms(90f) + 2f * BrakingDecel * d) / KmhToMs;
            Assert.LessOrEqual(allowed, ceiling + 0.01f,
                $"At {d} m the plan allowed {allowed} km/h, above the physical limit {ceiling}.");
            Assert.GreaterOrEqual(allowed, 90f - 0.01f,
                $"At {d} m the plan allowed {allowed} km/h, below the corner it is heading for.");

            previous = allowed;
        }

        Assert.AreEqual(90f, previous, 0.01f,
            "At the corner itself the plan must equal the corner speed exactly.");
    }

    [Test]
    public void ATightCornerIsVisibleMuchFurtherAwayThanALooseOne()
    {
        float tight = AISpeedProfile.SolveAllowedSpeedKmh(new[] { 90f }, new[] { 300f }, BrakingDecel);
        float loose = AISpeedProfile.SolveAllowedSpeedKmh(new[] { 200f }, new[] { 300f }, BrakingDecel);

        Assert.Less(tight, loose,
            "A slower corner must impose a slower plan at the same distance — that difference " +
            "is the entire reason the AI needs to look along the line rather than at one waypoint.");
    }

    // ------------------------------------------------------------ whole-line profile

    [Test]
    public void TheForwardProfileNeverExceedsAnyCornerSpeed()
    {
        int count = 12;
        var corners = new float[count];
        var segments = new float[count];
        for (int i = 0; i < count; i++)
        {
            corners[i] = 90f + (i % 4) * 50f;      // a repeating slow/fast pattern
            segments[i] = 11.3f;
        }

        var solved = new float[count];
        AISpeedProfile.SolveForwardProfileKmh(corners, segments, BrakingDecel, 0, solved);

        for (int i = 0; i < count; i++)
        {
            Assert.LessOrEqual(solved[i], corners[i] + 0.01f,
                $"Waypoint {i} was planned at {solved[i]} km/h, above its own corner speed " +
                $"{corners[i]}.");
        }
    }

    [Test]
    public void TheForwardProfileAgreesWithThePointSolver()
    {
        // Two implementations of the same idea must not drift apart. Sample the whole-line pass
        // and compare it against the from-here solver at the same point.
        int count = 8;
        var corners = new float[count];
        var segments = new float[count];
        for (int i = 0; i < count; i++)
        {
            corners[i] = 100f + i * 20f;
            segments[i] = 15f;
        }

        var solved = new float[count];
        AISpeedProfile.SolveForwardProfileKmh(corners, segments, BrakingDecel, 0, solved);

        for (int i = 0; i < count; i++)
        {
            var targets = new float[count];
            var distances = new float[count];
            float travelled = 0f;
            for (int j = 0; j < count; j++)
            {
                int stepIndex = (i + j) % count;
                targets[j] = corners[stepIndex];
                // The solver wants distance FROM THE CAR, so it must accumulate.
                distances[j] = travelled;
                travelled += segments[stepIndex];
            }

            float fromHere = AISpeedProfile.SolveAllowedSpeedKmh(targets, distances, BrakingDecel);
            Assert.LessOrEqual(solved[i], fromHere + 0.5f,
                $"Waypoint {i}: the profile pass allowed {solved[i]} km/h but the point solver " +
                $"only {fromHere} km/h. One of them is letting the car brake too late.");
        }
    }

    [Test]
    public void EachWaypointIsConstrainedByTheGapAheadOfItNotTheGapBehindIt()
    {
        // Four waypoints, one slow corner at index 3. segments[i] is the distance from i to i+1,
        // so segments[2] = 5 means waypoint 2 is only 5 m from that corner, while segments[1]
        // = 100 means waypoint 1 has 100 m of warning.
        var corners = new[] { 300f, 300f, 300f, 60f };
        var segments = new[] { 5f, 100f, 5f, 5f };

        var solved = new float[4];
        AISpeedProfile.SolveForwardProfileKmh(corners, segments, BrakingDecel, 0, solved);

        Assert.Less(solved[2], 100f,
            "Waypoint 2 is 5 m from a 60 km/h corner, so it must be planning to slow hard, not " +
            $"running at the line speed. Got {solved[2]} km/h. If this fails the pass is reading " +
            "the segment BEHIND each point instead of the one ahead, which happens to look " +
            "correct on a track whose segments are all the same length.");

        Assert.Greater(solved[1], 200f,
            "Waypoint 1 has 100 m of warning and must be nearly free. Got " +
            $"{solved[1]} km/h.");
    }

    [Test]
    public void ASteadyLineNeedsNoSlowing()
    {
        int count = 10;
        var corners = new float[count];
        var segments = new float[count];
        for (int i = 0; i < count; i++) { corners[i] = 200f; segments[i] = 20f; }

        var solved = new float[count];
        AISpeedProfile.SolveForwardProfileKmh(corners, segments, BrakingDecel, 0, solved);

        for (int i = 0; i < count; i++)
            Assert.AreEqual(200f, solved[i], 0.01f,
                "A constant-speed line must plan as a constant speed; anything else means the " +
                "pass is inventing braking that the corner does not ask for.");
    }

    // -------------------------------------------------------- braking capability

    [Test]
    public void ARealBrakingStepIsMeasured()
    {
        // 240 km/h to 200 km/h in one 0.01 s step at 100 Hz:
        //   (66.67 - 55.56) / 0.01 = 1111 m/s^2 -- implausible, so it should clamp.
        float measured = AISpeedProfile.MeasureBrakeDecel(30f, 240f, 200f, 0.01f, 100f);
        Assert.LessOrEqual(measured, 80f,
            "An implausible deceleration must be clamped, not believed.");

        // A realistic step: 240 to 236 km/h over 0.01 s is still absurd, so use a real interval.
        // 240 -> 236 km/h in 0.04 s  ->  (66.67 - 65.56) / 0.04 = 27.8 m/s^2
        float sane = AISpeedProfile.MeasureBrakeDecel(30f, 240f, 236f, 0.04f, 100f);
        Assert.AreEqual(27.8f, sane, 0.5f);
    }

    [Test]
    public void AStepThatGainedSpeedTeachesNothing()
    {
        float estimate = 30f;
        Assert.AreEqual(estimate,
            AISpeedProfile.MeasureBrakeDecel(estimate, 200f, 220f, 0.05f, 100f), 0.001f,
            "A car that accelerated tells us nothing about braking.");
    }

    [Test]
    public void ATooSlowStepIsNotMeasured()
    {
        float estimate = 30f;
        Assert.AreEqual(estimate,
            AISpeedProfile.MeasureBrakeDecel(estimate, 40f, 20f, 0.5f, 100f), 0.001f,
            "Near standstill the reading is drivetrain spool-up, not braking capability.");
    }

    [Test]
    public void AZeroLengthStepIsIgnored()
    {
        Assert.AreEqual(30f, AISpeedProfile.MeasureBrakeDecel(30f, 240f, 200f, 0f, 100f), 0.001f);
    }

    [Test]
    public void TheMeasuredEstimateFallsInsideTheSpeedProfileRange()
    {
        // A measured value has to be usable by the solver: 0.5 to 80 m/s^2.
        float low = AISpeedProfile.MeasureBrakeDecel(30f, 200f, 199.9f, 0.01f, 10f);
        Assert.GreaterOrEqual(low, 0.5f);

        float high = AISpeedProfile.MeasureBrakeDecel(30f, 300f, 100f, 0.01f, 10f);
        Assert.LessOrEqual(high, 80f);
    }
}