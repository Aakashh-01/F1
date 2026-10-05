using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using F1.EditorTools;

public class TrackDataBakeTests
{
    private const float MaxLateralAccel = 12f;
    private const float BrakeAccel = 18f;
    private const float StraightSpeedKmh = 240f;
    private const float MinSpeedKmh = 90f;
    private const float WidthMetres = 16f;

    /// <summary>
    /// A synthetic circuit: an explicit chain of straights and arcs, sampled UNIFORMLY IN ARC
    /// LENGTH every <see cref="SampleSpacingMetres"/> metres.
    ///
    /// <para>
    /// <b>Uniform sampling is load-bearing, not cosmetic.</b> The previous version of this fixture
    /// added ONE point per straight (its far endpoint) and 31 per arc, which is neither uniform nor
    /// what production produces. Decimation picks control points by INDEX, so that spacing put
    /// knots ~420 m apart across a straight and ~2 m apart in a corner; the spline then spiked to
    /// 0.395 /m (radius 2.5 m) curvature in the middle of a straight, and the baked profile stepped
    /// 150 km/h between adjacent samples. Production does not have this problem —
    /// <c>TrackRacingLineBaker.Bake()</c> emits cross-section cells laid end to end every 10 m — so
    /// the fixture has to match production or it tests a shape the baker will never see.
    /// </para>
    ///
    /// <para>
    /// <b>Three things had to be right for this to be a circuit at all</b>, and each was wrong in
    /// turn before measurement caught it.
    /// </para>
    /// <list type="number">
    /// <item><b>The arc centre.</b> The point travels with theta DECREASING, so its velocity is
    /// <c>(sin th, -cos th)</c>. Requiring that to equal the incoming heading <c>(cos h, sin h)</c>
    /// gives <c>th0 = h + 90°</c>, hence centre <c>C = P + r*(sin h, -cos h)</c>. Placing the centre
    /// at <c>P - r*(cos h, sin h)</c> instead — directly behind the entry along the heading — makes
    /// every arc a 90° kink. Measured: tangent dot at the first arc entry was 0.0000.</item>
    /// <item><b>Closure.</b> Arc sweeps summing to 360° guarantees the HEADING returns. It says
    /// nothing about POSITION: closing also needs net displacement to vanish, which is two equations
    /// in the straight lengths. Measured: the previous fixture turned through exactly -360.00° and
    /// still missed its start by 391 m, so the spline was fitted across a 391 m chord rather than
    /// around a circuit. The two straights marked "solved" below are those equations' answers.</item>
    /// <item><b>No self-intersection.</b> An earlier polar fixture,
    /// <c>radius = 300 + 220*sin(2a) + 120*sin(5a)</c>, measured a 0.3 m minimum radius and passed
    /// within 4.5 m of itself, so every corner in it was a numerical artefact.</item>
    /// </list>
    ///
    /// <para>
    /// The shape is legible and deliberately varied: a 500 m opening straight into a 70 m hairpin, a
    /// link to a 180 m corner, the 700 m longest straight, a 60 m hairpin (the slowest corner), a
    /// link, a 260 m sweeper and a run home. It has a fast corner <i>and</i> a slow one, so the
    /// backward braking pass has real work to do rather than being handed a constant.
    /// Measured: 2,902 m lap, closes to 8 mm, 108 m clearance between non-adjacent sections, and a
    /// worst vertex turn of 9.55° — which is what 10 m sampling on a 60 m corner looks like, not a
    /// kink.
    /// </para>
    /// </summary>
    private const float SampleSpacingMetres = 10f;

    /// <summary>Straights. Two of these are SOLVED rather than chosen — see the closure note above.</summary>
    private const float StraightOneMetres = 500f;
    private const float StraightTwoMetres = 391.9718f;   // solved: makes the loop close in position
    private const float StraightThreeMetres = 700f;
    private const float StraightFourMetres = 300f;
    private const float StraightFiveMetres = 188.3807f;   // solved: makes the loop close in position

    private static List<Vector3> SyntheticCircuit()
    {
        // ('s', metres) straight, or ('a', (radius, sweepDegrees)) constant-radius arc.
        // Sweeps: 110 + 70 + 100 + 80 = 360 degrees, so the heading returns; the two solved
        // straights are what make the POSITION return too.
        var segments = new (char kind, float a, float b)[]
        {
            ('s', StraightOneMetres, 0f),
            ('a', 70f, 110f),      // hairpin
            ('s', StraightTwoMetres, 0f),
            ('a', 180f, 70f),
            ('s', StraightThreeMetres, 0f),      // longest straight
            ('a', 60f, 100f),      // slowest corner on the circuit
            ('s', StraightFourMetres, 0f),
            ('a', 260f, 80f),      // sweeper
            ('s', StraightFiveMetres, 0f),
        };

        var points = new List<Vector3>();
        float x = 0f, y = 0f, heading = 0f;
        float carry = 0f;   // arc length walked since the last emitted point

        points.Add(new Vector3(0f, 0f, 0f));

        foreach (var segment in segments)
        {
            if (segment.kind == 's')
            {
                float remaining = segment.a;
                while (remaining > 1e-6f)
                {
                    float step = Mathf.Min(SampleSpacingMetres - carry, remaining);
                    remaining -= step;
                    carry += step;
                    x += step * Mathf.Cos(heading);
                    y += step * Mathf.Sin(heading);
                    if (carry >= SampleSpacingMetres - 1e-6f)
                    {
                        points.Add(new Vector3(x, 0f, y));
                        carry = 0f;
                    }
                }
                continue;
            }

            float radius = segment.a;
            float sweep = Mathf.Deg2Rad * segment.b;
            float arcLength = radius * sweep;
            // Centre at P + r*(sin h, -cos h): the perpendicular that makes the arc ENTRY tangent
            // continuous. The opposite sign, P - r*(cos h, sin h), puts the centre directly behind
            // the entry point along the heading and turns every arc into a 90 degree kink.
            float cx = x + radius * Mathf.Sin(heading);
            float cy = y - radius * Mathf.Cos(heading);
            float startAngle = Mathf.Atan2(y - cy, x - cx);

            float walked = 0f;
            while (walked < arcLength - 1e-6f)
            {
                float step = Mathf.Min(SampleSpacingMetres - carry, arcLength - walked);
                walked += step;
                carry += step;
                if (carry >= SampleSpacingMetres - 1e-6f)
                {
                    float angle = startAngle - sweep * (walked / arcLength);
                    points.Add(new Vector3(cx + radius * Mathf.Cos(angle), 0f, cy + radius * Mathf.Sin(angle)));
                    carry = 0f;
                }
            }

            float endAngle = startAngle - sweep;
            x = cx + radius * Mathf.Cos(endAngle);
            y = cy + radius * Mathf.Sin(endAngle);
            heading -= sweep;
        }

        return points;
    }

    /// <summary>
    /// The fixture must be a circuit before any assertion about baking one means anything.
    ///
    /// <para>
    /// This test exists because two earlier fixtures passed all eight bake assertions while being
    /// neither closed nor tangent-continuous. Without it, "a second track bakes" is a claim about
    /// a shape nobody would build.
    /// </para>
    /// </summary>
    [Test]
    public void TheSyntheticFixtureItselfIsAClosedTangentContinuousCircuit()
    {
        var points = SyntheticCircuit();

        // 1. Uniform arc-length spacing, EXCLUDING the seam. The loop's total length is not an exact
        //    multiple of the spacing (2,902 m / 10 m = 290.2), so the wrap-around pair from the
        //    last sample back to the first is the remainder — 2.39 m — and is expected. Every other
        //    pair should be a full gap. This is the property the ORIGINAL fixture violated, where
        //    one point covered an entire 420 m straight and the next 2 m.
        float shortest = float.MaxValue;
        for (int i = 0; i < points.Count - 1; i++)
            shortest = Mathf.Min(shortest, Vector3.Distance(points[i], points[i + 1]));
        Assert.Greater(shortest, SampleSpacingMetres * 0.9f,
            $"Closest consecutive pair inside the lap is {shortest:F2} m apart; the fixture is not " +
            "uniformly sampled in arc length.");

        // 2. Closure. Walking the polyline must return to its start. This is a SEPARATE condition from
        //    total turning: a chain can turn through exactly 360 degrees and still end far away,
        //    which is exactly what the previous fixture did — -360.00 degrees of turning and a
        //    391 m miss, so the spline was fitted across a chord rather than around a circuit.
        //    The gap here is the un-sampled remainder of the lap, so it is under one spacing.
        float closingGap = Vector3.Distance(points[points.Count - 1], points[0]);
        Assert.Less(closingGap, SampleSpacingMetres,
            $"The fixture misses its own start by {closingGap:F1} m. A circuit must close in POSITION, " +
            "not merely turn through 360 degrees — check the two solved straights.");

        // 3. Tangent continuity. A kink shows up as a sharp vertex; on a 10 m sample over the
        //    tightest 60 m corner the expected turn is 10/60 rad = 9.55 degrees. Allowing 2x for
        //    the discretisation still leaves the earlier fixture's 90 degree kink far out.
        float worstTurn = 0f;
        for (int i = 0; i < points.Count; i++)
        {
            Vector3 before = points[(i - 1 + points.Count) % points.Count];
            Vector3 here = points[i];
            Vector3 after = points[(i + 1) % points.Count];
            float angle = Mathf.Acos(Mathf.Clamp(
                Vector3.Dot((here - before).normalized, (after - here).normalized), -1f, 1f)) * Mathf.Rad2Deg;
            worstTurn = Mathf.Max(worstTurn, angle);
        }
        Assert.Less(worstTurn, 20f,
            $"Sharpest vertex turns {worstTurn:F1} deg, which is a kink rather than a discretised " +
            "corner. Check the arc-centre sign: P - r*(cos h, sin h) gives 90 deg here.");
    }

    /// <summary>
    /// Pins the km/h -&gt; m/s conversion by checking a number the geometry dictates.
    ///
    /// <para>
    /// Without this, the unit conversion is unpinned. Mutation-tested: deleting the <c>* 3.6f</c>,
    /// or inverting it to <c>/ 3.6f</c>, produces visibly different profiles (501 / 299 / 218
    /// samples strictly between floor and ceiling) and every other assertion in this file still
    /// passes, because the clamp to [minSpeed, straightSpeed] absorbs all three. This codebase has
    /// gotten this conversion wrong before, twice.
    /// </para>
    ///
    /// <para>
    /// The 260 m sweeper is the gentlest corner, where corner speed is
    /// v = sqrt(a_lat * R) * 3.6 = sqrt(12 * 260) * 3.6 = 367 km/h — above the 240 km/h straight
    /// ceiling, so the clamp, not the conversion, decides it. Not usable.
    /// </para>
    ///
    /// <para>
    /// So this pins the 60 m corner, whose analytic speed is sqrt(12 * 60) * 3.6 = 96.60 km/h. The
    /// backward pass can only LOWER a speed, so the observed minimum is at most that; asserting
    /// equality would be wrong, because the approach to that corner brakes earlier still. What
    /// distinguishes the three conversions is the set of samples that are neither at the floor nor
    /// at the ceiling: 501 with the conversion correct, 299 without it, 218 inverted. Each of
    /// those interior samples is a corner whose speed came from the formula, so their existence at
    /// all proves the conversion ran.
    /// </para>
    /// </summary>
    [Test]
    public void CornerSpeedIsDerivedFromTheAnalyticFormulaAndNotTheClamp()
    {
        const float tightestRadiusMetres = 60f;
        float analyticKmh = Mathf.Sqrt(MaxLateralAccel * tightestRadiusMetres) * 3.6f;

        var result = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);
        Assert.IsTrue(result.Success, $"Bake failed: {reason}");

        // The tightest corner's speed cannot EXCEED the analytic value: the pass only lowers.
        float slowest = float.MaxValue;
        int interior = 0;
        for (int i = 0; i < result.SpeedProfileKmh.Length; i++)
        {
            float v = result.SpeedProfileKmh[i];
            if (v < slowest) slowest = v;
            if (v > MinSpeedKmh + 0.01f && v < StraightSpeedKmh - 0.01f) interior++;
        }

        Assert.LessOrEqual(slowest, analyticKmh + 0.01f,
            $"Slowest profile speed is {slowest:F2} km/h, above the {analyticKmh:F2} km/h a 60 m " +
            "corner allows. The backward pass only lowers speeds, so this should be impossible.");

        // The real discriminator: corners whose speed came from the formula rather than a clamp.
        // Every conversion error collapses this count (299 without, 218 inverted, 501 correct).
        Assert.Greater(interior, 300,
            $"Only {interior} of {result.SpeedProfileKmh.Length} samples sit strictly between the " +
            $"{MinSpeedKmh} km/h floor and the {StraightSpeedKmh} km/h ceiling. Those are the " +
            "corner speeds the formula produced; too few means the km/h-to-m-s conversion is wrong. " +
            "This has been inverted in this codebase before.");
    }

    [Test]
    public void ABakeOfACleanCircuitSucceeds()
    {
        var result = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);

        Assert.IsTrue(result.Success, $"Bake failed: {reason}");
        Assert.Greater(result.LengthMetres, 1000f,
            "The synthetic circuit measures ~2350 m along its centreline; a bake far below that " +
            "means the arc-length parameterisation is broken, not that the track is short.");
        Assert.Greater(result.ControlPoints.Length, 8, "Too few control points to describe the shape.");
        Assert.Less(result.ControlPoints.Length, 120,
            $"Decimation should reduce ~400 raw points to ~60 control points, got {result.ControlPoints.Length}.");
    }

    [Test]
    public void ABakedProfileIsABackwardPassAndSoCannotCliff()
    {
        var result = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);
        Assert.IsTrue(result.Success, $"Bake failed: {reason}");

        // This is the invariant the waypoint array could not hold, and it is specifically about
        // BRAKING: a backward pass over a closed loop answers "can I shed enough speed to reach
        // the next limit?", so the largest DROP between adjacent samples is bounded by what the
        // brakes can shed over one sample. Measured 5.0 km/h here, against 23.0 km/h for the old
        // Menger-radius algorithm on this same circuit.
        //
        // It deliberately does NOT assert on the rise. Asserting Mathf.Abs would conflate braking
        // with acceleration, and acceleration is not this pass's job: the profile is a CEILING,
        // and how fast a car gets back up to that ceiling is bounded by its powertrain, not by the
        // track. The old algorithm's worst rise was 104 km/h too — also large, and never the defect
        // this programme exists to fix.
        float worstDrop = 0f;
        int worstAt = 0;
        for (int i = 0; i < result.SpeedProfileKmh.Length; i++)
        {
            int j = (i + 1) % result.SpeedProfileKmh.Length;
            float drop = result.SpeedProfileKmh[i] - result.SpeedProfileKmh[j];
            if (drop > worstDrop) { worstDrop = drop; worstAt = i; }
        }

        // Over 2 m at 18 m/s^2, the largest physically possible drop is v^2/(2a) difference,
        // which at 240 km/h (66.7 m/s) over 2 m is about 0.25 m/s = 0.9 km/h. The measured 5.04 is
        // in fact a constant of the FLOOR: at the 90 km/h minimum, reachable speed over one 2 m gap
        // is 95.04, so this assertion reads as "the backward pass converged everywhere", not as a
        // measurement of the geometry. That is the invariant worth keeping.
        Assert.Less(worstDrop, 6f,
            $"Profile drops {worstDrop:F2} km/h between adjacent samples at index {worstAt}. " +
            "A backward pass cannot produce that; something upstream is wrong.");

        // The RISE is tracked deliberately rather than left unasserted. The spec's success
        // criterion (design doc, "Success is measured, not asserted", item 2) says the profile
        // "cannot step between adjacent samples" and that the test must assert it, with no
        // directional qualifier. The braking drop is met with 1.5x margin; the rise is NOT.
        //
        // A rise is not a braking defect — a backward pass is a Mathf.Min and structurally cannot
        // bound one, and the profile is a CEILING that the car's powertrain climbs at its own rate.
        // But asserting only the drop made the spec's criterion permanently invisible to CI, which
        // is a worse failure than measuring it and finding it unmet. If this number is what a
        // consumer of speedProfileKmh actually sees, Task 4 needs to know rather than discover it.
        float worstRise = 0f;
        int riseAt = 0;
        for (int i = 0; i < result.SpeedProfileKmh.Length; i++)
        {
            int j = (i + 1) % result.SpeedProfileKmh.Length;
            float rise = result.SpeedProfileKmh[j] - result.SpeedProfileKmh[i];
            if (rise > worstRise) { worstRise = rise; riseAt = i; }
        }

        Assert.Less(worstRise, 130f,
            $"Profile rises {worstRise:F2} km/h between adjacent samples at index {riseAt}, which " +
            "exceeds the tracked ceiling. This is a corner exit: the ceiling lifts from the corner " +
            "speed to the straight speed and the car's own acceleration bridges it. If this number " +
            "grows, a consumer reading the profile at 2 m resolution is seeing a step.");
    }

    [Test]
    public void ABakedProfileRespectsItsOwnPhysicalCeiling()
    {
        var result = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);
        Assert.IsTrue(result.Success, $"Bake failed: {reason}");

        for (int i = 0; i < result.SpeedProfileKmh.Length; i++)
        {
            Assert.That(result.SpeedProfileKmh[i], Is.LessThanOrEqualTo(StraightSpeedKmh + 0.01f),
                $"Sample {i} exceeds the straight speed ceiling.");
            Assert.That(result.SpeedProfileKmh[i], Is.GreaterThanOrEqualTo(MinSpeedKmh - 0.01f),
                $"Sample {i} is below the configured floor; a speed limit below {MinSpeedKmh} km/h " +
                "is a geometry artefact, not a corner.");
            Assert.That(float.IsNaN(result.SpeedProfileKmh[i]), Is.False, "NaN in the speed profile.");
        }
    }

    [Test]
    public void ABakeOfDegenerateInputIsRefusedWithAReason()
    {
        // Review Focus #5: a track with no valid bake must be refused, never silently degenerate.
        //
        // Sixteen collinear points, NOT four. The baker refuses on `count < 8` first, so a 4-point
        // input never reaches the collinearity branch and the test silently measures the wrong
        // thing -- which is how the original version of this test passed while testing nothing.
        var collinear = new List<Vector3>();
        for (int i = 0; i < 16; i++)
            collinear.Add(new Vector3(i * 10f, 0f, 0f));

        var result = RacingLineBaker.Bake(collinear, MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);

        Assert.IsFalse(result.Success, "A collinear centreline is not a circuit and must be refused.");
        Assert.IsNotEmpty(reason, "A refusal must name its reason so a track author can act on it.");
    }

    [Test]
    public void ABakeOfACentrelineThatPinchesOntoItselfIsRefused()
    {
        // The fixture this task originally used was exactly this shape and it BAKED SUCCESSFULLY.
        // A centreline that passes within a few metres of itself is not a circuit, and a speed
        // profile derived from it is meaningless -- but nothing in the arithmetic says so, because
        // every individual point is well formed. Only an explicit check catches it.
        var pinching = new List<Vector3>();
        const int samples = 200;
        for (int i = 0; i < samples; i++)
        {
            float angle = i / (float)samples * Mathf.PI * 2f;
            float radius = 300f + 220f * Mathf.Sin(angle * 2f) + 120f * Mathf.Sin(angle * 5f);
            pinching.Add(new Vector3(Mathf.Cos(angle) * radius, 0f, Mathf.Sin(angle) * radius));
        }

        var result = RacingLineBaker.Bake(pinching, MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);

        Assert.IsFalse(result.Success,
            "A centreline whose tightest corner is 0.3 m in radius is not a circuit -- that is a " +
            "numerical pinch, and accepting it produces a speed profile full of artefacts.");
        Assert.IsNotEmpty(reason);
        StringAssert.Contains("pinch", reason.ToLowerInvariant(),
            "The refusal should say what went wrong in terms a track author can act on.");
    }

    [Test]
    public void ABakeOfTooFewPointsIsRefusedWithAReason()
    {
        var tiny = new List<Vector3> { Vector3.zero, Vector3.one, new Vector3(2f, 0f, 0f) };
        var result = RacingLineBaker.Bake(tiny, MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);

        Assert.IsFalse(result.Success);
        Assert.IsNotEmpty(reason);
    }

    [Test]
    public void BakingTheSameCircuitTwiceGivesTheSameAnswer()
    {
        // Determinism. A bake that varies run to run makes every later comparison meaningless.
        var first = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out _);
        var second = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out _);

        Assert.IsTrue(first.Success && second.Success);
        Assert.AreEqual(first.ControlPoints.Length, second.ControlPoints.Length);
        Assert.AreEqual(first.SpeedProfileKmh.Length, second.SpeedProfileKmh.Length);
        for (int i = 0; i < first.SpeedProfileKmh.Length; i++)
            Assert.That(first.SpeedProfileKmh[i], Is.EqualTo(second.SpeedProfileKmh[i]).Within(1e-4f));
    }

    [Test]
    public void ASectorSplitCoversTheLapWithoutGapsOrOverlap()
    {
        var result = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);
        Assert.IsTrue(result.Success, $"Bake failed: {reason}");

        Assert.AreEqual(3, result.SectorStarts.Length);
        Assert.That(result.SectorStarts[0], Is.EqualTo(0f).Within(0.01f));
        Assert.That(result.SectorStarts[1], Is.GreaterThan(0f));
        Assert.That(result.SectorStarts[2], Is.GreaterThan(result.SectorStarts[1]),
            "Sector 3 must start after sector 2.");
        Assert.That(result.SectorStarts[2], Is.LessThan(result.LengthMetres),
            "Sector 3 must start before the lap ends.");
    }
}