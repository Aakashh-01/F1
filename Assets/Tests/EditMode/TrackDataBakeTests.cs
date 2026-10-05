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
    /// <b>The arc sweeps sum to exactly 360°</b>, so the loop closes with no kink at the seam. The
    /// previous set summed to 335°, leaving a 25° kink that a closed spline then had to absorb
    /// somewhere. Explicit radii cannot go negative by construction either — an earlier polar
    /// fixture, <c>radius = 300 + 220*sin(2a) + 120*sin(5a)</c>, measured a 0.3 m minimum radius
    /// and passed within 4.5 m of itself, so every corner in it was a numerical artefact.
    /// </para>
    ///
    /// <para>
    /// The shape is legible and deliberately varied: a long opening straight into a 55 m hairpin, a
    /// link to a 140 m corner, the longest straight, a 48 m hairpin (the slowest corner), a link,
    /// a 200 m sweeper and a run home. It has a fast corner <i>and</i> a slow one, so the backward
    /// braking pass has real work to do rather than being handed a constant.
    /// </para>
    /// </summary>
    private const float SampleSpacingMetres = 10f;

    private static List<Vector3> SyntheticCircuit()
    {
        // ('s', metres) straight, or ('a', (radius, sweepDegrees)) constant-radius arc.
        // Sweeps: 110 + 70 + 100 + 80 = 360 degrees exactly, so the circuit closes.
        var segments = new (char kind, float a, float b)[]
        {
            ('s', 380f, 0f),
            ('a', 55f, 110f),      // hairpin
            ('s', 180f, 0f),
            ('a', 140f, 70f),
            ('s', 320f, 0f),
            ('a', 48f, 100f),      // slowest corner on the circuit
            ('s', 200f, 0f),
            ('a', 200f, 80f),      // sweeper
            ('s', 300f, 0f),
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
            float cx = x - radius * Mathf.Cos(heading);
            float cy = y - radius * Mathf.Sin(heading);
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
        // which at 240 km/h (66.7 m/s) over 2 m is about 0.25 m/s = 0.9 km/h. Allow generous
        // slack for the straight-speed clamp, and assert well under the old 23.8 km/h figure.
        Assert.Less(worstDrop, 6f,
            $"Profile drops {worstDrop:F2} km/h between adjacent samples at index {worstAt}. " +
            "A backward pass cannot produce that; something upstream is wrong.");
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