using NUnit.Framework;
using UnityEngine;

public class ArcLengthSplineTests
{
    private static Vector3[] Square(int perSide = 4)
    {
        var points = new System.Collections.Generic.List<Vector3>();
        for (int side = 0; side < 4; side++)
        {
            for (int i = 0; i < perSide; i++)
            {
                float t = i / (float)perSide;
                switch (side)
                {
                    case 0: points.Add(new Vector3(100f * t, 0f, 0f)); break;
                    case 1: points.Add(new Vector3(100f, 0f, 100f * t)); break;
                    case 2: points.Add(new Vector3(100f - 100f * t, 0f, 100f)); break;
                    default: points.Add(new Vector3(0f, 0f, 100f - 100f * t)); break;
                }
            }
        }
        return points.ToArray();
    }

    [Test]
    public void ASplineInterpolatesItsControlPoints()
    {
        Vector3[] control = Square();
        var spline = new ArcLengthSpline(control, true);

        for (int i = 0; i < control.Length; i++)
        {
            float s = spline.ArcLengthOfControlPoint(i);
            float d = Vector3.Distance(spline.GetPosition(s), control[i]);
            Assert.That(d, Is.LessThan(0.05f),
                $"Control point {i} is not interpolated: the spline passes {d:F3} m from it.");
        }
    }

    [Test]
    public void CurvatureIsContinuousAlongTheWholeSpline()
    {
        var spline = new ArcLengthSpline(Square(), true);

        // Measures CONTINUITY, not rate of change.
        //
        // A first difference divided by step measures |dk/ds|, which conflates "curvature jumps"
        // with "curvature changes fast". A tight corner does the second legitimately: this square
        // turns 90 degrees over ~25 m, so |dk/ds| near a corner is genuinely large and no
        // implementation can reduce it without changing the geometry.
        //
        // The second difference separates them. For a smooth function it is O(h^2); for a genuine
        // jump of size J it is O(J). At h = 0.05 m, smooth variation contributes ~0.8 * h^2 =
        // 0.002, so anything above ~0.05 is a discontinuity, not a corner.
        const float h = 0.05f;
        float worst = 0f, worstAt = 0f;
        int samples = Mathf.CeilToInt(spline.Length / h);

        for (int i = 1; i < samples - 1; i++)
        {
            float s = i * h;
            float second = Mathf.Abs(
                spline.GetCurvature(s - h) - 2f * spline.GetCurvature(s) + spline.GetCurvature(s + h));

            if (second > worst) { worst = second; worstAt = s; }
        }

        Assert.Less(worst, 0.05f,
            $"Curvature is discontinuous: second difference reaches {worst:F4} at s={worstAt:F1} m. " +
            "Smooth variation contributes ~0.002 at this step, so this is a jump in curvature, " +
            "which is the artefact the waypoint array produced.");
    }

    [Test]
    public void CurvatureMatchesTheAnalyticValueOnACircleOfKnownRadius()
    {
        const float radius = 80f;
        const int knots = 64;
        var control = new Vector3[knots];
        for (int i = 0; i < knots; i++)
        {
            float a = i / (float)knots * Mathf.PI * 2f;
            control[i] = new Vector3(Mathf.Cos(a) * radius, 0f, Mathf.Sin(a) * radius);
        }

        var spline = new ArcLengthSpline(control, true);
        float expected = 1f / radius;

        for (int i = 0; i < 200; i++)
        {
            float s = spline.Length * i / 200f;
            float actual = spline.GetCurvature(s);
            // 2% tolerance: curvature comes from a central difference over the arc-length
            // inversion, so its accuracy is bounded by the LUT resolution, not by the spline.
            Assert.That(Mathf.Abs(actual - expected), Is.LessThan(expected * 0.02f),
                $"Curvature at s={s:F1} m is {actual:F5}, expected ~{expected:F5}.");
        }
    }

    [Test]
    public void PositionIsContinuousAcrossTheStartFinishSeam()
    {
        var spline = new ArcLengthSpline(Square(), true);

        Vector3 justBefore = spline.GetPosition(spline.Length - 0.01f);
        Vector3 atStart = spline.GetPosition(0f);
        Assert.That(Vector3.Distance(justBefore, atStart), Is.LessThan(0.05f),
            "The spline must wrap continuously; a seam jump is a teleport at the start/finish line.");

        Vector3 tBefore = spline.GetTangent(spline.Length - 0.01f);
        Vector3 tAtStart = spline.GetTangent(0f);
        Assert.That(Vector3.Dot(tBefore, tAtStart), Is.GreaterThan(0.99f),
            "Tangent must be continuous across the seam.");
    }

    [Test]
    public void TangentIsUnitLengthAndPerpendicularToTheNormal()
    {
        var spline = new ArcLengthSpline(Square(), true);
        for (int i = 0; i < 100; i++)
        {
            float s = spline.Length * i / 100f;
            Assert.That(spline.GetTangent(s).magnitude, Is.EqualTo(1f).Within(1e-3f));
            Assert.That(Mathf.Abs(Vector3.Dot(spline.GetTangent(s), spline.GetNormal(s))),
                Is.LessThan(1e-3f), "Tangent and normal must be perpendicular.");
        }
    }

    [Test]
    public void DegenerateInputProducesFiniteValuesRatherThanNaN()
    {
        var collinear = new[]
        {
            new Vector3(0f, 0f, 0f), new Vector3(10f, 0f, 0f),
            new Vector3(20f, 0f, 0f), new Vector3(30f, 0f, 0f)
        };
        var duplicated = new[]
        {
            new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 0f),
            new Vector3(10f, 0f, 0f), new Vector3(20f, 0f, 0f)
        };

        foreach (var control in new[] { collinear, duplicated })
        {
            var spline = new ArcLengthSpline(control, false);
            for (int i = 0; i < 20; i++)
            {
                float s = spline.Length * i / 20f;
                Assert.That(float.IsNaN(spline.GetPosition(s).x), Is.False, "NaN position.");
                Assert.That(float.IsNaN(spline.GetCurvature(s)), Is.False, "NaN curvature.");
                Assert.That(float.IsNaN(spline.GetTangent(s).x), Is.False, "NaN tangent.");
            }
        }
    }

    [Test]
    public void TooFewControlPointsIsReportedRatherThanThrowing()
    {
        var tooFew = new ArcLengthSpline(new[] { Vector3.zero, Vector3.one }, false);
        Assert.IsFalse(tooFew.IsValid, "A 2-point line is not a spline and must report itself invalid.");
    }

    [Test]
    public void ArcLengthOfACircleApproachesTwoPiR()
    {
        const float radius = 80f;
        const int knots = 256;
        var control = new Vector3[knots];
        for (int i = 0; i < knots; i++)
        {
            float a = i / (float)knots * Mathf.PI * 2f;
            control[i] = new Vector3(Mathf.Cos(a) * radius, 0f, Mathf.Sin(a) * radius);
        }

        var spline = new ArcLengthSpline(control, true);
        float expected = 2f * Mathf.PI * radius;
        Assert.That(spline.Length, Is.EqualTo(expected).Within(expected * 0.02f),
            $"Circumference should be ~{expected:F2} m, measured {spline.Length:F2} m.");
    }
}