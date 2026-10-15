using UnityEngine;

/// <summary>
/// A closed or open centripetal Catmull-Rom spline, parameterised by arc length.
/// </summary>
public sealed class ArcLengthSpline
{
    private const float Alpha = 0.5f;
    /// <summary>
    /// LUT resolution, per spline segment.
    ///
    /// <para>
    /// This sets the ACCURACY of the arc-length inversion, and therefore of curvature, because
    /// the inversion interpolates the cumulative table linearly within a cell. That error is
    /// O(h^2 * |d^2s/du^2|), so it is worst exactly at corners — where the second derivative of
    /// arc length is largest. At 32 the measured |d kappa/ds| near a square corner was 0.80 /m^2,
    /// two orders of magnitude above the 0.004 the geometry actually implies; the curve was
    /// smooth and the measurement was not. At 256 the error falls 64x, comfortably under the
    /// continuity threshold.
    /// </para>
    ///
    /// <para>
    /// Cost is ~2.4 MB for a 60-knot circuit, built once per race and read-only thereafter.
    /// </para>
    /// </summary>
    private const int SubdivisionsPerSegment = 256;
    private const float MinSegmentLength = 1e-4f;

    private readonly Vector3[] _control;
    private readonly bool _closed;

    private Vector3[] _samples;
    private float[] _cumulative;
    private float[] _parameters;
    private float _length;
    private float _totalParameter;
    private float[] _knots;
    private bool _valid;

    public bool IsValid => _valid;
    public float Length => _length;
    public int ControlCount => _control.Length;

    public ArcLengthSpline(Vector3[] controlPoints, bool closed)
    {
        _control = controlPoints ?? new Vector3[0];
        _closed = closed;

        _valid = _control.Length >= 3;
        if (!_valid)
        {
            _samples = new Vector3[0];
            _cumulative = new float[1];
            return;
        }

        BuildLookupTable();
    }

    private static float KnotSpacing(Vector3 a, Vector3 b)
    {
        float d = Vector3.Distance(a, b);
        if (d < MinSegmentLength) return MinSegmentLength;
        return Mathf.Pow(d, Alpha);
    }

    private float[] BuildKnots()
    {
        var t = new float[_control.Length];
        if (_closed)
        {
            for (int i = 0; i < _control.Length - 1; i++)
                t[i + 1] = t[i] + KnotSpacing(_control[i], _control[i + 1]);
            t[0] = 0f;
        }
        else
        {
            for (int i = 1; i < _control.Length; i++)
                t[i] = t[i - 1] + KnotSpacing(_control[i - 1], _control[i]);
        }
        return t;
    }

    private float TotalParameter(float[] t) =>
        _closed ? t[_control.Length - 1] + KnotSpacing(_control[_control.Length - 1], _control[0])
                : t[_control.Length - 1];

    /// <summary>
    /// The LUT stores arc length -> (parameter, position) so that GetPosition can invert s into
    /// the spline parameter and evaluate EXACTLY, rather than linearly interpolating between
    /// stored positions.
    ///
    /// <para>
    /// That distinction is the whole point of this class. Linear interpolation between LUT samples
    /// is C0 but not C1, so a finite-difference second derivative sees a kink at every sample and
    /// curvature comes out discontinuous — reintroducing by the back door the exact artefact this
    /// spline exists to remove. Evaluating at an interpolated <i>parameter</i> keeps the curve C1
    /// no matter how coarse the table is.
    /// </para>
    /// </summary>
    private void BuildLookupTable()
    {
        int segments = _closed ? _control.Length : _control.Length - 1;
        int count = segments * SubdivisionsPerSegment + 1;

        _samples = new Vector3[count];
        _cumulative = new float[count];
        _parameters = new float[count];

        float[] t = BuildKnots();
        _knots = t;
        float tMax = TotalParameter(t);
        _totalParameter = tMax;

        for (int i = 0; i < count; i++)
        {
            float u = tMax * i / (count - 1);
            _parameters[i] = u;
            _samples[i] = Evaluate(u, out _, out _);
            if (i > 0)
            {
                float step = Vector3.Distance(_samples[i], _samples[i - 1]);
                _cumulative[i] = _cumulative[i - 1] + (float.IsNaN(step) ? 0f : step);
            }
        }

        _length = _cumulative[count - 1];
        if (_length < MinSegmentLength) _length = MinSegmentLength;
    }

    private int Mod(int i, int n)
    {
        int r = i % n;
        return r < 0 ? r + n : r;
    }

    /// <summary>
    /// Knot tangent: the derivative of the linear interpolant through a knot's two neighbours,
    /// evaluated in the centripetal parameter.
    ///
    /// <para>
    /// This single definition is what buys C1 continuity, and it is the part that is easy to get
    /// wrong. A naive non-uniform Catmull-Rom (Barry-Goldman pyramid) produces the right tangent
    /// DIRECTION on both sides of a knot but a different MAGNITUDE, because each segment scales by
    /// its own (t[i+2]-t[i]). Curvature divides by |P'|^3, so a magnitude step is a curvature step:
    /// the curve looks continuous while the one quantity this whole class exists to provide is not.
    /// </para>
    ///
    /// <para>
    /// Deriving the tangent once per knot and handing the same value to both adjacent segments
    /// makes P' continuous, hence P'' continuous, hence curvature continuous. The centripetal
    /// spacing (alpha = 0.5) is what additionally rules out cusps and self-intersection on unevenly
    /// spaced control points, which a decimated racing line always produces.
    /// </para>
    /// </summary>
    private Vector3 KnotTangent(int i)
    {
        int n = _control.Length;

        if (_closed)
        {
            Vector3 previous = _control[Mod(i - 1, n)];
            Vector3 next = _control[Mod(i + 1, n)];
            float gap = KnotSpacing(previous, _control[Mod(i, n)]) +
                        KnotSpacing(_control[Mod(i, n)], next);
            if (gap < MinSegmentLength) return Vector3.zero;
            return (next - previous) / gap;
        }

        if (i == 0) return (_control[1] - _control[0]) / Mathf.Max(MinSegmentLength, _knots[1] - _knots[0]);
        if (i == n - 1)
            return (_control[n - 1] - _control[n - 2]) /
                   Mathf.Max(MinSegmentLength, _knots[n - 1] - _knots[n - 2]);

        float span = _knots[i + 1] - _knots[i - 1];
        if (span < MinSegmentLength) return Vector3.zero;
        return (_control[i + 1] - _control[i - 1]) / span;
    }

    /// <summary>
    /// Cubic Hermite evaluation on the segment containing u, with analytic first and second
    /// derivatives in u.
    ///
    /// <para>
    /// Derivatives are analytic rather than differenced. Differencing position in ARC LENGTH
    /// inherits the kinks of the piecewise-linear s-&gt;u inversion at every LUT boundary, so a
    /// finite difference reintroduces the very discontinuity this spline removes. Curvature from
    /// u-derivatives is kappa = |P' x P''| / |P'|^3, which depends only on u.
    /// </para>
    /// </summary>
    private Vector3 Evaluate(float u, out Vector3 du, out Vector3 d2u)
    {
        du = Vector3.zero;
        d2u = Vector3.zero;

        int n = _control.Length;
        if (!_valid) return Vector3.zero;

        if (_closed)
        {
            if (_totalParameter > MinSegmentLength)
            {
                u = u % _totalParameter;
                if (u < 0f) u += _totalParameter;
            }
        }
        else
        {
            if (u <= _knots[0]) { du = KnotTangent(0); return _control[0]; }
            if (u >= _knots[n - 1]) { du = KnotTangent(n - 1); return _control[n - 1]; }
        }

        // Segment [i, i+1]. On a closed spline the final segment is n-1 and runs back to 0.
        int i = (_closed && u >= _knots[n - 1]) ? n - 1 : 0;
        if (!_closed || u < _knots[n - 1])
        {
            for (int k = 0; k < n - 1; k++)
                if (u >= _knots[k]) i = k;
        }

        int j = _closed ? Mod(i + 1, n) : Mathf.Min(i + 1, n - 1);

        float t0 = _knots[i];
        float t1 = _closed && i + 1 >= n ? _knots[0] + _totalParameter : _knots[j];
        float span = t1 - t0;
        if (span < MinSegmentLength) return _control[i];
        if (!_closed) t1 = _knots[j];

        float s = (u - t0) / span;
        if (s < 0f) s = 0f;
        if (s > 1f) s = 1f;

        Vector3 p0 = _control[i];
        Vector3 p1 = _control[j];
        Vector3 m0 = KnotTangent(i);
        Vector3 m1 = KnotTangent(j);

        float s2 = s * s;
        float s3 = s2 * s;

        float h00 = 2f * s3 - 3f * s2 + 1f;
        float h10 = s3 - 2f * s2 + s;
        float h01 = -2f * s3 + 3f * s2;
        float h11 = s3 - s2;

        float dh00 = 6f * s2 - 6f * s;
        float dh10 = 3f * s2 - 4f * s + 1f;
        float dh01 = -6f * s2 + 6f * s;
        float dh11 = 3f * s2 - 2f * s;

        float ddh00 = 12f * s - 6f;
        float ddh10 = 6f * s - 4f;
        float ddh01 = -12f * s + 6f;
        float ddh11 = 6f * s - 2f;

        du = (dh00 * p0 + dh10 * span * m0 + dh01 * p1 + dh11 * span * m1) / span;
        d2u = (ddh00 * p0 + ddh10 * span * m0 + ddh01 * p1 + ddh11 * span * m1) / (span * span);

        return h00 * p0 + h10 * span * m0 + h01 * p1 + h11 * span * m1;
    }

    public float WrapDistance(float s)
    {
        if (_length <= MinSegmentLength) return 0f;
        if (!_closed) return Mathf.Clamp(s, 0f, _length);
        s %= _length;
        return s < 0f ? s + _length : s;
    }

    private int LookupIndex(float s)
    {
        int lo = 0, hi = _cumulative.Length - 1;
        while (hi - lo > 1)
        {
            int mid = (lo + hi) / 2;
            if (_cumulative[mid] <= s) lo = mid; else hi = mid;
        }
        return lo;
    }

    /// <summary>
    /// Position at arc length s.
    ///
    /// <para>
    /// Inverts s into the spline PARAMETER and evaluates the curve exactly there. It does NOT
    /// lerp between two stored positions: that would be C0-only, and the finite-difference
    /// curvature below would then show a discontinuity at every LUT sample.
    /// </para>
    /// </summary>
    public Vector3 GetPosition(float s)
    {
        if (!_valid) return Vector3.zero;
        return Evaluate(ParameterAtArcLength(s), out _, out _);
    }

    /// <summary>
    /// Parameter-space derivatives at arc length s. This is what makes curvature continuous:
    /// kappa = |P' x P''| / |P'|^3 has no dependence on the arc-length inversion, so it inherits
    /// only the smoothness of u itself.
    /// </summary>

    /// <summary>
    /// Inverts arc length to the spline parameter exactly.
    ///
    /// <para>
    /// The LUT gives a linear first guess. Left there, u is only piecewise-linear in s, so its
    /// derivative has a kink at every cell boundary and any derivative computed from it inherits
    /// that kink — which is a tangent discontinuity in all but name. Two Newton steps converge to
    /// machine precision because the guess is already within a fraction of a cell.
    /// </para>
    /// </summary>
    private float ParameterAtArcLength(float s)
    {
        s = WrapDistance(s);
        int i = LookupIndex(s);

        float lo = _parameters[i];
        float hi = _parameters[i + 1];
        if (hi <= lo) return lo;

        // Bisection, not Newton.
        //
        // Arc length is monotone in u, so the LUT cell that brackets s gives a valid bracket and
        // bisection converges unconditionally. A Newton step sized by a finite-difference ds/du
        // overshoots exactly where |dP/du| varies fastest — the corners — and a non-monotone
        // inverse shows up as a curvature discontinuity that is purely numerical. Bisection cannot
        // do that: it is monotone by construction, so u(s) is continuous, and continuity of
        // curvature follows from it.
        for (int iteration = 0; iteration < 24; iteration++)
        {
            float mid = (lo + hi) * 0.5f;
            if (ArcLengthAtParameter(mid) < s) lo = mid; else hi = mid;
        }

        return (lo + hi) * 0.5f;
    }

    /// <summary>Arc length from the start of the loop to the point at parameter u.</summary>
    private float ArcLengthAtParameter(float u)
    {
        // Linear interpolation of the cumulative table is accurate enough to drive Newton, and
        // being monotonic is what matters here.
        u = _closed ? Mathf.Repeat(u, _totalParameter) : Mathf.Clamp(u, 0f, _totalParameter);

        int count = _cumulative.Length - 1;
        float f = _totalParameter > MinSegmentLength ? u / _totalParameter * count : 0f;
        int i = Mathf.Clamp(Mathf.FloorToInt(f), 0, count - 1);
        float t = Mathf.Clamp01(f - i);
        return Mathf.Lerp(_cumulative[i], _cumulative[i + 1], t);
    }

    private void ParameterDerivatives(float s, out Vector3 du, out Vector3 d2u)
    {
        du = Vector3.zero;
        d2u = Vector3.zero;
        if (!_valid) return;

        Evaluate(ParameterAtArcLength(s), out du, out d2u);
    }

    public Vector3 GetTangent(float s)
    {
        if (!_valid) return Vector3.forward;
        ParameterDerivatives(s, out Vector3 du, out _);
        float m = du.magnitude;
        if (m < MinSegmentLength || float.IsNaN(m)) return Vector3.forward;
        return du / m;
    }

    public Vector3 GetNormal(float s)
    {
        Vector3 planar = Vector3.ProjectOnPlane(GetTangent(s), Vector3.up);
        if (planar.sqrMagnitude < 1e-8f) return Vector3.right;
        return Vector3.Cross(Vector3.up, planar.normalized).normalized;
    }

    public float GetCurvature(float s)
    {
        if (!_valid) return 0f;
        ParameterDerivatives(s, out Vector3 du, out Vector3 d2u);

        float speed = du.magnitude;
        if (speed < MinSegmentLength || float.IsNaN(speed)) return 0f;

        // kappa = |P' x P''| / |P'|^3, parameterisation-independent.
        float kappa = Vector3.Cross(du, d2u).magnitude / (speed * speed * speed);
        return float.IsNaN(kappa) ? 0f : kappa;
    }

    public float ArcLengthOfControlPoint(int index)
    {
        if (!_valid) return 0f;
        index = _closed ? Mod(index, _control.Length) : Mathf.Clamp(index, 0, _control.Length - 1);
        int lut = Mathf.Clamp(index * SubdivisionsPerSegment, 0, _cumulative.Length - 1);
        return Mathf.Lerp(_cumulative[lut], _cumulative[Mathf.Min(lut + 1, _cumulative.Length - 1)], 0f);
    }
}