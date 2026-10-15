using UnityEngine;

namespace F1.TrackData
{
    /// <summary>
    /// The immutable per-race view of a track. Built once, then read every tick by every car.
    ///
    /// <para>
    /// This is the layer that replaces index-as-position. <see cref="ProjectToArcLength"/> is the
    /// single operation that three separate consumers each currently re-derive in their own way:
    /// <c>AIDriverController.UpdateProgressIndex</c>, <c>LapTracker.BuildArcTable</c> and
    /// <c>AILongitudinalPlanner.SolvePlannedSpeedKmh</c>. It is also what track-limit detection
    /// and sector timing need, and both of those are in the GDD.
    /// </para>
    ///
    /// <para>
    /// Read-only and allocation-free after construction. The per-tick path allocates nothing,
    /// because twenty cars calling this every physics step is the AI CPU budget pdf1 §4.1 names.
    /// </para>
    /// </summary>
    public sealed class TrackRuntime
    {
        /// <summary>Arc length between coarse search samples, metres.</summary>
        private const float CoarseStepMetres = 5f;

        /// <summary>Projection refinement iterations. See <see cref="ProjectToArcLength"/>.</summary>
        private const int RefineIterations = 6;

        private readonly ArcLengthSpline _centreline;
        private readonly float[] _speedProfileKmh;
        private readonly float[] _widthMetres;
        private readonly float[] _sectorStarts;
        private readonly float _length;
        private readonly float _speedStep;
        private readonly float _widthStep;

        /// <summary>
        /// Coarse samples for the projection search, built once. A lap is a few hundred of these,
        /// and scanning all of them per call is cheaper than maintaining any kind of spatial index
        /// for a loop that has no interior.
        /// </summary>
        private readonly Vector3[] _coarsePoints;

        public bool IsValid { get; }
        public float Length => _length;
        public ArcLengthSpline Centreline => _centreline;
        public float ReferenceLapSeconds { get; }

        /// <summary>
        /// Builds the runtime view. Never throws: a missing or unusable asset yields an invalid
        /// instance, because this runs during scene load and a throw there would take down a race
        /// rather than one car.
        /// </summary>
        public TrackRuntime(TrackData data)
        {
            ReferenceLapSeconds = 0f;

            if (data == null || data.ControlPoints == null || data.ControlPoints.Length < 3)
            {
                _centreline = null;
                _speedProfileKmh = System.Array.Empty<float>();
                _widthMetres = System.Array.Empty<float>();
                _sectorStarts = new float[3];
                _coarsePoints = System.Array.Empty<Vector3>();
                _length = 0f;
                _speedStep = 0f;
                _widthStep = 0f;
                IsValid = false;
                return;
            }

            _centreline = new ArcLengthSpline(data.ControlPoints, data.Closed);
            if (!_centreline.IsValid)
            {
                _speedProfileKmh = System.Array.Empty<float>();
                _widthMetres = System.Array.Empty<float>();
                _sectorStarts = new float[3];
                _coarsePoints = System.Array.Empty<Vector3>();
                _length = 0f;
                _speedStep = 0f;
                _widthStep = 0f;
                IsValid = false;
                return;
            }

            _speedProfileKmh = data.SpeedProfileKmh ?? System.Array.Empty<float>();
            _widthMetres = data.WidthMetres ?? System.Array.Empty<float>();
            _sectorStarts = data.SectorStarts != null && data.SectorStarts.Length == 3
                ? data.SectorStarts
                : new float[3];
            _length = _centreline.Length;
            ReferenceLapSeconds = data.ReferenceLapSeconds;

            _speedStep = _speedProfileKmh.Length > 1 ? _length / _speedProfileKmh.Length : 0f;
            _widthStep = _widthMetres.Length > 1 ? _length / _widthMetres.Length : 0f;

            int count = Mathf.Max(4, Mathf.CeilToInt(_length / CoarseStepMetres));
            _coarsePoints = new Vector3[count];
            for (int i = 0; i < count; i++)
                _coarsePoints[i] = _centreline.GetPosition(_length * i / count);

            IsValid = true;
        }

        public float WrapDistance(float s) => _centreline != null ? _centreline.WrapDistance(s) : 0f;

        public Vector3 GetPosition(float s) => _centreline != null ? _centreline.GetPosition(s) : Vector3.zero;
        public Vector3 GetTangent(float s) => _centreline != null ? _centreline.GetTangent(s) : Vector3.forward;
        public Vector3 GetNormal(float s) => _centreline != null ? _centreline.GetNormal(s) : Vector3.right;
        public float GetCurvature(float s) => _centreline != null ? _centreline.GetCurvature(s) : 0f;

        /// <summary>
        /// Physical speed limit at arc length s, km/h. Car-agnostic: this is what the tyres can do,
        /// not what any particular car does.
        /// </summary>
        public float GetSpeedLimitKmh(float s) => SampleProfile(_speedProfileKmh, _speedStep, s);

        /// <summary>Full track width at arc length s, metres. Half-width from the centreline is half this.</summary>
        public float GetWidth(float s) => SampleProfile(_widthMetres, _widthStep, s);

        /// <summary>
        /// Which of the three sectors contains arc length s, as 0, 1 or 2 — matching
        /// <c>F1.GameData.SectorIndex</c> (Sector1 = 0, Sector2 = 1, Sector3 = 2).
        ///
        /// <para>
        /// Returned as an int rather than that enum on purpose: <c>F1.GameData</c> already
        /// references <c>F1.Gameplay</c>, and <c>F1.Gameplay</c> will reference this assembly once
        /// <c>AIRacingLine</c> wraps it, so naming the enum here would close a reference cycle.
        /// Cast at the call site, which is where the sector-timing feature that needs it lives.
        /// </para>
        /// </summary>
        public int GetSector(float s)
        {
            s = WrapDistance(s);
            if (s >= _sectorStarts[2]) return 2;
            if (s >= _sectorStarts[1]) return 1;
            return 0;
        }

        /// <summary>Arc length at the start of the given sector, metres.</summary>
        public float GetSectorStart(int sector)
        {
            if (_sectorStarts.Length != 3) return 0f;
            return _sectorStarts[Mathf.Clamp(sector, 0, 2)];
        }

        private float SampleProfile(float[] samples, float step, float s)
        {
            if (samples.Length == 0 || step <= 0f) return 0f;

            s = WrapDistance(s);
            float f = s / step;
            int i = Mathf.Clamp(Mathf.FloorToInt(f), 0, samples.Length - 1);
            int j = Mathf.Min(i + 1, samples.Length - 1);
            return Mathf.Lerp(samples[i], samples[j], Mathf.Clamp01(f - i));
        }

        /// <summary>
        /// Projects a world position onto the centreline, returning arc length and signed lateral
        /// offset (positive to the right of travel).
        ///
        /// <para>
        /// Coarse scan for the nearest sample, then refinement by repeatedly stepping the guess
        /// along the curve's own tangent:
        /// <c>s += dot(world - P(s), T(s))</c>. This uses the analytic tangent rather than a
        /// differenced derivative, so the step is well founded and converges in a handful of
        /// iterations from the coarse scan's few-metre starting error.
        /// </para>
        ///
        /// <para>
        /// Returns false when the track is unusable. Callers must hold their last valid value rather
        /// than guess: a car that cannot find itself on the line has to slow down, not teleport.
        /// </para>
        /// </summary>
        public bool ProjectToArcLength(Vector3 world, out float s, out float lateralOffset)
        {
            s = 0f;
            lateralOffset = 0f;
            if (!IsValid) return false;

            Vector3 planar = Vector3.ProjectOnPlane(world, Vector3.up);

            int best = 0;
            float bestSqr = float.MaxValue;
            for (int i = 0; i < _coarsePoints.Length; i++)
            {
                Vector3 delta = _coarsePoints[i] - planar;
                float sqr = delta.sqrMagnitude;
                if (sqr < bestSqr)
                {
                    bestSqr = sqr;
                    best = i;
                }
            }

            float guess = _length * best / _coarsePoints.Length;

            for (int iteration = 0; iteration < RefineIterations; iteration++)
            {
                guess = WrapDistance(guess);
                Vector3 onLine = GetPosition(guess);
                Vector3 delta = planar - Vector3.ProjectOnPlane(onLine, Vector3.up);
                guess += Vector3.Dot(delta, GetTangent(guess));
            }

            s = WrapDistance(guess);

            Vector3 final = GetPosition(s);
            lateralOffset = Vector3.Dot(
                Vector3.ProjectOnPlane(world - final, Vector3.up), GetNormal(s));

            return true;
        }
    }
}