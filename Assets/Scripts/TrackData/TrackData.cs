using UnityEngine;

namespace F1.TrackData
{
    /// <summary>
    /// Everything a track contributes to gameplay, baked once and stored as an asset.
    ///
    /// <para>
    /// This is the answer to "we will have four or five tracks": a new track is a new asset plus
    /// a scene, and nothing else. No hand-placed waypoints, no per-track AI retuning.
    /// </para>
    ///
    /// <para>
    /// <b>Car-agnostic by design.</b> The speed profile here is a PHYSICAL limit — what the tyres
    /// can do — not what one car does. A Gen1 rental and a Gen3 unlock then differ because their
    /// CarDefinition differs, which is what pdf2's car-unlock progression (50k / 75k / 100k points)
    /// requires. A profile baked against one car's downforce would silently encode that car into
    /// every track on the project.
    /// </para>
    ///
    /// <para>
    /// Authoring fields and baked fields are separated deliberately. The control points are
    /// hand-editable; everything else is machine-derived and must be regenerated whenever the
    /// points change. <see cref="IsBaked"/> reports whether the two are in sync.
    /// </para>
    /// </summary>
    [CreateAssetMenu(menuName = "F1/Track Data", fileName = "TrackData_")]
    public class TrackData : ScriptableObject
    {
        [Header("Identity")]
        [Tooltip("Stable id — used for save keys and leaderboards. Never change after release.")]
        [SerializeField] private string _trackId;
        public string TrackId => _trackId;

        [Header("Authoring — edit these")]
        [Tooltip("Centreline control points, ~60 for a real circuit. The spline carries the shape.")]
        [SerializeField] private Vector3[] _controlPoints = new Vector3[0];
        public Vector3[] ControlPoints => _controlPoints;

        [Tooltip("Whether the centreline is a closed loop. Every circuit is.")]
        [SerializeField] private bool _closed = true;
        public bool Closed => _closed;

        [Header("Baked — regenerate with Tools/F1/Bake Track Data")]
        [Tooltip("Physical speed limit in km/h, sampled uniformly in arc length.")]
        [SerializeField] private float[] _speedProfileKmh = new float[0];
        public float[] SpeedProfileKmh => _speedProfileKmh;

        [Tooltip("Track width in metres, sampled uniformly in arc length.")]
        [SerializeField] private float[] _widthMetres = new float[0];
        public float[] WidthMetres => _widthMetres;

        [Tooltip("Total arc length of the centreline, metres.")]
        [SerializeField] private float _lengthMetres;
        public float LengthMetres => _lengthMetres;

        [Tooltip("A clean lap at the physical limit, seconds. The yardstick tiers are paced against.")]
        [SerializeField] private float _referenceLapSeconds;
        public float ReferenceLapSeconds => _referenceLapSeconds;

        [Tooltip("Arc length at the start of each of the three sectors, metres from the line start.")]
        [SerializeField] private float[] _sectorStarts = new float[3] { 0f, 0f, 0f };
        public float[] SectorStarts => _sectorStarts;

        [Tooltip("True when the baked arrays match the current control points.")]
        [SerializeField] private bool _isBaked;
        public bool IsBaked => _isBaked;

        /// <summary>Metres between speed-profile samples. Finer where curvature changes fast.</summary>
        public const float SpeedProfileResolutionMetres = 2f;

        /// <summary>Metres between width samples. Width varies slowly; this is plenty.</summary>
        public const float WidthResolutionMetres = 10f;

        /// <summary>Sample count for a lap, never below this however short the track.</summary>
        public const int MinimumProfileSamples = 32;

#if UNITY_EDITOR
        /// <summary>
        /// Authoring seam. Sets the control points and invalidates the bake, because changing the
        /// points invalidates everything derived from them.
        /// </summary>
        public void SetControlPoints(Vector3[] points, bool closed)
        {
            _controlPoints = points ?? new Vector3[0];
            _closed = closed;
            _isBaked = false;
        }

        /// <summary>
        /// Authoring seam. Fills every baked array with a single constant value.
        ///
        /// <para>
        /// A flat profile is not physically real — a circuit has corners — but it is the honest
        /// constant: it makes the arc-length machinery testable without pretending to have solved
        /// the speed derivation, which is a separate concern with its own tests.
        /// </para>
        /// </summary>
        public void WriteUniformProfile(float speedKmh, float widthMetres)
        {
            float length = _controlPoints != null && _controlPoints.Length >= 3
                ? new ArcLengthSpline(_controlPoints, _closed).Length
                : 1f;
            _lengthMetres = Mathf.Max(length, 1f);

            int speedCount = Mathf.Max(MinimumProfileSamples,
                Mathf.CeilToInt(_lengthMetres / SpeedProfileResolutionMetres));
            _speedProfileKmh = new float[speedCount];
            for (int i = 0; i < speedCount; i++) _speedProfileKmh[i] = speedKmh;

            int widthCount = Mathf.Max(MinimumProfileSamples,
                Mathf.CeilToInt(_lengthMetres / WidthResolutionMetres));
            _widthMetres = new float[widthCount];
            for (int i = 0; i < widthCount; i++) _widthMetres[i] = widthMetres;

            _sectorStarts = new[] { 0f, _lengthMetres / 3f, _lengthMetres * 2f / 3f };
            _referenceLapSeconds = _lengthMetres / Mathf.Max(1f, speedKmh / 3.6f);
            _isBaked = true;
        }
#endif

#if UNITY_EDITOR
        private void OnValidate()
        {
            if (string.IsNullOrEmpty(_trackId))
                _trackId = name.Replace("TrackData_", "").ToLowerInvariant();

            if (_sectorStarts == null || _sectorStarts.Length != 3)
                _sectorStarts = new float[3] { 0f, 0f, 0f };
        }
#endif
    }
}