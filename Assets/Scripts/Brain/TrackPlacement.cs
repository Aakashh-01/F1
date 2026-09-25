using UnityEngine;

/// <summary>
/// The canonical placement contract for a piece of track content.
///
/// Slice step S2's deliverable. The plan requires that pre-race and race use the *same*
/// track start basis, and that the race grid offset is derived from that same basis:
///
///     QualifyingSpawn = TrackStartPosition + TrackStartRotation
///     RaceSpawn      = GridSlots[gridPosition] relative to TrackStartPosition
///
/// This component is the single place those references live, so a track scene can be
/// loaded once and answer both questions consistently.
///
/// It deliberately does **not** compute grid slots. <see cref="RaceGridManager"/> already
/// owns that math (2-wide staggered rows from a grid anchor); this component only supplies
/// the anchor and the racing line, so there is one implementation of "where does car N
/// start" rather than two.
/// </summary>
[DisallowMultipleComponent]
public class TrackPlacement : MonoBehaviour
{
    [Header("Track References")]
    [Tooltip("The baked racing line. Required: AI, sectors and the grid all derive from it.")]
    [SerializeField] private AIRacingLine _racingLine;

    [Tooltip("The drivable surface this content is built on. Used for track-limit checks.")]
    [SerializeField] private Collider _trackSurface;

    [Header("Canonical Placement")]
    [Tooltip("Where the qualifying car spawns: the start/finish line, facing down the track.")]
    [SerializeField] private Transform _startPose;

    [Tooltip("Origin and forward direction for the starting grid. RaceGridManager reads this.")]
    [SerializeField] private Transform _gridAnchor;

    [Tooltip("Index into the racing line that is the start/finish line.")]
    [SerializeField] private int _startFinishIndex;

    [Header("Derived (baked)")]
    [Tooltip("Lap length in metres, measured along the baked line.")]
    [SerializeField] private float _lapLengthMeters = 1f;

    public AIRacingLine RacingLine => _racingLine;
    public Collider TrackSurface => _trackSurface;
    public Transform StartPose => _startPose;
    public Transform GridAnchor => _gridAnchor;
    public int StartFinishIndex => _startFinishIndex;
    public float LapLengthMeters => _lapLengthMeters;

    /// <summary>
    /// True when the content can actually place a car. A track with no racing line or no
    /// start pose is not drivable, and callers should say so rather than spawning a car
    /// at the world origin.
    /// </summary>
    public bool IsValid => _racingLine != null && _racingLine.Count > 0 && _startPose != null;

    /// <summary>Start/finish as a position and rotation, without a Transform.</summary>
    public void GetStartPose(out Vector3 position, out Quaternion rotation)
    {
        if (_startPose != null)
        {
            position = _startPose.position;
            rotation = _startPose.rotation;
            return;
        }

        // Fall back to the racing line so a missing transform degrades rather than
        // returning the world origin, which would spawn the car inside the scenery.
        if (_racingLine != null && _racingLine.Count > 0)
        {
            int index = Mathf.Clamp(_startFinishIndex, 0, _racingLine.Count - 1);
            position = _racingLine.GetPosition(index);
            Vector3 forward = _racingLine.GetSegmentForward(index);
            rotation = forward.sqrMagnitude > 0.0001f
                ? Quaternion.LookRotation(forward.normalized, Vector3.up)
                : Quaternion.identity;
            return;
        }

        position = Vector3.zero;
        rotation = Quaternion.identity;
    }

    /// <summary>
    /// World position at a distance travelled from the start line. Clamped to the lap.
    /// This is the basis for sector timing in S3 and later, so it lives here rather than
    /// being recomputed by each consumer.
    /// </summary>
    public Vector3 GetPositionAtDistance(float distanceMeters, out int waypointIndex)
    {
        waypointIndex = 0;
        if (_racingLine == null || _racingLine.Count == 0) return Vector3.zero;

        float lap = Mathf.Max(1f, _lapLengthMeters);
        float wrapped = Mathf.Repeat(distanceMeters, lap);
        return _racingLine.GetPointAtDistance(_startFinishIndex, wrapped, out waypointIndex);
    }

    /// <summary>Normalised progress around the lap, 0..1, from the start line.</summary>
    public float GetProgress01(float distanceMeters)
    {
        float lap = Mathf.Max(1f, _lapLengthMeters);
        return Mathf.Repeat(distanceMeters, lap) / lap;
    }

#if UNITY_EDITOR
    /// <summary>Editor-time configuration used by the track content builder.</summary>
    public void Configure(AIRacingLine racingLine, Collider surface, Transform startPose,
        Transform gridAnchor, int startFinishIndex, float lapLengthMeters)
    {
        _racingLine = racingLine;
        _trackSurface = surface;
        _startPose = startPose;
        _gridAnchor = gridAnchor;
        _startFinishIndex = startFinishIndex;
        _lapLengthMeters = Mathf.Max(1f, lapLengthMeters);
    }
#endif
}
