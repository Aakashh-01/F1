using System;
using UnityEngine;

/// <summary>
/// Measures lap progress and lap time for a car following a track's racing line.
///
/// Attached to any car, not just the player's: the race needs per-car lap counts, and
/// qualifying needs the player's best lap. One component serves both, so there is a single
/// definition of "a lap" in the game.
///
/// Lap detection uses <i>unwrapped forward distance</i> along the line rather than
/// "did the car cross the start line". That distinction matters:
///
///   - Standing still on the start line must not complete a lap.
///   - Reversing over the line must not complete a lap.
///   - Drifting back and forth across the line must not spam completions.
///
/// Distance is measured as the car's <i>signed arc-length position</i> along the racing
/// line: the closest point on the nearest segment, expressed as a distance from the line's
/// origin, with the per-frame difference wrapped to the shorter way round.
///
/// Two earlier approaches were tried and rejected:
///
///   - Change in nearest-waypoint index. A car idling on a waypoint boundary oscillates
///     between two indices and accumulates a whole waypoint of phantom progress per frame.
///   - Real displacement projected onto the segment tangent. Less prone to phantom progress,
///     but the tangent is ambiguous at a waypoint, so the wrap segment was measured
///     backwards and every lap came up roughly one segment short — never quite completing.
///
/// Arc length has neither failure mode: standing still gives exactly zero, reversing gives
/// a negative step, and the wrap is handled arithmetically.
///
/// Sector timing and the GDD's purple/green/yellow sector colours are deliberately not here.
/// They are later work; this is the minimum that lets the flow's race-entry gate open.
/// </summary>
[DisallowMultipleComponent]
public class LapTracker : MonoBehaviour
{
    [Header("Track")]
    [Tooltip("The track content being followed. Resolved at Awake if left empty.")]
    [SerializeField] private TrackPlacement _track;

    [Header("Anti-cheat / robustness")]
    [Tooltip("Largest per-frame step accepted as real progress. Anything larger (a " +
             "teleport, a respawn, a scene load) is ignored rather than counted as a lap.")]
    [SerializeField] private float _maxStepMetres = 40f;

    [Tooltip("Ignore progress while the car is airborne or off the surface, so a big " +
             "jump over a crest does not fabricate distance.")]
    [SerializeField] private bool _requireGrounded = true;

    [Tooltip("Layer the track surface is on, used for the grounded check.")]
    [SerializeField] private int _trackLayer = 0;

    [Tooltip("Slack allowed when deciding a lap is complete, in metres. Accumulating " +
             "float steps over 90 waypoints leaves the total a fraction of a millimetre " +
             "short of the exact lap length, which alone was enough to stop a full " +
             "circuit ever registering. This is float slack, not a gameplay allowance: " +
             "half a metre is well under one car length.")]
    [SerializeField] private float _lapCompletionTolerance = 0.5f;

    // Arc-length table over the racing line, built once. _segmentStart[i] is the distance
    // from waypoint 0 to waypoint i; _totalArc is the full lap.
    private float[] _segmentStart;
    private float _totalArc;

    private int _lastIndex = -1;
    private Vector3 _lastPosition;
    private float _previousArc;
    private float _unwrappedDistance;
    private bool _running;
    private float _lapProgress01 = -1f; // -1 = not pushed to consumers since the last reset

    /// <summary>Raised with this tracker and the completed lap's time, in seconds.</summary>
    public event Action<LapTracker, float> OnLapCompleted;

    /// <summary>Laps finished since the tracker was reset. Starts at 0.</summary>
    public int CompletedLaps { get; private set; }

    /// <summary>Time since the current lap began, in seconds.</summary>
    public float CurrentLapTime { get; private set; }

    /// <summary>Most recent completed lap time, in seconds. 0 before the first lap.</summary>
    public float LastLapTime { get; private set; }

    /// <summary>Fastest completed lap, in seconds. 0 before the first lap.</summary>
    public float BestLapTime { get; private set; }

    /// <summary>True on the frame a lap completes, so a caller can react once.</summary>
    public bool LapCompletedThisFrame { get; private set; }

    /// <summary>Distance travelled around the current lap, 0..lap length.</summary>
    public float LapDistance => WrapLapDistance();

    /// <summary>Total distance travelled since the last reset, in metres.</summary>
    public float TotalDistance => _unwrappedDistance;

    /// <summary>Progress around the lap, 0..1. The basis for sector timing later.</summary>
    public float LapProgress01
    {
        get
        {
            float lap = _track == null ? 0f : Mathf.Max(1f, _track.LapLengthMeters);
            return lap > 0f ? WrapLapDistance() / lap : 0f;
        }
    }

    /// <summary>
    /// Called by whoever presents this tracker (the HUD binder) once it has consumed
    /// <see cref="LapProgress01"/>. Records the value that was read, so
    /// <see cref="LapProgressChanged"/> can tell "has not moved since" from "moved".
    ///
    /// A consumer that redraws a progress bar needs to know whether the number actually
    /// changed. A car that has not moved has an identical value, and repainting it every
    /// frame is pure waste — but a tracker with no consumer must not swallow its own value
    /// either, hence an explicit acknowledgement rather than the getter self-marking.
    /// </summary>
    public void MarkLapProgressConsumed() => _lapProgress01 = LapProgress01;

    /// <summary>
    /// True when <see cref="LapProgress01"/> has moved since the last
    /// <see cref="MarkLapProgressConsumed"/>. Before anything has been read — and before a
    /// track is resolved, or before the first sample — it is always true, so an initial value
    /// is never suppressed. <see cref="ResetTracker"/> also drops the record, so the first
    /// read after a reset counts as changed whatever the car happens to be doing.
    /// </summary>
    public bool LapProgressChanged =>
        _track == null || !_track.IsValid || _lapProgress01 < 0f ||
        !Mathf.Approximately(_lapProgress01, LapProgress01);

    /// <summary>
    /// Total distance folded into the current lap, with the same slack the lap counter
    /// uses. Without it, accumulated float error leaves the odometer a fraction of a
    /// millimetre under a whole lap, so a car sitting exactly on the line reports ~100%
    /// progress instead of ~0 - the progress bar would jump backwards at the line.
    /// </summary>
    private float WrapLapDistance()
    {
        if (_track == null) return 0f;

        float lap = Mathf.Max(1f, _track.LapLengthMeters);
        float distance = Mathf.Repeat(_unwrappedDistance, lap);
        if (distance >= lap - _lapCompletionTolerance)
            distance = 0f;
        return distance;
    }

    public TrackPlacement Track => _track;

    /// <summary>
    /// Binds this tracker to a specific track and clears its state.
    ///
    /// The flow spawns cars explicitly rather than letting the tracker go looking, so the
    /// car that gets timed is the car that was placed on this track. Left to
    /// <c>FindAnyObjectByType</c>, a tracker that wakes before the track content finishes
    /// loading has no track at all and silently measures nothing.
    /// </summary>
    public void BindTrack(TrackPlacement track)
    {
        _track = track;
        ResetTracker();
    }

    private void Awake()
    {
        if (_track == null)
            _track = FindAnyObjectByType<TrackPlacement>();
    }

    private void Update()
    {
        Tick(Time.deltaTime);
    }

    /// <summary>
    /// Advances the tracker. Public so tests can drive it deterministically without
    /// relying on frame timing.
    /// </summary>
    public void Tick(float deltaTime)
    {
        LapCompletedThisFrame = false;

        if (_track == null || !_track.IsValid)
            return;

        if (!_running)
        {
            // First valid sample establishes the starting point. The car is placed on the
            // grid some way behind the line, so it must not "complete" a lap by being
            // snapped onto the line at reset.
            InitialiseFromCurrentPosition();
            return;
        }

        if (deltaTime <= 0f)
            return;

        if (_requireGrounded && !IsOnTrackSurface())
        {
            // Keep the last index so the gap is not counted as one enormous step when the
            // car lands again.
            return;
        }

        var line = _track.RacingLine;
        if (_segmentStart == null || _segmentStart.Length != line.Count)
            BuildArcTable(line);

        int index = line.FindNearestIndex(transform.position);
        if (index < 0)
            return;

        if (_lastIndex < 0)
        {
            _lastIndex = index;
            _lastPosition = transform.position;
            _previousArc = ArcAt(line, index);
            return;
        }

        Vector3 movement = transform.position - _lastPosition;
        _lastPosition = transform.position;
        _lastIndex = index;

        // Teleport / respawn guard: measured on the raw displacement so it fires no matter
        // which direction the car jumped. Counting a respawn as progress would fabricate
        // laps out of nothing.
        if (movement.magnitude > _maxStepMetres)
            return;

        // Signed progress along the line, wrapped to the shorter way round so the seam
        // between the last and first waypoint is not a discontinuity.
        float arc = ArcAt(line, index);
        float step = arc - _previousArc;
        if (step > _totalArc * 0.5f) step -= _totalArc;
        else if (step < -_totalArc * 0.5f) step += _totalArc;
        _previousArc = arc;

        // Strictly monotonic total distance since the last reset. It is deliberately not
        // decremented on a completed lap: CompletedLaps is a cumulative count, so the two
        // have to stay on the same scale. LapDistance derives from this with Mathf.Repeat.
        _unwrappedDistance += step;
        if (_unwrappedDistance < 0f)
            _unwrappedDistance = 0f;

        CurrentLapTime += deltaTime;

        int lapsDone = Mathf.FloorToInt(
            (_unwrappedDistance + _lapCompletionTolerance) / _track.LapLengthMeters);
        if (lapsDone > CompletedLaps)
        {
            // A single frame cannot realistically cross the line twice at racing speed, so
            // this is normally one completion; the loop keeps the count correct if it ever
            // does (at the cost of attributing the same lap time to both).
            for (int i = CompletedLaps; i < lapsDone; i++)
                CompleteLap();

            CompletedLaps = lapsDone;
        }
    }

    private void BuildArcTable(AIRacingLine line)
    {
        int count = Mathf.Max(1, line.Count);
        _segmentStart = new float[count];
        for (int i = 1; i < count; i++)
            _segmentStart[i] = _segmentStart[i - 1] +
                Vector3.Distance(line.GetPosition(i - 1), line.GetPosition(i));

        _totalArc = count > 1
            ? _segmentStart[count - 1] + Vector3.Distance(
                line.GetPosition(count - 1), line.GetPosition(0))
            : 1f;

        if (_totalArc < 0.001f) _totalArc = 1f;
    }

    /// <summary>
    /// Distance along the line from waypoint 0 to the point on the line closest to the
    /// car's position.
    /// </summary>
    private float ArcAt(AIRacingLine line, int index)
    {
        if (_segmentStart == null || index < 0 || index >= _segmentStart.Length)
            return 0f;

        Vector3 from = line.GetPosition(index);
        Vector3 to = line.GetPosition(index + 1);
        Vector3 segment = to - from;
        float lengthSq = segment.sqrMagnitude;
        if (lengthSq <= 0.001f)
            return _segmentStart[index];

        float t = Mathf.Clamp01(Vector3.Dot(transform.position - from, segment) / lengthSq);
        return _segmentStart[index] + t * Mathf.Sqrt(lengthSq);
    }

    private void InitialiseFromCurrentPosition()
    {
        var line = _track.RacingLine;
        if (_segmentStart == null || _segmentStart.Length != line.Count)
            BuildArcTable(line);

        _lastIndex = line.FindNearestIndex(transform.position);
        _lastPosition = transform.position;
        _previousArc = ArcAt(line, _lastIndex);
        _unwrappedDistance = 0f;
        _running = true;
        CurrentLapTime = 0f;
    }

    private void CompleteLap()
    {
        float lapTime = CurrentLapTime;
        LastLapTime = lapTime;
        if (BestLapTime <= 0f || lapTime < BestLapTime)
            BestLapTime = lapTime;

        // Only the clock resets here. The accumulated distance stays monotonic so it
        // remains on the same scale as CompletedLaps.
        CurrentLapTime = 0f;

        LapCompletedThisFrame = true;
        OnLapCompleted?.Invoke(this, lapTime);
    }

    private bool IsOnTrackSurface()
    {
        var surface = _track.TrackSurface;
        if (surface == null) return true;

        var origin = transform.position + Vector3.up * 50f;
        return Physics.Raycast(origin, Vector3.down, 60f, 1 << _trackLayer,
            QueryTriggerInteraction.Ignore);
    }

    /// <summary>Clears all lap state. Call when a car is placed on the grid.</summary>
    public void ResetTracker()
    {
        CompletedLaps = 0;
        CurrentLapTime = 0f;
        LastLapTime = 0f;
        BestLapTime = 0f;
        LapCompletedThisFrame = false;
        _unwrappedDistance = 0f;
        _lastIndex = -1;
        _lastPosition = Vector3.zero;
        _previousArc = 0f;
        _running = false;
        _lapProgress01 = -1f;
    }
}
