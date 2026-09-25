using UnityEngine;

namespace F1.GameFlow
{
    /// <summary>
    /// Reports completed laps from a <see cref="LapTracker"/> into the flow's qualifying
    /// record, which is what actually opens the race-entry gate.
    ///
    /// This is the join between "the car drove a lap" and "the flow may now let the player
    /// enter the race". Keeping it a separate component means:
    ///
    ///   - <see cref="LapTracker"/> stays pure measurement and knows nothing about the flow,
    ///     so the race can reuse it for AI cars without every AI reporting qualifying times.
    ///   - The rules about which laps count (rental cars, best lap, ghost data) stay in
    ///     <see cref="GameFlowManager.SetQualifyingTime"/>, which already enforces them and
    ///     is covered by the Phase 1 tests. This component deliberately does not reimplement
    ///     any of that.
    ///
    /// Ghost data is not supplied. The ghost car is deferred until after the slice, so the
    /// qualifying record is created without ghost data and the runtime behaves exactly as
    /// the Phase 1 contracts specify for a session with no ghost.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(LapTracker))]
    public class QualifyingLapReporter : MonoBehaviour
    {
        [Tooltip("Only report while the session is a qualifying session. The race uses " +
                 "LapTracker for per-car lap counts and must not overwrite the qualifying record.")]
        [SerializeField] private bool _qualifyingOnly = true;

        [Tooltip("Minimum plausible lap time. Protects the ranking from a tracker that " +
                 "reports a lap within one frame of starting, which would otherwise become " +
                 "the player's 'best' forever.")]
        [SerializeField] private float _minimumPlausibleLapSeconds = 5f;

        private LapTracker _tracker;
        private GameFlowManager _flow;

        private void Awake()
        {
            _tracker = GetComponent<LapTracker>();
        }

        private void OnEnable()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError(
                    "[QualifyingLapReporter] No GameFlowManager. Laps cannot be reported.", this);
                enabled = false;
                return;
            }

            _tracker.OnLapCompleted += OnLapCompleted;
        }

        private void OnDisable()
        {
            if (_tracker != null)
                _tracker.OnLapCompleted -= OnLapCompleted;
        }

        private void OnLapCompleted(LapTracker tracker, float lapTime)
        {
            if (_qualifyingOnly && _flow.CurrentSessionType != SessionType.Qualifying)
                return;

            if (lapTime < _minimumPlausibleLapSeconds)
            {
                Debug.LogWarning(
                    $"[QualifyingLapReporter] Rejected an implausible {lapTime:0.00}s lap.");
                return;
            }

            // No ghost data: the ghost car is deferred until after the slice. The Phase 1
            // contracts already handle a qualifying record with no ghost data, and the
            // player's own laps remain the basis for entering the race.
            _flow.SetQualifyingTime(lapTime, null);
        }
    }
}
