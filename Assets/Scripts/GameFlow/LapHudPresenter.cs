using UnityEngine;

namespace F1.GameFlow
{
    /// <summary>
    /// Drives the on-screen lap clock from a <see cref="LapTracker"/>.
    ///
    /// This is the last piece of slice step S3: the tracker measures, the reporter hands a
    /// completed lap to the flow, and this pushes the live values into the HUD. The three
    /// are separate on purpose — the race reuses the tracker for AI cars that must not
    /// report qualifying times, and the HUD contract belongs to the session that is
    /// actually on screen, not to the measurement.
    ///
    /// It lives on the car rather than in <c>TrackModeManager</c> because the car is the
    /// thing being timed and the car is what the flow spawns. The track scene does not own
    /// a specific car: one track content scene serves every car, every session and, later,
    /// every AI opponent. A binder on the car therefore needs no reference to find its
    /// tracker and follows the car wherever it is placed on the grid.
    ///
    /// The screen is reached through the flow's host-resolved handle, never through a
    /// <c>FindAnyObjectByType</c> search. That is the decision Phase 3 made for every other
    /// consumer: the search skips inactive objects, and a HUD is normally inactive until its
    /// session starts. The handle legitimately changes as the flow transitions, so it is
    /// re-read every push rather than cached.
    ///
    /// What it deliberately does not do: push sector times. The GDD's three-sector timing and
    /// purple/green/yellow colours are deferred, and the tracker's progress value is the
    /// raw material they will be built from. The lap <i>counter</i> is also not pushed — the
    /// qualifying screen has no such field, and the race's lap count belongs to S5, which
    /// owns the race distance and finish detection.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(LapTracker))]
    public class LapHudPresenter : MonoBehaviour
    {
        [Tooltip("Push lap times to the qualifying HUD. Turn off on a car that should be " +
                 "timed silently, such as a future ghost, which is neither this player's " +
                 "lap nor part of the race field's own timing.")]
        [SerializeField] private bool _qualifying = true;

        [Tooltip("Push lap times to the race HUD. Off by default because the race's own HUD " +
                 "wiring arrives with S5, which also owns the lap counter.")]
        [SerializeField] private bool _race = false;

        /// <summary>
        /// Turns the race lap-clock push on for a car that is running in a race.
        ///
        /// The flag is a serialized field rather than a constant because the same component
        /// and the same car prefab serve qualifying and racing, and only the session knows
        /// which one is on screen. A qualifying car must not push to the race HUD and a race
        /// car should, so the shell that brings the session up decides. Kept public and
        /// explicit rather than inferred from <c>CurrentSessionType</c> at push time, because
        /// inferring it there would mean a ghost or a replay car quietly driving a HUD it
        /// has no business touching.
        /// </summary>
        public void SetRacePush(bool enabled) => _race = enabled;

        [Tooltip("Push this tracker's lap progress (0..1) to any screen that declares it. " +
                 "Nothing consumes it yet; it is captured here so sector timing later has a " +
                 "measured value to work from rather than a second tracking system.")]
        [SerializeField] private bool _pushProgress = true;

        private LapTracker _tracker;
        private GameFlowManager _flow;

        /// <summary>The tracker driving this presenter. Exposed for tests and callers.</summary>
        public LapTracker Tracker => _tracker;

        private void Awake()
        {
            _tracker = GetComponent<LapTracker>();
        }

        private void OnEnable()
        {
            _flow = GameFlowManager.Instance;
        }

        private void Update()
        {
            if (_tracker == null || _flow == null)
                return;

            PushLapTime();
            PushProgress();
        }

        private void PushLapTime()
        {
            float current = _tracker.CurrentLapTime;
            float best = _tracker.BestLapTime;

            if (_qualifying && _flow.CurrentSessionType == SessionType.Qualifying)
            {
                var screen = _flow.QualifyingScreenInstance;
                if (!IsAlive(screen))
                {
                    WarnOnceAboutMissingHud();
                    return;
                }

                screen.UpdateLapTime(current, best);
                return;
            }

            if (_race && _flow.CurrentSessionType == SessionType.Race)
            {
                var screen = _flow.RaceScreenInstance;
                if (IsAlive(screen))
                    screen.UpdateLapTime(current, best);
            }
        }

        /// <summary>
        /// Whether a resolved screen is still usable.
        ///
        /// The screens are held behind their interfaces, and that matters more than it looks:
        /// Unity's "a destroyed object compares equal to null" behaviour is an operator on
        /// <see cref="Object"/>, so a reference reached through an <c>interface</c> is compared
        /// by plain reference equality and a torn-down HUD looks perfectly alive. Writing
        /// <c>screen == null</c> here would therefore happily call a destroyed component and
        /// throw a MissingReferenceException the frame a flow scene unloads. Re-checking
        /// through the <see cref="Object"/> static type restores the destroyed-object test.
        /// </summary>
        private static bool IsAlive(object screen)
        {
            switch (screen)
            {
                case null:
                    return false;
                case Object unityObject:
                    return unityObject != null;
                default:
                    return true;
            }
        }

        private void PushProgress()
        {
            if (!_pushProgress || !_tracker.LapProgressChanged)
                return;

            // Captured and immediately acknowledged. The lap-time push above is deliberately
            // unconditional — a HUD clock has to be driven every frame, since "no progress"
            // still means the clock is running — but progress only needs a redraw when it
            // actually moves, so acknowledging here is what stops an idle car repainting a
            // progress bar sixty times a second to display the same number.
            float progress = _tracker.LapProgress01;
            _tracker.MarkLapProgressConsumed();

            // No consumer yet. When sector timing lands, this is the value it consumes;
            // until then the read is intentional and observable, not dead work.
            _lastPushedProgress = progress;
            ProgressPushCount++;
        }

        /// <summary>Progress most recently read by this presenter, for tests and diagnostics.</summary>
        public float LastPushedProgress => _lastPushedProgress;
        private float _lastPushedProgress;

        /// <summary>
        /// How many times progress has actually been pushed. Equal to the number of frames
        /// the car's progress genuinely changed, which is what makes the "skips unchanged
        /// frames" behaviour observable rather than merely claimed.
        /// </summary>
        public int ProgressPushCount { get; private set; }

        private bool _warnedMissingHud;

        private void WarnOnceAboutMissingHud()
        {
            if (_warnedMissingHud)
                return;

            _warnedMissingHud = true;
            Debug.LogWarning(
                "[LapHudPresenter] Qualifying session with no QualifyingScreen resolved. " +
                "The lap clock is being measured but has nowhere to display. The qualifying " +
                "scene needs a FlowSceneHost with a QualifyingScreen prefab assigned — " +
                "that prefab does not exist yet, so this is expected until the pre-race " +
                "shell (S4) builds it.", this);
        }
    }
}

