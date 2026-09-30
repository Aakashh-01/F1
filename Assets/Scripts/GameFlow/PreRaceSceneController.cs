using System.Collections;
using UnityEngine;
using F1.GameData;

namespace F1.GameFlow
{
    /// <summary>
    /// Owns the behaviour of the pre-race qualifying shell: bring up the track, put the
    /// player's car on the start line, and wire the qualifying HUD to the session.
    ///
    /// The shell is a *shell*. It owns no track geometry and no timing logic — the track
    /// scene stays loaded underneath it and owns the racing line, start pose and grid anchor
    /// (invariant 18), while the car's <c>LapTracker</c> owns the measurement. This scene
    /// only decides when those things are needed and connects them to the screen.
    ///
    /// Track content is loaded *here*, not at track selection (invariant 14). The player picks
    /// a track in the hub without paying for its geometry; the geometry arrives when there is
    /// somewhere to drive it.
    ///
    /// The qualifying HUD is hosted by this scene via its <c>FlowSceneHost</c>, so the screen
    /// is found in this scene rather than by a global search. The flow already resolved the
    /// same instance; this is the scene's own half of the wiring, and finding it through the
    /// host is what keeps the two from drifting onto different objects.
    /// </summary>
    [DisallowMultipleComponent]
    public class PreRaceSceneController : MonoBehaviour, ISessionStartGate
    {
        [Header("Qualifying Mode Objects (optional)")]
        [Tooltip("Ghost car prefab. Intentionally unassigned: the ghost car is deferred " +
                 "until after the slice. The field exists so the deferred work has a home " +
                 "rather than inventing a second loading path when it lands.")]
        [SerializeField] private GameObject _ghostCarPrefab;

        private GameFlowManager _flow;
        private QualifyingScreen _screen;
        private PlayerCarSpawner _spawner;
        private TrackPlacement _track;
        private GameObject _activeGhost;
        private bool _sessionReady;

        /// <summary>
        /// How long the shell will wait for the routing service to go idle before giving up
        /// on loading the track content. Generous, because a real transition on a slow
        /// machine is not a failure — the point is only that "still busy" must eventually
        /// become an error rather than a wait that never ends.
        /// </summary>
        private const float ContentLoadIdleTimeoutSeconds = 30f;

        private void Start() => StartCoroutine(BringUpSession());

        /// <summary>
        /// Loads the track if it is not already resident, then spawns the car.
        ///
        /// Ordered deliberately: the car cannot be placed at a start pose that does not exist
        /// yet, and the timing stack cannot bind to a track that is still loading. Waiting for
        /// the content load to complete is what makes the spawn a single step rather than a
        /// chain of "is it ready yet" checks.
        /// </summary>
        private IEnumerator BringUpSession()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError("[PreRaceSceneController] No GameFlowManager found.", this);
                enabled = false;
                yield break;
            }

            _screen = GetComponentInChildren<QualifyingScreen>(true);
            if (_screen == null)
            {
                Debug.LogError(
                    "[PreRaceSceneController] No QualifyingScreen in this scene. The " +
                    "FlowSceneHost is probably missing its screen prefab — the HUD has " +
                    "nowhere to draw the lap clock.", this);
                enabled = false;
                yield break;
            }

            _spawner = GetComponentInChildren<PlayerCarSpawner>(true);
            if (_spawner == null)
            {
                Debug.LogError(
                    "[PreRaceSceneController] No PlayerCarSpawner in this scene, so no car " +
                    "can be placed on the track.", this);
                enabled = false;
                yield break;
            }

            // The camera is the car's own: spawning a car brings the camera and its
            // behaviour with it, so any scene that places a car gets a view without having
            // to know cameras exist. Nothing to wire here by design.

            _screen.OnBackPressed += OnBackPressed;
            _screen.OnRacePressed += OnRacePressed;
            _screen.OnRestartPressed += OnRestartRequested;
            _flow.OnQualifyingRetryRequested += OnQualifyingRetryRequested;
            _flow.OnQualifyingTimeSet += OnQualifyingTimeSet;

            // One frame first: the host instantiated the screen in Awake and the flow
            // resolves it as soon as the load completes. Letting the flow finish claiming the
            // scene before anything is configured keeps one owner at a time.
            yield return null;

            if (!_flow.IsTrackContentLoaded)
            {
                // The transition that brought this scene in is still finishing its own
                // scene work when Start runs, and the routing service refuses a second load
                // while one is in flight — so wait for it to go idle before asking. Without
                // this the content request is rejected as Busy and the session never starts.
                //
                // Bounded, because the alternative is worse than a slow start. If the
                // service is left permanently busy — a load that was refused, a scene
                // operation that never completed — this used to spin here forever, and the
                // shell never finished coming up. Nothing reports that: the car simply
                // never appears, and the session looks like it is still loading indefinitely
                // rather than like a failure.
                float idleDeadline = Time.realtimeSinceStartup + ContentLoadIdleTimeoutSeconds;
                while (_flow.SceneFlow != null && _flow.SceneFlow.IsBusy
                       && Time.realtimeSinceStartup < idleDeadline)
                {
                    yield return null;
                }

                if (_flow.SceneFlow != null && _flow.SceneFlow.IsBusy)
                {
                    Debug.LogError(
                        "[PreRaceSceneController] The scene loader is still busy after " +
                        $"{ContentLoadIdleTimeoutSeconds:F0}s, so the track content request " +
                        "would only be refused as Busy. The qualifying session cannot start.",
                        this);
                    yield break;
                }

                bool loadFailed = true;
                yield return _flow.LoadTrackContent(result =>
                {
                    if (result.Success) loadFailed = false;
                    else Debug.LogError($"[PreRaceSceneController] Track content load failed — {result}");
                });

                if (loadFailed || !_flow.IsTrackContentLoaded)
                {
                    Debug.LogError(
                        "[PreRaceSceneController] Could not load the track content, so there " +
                        "is nowhere to qualify. The session cannot start.", this);
                    yield break;
                }
            }

            _track = FindAnyObjectByType<TrackPlacement>();
            if (_track == null || !_track.IsValid)
            {
                Debug.LogError(
                    "[PreRaceSceneController] The track scene loaded but provides no valid " +
                    "TrackPlacement, so there is no start pose or racing line to use.", this);
                yield break;
            }

            _screen.SetTrackInfo(_flow.SelectedTrack);

            if (!_spawner.Spawn(_flow, _track))
            {
                Debug.LogError(
                    "[PreRaceSceneController] The player car could not be spawned, so the " +
                    "qualifying session cannot run.", this);
                yield break;
            }

            SpawnGhostIfRequested();
            _sessionReady = true;

            // The first attempt is driving, not a menu. The results panel is reached from
            // the lap that comes back from the flow, not from arriving here.
            _screen.ShowDrivingState();
            // Held here, not released, until the countdown finishes — see
            // BeginStartCountdown. The overlay finds this scene's gate through
            // ISessionStartGate and drives it.
            SetCarLocked(true);

            Debug.Log(
                $"[PreRaceSceneController] Qualifying ready on '{_track.gameObject.scene.name}' " +
                $"(lap {_track.LapLengthMeters:0}m). Race entry is " +
                (_flow.CanStartRace ? "open" : "blocked until a lap is completed") + ".");
        }

        // --- ISessionStartGate ---

        public bool IsSessionReady => _sessionReady;

        /// <summary>
        /// Holds or releases the car at the line, and stops the lap clock while held.
        ///
        /// The overlay plays the countdown and calls this to release on zero; it never owns
        /// the timing itself. The clock belongs here rather than there because only the
        /// controller knows when the car is genuinely on the grid — the countdown can finish
        /// before the track has streamed in, and a clock that started on scene load charged
        /// the driver for the loading time.
        /// </summary>
        public void SetStartGateLocked(bool locked)
        {
            SetCarLocked(locked);

            if (_spawner == null || _spawner.PlayerCar == null) return;
            var tracker = _spawner.PlayerCar.GetComponent<LapTracker>();
            if (tracker == null) return;

            if (locked) tracker.Suspend();
            else tracker.Resume();
        }

        /// <summary>
        /// The ghost is deferred, so this only runs if a prefab has been assigned. The Phase 1
        /// session contracts already decide whether a ghost should exist for this attempt
        /// (invariant 6: the first attempt is player-only), and this reads that decision
        /// rather than reimplementing it.
        /// </summary>
        private void SpawnGhostIfRequested()
        {
            if (_ghostCarPrefab == null) return;
            if (!_flow.ShouldSpawnGhostForCurrentAttempt) return;

            _track.GetStartPose(out var position, out var rotation);
            _activeGhost = Instantiate(_ghostCarPrefab, position, rotation);
            _activeGhost.name = "GhostCar";
        }

        private void OnQualifyingRetryRequested()
        {
            if (_flow == null || _flow.CurrentSessionType != SessionType.Qualifying)
                return;

            if (_activeGhost != null)
            {
                Destroy(_activeGhost);
                _activeGhost = null;
            }

            // Same car, same track, same wing, back on the canonical start pose with its lap
            // state cleared. Respawning instead would re-run physics setup for no benefit.
            if (_sessionReady)
                _spawner.ResetToStartPose(_track);

            SpawnGhostIfRequested();

            // A retry puts the player back out on track, so the options that were offered
            // for the lap just recorded are no longer what the screen is waiting on — and
            // the car has to be released, or Retry would park it on the line every time.
            _screen.ShowDrivingState();
            SetCarLocked(false);
        }

        /// <summary>
        /// Parks or releases the car.
        ///
        /// The results panel is a question, and a car still accelerating under it is not
        /// answering one. Releasing it again is Retry's job, because the two states have to
        /// agree: a panel offering Retry over a car that cannot be driven is a dead end.
        /// </summary>
        private void SetCarLocked(bool locked)
        {
            if (_spawner == null || _spawner.PlayerCar == null) return;

            var coordinator = _spawner.PlayerCar.GetComponent<VehiclePhysicsCoordinator>();
            if (coordinator != null)
                coordinator.InputLocked = locked;
        }

        /// <summary>
        /// A lap was accepted by the flow, so the session has something to offer.
        ///
        /// This is the screen's only route into the results state. Listening to the flow
        /// rather than polling <c>CanStartRace</c> each frame means the panel appears on the
        /// lap that set the record, and only if the session accepted that lap — a lap the
        /// session rejected never reaches this handler.
        /// </summary>
        private void OnQualifyingTimeSet(float bestLapTime)
        {
            if (_flow == null || _flow.CurrentSessionType != SessionType.Qualifying)
                return;

            _screen.ShowResultsState(bestLapTime);
            SetCarLocked(true);
        }

        private void OnBackPressed() => _flow.GoToWingSetup();

        /// <summary>
        /// The HUD's restart button. The retry itself is a flow decision — the session layer
        /// owns attempt counting and the ghost rules — and the flow raises
        /// <see cref="GameFlowManager.OnQualifyingRetryRequested"/>, which is where this
        /// component puts the car back on the line. So the button asks, and never acts.
        /// </summary>
        private void OnRestartRequested() => _flow.RetryQualifyingSession();

        private void OnRacePressed()
        {
            // The gate itself lives in the flow layer (invariant 5). The HUD's button is only
            // a request; the screen is not allowed to decide whether the player may race.
            if (!_flow.CanStartRace)
            {
                Debug.LogWarning(
                    "[PreRaceSceneController] Race requested without a qualifying record — " +
                    "the flow is refusing to enter the race.");
                return;
            }

            _flow.StartRaceSession();
        }

        private void OnDestroy()
        {
            if (_screen != null)
            {
                _screen.OnBackPressed -= OnBackPressed;
                _screen.OnRacePressed -= OnRacePressed;
                _screen.OnRestartPressed -= OnRestartRequested;
            }

            if (_flow != null)
            {
                _flow.OnQualifyingRetryRequested -= OnQualifyingRetryRequested;
                _flow.OnQualifyingTimeSet -= OnQualifyingTimeSet;
            }

            if (_activeGhost != null)
                Destroy(_activeGhost);
        }
    }
}
