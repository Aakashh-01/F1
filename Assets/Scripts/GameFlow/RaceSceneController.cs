using System.Collections;
using UnityEngine;
using F1.GameData;

namespace F1.GameFlow
{
    /// <summary>
    /// Owns the behaviour of the race shell: bring up the track, put the player's car on the
    /// grid slot its qualifying result earned, and place the AI field around it.
    ///
    /// A shell, in the same sense <see cref="PreRaceSceneController"/> is one. It owns no
    /// track geometry and no timing logic. The track scene stays loaded underneath it and
    /// keeps owning the racing line, the start pose and — importantly — the grid anchor
    /// (invariant 18). The car's <c>LapTracker</c> keeps owning measurement.
    ///
    /// What this component does own is the *session*: which field is racing, which slot the
    /// player starts from, and how many laps the race is. Those are decisions about a race,
    /// not about a circuit, so they belong in the race's own scene rather than in the shared
    /// track content every session loads.
    ///
    /// It replaces <c>TrackModeManager</c>, which held the race half of that job while the
    /// race still routed to the track scene. That component spawned its AI opponents at the
    /// world origin and had no grid at all; the field is now placed by the grid manager that
    /// was already on the track, which is what makes the grid a grid.
    /// </summary>
    [DisallowMultipleComponent]
    public class RaceSceneController : MonoBehaviour, ISessionStartGate
    {
        [Header("Race Configuration")]
        [Tooltip("Lap count for the race. Leave at -1 to use the selected track definition's " +
                 "standard race laps. Set it to a small number for a demo so a race can be " +
                 "driven to its finish in a sitting.")]
        [SerializeField] private int _raceLapsOverride = -1;

        [Tooltip("Optional. Overrides the AI field configured on the track's grid manager. " +
                 "Leave empty to race whatever field the track already has.")]
        [SerializeField] private AIFieldDefinition _aiFieldOverride;

        [Tooltip("Start the player at the back of the grid regardless of the qualifying " +
                 "result. For a client demo, where the whole AI field ahead of the player " +
                 "reads better than a pole position nobody earned.")]
        [SerializeField] private bool _startPlayerAtBackOfGrid;

        /// <summary>
        /// Whether this race ignores the qualifying result and starts the player at the back.
        ///
        /// Read-only on purpose: the override is a property of how this scene is authored, not
        /// a knob to be turned mid-session. It is exposed because where the player is
        /// actually starting is a fact other code has to be able to check — a HUD that
        /// reports the qualifying position while the car sits at the back is a HUD that is
        /// wrong, and the only way to catch that is to let something read what was decided.
        /// </summary>
        public bool StartsPlayerAtBackOfGrid => _startPlayerAtBackOfGrid;

        private GameFlowManager _flow;
        private PlayerCarSpawner _spawner;
        private TrackPlacement _track;
        private RaceGridManager _grid;
        private bool _sessionReady;

        /// <summary>Laps this race is run over, once the session is up.</summary>
        public int TotalLaps { get; private set; }

        /// <summary>True once the track, the car and the grid are all in place.</summary>
        public bool IsSessionReady => _sessionReady;

        private void Start() => StartCoroutine(BringUpRace());

        /// <summary>
        /// Brings the race up in the order the pieces depend on each other.
        ///
        /// Track content first, because the grid anchor and the start pose live in it and
        /// neither the car nor the field can be placed without them. The car next, because the
        /// grid manager is told which car is the player's. The grid last, because it places
        /// everyone at once and must not run while the track is still loading.
        /// </summary>
        private IEnumerator BringUpRace()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError("[RaceSceneController] No GameFlowManager found.", this);
                enabled = false;
                yield break;
            }

            _spawner = GetComponentInChildren<PlayerCarSpawner>(true);
            if (_spawner == null)
            {
                Debug.LogError(
                    "[RaceSceneController] No PlayerCarSpawner in this scene, so no car can " +
                    "be placed on the grid.", this);
                enabled = false;
                yield break;
            }

            // One frame first, for the same reason the pre-race shell does it: the host
            // instantiates the screen in Awake and the flow claims the scene as soon as the
            // load completes, and one owner at a time keeps the two from crossing.
            yield return null;

            if (!_flow.IsTrackContentLoaded)
            {
                // The transition that brought this scene in is still finishing its own scene
                // work when Start runs, and the routing service refuses a second load while
                // one is in flight — so wait for it to go idle before asking.
                while (_flow.SceneFlow != null && _flow.SceneFlow.IsBusy)
                    yield return null;

                bool loadFailed = true;
                yield return _flow.LoadTrackContent(result =>
                {
                    if (result.Success) loadFailed = false;
                    else Debug.LogError($"[RaceSceneController] Track content load failed — {result}");
                });

                if (loadFailed || !_flow.IsTrackContentLoaded)
                {
                    Debug.LogError(
                        "[RaceSceneController] Could not load the track content, so there is " +
                        "nowhere to race. The session cannot start.", this);
                    yield break;
                }
            }

            _track = FindAnyObjectByType<TrackPlacement>();
            if (_track == null || !_track.IsValid)
            {
                Debug.LogError(
                    "[RaceSceneController] The track scene loaded but provides no valid " +
                    "TrackPlacement, so there is no start pose or racing line to use.", this);
                yield break;
            }

            // The grid manager is the track content's, not this shell's: it holds the grid
            // anchor, which is geometry, and geometry belongs to the track (invariant 18).
            // This shell drives it; it does not duplicate it.
            _grid = FindAnyObjectByType<RaceGridManager>();
            if (_grid == null)
            {
                Debug.LogError(
                    "[RaceSceneController] The track content provides no RaceGridManager, so " +
                    "the field cannot be placed. The race would be a car on an empty track.", this);
                yield break;
            }

            if (!_spawner.Spawn(_flow, _track))
            {
                Debug.LogError(
                    "[RaceSceneController] The player car could not be spawned, so the race " +
                    "cannot start.", this);
                yield break;
            }

            int gridPosition = PlaceOnGrid();

            _sessionReady = true;
            PublishRaceInfo();

            // Held at the line until the countdown releases it. The overlay finds this
            // scene's gate through ISessionStartGate and drives the clock.
            SetStartGateLocked(true);

            Debug.Log(
                $"[RaceSceneController] Race ready on '{_track.gameObject.scene.name}' " +
                $"(lap {_track.LapLengthMeters:0}m, {TotalLaps} laps). Player starts P" +
                $"{gridPosition} of {_grid.aiFieldCount + 1}, field of " +
                $"{_grid.aiFieldCount}.");
        }

        // --- ISessionStartGate ---
        // IsSessionReady already existed on this class and satisfies the interface.

        /// <summary>
        /// Holds or releases the whole field at the line, and stops the lap clock while held.
        ///
        /// The AI cars are included deliberately. The grid puts the player's car on the line
        /// via the same path, so a countdown that held only the player would release into a
        /// field that had already been driving for three seconds — which is most of the
        /// reason the grid looked unsettled on the line.
        /// </summary>
        public void SetStartGateLocked(bool locked)
        {
            if (_spawner != null && _spawner.PlayerCar != null)
            {
                var coordinator = _spawner.PlayerCar.GetComponent<VehiclePhysicsCoordinator>();
                if (coordinator != null) coordinator.InputLocked = locked;

                var tracker = _spawner.PlayerCar.GetComponent<LapTracker>();
                if (tracker != null)
                {
                    if (locked) tracker.Suspend();
                    else tracker.Resume();
                }
            }

            if (_grid == null) return;
            foreach (var car in _grid.SpawnedCars)
            {
                if (car == null) continue;
                var ai = car.GetComponent<VehiclePhysicsCoordinator>();
                if (ai != null) ai.InputLocked = locked;
            }
        }

        /// <summary>
        /// Works out the player's slot, hands the grid everything it needs, and places the
        /// field.
        ///
        /// Returns the one-based position the player starts from, which is the number the HUD
        /// shows and the number a viewer will check the grid against.
        /// </summary>
        private int PlaceOnGrid()
        {
            var field = _aiFieldOverride != null ? _aiFieldOverride : _grid.aiField;
            if (_aiFieldOverride != null)
                _grid.aiField = _aiFieldOverride;

            int gridPosition = _flow.ResolveRaceGridPosition(field);

            // The qualifying result still gets resolved and recorded either way — it is the
            // session's business, and the HUD reports it. This only overrides where the car
            // is actually placed, so the number the player sees and the slot the car occupies
            // are the same number. Overriding only the local variable would have left the
            // session reporting a pole position the car was not in.
            //
            // The field size is read from the grid rather than the field argument, because
            // this method may have just assigned the override onto the grid and the grid is
            // what SpawnGrid counts from.
            if (_startPlayerAtBackOfGrid)
            {
                gridPosition = _grid.aiFieldCount + 1;
                _flow.SetRaceGridPositionOverride(gridPosition);
            }

            // The one indexing conversion in the slice. The session and the HUD count from
            // one; the grid manager counts from zero, because zero is a real slot. Doing the
            // conversion here — at the only boundary where the two meet — means neither side
            // has to know about the other's numbering.
            _grid.playerCar = _spawner.PlayerCar;
            _grid.includePlayerInGrid = true;
            _grid.playerGridPosition = Mathf.Max(0, gridPosition - 1);
            _playerGridSlot = _grid.playerGridPosition;

            _grid.SpawnGrid();
            return gridPosition;
        }

        /// <summary>
        /// Laps and grid position onto the race screen, if this scene hosts one yet.
        ///
        /// The race HUD is a later step, so the screen is legitimately absent for now. The
        /// information is computed either way, because a shell that had to recompute it once
        /// the HUD lands would be a second place for the same answer to live.
        /// </summary>
        private void PublishRaceInfo()
        {
            TotalLaps = _raceLapsOverride > 0
                ? _raceLapsOverride
                : (_flow.SelectedTrack != null ? _flow.SelectedTrack.StandardRaceLaps : 10);

            var screen = GetComponentInChildren<RaceScreen>(true);
            if (screen != null && _flow.SelectedTrack != null)
                screen.SetRaceInfo(_flow.SelectedTrack, TotalLaps, _flow.RaceGridPosition);
        }

        /// <summary>
        /// Live position and lap counter onto the race screen.
        ///
        /// <see cref="PublishRaceInfo"/> pushes the numbers that are fixed when the race is
        /// brought up. These two change every frame, and they are pushed from here rather
        /// than from a HUD component for the reason that method's own comment gives: this
        /// shell is the only thing that holds both halves of the answer. The field lives on
        /// the grid manager and the player's progress lives on the car's tracker, so a
        /// presenter standing off to the side would have to ask this object for both — at
        /// which point the answer is computed here anyway and only the drawing is elsewhere.
        ///
        /// Throttled rather than per-frame. Position only changes when somebody crosses a
        /// line or passes somebody, so recomputing the sort sixty times a second to repaint
        /// an identical string is pure cost. A tenth of a second is well inside what a
        /// viewer can read off a corner of the screen, and the lap counter is event-driven
        /// anyway because <see cref="LapTracker"/> raises a completion event.
        /// </summary>
        private void Update()
        {
            if (!_sessionReady) return;

            _hudPushTimer -= Time.unscaledDeltaTime;
            if (_hudPushTimer > 0f) return;
            _hudPushTimer = HudPushIntervalSeconds;

            PushLiveRaceInfo();
        }

        private const float HudPushIntervalSeconds = 0.1f;
        private float _hudPushTimer;
        private bool _racePushEnabled;

        /// <summary>
        /// Zero-based grid slot the player was placed in, set when the field is placed.
        /// The HUD's tiebreak for "nobody has moved yet" reads it; it is not the live
        /// position and must not be mistaken for one.
        /// </summary>
        private int _playerGridSlot = DriverIdentifier.NoGridSlot;

        private void PushLiveRaceInfo()
        {
            var screen = _flow != null ? _flow.RaceScreenInstance : null;
            if (screen == null) return;

            // The clock belongs to the tracker, not to here: LapHudPresenter is the tracker's
            // binder and already pushes current and best lap every frame. This only turns it
            // on, once, for the car that is actually racing.
            if (!_racePushEnabled && _spawner != null && _spawner.PlayerCar != null)
            {
                var binder = _spawner.PlayerCar.GetComponent<LapHudPresenter>();
                if (binder != null)
                {
                    binder.SetRacePush(true);
                    _racePushEnabled = true;
                }
            }

            int total = TotalFieldSize();
            int position = ComputePlayerPosition();
            if (position > 0)
                screen.UpdatePosition(position, total);

            screen.UpdateLap(ComputePlayerLap(), TotalLaps);
        }

        /// <summary>How many cars are classified, counting the player and every AI still standing.</summary>
        private int TotalFieldSize()
        {
            int total = 0;

            if (_spawner != null && _spawner.PlayerCar != null) total++;

            if (_grid != null)
            {
                foreach (var car in _grid.SpawnedCars)
                {
                    if (car != null) total++;
                }
            }

            // Never report a field of zero: the HUD would render "P1 / 0", which is the kind
            // of number that reads as a bug to a client and is always a bug here.
            return Mathf.Max(1, total);
        }

        /// <summary>
        /// The player's live race position, one-based, or 0 when it cannot be worked out.
        ///
        /// Ranked by laps completed and then by distance around the current lap. Lap count
        /// dominates deliberately: sorting on distance alone would rank a car one lap behind
        /// as the leader, because it is physically further along the same ribbon of tarmac.
        /// <see cref="LapTracker"/> keeps the two on the same scale for exactly this.
        ///
        /// Returns 0 rather than a guess when the player or its tracker is missing, so the
        /// caller can leave the last good number on screen instead of flashing "P1".
        /// </summary>
        private int ComputePlayerPosition()
        {
            if (_spawner == null || _spawner.PlayerCar == null) return 0;

            var playerTracker = _spawner.PlayerCar.GetComponent<LapTracker>();
            if (playerTracker == null) return 0;

            // The player's own grid slot, taken from where it was actually placed rather than
            // read off a component. The player's car prefab carries no DriverIdentifier —
            // the grid manager stamps one on each AI car — and it should not grow one here
            // to serve a HUD readout, since that prefab is hand-authored. The number this
            // shell already computed is the same identity, in the same zero-based numbering.
            int subjectSlot = _playerGridSlot;

            int ahead = 0;

            if (_grid != null)
            {
                foreach (var car in _grid.SpawnedCars)
                {
                    if (car == null) continue;

                    var tracker = car.GetComponent<LapTracker>();
                    if (tracker == null) continue;

                    if (IsAheadOf(playerTracker, tracker, subjectSlot)) ahead++;
                }
            }

            return ahead + 1;
        }

        private static bool IsAheadOf(LapTracker subject, LapTracker other, int subjectSlot)
        {
            if (other.CompletedLaps != subject.CompletedLaps)
                return other.CompletedLaps > subject.CompletedLaps;

            if (!Mathf.Approximately(other.LapDistance, subject.LapDistance))
                return other.LapDistance > subject.LapDistance;

            // Genuine tie, and on the grid that is every car at once: nobody has moved, so
            // laps and distance are all zero and a pure progress sort calls the whole field
            // level. It would then report the player P1 while the car sits ninth on the
            // road, which is the single most checkable number on the HUD.
            //
            // Grid order is the tiebreaker because it is the one ordering that is true
            // before a single metre is covered, and it stays a fair fallback later: two cars
            // within a float epsilon of each other are level, and the one that started
            // nearer the front is the one a viewer expects to be shown ahead.
            var id = other.GetComponent<DriverIdentifier>();
            int otherSlot = id == null ? DriverIdentifier.NoGridSlot : id.GridSlot;

            // An unclassified car sorts last, so one that was never placed on the grid never
            // appears to be ahead of a car that was.
            if (subjectSlot == DriverIdentifier.NoGridSlot) return false;
            if (otherSlot == DriverIdentifier.NoGridSlot) return true;

            return otherSlot < subjectSlot;
        }

        /// <summary>
        /// The lap the player is currently on, one-based.
        ///
        /// The tracker counts laps <i>finished</i>, so the lap being driven is one more than
        /// that. Showing "Lap 1 / 44" on the line rather than "Lap 0 / 44" is the whole
        /// difference between a counter and an off-by-one.
        /// </summary>
        private int ComputePlayerLap()
        {
            if (_spawner == null || _spawner.PlayerCar == null) return 1;

            var tracker = _spawner.PlayerCar.GetComponent<LapTracker>();
            return tracker == null ? 1 : tracker.CompletedLaps + 1;
        }
    }
}
