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
    public class RaceSceneController : MonoBehaviour
    {
        [Header("Race Configuration")]
        [Tooltip("Lap count for the race. Leave at -1 to use the selected track definition's " +
                 "standard race laps. Set it to a small number for a demo so a race can be " +
                 "driven to its finish in a sitting.")]
        [SerializeField] private int _raceLapsOverride = -1;

        [Tooltip("Optional. Overrides the AI field configured on the track's grid manager. " +
                 "Leave empty to race whatever field the track already has.")]
        [SerializeField] private AIFieldDefinition _aiFieldOverride;

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

            Debug.Log(
                $"[RaceSceneController] Race ready on '{_track.gameObject.scene.name}' " +
                $"(lap {_track.LapLengthMeters:0}m, {TotalLaps} laps). Player starts P" +
                $"{gridPosition} of {_grid.aiFieldCount + 1}, field of " +
                $"{_grid.aiFieldCount}.");
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

            // The one indexing conversion in the slice. The session and the HUD count from
            // one; the grid manager counts from zero, because zero is a real slot. Doing the
            // conversion here — at the only boundary where the two meet — means neither side
            // has to know about the other's numbering.
            _grid.playerCar = _spawner.PlayerCar;
            _grid.includePlayerInGrid = true;
            _grid.playerGridPosition = Mathf.Max(0, gridPosition - 1);

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
    }
}
