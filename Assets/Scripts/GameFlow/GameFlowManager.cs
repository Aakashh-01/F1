using UnityEngine;
using System;
using System.Collections.Generic;
using F1.GameData;
using F1.Progression;
using WingType = F1.GameData.WingType;

namespace F1.GameFlow
{
    /// <summary>
    /// Central game flow controller. Manages screen state, selections, and scene transitions.
    /// Persists across scenes (DontDestroyOnLoad). Single source of truth for current game session.
    /// All scene loading is delegated to <see cref="SceneFlowService"/>.
    /// </summary>
    public class GameFlowManager : MonoBehaviour
    {
        public enum GameScreen
        {
            Branding, // Game logo / splash
            MainMenu, // "Start", "Options", "Quit"

            /// <summary>
            /// RETIRED, AND IT MUST STAY AT INDEX 2.
            ///
            /// Car selection used to be its own scene; it is now the lobby's only screen, and
            /// 10_CarSelectionScene no longer ships. The member is kept only so the
            /// integers after it keep their values — FlowSceneHost serializes its hosted
            /// screen as this enum's integer, and deleting or moving this member would shift
            /// TrackSelection, WingSetup, Qualifying, Race and Results down by one and leave
            /// every remaining scene claiming the wrong screen. No scene hosts it any more.
            /// </summary>
            CarSelection,

            TrackSelection, // Pick track (owned/free/locked)
            WingSetup, // High/Low downforce selector
            Qualifying, // Unlimited laps, ghost car, sector timing
            Race, // Grid start, race to finish
            Results, // Post-race: points earned, unlocks
            Options, // Settings

            /// <summary>
            /// The lobby — the 3D garage, entered from loading and hosting car selection.
            ///
            /// APPENDED, never inserted, for the same reason CarSelection could not move.
            /// </summary>
            Lobby
        }

        [Header("Scene Names")]
        // Defaults come from FlowSceneNames so the assets, this component, the editor
        // scene builder and the tests cannot drift apart. An empty name means the screen
        // is hosted as an overlay by the current flow scene rather than in its own scene.
        [SerializeField]
        private string _brandingScene = FlowSceneNames.Loading;

        /// <summary>
        /// The lobby the game opens into. This is the 3D garage; car selection happens
        /// inside it as a second UI state rather than as a scene of its own, so nothing
        /// navigates here automatically any more.
        /// </summary>
        [SerializeField] private string _lobbyScene = FlowSceneNames.Lobby;

        [SerializeField] private string _mainMenuScene = "";
        [SerializeField] private string _trackSelectionScene = FlowSceneNames.TrackSelection;
        [SerializeField] private string _wingSetupScene = FlowSceneNames.WingSetup;
        [SerializeField] private string _resultsScene = FlowSceneNames.Results;

        // Qualifying routes to its own shell (S4); the track scene is content that the shell
        // loads additively. This default must move with the scene builder's, or a manager
        // created in code — which is how every test creates one — routes to the track scene
        // while the built scenes route to the shell. The two disagreeing is the drift the
        // comment above these fields exists to prevent.
        [SerializeField] private string _qualifyingScene = FlowSceneNames.PreRace;

        // The race has its own shell too (S5), for the same reason qualifying does. Both
        // shells load the track scene additively underneath themselves, so pre-race and race
        // still resolve the same live track objects (invariant 9).
        [SerializeField] private string _raceScene = FlowSceneNames.Race;

        [Header("References")] [SerializeField]
        private GameObject _uiRootPrefab; // Canvas with all screen controllers

        // --- State ---
        private GameScreen _currentScreen = GameScreen.Branding;
        private readonly GameSessionContext _session = new GameSessionContext();

        // Captured before DontDestroyOnLoad, which permanently rewrites gameObject.scene
        // to the "DontDestroyOnLoad" scene and makes it useless as a self-reference guard.
        private string _birthSceneName;

        public GameSessionContext Session => _session;

        /// <summary>Routing layer. Owns every scene load in the game.</summary>
        public SceneFlowService SceneFlow { get; private set; }

        public bool ShouldSpawnGhostForCurrentAttempt => _session.ShouldSpawnGhostForCurrentAttempt;

        public int RaceGridPosition => _session.RaceGridPosition;
        private int _raceFinishPosition = 1;
        private int _pointsEarned = 0;

        // Screen controllers (found after UI loads)
        private IBrandingScreen _brandingScreen;
        private ILobbyScreen _lobbyScreen;
        private IMainMenuScreen _mainMenuScreen;
        private ICarSelectionScreen _carSelectionScreen;
        private ITrackSelectionScreen _trackSelectionScreen;
        private IWingSetupScreen _wingSetupScreen;
        private IQualifyingScreen _qualifyingScreen;
        private IRaceScreen _raceScreen;
        private IResultsScreen _resultsScreen;
        private IOptionsScreen _optionsScreen;

        // Events for UI binding
        public event Action<GameScreen> OnScreenChanged;
        public event Action<CarDefinition> OnCarSelected;
        public event Action<TrackDefinition> OnTrackSelected;
        public event Action<WingType> OnWingSelected;
        public event Action<float> OnQualifyingTimeSet;
        public event Action OnQualifyingRetryRequested;
        public event Action<int, int> OnRaceFinished; // position, points

        /// <summary>
        /// A transition was asked for and refused — most often because the loader was
        /// already busy, which is exactly what happens if a player presses a button while a
        /// scene is still streaming. The flow has already rolled itself back to the screen
        /// that is genuinely loaded by the time this fires.
        /// </summary>
        public event Action<GameScreen, SceneLoadResult> OnTransitionFailed;

        // Singleton
        public static GameFlowManager Instance { get; private set; }

        private void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }

            Instance = this;

            // Must be read before DontDestroyOnLoad moves this object out of its scene.
            _birthSceneName = gameObject.scene.IsValid() ? gameObject.scene.name : null;

            DontDestroyOnLoad(gameObject);

            SceneFlow = new SceneFlowService(this);
            SceneFlow.AdoptFlowScene(_birthSceneName);

            // Ensure registry is initialized
            GameDataRegistry.Initialize();

            // Load player profile
            var _ = PlayerProfileManager.Current; // Triggers load

            // Subscribe to profile changes
            PlayerProfileManager.OnProfileLoaded += OnProfileLoaded;
            PlayerProfileManager.OnProfileSaved += OnProfileSaved;
        }

        private void OnDestroy()
        {
            if (Instance != this)
                return;

            PlayerProfileManager.OnProfileLoaded -= OnProfileLoaded;
            PlayerProfileManager.OnProfileSaved -= OnProfileSaved;
            Instance = null;
        }

        private void Start()
        {
            _session.FlowState = GameFlowState.Loading;
            TransitionToScreen(GameScreen.Branding);
        }

        private void OnProfileLoaded(PlayerProfile profile)
        {
            UnityEngine.Debug.Log(
                $"[GameFlowManager] Profile loaded: {profile.playerName}, Points: {profile.careerPoints}");
        }

        private void OnProfileSaved(PlayerProfile profile)
        {
            // Auto-saved
        }

        // --- Public API for UI screens ---

        public GameScreen CurrentScreen => _currentScreen;

        // Selection lives only on the session context. These are the single source of
        // truth — the manager keeps no parallel copies (finding L1).
        public CarDefinition SelectedCar => _session.SelectedCar;
        public TrackDefinition SelectedTrack => _session.SelectedTrack;
        public WingType SelectedWing => _session.SelectedWing;

        public float BestQualifyingTime => _session.Qualifying.BestLapTime;
        public string QualifyingGhostData => _session.Qualifying.BestGhostData;
        public bool HasGhostForNextAttempt => _session.HasGhostForNextAttempt;
        public bool CanStartRace => _session.CanStartRace;
        public GameFlowState CurrentFlowState => _session.FlowState;
        public SessionType CurrentSessionType => _session.SessionType;

        /// <summary>
        /// The car selection screen resolved from the hosting scene, or null. Exposed so
        /// the hosting mechanism itself can be verified without reaching into privates.
        /// </summary>
        public ICarSelectionScreen CarSelectionScreenInstance => _carSelectionScreen;

        /// <summary>
        /// The lobby screen resolved from the hosting scene, or null. Exposed for the same
        /// reason as the others: so the hosting mechanism can be verified without reaching
        /// into privates.
        /// </summary>
        public ILobbyScreen LobbyScreenInstance => _lobbyScreen;

        /// <summary>
        /// The qualifying screen resolved from the hosting scene, or null.
        ///
        /// Exposed as the session-scoped handle to the qualifying HUD, so the lap-timing
        /// binder can reach the screen the flow already resolved instead of running its own
        /// FindAnyObjectByType search. That search is exactly what Phase 3 removed: it skips
        /// inactive objects, and the HUD is routinely inactive until the session starts
        /// (latent defect L4). The value legitimately changes when the flow transitions, so
        /// callers must re-read it rather than cache it.
        /// </summary>
        public IQualifyingScreen QualifyingScreenInstance => _qualifyingScreen;

        /// <summary>
        /// The race screen resolved from the hosting scene, or null. Same contract as
        /// <see cref="QualifyingScreenInstance"/>: host-resolved, and re-read on each use.
        /// </summary>
        public IRaceScreen RaceScreenInstance => _raceScreen;

        /// <summary>
        /// Re-enters the loading scene. Needed on a failed load, and by the entry route's
        /// own first transition.
        /// </summary>
        public void GoToBranding()
        {
            _session.FlowState = GameFlowState.Loading;
            TransitionToScreen(GameScreen.Branding);
        }

        public void SelectCar(CarDefinition car)
        {
            if (car == null) return;
            var profile = PlayerProfileManager.Current;
            if (profile != null && !profile.IsCarAvailable(car.CarId))
            {
                UnityEngine.Debug.LogWarning($"[GameFlowManager] Car {car.CarId} not available");
                return;
            }

            _session.SetSelection(car, _session.SelectedTrack, _session.SelectedWing);
            OnCarSelected?.Invoke(car);
            UnityEngine.Debug.Log($"[GameFlowManager] Car selected: {car.DisplayName}");
        }

        public void SelectTrack(TrackDefinition track)
        {
            if (track == null) return;
            var profile = PlayerProfileManager.Current;
            if (profile != null && !profile.OwnsTrack(track.TrackId) && track.UnlockCostPoints > 0)
            {
                UnityEngine.Debug.LogWarning($"[GameFlowManager] Track {track.TrackId} not owned");
                return;
            }

            _session.SetSelection(_session.SelectedCar, track, _session.SelectedWing);
            OnTrackSelected?.Invoke(track);
            UnityEngine.Debug.Log($"[GameFlowManager] Track selected: {track.DisplayName}");
        }

        public void SelectWing(WingType wing)
        {
            _session.SetSelection(_session.SelectedCar, _session.SelectedTrack, wing);
            OnWingSelected?.Invoke(wing);
            UnityEngine.Debug.Log($"[GameFlowManager] Wing selected: {wing}");

            // Save preference for this track
            var track = _session.SelectedTrack;
            if (track != null)
            {
                var profile = PlayerProfileManager.Current;
                profile?.SetWingPreference(track.TrackId, (int)wing);
            }
        }

        public void SetQualifyingTime(float lapTime, string ghostData)
        {
            if (!_session.CanStartQualifying)
            {
                UnityEngine.Debug.LogWarning(
                    "[GameFlowManager] Select a car and track before recording qualifying time");
                return;
            }

            var car = _session.SelectedCar;
            var track = _session.SelectedTrack;
            bool countForRanking = PlayerProfileManager.Current != null &&
                                   PlayerProfileManager.Current.OwnsCar(car.CarId);
            bool isBest = _session.RecordQualifyingLap(lapTime, ghostData, countForRanking);
            if (isBest && countForRanking && track != null)
            {
                var profile = PlayerProfileManager.Current;
                profile?.SetBestLapTime(track.TrackId, lapTime);
                if (!string.IsNullOrWhiteSpace(ghostData))
                    profile?.SetGhostData(track.TrackId, ghostData);
            }

            OnQualifyingTimeSet?.Invoke(_session.Qualifying.BestLapTime);
        }

        public void SetRaceResult(int finishPosition, int pointsEarned)
        {
            _raceFinishPosition = finishPosition;
            _pointsEarned = pointsEarned;

            var profile = PlayerProfileManager.Current;
            if (profile != null)
            {
                profile.AddCareerPoints(pointsEarned);
                profile.RecordRaceFinish(finishPosition);
            }

            OnRaceFinished?.Invoke(finishPosition, pointsEarned);
        }

        // --- Navigation ---

        public void GoToMainMenu() => TransitionToScreen(GameScreen.MainMenu);

        /// <summary>
        /// Opens the lobby — the 3D garage. This is where the game starts after loading.
        /// </summary>
        public void GoToLobby()
        {
            _session.FlowState = GameFlowState.Lobby;
            TransitionToScreen(GameScreen.Lobby);
        }

        public void GoToTrackSelection()
        {
            _session.FlowState = GameFlowState.TrackSelection;
            TransitionToScreen(GameScreen.TrackSelection);
        }

        public void GoToWingSetup()
        {
            _session.FlowState = GameFlowState.WingSetup;
            TransitionToScreen(GameScreen.WingSetup);
        }

        public void GoToQualifying()
        {
            _session.FlowState = GameFlowState.PreRace;
            TransitionToScreen(GameScreen.Qualifying);
        }

        public void GoToRace()
        {
            _session.FlowState = GameFlowState.Race;
            TransitionToScreen(GameScreen.Race);
        }

        public void GoToResults()
        {
            _session.FlowState = GameFlowState.Results;
            TransitionToScreen(GameScreen.Results);
        }

        public void GoToOptions() => TransitionToScreen(GameScreen.Options);

        /// <summary>
        /// Loads the selected track's content scene additively. Used by the pre-race and
        /// race shells so both resolve the same live track objects.
        ///
        /// Returns the coroutine so a caller can wait for the content to actually be there
        /// before placing a car on it. The routing service refuses a second load while one
        /// is in flight, so a caller that polls for readiness instead of awaiting this will
        /// eventually be told it is Busy and has to handle that as a failure.
        /// </summary>
        public Coroutine LoadTrackContent(Action<SceneLoadResult> onComplete = null)
        {
            var sceneName = GetSelectedTrackSceneName();
            if (SceneFlow == null)
            {
                var failure = SceneLoadResult.Fail(sceneName, SceneLoadStatus.LoadFailed,
                    "SceneFlowService is not initialised.");
                onComplete?.Invoke(failure);
                return null;
            }

            return SceneFlow.LoadContent(sceneName, onComplete);
        }

        public void UnloadTrackContent(Action onComplete = null)
        {
            if (SceneFlow == null)
            {
                onComplete?.Invoke();
                return;
            }

            SceneFlow.UnloadContent(GetSelectedTrackSceneName(), onComplete);
        }

        public bool IsTrackContentLoaded =>
            SceneFlow != null && SceneFlow.IsContentLoaded(GetSelectedTrackSceneName());

        public void StartQualifyingSession()
        {
            if (!_session.CanStartQualifying)
            {
                UnityEngine.Debug.LogWarning(
                    "[GameFlowManager] Cannot start qualifying without a selected car and track");
                return;
            }

            _session.BeginQualifyingSession();
            var profile = PlayerProfileManager.Current;
            var car = _session.SelectedCar;
            var track = _session.SelectedTrack;
            if (profile != null && car != null && track != null && profile.OwnsCar(car.CarId))
                _session.RestoreSavedBestGhost(profile.GetBestLapTime(track.TrackId),
                    profile.GetGhostData(track.TrackId));

            _session.FlowState = GameFlowState.PreRace;
            GoToQualifying();
        }

        public void RetryQualifyingSession()
        {
            if (_session.Qualifying.AttemptCount <= 0)
            {
                StartQualifyingSession();
                return;
            }

            _session.BeginQualifyingRetry();
            _session.FlowState = GameFlowState.PreRace;
            OnQualifyingRetryRequested?.Invoke();
            GoToQualifying();
        }

        public void StartRaceSession()
        {
            if (!_session.CanStartRace)
            {
                UnityEngine.Debug.LogWarning("[GameFlowManager] Race requires a completed qualifying session");
                return;
            }

            _session.SessionType = SessionType.Race;
            _session.FlowState = GameFlowState.Race;
            GoToRace();
        }

        /// <summary>
        /// Works out which grid slot the qualifying result earns, and records it.
        ///
        /// The rule itself lives in <see cref="GridPositionResolver"/>, which is a pure
        /// function; this method is the seam that feeds it the session's lap and the field's
        /// benchmarks, and stores the answer back on the session. The policy is separated from
        /// the singleton so it can be tested directly rather than through a flow instance.
        ///
        /// The benchmarks exist because qualifying is player-only (invariant 6), so the lap
        /// the player set has nothing on track to be compared against. Without them the slot
        /// would stay at its default of 1 and every race would start from pole, which makes
        /// the whole qualifying session decorative.
        /// </summary>
        /// <returns>The one-based grid position, which is also stored on the session.</returns>
        public int ResolveRaceGridPosition(AIFieldDefinition field)
        {
            int position = field == null
                ? GridPositionResolver.Resolve(_session.Qualifying.BestLapTime, 0, null)
                : GridPositionResolver.Resolve(
                    _session.Qualifying.BestLapTime, field.Count, field.GetBenchmarkLapSeconds);

            _session.SetRaceGridPosition(position);
            return position;
        }

        /// <summary>
        /// Records where the player is actually starting, when that is not what the
        /// qualifying result earned.
        ///
        /// A caller that overrides the grid slot must say so here as well. The stored value
        /// is what the race HUD reads through <see cref="RaceGridPosition"/>, so overriding
        /// only the slot the car is placed into would leave the HUD reporting a pole position
        /// the car was never in — a number a viewer can check against the grid, and wrong.
        ///
        /// The qualifying result is not erased by this. It is still resolved and still means
        /// something; this only says the car is not standing on it.
        /// </summary>
        public void SetRaceGridPositionOverride(int oneBasedPosition) =>
            _session.SetRaceGridPosition(oneBasedPosition);

        public void RestartSession()
        {
            if (_session.SessionType == SessionType.Qualifying) GoToQualifying();
            else if (_session.SessionType == SessionType.Race) GoToRace();
        }

        public void ReturnToMainMenu()
        {
            _session.SessionType = SessionType.None;
            _session.ResetSessionData();
            GoToMainMenu();
        }

        // --- Screen transition logic ---

        private void TransitionToScreen(GameScreen targetScreen)
        {
            if (_currentScreen == targetScreen) return;

            UnityEngine.Debug.Log($"[GameFlowManager] Transition: {_currentScreen} -> {targetScreen}");

            // Captured so a failed load can be undone. The screen is NOT committed below
            // until the load reports success.
            GameScreen previousScreen = _currentScreen;

            string sceneName = GetSceneNameForScreen(targetScreen);

            if (string.IsNullOrEmpty(sceneName))
            {
                // This screen is a UI overlay hosted by the current flow scene.
                _currentScreen = targetScreen;
                InitializeScreenController(targetScreen);
                OnScreenChanged?.Invoke(targetScreen);
                return;
            }

            if (SceneFlow == null)
            {
                UnityEngine.Debug.LogError(
                    $"[GameFlowManager] Cannot route to {targetScreen}: no SceneFlowService.");
                return;
            }

            // The service decides whether this is a real transition, tracks the target
            // scene by handle, preserves additively loaded content, and reports failures.
            SceneFlow.LoadFlowScene(sceneName, result =>
            {
                if (!result.Success)
                {
                    // Rolled back rather than left pointing at a scene that never loaded.
                    //
                    // This used to set _currentScreen (and, in the GoTo* callers, the
                    // session's FlowState) *before* asking for the load, so a failure left
                    // the manager convinced it had arrived. Reproduced by asking for the
                    // race while the loading scene was still mid-transition: the service
                    // refuses with Busy, the manager logged the error and returned, and the
                    // flow sat in the lobby reporting FlowState == Race with no way back —
                    // every later GoToRace short-circuited on "already there".
                    //
                    // A refused load is a legitimate outcome, not a broken one. The screen
                    // that is actually loaded is still the previous one, so saying so is
                    // the only truthful answer, and it leaves the player somewhere they can
                    // press a button and try again.
                    UnityEngine.Debug.LogError(
                        $"[GameFlowManager] Failed to enter {targetScreen} — {result}");

                    _currentScreen = previousScreen;
                    _session.FlowState = FlowStateFor(previousScreen);

                    // Re-announced so any screen controller that optimistically moved on is
                    // put back in step with the screen that is really hosted.
                    OnScreenChanged?.Invoke(previousScreen);
                    OnTransitionFailed?.Invoke(targetScreen, result);
                    return;
                }

                _currentScreen = targetScreen;
                // The committing transition owns the state as well as the screen. Without
                // this, an overlapping pair of transitions could leave them disagreeing: a
                // refused transition rolls the state back, and a different transition that
                // was already in flight then succeeds and moves the screen without moving
                // the state, so the flow reports one thing and shows another. Every GoTo*
                // caller sets the same value up front — this does not change any of them,
                // it just makes the value a property of the screen that actually loaded
                // rather than of whichever call happened to be written last.
                _session.FlowState = FlowStateFor(targetScreen);
                InitializeScreenController(targetScreen);
                OnScreenChanged?.Invoke(targetScreen);
            });
        }

        /// <summary>
        /// The session state a screen implies, used to roll a failed transition back to a
        /// truthful value. Branding and the main menu are overlay-only and never own a scene,
        /// so they map to <see cref="GameFlowState.Loading"/>, which is where a session that
        /// has not chosen anything belongs.
        /// </summary>
        private static GameFlowState FlowStateFor(GameScreen screen)
        {
            switch (screen)
            {
                case GameScreen.Lobby: return GameFlowState.Lobby;
                case GameScreen.TrackSelection: return GameFlowState.TrackSelection;
                case GameScreen.WingSetup: return GameFlowState.WingSetup;
                // The screen is Qualifying even though the scene and the state are both
                // called PreRace — 40_PreRaceScene hosts the qualifying HUD. Matching the
                // scene name here would not compile, which is the only reason this is worth
                // stating: the two vocabularies genuinely differ.
                case GameScreen.Qualifying: return GameFlowState.PreRace;
                case GameScreen.Race: return GameFlowState.Race;
                case GameScreen.Results: return GameFlowState.Results;
                default: return GameFlowState.Loading;
            }
        }

        private string GetSceneNameForScreen(GameScreen screen)
        {
            switch (screen)
            {
                case GameScreen.Branding: return _brandingScene;
                case GameScreen.Lobby: return _lobbyScene;
                case GameScreen.MainMenu: return _mainMenuScene;
                case GameScreen.TrackSelection: return _trackSelectionScene;
                case GameScreen.WingSetup: return _wingSetupScene;
                case GameScreen.Qualifying: return _qualifyingScene;
                case GameScreen.Race: return _raceScene;
                case GameScreen.Results: return _resultsScene;
                case GameScreen.Options: return _mainMenuScene; // Options overlay on menu
                default: return null;
            }
        }

        private void InitializeScreenController(GameScreen screen)
        {
            // Preferred path: ask the scene that was just loaded which screen it hosts.
            // This is reliable even when the screen starts inactive, which
            // FindAnyObjectByType is not — that API skips inactive objects, and that was
            // latent defect L4 in the plan.
            var host = FindSceneHost(screen);
            if (host != null)
            {
                if (host.HasScreenInstance)
                {
                    host.ScreenInstance.Initialize(this);
                    AssignScreen(screen, host.ScreenInstance);
                    return;
                }

                // A host with no screen prefab is legitimate while a screen's UI has not
                // been built yet (the track scene has no qualifying/race HUD until
                // Phase 6/7). Surface it so the omission is visible, but do not treat it
                // as a broken flow.
                UnityEngine.Debug.LogWarning(
                    $"[GameFlowManager] Scene '{SceneFlow?.CurrentFlowSceneName}' hosts " +
                    $"{screen} but has no screen prefab assigned, so no UI was initialized.", this);
                return;
            }

            // Legacy fallback for scenes that predate FlowSceneHost. Track_01 still finds
            // its screens this way until Phase 5 splits it into flow shells.
            if (TryFindLegacyScreen(screen, out var legacy))
            {
                legacy.Initialize(this);
                AssignScreen(screen, legacy);
                return;
            }

            UnityEngine.Debug.LogError(
                $"[GameFlowManager] Entered {screen} but found no screen controller. " +
                $"Scene '{SceneFlow?.CurrentFlowSceneName}' needs a FlowSceneHost declaring " +
                $"it hosts {screen}.", this);
        }

        private void AssignScreen(GameScreen screen, ScreenController controller)
        {
            switch (screen)
            {
                case GameScreen.Branding: _brandingScreen = controller as BrandingScreen; break;
                case GameScreen.Lobby: _lobbyScreen = controller as LobbyScreen; break;
                case GameScreen.MainMenu: _mainMenuScreen = controller as MainMenuScreen; break;
                case GameScreen.CarSelection: _carSelectionScreen = controller as CarSelectionScreen; break;
                case GameScreen.TrackSelection: _trackSelectionScreen = controller as TrackSelectionScreen; break;
                case GameScreen.WingSetup: _wingSetupScreen = controller as WingSetupScreen; break;
                case GameScreen.Qualifying: _qualifyingScreen = controller as QualifyingScreen; break;
                case GameScreen.Race: _raceScreen = controller as RaceScreen; break;
                case GameScreen.Results: _resultsScreen = controller as ResultsScreen; break;
                case GameScreen.Options: _optionsScreen = controller as OptionsScreen; break;
            }
        }

        private bool TryFindLegacyScreen(GameScreen screen, out ScreenController controller)
        {
            controller = screen switch
            {
                GameScreen.Branding => FindAnyObjectByType<BrandingScreen>(),
                GameScreen.MainMenu => FindAnyObjectByType<MainMenuScreen>(),
                GameScreen.CarSelection => FindAnyObjectByType<CarSelectionScreen>(),
                GameScreen.TrackSelection => FindAnyObjectByType<TrackSelectionScreen>(),
                GameScreen.WingSetup => FindAnyObjectByType<WingSetupScreen>(),
                GameScreen.Qualifying => FindAnyObjectByType<QualifyingScreen>(),
                GameScreen.Race => FindAnyObjectByType<RaceScreen>(),
                GameScreen.Results => FindAnyObjectByType<ResultsScreen>(),
                GameScreen.Options => FindAnyObjectByType<OptionsScreen>(),
                _ => null
            };

            return controller != null;
        }

        // --- Scene host registry ---

        // Hosts are held as a list rather than a Dictionary keyed by screen, because a
        // host's hosted screen can change over its lifetime: the shared track scene
        // serves Qualifying then Race as the session changes. A dictionary would keep a
        // stale key pointing at that host under its original screen, and the Race
        // lookup would miss it entirely.
        private readonly List<FlowSceneHost> _sceneHosts = new List<FlowSceneHost>();

        /// <summary>
        /// Called by <see cref="FlowSceneHost"/> from its Awake. The host has already
        /// instantiated its screen by this point.
        /// </summary>
        public void RegisterSceneHost(FlowSceneHost host)
        {
            if (host == null || _sceneHosts.Contains(host))
                return;

            _sceneHosts.Add(host);
        }

        public void UnregisterSceneHost(FlowSceneHost host)
        {
            if (host == null) return;
            _sceneHosts.Remove(host);
        }

        private FlowSceneHost FindSceneHost(GameScreen screen)
        {
            foreach (var host in _sceneHosts)
            {
                if (host != null && host.HostedScreen == screen)
                    return host;
            }

            return null;
        }

        // --- Car physics profile for current selection ---

        /// <summary>
        /// Creates a VehiclePhysicsProfile with the selected car and wing applied.
        /// Call this when spawning the player car in Qualifying/Race scenes.
        /// </summary>
        public VehiclePhysicsProfile CreatePlayerPhysicsProfile()
        {
            var car = _session.SelectedCar;
            if (car == null) return null;
            return car.CreateProfileWithWing(_session.SelectedWing);
        }

        /// <summary>
        /// Gets the track scene name for the selected track.
        /// </summary>
        public string GetSelectedTrackSceneName()
        {
            return _session.SelectedTrack?.SceneName;
        }

        /// <summary>
        /// Gets the racing line for the selected track (for AI, ghost, sectors).
        /// </summary>
        public AIRacingLine GetSelectedTrackRacingLine()
        {
            return _session.SelectedTrack?.RacingLineReference;
        }

        // --- Auto-save ---
        private void OnApplicationPause(bool pause)
        {
            if (pause) PlayerProfileManager.AutoSaveIfDirty();
        }

        private void OnApplicationFocus(bool focus)
        {
            if (!focus) PlayerProfileManager.AutoSaveIfDirty();
        }
    }
}