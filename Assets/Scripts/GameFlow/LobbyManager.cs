using UnityEngine;
using System;
using System.Collections.Generic;
using F1.GameData;
using F1.Progression;
using GameScreen = F1.GameFlow.GameFlowManager.GameScreen;

namespace F1.GameFlow
{
    /// <summary>
    /// Manages screen switching within the single LobbyScene.
    /// Instantiates prefab screens and toggles them on/off.
    /// No scene transitions for menu flow.
    /// </summary>
    public class LobbyManager : MonoBehaviour
    {
        [Header("Lobby Root")] [SerializeField]
        private Transform _screenRoot; // Canvas transform where screens live

        [Header("Screen Prefabs")] [SerializeField]
        private BrandingScreen _brandingPrefab;

        [SerializeField] private MainMenuScreen _mainMenuPrefab;
        [SerializeField] private CarSelectionScreen _carSelectionPrefab;
        [SerializeField] private TrackSelectionScreen _trackSelectionPrefab;
        [SerializeField] private WingSetupScreen _wingSetupPrefab;

        // Current active screen
        private ScreenController _currentScreen;

        // All instantiated screens (cached for toggling)
        private readonly Dictionary<GameScreen, ScreenController> _screens = new();

        private GameFlowManager _flow;

        private void Awake()
        {
            if (_screenRoot == null)
                Debug.LogError("[LobbyManager] ScreenRoot is not assigned!");
        }

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError("[LobbyManager] No GameFlowManager found!");
                return;
            }

            // Instantiate all screen prefabs
            InstantiateAllScreens();

            // Subscribe to screen change events
            _flow.OnScreenChanged += OnFlowScreenChanged;

            // Show branding screen initially
            SwitchTo(GameScreen.Branding);
        }

        private void InstantiateAllScreens()
        {
            if (_brandingPrefab != null) _screens[GameScreen.Branding] = Instantiate(_brandingPrefab, _screenRoot);
            if (_mainMenuPrefab != null) _screens[GameScreen.MainMenu] = Instantiate(_mainMenuPrefab, _screenRoot);
            if (_carSelectionPrefab != null)
                _screens[GameScreen.CarSelection] = Instantiate(_carSelectionPrefab, _screenRoot);
            if (_trackSelectionPrefab != null)
                _screens[GameScreen.TrackSelection] = Instantiate(_trackSelectionPrefab, _screenRoot);
            if (_wingSetupPrefab != null) _screens[GameScreen.WingSetup] = Instantiate(_wingSetupPrefab, _screenRoot);

            // Pre-initialize (GameFlowManager's FindAnyObjectByType skips inactive objects,
            // and these are hidden right below) and show only via SwitchTo.
            foreach (var screen in _screens.Values)
            {
                screen.Initialize(_flow);
                screen.Hide();
            }

            WireNavigation();
        }

        /// <summary>
        /// Subscribes screen events to GameFlowManager navigation — the UI-to-flow glue.
        /// </summary>
        private void WireNavigation()
        {
            if (_screens.TryGetValue(GameScreen.Branding, out var branding))
                ((BrandingScreen)branding).OnBrandingComplete += _flow.GoToLobby;

            if (_screens.TryGetValue(GameScreen.MainMenu, out var menu))
            {
                ((MainMenuScreen)menu).OnStartPressed += _flow.GoToLobby;
                ((MainMenuScreen)menu).OnQuitPressed += QuitGame;
                // OnOptionsPressed intentionally unwired: no OptionsScreen prefab in this phase.
            }

            if (_screens.TryGetValue(GameScreen.CarSelection, out var cars))
            {
                ((CarSelectionScreen)cars).OnCarSelected += OnCarChosen;
                ((CarSelectionScreen)cars).OnCarRented += OnCarChosen;
            }

            if (_screens.TryGetValue(GameScreen.TrackSelection, out var tracks))
                ((TrackSelectionScreen)tracks).OnTrackSelected += OnTrackChosen;

            if (_screens.TryGetValue(GameScreen.WingSetup, out var wing))
            {
                ((WingSetupScreen)wing).OnContinuePressed += _flow.StartQualifyingSession;
                ((WingSetupScreen)wing).OnBackPressed += _flow.GoToTrackSelection;
            }
        }

        // Auto-advance: this manager predates the hub and has no Continue button of its own.
        //
        // A chosen car used to route to a dedicated car selection screen, which is retired.
        // Car selection is now the lobby's own second UI state, so the next step after a car
        // is chosen is track selection.
        private void OnCarChosen(CarDefinition _) => _flow.GoToTrackSelection();
        private void OnTrackChosen(TrackDefinition _) => _flow.GoToWingSetup();

        private static void QuitGame()
        {
            Application.Quit();
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#endif
        }

        private void UnwireNavigation()
        {
            if (_screens.TryGetValue(GameScreen.Branding, out var branding))
                ((BrandingScreen)branding).OnBrandingComplete -= _flow.GoToLobby;

            if (_screens.TryGetValue(GameScreen.MainMenu, out var menu))
            {
                ((MainMenuScreen)menu).OnStartPressed -= _flow.GoToLobby;
                ((MainMenuScreen)menu).OnQuitPressed -= QuitGame;
            }

            if (_screens.TryGetValue(GameScreen.CarSelection, out var cars))
            {
                ((CarSelectionScreen)cars).OnCarSelected -= OnCarChosen;
                ((CarSelectionScreen)cars).OnCarRented -= OnCarChosen;
            }

            if (_screens.TryGetValue(GameScreen.TrackSelection, out var tracks))
                ((TrackSelectionScreen)tracks).OnTrackSelected -= OnTrackChosen;

            if (_screens.TryGetValue(GameScreen.WingSetup, out var wing))
            {
                ((WingSetupScreen)wing).OnContinuePressed -= _flow.StartQualifyingSession;
                ((WingSetupScreen)wing).OnBackPressed -= _flow.GoToTrackSelection;
            }
        }

        private void OnFlowScreenChanged(GameScreen screen)
        {
            SwitchTo(screen);
        }

        public void SwitchTo(GameScreen screen)
        {
            // Hide current
            if (_currentScreen != null)
                _currentScreen.Hide();

            // Show target
            if (_screens.TryGetValue(screen, out var target))
            {
                target.Show();
                _currentScreen = target;
            }
            else
            {
                Debug.LogWarning($"[LobbyManager] No prefab registered for screen: {screen}");
            }
        }

        private void OnDestroy()
        {
            if (_flow != null)
            {
                _flow.OnScreenChanged -= OnFlowScreenChanged;
                if (_screens.Count > 0)
                    UnwireNavigation();
            }
        }
    }
}