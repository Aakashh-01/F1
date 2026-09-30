using UnityEngine;
using System;
using System.Collections.Generic;
using F1.GameData;
using F1.Progression;

namespace F1.GameFlow
{
    /// <summary>
    /// Base class for all screen controllers. Inherits from MonoBehaviour to be findable by Unity.
    /// </summary>
    public abstract class ScreenController : MonoBehaviour, IScreenController
    {
        public abstract void Initialize(GameFlowManager flowManager);
        public abstract void Show();
        public abstract void Hide();
    }

    /// <summary>
    /// Branding / splash screen — shows the game name, the Pearl-Lemon attribution the
    /// GDD requires, and loading progress.
    /// </summary>
    public abstract class BrandingScreen : ScreenController, IBrandingScreen
    {
        public event Action OnBrandingComplete;

        public override void Initialize(GameFlowManager flowManager) { }
        public override void Show() { gameObject.SetActive(true); }
        public override void Hide() { gameObject.SetActive(false); }

        public abstract void SetLoadingProgress(float normalized, string status);
        public abstract void SetLoadingError(string message);

        protected void TriggerBrandingComplete() => OnBrandingComplete?.Invoke();
    }

    /// <summary>
    /// Main menu — Start, Options, Quit.
    /// </summary>
    public abstract class MainMenuScreen : ScreenController, IMainMenuScreen
    {
        public event Action OnStartPressed;
        public event Action OnOptionsPressed;
        public event Action OnQuitPressed;

        public override void Initialize(GameFlowManager flowManager) { }
        public override void Show() { gameObject.SetActive(true); }
        public override void Hide() { gameObject.SetActive(false); }

        protected void TriggerStartPressed() => OnStartPressed?.Invoke();
        protected void TriggerOptionsPressed() => OnOptionsPressed?.Invoke();
        protected void TriggerQuitPressed() => OnQuitPressed?.Invoke();
    }

    /// <summary>
    /// The lobby — the 3D garage, showing car selection.
    ///
    /// The screen is always active while the garage is on screen, and it is always showing
    /// the car tiles. There is no hub state and no Start Race button any more, so nothing
    /// here decides which panels are up: <see cref="Show"/> is the plain active toggle the
    /// base class defines.
    /// </summary>
    public abstract class LobbyScreen : ScreenController, ILobbyScreen
    {
        public event Action OnNextPressed;
        public event Action<CarDefinition> OnCarChosen;

        public override void Initialize(GameFlowManager flowManager) { }
        public override void Show() { gameObject.SetActive(true); }
        public override void Hide() { gameObject.SetActive(false); }

        public abstract void SetCurrency(int amount);
        public abstract void RefreshFromProfile();
        public abstract void SetChosenCar(CarDefinition car);

        protected void TriggerNextPressed() => OnNextPressed?.Invoke();
        protected void TriggerCarChosen(CarDefinition car) => OnCarChosen?.Invoke(car);
    }

    /// <summary>
    /// Car selection screen — shows owned, rentable, locked cars by generation.
    /// </summary>
    public abstract class CarSelectionScreen : ScreenController, ICarSelectionScreen
    {
        public event Action<CarDefinition> OnCarSelected;
        public event Action<CarDefinition> OnCarRented;
        public event Action<CarDefinition> OnCarUnlockRequested;
        public event Action OnBackPressed;

        public override void Initialize(GameFlowManager flowManager) { }
        public override void Show() { gameObject.SetActive(true); }
        public override void Hide() { gameObject.SetActive(false); }

        protected void TriggerCarSelected(CarDefinition car) => OnCarSelected?.Invoke(car);
        protected void TriggerCarRented(CarDefinition car) => OnCarRented?.Invoke(car);
        protected void TriggerCarUnlockRequested(CarDefinition car) => OnCarUnlockRequested?.Invoke(car);

        public abstract void PopulateCars(IReadOnlyList<CarDefinition> owned, IReadOnlyList<CarDefinition> rentable, IReadOnlyList<CarDefinition> locked);
        public abstract void SetSelectedCar(CarDefinition car);
    }

    /// <summary>
    /// Track selection screen — shows owned/free and locked tracks.
    /// </summary>
    public abstract class TrackSelectionScreen : ScreenController, ITrackSelectionScreen
    {
        public event Action<TrackDefinition> OnTrackSelected;
        public event Action<TrackDefinition> OnTrackUnlockRequested;
        public event Action OnBackPressed;

        public override void Initialize(GameFlowManager flowManager) { }
        public override void Show() { gameObject.SetActive(true); }
        public override void Hide() { gameObject.SetActive(false); }

        protected void TriggerTrackSelected(TrackDefinition track) => OnTrackSelected?.Invoke(track);
        protected void TriggerTrackUnlockRequested(TrackDefinition track) => OnTrackUnlockRequested?.Invoke(track);

        public abstract void PopulateTracks(IReadOnlyList<TrackDefinition> owned, IReadOnlyList<TrackDefinition> locked);
        public abstract void SetSelectedTrack(TrackDefinition track);
    }

    /// <summary>
    /// Wing setup screen — High Downforce vs Low Downforce with track-specific recommendation.
    /// </summary>
    public abstract class WingSetupScreen : ScreenController, IWingSetupScreen
    {
        public event Action<WingType> OnWingSelected;
        public event Action OnBackPressed;
        public event Action OnContinuePressed;

        public override void Initialize(GameFlowManager flowManager) { }
        public override void Show() { gameObject.SetActive(true); }
        public override void Hide() { gameObject.SetActive(false); }

        protected void TriggerWingSelected(WingType wing) => OnWingSelected?.Invoke(wing);
        protected void TriggerBackPressed() => OnBackPressed?.Invoke();
        protected void TriggerContinuePressed() => OnContinuePressed?.Invoke();

        public abstract void Setup(CarDefinition car, TrackDefinition track, WingType currentWing);
    }

    /// <summary>
        /// Mode selection screen — Qualifying or Race after wing setup.
        /// </summary>
        public abstract class ModeSelectionScreen : ScreenController, IModeSelectionScreen
        {
            public event Action OnQualifyingPressed;
            public event Action OnRacePressed;
            public event Action OnBackPressed;

            public override void Initialize(GameFlowManager flowManager) { }
            public override void Show() { enabled = true; }
            public override void Hide() { enabled = false; }

            protected void TriggerQualifyingPressed() => OnQualifyingPressed?.Invoke();
            protected void TriggerRacePressed() => OnRacePressed?.Invoke();
            protected void TriggerBackPressed() => OnBackPressed?.Invoke();

            public abstract void Setup(CarDefinition car, TrackDefinition track, WingType wing);
        }

        /// <summary>
        /// Qualifying screen — unlimited laps, ghost car, sector timing, racing line.
        /// </summary>
        public abstract class QualifyingScreen : ScreenController, IQualifyingScreen
        {
            public event Action OnBackPressed;
            public event Action OnRacePressed;
            public event Action OnRestartPressed;

            public override void Initialize(GameFlowManager flowManager) { }
            public override void Show() { enabled = true; }
            public override void Hide() { enabled = false; }

            protected void TriggerBackPressed() => OnBackPressed?.Invoke();
            protected void TriggerRacePressed() => OnRacePressed?.Invoke();
            protected void TriggerRestartPressed() => OnRestartPressed?.Invoke();

            public abstract void SetGhostData(string ghostJson);
            public abstract void UpdateLapTime(float currentLapTime, float bestLapTime);
            public abstract void UpdateSectorTimes(float[] sectorTimes, SectorColor[] sectorColors);
            public abstract void SetTrackInfo(TrackDefinition track);

            /// <summary>
            /// The two states a qualifying attempt moves between.
            ///
            /// A real qualifying session is a lap, not a menu: the first attempt puts the
            /// player out alone with the clock running and nothing to click, and the options
            /// (Back / Retry / Go to Race) only appear once a lap has been recorded. Modelling
            /// that as a state rather than a set of always-live buttons is what stops the
            /// screen offering "Go to Race" to a player who has not earned it — the gate still
            /// lives in the flow layer, but the screen should not be advertising it either.
            /// </summary>
            public abstract void ShowDrivingState();
            public abstract void ShowResultsState(float lapTime);
        }

        /// <summary>
        /// Race screen — HUD, position, lap counter, sector times.
        /// </summary>
        public abstract class RaceScreen : ScreenController, IRaceScreen
        {
            public event Action OnPausePressed;

            public override void Initialize(GameFlowManager flowManager) { }
            public override void Show() { enabled = true; }
            public override void Hide() { enabled = false; }

            protected void TriggerPausePressed() => OnPausePressed?.Invoke();

            public abstract void SetRaceInfo(TrackDefinition track, int totalLaps, int gridPosition);
            public abstract void UpdatePosition(int position, int totalCars);
            public abstract void UpdateLap(int currentLap, int totalLaps);
            public abstract void UpdateLapTime(float currentLapTime, float bestLapTime);
            public abstract void UpdateSectorTimes(float[] sectorTimes, SectorColor[] sectorColors);
            public abstract void ShowFinishPosition(int position, int pointsEarned);
        }

        /// <summary>
        /// Results screen — post-race summary, points earned, unlocks available.
        /// </summary>
        public abstract class ResultsScreen : ScreenController, IResultsScreen
        {
            public event Action OnContinuePressed;
            public event Action OnRetryPressed;
            public event Action OnMainMenuPressed;

            public override void Initialize(GameFlowManager flowManager) { }
            public override void Show() { enabled = true; }
            public override void Hide() { enabled = false; }

            protected void TriggerContinuePressed() => OnContinuePressed?.Invoke();
            protected void TriggerRetryPressed() => OnRetryPressed?.Invoke();
            protected void TriggerMainMenuPressed() => OnMainMenuPressed?.Invoke();

            public abstract void ShowResults(int finishPosition, int pointsEarned, int newTotalPoints,
                CarDefinition unlockedCar = null, TrackDefinition unlockedTrack = null);
        }

        /// <summary>
        /// Options screen — settings, controls, audio.
        /// </summary>
        public abstract class OptionsScreen : ScreenController, IOptionsScreen
        {
            public event Action OnBackPressed;

            public override void Initialize(GameFlowManager flowManager) { }
            public override void Show() { enabled = true; }
            public override void Hide() { enabled = false; }

            protected void TriggerBackPressed() => OnBackPressed?.Invoke();

            public abstract void LoadSettings(PlayerProfile profile);
        }
    }