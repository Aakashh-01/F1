using UnityEngine;
using System;
using System.Collections.Generic;
using F1.GameData;
using F1.Progression;

namespace F1.GameFlow
{
    /// <summary>
    /// Base interface for all screen controllers.
    /// </summary>
    public interface IScreenController
    {
        void Initialize(GameFlowManager flowManager);
        void Show();
        void Hide();
    }

    /// <summary>
    /// Branding / splash screen — shows the game name, the Pearl-Lemon attribution the
    /// GDD requires, and loading progress. The GDD specifies no "press any key" gate, so
    /// the screen advances automatically once loading completes.
    /// </summary>
    public interface IBrandingScreen : IScreenController
    {
        event Action OnBrandingComplete;

        /// <summary>Normalized 0..1 load progress, plus a short human-readable status.</summary>
        void SetLoadingProgress(float normalized, string status);

        /// <summary>Reports a load failure in place of progressing further.</summary>
        void SetLoadingError(string message);
    }

    /// <summary>
    /// Main menu — Start, Options, Quit.
    /// </summary>
    public interface IMainMenuScreen : IScreenController
    {
        event Action OnStartPressed;
        event Action OnOptionsPressed;
        event Action OnQuitPressed;
    }

    /// <summary>
    /// The lobby — the 3D garage, showing car selection.
    ///
    /// This screen used to carry two states: a hub (currency, settings, tasks and a START
    /// RACE button) and the car tiles, swapped by pressing Start Race. The hub is gone. The
    /// lobby now opens straight onto the tiles, so there is no state to swap and nothing
    /// to press first — the car list IS the lobby's first screen.
    ///
    /// The car, camera and lighting are untouched by any of this: the same garage is on
    /// screen throughout, which was always the point of folding car selection into the
    /// lobby rather than putting a menu in front of it.
    /// </summary>
    public interface ILobbyScreen : IScreenController
    {
        /// <summary>
        /// Next pressed, with a car chosen. This is the point the flow leaves the garage
        /// and loads track selection.
        /// </summary>
        event Action OnNextPressed;

        /// <summary>
        /// A car tile was chosen. Raised on selection only - not on rent or unlock, which
        /// have their own paths. The screen does NOT navigate: it marks the tile and enables
        /// Next, and the flow moves on only when Next is pressed.
        /// </summary>
        event Action<CarDefinition> OnCarChosen;

        /// <summary>Premium-currency readout shown above the tiles.</summary>
        void SetCurrency(int amount);

        /// <summary>
        /// Re-reads the currency figure from the player profile.
        ///
        /// Declared on the interface rather than reached by casting to the concrete impl
        /// because F1.GameFlow is its own assembly and cannot see the UI scripts, which live
        /// in the predefined Assembly-CSharp. The other scene controllers keep the same
        /// boundary by talking to the abstract ScreenController types.
        /// </summary>
        void RefreshFromProfile();

        /// <summary>
        /// Marks a car as the chosen tile, or clears the choice when passed null.
        ///
        /// The scene controller calls this back with whatever the flow ACCEPTED, rather than
        /// the screen deciding for itself. Tapping a rental the player has no rental for is
        /// rejected by <c>SelectCar</c>, and if the screen kept its own optimistic mark the
        /// tile would look chosen, Next would light up, and pressing it would do nothing.
        /// </summary>
        void SetChosenCar(CarDefinition car);
    }

    /// <summary>
    /// The garage — the client-demo replacement for the lobby's car tiles.
    ///
    /// The 3D car on the turntable IS the selection: prev/next arrows cycle a focus car,
    /// the panel describes it, and the flow's session car follows the focus automatically
    /// whenever the focused car is usable. There is no select button and no tile strip.
    ///
    /// The screen owns widgets and events only. The cycle order, the availability check
    /// and the session-car update all live in the scene controller, so replacing this
    /// view with the real art later touches nothing above the screen layer.
    /// </summary>
    public interface IGarageScreen : ILobbyScreen
    {
        /// <summary>
        /// The view changed which car it displays on its own initiative (a gesture or a
        /// control the controller did not route). The controller re-focuses the named car.
        /// The controller's own focus changes are pushed with <see cref="ShowFocusedCar"/>
        /// and must not echo back through this event.
        /// </summary>
        event Action<CarDefinition> OnFocusChanged;

        /// <summary>The previous-car arrow was pressed.</summary>
        event Action OnPrevCarPressed;

        /// <summary>The next-car arrow was pressed.</summary>
        event Action OnNextCarPressed;

        /// <summary>A drag on the car began.</summary>
        event Action OnDragStarted;

        /// <summary>The drag moved, as horizontal pointer delta in pixels.</summary>
        event Action<float> OnDragDelta;

        /// <summary>The drag ended.</summary>
        event Action OnDragEnded;

        /// <summary>
        /// Shows <paramref name="car"/> as the focused car. <paramref name="isSessionCar"/>
        /// is true when the flow accepted it as the session car;
        /// <paramref name="isAvailable"/> is false for locked cars, which render their
        /// stats but cannot be taken into a session.
        /// </summary>
        void ShowFocusedCar(CarDefinition car, bool isSessionCar, bool isAvailable);
    }

    /// <summary>
    /// Car selection screen — shows owned, rentable, locked cars by generation.
    /// </summary>
    public interface ICarSelectionScreen : IScreenController
    {
        event Action<CarDefinition> OnCarSelected;
        event Action<CarDefinition> OnCarRented;
        event Action<CarDefinition> OnCarUnlockRequested;
        event Action OnBackPressed;

        void PopulateCars(IReadOnlyList<CarDefinition> owned, IReadOnlyList<CarDefinition> rentable, IReadOnlyList<CarDefinition> locked);
        void SetSelectedCar(CarDefinition car);
    }

    /// <summary>
    /// Track selection screen — shows owned/free and locked tracks.
    /// </summary>
    public interface ITrackSelectionScreen : IScreenController
    {
        event Action<TrackDefinition> OnTrackSelected;
        event Action<TrackDefinition> OnTrackUnlockRequested;
        event Action OnBackPressed;

        void PopulateTracks(IReadOnlyList<TrackDefinition> owned, IReadOnlyList<TrackDefinition> locked);
        void SetSelectedTrack(TrackDefinition track);
    }

    /// <summary>
    /// The track panel — the second hosted screen of the menu hub. Same carousel grammar
    /// as the garage's car panel: prev/next arrows cycle a focus track, the centre slot is
    /// the hero, and focusing a raceable track makes it the session track immediately.
    /// Focusing an unraceable circuit — locked, or no scene of its own — is display-only:
    /// its stats render, Confirm goes dark, the session track is unchanged.
    ///
    /// The screen owns widgets and events only; cycle order, availability and navigation
    /// live in the scene controller.
    /// </summary>
    public interface ITrackPanelScreen : ITrackSelectionScreen
    {
        event Action OnPrevTrackPressed;
        event Action OnNextTrackPressed;

        /// <summary>Confirm was pressed with a raceable session track expected.</summary>
        event Action OnConfirmPressed;

        /// <summary>
        /// Renders <paramref name="track"/> as the centre slot with its neighbours flanking.
        /// <paramref name="isSelected"/> is true when the flow accepted the track as the
        /// session track; <paramref name="isAvailable"/> is false for circuits with no
        /// scene, which render but cannot be raced.
        /// </summary>
        void ShowFocusedTrack(TrackDefinition track, bool isSelected, bool isAvailable);
    }

    /// <summary>
    /// Wing setup screen — High Downforce vs Low Downforce with track-specific recommendation.
    /// </summary>
    public interface IWingSetupScreen : IScreenController
    {
        event Action<WingType> OnWingSelected;
        event Action OnBackPressed;
        event Action OnContinuePressed;

        void Setup(CarDefinition car, TrackDefinition track, WingType currentWing);
    }

    /// <summary>
    /// Mode selection screen — Qualifying or Race after wing setup.
    /// </summary>
    public interface IModeSelectionScreen : IScreenController
    {
        event Action OnQualifyingPressed;
        event Action OnRacePressed;
        event Action OnBackPressed;

        void Setup(CarDefinition car, TrackDefinition track, WingType wing);
    }

    /// <summary>
    /// Qualifying screen — unlimited laps, ghost car, sector timing, racing line.
    /// </summary>
    public interface IQualifyingScreen : IScreenController
    {
        event Action OnBackPressed;
        event Action OnRacePressed;
        event Action OnRestartPressed;

        void SetGhostData(string ghostJson);
        void UpdateLapTime(float currentLapTime, float bestLapTime);
        void UpdateSectorTimes(float[] sectorTimes, SectorColor[] sectorColors);
        void SetTrackInfo(TrackDefinition track);

        /// <summary>
        /// Puts the screen in its driving state: live lap clock, no session options offered.
        /// This is the first-attempt state — the player is out on track alone, so the screen
        /// offers nothing to do until a lap is actually recorded.
        /// </summary>
        void ShowDrivingState();

        /// <summary>
        /// Puts the screen in its results state: the recorded lap time and the three session
        /// options. Reached once the flow has accepted a qualifying lap.
        /// </summary>
        void ShowResultsState(float lapTime);
    }

    /// <summary>
    /// Race screen — HUD, position, lap counter, sector times.
    /// </summary>
    public interface IRaceScreen : IScreenController
    {
        event Action OnPausePressed;

        void SetRaceInfo(TrackDefinition track, int totalLaps, int gridPosition);
        void UpdatePosition(int position, int totalCars);
        void UpdateLap(int currentLap, int totalLaps);
        void UpdateLapTime(float currentLapTime, float bestLapTime);
        void UpdateSectorTimes(float[] sectorTimes, SectorColor[] sectorColors);
        void ShowFinishPosition(int position, int pointsEarned);
    }

    /// <summary>
    /// Results screen — post-race summary, points earned, unlocks available.
    /// </summary>
    public interface IResultsScreen : IScreenController
    {
        event Action OnContinuePressed;
        event Action OnRetryPressed;
        event Action OnMainMenuPressed;

        void ShowResults(int finishPosition, int pointsEarned, int newTotalPoints,
            CarDefinition unlockedCar = null, TrackDefinition unlockedTrack = null);
    }

    /// <summary>
    /// Options screen — settings, controls, audio.
    /// </summary>
    public interface IOptionsScreen : IScreenController
    {
        event Action OnBackPressed;

        void LoadSettings(PlayerProfile profile);
    }
}