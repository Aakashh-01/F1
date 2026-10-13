namespace F1.GameFlow
{
    /// <summary>
    /// Canonical flow scene names. Scene assets, <c>GameFlowManager</c>'s serialized
    /// scene fields, the editor scene builder and the tests all read from here so the
    /// four cannot drift apart — a scene renamed in one place and not the others is
    /// exactly the class of defect recorded as finding B1.
    /// </summary>
    public static class FlowSceneNames
    {
        /// <summary>Entry scene. Hosts the persistent flow root, branding and loading.</summary>
        public const string Loading = "00_LoadingScene";

        /// <summary>
        /// The lobby. This is the menu hub: the 3D garage with the car panel
        /// (<c>GameScreen.Lobby</c>) and the track panel (<c>GameScreen.TrackSelection</c>)
        /// hosted as its two screens, staged by <see cref="GarageHubView"/>. The displayed
        /// car and the displayed track each ARE the selection.
        /// </summary>
        public const string Lobby = "05_GarageScene";

        /// <summary>
        /// The garage scene's predecessor. Deleted from disk by the menu-hub rework (the
        /// hub scene's framing is authored in GarageBuilder now), kept as a constant for
        /// the build-list exclusion tests, on the pattern of <see cref="CarSelection"/>.
        /// </summary>
        public const string RetiredGarageLobby = "05_LobbyScene";

        /// <summary>
        /// Car selection. The scene no longer ships — it is gone from the build list and the
        /// file is deleted. Car selection lives inside <see cref="Lobby"/> as its only screen.
        ///
        /// The constant is kept because CarSelectionSceneController still references it, and
        /// because <c>GameScreen.CarSelection</c> must keep its integer value.
        /// </summary>
        public const string CarSelection = "10_CarSelectionScene";

        /// <summary>
        /// Track selection. Since the menu-hub rework this NAMES THE HUB SCENE: the track
        /// panel is 05_GarageScene's second hosted screen, so GoToTrackSelection resolves
        /// through SceneFlowService's same-scene guard into an in-place panel swap, and
        /// wing setup's BACK lands on the hub with the track panel showing.
        /// </summary>
        public const string TrackSelection = "05_GarageScene";

        /// <summary>
        /// Track selection's predecessor: its own scene. Deleted from disk by the
        /// menu-hub rework; kept as a constant for the build-list exclusion tests.
        /// </summary>
        public const string RetiredTrackSelection = "20_TrackSelectionScene";

        public const string WingSetup = "30_WingSetupScene";
        public const string PreRace = "40_PreRaceScene";
        public const string Race = "50_RaceScene";
        public const string Results = "60_ResultsScene";

        /// <summary>
        /// Reusable track content. Loaded additively and kept resident across flow
        /// transitions so pre-race and race resolve the same live track objects.
        /// </summary>
        public const string TrackContent = "Track_01";

        /// <summary>
        /// The pre-Phase-3 single-scene lobby. Retained on disk for reference but not in
        /// the build list; it is NOT <see cref="Lobby"/>. The active route is
        /// Loading -> Lobby -> ... .
        /// </summary>
        public const string LegacyLobby = "LobbyScene";
    }
}
