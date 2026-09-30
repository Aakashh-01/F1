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
        /// The lobby. This is the 3D garage: the car stands in it and does not move.
        ///
        /// It hosts car selection directly — the tile strip and Next are the first thing on
        /// screen, with no hub panel and no Start Race button in front of them. It is
        /// deliberately NOT a separate menu screen in front of the garage, and there is no
        /// "click Car Selection to navigate" step between loading and it.
        ///
        /// The hub that used to sit here (currency, settings, task list, Start Race) was
        /// removed 2026-09-30; it was a deliberate reversal of the original "no main menu
        /// between loading and car selection" requirement, decided 2026-09-26, and has since
        /// been reversed back.
        /// </summary>
        public const string Lobby = "05_LobbyScene";

        /// <summary>
        /// Car selection. The scene no longer ships — it is gone from the build list and the
        /// file is deleted. Car selection lives inside <see cref="Lobby"/> as its only screen.
        ///
        /// The constant is kept because CarSelectionSceneController still references it, and
        /// because <c>GameScreen.CarSelection</c> must keep its integer value.
        /// </summary>
        public const string CarSelection = "10_CarSelectionScene";

        public const string TrackSelection = "20_TrackSelectionScene";
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
