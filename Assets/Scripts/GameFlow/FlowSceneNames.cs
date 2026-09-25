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
        /// The lobby hub. Per the client this is the first interactive screen — there is
        /// no main menu between loading and car selection.
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
        /// the build list; the active route is Loading -> CarSelection -> ... .
        /// </summary>
        public const string LegacyLobby = "LobbyScene";
    }
}
