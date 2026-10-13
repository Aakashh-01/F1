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
        /// Since the client-demo UI rework this is <c>05_GarageScene</c> — a scene rebuilt
        /// from the lobby's captured garage rig, hosting the garage screen where the
        /// displayed car IS the selection. The scene it replaced remains on disk as
        /// <see cref="RetiredGarageLobby"/> but is out of the build list and the route.
        /// </summary>
        public const string Lobby = "05_GarageScene";

        /// <summary>
        /// The garage scene's predecessor: the hand-authored lobby whose tile strip the
        /// garage screen replaced. Kept on disk — it is the only copy of the hand-tuned
        /// garage rig the builder captures from, and the rollback path — but not in the
        /// build list. Its garage rig source is <see cref="Lobby"/>'s scene, so nothing
        /// here may ever save over it.
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
