using UnityEngine;

namespace F1.GameFlow
{
    /// <summary>How a <see cref="FlowSceneHost"/> decides which screen it hosts.</summary>
    public enum HostScreenMode
    {
        /// <summary>Always the screen named on the host itself.</summary>
        Fixed = 0,

        /// <summary>
        /// Derive it from the current session type: Qualifying while qualifying,
        /// Race while racing.
        ///
        /// Retired. The track scene used this while it doubled as both the qualifying and
        /// the race scene, but once each flow scene owned exactly one screen the mode
        /// became a source of ambiguity rather than a convenience: two hosts could claim
        /// the same screen and which one won depended on load order. Hosts now declare a
        /// single fixed screen. Kept because removing an enum member from a serialized
        /// field's type is a data migration, which does not belong in a scene refactor.
        /// </summary>
        QualifyingOrRace = 1
    }

    /// <summary>
    /// Declares that this scene owns one flow screen, and instantiates that screen if a
    /// prefab is assigned.
    ///
    /// This replaces the old model where <c>LobbyManager</c> instantiated every lobby
    /// screen up front and toggled their visibility. That model does not survive a
    /// multi-scene lobby: each flow scene now hosts exactly the screen it owns, so a new
    /// hub destination is a new scene plus a host, with no change to
    /// <see cref="GameFlowManager"/>.
    ///
    /// Registration happens in <c>Awake</c>, not <c>Start</c>. An additively loaded
    /// scene's <c>Awake</c> is guaranteed to have run by the time
    /// <c>SceneManager.LoadSceneAsync</c> reports completion, whereas <c>Start</c> is
    /// only guaranteed to have run by the next frame — too late for the flow manager,
    /// which resolves the screen controller the moment the load completes.
    /// </summary>
    [DisallowMultipleComponent]
    public class FlowSceneHost : MonoBehaviour
    {
        [Header("Hosted Screen")]
        [SerializeField] private HostScreenMode _screenMode = HostScreenMode.Fixed;

        [Tooltip("The screen this scene hosts. Ignored when Screen Mode is QualifyingOrRace.")]
        [SerializeField] private GameFlowManager.GameScreen _hostedScreen =
            GameFlowManager.GameScreen.Branding;

        [Header("Screen Prefab")]
        [Tooltip("Optional. When assigned, the screen is instantiated on Awake. " +
                 "Leave empty for scenes that host no UI yet (the track scene has no " +
                 "qualifying or race HUD prefab until Phase 6/7).")]
        [SerializeField] private GameObject _screenPrefab;

        [Tooltip("Parent for the instantiated screen. Defaults to this transform.")]
        [SerializeField] private Transform _screenParent;

        private bool _registered;

        /// <summary>The screen this scene hosts, resolved against the current session.</summary>
        public GameFlowManager.GameScreen HostedScreen
        {
            get
            {
                if (_screenMode == HostScreenMode.QualifyingOrRace)
                {
                    var session = GameFlowManager.Instance != null
                        ? GameFlowManager.Instance.CurrentSessionType
                        : SessionType.None;

                    return session == SessionType.Race
                        ? GameFlowManager.GameScreen.Race
                        : GameFlowManager.GameScreen.Qualifying;
                }

                return _hostedScreen;
            }
        }

        /// <summary>
        /// The live screen controller, or null when this scene hosts no UI. Null is a
        /// legitimate state, not an error — see <see cref="HasScreenInstance"/>.
        /// </summary>
        public ScreenController ScreenInstance { get; private set; }

        public bool HasScreenInstance => ScreenInstance != null;

        private void Awake()
        {
            EnsureScreenInstance();
            Register();
        }

        private void OnDestroy()
        {
            if (_registered && GameFlowManager.Instance != null)
                GameFlowManager.Instance.UnregisterSceneHost(this);

            _registered = false;
        }

        private void EnsureScreenInstance()
        {
            if (ScreenInstance != null || _screenPrefab == null)
                return;

            var parent = _screenParent != null ? _screenParent : transform;
            var instance = Instantiate(_screenPrefab, parent);
            instance.name = $"{_screenPrefab.name} (Hosted)";

            var controller = instance.GetComponent<ScreenController>();
            if (controller == null)
            {
                // Instantiating something that is not a screen would fail silently later
                // and cost a long debugging session. Fail here, loudly.
                Debug.LogError(
                    $"[FlowSceneHost] Prefab '{_screenPrefab.name}' on '{name}' has no " +
                    $"{nameof(ScreenController)} component. It cannot be hosted as a screen.", this);
                Destroy(instance);
                return;
            }

            ScreenInstance = controller;
        }

        private void Register()
        {
            var flow = GameFlowManager.Instance;
            if (flow == null)
            {
                Debug.LogError(
                    $"[FlowSceneHost] '{name}' cannot register: no GameFlowManager exists. " +
                    "Every flow scene requires the persistent flow root.", this);
                return;
            }

            flow.RegisterSceneHost(this);
            _registered = true;
        }

#if UNITY_EDITOR
        /// <summary>Editor-time configuration used by the scene builder.</summary>
        public void Configure(GameFlowManager.GameScreen screen, GameObject screenPrefab,
            HostScreenMode mode = HostScreenMode.Fixed, Transform parent = null)
        {
            _hostedScreen = screen;
            _screenPrefab = screenPrefab;
            _screenMode = mode;
            _screenParent = parent;
        }
#endif
    }
}
