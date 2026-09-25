using UnityEngine;
using UnityEngine.SceneManagement;

namespace F1.GameFlow
{
    /// <summary>
    /// Creates the single persistent flow root. Place one instance in the entry scene
    /// (LobbyScene today, <c>00_LoadingScene</c> from Phase 3) and nothing else needs to
    /// change when the entry scene is replaced.
    ///
    /// The root owns:
    ///   - <see cref="GameFlowManager"/> — session state and navigation.
    ///   - <see cref="SceneFlowService"/> — every scene load in the game.
    ///   - <see cref="EventSystem"/> — created only if the entry scene has none, so a
    ///     player build can never end up with zero or two event systems.
    /// </summary>
    [DisallowMultipleComponent]
    public class FlowBootstrap : MonoBehaviour
    {
        public const string FlowRootName = "[FlowRoot]";

        [Header("Bootstrap Options")]
        [Tooltip("Create an EventSystem if the entry scene does not already have one.")]
        [SerializeField] private bool _ensureEventSystem = true;

        [Tooltip("Abort and log an error if a scene load fails, instead of continuing with a broken state.")]
        [SerializeField] private bool _logLoadFailures = true;

        private static FlowBootstrap _instance;

        /// <summary>The persistent root, once <see cref="EnsureRoot"/> has run.</summary>
        public static FlowRoot Root { get; private set; }

        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                // A second bootstrap in an additively loaded scene is a bug, not a feature.
                Debug.LogWarning(
                    "[FlowBootstrap] A bootstrap already exists. Destroying the duplicate.", this);
                Destroy(gameObject);
                return;
            }

            _instance = this;
            EnsureRootOn(gameObject);
        }

        private void OnDestroy()
        {
            var flow = GameFlowManager.Instance;
            if (flow?.SceneFlow != null)
                flow.SceneFlow.OnSceneLoadFailed -= OnLoadFailed;

            if (_instance == this)
                _instance = null;
        }

        /// <summary>
        /// Creates the persistent root if it is not already running, and returns it.
        /// Safe to call from a test, an editor tool, or the entry scene.
        /// </summary>
        public static FlowRoot EnsureRoot()
        {
            if (Root == null)
                EnsureRootOn(null);
            return Root;
        }

        /// <summary>
        /// <paramref name="preferredHost"/> is the GameObject this bootstrap sits on, if
        /// any. Unity does not guarantee the Awake order of two components on the same
        /// object, so this component may run before GameFlowManager.Awake has set
        /// <c>Instance</c> — but the component itself is already present and can be
        /// adopted, which is why the search starts from the host object.
        /// </summary>
        private static void EnsureRootOn(GameObject preferredHost)
        {
            if (Root != null)
                return;

            var manager = preferredHost != null
                ? preferredHost.GetComponent<GameFlowManager>()
                : null;

            if (manager == null)
                manager = FindAnyObjectByType<GameFlowManager>(FindObjectsInactive.Include);

            if (manager == null)
                manager = GameFlowManager.Instance;

            if (manager != null)
            {
                // An existing manager (legacy scene wiring) becomes the root.
                Root = manager.GetComponent<FlowRoot>();
                if (Root == null)
                    Root = manager.gameObject.AddComponent<FlowRoot>();
                return;
            }

            var go = new GameObject(FlowRootName);
            Root = go.AddComponent<FlowRoot>();
            go.AddComponent<GameFlowManager>();
        }

        private void Start()
        {
            if (_ensureEventSystem)
                EnsureEventSystem();

            if (!_logLoadFailures)
                return;

            var flow = GameFlowManager.Instance;
            if (flow?.SceneFlow == null)
            {
                Debug.LogError("[FlowBootstrap] No SceneFlowService to observe.", this);
                return;
            }

            flow.SceneFlow.OnSceneLoadFailed += OnLoadFailed;
        }

        private void OnLoadFailed(SceneLoadResult result)
        {
            Debug.LogError($"[FlowBootstrap] Scene load failed — {result}", this);
        }

        private void EnsureEventSystem()
        {
            if (FindAnyObjectByType<UnityEngine.EventSystems.EventSystem>() != null)
                return;

            var go = new GameObject("EventSystem",
                typeof(UnityEngine.EventSystems.EventSystem),
                typeof(UnityEngine.EventSystems.StandaloneInputModule));
            DontDestroyOnLoad(go);
        }
    }
}
