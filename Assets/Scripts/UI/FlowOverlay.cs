using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameFlow;

namespace F1.UI
{
    /// <summary>
    /// The two full-screen moments that are not a scene of their own: the card shown while
    /// the flow moves between flow scenes, and the start countdown played once a session
    /// scene is ready and the car is on the grid.
    ///
    /// One object on <c>DontDestroyOnLoad</c>, bootstrapped at runtime from a prefab in
    /// Resources. Deliberately not a screen in the FlowSceneHost family: those are
    /// instantiated per scene by their host and torn down with it, whereas this has to
    /// outlive a transition and come back afterwards. The host also only knows how to show
    /// one of its own screens, and a transition sits between two scenes rather than inside
    /// either.
    ///
    /// Bootstrap order is handled by rebinding rather than by assuming: the flow root is
    /// created by the same <c>RuntimeInitializeOnLoadMethod</c> pass, so there is no
    /// guarantee the manager exists on the first frame this runs. <see cref="Bind"/> is
    /// retried each frame until it succeeds.
    /// </summary>
    [DisallowMultipleComponent]
    public class FlowOverlay : MonoBehaviour
    {
        public const string PrefabResourcePath = "FlowOverlay_Prefab";

        [Header("Transition card")]
        [SerializeField] private GameObject _transitionGroup;
        [SerializeField] private Image _transitionBackground;
        [SerializeField] private Image _transitionFill;
        [SerializeField] private TMP_Text _transitionStatus;

        [Header("Start countdown")]
        [SerializeField] private GameObject _countdownGroup;
        [SerializeField] private TMP_Text _countdownNumber;
        [SerializeField] private TMP_Text _countdownCaption;

        private static FlowOverlay _instance;
        private bool _bound;
        private Coroutine _countdown;
        private Action _countdownComplete;
        private Coroutine _gateRoutine;

        public static FlowOverlay Instance
        {
            get
            {
                if (_instance == null)
                {
                    var prefab = Resources.Load<GameObject>(PrefabResourcePath);
                    if (prefab == null)
                    {
                        Debug.LogError(
                            $"[FlowOverlay] No prefab at Resources/{PrefabResourcePath}. Run " +
                            "Tools/UI/Build Flow Overlay, or the transition and countdown " +
                            "simply never appear.");
                        return null;
                    }
                    var go = Instantiate(prefab);
                    go.name = "FlowOverlay";
                    DontDestroyOnLoad(go);
                    _instance = go.GetComponent<FlowOverlay>();
                }
                return _instance;
            }
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Bootstrap() => _ = Instance;

        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                Destroy(gameObject);
                return;
            }
            _instance = this;
            DontDestroyOnLoad(gameObject);
            HideTransition();
            HideCountdown();
        }

        private void Update()
        {
            if (!_bound) Bind();
        }

        private void OnDestroy()
        {
            if (_instance == this) _instance = null;
        }

        /// <summary>
        /// Subscribes to the flow service, retrying until the manager exists. The service is
        /// raised on <c>GameFlowManager</c>, which the same bootstrap pass may not have
        /// created yet, so a null here is expected on the first frames rather than an error.
        /// </summary>
        private void Bind()
        {
            var manager = GameFlowManager.Instance;
            if (manager == null || manager.SceneFlow == null) return;

            var flow = manager.SceneFlow;
            flow.OnFlowSceneLoadStarted += OnFlowLoadStarted;
            flow.OnProgressChanged += OnProgress;
            flow.OnFlowSceneChanged += OnFlowSceneDone;
            flow.OnSceneLoadFailed += OnFlowSceneDone;
            _bound = true;
        }

        private void OnFlowLoadStarted(string sceneName)
        {
            // Only the two session transitions get a loading card. Showing it on EVERY flow
            // scene change put one between the branding screen and the lobby as well, and
            // the loading scene is already a loading screen — the result was two screens in
            // a row, the second of them a flash lasting under a second before the lobby
            // appeared. The card is for the two transitions that are not already a screen:
            // wing -> pre-race and pre-race -> race.
            if (!IsSessionScene(sceneName)) return;

            if (_transitionStatus != null)
                _transitionStatus.text = sceneName.Replace("_", " ");

            ShowTransition();
        }

        private static bool IsSessionScene(string sceneName)
        {
            if (string.IsNullOrEmpty(sceneName)) return false;
            return sceneName == FlowSceneNames.PreRace || sceneName == FlowSceneNames.Race;
        }

        private void OnProgress(float value)
        {
            if (_transitionFill != null)
                _transitionFill.fillAmount = Mathf.Clamp01(value);
        }

        private void OnFlowSceneDone(SceneLoadResult result)
        {
            HideTransition();
            // Only the two session scenes open with a lights-out beat. Starting one on every
            // transition would put a countdown in front of the lobby and the menus.
            if (IsCountdownScene(result.SceneName)) StartGateCountdown();
        }

        /// <summary>
        /// The flow scenes that open on a countdown. Read from the flow's own names so a
        /// rename cannot silently drop the countdown.
        /// </summary>
        private static bool IsCountdownScene(string sceneName)
        {
            if (string.IsNullOrEmpty(sceneName)) return false;
            if (sceneName == FlowSceneNames.PreRace) return true;
            if (sceneName == FlowSceneNames.Race) return true;
            return false;
        }

        private void StartGateCountdown()
        {
            if (_gateRoutine != null) StopCoroutine(_gateRoutine);
            _gateRoutine = StartCoroutine(StartGateRoutine());
        }

        /// <summary>Seconds on the clock. Long enough to read, short enough not to bore.</summary>
        private const float CountdownSeconds = 3f;

        // --- Transition card ---

        private void ShowTransition()
        {
            if (_transitionFill != null) _transitionFill.fillAmount = 0f;
            SetActive(_transitionGroup, true);
        }

        public void HideTransition() => SetActive(_transitionGroup, false);

        // --- Driving the start gate ---

        /// <summary>
        /// Plays the countdown for whichever session scene has just become live.
        ///
        /// Driven from the flow rather than the other way round: F1.GameFlow cannot see
        /// Assembly-CSharp, so a controller cannot start this. It publishes
        /// ISessionStartGate instead and this finds whoever implements it.
        ///
        /// Polls rather than subscribing because the gate appears partway through the
        /// scene's own bring-up coroutine — it does not exist when the scene loads. Polling
        /// is cheap, bounded, and stops on its own the moment the gate reports ready.
        /// </summary>
        private IEnumerator StartGateRoutine()
        {
            const float timeout = 30f;
            float waited = 0f;

            while (waited < timeout)
            {
                var gate = FindStartGate();
                if (gate != null)
                {
                    string caption = CaptionFor(gate);
                    gate.SetStartGateLocked(true);
                    PlayCountdown(caption, CountdownSeconds, () => gate.SetStartGateLocked(false));
                    yield break;
                }

                waited += Time.unscaledDeltaTime;
                yield return null;
            }

            Debug.LogWarning("[FlowOverlay] No session start gate became ready within " +
                             $"{timeout:F0}s; skipping the countdown and releasing the field.");
            ReleaseAllGates();
        }

        /// <summary>
        /// Finds the gate that is actually live.
        ///
        /// Both <c>PreRaceSceneController</c> and <c>RaceSceneController</c> implement
        /// <see cref="ISessionStartGate"/>, and the flow loads the session scene additively
        /// BEFORE unloading the one it is leaving. There is therefore a real window in which
        /// two gates are alive at once, and this used to return whichever came first in an
        /// explicitly unordered sweep. When that was the outgoing pre-race controller, the
        /// countdown released THAT scene's car and the race scene stayed locked at the line
        /// for the rest of the session — a race you could not drive away from, with no error
        /// anywhere to explain it.
        ///
        /// Only a gate that reports itself ready is returned, so the live scene wins by
        /// construction: the scene that has finished bringing up says so, and the one still
        /// waiting to be set up does not. The caller polls, so returning nothing yet is a
        /// normal outcome rather than a failure.
        /// </summary>
        private static ISessionStartGate FindStartGate()
        {
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include,
                                                                FindObjectsSortMode.None))
            {
                if (mb is ISessionStartGate gate && gate.IsSessionReady) return gate;
            }
            return null;
        }

        /// <summary>
        /// Releases every gate that is holding a car, whatever scene it belongs to.
        ///
        /// This is the backstop for the one failure the countdown cannot fix: a scene that
        /// locks itself on the way up (<c>RaceSceneController</c> locks the whole field in its
        /// own bring-up, before this overlay ever polls) is only ever unlocked by the overlay
        /// finding it. If the poll times out, that car is held at the line forever and
        /// <c>InputLocked</c> zeroes the drivetrain every step, so the field is not braked, it
        /// is inert. Log-and-give-up turned a recoverable wait into a dead session; unlocking
        /// costs nothing and is the difference between a late start and no start.
        /// </summary>
        private static void ReleaseAllGates()
        {
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include,
                                                                FindObjectsSortMode.None))
            {
                if (mb is ISessionStartGate gate) gate.SetStartGateLocked(false);
            }
        }

        private static string CaptionFor(ISessionStartGate gate) =>
            gate is RaceSceneController ? "LIGHTS OUT" : "GET READY";

        // --- Start countdown ---

        /// <summary>
        /// Plays a 3-2-1 countdown, then GO, and calls <paramref name="onComplete"/> on the
        /// frame the count reaches zero.
        ///
        /// Uses unscaled time on purpose. This runs immediately after a scene load, and if it
        /// inherited a paused or time-scaled state the car would be released at a moment
        /// decided by the timescale rather than by the countdown the player just watched.
        /// </summary>
        public void PlayCountdown(string caption, float seconds, Action onComplete)
        {
            // An interrupted countdown must still finish its own contract. The callback is
            // what releases a start gate, so dropping it on the floor when a second countdown
            // supersedes the first left that gate locked at the line forever — a car held by
            // InputLocked with the drivetrain zeroed is inert, not braked, and nothing in the
            // scene says why. Whatever a countdown was going to release, it releases now.
            if (_countdown != null)
            {
                StopCoroutine(_countdown);
                _countdown = null;
                var pending = _countdownComplete;
                _countdownComplete = null;
                pending?.Invoke();
            }

            _countdownComplete = onComplete;
            _countdown = StartCoroutine(CountdownRoutine(caption, seconds, onComplete));
        }

        private IEnumerator CountdownRoutine(string caption, float seconds, Action onComplete)
        {
            SetActive(_countdownGroup, true);
            if (_countdownCaption != null) _countdownCaption.text = caption ?? "";

            int whole = Mathf.Max(1, Mathf.CeilToInt(seconds));
            for (int n = whole; n >= 1; n--)
            {
                if (_countdownNumber != null) _countdownNumber.text = n.ToString();
                yield return new WaitForSecondsRealtime(1f);
            }

            if (_countdownNumber != null) _countdownNumber.text = "GO";
            yield return new WaitForSecondsRealtime(0.6f);

            HideCountdown();
            _countdown = null;
            // Cleared before it runs so an interrupt landing on this frame cannot invoke it
            // a second time. The gate is released either way; the release is idempotent.
            _countdownComplete = null;
            onComplete?.Invoke();
        }

        public void HideCountdown() => SetActive(_countdownGroup, false);

        /// <summary>True while a countdown is playing, so callers do not stack two.</summary>
        public bool IsCountingDown => _countdown != null;

        private static void SetActive(GameObject go, bool active)
        {
            if (go != null && go.activeSelf != active) go.SetActive(active);
        }
    }
}
