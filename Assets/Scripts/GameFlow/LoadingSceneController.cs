using System.Collections;
using UnityEngine;
using F1.GameData;
using F1.Progression;

namespace F1.GameFlow
{
    /// <summary>
    /// Owns the behaviour of the loading scene: it shows branding, reports load
    /// progress, and advances to car selection once the session is genuinely ready.
    ///
    /// Per invariant 17 this scene's controller — not the screen — decides to navigate.
    /// The screen is a pure view.
    ///
    /// The GDD specifies no "press any key" gate on branding, so the scene advances on
    /// its own once the data and profile are ready. A minimum on-screen time is kept so
    /// the branding is actually readable rather than flashing for one frame.
    /// </summary>
    [DisallowMultipleComponent]
    public class LoadingSceneController : MonoBehaviour
    {
        [Header("Advance")]
        [Tooltip("The client requires no tap/click before car selection.")]
        [SerializeField] private bool _autoAdvance = true;

        [Tooltip("Minimum time branding stays on screen, so it is readable.")]
        [SerializeField] private float _minimumDisplaySeconds = 1.5f;

        [Header("Status")]
        [SerializeField] private string _statusLoadingData = "Loading game data";
        [SerializeField] private string _statusLoadingProfile = "Loading player profile";
        [SerializeField] private string _statusPreparing = "Preparing";
        [SerializeField] private string _statusComplete = "Ready";

        private GameFlowManager _flow;
        private BrandingScreen _branding;
        private float _progress;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError(
                    "[LoadingSceneController] No GameFlowManager. The loading scene must host " +
                    "the persistent flow root.", this);
                enabled = false;
                return;
            }

            _branding = GetComponentInChildren<BrandingScreen>(true);
            if (_branding == null)
                Debug.LogWarning(
                    "[LoadingSceneController] No BrandingScreen found; the loading scene " +
                    "will show no branding.", this);

            if (_flow.SceneFlow != null)
            {
                _flow.SceneFlow.OnProgressChanged += OnProgressChanged;
                _flow.SceneFlow.OnSceneLoadFailed += OnSceneLoadFailed;
            }

            StartCoroutine(LoadThenAdvance());
        }

        private void OnDestroy()
        {
            if (_flow?.SceneFlow == null) return;
            _flow.SceneFlow.OnProgressChanged -= OnProgressChanged;
            _flow.SceneFlow.OnSceneLoadFailed -= OnSceneLoadFailed;
        }

        private void OnProgressChanged(float value) => _progress = Mathf.Max(_progress, value);

        private void OnSceneLoadFailed(SceneLoadResult result)
        {
            // Stay put and report. Advancing past a failed load would drop the player
            // into a scene with no session data behind it.
            _branding?.SetLoadingError(result.Message);
        }

        private IEnumerator LoadThenAdvance()
        {
            // The registry initializes itself via RuntimeInitializeOnLoadMethod, but touch
            // it so the wait below is meaningful even if that ordering changes.
            GameDataRegistry.Initialize();
            _branding?.SetLoadingProgress(_progress, _statusLoadingData);
            yield return null;

            while (PlayerProfileManager.Current == null)
            {
                _branding?.SetLoadingProgress(_progress, _statusLoadingProfile);
                yield return null;
            }

            _branding?.SetLoadingProgress(_progress, _statusPreparing);

            // Hold the branding for a readable minimum.
            float deadline = Time.unscaledTime + Mathf.Max(0f, _minimumDisplaySeconds);
            while (Time.unscaledTime < deadline)
                yield return null;

            _branding?.SetLoadingProgress(1f, _statusComplete);
            // One more frame so the completed state is actually rendered before the
            // scene is torn down.
            yield return null;

            if (!_autoAdvance)
                yield break;

            // Invariant 1: no main menu between loading and car selection.
            _flow.GoToCarSelection();
        }
    }
}
