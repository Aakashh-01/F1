using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameFlow;

namespace F1.UI
{
    /// <summary>
    /// Concrete BrandingScreen — the GDD's branding requirement (game/app name plus the
    /// "Developed by Pearl-Lemon" attribution) plus the loading progress indicator.
    ///
    /// The screen is a pure view. The decision to advance belongs to
    /// <see cref="LoadingSceneController"/>; this class never navigates on its own.
    /// </summary>
    public class BrandingScreenImpl : BrandingScreen
    {
        [Header("Branding")]
        [SerializeField] private TMP_Text _titleText;
        [SerializeField] private TMP_Text _subtitleText;
        [Tooltip("The GDD requires 'Developed by Pearl-Lemon' on the branding screen.")]
        [SerializeField] private TMP_Text _attributionText;

        [Header("Loading Progress")]
        [SerializeField] private GameObject _progressRoot;
        [SerializeField] private Image _progressFill;
        [SerializeField] private TMP_Text _progressText;
        [SerializeField] private TMP_Text _errorText;

        private static readonly Color NormalStatus = new Color(0.72f, 0.84f, 0.94f, 0.96f);
        private static readonly Color ErrorColor = new Color(1f, 0.42f, 0.42f, 1f);

        public override void Initialize(GameFlowManager flowManager)
        {
        }

        public override void Show()
        {
            gameObject.SetActive(true);
            SetVisible(_titleText, true);
            SetVisible(_subtitleText, true);
            SetVisible(_attributionText, true);
            if (_progressRoot != null) _progressRoot.SetActive(true);
        }

        public override void Hide()
        {
            gameObject.SetActive(false);
        }

        public override void SetLoadingProgress(float normalized, string status)
        {
            float clamped = Mathf.Clamp01(normalized);

            if (_progressFill != null)
                _progressFill.fillAmount = clamped;

            if (_progressText != null)
                _progressText.text = string.IsNullOrEmpty(status)
                    ? $"{Mathf.RoundToInt(clamped * 100f)}%"
                    : status;

            if (_progressText != null)
                _progressText.color = NormalStatus;

            if (_errorText != null)
                _errorText.text = "";
        }

        public override void SetLoadingError(string message)
        {
            if (_errorText != null)
            {
                _errorText.text = message;
                _errorText.color = ErrorColor;
            }

            if (_progressText != null)
                _progressText.color = ErrorColor;

            Debug.LogError($"[BrandingScreen] Loading failed: {message}");
        }

        private static void SetVisible(Component component, bool visible)
        {
            if (component != null)
                component.gameObject.SetActive(visible);
        }
    }
}
