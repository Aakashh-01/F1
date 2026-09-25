using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;

namespace F1.UI
{
    /// <summary>
    /// Individual track card in TrackSelectionScreen.
    /// Shows name, short code, unlock cost, and click handler.
    /// </summary>
    public class TrackCardImpl : MonoBehaviour
    {
        [Header("UI")]
        [SerializeField] private TMP_Text _nameText;
        [SerializeField] private TMP_Text _shortCodeText;
        [SerializeField] private TMP_Text _costText;
        [SerializeField] private TMP_Text _statusText;
        [SerializeField] private Image _trackImage;
        [SerializeField] private Button _cardButton;
        [SerializeField] private GameObject _selectedIndicator;

        [Header("Unavailable circuits")]
        [Tooltip("Shown instead of the cost line when the circuit has no scene yet.")]
        [SerializeField] private GameObject _comingSoonBadge;
        [SerializeField] private CanvasGroup _contentGroup;
        [SerializeField, Range(0f, 1f)] private float _unavailableAlpha = 0.45f;

        public event System.Action<TrackDefinition> OnCardClicked;

        private TrackDefinition _track;

        public void SetTrack(TrackDefinition track)
        {
            _track = track;

            if (_trackImage != null && track != null)
                _trackImage.sprite = track.Thumbnail != null ? track.Thumbnail : track.MinimapTexture;

            if (_nameText != null)
                _nameText.text = track?.DisplayName ?? "Unknown";
            if (_shortCodeText != null)
                _shortCodeText.text = track?.ShortCode ?? "???";

            // A circuit with no scene cannot be entered, so it reads differently from one
            // that exists but is still locked behind progression. "LOCKED" promises you can
            // unlock it; "COMING SOON" tells the truth about what is actually in the build.
            bool available = SceneAvailability.IsAvailable(track);

            if (_comingSoonBadge != null)
                _comingSoonBadge.SetActive(!available);

            if (_contentGroup != null)
                _contentGroup.alpha = available ? 1f : _unavailableAlpha;

            if (_cardButton != null)
                _cardButton.interactable = available;

            if (_costText != null)
            {
                if (track == null) _costText.text = "";
                else if (!available) _costText.text = "";
                else if (track.UnlockCostPoints == 0) _costText.text = "FREE";
                else _costText.text = $"{track.UnlockCostPoints} pts";
            }
            if (_statusText != null)
            {
                if (track == null) _statusText.text = "";
                else if (!available) _statusText.text = "COMING SOON";
                else if (track.UnlockCostPoints == 0) _statusText.text = "AVAILABLE";
                else _statusText.text = $"LOCKED ({track.UnlockCostPoints} pts)";
            }
        }

        public void SetSelected(string selectedTrackId)
        {
            if (_selectedIndicator != null)
                _selectedIndicator.SetActive(_track != null && _track.TrackId == selectedTrackId);
        }

        private void HandleCardClicked()
        {
            // Guard here as well as on the button: a card can be clicked programmatically
            // and an unavailable circuit must never reach the scene loader.
            if (_track != null && SceneAvailability.IsAvailable(_track))
                OnCardClicked?.Invoke(_track);
        }

        private void Awake()
        {
            if (_cardButton != null)
                _cardButton.onClick.AddListener(HandleCardClicked);
        }

        private void OnDestroy()
        {
            if (_cardButton != null)
                _cardButton.onClick.RemoveListener(HandleCardClicked);
        }
    }
}
