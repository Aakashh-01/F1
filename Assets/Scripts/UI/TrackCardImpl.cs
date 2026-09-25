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
        [SerializeField] private Button _cardButton;
        [SerializeField] private GameObject _selectedIndicator;

        public event System.Action<TrackDefinition> OnCardClicked;

        private TrackDefinition _track;

        public void SetTrack(TrackDefinition track)
        {
            _track = track;
            if (_nameText != null)
                _nameText.text = track?.DisplayName ?? "Unknown";
            if (_shortCodeText != null)
                _shortCodeText.text = track?.ShortCode ?? "???";
            if (_costText != null)
            {
                if (track == null) _costText.text = "";
                else if (track.UnlockCostPoints == 0) _costText.text = "FREE";
                else _costText.text = $"{track.UnlockCostPoints} pts";
            }
            if (_statusText != null)
            {
                if (track == null) _statusText.text = "";
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
            if (_track != null)
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
