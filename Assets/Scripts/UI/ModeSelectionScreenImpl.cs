using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;

namespace F1.UI
{
    /// <summary>
    /// Concrete ModeSelectionScreen — Qualifying vs Race + Back.
    /// </summary>
    public class ModeSelectionScreenImpl : ModeSelectionScreen
    {
        [Header("UI")]
        [SerializeField] private TMP_Text _titleText;
        [SerializeField] private Button _qualifyingButton;
        [SerializeField] private Button _raceButton;
        [SerializeField] private Button _backButton;

        private GameFlowManager _flow;

        public override void Initialize(GameFlowManager flowManager)
        {
            _flow = flowManager;
        }

        public override void Setup(CarDefinition car, TrackDefinition track, WingType wing)
        {
            if (_titleText != null)
                _titleText.text = $"Mode Selection";
        }

        public override void Show()
        {
            gameObject.SetActive(true);
        }

        public override void Hide()
        {
            gameObject.SetActive(false);
        }

        public void OnQualifyingClicked()
        {
            TriggerQualifyingPressed();
        }

        public void OnRaceClicked()
        {
            TriggerRacePressed();
        }

        public void OnBackClicked()
        {
            TriggerBackPressed();
        }
    }
}
