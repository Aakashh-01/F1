using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;

namespace F1.UI
{
    /// <summary>
    /// Concrete ResultsScreen — finish position, points, unlocks, continue/retry/main menu.
    /// </summary>
    public class ResultsScreenImpl : ResultsScreen
    {
        [Header("Results")]
        [SerializeField] private TMP_Text _finishPositionText;
        [SerializeField] private TMP_Text _pointsEarnedText;
        [SerializeField] private TMP_Text _newTotalPointsText;

        [Header("Unlocks")]
        [SerializeField] private GameObject _unlockPanel;
        [SerializeField] private TMP_Text _unlockTitleText;
        [SerializeField] private GameObject _carUnlockItem;
        [SerializeField] private GameObject _trackUnlockItem;

        [Header("Buttons")]
        [SerializeField] private Button _continueButton;
        [SerializeField] private Button _retryButton;
        [SerializeField] private Button _mainMenuButton;

        private GameFlowManager _flow;

        public override void Initialize(GameFlowManager flowManager)
        {
            _flow = flowManager;
        }

        public override void ShowResults(int finishPosition, int pointsEarned, int newTotalPoints,
            CarDefinition unlockedCar = null, TrackDefinition unlockedTrack = null)
        {
            if (_finishPositionText != null)
                _finishPositionText.text = $"Finished: P{finishPosition}";
            if (_pointsEarnedText != null)
                _pointsEarnedText.text = $"+{pointsEarned} pts";
            if (_newTotalPointsText != null)
                _newTotalPointsText.text = $"Total: {newTotalPoints} pts";

            if (_unlockPanel != null)
            {
                bool hasUnlocks = unlockedCar != null || unlockedTrack != null;
                _unlockPanel.SetActive(hasUnlocks);
                if (hasUnlocks && _unlockTitleText != null)
                    _unlockTitleText.text = "NEW UNLOCKS!";

                if (unlockedCar != null && _carUnlockItem != null)
                    _carUnlockItem.SetActive(true);
                else if (_carUnlockItem != null)
                    _carUnlockItem.SetActive(false);

                if (unlockedTrack != null && _trackUnlockItem != null)
                    _trackUnlockItem.SetActive(true);
                else if (_trackUnlockItem != null)
                    _trackUnlockItem.SetActive(false);
            }

            gameObject.SetActive(true);
        }

        public override void Show()
        {
            gameObject.SetActive(true);
        }

        public override void Hide()
        {
            gameObject.SetActive(false);
            if (_unlockPanel != null)
                _unlockPanel.SetActive(false);
        }

        public void OnContinueClicked() => TriggerContinuePressed();
        public void OnRetryClicked() => TriggerRetryPressed();
        public void OnMainMenuClicked() => TriggerMainMenuPressed();
    }
}
