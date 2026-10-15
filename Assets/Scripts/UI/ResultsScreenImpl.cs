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
        [SerializeField] private TMP_Text _carUnlockText;
        [SerializeField] private TMP_Text _trackUnlockText;

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
            // Just the position, not "Finished: P3". The word sits in a static caption above this, and
            // keeping it out of the string is what lets the number be set large enough to read
            // across a room — at headline sizes the longer form wraps onto two lines in the
            // canvas box it is given, which is the failure a screenshot catches and an
            // assertion does not.
            if (_finishPositionText != null)
                _finishPositionText.text = $"P{finishPosition}";
            if (_pointsEarnedText != null)
                _pointsEarnedText.text = $"+{pointsEarned} pts";
            if (_newTotalPointsText != null)
                _newTotalPointsText.text = $"Total: {newTotalPoints} pts";

            if (_unlockPanel != null)
            {
                // Announced, not granted. GameFlowManager hands over the cheapest car and
                // track the player can now AFFORD; the points are still theirs and the unlock
                // is still a deliberate tap in the lobby. Calling this "NEW UNLOCKS!" would
                // tell the player they already own something they do not, and would be a lie
                // the very next click corrects.
                bool hasCar = unlockedCar != null;
                bool hasTrack = unlockedTrack != null;
                _unlockPanel.SetActive(hasCar || hasTrack);

                if (_unlockTitleText != null)
                    _unlockTitleText.text = "AVAILABLE TO UNLOCK";

                if (_carUnlockText != null)
                {
                    _carUnlockText.gameObject.SetActive(hasCar);
                    if (hasCar)
                        _carUnlockText.text =
                            $"{unlockedCar.DisplayName} — {unlockedCar.UnlockCostPoints:N0} pts";
                }

                if (_trackUnlockText != null)
                {
                    _trackUnlockText.gameObject.SetActive(hasTrack);
                    if (hasTrack)
                        _trackUnlockText.text =
                            $"{unlockedTrack.DisplayName} — {unlockedTrack.UnlockCostPoints:N0} pts";
                }
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
