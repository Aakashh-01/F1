using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;

namespace F1.UI
{
    /// <summary>
    /// Concrete WingSetupScreen — High/Low Downforce toggle + Continue/Back.
    /// </summary>
    public class WingSetupScreenImpl : WingSetupScreen
    {
        [Header("UI")]
        [SerializeField] private TMP_Text _titleText;
        [SerializeField] private TMP_Text _recommendationText;
        [SerializeField] private Toggle _highDownforceToggle;
        [SerializeField] private Toggle _lowDownforceToggle;
        [SerializeField] private Button _continueButton;
        [SerializeField] private Button _backButton;

        private GameFlowManager _flow;
        private WingType _currentWing;

        public override void Initialize(GameFlowManager flowManager)
        {
            _flow = flowManager;
        }

        public override void Setup(CarDefinition car, TrackDefinition track, WingType currentWing)
        {
            _currentWing = currentWing;

            if (_titleText != null)
                _titleText.text = $"Wing Setup";

            if (_recommendationText != null && track != null)
            {
                var recommendation = GetWingRecommendation(track);
                _recommendationText.text = recommendation;
            }

            // SetIsOnWithoutNotify, not the isOn setter: this method is configuring the
            // view, not recording a user choice. The setter fires onValueChanged, which
            // routes into OnHighDownforceToggled -> _flow.SelectWing(). That callback
            // fires while the flow has not wired the screen up yet — a scene controller's
            // Start runs before the flow resolves this screen — so it dereferenced a null
            // flow, and it would also have silently applied a wing the player never chose.
            if (_highDownforceToggle != null)
                _highDownforceToggle.SetIsOnWithoutNotify(currentWing == WingType.HighDownforce);
            if (_lowDownforceToggle != null)
                _lowDownforceToggle.SetIsOnWithoutNotify(currentWing == WingType.LowDownforce);
        }

        private string GetWingRecommendation(TrackDefinition track)
        {
            if (track == null) return "Select a wing setup";
            if (track.TrackLengthMeters < 3000f)
                return "RECOMMENDED: High Downforce — technical track";
            return "RECOMMENDED: Low Downforce — high-speed track";
        }

        public override void Show()
        {
            gameObject.SetActive(true);
        }

        public override void Hide()
        {
            gameObject.SetActive(false);
        }

        public void OnContinueClicked()
        {
            TriggerContinuePressed();
        }

        public void OnBackClicked()
        {
            TriggerBackPressed();
        }

        public void OnHighDownforceToggled(bool isOn)
        {
            if (!isOn) return;
            ApplyWing(WingType.HighDownforce);
        }

        public void OnLowDownforceToggled(bool isOn)
        {
            if (!isOn) return;
            ApplyWing(WingType.LowDownforce);
        }

        private void ApplyWing(WingType wing)
        {
            _currentWing = wing;

            // The flow wires this screen in just after the scene loads. A handler can
            // fire before that (a prefab-serialized toggle callback, for example), so
            // never assume Initialize has already run.
            if (_flow == null)
            {
                Debug.LogWarning(
                    "[WingSetupScreen] Wing changed before the flow wired this screen; " +
                    "the selection was not applied to the session.");
                return;
            }

            _flow.SelectWing(wing);
            TriggerWingSelected(wing);
        }
    }
}
