using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;

namespace F1.UI
{
    /// <summary>
    /// Concrete QualifyingScreen — lap time, sector times, ghost toggle, back/race/restart.
    /// </summary>
    public class QualifyingScreenImpl : QualifyingScreen
    {
        [Header("Track Info")]
        [SerializeField] private TMP_Text _trackNameText;

        [Header("Timing")]
        [SerializeField] private TMP_Text _currentLapTimeText;
        [SerializeField] private TMP_Text _bestLapText;
        [SerializeField] private TMP_Text _sector1Text;
        [SerializeField] private TMP_Text _sector2Text;
        [SerializeField] private TMP_Text _sector3Text;

        [Header("Ghost")]
        [SerializeField] private Toggle _ghostToggle;

        [Header("Buttons")]
        [Tooltip("The three session options. They live inside the results panel, so they are " +
                 "hidden while the player is out driving and only become reachable once a lap " +
                 "has been recorded.")]
        [SerializeField] private Button _backButton;
        [SerializeField] private Button _raceButton;
        [SerializeField] private Button _restartButton;

        [Header("Results Panel")]
        [Tooltip("Shown in place of the live clock once a qualifying lap is recorded.")]
        [SerializeField] private GameObject _resultsPanel;
        [SerializeField] private TMP_Text _resultLapTimeText;

        private GameFlowManager _flow;
        private TrackDefinition _currentTrack;
        private bool _resultsShown;

        public override void Initialize(GameFlowManager flowManager)
        {
            _flow = flowManager;
        }

        public override void SetTrackInfo(TrackDefinition track)
        {
            _currentTrack = track;
            if (_trackNameText != null)
                _trackNameText.text = track?.DisplayName ?? "Unknown Track";
        }

        public override void UpdateLapTime(float currentLapTime, float bestLapTime)
        {
            if (_currentLapTimeText != null)
                _currentLapTimeText.text = FormatTime(currentLapTime);
            if (_bestLapText != null && bestLapTime > 0f)
                _bestLapText.text = $"Best: {FormatTime(bestLapTime)}";
        }

        public override void UpdateSectorTimes(float[] sectorTimes, SectorColor[] sectorColors)
        {
            if (sectorTimes == null || sectorTimes.Length < 3) return;
            if (_sector1Text != null)
                _sector1Text.text = FormatTime(sectorTimes[0]);
            if (_sector2Text != null)
                _sector2Text.text = FormatTime(sectorTimes[1]);
            if (_sector3Text != null)
                _sector3Text.text = FormatTime(sectorTimes[2]);
        }

        public override void SetGhostData(string ghostJson)
        {
            // Ghost data consumed by track ghost system
        }

        public override void Show()
        {
            gameObject.SetActive(true);
            ApplyState();
        }

        public override void Hide()
        {
            gameObject.SetActive(false);
        }

        /// <summary>
        /// Out on track: the clock runs and the session options are not on screen. The
        /// player is alone on the first attempt, so there is nothing to decide yet.
        /// </summary>
        public override void ShowDrivingState()
        {
            _resultsShown = false;
            ApplyState();
        }

        /// <summary>
        /// Lap recorded: show the time and the three options. The lap shown is the one the
        /// flow accepted, so the number on screen is the same number that gates the race —
        /// a lap the session rejected never reaches here.
        /// </summary>
        public override void ShowResultsState(float lapTime)
        {
            _resultsShown = true;
            if (_resultLapTimeText != null && lapTime > 0f)
                _resultLapTimeText.text = FormatTime(lapTime);
            ApplyState();
        }

        /// <summary>
        /// Applies whichever state is current. Both entry points funnel through here so the
        /// screen cannot end up showing a live clock behind a results panel, which is the
        /// shape a "just set a field" implementation would leave behind.
        /// </summary>
        private void ApplyState()
        {
            if (_resultsPanel != null)
                _resultsPanel.SetActive(_resultsShown);

            // The live readouts belong to the driving state. Hiding them behind the panel
            // as well would leave two lap times on screen at once.
            if (_currentLapTimeText != null)
                _currentLapTimeText.gameObject.SetActive(!_resultsShown);
            if (_bestLapText != null)
                _bestLapText.gameObject.SetActive(!_resultsShown);
        }

        public void OnBackClicked() => TriggerBackPressed();
        public void OnRaceClicked() => TriggerRacePressed();
        public void OnRestartClicked() => TriggerRestartPressed();
        public void OnGhostToggled(bool enabled) { }

        private string FormatTime(float s)
        {
            int m = Mathf.FloorToInt(s / 60f);
            int sec = Mathf.FloorToInt(s % 60f);
            int ms = Mathf.FloorToInt((s % 1f) * 1000f);
            return $"{m}:{sec:D2}.{ms:D3}";
        }
    }
}
