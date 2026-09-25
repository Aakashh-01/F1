using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;

namespace F1.UI
{
    /// <summary>
    /// Concrete RaceScreen — HUD with position, lap counter, lap time, sector times, finish.
    /// </summary>
    public class RaceScreenImpl : RaceScreen
    {
        [Header("Track Info")]
        [SerializeField] private TMP_Text _trackNameText;
        [SerializeField] private TMP_Text _totalLapsText;

        [Header("Position")]
        [SerializeField] private TMP_Text _positionText;

        [Header("Lap")]
        [SerializeField] private TMP_Text _currentLapText;
        [SerializeField] private TMP_Text _lapTimeText;
        [SerializeField] private TMP_Text _bestLapText;

        [Header("Sectors")]
        [SerializeField] private TMP_Text _sector1Text;
        [SerializeField] private TMP_Text _sector2Text;
        [SerializeField] private TMP_Text _sector3Text;

        [Header("Finish Panel")]
        [SerializeField] private GameObject _finishPanel;
        [SerializeField] private TMP_Text _finishPositionText;
        [SerializeField] private TMP_Text _pointsEarnedText;

        [Header("Buttons")]
        [SerializeField] private Button _pauseButton;

        private GameFlowManager _flow;

        public override void Initialize(GameFlowManager flowManager)
        {
            _flow = flowManager;
        }

        public override void SetRaceInfo(TrackDefinition track, int totalLaps, int gridPosition)
        {
            if (_trackNameText != null)
                _trackNameText.text = track?.DisplayName ?? "Unknown Track";
            if (_totalLapsText != null)
                _totalLapsText.text = $"Laps: {totalLaps} | Grid: {gridPosition}";
        }

        public override void UpdatePosition(int position, int totalCars)
        {
            if (_positionText != null)
                _positionText.text = $"{position} / {totalCars}";
        }

        public override void UpdateLap(int currentLap, int totalLaps)
        {
            if (_currentLapText != null)
                _currentLapText.text = $"Lap {currentLap} / {totalLaps}";
        }

        public override void UpdateLapTime(float currentLapTime, float bestLapTime)
        {
            if (_lapTimeText != null)
                _lapTimeText.text = FormatTime(currentLapTime);
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

        public override void ShowFinishPosition(int position, int pointsEarned)
        {
            if (_finishPanel != null)
                _finishPanel.SetActive(true);
            if (_finishPositionText != null)
                _finishPositionText.text = $"Finished: P{position}";
            if (_pointsEarnedText != null)
                _pointsEarnedText.text = $"+{pointsEarned} pts";
        }

        public override void Show()
        {
            gameObject.SetActive(true);
        }

        public override void Hide()
        {
            gameObject.SetActive(false);
            if (_finishPanel != null)
                _finishPanel.SetActive(false);
        }

        public void OnPauseClicked() => TriggerPausePressed();

        private string FormatTime(float s)
        {
            int m = Mathf.FloorToInt(s / 60f);
            int sec = Mathf.FloorToInt(s % 60f);
            int ms = Mathf.FloorToInt((s % 1f) * 1000f);
            return $"{m}:{sec:D2}.{ms:D3}";
        }
    }
}
