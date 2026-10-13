using System;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;
using F1.Progression;

namespace F1.UI
{
    /// <summary>
    /// The client-demo garage view: car header, stats panel, player profile, dual
    /// currency, prev/next arrows and Continue.
    ///
    /// Pure rendering. Every action leaves through the <see cref="GarageScreen"/> events
    /// and returns through the view methods, so replacing this class — and the prefab it
    /// sits on — with the real UI touches nothing above the screen layer. Every widget is
    /// null-guarded because scenes are also loaded raw by tests, where nothing is wired.
    /// </summary>
    [DisallowMultipleComponent]
    public class GarageScreenImpl : GarageScreen
    {
        /// <summary>One bar row of the stats panel.</summary>
        [Serializable]
        public struct StatRow
        {
            public TMP_Text Value;
            public Image Fill;
        }

        [Header("Car Panel")]
        [SerializeField] private TMP_Text _carNameText;
        [SerializeField] private TMP_Text _carGenText;
        [SerializeField] private TMP_Text _carStatusText;
        [SerializeField] private TMP_Text _engineValueText;

        [Header("Stat Rows — order: Power, Torque, Top Speed, 0-100, Weight")]
        [SerializeField] private StatRow[] _statRows;

        [Header("Currency")]
        [SerializeField] private TMP_Text _careerPointsText;
        [SerializeField] private TMP_Text _premiumText;

        [Header("Player Profile")]
        [SerializeField] private TMP_Text _playerNameText;
        [SerializeField] private TMP_Text _teamText;
        [SerializeField] private TMP_Text _levelText;
        [SerializeField] private Image _xpFill;

        [Header("Actions")]
        [SerializeField] private Button _continueButton;
        [SerializeField] private Button _prevButton;
        [SerializeField] private Button _nextButton;

        [Header("Scene Extras")]
        [SerializeField] private GarageDragLayer _dragLayer;
        [SerializeField] private CanvasGroup _lockedDimmer;

        private CarDefinition _focusedCar;
        private CarDefinition _chosenCar;

        private void Awake()
        {
            // Self-wired rather than UnityEvent-persisted: the buttons then work however
            // the prefab was produced, and the builder has no two-pass wiring step to get
            // wrong (the known inert-buttons trap).
            if (_continueButton != null) _continueButton.onClick.AddListener(HandleContinueClicked);
            if (_prevButton != null) _prevButton.onClick.AddListener(HandlePrevClicked);
            if (_nextButton != null) _nextButton.onClick.AddListener(HandleNextClicked);

            if (_dragLayer != null)
            {
                _dragLayer.DragStarted += TriggerDragStarted;
                _dragLayer.DragDelta += TriggerDragDelta;
                _dragLayer.DragEnded += TriggerDragEnded;
            }
        }

        private void OnDestroy()
        {
            if (_continueButton != null) _continueButton.onClick.RemoveListener(HandleContinueClicked);
            if (_prevButton != null) _prevButton.onClick.RemoveListener(HandlePrevClicked);
            if (_nextButton != null) _nextButton.onClick.RemoveListener(HandleNextClicked);

            if (_dragLayer != null)
            {
                _dragLayer.DragStarted -= TriggerDragStarted;
                _dragLayer.DragDelta -= TriggerDragDelta;
                _dragLayer.DragEnded -= TriggerDragEnded;
            }
        }

        private void HandleContinueClicked() => TriggerNextPressed();
        private void HandlePrevClicked() => TriggerPrevCarPressed();
        private void HandleNextClicked() => TriggerNextCarPressed();

        /// <summary>
        /// The career-points readout. Unlocks cost careerPoints, so this — not the premium
        /// figure — is what SetCurrency means here.
        /// </summary>
        public override void SetCurrency(int amount)
        {
            if (_careerPointsText != null)
                _careerPointsText.text = amount.ToString("N0");
        }

        public override void RefreshFromProfile()
        {
            var profile = PlayerProfileManager.Current;
            if (profile == null) return;

            SetCurrency(profile.careerPoints);
            if (_premiumText != null) _premiumText.text = profile.premiumCurrency.ToString("N0");
            if (_playerNameText != null) _playerNameText.text = profile.playerName;
            if (_teamText != null) _teamText.text = profile.teamName;
            if (_levelText != null) _levelText.text = $"LEVEL {profile.playerLevel}";
            if (_xpFill != null) _xpFill.fillAmount = profile.XpProgressToNextLevel();
        }

        public override void SetChosenCar(CarDefinition car)
        {
            _chosenCar = car;
            UpdateContinue();
        }

        public override void ShowFocusedCar(CarDefinition car, bool isSessionCar, bool isAvailable)
        {
            _focusedCar = car;
            if (car == null) return;

            if (_carNameText != null) _carNameText.text = car.DisplayName;
            if (_carGenText != null) _carGenText.text = $"GEN {car.Generation}";
            if (_carStatusText != null)
                _carStatusText.text = BuildStatus(car, isSessionCar, isAvailable);
            if (_engineValueText != null)
                _engineValueText.text = string.IsNullOrEmpty(car.EngineLabel) ? "—" : car.EngineLabel;

            RenderStats(car);
            if (_lockedDimmer != null) _lockedDimmer.alpha = isAvailable ? 1f : 0.55f;
            UpdateContinue();
        }

        private void RenderStats(CarDefinition car)
        {
            if (_statRows == null || _statRows.Length < 5) return;

            SetRow(_statRows[0], $"{car.PowerHP:0} HP",
                CarDisplayStats.Normalize01(CarDisplayStats.PowerHP, car.PowerHP));
            SetRow(_statRows[1], $"{car.TorqueNm:0} Nm",
                CarDisplayStats.Normalize01(CarDisplayStats.TorqueNm, car.TorqueNm));
            SetRow(_statRows[2], $"{car.TopSpeedKph:0} km/h",
                CarDisplayStats.Normalize01(CarDisplayStats.TopSpeedKph, car.TopSpeedKph));
            // Inverted: a faster 0-100 time is a fuller bar, like every other row.
            SetRow(_statRows[3], $"{car.Accel0100Seconds:0.0}s",
                1f - CarDisplayStats.Normalize01(CarDisplayStats.Accel0100Seconds, car.Accel0100Seconds));
            SetRow(_statRows[4], $"{car.WeightKg:0} kg",
                CarDisplayStats.Normalize01(CarDisplayStats.WeightKg, car.WeightKg));
        }

        private static void SetRow(in StatRow row, string value, float normalized)
        {
            if (row.Value != null) row.Value.text = value;
            if (row.Fill != null) row.Fill.fillAmount = normalized;
        }

        private void UpdateContinue()
        {
            if (_continueButton == null) return;
            bool chosenIsFocused = _chosenCar != null && _focusedCar != null
                && _chosenCar.CarId == _focusedCar.CarId;
            _continueButton.interactable = chosenIsFocused;
        }

        /// <summary>
        /// SELECTED outranks everything — it is the state the player acted to reach. A
        /// rental the player has to rent first shows its price, same as a points-locked
        /// car, because both are "not yours yet, here is what it costs".
        /// </summary>
        private static string BuildStatus(CarDefinition car, bool isSessionCar, bool isAvailable)
        {
            if (isSessionCar) return "SELECTED";

            var profile = PlayerProfileManager.Current;
            if (profile != null && profile.HasActiveRental(car.CarId)) return "RENTED (24h)";
            if (profile != null && profile.OwnsCar(car.CarId)) return "OWNED";
            if (car.IsRental) return $"RENTAL · ${car.RentalCost24h:0.00} / 24h";
            if (!isAvailable) return $"LOCKED · {car.UnlockCostPoints} pts";
            return "FREE";
        }
    }
}
