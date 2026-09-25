using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;

namespace F1.UI
{
    /// <summary>
    /// Individual car card in CarSelectionScreen.
    /// Shows name, generation, cost, and click handler.
    /// </summary>
    public class CarCardImpl : MonoBehaviour
    {
        [Header("UI")]
        [SerializeField] private TMP_Text _nameText;
        [SerializeField] private TMP_Text _genText;
        [SerializeField] private TMP_Text _costText;
        [SerializeField] private TMP_Text _statusText;
        [SerializeField] private Image _carImage;
        [SerializeField] private Button _cardButton;
        [SerializeField] private GameObject _selectedIndicator;

        public event System.Action<CarDefinition> OnCardClicked;

        private CarDefinition _car;

        public void SetCar(CarDefinition car)
        {
            _car = car;
            if (_carImage != null)
                _carImage.sprite = car?.Icon;
            if (_nameText != null)
                _nameText.text = car?.DisplayName ?? "Unknown";
            if (_genText != null)
                _genText.text = $"Gen {car?.Generation ?? 1}";
            if (_costText != null)
            {
                if (car == null) _costText.text = "";
                else if (car.IsRental) _costText.text = $"${car.RentalCost24h:0.00} / 24h";
                else if (car.UnlockCostPoints == 0) _costText.text = "FREE";
                else _costText.text = $"{car.UnlockCostPoints} pts";
            }
            if (_statusText != null)
                _statusText.text = BuildStatus(car);
        }

        /// <summary>
        /// Ownership has to be read from the profile, not inferred from the price. A
        /// rental variant also costs zero points, so treating "free" as "owned" labelled
        /// every rental as owned.
        /// </summary>
        private static string BuildStatus(CarDefinition car)
        {
            if (car == null) return "";

            var profile = F1.Progression.PlayerProfileManager.Current;
            if (profile != null && profile.OwnsCar(car.CarId))
                return "OWNED";

            if (profile != null && profile.HasActiveRental(car.CarId))
                return "RENTED (24h)";

            if (car.IsRental)
                return "RENTAL";

            if (car.UnlockCostPoints == 0)
                return "FREE";

            return $"LOCKED ({car.UnlockCostPoints} pts)";
        }

        public void SetSelected(string selectedCarId)
        {
            if (_selectedIndicator != null)
                _selectedIndicator.SetActive(_car != null && _car.CarId == selectedCarId);
        }

        private void HandleCardClicked()
        {
            if (_car != null)
                OnCardClicked?.Invoke(_car);
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
