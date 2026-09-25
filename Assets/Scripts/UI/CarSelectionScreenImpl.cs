using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using F1.GameData;
using F1.GameFlow;
using F1.Progression;

namespace F1.UI
{
    /// <summary>
    /// Concrete CarSelectionScreen — 3 columns (Owned / Rentable / Locked) with car cards.
    /// </summary>
    public class CarSelectionScreenImpl : CarSelectionScreen
    {
        [Header("Columns")]
        [SerializeField] private RectTransform _ownedColumn;
        [SerializeField] private RectTransform _rentableColumn;
        [SerializeField] private RectTransform _lockedColumn;

        [Header("Prefabs")]
        [SerializeField] private CarCardImpl _carCardPrefab;

        private GameFlowManager _flow;
        private readonly List<CarCardImpl> _allCards = new();

        public override void Initialize(GameFlowManager flowManager)
        {
            _flow = flowManager;
            RefreshCarLists();
        }

        public override void Show()
        {
            gameObject.SetActive(true);
        }

        public override void Hide()
        {
            gameObject.SetActive(false);
        }

        public override void PopulateCars(IReadOnlyList<CarDefinition> owned,
            IReadOnlyList<CarDefinition> rentable, IReadOnlyList<CarDefinition> locked)
        {
            ClearCards();

            foreach (var car in owned)
                AddCard(car, _ownedColumn, OnCarOwnedClicked);
            foreach (var car in rentable)
                AddCard(car, _rentableColumn, OnCarRentableClicked);
            foreach (var car in locked)
                AddCard(car, _lockedColumn, OnCarLockedClicked);
        }

        public override void SetSelectedCar(CarDefinition car)
        {
            foreach (var card in _allCards)
                card.SetSelected(car.CarId);
        }

        private void RefreshCarLists()
        {
            var owned = ProgressionRegistry.GetOwnedCars();
            var rentable = ProgressionRegistry.GetAvailableCars();
            var locked = ProgressionRegistry.GetLockedCars();
            PopulateCars(owned, rentable, locked);
        }

        private void AddCard(CarDefinition car, RectTransform column, System.Action<CarDefinition> onClick)
        {
            if (_carCardPrefab == null || column == null) return;
            var card = Instantiate(_carCardPrefab, column);
            card.SetCar(car);
            card.OnCardClicked += onClick;
            _allCards.Add(card);
        }

        private void ClearCards()
        {
            foreach (var card in _allCards)
                if (card != null) Destroy(card.gameObject);
            _allCards.Clear();
        }

        private void OnCarOwnedClicked(CarDefinition car)
        {
            _flow.SelectCar(car);
            SetSelectedCar(car);
            TriggerCarSelected(car);
        }

        private void OnCarRentableClicked(CarDefinition car)
        {
            if (ProgressionRegistry.TryRentCar(car.CarId))
            {
                _flow.SelectCar(car);
                SetSelectedCar(car);
                TriggerCarRented(car);
            }
        }

        private void OnCarLockedClicked(CarDefinition car)
        {
            if (ProgressionRegistry.TryUnlockCar(car.CarId))
            {
                TriggerCarUnlockRequested(car);
                RefreshCarLists();
            }
        }

        private void OnDestroy()
        {
            ClearCards();
        }
    }
}
