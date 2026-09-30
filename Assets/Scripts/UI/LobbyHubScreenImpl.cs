using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;
using F1.Progression;

namespace F1.UI
{
    /// <summary>
    /// The lobby screen — the car selection tiles over the same 3D garage.
    ///
    /// The hub panel (currency, settings, tasks and a START RACE button) is gone. The lobby
    /// now opens straight onto the car tiles, so there is no panel to swap and nothing to
    /// press first. What is left is the tile strip, the currency readout and Next.
    ///
    /// This class only ever touches the selection panel's widgets. The garage, the car, the
    /// camera and the lighting belong to the scene, not to the screen, so the car never
    /// moves, blinks or re-loads when the player picks a different tile on screen.
    /// </summary>
    public class LobbyHubScreenImpl : LobbyScreen
    {
        [Header("Selection Widgets")]
        [SerializeField] private TextMeshProUGUI _currencyText;
        [SerializeField] private RectTransform _tileStrip;
        [SerializeField] private CarCardImpl _carCardPrefab;
        [SerializeField] private Button _nextButton;

        private readonly System.Collections.Generic.List<CarCardImpl> _tiles = new();
        private GameFlowManager _flow;
        private CarDefinition _chosen;

        public override void Initialize(GameFlowManager flowManager)
        {
            _flow = flowManager;
            PopulateTiles();
        }

        public override void Show()
        {
            gameObject.SetActive(true);
            // Re-entering the lobby after a race re-reads ownership and re-marks the
            // session car, so a rental bought mid-session shows as owned when they come back.
            RefreshTiles();
        }

        public override void Hide()
        {
            gameObject.SetActive(false);
        }

        public override void SetCurrency(int amount)
        {
            if (_currencyText != null)
                _currencyText.text = amount.ToString("N0");
        }

        /// <summary>
        /// Builds one tile per car into the strip, in the order owned, rentable, locked.
        ///
        /// The tier used to be expressed by which of three columns a card sat in. A
        /// horizontal strip has no columns, so the ordering carries it instead and each tile
        /// keeps its own status line - OWNED / RENTAL / LOCKED - which the card already
        /// renders. Nothing is lost, it is just read left to right now.
        /// </summary>
        private void PopulateTiles()
        {
            ClearTiles();
            if (_carCardPrefab == null || _tileStrip == null)
            {
                Debug.LogError("[Lobby] Hub screen has no car card prefab or tile strip; " +
                               "the selection state will be empty.", this);
                return;
            }

            AddTiles(ProgressionRegistry.GetOwnedCars());
            AddTiles(ProgressionRegistry.GetAvailableCars());
            AddTiles(ProgressionRegistry.GetLockedCars());
        }

        private void AddTiles(System.Collections.Generic.IReadOnlyList<CarDefinition> cars)
        {
            foreach (var car in cars)
            {
                var tile = Instantiate(_carCardPrefab, _tileStrip);
                tile.gameObject.name = $"Tile_{car.CarId}";
                tile.SetCar(car);
                tile.OnCardClicked += OnTileClicked;
                _tiles.Add(tile);
            }
        }

        private void OnTileClicked(CarDefinition car)
        {
            // Optimistically mark it so the tap feels responsive, then let the controller
            // push back whatever the flow accepted. A rejected tap therefore un-marks
            // itself rather than leaving a tile lit that the session will never use.
            _chosen = car;
            RefreshTiles();
            TriggerCarChosen(car);
        }

        /// <summary>
        /// Re-reads ownership and re-marks the chosen tile.
        ///
        /// Run every time the selection panel comes up rather than only at build time,
        /// because renting or unlocking a car changes which bucket it belongs in and the
        /// strip has to agree with the profile.
        /// </summary>
        private void RefreshTiles()
        {
            foreach (var tile in _tiles)
            {
                if (tile == null) continue;
                tile.SetSelected(_chosen?.CarId);
            }

            // Next stays dead until a car is chosen. It is also the only route onward, so
            // leaving it live with nothing selected would dead-end the player.
            if (_nextButton != null) _nextButton.interactable = _chosen != null;
        }

        private void ClearTiles()
        {
            foreach (var tile in _tiles)
            {
                if (tile == null) continue;
                tile.OnCardClicked -= OnTileClicked;
                Destroy(tile.gameObject);
            }
            _tiles.Clear();
        }

        /// <summary>
        /// Marks a car as chosen without a click, or clears the choice with null. Called
        /// back by the scene controller with the car the flow actually accepted, and on
        /// show so a returning player sees their session car already selected.
        /// </summary>
        public override void SetChosenCar(CarDefinition car)
        {
            _chosen = car;
            RefreshTiles();
        }

        /// <summary>
        /// Refreshes the currency readout from the profile. Called by the scene controller on
        /// show, so a race that earned or spent something is reflected on return.
        /// </summary>
        public override void RefreshFromProfile()
        {
            var profile = PlayerProfileManager.Current;
            if (profile != null) SetCurrency(profile.premiumCurrency);
        }

        // --- Button handlers, wired by the scene builder ---

        public void OnNextClicked() => TriggerNextPressed();

        private void OnDestroy()
        {
            ClearTiles();
        }
    }
}
