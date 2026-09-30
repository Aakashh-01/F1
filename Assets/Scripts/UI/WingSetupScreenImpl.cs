using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;
using F1.Lobby;

namespace F1.UI
{
    /// <summary>
    /// Concrete WingSetupScreen — two wing tiles, the track's recommendation, and
    /// Continue/Back.
    ///
    /// The tiles are a control, not a readout. Pressing one both records the choice and
    /// re-poses the 3D wing behind them, so the screen answers "what does this actually do"
    /// by showing it rather than by printing a number. That is the reason the screen is
    /// built over the garage at all; a plain 2D chooser would not need the car.
    /// </summary>
    public class WingSetupScreenImpl : WingSetupScreen
    {
        [Header("UI")]
        [SerializeField] private TMP_Text _titleText;
        [SerializeField] private TMP_Text _recommendationText;
        [Tooltip("Strip along the bottom that holds the two wing tiles.")]
        [SerializeField] private RectTransform _tileStrip;
        [Tooltip("The tile prefab instantiated once per WingType.")]
        [SerializeField] private WingCardImpl _wingCardPrefab;
        [SerializeField] private Button _continueButton;
        [SerializeField] private Button _backButton;

        private readonly System.Collections.Generic.List<WingCardImpl> _tiles = new();

        private GameFlowManager _flow;
        private CarDefinition _car;
        private WingType _currentWing;
        private WingAngleDriver _driver;
        private bool _driverLookedUp;

        public override void Initialize(GameFlowManager flowManager)
        {
            _flow = flowManager;

            // The flow and the scene controller reach this screen in an order that is not
            // fixed: the controller calls Setup in Start, and the flow's own wiring can land
            // either side of it. Building only when the strip is empty means whichever runs
            // first produces a correct screen and the other one tops it up.
            if (_tiles.Count == 0) PopulateTiles();
            RefreshTiles();
        }

        public override void Setup(CarDefinition car, TrackDefinition track, WingType currentWing)
        {
            _car = car;
            _currentWing = currentWing;

            if (_titleText != null)
                _titleText.text = "Wing Setup";

            if (_recommendationText != null && track != null)
                _recommendationText.text = GetWingRecommendation(track);

            // Always rebuilt, unlike Initialize's fill-if-empty: the tiles carry the chosen
            // car's artwork, so a car that changed since the last visit has to redraw them.
            PopulateTiles();
            RefreshTiles();

            // Pose the wing to the wing the session already has, without animating. This is
            // existing state being displayed rather than a choice being made, so it snaps —
            // the eased swing belongs to a tile press, where the motion is the feedback.
            PoseWingImmediate(currentWing);
        }

        private string GetWingRecommendation(TrackDefinition track)
        {
            if (track == null) return "Select a wing setup";
            if (track.TrackLengthMeters < 3000f)
                return "RECOMMENDED: High Downforce — technical track";
            return "RECOMMENDED: Low Downforce — high-speed track";
        }

        // --- Tiles ---

        private void PopulateTiles()
        {
            ClearTiles();

            if (_wingCardPrefab == null || _tileStrip == null)
            {
                Debug.LogError("[WingSetupScreen] No wing card prefab or tile strip; " +
                               "the screen will show no wing options.", this);
                return;
            }

            // High first. It is enum 0 and it is the recommendation on a technical circuit,
            // so the left-hand tile is the one a player is most likely to want.
            AddTile(WingType.HighDownforce);
            AddTile(WingType.LowDownforce);
        }

        private void AddTile(WingType wing)
        {
            var tile = Instantiate(_wingCardPrefab, _tileStrip);
            tile.gameObject.name = $"Tile_{wing}";
            // GetAeroForWing dereferences its receiver, so this has to go through _car's
            // null check rather than calling it directly. With no car the tile still builds
            // and shows its label; only the artwork is missing.
            tile.SetWing(wing, _car?.GetAeroForWing(wing));
            tile.OnCardClicked += OnTileClicked;
            _tiles.Add(tile);
        }

        private void OnTileClicked(WingType wing) => ApplyWing(wing);

        private void RefreshTiles()
        {
            foreach (var tile in _tiles)
            {
                if (tile != null) tile.SetSelected(_currentWing);
            }
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

        // --- The 3D wing ---

        /// <summary>
        /// Finds the driver on the car. Cached because it is only ever looked up once and
        /// the car does not move between the screen's Setup and a tile press.
        ///
        /// Null-tolerant on purpose: without a driver the screen still works as a plain 2D
        /// chooser, which is what it has to fall back to if the 3D is ever backed out.
        /// </summary>
        private WingAngleDriver ResolveDriver()
        {
            if (_driverLookedUp) return _driver;
            _driver = FindFirstObjectByType<WingAngleDriver>();
            _driverLookedUp = true;
            return _driver;
        }

        private void PoseWingImmediate(WingType wing)
        {
            var driver = ResolveDriver();
            if (driver == null) return;

            if (wing == WingType.HighDownforce) driver.SetHighDownforceImmediate();
            else driver.SetLowDownforceImmediate();
        }

        private void ApplyWing(WingType wing)
        {
            _currentWing = wing;
            RefreshTiles();

            // Eased, not immediate: the swing between the two setups is the entire point of
            // showing the car, and a snap throws away the only feedback the screen gives.
            var driver = ResolveDriver();
            if (driver != null)
            {
                if (wing == WingType.HighDownforce) driver.SetHighDownforce();
                else driver.SetLowDownforce();
            }

            // The flow wires this screen in just after the scene loads. A handler can fire
            // before that (a prefab-serialized callback, for example), so never assume
            // Initialize has already run.
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

        private void OnDestroy()
        {
            ClearTiles();
        }
    }
}
