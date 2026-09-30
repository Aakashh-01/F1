using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;

namespace F1.UI
{
    /// <summary>
    /// One wing tile on the wing setup screen — High or Low downforce.
    ///
    /// Shaped like <see cref="CarCardImpl"/>: artwork, two lines of text, and a
    /// SelectedIndicator under that exact child name, so the selected-state pattern the car
    /// and track screens already use carries over instead of being reinvented here.
    ///
    /// The tile is a control, not a readout. The car behind it is the preview, so pressing a
    /// tile re-poses the 3D wing and the player watches the consequence rather than reading
    /// a number. That is why there is no downforceCoeff on the tile — the player is choosing
    /// a feel, and the wing moving is what tells them which one they chose.
    /// </summary>
    public class WingCardImpl : MonoBehaviour
    {
        [Header("UI")]
        [SerializeField] private TMP_Text _titleText;
        [SerializeField] private TMP_Text _blurbText;
        [SerializeField] private Image _wingImage;
        [SerializeField] private Button _cardButton;
        [SerializeField] private GameObject _selectedIndicator;

        public event System.Action<WingType> OnCardClicked;

        private WingType _wing;
        private bool _hasWing;

        /// <summary>
        /// The plain-English consequence of each setup. A coefficient would be the honest
        /// number and the useless one: nobody can feel 5.2 against 4.1, but everybody can
        /// feel "slower on the straights". The recommendation line on the screen is framed
        /// this way already, so the tiles agree with it.
        /// </summary>
        private static string TitleFor(WingType wing) => wing == WingType.HighDownforce
            ? "HIGH DOWNFORCE"
            : "LOW DOWNFORCE";

        private static string BlurbFor(WingType wing) => wing == WingType.HighDownforce
            ? "More grip, slower on the straights"
            : "Faster, less grip through corners";

        public void SetWing(WingType wing, WingAeroProfile profile)
        {
            _wing = wing;
            _hasWing = true;

            if (_titleText != null) _titleText.text = TitleFor(wing);
            if (_blurbText != null) _blurbText.text = BlurbFor(wing);

            // The sprite is optional by design — WingAeroProfile's own tooltip says a
            // profile with no artwork shows its label only — so a missing icon leaves the
            // panel empty rather than blanking the tile.
            if (_wingImage != null) _wingImage.sprite = profile != null ? profile.icon : null;
        }

        public void SetSelected(WingType selected)
        {
            if (_selectedIndicator == null) return;

            // Guarded on _hasWing, not on a wing value. WingType.HighDownforce is 0, so an
            // unconfigured tile would otherwise default to High and light up as the
            // selected one whenever the session happened to be on High.
            _selectedIndicator.SetActive(_hasWing && _wing == selected);
        }

        private void HandleCardClicked() => OnCardClicked?.Invoke(_wing);

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
