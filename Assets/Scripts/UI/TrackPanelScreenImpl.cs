using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;

namespace F1.UI
{
    /// <summary>
    /// The track panel view: a three-slot carousel (left dimmed / centre hero / right
    /// dimmed) with prev/next arrows, BACK and CONFIRM, over the shared hub chrome.
    ///
    /// Pure rendering. The focus index, the cycle order and every navigation decision
    /// live in the scene controller; this class draws whatever
    /// <see cref="ShowFocusedTrack"/> pushes and raises events for whatever is pressed.
    /// Every widget is null-guarded because scenes are also loaded raw by tests.
    /// </summary>
    [DisallowMultipleComponent]
    public class TrackPanelScreenImpl : TrackPanelScreen
    {
        /// <summary>One visible carousel slot.</summary>
        [Serializable]
        public struct TrackSlot
        {
            public GameObject Root;
            public Image Art;
            public TMP_Text ShortCode;
            public TMP_Text Name;
            public TMP_Text Length;
            public TMP_Text Laps;
            public TMP_Text Difficulty;
            public GameObject SelectedBadge;
            public CanvasGroup Dim;
        }

        [Header("Header")]
        [SerializeField] private TMP_Text _titleText;

        [Header("Slots — order: left, centre, right")]
        [SerializeField] private TrackSlot[] _slots;

        [Header("Actions")]
        [SerializeField] private Button _backButton;
        [SerializeField] private Button _prevButton;
        [SerializeField] private Button _nextButton;
        [SerializeField] private Button _confirmButton;

        [Header("Shared chrome")]
        [SerializeField] private ProfileWidget _profile;
        [SerializeField] private CurrencyWidget _currency;

        // A dimmed neighbour must still read as inactive rather than broken. The centre
        // slot is always full alpha; the sides blend toward the backdrop.
        private const float SideAlpha = 0.6f;

        // Mirrors SlimUiSkin.ArtWellFill — that class lives in the editor assembly and
        // cannot be referenced from runtime code.
        private static readonly Color EmptyArtFill = new Color(0.045f, 0.050f, 0.072f, 1f);

        private readonly List<TrackDefinition> _tracks = new List<TrackDefinition>();
        private string _selectedTrackId;

        private void Awake()
        {
            // Self-wired rather than UnityEvent-persisted — see GarageScreenImpl.Awake.
            if (_backButton != null) _backButton.onClick.AddListener(HandleBackClicked);
            if (_prevButton != null) _prevButton.onClick.AddListener(HandlePrevClicked);
            if (_nextButton != null) _nextButton.onClick.AddListener(HandleNextClicked);
            if (_confirmButton != null) _confirmButton.onClick.AddListener(HandleConfirmClicked);
        }

        private void OnDestroy()
        {
            if (_backButton != null) _backButton.onClick.RemoveListener(HandleBackClicked);
            if (_prevButton != null) _prevButton.onClick.RemoveListener(HandlePrevClicked);
            if (_nextButton != null) _nextButton.onClick.RemoveListener(HandleNextClicked);
            if (_confirmButton != null) _confirmButton.onClick.RemoveListener(HandleConfirmClicked);
        }

        private void HandleBackClicked() => TriggerBackPressed();
        private void HandlePrevClicked() => TriggerPrevTrackPressed();
        private void HandleNextClicked() => TriggerNextTrackPressed();
        private void HandleConfirmClicked() => TriggerConfirmPressed();

        public override void Initialize(GameFlowManager flowManager)
        {
            RefreshFromProfile();
        }

        // Not overrides: the chrome contract (SetCurrency/RefreshFromProfile) belongs to
        // ILobbyScreen, and the flow never pushes profile updates to track screens. The
        // panel refreshes its own chrome on Initialize and when the hub raises it.
        public void RefreshFromProfile()
        {
            if (_profile != null) _profile.Refresh();
            if (_currency != null) _currency.Refresh();
        }

        public void SetCurrency(int amount)
        {
            if (_currency != null) _currency.SetCareerPoints(amount);
        }

        public override void PopulateTracks(IReadOnlyList<TrackDefinition> owned,
            IReadOnlyList<TrackDefinition> locked)
        {
            _tracks.Clear();
            _tracks.AddRange(owned);
            _tracks.AddRange(locked);
        }

        public override void SetSelectedTrack(TrackDefinition track)
        {
            _selectedTrackId = track != null ? track.TrackId : null;
        }

        public override void ShowFocusedTrack(TrackDefinition track, bool isSelected, bool isAvailable)
        {
            int index = track != null ? _tracks.FindIndex(t => t.TrackId == track.TrackId) : -1;
            if (index < 0 || _slots == null || _slots.Length < 3)
            {
                ShowEmpty();
                return;
            }

            ShowSlot(_slots[0], _tracks[Wrap(index - 1)], SideAlpha);
            ShowSlot(_slots[1], _tracks[index], 1f);
            ShowSlot(_slots[2], _tracks[Wrap(index + 1)], SideAlpha);

            // Confirm keys off the flow's verdict, not off availability: a locked focus
            // over a still-selected session track must not offer to race the locked one.
            if (_confirmButton != null) _confirmButton.interactable = isSelected;
        }

        private int Wrap(int i) => ((i % _tracks.Count) + _tracks.Count) % _tracks.Count;

        private void ShowEmpty()
        {
            if (_slots != null)
                foreach (var slot in _slots)
                    if (slot.Root != null) slot.Root.SetActive(false);
            if (_confirmButton != null) _confirmButton.interactable = false;
        }

        private void ShowSlot(TrackSlot slot, TrackDefinition track, float alpha)
        {
            if (slot.Root == null) return;
            slot.Root.SetActive(true);

            bool hasArt = track != null && (track.Thumbnail != null || track.MinimapTexture != null);
            if (slot.Art != null)
            {
                slot.Art.sprite = hasArt
                    ? (track.Thumbnail != null ? track.Thumbnail : track.MinimapTexture)
                    : null;
                // Without art the well renders as a dark panel rather than Unity's white
                // "missing sprite" quad.
                slot.Art.color = hasArt ? Color.white : EmptyArtFill;
            }
            if (slot.ShortCode != null) slot.ShortCode.text = track?.ShortCode ?? "";
            if (slot.Name != null) slot.Name.text = track?.DisplayName ?? "—";
            if (slot.Length != null)
                slot.Length.text = track != null ? $"{track.TrackLengthMeters / 1000f:0.000} KM" : "—";
            if (slot.Laps != null)
                slot.Laps.text = track != null ? $"{track.StandardRaceLaps}" : "—";
            if (slot.Difficulty != null)
                slot.Difficulty.text = string.IsNullOrEmpty(track?.DifficultyLabel) ? "—" : track.DifficultyLabel;
            if (slot.SelectedBadge != null)
                slot.SelectedBadge.SetActive(track != null && track.TrackId == _selectedTrackId);
            if (slot.Dim != null) slot.Dim.alpha = alpha;
        }
    }
}
