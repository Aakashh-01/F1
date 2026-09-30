using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using F1.GameData;
using F1.GameFlow;
using F1.Progression;

namespace F1.UI
{
    /// <summary>
    /// Concrete TrackSelectionScreen — a centred hero tile for the playable circuit, with the
    /// unbuilt ones flanking it.
    ///
    /// This used to be two columns, Owned and Locked. With one playable track that produced a
    /// ragged 2x3 grid with a hole in it, and the single playable circuit sat in the top-left
    /// corner where it read as just another option rather than the only one you can race.
    /// The column split was also carrying the owned/locked distinction twice — the per-card
    /// AVAILABLE / COMING SOON status line already says it — so it went.
    /// </summary>
    public class TrackSelectionScreenImpl : TrackSelectionScreen
    {
        [Header("Columns")]
        [Tooltip("Centre column. Holds the playable circuit as a larger hero tile.")]
        [SerializeField] private RectTransform _heroColumn;

        [Tooltip("Flanking columns for the circuits that have no scene yet.")]
        [SerializeField] private RectTransform _lockedLeftColumn;
        [SerializeField] private RectTransform _lockedRightColumn;

        [Header("Prefabs")]
        [SerializeField] private TrackCardImpl _trackCardPrefab;

        // The hero is larger, and its artwork grows with it. The art panel is anchored to the
        // card's top edge with a fixed height, so a bigger card without a bigger panel would
        // just leave a band of empty bodywork under the picture.
        private static readonly Vector2 HeroSize = new Vector2(430f, 380f);
        private const float HeroArtHeight = 232f;
        private static readonly Vector2 SupportingSize = new Vector2(250f, 280f);
        private const float SupportingArtHeight = 165f;

        private GameFlowManager _flow;
        private readonly List<TrackCardImpl> _allCards = new();

        public override void Initialize(GameFlowManager flowManager)
        {
            _flow = flowManager;
            RefreshTrackLists();
        }

        public override void Show()
        {
            gameObject.SetActive(true);
        }

        public override void Hide()
        {
            gameObject.SetActive(false);
        }

        public override void PopulateTracks(IReadOnlyList<TrackDefinition> owned,
            IReadOnlyList<TrackDefinition> locked)
        {
            ClearCards();

            // Partition by AVAILABILITY, not by ownership.
            //
            // ProgressionRegistry.GetOwnedTracks returns what the profile owns, and both
            // Monaco and Spa are free, so both arrive in `owned`. But Spa points at the same
            // corner as Monaco and is flagged UnderConstruction, so it cannot be raced. Sizing
            // the hero off the ownership list put Spa in the centre column beside Monaco,
            // which read as two playable options when there is only one. SceneAvailability is
            // the same test the card uses to draw its COMING SOON badge, so the layout and the
            // badge can no longer disagree about what is playable.
            var playable = new List<TrackDefinition>();
            var upcoming = new List<TrackDefinition>();

            foreach (var track in owned)
                (SceneAvailability.IsAvailable(track) ? playable : upcoming).Add(track);
            foreach (var track in locked)
                upcoming.Add(track);

            // The first playable track is the hero. Any further playable tracks stack below it
            // in the same column rather than displacing it, so the hero only changes when the
            // one it is showing is genuinely gone.
            for (int i = 0; i < playable.Count; i++)
            {
                bool isHero = i == 0;
                AddCard(playable[i], _heroColumn, OnTrackOwnedClicked,
                        isHero ? HeroSize : SupportingSize,
                        isHero ? HeroArtHeight : SupportingArtHeight);
            }

            // Flanked two-and-two so the composition is balanced either side of the hero.
            int perSide = Mathf.CeilToInt(upcoming.Count / 2f);
            for (int i = 0; i < upcoming.Count; i++)
            {
                var column = i < perSide ? _lockedLeftColumn : _lockedRightColumn;
                AddCard(upcoming[i], column, OnTrackLockedClicked,
                        SupportingSize, SupportingArtHeight);
            }
        }

        public override void SetSelectedTrack(TrackDefinition track)
        {
            foreach (var card in _allCards)
                card.SetSelected(track.TrackId);
        }

        private void RefreshTrackLists()
        {
            var owned = ProgressionRegistry.GetOwnedTracks();
            var locked = ProgressionRegistry.GetLockedTracks();
            PopulateTracks(owned, locked);
        }

        private void AddCard(TrackDefinition track, RectTransform column,
            System.Action<TrackDefinition> onClick, Vector2 size, float artHeight)
        {
            if (_trackCardPrefab == null || column == null) return;
            var card = Instantiate(_trackCardPrefab, column);
            ResizeCard(card.GetComponent<RectTransform>(), size, artHeight);
            card.SetTrack(track);
            card.OnCardClicked += onClick;
            _allCards.Add(card);
        }

        /// <summary>
        /// Resizes a card instance, and relays out its artwork panel and every label with it.
        ///
        /// The layout groups run with childControlWidth and childControlHeight false, so they
        /// keep whatever sizeDelta the card carries — which is the only way one prefab can
        /// serve as both a hero tile and a supporting one.
        ///
        /// Everything inside has to move too. The art panel is pinned to the card's top edge
        /// at a fixed height, so a taller card with an unchanged panel leaves dead space under
        /// the picture instead of a bigger picture. The labels are worse: the builder
        /// positions them as fractions of the card height, so reusing those fractions on a
        /// taller card slides them up INTO the artwork. Hence the text region is recomputed
        /// from the new art height and the five labels are laid out as even bands across it,
        /// which adapts to any card size rather than to the one the builder happened to make.
        /// </summary>
        private static void ResizeCard(RectTransform card, Vector2 size, float artHeight)
        {
            if (card == null) return;
            card.sizeDelta = size;

            const float pad = 8f;
            float cardHeight = size.y;

            var art = card.Find("TrackArt") as RectTransform;
            if (art != null)
            {
                art.anchorMin = new Vector2(0f, 1f);
                art.anchorMax = new Vector2(1f, 1f);
                art.pivot = new Vector2(0.5f, 1f);
                art.offsetMin = new Vector2(pad, -(artHeight + pad));
                art.offsetMax = new Vector2(-pad, -pad);
            }

            // Space between the bottom of the art and the bottom of the card.
            float textTop = cardHeight - artHeight - pad;
            if (textTop <= 0f) return;

            // Bottom-up: button, status, cost, gen, name. GenText and ShortCodeText are
            // alternatives on different card prefabs, so only one is present at a time.
            string[] order = { "CardButton", "StatusText", "CostText", "GenText", "NameText" };
            float band = textTop / order.Length;

            for (int i = 0; i < order.Length; i++)
            {
                var label = card.Find(order[i]) as RectTransform;
                if (label == null) continue;

                float centre = band * (i + 0.5f);
                float h = band * 0.74f;
                float half = h * 0.5f;

                label.anchorMin = new Vector2(0.5f, centre / cardHeight);
                label.anchorMax = new Vector2(0.5f, centre / cardHeight);
                label.pivot = new Vector2(0.5f, 0.5f);
                label.anchoredPosition = Vector2.zero;
                label.offsetMin = new Vector2(-(size.x * 0.5f - 12f), -half);
                label.offsetMax = new Vector2(size.x * 0.5f - 12f, half);
            }
        }

        private void ClearCards()
        {
            foreach (var card in _allCards)
                if (card != null) Destroy(card.gameObject);
            _allCards.Clear();
        }

        private void OnTrackOwnedClicked(TrackDefinition track)
        {
            _flow.SelectTrack(track);
            SetSelectedTrack(track);
            TriggerTrackSelected(track);
        }

        private void OnTrackLockedClicked(TrackDefinition track)
        {
            if (ProgressionRegistry.TryUnlockTrack(track.TrackId))
            {
                TriggerTrackUnlockRequested(track);
                RefreshTrackLists();
            }
        }

        private void OnDestroy()
        {
            ClearCards();
        }
    }
}
