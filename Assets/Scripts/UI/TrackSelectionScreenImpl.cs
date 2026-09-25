using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using F1.GameData;
using F1.GameFlow;
using F1.Progression;

namespace F1.UI
{
    /// <summary>
    /// Concrete TrackSelectionScreen — 2 columns (Owned / Locked) with track cards.
    /// </summary>
    public class TrackSelectionScreenImpl : TrackSelectionScreen
    {
        [Header("Columns")]
        [SerializeField] private RectTransform _ownedColumn;
        [SerializeField] private RectTransform _lockedColumn;

        [Header("Prefabs")]
        [SerializeField] private TrackCardImpl _trackCardPrefab;

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

            foreach (var track in owned)
                AddCard(track, _ownedColumn, OnTrackOwnedClicked);
            foreach (var track in locked)
                AddCard(track, _lockedColumn, OnTrackLockedClicked);
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

        private void AddCard(TrackDefinition track, RectTransform column, System.Action<TrackDefinition> onClick)
        {
            if (_trackCardPrefab == null || column == null) return;
            var card = Instantiate(_trackCardPrefab, column);
            card.SetTrack(track);
            card.OnCardClicked += onClick;
            _allCards.Add(card);
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
