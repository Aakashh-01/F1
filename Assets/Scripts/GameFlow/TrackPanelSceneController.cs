using System.Collections.Generic;
using UnityEngine;
using F1.GameData;
using F1.Progression;

namespace F1.GameFlow
{
    /// <summary>
    /// Owns the track panel's behaviour inside the menu hub: wires the panel's events to
    /// the flow and runs the focus cycle.
    ///
    /// The panel is the second hosted screen of 05_GarageScene. Its host root starts
    /// inactive and the hub view raises it only when the flow's current screen is
    /// TrackSelection — which is also what defers this Start until the panel is actually
    /// on stage. That deferral is load-bearing: binding and focusing must not run while
    /// the car panel owns the scene, or entering the garage would silently pick a track.
    ///
    /// Same rule as the garage's car panel: the displayed track IS the selection.
    /// Focusing a raceable circuit makes it the session track immediately, with the
    /// flow's verdict pushed back through the screen; focusing an unbuilt one is
    /// display-only and Confirm goes dark.
    /// </summary>
    [DisallowMultipleComponent]
    public class TrackPanelSceneController : MonoBehaviour
    {
        private GameFlowManager _flow;
        private TrackPanelScreen _screen;

        private readonly List<TrackDefinition> _cycle = new List<TrackDefinition>();
        private int _focusIndex;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError("[TrackPanelSceneController] No GameFlowManager found.", this);
                enabled = false;
                return;
            }

            _screen = GetComponentInChildren<TrackPanelScreen>(true);
            if (_screen == null)
            {
                Debug.LogError(
                    "[TrackPanelSceneController] No TrackPanelScreen under this host. The " +
                    "FlowSceneHost is probably missing its screen prefab.", this);
                enabled = false;
                return;
            }

            _screen.OnPrevTrackPressed += OnPrevTrackPressed;
            _screen.OnNextTrackPressed += OnNextTrackPressed;
            _screen.OnBackPressed += OnBackPressed;
            _screen.OnConfirmPressed += OnConfirmPressed;

            RebuildCycle();
            if (_cycle.Count == 0)
            {
                Debug.LogError("[TrackPanelSceneController] No tracks are registered.", this);
                return;
            }

            _screen.PopulateTracks(_cycle, System.Array.Empty<TrackDefinition>());
            FocusInitialTrack();
        }

        private void OnDestroy()
        {
            if (_screen == null) return;

            _screen.OnPrevTrackPressed -= OnPrevTrackPressed;
            _screen.OnNextTrackPressed -= OnNextTrackPressed;
            _screen.OnBackPressed -= OnBackPressed;
            _screen.OnConfirmPressed -= OnConfirmPressed;
        }

        private void OnPrevTrackPressed() => MoveFocus(-1);

        private void OnNextTrackPressed() => MoveFocus(+1);

        private void MoveFocus(int direction)
        {
            if (_cycle.Count == 0) return;
            _focusIndex = ((_focusIndex + direction) % _cycle.Count + _cycle.Count) % _cycle.Count;
            Focus(_cycle[_focusIndex]);
        }

        /// <summary>
        /// Entry focus starts on the session track when there is one, so returning from
        /// wing setup shows what was raced; otherwise the first track in cycle order.
        /// </summary>
        private void FocusInitialTrack()
        {
            _focusIndex = 0;
            var chosen = _flow.SelectedTrack;
            if (chosen != null)
            {
                for (int i = 0; i < _cycle.Count; i++)
                {
                    if (_cycle[i].TrackId == chosen.TrackId)
                    {
                        _focusIndex = i;
                        break;
                    }
                }
            }

            Focus(_cycle[_focusIndex]);
        }

        /// <summary>
        /// Owned tracks first, then locked, each in registry order — the same shape as
        /// the car cycle, so cycling forward moves from what the player can race towards
        /// what they cannot yet.
        /// </summary>
        private void RebuildCycle()
        {
            _cycle.Clear();
            foreach (var track in ProgressionRegistry.GetOwnedTracks())
                _cycle.Add(track);
            foreach (var track in ProgressionRegistry.GetLockedTracks())
                _cycle.Add(track);
        }

        private void Focus(TrackDefinition track)
        {
            if (track == null) return;

            // Raceable, not merely owned: Spa is owned but shares its corner with Monaco
            // and has no scene of its own, so it must stay display-only. Locked circuits
            // are turned away by SelectTrack's own ownership guard (GameFlowManager.cs:280),
            // so they land display-only through the same verdict push-back.
            bool isAvailable = SceneAvailability.IsAvailable(track);
            if (isAvailable)
            {
                _flow.SelectTrack(track);
            }
            else
            {
                Debug.Log(
                    $"[TrackPanelSceneController] '{track.TrackId}' is not raceable yet; the " +
                    "session track is unchanged.");
            }

            _screen.SetSelectedTrack(_flow.SelectedTrack);
            _screen.ShowFocusedTrack(track, _flow.SelectedTrack == track, isAvailable);
        }

        private void OnConfirmPressed()
        {
            if (_flow.SelectedTrack == null)
            {
                Debug.Log("[TrackPanelSceneController] Confirm pressed with no session track.");
                return;
            }

            if (!SceneAvailability.IsAvailable(_flow.SelectedTrack))
            {
                Debug.Log(
                    $"[TrackPanelSceneController] '{_flow.SelectedTrack.TrackId}' has no scene; " +
                    "staying on the panel.");
                return;
            }

            // Invariant 3: track selection always precedes wing selection.
            _flow.GoToWingSetup();
        }

        private void OnBackPressed() => _flow.GoToLobby();
    }
}
