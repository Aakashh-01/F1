using UnityEngine;
using F1.GameData;
using F1.Progression;

namespace F1.GameFlow
{
    /// <summary>
    /// Owns the behaviour of the track-selection scene: it wires the screen's events to
    /// the flow and nothing else.
    ///
    /// The screen itself applies the selection (<c>TrackSelectionScreenImpl</c> calls
    /// <c>GameFlowManager.SelectTrack</c> before raising the event), so by the time this
    /// reacts the track is already part of the session. This controller's only job is
    /// navigation.
    /// </summary>
    [DisallowMultipleComponent]
    public class TrackSelectionSceneController : MonoBehaviour
    {
        private GameFlowManager _flow;
        private TrackSelectionScreen _screen;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError("[TrackSelectionSceneController] No GameFlowManager found.", this);
                enabled = false;
                return;
            }

            _screen = GetComponentInChildren<TrackSelectionScreen>(true);
            if (_screen == null)
            {
                Debug.LogError(
                    "[TrackSelectionSceneController] No TrackSelectionScreen in this scene. " +
                    "The FlowSceneHost is probably missing its screen prefab.", this);
                enabled = false;
                return;
            }

            _screen.OnTrackSelected += OnTrackSelected;
            _screen.OnBackPressed += OnBackPressed;
            _screen.OnTrackUnlockRequested += OnTrackUnlockRequested;
        }

        private void OnDestroy()
        {
            if (_screen == null) return;
            _screen.OnTrackSelected -= OnTrackSelected;
            _screen.OnBackPressed -= OnBackPressed;
            _screen.OnTrackUnlockRequested -= OnTrackUnlockRequested;
        }

        private void OnTrackSelected(TrackDefinition track)
        {
            // The screen has already applied the selection; re-assert it so the session
            // is authoritative regardless of how the event was raised.
            _flow.SelectTrack(track);
            _screen.SetSelectedTrack(_flow.SelectedTrack);

            if (_flow.SelectedTrack == null)
            {
                Debug.LogWarning(
                    $"[TrackSelectionSceneController] '{track?.TrackId}' was not accepted. " +
                    "It is probably still locked.");
                return;
            }

            // Invariant 3: track selection always precedes wing selection.
            _flow.GoToWingSetup();
        }

        private void OnBackPressed() => _flow.GoToCarSelection();

        private void OnTrackUnlockRequested(TrackDefinition track)
        {
            Debug.Log(
                $"[TrackSelectionSceneController] Unlocked {track?.DisplayName} " +
                $"({ProgressionRegistry.CanAffordTrack(track?.TrackId ?? string.Empty)}).", this);
        }
    }
}
