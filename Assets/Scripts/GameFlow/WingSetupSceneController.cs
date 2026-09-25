using UnityEngine;
using F1.GameData;

namespace F1.GameFlow
{
    /// <summary>
    /// Owns the behaviour of the wing-setup scene.
    ///
    /// Two jobs beyond wiring: it pushes the current car/track/wing into the screen so the
    /// track-specific wing recommendation renders, and it starts the qualifying session on
    /// Continue.
    ///
    /// The screen applies the wing itself (<c>WingSetupScreenImpl</c> calls
    /// <c>GameFlowManager.SelectWing</c> when a toggle flips), which also persists the
    /// per-track preference.
    /// </summary>
    [DisallowMultipleComponent]
    public class WingSetupSceneController : MonoBehaviour
    {
        private GameFlowManager _flow;
        private WingSetupScreen _screen;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError("[WingSetupSceneController] No GameFlowManager found.", this);
                enabled = false;
                return;
            }

            _screen = GetComponentInChildren<WingSetupScreen>(true);
            if (_screen == null)
            {
                Debug.LogError(
                    "[WingSetupSceneController] No WingSetupScreen in this scene. " +
                    "The FlowSceneHost is probably missing its screen prefab.", this);
                enabled = false;
                return;
            }

            _screen.OnContinuePressed += OnContinuePressed;
            _screen.OnBackPressed += OnBackPressed;

            // The screen needs the selection to render its recommendation, and nothing
            // else feeds it — the flow knows the current values.
            _screen.Setup(_flow.SelectedCar, _flow.SelectedTrack, _flow.SelectedWing);
        }

        private void OnDestroy()
        {
            if (_screen == null) return;
            _screen.OnContinuePressed -= OnContinuePressed;
            _screen.OnBackPressed -= OnBackPressed;
        }

        private void OnContinuePressed()
        {
            if (!_flow.Session.IsSelectionComplete)
            {
                Debug.LogWarning(
                    "[WingSetupSceneController] Cannot start qualifying without both a car " +
                    "and a track selected.");
                return;
            }

            // Invariant 4: wing selection always precedes qualifying.
            _flow.StartQualifyingSession();
        }

        private void OnBackPressed() => _flow.GoToTrackSelection();
    }
}
