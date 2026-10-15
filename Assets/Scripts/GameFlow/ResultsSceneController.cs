using UnityEngine;

namespace F1.GameFlow
{
    /// <summary>
    /// Owns the behaviour of the results scene: it wires the screen's events to the flow and
    /// nothing else.
    ///
    /// The screen itself only draws. <see cref="F1.UI.ResultsScreenImpl"/> renders whatever the
    /// flow presented, and the flow presents from the transition that commits to this scene —
    /// so by the time this reacts, the result is already recorded and scored. This controller's
    /// only job is navigation, exactly as <see cref="TrackSelectionSceneController"/> is.
    /// </summary>
    [DisallowMultipleComponent]
    public class ResultsSceneController : MonoBehaviour
    {
        private GameFlowManager _flow;
        private ResultsScreen _screen;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError("[ResultsSceneController] No GameFlowManager found.", this);
                enabled = false;
                return;
            }

            _screen = GetComponentInChildren<ResultsScreen>(true);
            if (_screen == null)
            {
                Debug.LogError(
                    "[ResultsSceneController] No ResultsScreen in this scene. The FlowSceneHost " +
                    "is probably missing its screen prefab.", this);
                enabled = false;
                return;
            }

            _screen.OnContinuePressed += OnContinuePressed;
            _screen.OnRetryPressed += OnRetryPressed;

            // OnMainMenuPressed is deliberately left unsubscribed. There is no main menu: the
            // lobby is the game's hub and GameFlowManager._mainMenuScene is empty, so routing
            // to it lands on the overlay branch of TransitionToScreen, finds no host and no
            // legacy MainMenuScreen, and logs an error while leaving the player exactly where
            // they were. The lobby button already goes where a main menu would have.
            // The event is kept on the screen because removing a member from a serialized
            // event's type is a data migration, not a cleanup.
        }

        private void OnDestroy()
        {
            if (_screen == null) return;
            _screen.OnContinuePressed -= OnContinuePressed;
            _screen.OnRetryPressed -= OnRetryPressed;
        }

        /// <summary>
        /// Continue returns to the LOBBY, not to a main menu and not to qualifying.
        ///
        /// The lobby is where the points just earned become spendable — it hosts car selection,
        /// and the car tiles are where a new balance turns into an unlock — so ending the race
        /// there is what closes the GDD's loop rather than restarting it halfway.
        /// </summary>
        private void OnContinuePressed() => _flow.GoToLobby();

        /// <summary>
        /// Retry runs the same race again.
        ///
        /// <see cref="GameFlowManager.RestartSession"/> rather than
        /// <see cref="GameFlowManager.StartRaceSession"/>, because restarting is not starting:
        /// it must not re-check the qualifying gate that has already been satisfied this
        /// session, and it must not reset the attempt counter. GoToRace clears the previous
        /// result on the way in, so the second race is scored on its own merit.
        /// </summary>
        private void OnRetryPressed() => _flow.RestartSession();
    }
}