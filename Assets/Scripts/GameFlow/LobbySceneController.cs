using UnityEngine;
using F1.GameData;
using F1.GameFlow;

namespace F1.GameFlow
{
    /// <summary>
    /// Owns the lobby scene: wires the car-selection screen to the flow manager.
    ///
    /// The lobby is the 3D garage and the car tiles are its only screen — there is no hub
    /// state and no START RACE button in front of them any more. The flow leaves the lobby
    /// in exactly one place: when the player presses Next with a car chosen, which loads
    /// track selection.
    /// </summary>
    [DisallowMultipleComponent]
    public class LobbySceneController : MonoBehaviour
    {
        private GameFlowManager _flow;
        private LobbyScreen _screen;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError(
                    "[LobbySceneController] No GameFlowManager found.", this);
                enabled = false;
                return;
            }

            _screen = GetComponentInChildren<LobbyScreen>(true);
            if (_screen == null)
            {
                Debug.LogError(
                    "[LobbySceneController] No LobbyScreen in this scene. The FlowSceneHost is " +
                    "probably missing its screen prefab.", this);
                enabled = false;
                return;
            }

            _screen.OnNextPressed += OnNextPressed;
            _screen.OnCarChosen += OnCarChosen;

            RefreshCurrency();
        }

        private void OnDestroy()
        {
            if (_screen == null) return;
            _screen.OnNextPressed -= OnNextPressed;
            _screen.OnCarChosen -= OnCarChosen;
        }

        /// <summary>
        /// A tile was tapped. Selecting does not navigate — it makes the car the session car
        /// and enables Next. The old flow advanced the moment a card was clicked, which is
        /// what made a Select button and a Next button meaningless.
        /// </summary>
        private void OnCarChosen(CarDefinition car)
        {
            if (car == null) return;

            _flow.SelectCar(car);
            if (_flow.SelectedCar == null)
            {
                Debug.Log($"[LobbySceneController] '{car.CarId}' was not accepted as the " +
                          "session car. Check ownership or an active rental.");
            }

            // Push the flow's verdict back to the screen rather than leaving the screen's
            // optimistic mark in place. A rejected tap (a rental with no active rental, an
            // unaffordable unlock) has to leave no tile selected and Next disabled, or the
            // player can select a car the session will never carry and press Next into
            // nothing happening.
            _screen.SetChosenCar(_flow.SelectedCar);
        }

        private void OnNextPressed()
        {
            // Track selection needs a car in the session. Phase B supplies the tiles and the
            // Next button is only enabled once one is chosen; this guard is what stops a
            // premature press from loading a track selection with no car to race.
            if (_flow.SelectedCar == null)
            {
                Debug.Log("[LobbySceneController] Next pressed with no car selected; " +
                          "staying in the lobby.");
                return;
            }

            if (!SceneFlowService.IsSceneAvailable(FlowSceneNames.TrackSelection))
            {
                Debug.Log($"[LobbySceneController] '{FlowSceneNames.TrackSelection}' is not " +
                          "available yet, so the lobby stays here.");
                return;
            }

            _flow.GoToTrackSelection();
        }

        private void RefreshCurrency()
        {
            _screen.RefreshFromProfile();
        }
    }
}
