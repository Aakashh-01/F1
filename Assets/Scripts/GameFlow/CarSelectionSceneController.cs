using UnityEngine;
using F1.GameData;
using F1.Progression;

namespace F1.GameFlow
{
    /// <summary>
    /// Owns the behaviour of the car-selection hub: it wires the screen's events to the
    /// flow manager and keeps the selected car highlighted.
    ///
    /// This is the scene-controller half of the split from <c>LobbyManager</c>. Each
    /// flow scene gets one of these; adding a future hub destination is a new scene plus
    /// a controller, with no change to <see cref="GameFlowManager"/>.
    /// </summary>
    [DisallowMultipleComponent]
    public class CarSelectionSceneController : MonoBehaviour
    {
        private GameFlowManager _flow;
        private CarSelectionScreen _screen;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError(
                    "[CarSelectionSceneController] No GameFlowManager found.", this);
                enabled = false;
                return;
            }

            _screen = GetComponentInChildren<CarSelectionScreen>(true);
            if (_screen == null)
            {
                Debug.LogError(
                    "[CarSelectionSceneController] No CarSelectionScreen in this scene. " +
                    "The FlowSceneHost is probably missing its screen prefab.", this);
                enabled = false;
                return;
            }

            _screen.OnCarSelected += OnCarSelected;
            _screen.OnCarRented += OnCarSelected;
            _screen.OnCarUnlockRequested += OnCarUnlockRequested;

            HighlightCurrentSelection();
        }

        private void OnDestroy()
        {
            if (_screen == null) return;
            _screen.OnCarSelected -= OnCarSelected;
            _screen.OnCarRented -= OnCarSelected;
            _screen.OnCarUnlockRequested -= OnCarUnlockRequested;
        }

        private void OnCarSelected(CarDefinition car)
        {
            // SelectCar applies the ownership/rental rules and updates the session, so a
            // rented car that the player has no rental for is rejected here rather than
            // silently becoming the session car.
            _flow.SelectCar(car);
            _screen.SetSelectedCar(_flow.SelectedCar);

            if (_flow.SelectedCar == null)
            {
                Debug.LogWarning(
                    $"[CarSelectionSceneController] '{car?.CarId}' was not accepted as the " +
                    "session car. Check ownership or an active rental.");
            }

            // NOTE: this used to advance to track selection here, immediately, on click.
            // It no longer does. The live route reaches car selection as the lobby's
            // selection state, where choosing a tile only marks it and enables Next; see
            // LobbySceneController. This scene is now off the main route, but the behaviour
            // is kept consistent so returning to it does not reintroduce a hop the player
            // cannot see coming.
        }

        private void OnCarUnlockRequested(CarDefinition car)
        {
            if (car == null) return;

            if (ProgressionRegistry.TryUnlockCar(car.CarId))
            {
                Debug.Log($"[CarSelectionSceneController] Unlocked {car.DisplayName}.");
                _flow.SelectCar(car);
            }
            else
            {
                Debug.Log(
                    $"[CarSelectionSceneController] Cannot afford {car.DisplayName} " +
                    $"({car.UnlockCostPoints} pts).", this);
            }

            _screen.SetSelectedCar(_flow.SelectedCar);
        }

        private void HighlightCurrentSelection()
        {
            if (_flow?.SelectedCar != null)
                _screen.SetSelectedCar(_flow.SelectedCar);
        }
    }
}
