using System.Collections.Generic;
using UnityEngine;
using F1.GameData;
using F1.Progression;

namespace F1.GameFlow
{
    /// <summary>
    /// Owns the garage scene: wires the garage screen to the flow manager, and the
    /// screen's events to the scene-side car display (turntable, showcase swapper).
    ///
    /// The garage's one rule is that the displayed car IS the selection: cycling focus
    /// onto a car the profile allows makes it the session car immediately, with no
    /// confirm step. Cycling onto a locked car is display-only — its stats render, but
    /// the session car stays whatever it was and Continue goes dark.
    ///
    /// The focus order is owned cars, then active rentals, then locked, so cycling
    /// forward moves from what the player can use towards what they cannot yet.
    /// </summary>
    [DisallowMultipleComponent]
    public class GarageSceneController : MonoBehaviour
    {
        private GameFlowManager _flow;
        private GarageScreen _screen;
        private GarageTurntable _turntable;
        private ShowcaseCarSwapper _swapper;

        private readonly List<CarDefinition> _cycle = new List<CarDefinition>();
        private int _focusIndex;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError("[GarageSceneController] No GameFlowManager found.", this);
                enabled = false;
                return;
            }

            _screen = GetComponentInChildren<GarageScreen>(true);
            if (_screen == null)
            {
                Debug.LogError(
                    "[GarageSceneController] No GarageScreen in this scene. The FlowSceneHost " +
                    "is probably missing its screen prefab.", this);
                enabled = false;
                return;
            }

            _turntable = FindAnyObjectByType<GarageTurntable>();
            _swapper = FindAnyObjectByType<ShowcaseCarSwapper>();
            if (_turntable == null)
                Debug.LogWarning(
                    "[GarageSceneController] No GarageTurntable in the scene; the car will " +
                    "not spin or respond to drags.", this);
            if (_swapper == null)
                Debug.LogWarning(
                    "[GarageSceneController] No ShowcaseCarSwapper in the scene; the " +
                    "displayed car will not change.", this);

            _screen.OnPrevCarPressed += OnPrevCarPressed;
            _screen.OnNextCarPressed += OnNextCarPressed;
            _screen.OnNextPressed += OnNextPressed;
            if (_turntable != null)
            {
                _screen.OnDragStarted += _turntable.NotifyDragStarted;
                _screen.OnDragDelta += _turntable.ApplyDrag;
                _screen.OnDragEnded += _turntable.NotifyDragEnded;
            }

            _screen.RefreshFromProfile();
            FocusInitialCar();
        }

        private void OnDestroy()
        {
            if (_screen == null) return;

            _screen.OnPrevCarPressed -= OnPrevCarPressed;
            _screen.OnNextCarPressed -= OnNextCarPressed;
            _screen.OnNextPressed -= OnNextPressed;
            if (_turntable != null)
            {
                _screen.OnDragStarted -= _turntable.NotifyDragStarted;
                _screen.OnDragDelta -= _turntable.ApplyDrag;
                _screen.OnDragEnded -= _turntable.NotifyDragEnded;
            }
        }

        private void OnPrevCarPressed() => MoveFocus(-1);

        private void OnNextCarPressed() => MoveFocus(+1);

        private void MoveFocus(int direction)
        {
            if (_cycle.Count == 0) return;
            _focusIndex = ((_focusIndex + direction) % _cycle.Count + _cycle.Count) % _cycle.Count;
            Focus(_cycle[_focusIndex]);
        }

        /// <summary>
        /// Entry focus starts on the session car when there is one, so returning from a
        /// race shows the car that was raced; otherwise the first car in cycle order.
        /// </summary>
        private void FocusInitialCar()
        {
            RebuildCycle();

            _focusIndex = 0;
            var chosen = _flow.SelectedCar;
            if (chosen != null)
            {
                for (int i = 0; i < _cycle.Count; i++)
                {
                    if (_cycle[i].CarId == chosen.CarId)
                    {
                        _focusIndex = i;
                        break;
                    }
                }
            }

            if (_cycle.Count > 0)
                Focus(_cycle[_focusIndex]);
            else
                Debug.LogError("[GarageSceneController] No cars are registered in GameDataRegistry.", this);
        }

        /// <summary>
        /// Owned first, then cars under an active rental, then everything locked. Three
        /// passes over the registry keeps each bucket in the registry's generation order
        /// and keeps the buckets disjoint.
        /// </summary>
        private void RebuildCycle()
        {
            _cycle.Clear();
            var profile = PlayerProfileManager.Current;
            if (profile == null)
            {
                _cycle.AddRange(GameDataRegistry.AllCars);
                return;
            }

            foreach (var car in GameDataRegistry.AllCars)
                if (profile.OwnsCar(car.CarId))
                    _cycle.Add(car);

            foreach (var car in GameDataRegistry.AllCars)
                if (!profile.OwnsCar(car.CarId) && profile.IsCarAvailable(car.CarId))
                    _cycle.Add(car);

            foreach (var car in GameDataRegistry.AllCars)
                if (!profile.IsCarAvailable(car.CarId))
                    _cycle.Add(car);
        }

        /// <summary>
        /// Makes <paramref name="car"/> the focused car: swap the showcase model, reset
        /// the turntable, render the panel. The same push-back contract as the tiles
        /// applies — the flow's verdict is pushed to the screen rather than the screen
        /// trusting the request, so Continue only lights up for a car the session will
        /// actually carry.
        /// </summary>
        private void Focus(CarDefinition car)
        {
            if (car == null) return;

            if (_swapper != null) _swapper.Show(car);
            if (_turntable != null) _turntable.ResetPose();

            var profile = PlayerProfileManager.Current;
            bool isAvailable = profile == null || profile.IsCarAvailable(car.CarId);

            if (isAvailable)
            {
                _flow.SelectCar(car);
            }
            else
            {
                Debug.Log($"[GarageSceneController] '{car.CarId}' is locked; the session car is unchanged.");
            }

            _screen.ShowFocusedCar(car, _flow.SelectedCar == car, isAvailable);
            _screen.SetChosenCar(_flow.SelectedCar);
        }

        private void OnNextPressed()
        {
            if (_flow.SelectedCar == null)
            {
                Debug.Log("[GarageSceneController] Continue pressed with no session car; staying in the garage.");
                return;
            }

            if (!SceneFlowService.IsSceneAvailable(FlowSceneNames.TrackSelection))
            {
                Debug.Log($"[GarageSceneController] '{FlowSceneNames.TrackSelection}' is not " +
                          "available yet, so the garage stays here.");
                return;
            }

            _flow.GoToTrackSelection();
        }
    }
}
