using UnityEngine;

namespace F1.GameFlow
{
    /// <summary>
    /// Yaw-only turntable under the garage's showcase car.
    ///
    /// The idle spin is slow on purpose: the car has to stay readable as a selection, not
    /// perform as a turntable demo. Dragging pauses the spin by easing it to zero rather
    /// than cutting it, and it eases back in a beat after release so the car does not
    /// lurch out of the player's hand.
    ///
    /// Sits on the car socket in the garage scene, above the swapper's instance in the
    /// hierarchy — the swapper instantiates at local identity, so rotating this transform
    /// rotates whatever car is on display.
    /// </summary>
    [DisallowMultipleComponent]
    public class GarageTurntable : MonoBehaviour
    {
        [Header("Auto Spin")]
        [Tooltip("Idle spin speed in degrees per second.")]
        [SerializeField] private float _autoSpinDegreesPerSecond = 12f;

        [Tooltip("How fast the spin ramps to zero while dragging and back afterwards.")]
        [SerializeField] private float _spinLerpSpeed = 8f;

        [Tooltip("Seconds after release before the spin starts easing back in.")]
        [SerializeField] private float _resumeDelaySeconds = 0.6f;

        [Header("Drag")]
        [Tooltip("Yaw degrees per pixel of horizontal drag.")]
        [SerializeField] private float _dragDegreesPerPixel = 0.2f;

        private float _socketYaw;
        private float _spin;
        private bool _dragging;
        private float _lastDragEndTime = float.NegativeInfinity;

        private void Awake()
        {
            _socketYaw = transform.localEulerAngles.y;
        }

        private void Update()
        {
            float target = _dragging || Time.time - _lastDragEndTime < _resumeDelaySeconds
                ? 0f
                : _autoSpinDegreesPerSecond;

            // Clamp01: on a frame hitch (alt-tab, editor stall) deltaTime can push the
            // lerp factor past 1 and overshoot the target, briefly spinning the car
            // backwards after a drag.
            _spin = Mathf.Lerp(_spin, target, Mathf.Clamp01(_spinLerpSpeed * Time.deltaTime));
            if (target == 0f && Mathf.Abs(_spin) < 0.01f) _spin = 0f;

            var euler = transform.localEulerAngles;
            transform.localRotation = Quaternion.Euler(euler.x, euler.y + _spin * Time.deltaTime, euler.z);
        }

        public void NotifyDragStarted() => _dragging = true;

        /// <summary>Rotates the car by <paramref name="deltaX"/> pixels' worth of yaw.</summary>
        public void ApplyDrag(float deltaX)
        {
            var euler = transform.localEulerAngles;
            transform.localRotation = Quaternion.Euler(euler.x, euler.y + deltaX * _dragDegreesPerPixel, euler.z);
        }

        public void NotifyDragEnded()
        {
            _dragging = false;
            _lastDragEndTime = Time.time;
        }

        /// <summary>Returns to the socket's authored yaw — called when the focused car changes.</summary>
        public void ResetPose()
        {
            var euler = transform.localEulerAngles;
            transform.localRotation = Quaternion.Euler(euler.x, _socketYaw, euler.z);
        }
    }
}
