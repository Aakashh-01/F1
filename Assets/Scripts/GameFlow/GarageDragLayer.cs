using System;
using UnityEngine;
using UnityEngine.EventSystems;

namespace F1.GameFlow
{
    /// <summary>
    /// An invisible full-canvas layer that turns drags anywhere into turntable input.
    ///
    /// It is authored as the first child of the garage canvas, so every panel sits above
    /// it and consumes its own raycasts; any pointer that misses a panel lands here. It
    /// speaks the uGUI drag interfaces, so it behaves identically under
    /// <c>InputSystemUIInputModule</c> and a legacy <c>StandaloneInputModule</c>.
    /// </summary>
    [DisallowMultipleComponent]
    public class GarageDragLayer : MonoBehaviour, IBeginDragHandler, IDragHandler, IEndDragHandler
    {
        /// <summary>A drag began on the layer.</summary>
        public event Action DragStarted;

        /// <summary>The drag moved; the argument is the horizontal pointer delta in pixels.</summary>
        public event Action<float> DragDelta;

        /// <summary>The drag ended.</summary>
        public event Action DragEnded;

        private float _lastPointerX;

        public void OnBeginDrag(PointerEventData eventData)
        {
            _lastPointerX = eventData.position.x;
            DragStarted?.Invoke();
        }

        public void OnDrag(PointerEventData eventData)
        {
            float deltaX = eventData.position.x - _lastPointerX;
            _lastPointerX = eventData.position.x;
            DragDelta?.Invoke(deltaX);
        }

        public void OnEndDrag(PointerEventData eventData)
        {
            DragEnded?.Invoke();
        }
    }
}
