using System.Collections;
using UnityEngine;

namespace F1.GameFlow
{
    /// Renders the garage's realtime reflection probe once at startup.
    ///
    /// The probe uses ViaScripting refresh so it costs nothing per frame, but that
    /// also means a build/play session would otherwise get an empty (black) cubemap
    /// and the glossy floor would lose its strip-light pools. Two frames: one for
    /// lights/volumes to settle, then the capture.
    [DisallowMultipleComponent]
    [RequireComponent(typeof(ReflectionProbe))]
    public class GarageProbeOnce : MonoBehaviour
    {
        private ReflectionProbe _probe;

        private void Awake()
        {
            _probe = GetComponent<ReflectionProbe>();
            StartCoroutine(CaptureAfterSettle());
        }

        private IEnumerator CaptureAfterSettle()
        {
            yield return new WaitForEndOfFrame();
            yield return new WaitForEndOfFrame();
            _probe.RenderProbe();
        }
    }
}
