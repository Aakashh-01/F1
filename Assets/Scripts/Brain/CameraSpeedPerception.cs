using UnityEngine;
#if CINEMACHINE
using Cinemachine;
#endif

public class CameraSpeedPerception : MonoBehaviour
{
    public VehiclePhysicsProfile physicsProfile;
    public Rigidbody targetRigidbody;
    public Camera unityCamera;
    public bool useProfileSettings = true;

    [Header("Fallback Settings")]
    public float baseFov = 60f;
    public float maxFov = 78f;
    public float maxFovSpeedKmh = 260f;
    public float fovSmoothTime = 0.18f;

    private float _fovVelocity;

    private void Awake()
    {
        if (unityCamera == null)
            unityCamera = Camera.main;
    }

    private void LateUpdate()
    {
        if (unityCamera == null || targetRigidbody == null)
            return;

        CameraSpeedProfile settings = useProfileSettings && physicsProfile != null ? physicsProfile.camera : null;
        float startFov = settings != null ? settings.baseFov : baseFov;
        float targetMaxFov = settings != null ? settings.maxFov : maxFov;
        float fovSpeed = settings != null ? settings.maxFovSpeedKmh : maxFovSpeedKmh;
        float smoothTime = settings != null ? settings.fovSmoothTime : fovSmoothTime;

#if UNITY_6000_0_OR_NEWER
        float speedKmh = targetRigidbody.linearVelocity.magnitude * 3.6f;
#else
        float speedKmh = targetRigidbody.velocity.magnitude * 3.6f;
#endif
        float speedT = Mathf.InverseLerp(0f, Mathf.Max(1f, fovSpeed), speedKmh);
        float targetFov = Mathf.Lerp(startFov, targetMaxFov, speedT);
        unityCamera.fieldOfView = Mathf.SmoothDamp(unityCamera.fieldOfView, targetFov, ref _fovVelocity, smoothTime);
    }
}
