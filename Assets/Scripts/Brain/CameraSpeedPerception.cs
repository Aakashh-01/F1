using UnityEngine;
using System.Reflection;

public class CameraSpeedPerception : MonoBehaviour
{
    public VehiclePhysicsProfile physicsProfile;
    public Rigidbody targetRigidbody;
    public Camera unityCamera;
    public Component cinemachineCamera;
    public bool useProfileSettings = true;

    [Header("Fallback Settings")]
    public float baseFov = 60f;
    public float maxFov = 72f;
    public float maxFovSpeedKmh = 260f;
    public float fovSmoothTime = 0.25f;

    private float _fovVelocity;

    private void Awake()
    {
        if (unityCamera == null)
            unityCamera = Camera.main;

        if (cinemachineCamera == null)
            cinemachineCamera = FindCinemachineCamera();
    }

    private void LateUpdate()
    {
        if (targetRigidbody == null || (unityCamera == null && cinemachineCamera == null))
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
        float fallbackFov = unityCamera != null ? unityCamera.fieldOfView : startFov;
        float currentFov = cinemachineCamera != null ? GetCinemachineFov(cinemachineCamera, fallbackFov) : fallbackFov;
        float fov = Time.deltaTime > 0f
            ? Mathf.SmoothDamp(currentFov, targetFov, ref _fovVelocity, smoothTime)
            : targetFov;

        if (cinemachineCamera != null)
            SetCinemachineFov(cinemachineCamera, fov);

        if (unityCamera != null)
            unityCamera.fieldOfView = fov;
    }

    private static Component FindCinemachineCamera()
    {
        MonoBehaviour[] behaviours = FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None);
        for (int i = 0; i < behaviours.Length; i++)
        {
            MonoBehaviour behaviour = behaviours[i];
            if (behaviour != null && behaviour.GetType().FullName == "Unity.Cinemachine.CinemachineCamera")
                return behaviour;
        }

        return null;
    }

    private static float GetCinemachineFov(Component camera, float fallback)
    {
        if (!TryGetLens(camera, out object lens, out FieldInfo lensField))
            return fallback;

        FieldInfo fovField = lens.GetType().GetField("FieldOfView");
        return fovField != null ? (float)fovField.GetValue(lens) : fallback;
    }

    private static void SetCinemachineFov(Component camera, float fov)
    {
        if (!TryGetLens(camera, out object lens, out FieldInfo lensField))
            return;

        FieldInfo fovField = lens.GetType().GetField("FieldOfView");
        if (fovField == null)
            return;

        fovField.SetValue(lens, fov);
        lensField.SetValue(camera, lens);
    }

    private static bool TryGetLens(Component camera, out object lens, out FieldInfo lensField)
    {
        lens = null;
        lensField = null;
        if (camera == null)
            return false;

        lensField = camera.GetType().GetField("Lens", BindingFlags.Instance | BindingFlags.Public);
        if (lensField == null)
            return false;

        lens = lensField.GetValue(camera);
        return lens != null;
    }
}
