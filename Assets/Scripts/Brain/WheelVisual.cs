using UnityEngine;

public class WheelVisual : MonoBehaviour
{
    [Header("References")]
    public Transform wheelMesh;
    public Rigidbody carRb;
    public RaycastWheel physicsWheel; // Must be linked in the Inspector!
    public SteeringSystem steeringSystem;

    [Header("Settings")]
    public float wheelRadius = 0.34f;
    [Tooltip("How fast the visual wheel snaps to the physics wheel. Higher = tighter.")]
    public float visualSmoothing = 30f;
    public bool steerable;
    [Range(0f, 2.5f)] public float steeringYawScale = 1.35f;
    public bool alignToPhysicsWheelCenter = true;
    public bool autoResolveWheelMesh = true;

    private float _rotationX;
    private Vector3 _initialPosition;
    private Quaternion _initialRotation;
    private Vector3 _initialPhysicsLocalPosition;
    private Vector3 _currentWorldPosition;
    private bool _hasPhysicsLocalPosition;

    void Awake()
    {
        ResolveReferences();
    }

    void Start()
    {
        ResolveReferences();

        if (wheelMesh != null)
        {
            // Store the Sketchfab offset
            _initialPosition = wheelMesh.localPosition;
            _initialRotation = wheelMesh.localRotation;
            _currentWorldPosition = wheelMesh.position;

            if (physicsWheel != null)
            {
                _initialPhysicsLocalPosition = physicsWheel.transform.InverseTransformPoint(wheelMesh.position);
                _hasPhysicsLocalPosition = true;
            }
        }
    }

    void LateUpdate()
    {
        ResolveReferences();
        if (wheelMesh == null || physicsWheel == null) return;

        _rotationX = Mathf.Repeat(
            _rotationX + physicsWheel.VisualSpinDegreesPerSecond * Time.deltaTime,
            360f);

        float positionBlend = 1f - Mathf.Exp(-Mathf.Max(0.01f, visualSmoothing) * Time.deltaTime);
        if (alignToPhysicsWheelCenter)
        {
            EnsurePhysicsLocalPosition();
            Vector3 targetWorldPosition = GetPhysicsWheelCenterWorld();
            _currentWorldPosition = targetWorldPosition;
            wheelMesh.position = _currentWorldPosition;
        }
        else
        {
            float targetY = GetLegacySuspensionOffset();
            Vector3 targetLocalPosition = _initialPosition + new Vector3(0f, targetY, 0f);
            wheelMesh.localPosition = Vector3.Lerp(wheelMesh.localPosition, targetLocalPosition, positionBlend);
        }

        float steerYaw = steerable && steeringSystem != null ? steeringSystem.CurrentSteerAngle * steeringYawScale : 0f;
        wheelMesh.localRotation = _initialRotation * Quaternion.Euler(_rotationX, steerYaw, 0f);
    }

    private Vector3 GetPhysicsWheelCenterWorld()
    {
        Vector3 localPosition = _initialPhysicsLocalPosition;

        if (physicsWheel.IsGrounded)
        {
            Vector3 centerWorld = physicsWheel.ContactPoint + physicsWheel.transform.up * physicsWheel.wheelRadius;
            float centerLocalY = physicsWheel.transform.InverseTransformPoint(centerWorld).y;
            localPosition.y = centerLocalY;
        }
        else
        {
            localPosition.y = -physicsWheel.suspensionLength;
        }

        return physicsWheel.transform.TransformPoint(localPosition);
    }

    private void EnsurePhysicsLocalPosition()
    {
        if (_hasPhysicsLocalPosition)
            return;

        _initialPhysicsLocalPosition = physicsWheel.transform.InverseTransformPoint(wheelMesh.position);
        _currentWorldPosition = wheelMesh.position;
        _hasPhysicsLocalPosition = true;
    }

    private float GetLegacySuspensionOffset()
    {
        if (physicsWheel.IsGrounded)
        {
            float suspOffset = (physicsWheel.SuspensionTravel - physicsWheel.restLengthRatio) * physicsWheel.suspensionLength;
            return -suspOffset;
        }

        return -(1f - physicsWheel.restLengthRatio) * physicsWheel.suspensionLength;
    }

    private void ResolveReferences()
    {
        if (physicsWheel == null)
            physicsWheel = GetComponent<RaycastWheel>();

        if (carRb == null)
            carRb = GetComponentInParent<Rigidbody>();

        if (steeringSystem == null)
            steeringSystem = GetComponentInParent<SteeringSystem>();

        if (!autoResolveWheelMesh)
            return;

        if (wheelMesh == null || !IsExpectedWheelMesh(wheelMesh))
        {
            Transform resolved = FindExpectedWheelMesh();
            if (resolved != null && resolved != wheelMesh)
            {
                wheelMesh = resolved;
                _hasPhysicsLocalPosition = false;
            }
        }
    }

    private Transform FindExpectedWheelMesh()
    {
        string suffix = GetVisualMeshSuffix();
        if (string.IsNullOrEmpty(suffix))
            return null;

        Transform root = carRb != null ? carRb.transform : transform.root;
        Transform[] candidates = root.GetComponentsInChildren<Transform>(true);
        string exactName = "WHEEL_" + suffix;

        for (int i = 0; i < candidates.Length; i++)
        {
            if (candidates[i].name.StartsWith(exactName, System.StringComparison.OrdinalIgnoreCase))
                return candidates[i];
        }

        for (int i = 0; i < candidates.Length; i++)
        {
            string candidateName = candidates[i].name;
            if (candidateName.IndexOf("wheel", System.StringComparison.OrdinalIgnoreCase) >= 0
                && candidateName.IndexOf(suffix, System.StringComparison.OrdinalIgnoreCase) >= 0)
                return candidates[i];
        }

        return null;
    }

    private bool IsExpectedWheelMesh(Transform candidate)
    {
        string suffix = GetVisualMeshSuffix();
        if (candidate == null || string.IsNullOrEmpty(suffix))
            return true;

        string candidateName = candidate.name;
        return candidateName.IndexOf("wheel", System.StringComparison.OrdinalIgnoreCase) >= 0
            && candidateName.IndexOf(suffix, System.StringComparison.OrdinalIgnoreCase) >= 0;
    }

    private string GetVisualMeshSuffix()
    {
        string wheelName = physicsWheel != null ? physicsWheel.name : name;
        switch (wheelName)
        {
            case "FL": return "LF";
            case "FR": return "RF";
            case "RL": return "LR";
            case "RR": return "RR";
            default: return null;
        }
    }
}
