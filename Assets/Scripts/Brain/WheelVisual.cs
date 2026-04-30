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
    [Range(0f, 1f)] public float steeringYawScale = 1f;

    private float _rotationX;
    private Vector3 _initialPosition;
    private Quaternion _initialRotation;
    private float _currentY;

    void Start()
    {
        if (steeringSystem == null)
            steeringSystem = GetComponentInParent<SteeringSystem>();

        if (wheelMesh != null)
        {
            // Store the Sketchfab offset
            _initialPosition = wheelMesh.localPosition;
            _initialRotation = wheelMesh.localRotation;
            _currentY = 0f;
        }
    }

    void Update()
    {
        if (wheelMesh == null || carRb == null || physicsWheel == null) return;

        // 1. CALCULATE TIRE SPIN
        Vector3 localVel = transform.InverseTransformDirection(carRb.GetPointVelocity(transform.position));
        float forwardSpeed = localVel.z;
        _rotationX += (forwardSpeed / (2f * Mathf.PI * wheelRadius)) * 360f * Time.deltaTime;

        // 2. CALCULATE TARGET SUSPENSION BOUNCE
        float targetY = 0f;
        if (physicsWheel.IsGrounded)
        {
            float suspOffset = (physicsWheel.SuspensionTravel - physicsWheel.restLengthRatio) * physicsWheel.suspensionLength;
            targetY = -suspOffset;
        }
        else
        {
            // Drop tire to full extension when in the air
            targetY = -(1f - physicsWheel.restLengthRatio) * physicsWheel.suspensionLength;
        }

        // stops the wheel from stuttering visually at high framerates 
        _currentY = Mathf.Lerp(_currentY, targetY, Time.deltaTime * visualSmoothing);

        // 4. COMBINE STEERING + SPIN + SMOOTH SUSPENSION
        float steerYaw = steerable && steeringSystem != null ? steeringSystem.CurrentSteerAngle * steeringYawScale : 0f;
        wheelMesh.localPosition = _initialPosition + new Vector3(0, _currentY, 0);
        wheelMesh.localRotation = _initialRotation * Quaternion.Euler(_rotationX, steerYaw, 0f);
    }
}
