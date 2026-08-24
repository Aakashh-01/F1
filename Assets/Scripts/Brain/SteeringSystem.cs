using UnityEngine;

public class SteeringSystem : MonoBehaviour
{
    [Header("References")]
    public Rigidbody rb;
    public TractionSystem tractionSystem;
    public AdvancedSteeringAssist advancedAssist;
    public Transform wheelFL;
    public Transform wheelFR;

    [Header("Steering Settings")]
    public float maxSteerAngle = 16f;
    public AnimationCurve speedSensitivityCurve = new AnimationCurve(
        new Keyframe(0f, 1f),
        new Keyframe(80f, 0.82f),
        new Keyframe(150f, 0.62f),
        new Keyframe(220f, 0.44f),
        new Keyframe(320f, 0.3f));

    [Header("Smoothing")]
    public float lowSpeedSmoothTime = 0.06f;
    public float highSpeedSmoothTime = 0.18f;
    public float speedThreshold = 220f;

    [Header("Ackermann & Assist")]
    [Range(0f, 0.2f)] public float ackermannFactor = 0.15f;

    // Superseded by AdvancedSteeringAssist — kept only for serialization
    // compatibility with existing prefabs and VehiclePhysicsProfile assets.
    // These are no longer read at runtime.
    [Range(0f, 1f)] public float oversteerAssistStrength = 0.3f;
    [Range(0f, 1f)] public float understeerAssistStrength = 0.15f;
    public float slipThreshold = 8f;

    [HideInInspector] public float CurrentSteerAngle;
    [HideInInspector] public float LastAssistAngle;

    public bool UseExternalSimulation { get; set; }

    private float _currentInput;
    private float _steeringVelocity;
    private float _targetInput;

    private void Awake()
    {
        if (rb == null)
            rb = GetComponent<Rigidbody>();
    }

    private void Update()
    {
        if (UseExternalSimulation)
            return;

        _targetInput = Input.GetAxisRaw("Horizontal");
    }

    private void FixedUpdate()
    {
        if (UseExternalSimulation)
            return;

        Simulate(null);
    }

    public void Simulate(VehiclePhysicsCoordinator coordinator)
    {
        if (coordinator != null)
            _targetInput = coordinator.SteeringInput;

        float currentSpeed = coordinator != null ? coordinator.SpeedKmh : GetSpeedKmh();
        float t = Mathf.InverseLerp(0f, speedThreshold, currentSpeed);
        float smoothTime = Mathf.Lerp(lowSpeedSmoothTime, highSpeedSmoothTime, t);

        _currentInput = Mathf.SmoothDamp(
            _currentInput,
            _targetInput,
            ref _steeringVelocity,
            smoothTime,
            Mathf.Infinity,
            Time.fixedDeltaTime);

        float sensitivity = speedSensitivityCurve != null ? speedSensitivityCurve.Evaluate(currentSpeed) : 1f;
        float targetAngle = _currentInput * maxSteerAngle * sensitivity;
        targetAngle += CalculateAssistAngle(coordinator, currentSpeed);
        targetAngle = Mathf.Clamp(targetAngle, -maxSteerAngle, maxSteerAngle);

        ApplySteering(targetAngle);
    }

    public void ApplyProfile(SteeringAssistProfile profile)
    {
        if (profile == null)
            return;

        maxSteerAngle = profile.maxSteerAngle;
        speedSensitivityCurve = profile.speedSensitivityCurve;
        lowSpeedSmoothTime = profile.lowSpeedSmoothTime;
        highSpeedSmoothTime = profile.highSpeedSmoothTime;
        speedThreshold = profile.speedThresholdKmh;
        ackermannFactor = profile.ackermannFactor;
        oversteerAssistStrength = profile.oversteerAssistStrength;
        understeerAssistStrength = profile.understeerAssistStrength;
        slipThreshold = profile.slipThresholdDegrees;
    }

    private float CalculateAssistAngle(VehiclePhysicsCoordinator coordinator, float speedKmh)
    {
        // AdvancedSteeringAssist is the single authoritative assist path.
        LastAssistAngle = advancedAssist != null ? advancedAssist.Simulate(coordinator) : 0f;
        return LastAssistAngle;
    }

    private void ApplySteering(float angle)
    {
        float angleInner = angle * (1f + ackermannFactor);
        float angleOuter = angle * (1f - ackermannFactor);

        if (wheelFL != null)
            wheelFL.localRotation = Quaternion.Euler(0f, angle > 0f ? angleOuter : angleInner, 0f);

        if (wheelFR != null)
            wheelFR.localRotation = Quaternion.Euler(0f, angle > 0f ? angleInner : angleOuter, 0f);

        CurrentSteerAngle = angle;
    }

    private float GetSpeedKmh()
    {
        if (rb == null)
            return 0f;

#if UNITY_6000_0_OR_NEWER
        return rb.linearVelocity.magnitude * 3.6f;
#else
        return rb.velocity.magnitude * 3.6f;
#endif
    }
}
