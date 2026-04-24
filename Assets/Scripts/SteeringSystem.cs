using UnityEngine;

public class SteeringSystem : MonoBehaviour
{
    [Header("References")]
    public Rigidbody rb;
    public TractionSystem tractionSystem; // To read slip for assist
    public Transform wheelFL;
    public Transform wheelFR;

    [Header("Steering Settings")]
    public float maxSteerAngle = 25f;
    public AnimationCurve speedSensitivityCurve; // High speed = lower multiplier

    [Header("Smoothing (Low-Pass)")]
    public float lowSpeedSmoothTime = 0.05f;
    public float highSpeedSmoothTime = 0.2f;
    public float speedThreshold = 50f; // Speed to transition to high smoothing

    [Header("Ackermann & Assist")]
    [Range(0, 0.2f)] public float ackermannFactor = 0.15f;
    [Range(0, 1f)] public float oversteerAssistStrength = 0.3f;
    public float slipThreshold = 8f; // Degrees of slip before assist kicks in

    private float _currentInput;
    private float _steeringVelocity;
    private float _clampedAngle;

    private float _targetInput;

    void Update()
    {
        // RULE 1: ALWAYS read input in Update never miss a quick key tap
        _targetInput = Input.GetAxisRaw("Horizontal");
    }

    void FixedUpdate()
    {
        // RULE 2: ALL smoothing and rotation MUST happen in FixedUpdate for Physics sync
#if UNITY_6000_0_OR_NEWER
        float currentSpeed = rb.linearVelocity.magnitude * 3.6f; // km/h
#else
        float currentSpeed = rb.velocity.magnitude * 3.6f; // km/h
#endif

        // 1. Calculate Dynamic Smoothing Time
        float t = Mathf.InverseLerp(0, speedThreshold, currentSpeed);
        float currentSmoothTime = Mathf.Lerp(lowSpeedSmoothTime, highSpeedSmoothTime, t);

        // 2. Smooth the Input (Forced to use fixed physics time)
        _currentInput = Mathf.SmoothDamp(
            _currentInput,
            _targetInput,
            ref _steeringVelocity,
            currentSmoothTime,
            Mathf.Infinity,
            Time.fixedDeltaTime 
        );

        // 3. Apply Speed Sensitivity Curve
        float sensitivity = speedSensitivityCurve.Evaluate(currentSpeed);
        float targetAngle = _currentInput * maxSteerAngle * sensitivity;

        // 4. Subtle Oversteer Assist
        targetAngle += CalculateCounterSteer(currentSpeed);

        ApplySteering(targetAngle);
    }

    float CalculateCounterSteer(float speed)
    {
        if (speed < 10f) return 0f; // No assist at walking speed

        // Average slip angle from rear wheels (Indices 2 and 3)
        float rearSlip = (tractionSystem.LateralGripCoeff[2] + tractionSystem.LateralGripCoeff[3]) / 2f;

        // If rear is sliding out significantly
        if (Mathf.Abs(rearSlip) > slipThreshold)
        {
            // Return a counter-force to nudge the wheels toward the slide
            return -rearSlip * oversteerAssistStrength;
        }
        return 0f;
    }

    void ApplySteering(float angle)
    {
        // Ackermann Logic: Inner wheel turns more than outer
        float angleInner = angle * (1f + ackermannFactor);
        float angleOuter = angle * (1f - ackermannFactor);

        // Apply to visuals/raycast origins
        wheelFL.localRotation = Quaternion.Euler(0, angle > 0 ? angleOuter : angleInner, 0);
        wheelFR.localRotation = Quaternion.Euler(0, angle > 0 ? angleInner : angleOuter, 0);
    }
}