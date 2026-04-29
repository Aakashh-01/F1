using UnityEngine;

public class SteeringSystem : MonoBehaviour
{
    [Header("References")]
    public Rigidbody rb;
    public TractionSystem tractionSystem;
    public Transform wheelFL;
    public Transform wheelFR;

    [Header("Steering Settings")]
    public float maxSteerAngle = 25f;
    public AnimationCurve speedSensitivityCurve = AnimationCurve.Linear(0f, 1f, 300f, 0.1f);

    [Header("Smoothing")]
    public float lowSpeedSmoothTime = 0.05f;
    public float highSpeedSmoothTime = 0.2f;
    public float speedThreshold = 50f;

    [Header("Ackermann & Assist")]
    [Range(0f, 0.2f)] public float ackermannFactor = 0.15f;
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
        LastAssistAngle = 0f;
        if (speedKmh < 10f)
            return 0f;

        float rearSlip = coordinator != null ? coordinator.AverageRearSlipAngle : GetAverageSlip(2, 3);
        float frontSlip = coordinator != null ? coordinator.AverageFrontSlipAngle : GetAverageSlip(0, 1);

        if (Mathf.Abs(rearSlip) > slipThreshold)
        {
            LastAssistAngle = -rearSlip * oversteerAssistStrength;
            return LastAssistAngle;
        }

        if (Mathf.Abs(frontSlip) > slipThreshold && Mathf.Sign(frontSlip) == Mathf.Sign(_currentInput))
        {
            LastAssistAngle = -frontSlip * understeerAssistStrength;
            return LastAssistAngle;
        }

        return 0f;
    }

    private float GetAverageSlip(int leftIndex, int rightIndex)
    {
        if (tractionSystem == null || tractionSystem.wheels == null)
            return 0f;

        float slip = 0f;
        int count = 0;
        AddSlip(leftIndex, ref slip, ref count);
        AddSlip(rightIndex, ref slip, ref count);
        return count > 0 ? slip / count : 0f;
    }

    private void AddSlip(int index, ref float slip, ref int count)
    {
        if (index < 0 || index >= tractionSystem.wheels.Length)
            return;

        RaycastWheel wheel = tractionSystem.wheels[index];
        if (wheel == null || !wheel.IsGrounded)
            return;

        slip += wheel.LocalSlipVector.x;
        count++;
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
