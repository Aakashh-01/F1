using UnityEngine;

public class AdvancedBrakingSystem : MonoBehaviour
{
    [Header("References")]
    public Rigidbody rb;
    public TractionSystem tractionSystem;
    public WeightTransfer weightTransfer;
    public DrivetrainBrakeSystem drivetrain;

    [Header("Virtual Brake Pedal")]
    [Range(0.5f, 20f)] public float brakePressRate = 7.5f;
    [Range(0.5f, 20f)] public float brakeReleaseRate = 10f;
    public AnimationCurve brakePressureCurve = new AnimationCurve(
        new Keyframe(0f, 0f),
        new Keyframe(0.35f, 0.24f),
        new Keyframe(0.75f, 0.82f),
        new Keyframe(1f, 1f));

    [Header("Late Braking")]
    public AnimationCurve lateBrakeMultiplierBySpeed = new AnimationCurve(
        new Keyframe(0f, 0.88f),
        new Keyframe(120f, 1f),
        new Keyframe(220f, 1.12f),
        new Keyframe(320f, 1.18f));
    [Range(0.5f, 1.5f)] public float maxLateBrakeMultiplier = 1.18f;

    [Header("Lockup")]
    [Range(0.1f, 1.5f)] public float frontLockupThreshold = 0.96f;
    [Range(0.1f, 1.5f)] public float rearLockupThreshold = 0.84f;
    [Range(0.02f, 0.8f)] public float lockupBlendRange = 0.24f;
    [Range(0f, 0.8f)] public float maxLockupEfficiencyLoss = 0.36f;

    [Header("Trail Braking")]
    [Range(0f, 1f)] public float trailBrakeSupportStrength = 0.62f;
    [Range(0f, 0.25f)] public float trailBrakeFrontBiasShift = 0.08f;
    [Range(0f, 1f)] public float trailBrakeSteeringThreshold = 0.18f;
    [Range(0f, 220f)] public float trailBrakeMinSpeedKmh = 45f;

    [Header("Rear Instability")]
    [Range(0f, 1f)] public float rearInstabilityStrength = 0.36f;
    [Range(0f, 8000f)] public float maxRearInstabilityYawTorque = 2800f;
    [Range(0f, 220f)] public float rearInstabilityStartSpeedKmh = 80f;

    [HideInInspector] public float RawBrakeInput;
    [HideInInspector] public float VirtualBrakePressure;
    [HideInInspector] public float ShapedBrakePressure;
    [HideInInspector] public float EffectiveBrakePressure;
    [HideInInspector] public float LateBrakeMultiplier = 1f;
    [HideInInspector] public float FrontLockupAmount;
    [HideInInspector] public float RearLockupAmount;
    [HideInInspector] public float BrakeEfficiency = 1f;
    [HideInInspector] public float TrailBrakeBlend;
    [HideInInspector] public float RearInstabilityAmount;
    [HideInInspector] public float RearInstabilityYawTorque;
    [HideInInspector] public float DynamicFrontBrakeBias = 0.68f;

    public bool UseExternalSimulation { get; set; }

    private void Awake()
    {
        ResolveReferences();
    }

    private void FixedUpdate()
    {
        if (UseExternalSimulation)
            return;

        Simulate(null);
    }

    public void Simulate(VehiclePhysicsCoordinator coordinator)
    {
        ResolveReferences();

        float speedKmh = coordinator != null ? coordinator.SpeedKmh : GetSpeedKmh();
        float steeringInput = coordinator != null ? coordinator.SteeringInput : Input.GetAxisRaw("Horizontal");
        RawBrakeInput = coordinator != null ? coordinator.BrakeInput : Mathf.Max(0f, -Input.GetAxisRaw("Vertical"));
        float baseFrontBrakeBias = drivetrain != null ? drivetrain.frontBrakeBias : 0.68f;

        float rate = RawBrakeInput > VirtualBrakePressure ? brakePressRate : brakeReleaseRate;
        VirtualBrakePressure = Mathf.MoveTowards(
            VirtualBrakePressure,
            RawBrakeInput,
            rate * Time.fixedDeltaTime);

        ShapedBrakePressure = brakePressureCurve != null
            ? Mathf.Clamp01(brakePressureCurve.Evaluate(VirtualBrakePressure))
            : VirtualBrakePressure;

        LateBrakeMultiplier = Mathf.Min(
            maxLateBrakeMultiplier,
            lateBrakeMultiplierBySpeed != null ? lateBrakeMultiplierBySpeed.Evaluate(speedKmh) : 1f);
        LateBrakeMultiplier = Mathf.Max(0f, LateBrakeMultiplier);

        TrailBrakeBlend = CalculateTrailBrakeBlend(speedKmh, steeringInput, ShapedBrakePressure);
        DynamicFrontBrakeBias = Mathf.Clamp01(baseFrontBrakeBias + TrailBrakeBlend * trailBrakeFrontBiasShift);

        CalculateLockup(speedKmh, DynamicFrontBrakeBias);

        float lockupAmount = Mathf.Max(FrontLockupAmount, RearLockupAmount);
        BrakeEfficiency = Mathf.Clamp01(1f - lockupAmount * maxLockupEfficiencyLoss);
        EffectiveBrakePressure = ShapedBrakePressure * LateBrakeMultiplier * BrakeEfficiency;

        CalculateRearInstability(speedKmh, steeringInput);
    }

    public void ApplyProfile(AdvancedBrakeProfile profile)
    {
        if (profile == null)
            return;

        brakePressRate = profile.brakePressRate;
        brakeReleaseRate = profile.brakeReleaseRate;
        brakePressureCurve = profile.brakePressureCurve;
        lateBrakeMultiplierBySpeed = profile.lateBrakeMultiplierBySpeed;
        maxLateBrakeMultiplier = profile.maxLateBrakeMultiplier;
        frontLockupThreshold = profile.frontLockupThreshold;
        rearLockupThreshold = profile.rearLockupThreshold;
        lockupBlendRange = profile.lockupBlendRange;
        maxLockupEfficiencyLoss = profile.maxLockupEfficiencyLoss;
        trailBrakeSupportStrength = profile.trailBrakeSupportStrength;
        trailBrakeFrontBiasShift = profile.trailBrakeFrontBiasShift;
        trailBrakeSteeringThreshold = profile.trailBrakeSteeringThreshold;
        trailBrakeMinSpeedKmh = profile.trailBrakeMinSpeedKmh;
        rearInstabilityStrength = profile.rearInstabilityStrength;
        maxRearInstabilityYawTorque = profile.maxRearInstabilityYawTorque;
        rearInstabilityStartSpeedKmh = profile.rearInstabilityStartSpeedKmh;
    }

    private float CalculateTrailBrakeBlend(float speedKmh, float steeringInput, float pressure)
    {
        float steeringBlend = Mathf.InverseLerp(trailBrakeSteeringThreshold, 1f, Mathf.Abs(steeringInput));
        float speedBlend = Mathf.InverseLerp(trailBrakeMinSpeedKmh, trailBrakeMinSpeedKmh + 120f, speedKmh);
        return Mathf.Clamp01(pressure * steeringBlend * speedBlend * trailBrakeSupportStrength);
    }

    private void CalculateLockup(float speedKmh, float frontBias)
    {
        float frontSupport = GetAxleLoadSupport(true);
        float rearSupport = GetAxleLoadSupport(false);
        float speedPressure = Mathf.Lerp(0.92f, 1.08f, Mathf.InverseLerp(80f, 320f, speedKmh));
        float frontDemand = ShapedBrakePressure * LateBrakeMultiplier * speedPressure * Mathf.Lerp(0.88f, 1.16f, frontBias);
        float rearDemand = ShapedBrakePressure * LateBrakeMultiplier * speedPressure * Mathf.Lerp(1.16f, 0.88f, frontBias);

        float frontThreshold = frontLockupThreshold * frontSupport;
        float rearThreshold = rearLockupThreshold * rearSupport;

        FrontLockupAmount = Mathf.InverseLerp(frontThreshold, frontThreshold + lockupBlendRange, frontDemand);
        RearLockupAmount = Mathf.InverseLerp(rearThreshold, rearThreshold + lockupBlendRange, rearDemand);
    }

    private float GetAxleLoadSupport(bool front)
    {
        if (weightTransfer == null)
            return 1f;

        float frontLoad = Mathf.Max(0f, weightTransfer.FrontAxleLoad);
        float rearLoad = Mathf.Max(0f, weightTransfer.RearAxleLoad);
        float total = frontLoad + rearLoad;
        if (total <= 0.01f)
            return 1f;

        float ratio = front ? frontLoad / total : rearLoad / total;
        float neutral = front ? 0.52f : 0.48f;
        return Mathf.Clamp(ratio / neutral, front ? 0.85f : 0.65f, front ? 1.28f : 1.15f);
    }

    private void CalculateRearInstability(float speedKmh, float steeringInput)
    {
        float speedBlend = Mathf.InverseLerp(rearInstabilityStartSpeedKmh, rearInstabilityStartSpeedKmh + 160f, speedKmh);
        float steeringBlend = Mathf.Lerp(0.45f, 1f, Mathf.Abs(steeringInput));
        RearInstabilityAmount = Mathf.Clamp01(RearLockupAmount * ShapedBrakePressure * speedBlend * steeringBlend * rearInstabilityStrength);

        float yawSign = Mathf.Abs(steeringInput) > 0.05f ? Mathf.Sign(steeringInput) : Mathf.Sign(GetAverageRearSlip());
        if (Mathf.Approximately(yawSign, 0f))
            yawSign = 1f;

        RearInstabilityYawTorque = yawSign * RearInstabilityAmount * maxRearInstabilityYawTorque;
    }

    private float GetAverageRearSlip()
    {
        if (tractionSystem == null || tractionSystem.wheels == null)
            return 0f;

        float slip = 0f;
        int count = 0;
        for (int i = 2; i < 4 && i < tractionSystem.wheels.Length; i++)
        {
            RaycastWheel wheel = tractionSystem.wheels[i];
            if (wheel == null || !wheel.IsGrounded)
                continue;

            slip += wheel.LocalSlipVector.x;
            count++;
        }

        return count > 0 ? slip / count : 0f;
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

    private void ResolveReferences()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
        if (tractionSystem == null) tractionSystem = GetComponent<TractionSystem>();
        if (weightTransfer == null) weightTransfer = GetComponent<WeightTransfer>();
        if (drivetrain == null) drivetrain = GetComponent<DrivetrainBrakeSystem>();
    }
}
