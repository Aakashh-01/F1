using UnityEngine;

public enum SteeringAssistLevel
{
    Low,
    Medium,
    High
}

public enum TractionLossState
{
    None,
    FrontUndersteer,
    RearOversteer,
    FourWheelSlide
}

public class AdvancedSteeringAssist : MonoBehaviour
{
    [Header("References")]
    public TractionSystem tractionSystem;

    [Header("Assist Level")]
    public SteeringAssistLevel assistLevel = SteeringAssistLevel.Medium;

    [Header("Slip Detection")]
    [Range(1f, 30f)] public float rearSlipThresholdDegrees = 8f;
    [Range(1f, 30f)] public float frontSlipThresholdDegrees = 9f;
    [Range(2f, 45f)] public float severeSlipDegrees = 18f;
    [Range(0.2f, 1.5f)] public float tractionLossUtilisation = 0.92f;
    [Range(0f, 80f)] public float minimumAssistSpeedKmh = 18f;

    [Header("Correction")]
    [Range(0f, 1f)] public float recoveryStrength = 0.28f;
    [Range(0f, 1f)] public float countersteerStrength = 0.42f;
    [Range(0f, 1f)] public float understeerStrength = 0.18f;
    [Range(0f, 18f)] public float maxAssistAngle = 7f;
    [Range(0f, 1f)] public float opposingInputRetention = 0.42f;

    [Header("Smoothing")]
    [Range(0.01f, 0.8f)] public float assistSmoothingTime = 0.14f;
    [Range(0.01f, 0.8f)] public float mobileTapSmoothingTime = 0.12f;
    [Range(0.01f, 0.8f)] public float overrideBlendInTime = 0.18f;
    [Range(0.01f, 1.2f)] public float overrideBlendOutTime = 0.28f;

    [HideInInspector] public float RawAssistAngle;
    [HideInInspector] public float SmoothedAssistAngle;
    [HideInInspector] public float PlayerOverrideFactor;
    [HideInInspector] public float SmoothedPlayerIntent;
    [HideInInspector] public TractionLossState CurrentTractionLossState;

    private float _assistVelocity;
    private float _intentVelocity;
    private float _overrideVelocity;

    public bool UseExternalSimulation { get; set; }

    private void Awake()
    {
        if (tractionSystem == null)
            tractionSystem = GetComponent<TractionSystem>();
    }

    private void FixedUpdate()
    {
        if (UseExternalSimulation)
            return;

        Simulate(null);
    }

    public float Simulate(VehiclePhysicsCoordinator coordinator)
    {
        float speedKmh = coordinator != null ? coordinator.SpeedKmh : GetSpeedKmh();
        float steeringInput = coordinator != null ? coordinator.SteeringInput : Input.GetAxisRaw("Horizontal");
        float frontSlip = coordinator != null ? coordinator.AverageFrontSlipAngle : GetAverageSlip(0, 1);
        float rearSlip = coordinator != null ? coordinator.AverageRearSlipAngle : GetAverageSlip(2, 3);
        float averageGripUtilisation = GetAverageGripUtilisation();

        SmoothedPlayerIntent = Mathf.SmoothDamp(
            SmoothedPlayerIntent,
            steeringInput,
            ref _intentVelocity,
            mobileTapSmoothingTime,
            Mathf.Infinity,
            Time.fixedDeltaTime);

        RawAssistAngle = CalculateRawAssist(speedKmh, steeringInput, frontSlip, rearSlip, averageGripUtilisation);
        PlayerOverrideFactor = CalculatePlayerOverrideFactor(RawAssistAngle, SmoothedPlayerIntent);

        float protectedAssist = ApplyPlayerOverride(RawAssistAngle, SmoothedPlayerIntent, PlayerOverrideFactor);
        SmoothedAssistAngle = Mathf.SmoothDamp(
            SmoothedAssistAngle,
            protectedAssist,
            ref _assistVelocity,
            assistSmoothingTime,
            Mathf.Infinity,
            Time.fixedDeltaTime);

        return SmoothedAssistAngle;
    }

    public void ApplyProfile(SteeringAssistAdvancedProfile profile)
    {
        if (profile == null)
            return;

        assistLevel = profile.assistLevel;
        rearSlipThresholdDegrees = profile.rearSlipThresholdDegrees;
        frontSlipThresholdDegrees = profile.frontSlipThresholdDegrees;
        severeSlipDegrees = profile.severeSlipDegrees;
        tractionLossUtilisation = profile.tractionLossUtilisation;
        minimumAssistSpeedKmh = profile.minimumAssistSpeedKmh;
        recoveryStrength = profile.recoveryStrength;
        countersteerStrength = profile.countersteerStrength;
        understeerStrength = profile.understeerStrength;
        maxAssistAngle = profile.maxAssistAngle;
        opposingInputRetention = profile.opposingInputRetention;
        assistSmoothingTime = profile.assistSmoothingTime;
        mobileTapSmoothingTime = profile.mobileTapSmoothingTime;
        overrideBlendInTime = profile.overrideBlendInTime;
        overrideBlendOutTime = profile.overrideBlendOutTime;
    }

    private float CalculateRawAssist(float speedKmh, float steeringInput, float frontSlip, float rearSlip, float averageGripUtilisation)
    {
        CurrentTractionLossState = TractionLossState.None;

        if (speedKmh < minimumAssistSpeedKmh)
            return 0f;

        float rearAmount = SlipAmount(rearSlip, rearSlipThresholdDegrees);
        float frontAmount = SlipAmount(frontSlip, frontSlipThresholdDegrees);
        bool highUtilisation = averageGripUtilisation >= tractionLossUtilisation;

        if (rearAmount > 0.05f && frontAmount > 0.05f && highUtilisation)
            CurrentTractionLossState = TractionLossState.FourWheelSlide;
        else if (rearAmount > 0.05f)
            CurrentTractionLossState = TractionLossState.RearOversteer;
        else if (frontAmount > 0.05f)
            CurrentTractionLossState = TractionLossState.FrontUndersteer;

        float levelScale = GetAssistLevelScale();
        float rearCorrection = -rearSlip * countersteerStrength * recoveryStrength * rearAmount;
        float frontCorrection = 0f;

        if (frontAmount > 0.05f && Mathf.Abs(steeringInput) > 0.05f && Mathf.Sign(frontSlip) == Mathf.Sign(steeringInput))
            frontCorrection = -frontSlip * understeerStrength * frontAmount;

        float raw = (rearCorrection + frontCorrection) * levelScale;
        float cap = maxAssistAngle * levelScale;
        return Mathf.Clamp(raw, -cap, cap);
    }

    private float CalculatePlayerOverrideFactor(float rawAssistAngle, float smoothedIntent)
    {
        float targetOverride = 0f;
        if (Mathf.Abs(rawAssistAngle) > 0.01f && Mathf.Abs(smoothedIntent) > 0.01f)
        {
            float intentAmount = Mathf.InverseLerp(0.08f, 0.85f, Mathf.Abs(smoothedIntent));
            bool playerAlreadyRecovering = Mathf.Sign(smoothedIntent) == Mathf.Sign(rawAssistAngle);
            targetOverride = playerAlreadyRecovering ? intentAmount : intentAmount * 0.55f;
        }

        float smoothTime = targetOverride > PlayerOverrideFactor ? overrideBlendInTime : overrideBlendOutTime;
        return Mathf.SmoothDamp(
            PlayerOverrideFactor,
            targetOverride,
            ref _overrideVelocity,
            smoothTime,
            Mathf.Infinity,
            Time.fixedDeltaTime);
    }

    private float ApplyPlayerOverride(float rawAssistAngle, float smoothedIntent, float overrideFactor)
    {
        if (Mathf.Abs(rawAssistAngle) <= 0.01f)
            return 0f;

        bool playerAlreadyRecovering = Mathf.Abs(smoothedIntent) > 0.01f && Mathf.Sign(smoothedIntent) == Mathf.Sign(rawAssistAngle);
        float minimumRetention = playerAlreadyRecovering ? 0f : opposingInputRetention;
        float retention = Mathf.Lerp(1f, minimumRetention, Mathf.Clamp01(overrideFactor));
        return rawAssistAngle * retention;
    }

    private float SlipAmount(float slipDegrees, float threshold)
    {
        float severe = Mathf.Max(threshold + 0.1f, severeSlipDegrees);
        return Mathf.InverseLerp(threshold, severe, Mathf.Abs(slipDegrees));
    }

    private float GetAssistLevelScale()
    {
        switch (assistLevel)
        {
            case SteeringAssistLevel.Low:
                return 0.65f;
            case SteeringAssistLevel.High:
                return 1.35f;
            default:
                return 1f;
        }
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
        if (tractionSystem == null || tractionSystem.wheels == null || index < 0 || index >= tractionSystem.wheels.Length)
            return;

        RaycastWheel wheel = tractionSystem.wheels[index];
        if (wheel == null || !wheel.IsGrounded)
            return;

        slip += wheel.LocalSlipVector.x;
        count++;
    }

    private float GetAverageGripUtilisation()
    {
        if (tractionSystem == null || tractionSystem.GripUtilisation == null || tractionSystem.GripUtilisation.Length == 0)
            return 0f;

        float total = 0f;
        int count = 0;
        for (int i = 0; i < tractionSystem.GripUtilisation.Length; i++)
        {
            total += tractionSystem.GripUtilisation[i];
            count++;
        }

        return count > 0 ? total / count : 0f;
    }

    private float GetSpeedKmh()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb == null)
            return 0f;

#if UNITY_6000_0_OR_NEWER
        return rb.linearVelocity.magnitude * 3.6f;
#else
        return rb.velocity.magnitude * 3.6f;
#endif
    }
}
