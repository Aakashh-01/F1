using UnityEngine;

public class DrivetrainBrakeSystem : MonoBehaviour
{
    [Header("References")]
    public Rigidbody rb;
    public RaycastWheel[] wheels = new RaycastWheel[4];

    [Header("Motor")]
    public float motorForce = 85000f;
    public float throttleSpoolSpeed = 1.5f;
    [Range(0f, 1f)] public float rearDriveBias = 1f;
    [Range(0.5f, 2.5f)] public float tireForceLimitMultiplier = 1.6f;
    public AnimationCurve motorForceBySpeed = AnimationCurve.Linear(0f, 1f, 320f, 0.2f);

    [Header("Brakes")]
    public float brakeForce = 90000f;
    public float brakeSpoolSpeed = 6f;
    [Range(0f, 1f)] public float frontBrakeBias = 0.68f;
    public AnimationCurve brakeForceBySpeed = AnimationCurve.Linear(0f, 0.65f, 320f, 1f);

    [Header("Brake Stability")]
    [Range(0f, 1f)] public float brakeSteerStability = 0.75f;
    [Range(0f, 1f)] public float brakeSteerFrontBias = 0.78f;
    [Range(0f, 250f)] public float brakeSteerBlendSpeedKmh = 80f;

    [Header("Reverse")]
    public float reverseMaxSpeedKmh = 12f;

    [HideInInspector] public float CurrentThrottle;
    [HideInInspector] public float CurrentBrake;
    [HideInInspector] public float CurrentReverse;
    [HideInInspector] public float LastMotorForce;
    [HideInInspector] public float LastBrakeForce;
    [HideInInspector] public float LastReverseForce;

    public bool UseExternalSimulation { get; set; }

    private VehiclePhysicsCoordinator _coordinator;
    private const float ReverseForceMultiplier = 0.45f;

    protected virtual void Awake()
    {
        ResolveReferences();
    }

    protected virtual void FixedUpdate()
    {
        if (UseExternalSimulation)
            return;

        Simulate(null);
    }

    public void Simulate(VehiclePhysicsCoordinator coordinator)
    {
        _coordinator = coordinator;
        ResolveReferences();

        float targetThrottle;
        float targetBrakeInput;

        if (coordinator != null)
        {
            targetThrottle = coordinator.ThrottleInput;
            targetBrakeInput = coordinator.BrakeInput;
        }
        else
        {
            float vertical = Input.GetAxisRaw("Vertical");
            targetThrottle = Mathf.Max(0f, vertical);
            targetBrakeInput = Mathf.Max(0f, -vertical);
        }

        DetermineBrakeAndReverseTargets(targetThrottle, targetBrakeInput, out float targetBrake, out float targetReverse);

        CurrentThrottle = Mathf.MoveTowards(CurrentThrottle, targetThrottle, throttleSpoolSpeed * Time.fixedDeltaTime);
        CurrentBrake = Mathf.MoveTowards(CurrentBrake, targetBrake, brakeSpoolSpeed * Time.fixedDeltaTime);
        CurrentReverse = Mathf.MoveTowards(CurrentReverse, targetReverse, throttleSpoolSpeed * Time.fixedDeltaTime);

        ApplyDrive(CurrentThrottle);
        ApplyReverse(CurrentReverse);
        ApplyBrake(CurrentBrake);
    }

    public void ApplyProfile(DrivetrainBrakeProfile profile)
    {
        if (profile == null)
            return;

        motorForce = profile.motorForce;
        brakeForce = profile.brakeForce;
        throttleSpoolSpeed = profile.throttleSpoolSpeed;
        brakeSpoolSpeed = profile.brakeSpoolSpeed;
        rearDriveBias = profile.rearDriveBias;
        frontBrakeBias = profile.frontBrakeBias;
        brakeSteerStability = profile.brakeSteerStability;
        brakeSteerFrontBias = profile.brakeSteerFrontBias;
        brakeSteerBlendSpeedKmh = profile.brakeSteerBlendSpeedKmh;
        reverseMaxSpeedKmh = profile.reverseMaxSpeedKmh;
        motorForceBySpeed = profile.motorForceBySpeed;
        brakeForceBySpeed = profile.brakeForceBySpeed;
    }

    private void ApplyDrive(float throttle)
    {
        LastMotorForce = 0f;

        if (throttle <= 0.01f || rb == null)
            return;

        float speedKmh = GetSpeedKmh();
        float speedScale = motorForceBySpeed != null ? motorForceBySpeed.Evaluate(speedKmh) : 1f;
        float totalForce = throttle * motorForce * Mathf.Max(0f, speedScale);
        LastMotorForce = totalForce;

        Vector3 force = transform.forward * totalForce;
        ApplyAxleDistributedForce(force, rearDriveBias, true);
    }

    private void ApplyReverse(float reverse)
    {
        LastReverseForce = 0f;

        if (reverse <= 0.01f || rb == null)
            return;

        float forwardSpeedKmh = GetSignedForwardSpeedKmh();
        if (forwardSpeedKmh <= -Mathf.Abs(reverseMaxSpeedKmh))
            return;

        float speedScale = motorForceBySpeed != null ? motorForceBySpeed.Evaluate(Mathf.Abs(forwardSpeedKmh)) : 1f;
        float totalForce = reverse * motorForce * Mathf.Max(0f, speedScale) * ReverseForceMultiplier;
        LastReverseForce = totalForce;

        Vector3 force = -transform.forward * totalForce;
        ApplyAxleDistributedForce(force, rearDriveBias, true);
    }

    private void ApplyBrake(float brake)
    {
        LastBrakeForce = 0f;

        if (brake <= 0.01f || rb == null)
            return;

        float speedKmh = GetSpeedKmh();
        float speedScale = brakeForceBySpeed != null ? brakeForceBySpeed.Evaluate(speedKmh) : 1f;
        float totalForce = brake * brakeForce * Mathf.Max(0f, speedScale);
        float steeringAmount = GetSteeringAmount();
        float stabilityBlend = Mathf.Clamp01(steeringAmount * Mathf.InverseLerp(0f, brakeSteerBlendSpeedKmh, speedKmh));
        totalForce *= 1f - (brakeSteerStability * stabilityBlend);
        LastBrakeForce = totalForce;

#if UNITY_6000_0_OR_NEWER
        Vector3 velocity = rb.linearVelocity;
#else
        Vector3 velocity = rb.velocity;
#endif
        Vector3 planarVelocity = Vector3.ProjectOnPlane(velocity, transform.up);
        if (planarVelocity.sqrMagnitude < 0.05f)
            return;

        Vector3 brakeForceVector = CalculateBrakeForceVector(planarVelocity, totalForce);
        if (brakeForceVector.sqrMagnitude <= 0.0001f)
            return;

        float stableFrontBias = Mathf.Lerp(frontBrakeBias, brakeSteerFrontBias, stabilityBlend);
        ApplyAxleDistributedForce(brakeForceVector, 1f - stableFrontBias, false);
    }

    private Vector3 CalculateBrakeForceVector(Vector3 planarVelocity, float totalForce)
    {
        float forwardSpeed = Vector3.Dot(planarVelocity, transform.forward);
        if (Mathf.Abs(forwardSpeed) < 0.1f)
            return Vector3.zero;

        return -Mathf.Sign(forwardSpeed) * transform.forward * totalForce;
    }

    private void ApplyAxleDistributedForce(Vector3 force, float rearBias, bool rearDriveOnly)
    {
        Vector3 frontForce = force * (1f - rearBias);
        Vector3 rearForce = force * rearBias;

        if (rearDriveOnly)
            frontForce = Vector3.zero;

        ApplyForceAtWheelPair(0, 1, frontForce);
        ApplyForceAtWheelPair(2, 3, rearForce);
    }

    private void ApplyForceAtWheelPair(int leftIndex, int rightIndex, Vector3 force)
    {
        if (force.sqrMagnitude <= 0.0001f)
            return;

        int groundedCount = CountGroundedWheels(leftIndex, rightIndex);
        if (groundedCount == 0)
            return;

        Vector3 perWheel = force / groundedCount;
        ApplyForceAtWheel(leftIndex, perWheel);
        ApplyForceAtWheel(rightIndex, perWheel);
    }

    private void ApplyForceAtWheel(int index, Vector3 force)
    {
        if (wheels == null || index < 0 || index >= wheels.Length || wheels[index] == null || !wheels[index].IsGrounded)
            return;

        RaycastWheel wheel = wheels[index];
        float maxForce = wheel.NormalForce * tireForceLimitMultiplier;
        if (maxForce <= 0f)
            return;

        if (force.magnitude > maxForce)
            force = force.normalized * maxForce;

        Vector3 position = wheel.ContactPoint;

        if (_coordinator != null)
            _coordinator.QueueForceAtPosition(force, position);
        else
            rb.AddForceAtPosition(force, position);
    }

    private int CountGroundedWheels(int leftIndex, int rightIndex)
    {
        int count = 0;
        if (IsWheelGrounded(leftIndex)) count++;
        if (IsWheelGrounded(rightIndex)) count++;
        return count;
    }

    private bool IsWheelGrounded(int index)
    {
        return wheels != null
            && index >= 0
            && index < wheels.Length
            && wheels[index] != null
            && wheels[index].IsGrounded;
    }

    private float GetSpeedKmh()
    {
        if (_coordinator != null)
            return _coordinator.SpeedKmh;

        if (rb == null)
            return 0f;

#if UNITY_6000_0_OR_NEWER
        return rb.linearVelocity.magnitude * 3.6f;
#else
        return rb.velocity.magnitude * 3.6f;
#endif
    }

    private float GetSteeringAmount()
    {
        if (_coordinator != null)
            return Mathf.Abs(_coordinator.SteeringInput);

        return Mathf.Abs(Input.GetAxisRaw("Horizontal"));
    }

    private void DetermineBrakeAndReverseTargets(float throttleInput, float brakeInput, out float brakeTarget, out float reverseTarget)
    {
        brakeTarget = 0f;
        reverseTarget = 0f;

        if (!ShouldUseReverse(brakeInput, throttleInput, GetSignedForwardSpeedKmh()))
        {
            brakeTarget = brakeInput;
            return;
        }

        reverseTarget = brakeInput;
    }

    private bool ShouldUseReverse(float brakeInput, float throttleInput, float forwardSpeedKmh)
    {
        if (brakeInput <= 0.01f || throttleInput > 0.01f)
            return false;

        return forwardSpeedKmh <= 2f;
    }

    private float GetSignedForwardSpeedKmh()
    {
        if (rb == null)
            return 0f;

#if UNITY_6000_0_OR_NEWER
        Vector3 velocity = rb.linearVelocity;
#else
        Vector3 velocity = rb.velocity;
#endif
        return Vector3.Dot(velocity, transform.forward) * 3.6f;
    }

    private void ResolveReferences()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();

        RaycastWheel[] found = GetComponentsInChildren<RaycastWheel>();
        AssignWheelByName(found, "FL", 0);
        AssignWheelByName(found, "FR", 1);
        AssignWheelByName(found, "RL", 2);
        AssignWheelByName(found, "RR", 3);

        for (int i = 0; i < Mathf.Min(wheels.Length, found.Length); i++)
        {
            if (wheels[i] == null)
                wheels[i] = found[i];
        }
    }

    private void AssignWheelByName(RaycastWheel[] found, string wheelName, int index)
    {
        if (found == null || index < 0 || index >= wheels.Length)
            return;

        for (int i = 0; i < found.Length; i++)
        {
            RaycastWheel wheel = found[i];
            if (wheel != null && wheel.name == wheelName)
            {
                wheels[index] = wheel;
                return;
            }
        }
    }
}
