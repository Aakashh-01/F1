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
    [Range(0f, 1f)] public float frontBrakeBias = 0.58f;
    public AnimationCurve brakeForceBySpeed = AnimationCurve.Linear(0f, 0.65f, 320f, 1f);

    [Header("Reverse")]
    public float reverseMaxSpeedKmh = 12f;

    [HideInInspector] public float CurrentThrottle;
    [HideInInspector] public float CurrentBrake;
    [HideInInspector] public float LastMotorForce;
    [HideInInspector] public float LastBrakeForce;

    public bool UseExternalSimulation { get; set; }

    private VehiclePhysicsCoordinator _coordinator;

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
        float targetBrake;

        if (coordinator != null)
        {
            targetThrottle = coordinator.ThrottleInput;
            targetBrake = coordinator.BrakeInput;
        }
        else
        {
            float vertical = Input.GetAxisRaw("Vertical");
            targetThrottle = Mathf.Max(0f, vertical);
            targetBrake = Mathf.Max(0f, -vertical);
        }

        CurrentThrottle = Mathf.MoveTowards(CurrentThrottle, targetThrottle, throttleSpoolSpeed * Time.fixedDeltaTime);
        CurrentBrake = Mathf.MoveTowards(CurrentBrake, targetBrake, brakeSpoolSpeed * Time.fixedDeltaTime);

        ApplyDrive(CurrentThrottle);
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

    private void ApplyBrake(float brake)
    {
        LastBrakeForce = 0f;

        if (brake <= 0.01f || rb == null)
            return;

        float speedKmh = GetSpeedKmh();
        float speedScale = brakeForceBySpeed != null ? brakeForceBySpeed.Evaluate(speedKmh) : 1f;
        float totalForce = brake * brakeForce * Mathf.Max(0f, speedScale);
        LastBrakeForce = totalForce;

#if UNITY_6000_0_OR_NEWER
        Vector3 velocity = rb.linearVelocity;
#else
        Vector3 velocity = rb.velocity;
#endif
        Vector3 planarVelocity = Vector3.ProjectOnPlane(velocity, transform.up);
        if (planarVelocity.sqrMagnitude < 0.05f)
            return;

        Vector3 brakeForceVector = -planarVelocity.normalized * totalForce;
        ApplyAxleDistributedForce(brakeForceVector, 1f - frontBrakeBias, false);
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
