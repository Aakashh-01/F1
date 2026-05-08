using UnityEngine;

[DefaultExecutionOrder(-100)]
[RequireComponent(typeof(Rigidbody))]
public class VehiclePhysicsCoordinator : MonoBehaviour
{
    [Header("Tuning")]
    public VehiclePhysicsProfile physicsProfile;
    public bool applyProfileOnAwake = true;

    [Header("Systems")]
    public Rigidbody rb;
    public RaycastWheel[] wheels = new RaycastWheel[4];
    public DrivetrainBrakeSystem drivetrain;
    public DownforceSystem downforce;
    public SteeringSystem steering;
    public TractionSystem traction;
    public WeightTransfer weightTransfer;

    [Header("Visual Body Alignment")]
    public Transform visualBody;
    public bool alignVisualBodyToSuspension = true;
    [Range(0f, 12f)] public float visualPitchFromSuspension = 1.8f;
    [Range(0f, 12f)] public float visualRollFromSuspension = 1.45f;
    [Range(0f, 6f)] public float visualPitchFromLoadTransfer = 1f;
    [Range(0f, 6f)] public float visualRollFromLoadTransfer = 0.75f;
    [Range(0f, 0.15f)] public float visualVerticalTravel = 0.024f;
    [Range(1f, 30f)] public float visualBodyPositionSharpness = 5.5f;
    [Range(1f, 30f)] public float visualBodyRotationSharpness = 4.6f;
    [Range(20f, 320f)] public float visualSuspensionFadeStartKmh = 120f;
    [Range(20f, 360f)] public float visualSuspensionFadeEndKmh = 220f;
    [Range(0f, 1f)] public float highSpeedSuspensionInfluence = 0.14f;
    [Range(0f, 2f)] public float highSpeedAeroPitchDegrees = 0.34f;
    [Range(0f, 0.05f)] public float highSpeedAeroDrop = 0.012f;

    [Header("Runtime State")]
    [SerializeField] private float speedMs;
    [SerializeField] private float speedKmh;
    [SerializeField] private float throttleInput;
    [SerializeField] private float brakeInput;
    [SerializeField] private float steeringInput;
    [SerializeField] private float averageFrontSlipAngle;
    [SerializeField] private float averageRearSlipAngle;

    private readonly ForceRequest[] _forceRequests = new ForceRequest[64];
    private int _forceRequestCount;
    private Vector3 _visualBodyBaseLocalPosition;
    private Quaternion _visualBodyBaseLocalRotation = Quaternion.identity;
    private Vector3 _visualBodyPositionVelocity;
    private Vector3 _visualBodyTargetLocalPosition;
    private Vector3 _visualBodyTargetVelocity;
    private float _visualBodyPitch;
    private float _visualBodyPitchVelocity;
    private float _visualBodyRoll;
    private float _visualBodyRollVelocity;
    private bool _visualBodyInitialized;

    public float SpeedMs => speedMs;
    public float SpeedKmh => speedKmh;
    public float ThrottleInput => throttleInput;
    public float BrakeInput => brakeInput;
    public float SteeringInput => steeringInput;
    public float AverageFrontSlipAngle => averageFrontSlipAngle;
    public float AverageRearSlipAngle => averageRearSlipAngle;

    private void Awake()
    {
        ResolveReferences();

        if (applyProfileOnAwake && physicsProfile != null)
            ApplyProfile(physicsProfile);

        AlignToGroundAtStartup();
        SetExternalSimulation(true);
        ValidateRigidbodyForHighSpeed();
    }

    private void OnDisable()
    {
        SetExternalSimulation(false);
    }

    private void Update()
    {
        float keyboardSteering = Input.GetAxisRaw("Horizontal");
        float keyboardVertical = Input.GetAxisRaw("Vertical");
        float keyboardThrottle = Mathf.Max(0f, keyboardVertical);
        float keyboardBrake = Mathf.Max(0f, -keyboardVertical);

        steeringInput = Mathf.Abs(keyboardSteering) > 0.01f ? keyboardSteering : MobileTouchControls.Steering;
        throttleInput = keyboardThrottle > 0.01f ? keyboardThrottle : MobileTouchControls.Throttle;
        brakeInput = keyboardBrake > 0.01f ? keyboardBrake : MobileTouchControls.Brake;
    }

    private void FixedUpdate()
    {
        RefreshState();
        _forceRequestCount = 0;

        for (int i = 0; i < wheels.Length; i++)
        {
            if (wheels[i] != null)
                wheels[i].Simulate(this);
        }

        if (drivetrain != null) drivetrain.Simulate(this);
        if (downforce != null) downforce.Simulate(this);
        if (steering != null) steering.Simulate(this);
        if (traction != null) traction.Simulate(this);
        if (weightTransfer != null) weightTransfer.Simulate(this);

        FlushForces();
    }

    private void LateUpdate()
    {
        UpdateVisualBodyAlignment();
    }

    public void QueueForce(Vector3 force, ForceMode mode = ForceMode.Force)
    {
        QueueForceAtPosition(force, rb.worldCenterOfMass, mode);
    }

    public void QueueForceAtPosition(Vector3 force, Vector3 position, ForceMode mode = ForceMode.Force)
    {
        if (force.sqrMagnitude <= 0.000001f || _forceRequestCount >= _forceRequests.Length)
            return;

        _forceRequests[_forceRequestCount++] = new ForceRequest
        {
            force = force,
            position = position,
            mode = mode
        };
    }

    public void RefreshState()
    {
        if (rb == null)
            return;

#if UNITY_6000_0_OR_NEWER
        Vector3 velocity = rb.linearVelocity;
#else
        Vector3 velocity = rb.velocity;
#endif
        speedMs = velocity.magnitude;
        speedKmh = speedMs * 3.6f;

        float frontSlip = 0f;
        float rearSlip = 0f;
        int frontCount = 0;
        int rearCount = 0;

        for (int i = 0; i < wheels.Length; i++)
        {
            RaycastWheel wheel = wheels[i];
            if (wheel == null || !wheel.IsGrounded)
                continue;

            if (i < 2)
            {
                frontSlip += wheel.LocalSlipVector.x;
                frontCount++;
            }
            else
            {
                rearSlip += wheel.LocalSlipVector.x;
                rearCount++;
            }
        }

        averageFrontSlipAngle = frontCount > 0 ? frontSlip / frontCount : 0f;
        averageRearSlipAngle = rearCount > 0 ? rearSlip / rearCount : 0f;
    }

    public void ApplyProfile(VehiclePhysicsProfile profile)
    {
        if (profile == null)
            return;

        if (wheels != null)
        {
            foreach (RaycastWheel wheel in wheels)
            {
                if (wheel == null) continue;
                wheel.suspensionLength = profile.suspension.suspensionLength;
                wheel.springStiffness = profile.suspension.springStiffness;
                wheel.damperStrength = profile.suspension.damperStrength;
                wheel.restLengthRatio = profile.suspension.restLengthRatio;
                wheel.wheelRadius = profile.suspension.physicsWheelRadius;
            }
        }

        if (traction != null) traction.ApplyProfile(profile.tireGrip);
        if (downforce != null) downforce.ApplyProfile(profile.aero);
        if (steering != null) steering.ApplyProfile(profile.steering);
        if (drivetrain != null) drivetrain.ApplyProfile(profile.drivetrain);
    }

    private void ResolveReferences()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
        if (drivetrain == null) drivetrain = GetComponent<DrivetrainBrakeSystem>();
        if (downforce == null) downforce = GetComponent<DownforceSystem>();
        if (steering == null) steering = GetComponent<SteeringSystem>();
        if (traction == null) traction = GetComponent<TractionSystem>();
        if (weightTransfer == null) weightTransfer = GetComponent<WeightTransfer>();
        if (visualBody == null)
        {
            Transform foundVisual = transform.Find("Car_Model");
            if (foundVisual != null)
                visualBody = foundVisual;
        }

        RaycastWheel[] foundWheels = GetComponentsInChildren<RaycastWheel>();
        AssignWheelByName(foundWheels, "FL", 0);
        AssignWheelByName(foundWheels, "FR", 1);
        AssignWheelByName(foundWheels, "RL", 2);
        AssignWheelByName(foundWheels, "RR", 3);

        for (int i = 0; i < Mathf.Min(wheels.Length, foundWheels.Length); i++)
        {
            if (wheels[i] == null)
                wheels[i] = foundWheels[i];
        }

        SyncSystemWheelReferences();
        InitializeVisualBodyReference();
    }

    private void AssignWheelByName(RaycastWheel[] foundWheels, string wheelName, int index)
    {
        if (foundWheels == null || index < 0 || index >= wheels.Length)
            return;

        for (int i = 0; i < foundWheels.Length; i++)
        {
            RaycastWheel wheel = foundWheels[i];
            if (wheel != null && wheel.name == wheelName)
            {
                wheels[index] = wheel;
                return;
            }
        }
    }

    private void SyncSystemWheelReferences()
    {
        if (drivetrain != null) drivetrain.wheels = wheels;
        if (traction != null) traction.wheels = wheels;
        if (weightTransfer != null) weightTransfer.wheels = wheels;

        if (steering != null)
        {
            steering.wheelFL = wheels.Length > 0 && wheels[0] != null ? wheels[0].transform : steering.wheelFL;
            steering.wheelFR = wheels.Length > 1 && wheels[1] != null ? wheels[1].transform : steering.wheelFR;
        }
    }

    private void SetExternalSimulation(bool enabled)
    {
        if (drivetrain != null) drivetrain.UseExternalSimulation = enabled;
        if (downforce != null) downforce.UseExternalSimulation = enabled;
        if (steering != null) steering.UseExternalSimulation = enabled;
        if (traction != null) traction.UseExternalSimulation = enabled;
        if (weightTransfer != null) weightTransfer.UseExternalSimulation = enabled;

        for (int i = 0; i < wheels.Length; i++)
        {
            if (wheels[i] != null)
                wheels[i].UseExternalSimulation = enabled;
        }
    }

    private void FlushForces()
    {
        for (int i = 0; i < _forceRequestCount; i++)
        {
            ForceRequest request = _forceRequests[i];
            rb.AddForceAtPosition(request.force, request.position, request.mode);
        }
    }

    private void ValidateRigidbodyForHighSpeed()
    {
        if (rb == null)
            return;

        if (rb.linearDamping > 0.05f)
            rb.linearDamping = 0.02f;

        Vector3 tensor = rb.inertiaTensor;
        if (tensor.x <= 0f || tensor.y <= 0f || tensor.z <= 0f)
        {
            tensor.x = Mathf.Max(tensor.x, rb.mass * 0.8f);
            tensor.y = Mathf.Max(tensor.y, rb.mass * 1.2f);
            tensor.z = Mathf.Max(tensor.z, rb.mass * 0.8f);
            rb.inertiaTensor = tensor;
            Debug.LogWarning($"[VehiclePhysicsCoordinator] Corrected invalid inertia tensor on {name}: {tensor}");
        }
    }

    private void InitializeVisualBodyReference()
    {
        if (visualBody == null || _visualBodyInitialized)
            return;

        _visualBodyBaseLocalPosition = visualBody.localPosition;
        _visualBodyBaseLocalRotation = visualBody.localRotation;
        _visualBodyTargetLocalPosition = _visualBodyBaseLocalPosition;
        _visualBodyInitialized = true;
    }

    private void UpdateVisualBodyAlignment()
    {
        if (!alignVisualBodyToSuspension || visualBody == null || !_visualBodyInitialized || wheels == null || wheels.Length < 4)
            return;

        float fl = GetCompressionDelta(0);
        float fr = GetCompressionDelta(1);
        float rl = GetCompressionDelta(2);
        float rr = GetCompressionDelta(3);

        float frontCompression = (fl + fr) * 0.5f;
        float rearCompression = (rl + rr) * 0.5f;
        float leftCompression = (fl + rl) * 0.5f;
        float rightCompression = (fr + rr) * 0.5f;
        float averageCompression = (fl + fr + rl + rr) * 0.25f;
        float speedT = Mathf.InverseLerp(
            visualSuspensionFadeStartKmh,
            Mathf.Max(visualSuspensionFadeStartKmh + 1f, visualSuspensionFadeEndKmh),
            speedKmh);
        float suspensionInfluence = Mathf.Lerp(1f, highSpeedSuspensionInfluence, speedT);

        float targetPitch = (frontCompression - rearCompression) * visualPitchFromSuspension * suspensionInfluence;
        float targetRoll = (leftCompression - rightCompression) * visualRollFromSuspension * suspensionInfluence;
        targetPitch += highSpeedAeroPitchDegrees * speedT;

        if (weightTransfer != null)
        {
            targetPitch += weightTransfer.LongTransferG * visualPitchFromLoadTransfer * Mathf.Lerp(1f, 0.65f, speedT);
            targetRoll += -weightTransfer.LatTransferG * visualRollFromLoadTransfer * Mathf.Lerp(1f, 0.55f, speedT);
        }

        float verticalOffset = (-averageCompression * visualVerticalTravel * suspensionInfluence) - (highSpeedAeroDrop * speedT);
        Vector3 rawTargetLocalPosition = _visualBodyBaseLocalPosition + Vector3.up * verticalOffset;

        float positionSmoothTime = 1f / Mathf.Max(0.01f, visualBodyPositionSharpness);
        float rotationSmoothTime = 1f / Mathf.Max(0.01f, visualBodyRotationSharpness);
        _visualBodyTargetLocalPosition = Vector3.SmoothDamp(
            _visualBodyTargetLocalPosition,
            rawTargetLocalPosition,
            ref _visualBodyTargetVelocity,
            positionSmoothTime,
            Mathf.Infinity,
            Time.deltaTime);
        _visualBodyPitch = Mathf.SmoothDampAngle(_visualBodyPitch, targetPitch, ref _visualBodyPitchVelocity, rotationSmoothTime);
        _visualBodyRoll = Mathf.SmoothDampAngle(_visualBodyRoll, targetRoll, ref _visualBodyRollVelocity, rotationSmoothTime);

        Quaternion targetLocalRotation = _visualBodyBaseLocalRotation * Quaternion.Euler(_visualBodyPitch, 0f, _visualBodyRoll);

        visualBody.localPosition = Vector3.SmoothDamp(
            visualBody.localPosition,
            _visualBodyTargetLocalPosition,
            ref _visualBodyPositionVelocity,
            positionSmoothTime,
            Mathf.Infinity,
            Time.deltaTime);
        visualBody.localRotation = targetLocalRotation;
    }

    private float GetCompressionDelta(int wheelIndex)
    {
        if (wheelIndex < 0 || wheelIndex >= wheels.Length || wheels[wheelIndex] == null || !wheels[wheelIndex].IsGrounded)
            return 0f;

        RaycastWheel wheel = wheels[wheelIndex];
        return Mathf.Clamp(wheel.SuspensionTravel - wheel.restLengthRatio, -1f, 1f);
    }

    private void AlignToGroundAtStartup()
    {
        if (rb == null || wheels == null || wheels.Length == 0)
            return;

        float totalAnchorDelta = 0f;
        int groundedSamples = 0;

        for (int i = 0; i < wheels.Length; i++)
        {
            RaycastWheel wheel = wheels[i];
            if (wheel == null || !wheel.TrySampleGround(out _, out float currentAnchorDistance))
                continue;

            float desiredAnchorDistance = wheel.GetRestAnchorDistance();
            totalAnchorDelta += currentAnchorDistance - desiredAnchorDistance;
            groundedSamples++;
        }

        if (groundedSamples < 2)
            return;

        float averageAnchorDelta = totalAnchorDelta / groundedSamples;
        if (Mathf.Abs(averageAnchorDelta) < 0.001f)
            return;

        averageAnchorDelta = Mathf.Clamp(averageAnchorDelta, -0.5f, 0.5f);
        Vector3 correction = -transform.up * averageAnchorDelta;
        rb.position += correction;
        Physics.SyncTransforms();
    }

    private struct ForceRequest
    {
        public Vector3 force;
        public Vector3 position;
        public ForceMode mode;
    }
}
