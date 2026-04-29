using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class WeightTransfer : MonoBehaviour
{
    [Header("Wheel References (FL, FR, RL, RR)")]
    public RaycastWheel[] wheels = new RaycastWheel[4];

    [Header("System References")]
    public TractionSystem tractionSystem;
    public DownforceSystem downforceSystem;

    [Header("Mass Distribution")]
    [Range(0.1f, 0.6f)] public float comHeight = 0.27f;
    [Range(2.0f, 4.0f)] public float wheelbase = 3.0f;
    [Range(1.2f, 2.2f)] public float trackWidth = 1.8f;
    [Range(0.3f, 0.7f)] public float staticFrontBias = 0.45f;

    [Header("Weight Transfer Rates")]
    [Range(1f, 15f)] public float longTransferRate = 5f;
    [Range(1f, 15f)] public float latTransferRate = 6f;
    [Range(0.01f, 0.2f)] public float comSmoothing = 0.06f;

    [Header("Anti-Roll Bars")]
    [Range(0f, 150000f)] public float frontAntiRollStiffness = 90000f;
    [Range(0f, 150000f)] public float rearAntiRollStiffness = 70000f;

    [Header("CoM Travel Limits (metres, local space from static position)")]
    [Range(0f, 0.15f)] public float maxLongShift = 0.06f;
    [Range(0f, 0.10f)] public float maxLatShift = 0.04f;
    [Range(0f, 0.05f)] public float maxVertShift = 0.02f;

    [HideInInspector] public float FrontAxleLoad;
    [HideInInspector] public float RearAxleLoad;
    [HideInInspector] public float LeftSideLoad;
    [HideInInspector] public float RightSideLoad;
    [HideInInspector] public float LongTransferG;
    [HideInInspector] public float LatTransferG;
    [HideInInspector] public Vector3 CurrentCoMLocal;

    public bool UseExternalSimulation { get; set; }

    private Rigidbody _rb;
    private VehiclePhysicsCoordinator _coordinator;
    private Vector3 _staticCoMLocal;
    private Vector3 _targetCoMLocal;
    private Vector3 _comVelocity;
    private Vector3 _previousLocalVelocity;
    private float _longAccel;
    private float _latAccel;
    private float _accelSmoothVelLong;
    private float _accelSmoothVelLat;

    private const int FL = 0, FR = 1, RL = 2, RR = 3;

    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();

#if UNITY_6000_0_OR_NEWER
        _rb.automaticCenterOfMass = false;
        _rb.automaticInertiaTensor = false;
#endif

        ValidateReferences();
        InitializeCenterOfMass();
    }

    private void FixedUpdate()
    {
        if (UseExternalSimulation)
            return;

        Simulate(null);
    }

    public void Simulate(VehiclePhysicsCoordinator coordinator)
    {
        _coordinator = coordinator;
        float dt = Time.fixedDeltaTime;

        MeasureAccelerations(dt);
        ComputeTargetCoM();

        CurrentCoMLocal = Vector3.SmoothDamp(CurrentCoMLocal, _targetCoMLocal, ref _comVelocity, comSmoothing);
        _rb.centerOfMass = CurrentCoMLocal;

        ComputeAxleLoads();
        ApplyAntiRollTorque();
    }

    private void InitializeCenterOfMass()
    {
        if (wheels[0] == null || wheels[1] == null || wheels[2] == null || wheels[3] == null)
        {
            _staticCoMLocal = new Vector3(0f, comHeight, 0f);
        }
        else
        {
            float frontZ = (wheels[0].transform.localPosition.z + wheels[1].transform.localPosition.z) * 0.5f;
            float rearZ = (wheels[2].transform.localPosition.z + wheels[3].transform.localPosition.z) * 0.5f;
            float balancedZ = Mathf.Lerp(rearZ, frontZ, staticFrontBias);
            _staticCoMLocal = new Vector3(0f, comHeight, balancedZ);
        }

        _rb.centerOfMass = _staticCoMLocal;
        _targetCoMLocal = _staticCoMLocal;
        CurrentCoMLocal = _staticCoMLocal;
    }

    private void MeasureAccelerations(float dt)
    {
#if UNITY_6000_0_OR_NEWER
        Vector3 localVelocity = transform.InverseTransformDirection(_rb.linearVelocity);
#else
        Vector3 localVelocity = transform.InverseTransformDirection(_rb.velocity);
#endif
        Vector3 localAcceleration = (localVelocity - _previousLocalVelocity) / Mathf.Max(dt, 0.001f);
        _previousLocalVelocity = localVelocity;

        _longAccel = Mathf.SmoothDamp(_longAccel, localAcceleration.z, ref _accelSmoothVelLong, 0.05f);
        _latAccel = Mathf.SmoothDamp(_latAccel, localAcceleration.x, ref _accelSmoothVelLat, 0.05f);
    }

    private void ComputeTargetCoM()
    {
        float gravity = Mathf.Max(Physics.gravity.magnitude, 0.001f);

        float longShiftRaw = -(_longAccel * comHeight) / (gravity * wheelbase);
        float latShiftRaw = (_latAccel * comHeight) / (gravity * trackWidth);

        float longShift = Mathf.Clamp(longShiftRaw * longTransferRate, -maxLongShift, maxLongShift);
        float latShift = Mathf.Clamp(latShiftRaw * latTransferRate, -maxLatShift, maxLatShift);
        float vertShift = Mathf.Clamp(Mathf.Abs(latShift) * 0.3f, 0f, maxVertShift);

        _targetCoMLocal = _staticCoMLocal + new Vector3(latShift, vertShift, longShift);
        LongTransferG = longShift / Mathf.Max(maxLongShift, 0.001f);
        LatTransferG = latShift / Mathf.Max(maxLatShift, 0.001f);
    }

    private void ComputeAxleLoads()
    {
        float fl = wheels[FL] != null ? wheels[FL].NormalForce : 0f;
        float fr = wheels[FR] != null ? wheels[FR].NormalForce : 0f;
        float rl = wheels[RL] != null ? wheels[RL].NormalForce : 0f;
        float rr = wheels[RR] != null ? wheels[RR].NormalForce : 0f;

        float frontDown = downforceSystem != null ? downforceSystem.FrontDownforce : 0f;
        float rearDown = downforceSystem != null ? downforceSystem.RearDownforce : 0f;

        FrontAxleLoad = fl + fr + frontDown;
        RearAxleLoad = rl + rr + rearDown;
        LeftSideLoad = fl + rl;
        RightSideLoad = fr + rr;
    }

    private void ApplyAntiRollTorque()
    {
        ApplyAntiRollPair(FL, FR, frontAntiRollStiffness);
        ApplyAntiRollPair(RL, RR, rearAntiRollStiffness);
    }

    private void ApplyAntiRollPair(int leftIndex, int rightIndex, float stiffness)
    {
        RaycastWheel left = wheels[leftIndex];
        RaycastWheel right = wheels[rightIndex];

        if (left == null || right == null || !left.IsGrounded || !right.IsGrounded)
            return;

        float settleBlend = Mathf.Min(left.SettleProgress, right.SettleProgress);
        if (settleBlend <= 0f)
            return;

        float travelDifference = left.SuspensionTravel - right.SuspensionTravel;
        float antiRollForce = travelDifference * stiffness;
        antiRollForce *= settleBlend * settleBlend;
        antiRollForce = Mathf.Clamp(antiRollForce, -25000f, 25000f);

        AddForceAtPosition(transform.up * -antiRollForce, left.transform.position);
        AddForceAtPosition(transform.up * antiRollForce, right.transform.position);
    }

    private void AddForceAtPosition(Vector3 force, Vector3 position)
    {
        if (_coordinator != null)
            _coordinator.QueueForceAtPosition(force, position);
        else
            _rb.AddForceAtPosition(force, position);
    }

    public WeightTransferStats GetStats()
    {
        return new WeightTransferStats
        {
            frontAxleLoad = FrontAxleLoad,
            rearAxleLoad = RearAxleLoad,
            leftSideLoad = LeftSideLoad,
            rightSideLoad = RightSideLoad,
            longTransferG = LongTransferG,
            latTransferG = LatTransferG,
            comOffsetLocal = CurrentCoMLocal - _staticCoMLocal
        };
    }

    private void ValidateReferences()
    {
        if (tractionSystem == null) Debug.LogError("[WeightTransfer] TractionSystem not assigned.");
        if (downforceSystem == null) Debug.LogError("[WeightTransfer] DownforceSystem not assigned.");
        for (int i = 0; i < 4; i++)
            if (wheels[i] == null) Debug.LogError($"[WeightTransfer] Wheel [{i}] not assigned.");
    }

    private void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;

        Vector3 worldCoM = transform.TransformPoint(CurrentCoMLocal);
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(worldCoM, 0.08f);

        Vector3 staticCoMWorld = transform.TransformPoint(_staticCoMLocal);
        Gizmos.color = new Color(1f, 1f, 0f, 0.3f);
        Gizmos.DrawWireSphere(staticCoMWorld, 0.06f);

        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(staticCoMWorld, worldCoM);

        DrawAxleLoadBar(wheels[FL], wheels[FR], FrontAxleLoad, Color.green);
        DrawAxleLoadBar(wheels[RL], wheels[RR], RearAxleLoad, Color.magenta);
    }

    private void DrawAxleLoadBar(RaycastWheel left, RaycastWheel right, float load, Color color)
    {
        if (left == null || right == null) return;
        Vector3 mid = (left.transform.position + right.transform.position) * 0.5f;
        Gizmos.color = color;
        Gizmos.DrawLine(mid, mid + Vector3.up * (load * 0.000015f));
    }
}

[System.Serializable]
public struct WeightTransferStats
{
    public float frontAxleLoad;
    public float rearAxleLoad;
    public float leftSideLoad;
    public float rightSideLoad;
    public float longTransferG;
    public float latTransferG;
    public Vector3 comOffsetLocal;
}
