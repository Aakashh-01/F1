using UnityEngine;

/// Responsibilities:
///   1. Compute dynamic centre-of-mass shift under acceleration, braking, cornering
///   2. Adjust per-axle load bias every FixedUpdate tick
///   3. Compose the final anti-roll torque to prevent unrealistic body roll
///   4. Apply a single consolidated Rigidbody.AddForce / AddTorque per tick
///      (all other scripts output forces; this script is the sole applier of
///       the composed result — prevents double-application conflicts)
///   5. Expose per-axle load distribution for PhysicsDebugPanel
///
/// Reads from:
///   RaycastWheel[4]   — NormalForce, IsGrounded, ContactPoint
///   TractionSystem    — WheelGripForces[4]
///   DownforceSystem   — FrontDownforce, RearDownforce, SpeedMs
///
/// Applies to:  Rigidbody (CoM position + anti-roll torque only —
///              suspension + traction forces are applied by their own scripts)

[RequireComponent(typeof(Rigidbody))]
public class WeightTransfer : MonoBehaviour
{
    // --------------------------------
    //  INSPECTOR — Wheel references
    // --------------------------------
    [Header("Wheel References (FL, FR, RL, RR)")]
    public RaycastWheel[] wheels = new RaycastWheel[4];

    // ---------------------------------------
    //  INSPECTOR — Sister system references
    // ---------------------------------------
    [Header("System References")]
    public TractionSystem tractionSystem;
    public DownforceSystem downforceSystem;

    // -------------------------------------
    //  INSPECTOR — Car mass distribution
    // -------------------------------------
    [Header("Mass Distribution")]
    [Tooltip("Centre of mass height above the wheel contact plane (metres). " +
             "Higher = more weight transfer under accel/brake. F1 ≈ 0.25–0.30 m.")]
    [Range(0.1f, 0.6f)] public float comHeight = 0.27f;

    [Tooltip("Wheelbase (metres) — front to rear axle distance.")]
    [Range(2.0f, 4.0f)] public float wheelbase = 3.0f;

    [Tooltip("Track width (metres) — left to right wheel distance.")]
    [Range(1.2f, 2.2f)] public float trackWidth = 1.8f;

    [Tooltip("Static front axle load fraction (0=all rear, 1=all front). " +
             "F1 ≈ 0.45 front, 0.55 rear.")]
    [Range(0.3f, 0.7f)] public float staticFrontBias = 0.45f;

    // -------------------------------
    //  INSPECTOR — Transfer rates
    // -------------------------------
    [Header("Weight Transfer Rates")]
    [Tooltip("How quickly the CoM shifts under longitudinal load (accel/brake). " +
             "Higher = snappier response. Range: 2–10.")]
    [Range(1f, 15f)] public float longTransferRate = 5f;

    [Tooltip("How quickly the CoM shifts under lateral load (cornering). " +
             "Higher = snappier response.")]
    [Range(1f, 15f)] public float latTransferRate = 6f;

    [Tooltip("Smoothing time for CoM position changes (seconds). " +
             "Prevents jitter. Range: 0.02–0.15.")]
    [Range(0.01f, 0.2f)] public float comSmoothing = 0.06f;

    // ---------------------------
    //  INSPECTOR — Anti-roll
    // ---------------------------
    [Header("Anti-Roll Bars")]
    [Tooltip("Front anti-roll bar stiffness (N·m per radian of body roll). " +
             "0 = no bar. F1 front ≈ 80 000–120 000.")]
    [Range(0f, 150000f)] public float frontAntiRollStiffness = 90000f;

    [Tooltip("Rear anti-roll bar stiffness. F1 rear ≈ 60 000–100 000.")]
    [Range(0f, 150000f)] public float rearAntiRollStiffness = 70000f;

    // ----------------------------
    //  INSPECTOR — CoM limit
    // ----------------------------
    [Header("CoM Travel Limits (metres, local space from static position)")]
    [Range(0f, 0.15f)] public float maxLongShift = 0.06f;   // fore-aft
    [Range(0f, 0.10f)] public float maxLatShift = 0.04f;   // left-right
    [Range(0f, 0.05f)] public float maxVertShift = 0.02f;   // up-down (squat/dive)

    // ---------------------------------------------------
    //  PUBLIC OUTPUTS  (PhysicsDebugPanel reads these)
    // ---------------------------------------------------
    [HideInInspector] public float FrontAxleLoad;    // N
    [HideInInspector] public float RearAxleLoad;     // N
    [HideInInspector] public float LeftSideLoad;     // N
    [HideInInspector] public float RightSideLoad;    // N
    [HideInInspector] public float LongTransferG;    // longitudinal transfer as fraction of total
    [HideInInspector] public float LatTransferG;     // lateral transfer as fraction of total
    [HideInInspector] public Vector3 CurrentCoMLocal; // current CoM in local space

    // ---------------------------
    //  PRIVATE
    // ---------------------------
    private Rigidbody _rb;
    private Vector3 _staticCoMLocal;       // CoM at rest (set in Awake)
    private Vector3 _targetCoMLocal;       // where physics wants CoM to be
    private Vector3 _comVelocity;          // SmoothDamp velocity ref

    private float _prevSpeedMs;
    private float _longAccel;            // smoothed longitudinal acceleration (m/s²)
    private float _latAccel;             // smoothed lateral acceleration (m/s²)
    private float _accelSmoothVelLong;
    private float _accelSmoothVelLat;

    private const int FL = 0, FR = 1, RL = 2, RR = 3;

    // ============================
    //  LIFECYCLE
    // ============================
    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();
 
        // tell Unity to stop auto-calculating CoM based on the box collider
#if UNITY_6000_0_OR_NEWER
        _rb.automaticCenterOfMass = false;
        _rb.automaticInertiaTensor = false;
#endif

        ValidateReferences();

        // Find the midpoint between axles
        float frontZ = (wheels[0].transform.localPosition.z + wheels[1].transform.localPosition.z) / 2f;
        float rearZ = (wheels[2].transform.localPosition.z + wheels[3].transform.localPosition.z) / 2f;

        // Force the CoM to the center of the wheels
        // 0.5f means dead-center. 0.45f means slightly toward the rear.
        float balancedZ = Mathf.Lerp(rearZ, frontZ, staticFrontBias);

        _rb.centerOfMass = new Vector3(0f, comHeight, balancedZ);

        _staticCoMLocal = _rb.centerOfMass;
        _targetCoMLocal = _staticCoMLocal;
        CurrentCoMLocal = _staticCoMLocal;

        Debug.Log($"[WeightTransfer] CoM Locked at: {_rb.centerOfMass}");
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // 1. Measure accelerations 
        MeasureAccelerations(dt);

        // 2. Compute target CoM shift 
        ComputeTargetCoM();

        // 3. Smooth CoM toward target 
        CurrentCoMLocal = Vector3.SmoothDamp(
            CurrentCoMLocal, _targetCoMLocal, ref _comVelocity, comSmoothing);

        _rb.centerOfMass = CurrentCoMLocal;

        // 4. Compute per-axle / per-side loads 
        ComputeAxleLoads();

        // 5. Anti-roll torque 
        ApplyAntiRollTorque();
    }

    //  ACCELERATION MEASUREMENT
private void MeasureAccelerations(float dt)
    {
        // Unity 6 renamed Rigidbody.velocity to linearVelocity
#if UNITY_6000_0_OR_NEWER
        float speedMs = _rb.linearVelocity.magnitude;
#else
        float speedMs = _rb.velocity.magnitude;
#endif

        // Longitudinal: forward/back acceleration in local space
#if UNITY_6000_0_OR_NEWER
        float rawLongAccel = Vector3.Dot(_rb.linearVelocity - _prevSpeedMs * transform.forward,
                                          transform.forward) / Mathf.Max(dt, 0.001f);
#else
        float rawLongAccel = Vector3.Dot(_rb.velocity - _prevSpeedMs * transform.forward,
                                          transform.forward) / Mathf.Max(dt, 0.001f);
#endif

        // Lateral: left/right acceleration (centripetal + sideslip)
        float rawLatAccel = Vector3.Dot(_rb.GetPointVelocity(transform.position),
                                         transform.right);

        // Smooth to avoid jitter from numerical noise
        _longAccel = Mathf.SmoothDamp(_longAccel, rawLongAccel, ref _accelSmoothVelLong, 0.05f);
        _latAccel = Mathf.SmoothDamp(_latAccel, rawLatAccel, ref _accelSmoothVelLat, 0.05f);

        _prevSpeedMs = speedMs;
    }

    // =========================
    //  TARGET COM CALCULATION
    // =========================

    /// Classic weight transfer formula:
    ///   ΔW_long = (m * a_long * h) / L      (fore-aft)
    ///   ΔW_lat  = (m * a_lat  * h) / T      (left-right)
    /// convert these force shifts into CoM displacements so the Rigidbody
    /// physics propagates them naturally to all dependent systems.
    
    private void ComputeTargetCoM()
    {
        float totalMass = _rb.mass;
        float g = Physics.gravity.magnitude;   // use project gravity setting

        // - Longitudinal shift -
        // Positive accel → CoM moves rearward (squat)
        // Negative accel (braking) → CoM moves forward (dive)
        float longForce = totalMass * _longAccel;
        float longShiftRaw = -(longForce * comHeight) / (totalMass * g * wheelbase);
        float longShift = Mathf.Clamp(longShiftRaw * longTransferRate, -maxLongShift, maxLongShift);

        // - Lateral shift -
        // Positive lat velocity (turning right) → CoM moves right
        float latForce = totalMass * _latAccel;
        float latShiftRaw = (latForce * comHeight) / (totalMass * g * trackWidth);
        float latShift = Mathf.Clamp(latShiftRaw * latTransferRate, -maxLatShift, maxLatShift);

        // - Vertical shift (squat / dive) -
        // Small vertical CoM rise under cornering (body roll effect)
        float vertShift = Mathf.Clamp(
            Mathf.Abs(latShift) * 0.3f, 0f, maxVertShift);

        // - Compose target -
        _targetCoMLocal = _staticCoMLocal + new Vector3(latShift, vertShift, longShift);

        // - Store normalised for debug panel -
        LongTransferG = longShift / Mathf.Max(maxLongShift, 0.001f);
        LatTransferG = latShift / Mathf.Max(maxLatShift, 0.001f);
    }

    // ========================
    //  AXLE LOAD COMPUTATION
    // ========================
    private void ComputeAxleLoads()
    {
        // Sum normal forces reported by each wheel
        float fl = wheels[FL] != null ? wheels[FL].NormalForce : 0f;
        float fr = wheels[FR] != null ? wheels[FR].NormalForce : 0f;
        float rl = wheels[RL] != null ? wheels[RL].NormalForce : 0f;
        float rr = wheels[RR] != null ? wheels[RR].NormalForce : 0f;

        // Add downforce contribution
        float frontDown = downforceSystem != null ? downforceSystem.FrontDownforce : 0f;
        float rearDown = downforceSystem != null ? downforceSystem.RearDownforce : 0f;

        FrontAxleLoad = fl + fr + frontDown;
        RearAxleLoad = rl + rr + rearDown;
        LeftSideLoad = fl + rl;
        RightSideLoad = fr + rr;
    }

    // ========================
    //  ANTI-ROLL TORQUE
    // ========================

    /// Anti-roll bar simulation.
    /// Computes the suspension travel difference between left and right wheels
    /// on each axle, then applies a corrective torque around the car's local
    /// forward axis to resist body roll.
   
    /// This is a force-based model (not constraint-based) — it works within
    /// the Rigidbody simulation without additional joints.
    
    private void ApplyAntiRollTorque()
    {
        // Front axle
        if (wheels[FL] != null && wheels[FR] != null)
        {
           // prevents massive one-sided explosions when landing.
            if (wheels[FL].IsGrounded && wheels[FR].IsGrounded)
            {
                float travelDiffFront = wheels[FL].SuspensionTravel - wheels[FR].SuspensionTravel;
                float antiRollForceFront = travelDiffFront * frontAntiRollStiffness;

                // Apply force at the wheel's local anchor (transform.position), NOT the ground (ContactPoint)
                _rb.AddForceAtPosition(transform.up * -antiRollForceFront, wheels[FL].transform.position);
                _rb.AddForceAtPosition(transform.up * antiRollForceFront, wheels[FR].transform.position);
            }
        }

        // Rear axle
        if (wheels[RL] != null && wheels[RR] != null)
        {
            if (wheels[RL].IsGrounded && wheels[RR].IsGrounded)
            {
                float travelDiffRear = wheels[RL].SuspensionTravel - wheels[RR].SuspensionTravel;
                float antiRollForceRear = travelDiffRear * rearAntiRollStiffness;

                _rb.AddForceAtPosition(transform.up * -antiRollForceRear, wheels[RL].transform.position);
                _rb.AddForceAtPosition(transform.up * antiRollForceRear, wheels[RR].transform.position);
            }
        }
    }

    // =================
    //  UTILITY
    // =================
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

    // ================
    //  GIZMOS
    // ================
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;

        // Draw current CoM as a sphere
        Vector3 worldCoM = transform.TransformPoint(CurrentCoMLocal);
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(worldCoM, 0.08f);

        // Draw static CoM reference
        Vector3 staticCoMWorld = transform.TransformPoint(_staticCoMLocal);
        Gizmos.color = new Color(1f, 1f, 0f, 0.3f);
        Gizmos.DrawWireSphere(staticCoMWorld, 0.06f);

        // Line from static to current (shows transfer direction)
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(staticCoMWorld, worldCoM);

        // Axle load bars
        DrawAxleLoadBar(wheels[FL], wheels[FR], FrontAxleLoad, Color.green);
        DrawAxleLoadBar(wheels[RL], wheels[RR], RearAxleLoad, Color.magenta);
    }

    private void DrawAxleLoadBar(RaycastWheel wL, RaycastWheel wR, float load, Color col)
    {
        if (wL == null || wR == null) return;
        Vector3 mid = (wL.transform.position + wR.transform.position) * 0.5f;
        Gizmos.color = col;
        Gizmos.DrawLine(mid, mid + Vector3.up * (load * 0.000015f));
    }
}


/// Plain data struct for PhysicsDebugPanel.
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