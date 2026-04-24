using UnityEngine;

/// Responsibilities:
///   1. Read slip vectors from all 4 RaycastWheel instances each FixedUpdate
///   2. Apply Pacejka Magic Formula (full B,C,D,E) per wheel
///   3. Scale grip by load sensitivity — more normal force = more grip,
///      but with diminishing returns (real tyre behaviour)
///   4. Handle combined slip (lateral + longitudinal interaction via friction ellipse)
///   5. Expose per-wheel grip coefficients for WeightTransfer and DebugPanel
/// Reads from:  RaycastWheel.LocalSlipVector, RaycastWheel.NormalForce
/// Outputs:     WheelGripForces[4], CombinedGripFL/FR/RL/RR
/// Applies:     Traction forces at each contact patch via Rigidbody

[RequireComponent(typeof(Rigidbody))]
public class TractionSystem : MonoBehaviour
{
    // ---------------------------------
    //  INSPECTOR — Wheel references
    // ---------------------------------
    [Header("Wheel References")]
    [Tooltip("Assign in order: FL, FR, RL, RR")]
    public RaycastWheel[] wheels = new RaycastWheel[4];

    // ------------------------------------------------------------------------
    //  INSPECTOR — Pacejka coefficients (shared across all 4 wheels for Phase 1)
    // -------------------------------------------------------------------------
    [Header("Pacejka Lateral (cornering)")]
    [Tooltip("Shape factor — controls curve width. Typical: 1.3–2.0")]
    [Range(1.0f, 2.5f)] public float lateralC = 1.5f;

    [Tooltip("Peak lateral grip factor. Scales with normal load via load sensitivity.")]
    [Range(0.5f, 2.5f)] public float lateralD = 1.6f;

    [Tooltip("Stiffness factor. Controls initial slope. Typical: 8–15")]
    [Range(4f, 20f)] public float lateralB = 10f;

    [Tooltip("Curvature factor — controls falloff past peak. Typical: 0.8–1.0")]
    [Range(0.5f, 1.2f)] public float lateralE = 0.97f;

    [Header("Pacejka Longitudinal (drive / brake)")]
    [Range(1.0f, 2.5f)] public float longC = 1.65f;
    [Range(0.5f, 2.5f)] public float longD = 1.8f;
    [Range(4f, 20f)] public float longB = 11f;
    [Range(0.5f, 1.2f)] public float longE = 0.97f;

    // ---------------------------------
    //  INSPECTOR — Load sensitivity
    // ---------------------------------
    [Header("Load Sensitivity")]
    [Tooltip("Reference normal force for grip scaling (N). " +
             "At this load, grip = D coefficient exactly. " +
             "Typical for F1 per-wheel: 3000–5000 N")]
    [Range(1000f, 8000f)] public float referenceLoad = 3500f;

    [Tooltip("Load sensitivity exponent. " +
             "< 1.0 = diminishing returns (realistic). " +
             "1.0 = perfectly linear. " +
             "Typical: 0.6–0.8")]
    [Range(0.4f, 1.0f)] public float loadSensitivity = 0.68f;

    // ---------------------------------
    //  INSPECTOR — Friction ellipse
    // ---------------------------------
    [Header("Friction Ellipse (combined slip)")]
    [Tooltip("Lateral grip circle radius weight. 1.0 = pure lateral, no reduction from long slip.")]
    [Range(0.5f, 1.5f)] public float lateralRadius = 1.0f;

    [Tooltip("Longitudinal grip circle radius weight.")]
    [Range(0.5f, 1.5f)] public float longRadius = 1.0f;

    // ----------------------------------------------------
    //  INSPECTOR — Speed bands (grip multiplier ramps)
    // ----------------------------------------------------
    [Header("Speed Band Grip Multipliers")]
    [Tooltip("Grip multiplier at low speed (< lowSpeedThreshold m/s). " +
             "Slight reduction simulates cold tyre at low speed.")]
    [Range(0.6f, 1.0f)] public float lowSpeedGripMult = 0.82f;

    [Tooltip("Low speed threshold (m/s).")]
    [Range(5f, 25f)] public float lowSpeedThreshold = 15f;

    [Tooltip("Grip multiplier at mid speed band (normal operating range).")]
    [Range(0.8f, 1.2f)] public float midSpeedGripMult = 1.0f;

    [Tooltip("High speed threshold (m/s). Above this, grip tapers slightly.")]
    [Range(40f, 90f)] public float highSpeedThreshold = 60f;

    [Tooltip("Grip multiplier above highSpeedThreshold. " +
             "Simulates tyres running toward thermal limit.")]
    [Range(0.7f, 1.0f)] public float highSpeedGripMult = 0.88f;

    // ------------------------------------------------------------------
    //  PUBLIC OUTPUTS  (read by WeightTransfer and PhysicsDebugPanel)
    // -----------------------------------------------------------------

    /// Final traction force vector applied per wheel (world space).
    [HideInInspector] public Vector3[] WheelGripForces = new Vector3[4];

    ///Combined grip utilisation per wheel (0 = no slip, 1 = at limit).
    [HideInInspector] public float[] GripUtilisation = new float[4];

    /// Effective lateral grip coefficient per wheel this frame.
    [HideInInspector] public float[] LateralGripCoeff = new float[4];

    /// Effective longitudinal grip coefficient per wheel this frame.
    [HideInInspector] public float[] LongGripCoeff = new float[4];

    // ----------------
    //  PRIVATE
    // ----------------
    private Rigidbody _rb;
    private float _speedMs;

    // Wheel index constants for readability
    private const int FL = 0, FR = 1, RL = 2, RR = 3;

    // =================
    //  LIFECYCLE
    // =================
    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();

        if (wheels.Length != 4)
            Debug.LogError("[TractionSystem] Exactly 4 wheels must be assigned (FL, FR, RL, RR).");

        for (int i = 0; i < 4; i++)
        {
            WheelGripForces[i] = Vector3.zero;
            GripUtilisation[i] = 0f;
            LateralGripCoeff[i] = 0f;
            LongGripCoeff[i] = 0f;
        }
    }

    private void FixedUpdate()
    {
#if UNITY_6000_0_OR_NEWER
        _speedMs = _rb.linearVelocity.magnitude;
#else
        _speedMs = _rb.velocity.magnitude;
#endif

        for (int i = 0; i < wheels.Length; i++)
        {
            RaycastWheel w = wheels[i];
            if (w == null || !w.IsGrounded) continue;

            ProcessWheel(i, w);
        }
    }

    // ==========================
    //  PER-WHEEL PROCESSING
    // ==========================
    private void ProcessWheel(int index, RaycastWheel w)
    {
        float normalForce = w.NormalForce;

        // If wheel is in the air, reset UI and do nothing
        if (normalForce <= 0f)
        {
            GripUtilisation[index] = 0f;
            return;
        }

        // -------------------------------------------------
        //  STATE 1: THE PARKING STATE (Speed < 1 m/s)
        //  Bypasses Pacejka completely to prevent divide-by-zero jitter.
        // --------------------------------------------------
        if (_speedMs < 1.0f)
        {
            // 1. Tell the Debug Panel that the tires are relaxed
            GripUtilisation[index] = 0f;
            LateralGripCoeff[index] = 0f;
            LongGripCoeff[index] = 0f;
            WheelGripForces[index] = Vector3.zero;

            // 2. Apply gentle Viscous Damping
            Vector3 contactVel = _rb.GetPointVelocity(w.transform.position);

            // Oppose the current velocity. Scale by normal force so it scales with car weight.            
            Vector3 dampForce = -contactVel * (normalForce * 0.3f);

            // Zero out the Y axis don't accidentally fight the suspension springs
            dampForce.y = 0f;

            // Apply at the wheel axle, NOT the floor (Anti-Wheelie)
            if (dampForce.sqrMagnitude > 0.001f)
                _rb.AddForceAtPosition(dampForce, w.transform.position);

            // EXIT EARLY Do not run the complex math below.
            return;
        }

        // ----------------------------------------------------
        //  STATE 2: THE DRIVING STATE (Speed >= 1 m/s)
        //  Full Pacejka Formula for F1 racing dynamics.
        // ----------------------------------------------------

        // Load-sensitive peak grip (diminishing returns)
        float loadRatio = normalForce / referenceLoad;
        float gripScale = Mathf.Pow(loadRatio, loadSensitivity);
        float speedMult = GetSpeedBandMultiplier(_speedMs);

        float effectiveLateralD = lateralD * gripScale * speedMult;
        float effectiveLongD = longD * gripScale * speedMult;

        // Pacejka per axis
        float slipAngle = w.LocalSlipVector.x;

        // TEMPORARY FIX: Force slip ratio to 0. 
        // This stops the car from thinking the brakes are locked while we use the BasicMotor!
        float slipRatio = 0f;

        float lateralCoeff = Pacejka(slipAngle, lateralB, lateralC, effectiveLateralD, lateralE);
        float longCoeff = Pacejka(slipRatio, longB, longC, effectiveLongD, longE);

        LateralGripCoeff[index] = lateralCoeff;
        LongGripCoeff[index] = longCoeff;

        // Friction ellipse (combined slip)
        float maxLateral = effectiveLateralD * normalForce * lateralRadius;
        float maxLong = effectiveLongD * normalForce * longRadius;

        float rawLateral = lateralCoeff * normalForce;
        float rawLong = longCoeff * normalForce;

        // Combined slip utilisation (0 = no load, 1 = at friction limit)
        float utilisation = 0f;
        if (maxLateral > 0f && maxLong > 0f)
        {
            float normLat = rawLateral / maxLateral;
            float normLong = rawLong / maxLong;
            utilisation = Mathf.Sqrt(normLat * normLat + normLong * normLong);
        }

        GripUtilisation[index] = utilisation;

        // If combined slip exceeds ellipse, reduce both forces proportionally
        float ellipseScale = 1f;
        if (utilisation > 1f)
            ellipseScale = 1f / utilisation;

        float finalLateral = rawLateral * ellipseScale;
        float finalLong = rawLong * ellipseScale;

        // World-space force vectors
        Vector3 wheelRight = w.transform.right;
        Vector3 wheelForward = w.transform.forward;

        // Lateral force: negative because slip angle is + when sliding outward
        Vector3 lateralForce = -wheelRight * finalLateral;
        Vector3 longForce = wheelForward * finalLong;

        Vector3 totalForce = lateralForce + longForce;
        WheelGripForces[index] = totalForce;

        // Apply traction at the wheel axle (Anti-Wheelie fix)
        if (totalForce.sqrMagnitude > 0.01f)
            _rb.AddForceAtPosition(totalForce, w.transform.position);
    }
    // ==========================
    //  PACEJKA FORMULA
    // ==========================
    
    /// Full Pacejka  Formula:  y = D * sin(C * atan(B*x - E*(B*x - atan(B*x))))
    ///
    /// Parameters:
    ///   x = slip input (angle in degrees for lateral, ratio for longitudinal)
    ///   B = stiffness factor  (controls initial slope steepness)
    ///   C = shape factor      (controls curve width / peak sharpness)
    ///   D = peak value        (maximum grip force coefficient)
    ///   E = curvature factor  (controls post-peak falloff)
    ///
    /// Returns a signed coefficient in approximately [-D, +D].
    
    private float Pacejka(float x, float B, float C, float D, float E)
    {
        float Bx = B * x;
        float inner = Bx - E * (Bx - Mathf.Atan(Bx));
        return D * Mathf.Sin(C * Mathf.Atan(inner));
    }

    // ===========================
    //  SPEED BAND MULTIPLIER
    // ===========================
  
    /// Returns a grip multiplier based on current speed.
    /// Low speed  → cold-tyre penalty
    /// Mid speed  → optimal operating window
    /// High speed → thermal limit taper
   
    private float GetSpeedBandMultiplier(float speed)
    {
        if (speed < lowSpeedThreshold)
        {
            float t = speed / lowSpeedThreshold;
            return Mathf.Lerp(lowSpeedGripMult, midSpeedGripMult, t);
        }
         
        // F1 cars should maintain 100% grip at high speeds due to aero efficiency.
        return midSpeedGripMult;
    }

    // ===========================================
    //  UTILITY — called by PhysicsDebugPanel
    // ==========================================
    public TractionStats GetStats()
    {
        float avgUtil = 0f;
        for (int i = 0; i < 4; i++) avgUtil += GripUtilisation[i];

        return new TractionStats
        {
            gripUtilFL = GripUtilisation[FL],
            gripUtilFR = GripUtilisation[FR],
            gripUtilRL = GripUtilisation[RL],
            gripUtilRR = GripUtilisation[RR],
            avgUtilisation = avgUtil * 0.25f,
            speedBandMult = GetSpeedBandMultiplier(_speedMs)
        };
    }

    // =========================================================
    //  GIZMOS — per-wheel grip utilisation bars in Scene view
    // =========================================================
    private void OnDrawGizmos()
    {
        for (int i = 0; i < wheels.Length; i++)
        {
            if (wheels[i] == null || !wheels[i].IsGrounded) continue;

            float util = GripUtilisation[i];

            // Colour: green → yellow → red as utilisation increases
            Color c = Color.Lerp(Color.green, Color.red, util);
            Gizmos.color = c;

            Vector3 from = wheels[i].ContactPoint;
            Vector3 to = from + Vector3.up * (util * 0.6f);
            Gizmos.DrawLine(from, to);
            Gizmos.DrawWireSphere(to, 0.04f);

            // Draw combined grip force direction
            if (WheelGripForces[i].sqrMagnitude > 1f)
            {
                Gizmos.color = new Color(1f, 0.9f, 0.2f, 0.6f);
                Gizmos.DrawLine(from, from + WheelGripForces[i].normalized * 0.4f);
            }
        }
    }
}


/// Plain data struct for PhysicsDebugPanel.
[System.Serializable]
public struct TractionStats
{
    public float gripUtilFL;
    public float gripUtilFR;
    public float gripUtilRL;
    public float gripUtilRR;
    public float avgUtilisation;
    public float speedBandMult;
}