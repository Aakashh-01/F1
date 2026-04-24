using UnityEngine;


/// Responsibilities:
///   1. Compute aerodynamic downforce  F = 0.5 * rho * v² * Cl * A
///      (simplified: downforceCoeff * v²)
///   2. Apply front/rear downforce bias to simulate wing balance
///   3. Apply lateral stability drag at high speed to prevent unrealistic
///      sideslip (replaces missing aero model for Phase 1)
///   4. Expose all tuning values to the Inspector + PhysicsDebugPanel
///
/// Reads from:    Rigidbody.velocity
/// Outputs:       DownforceTotal, FrontDownforce, RearDownforce  (read by WeightTransfer)
/// Applies to:    Rigidbody directly via AddForceAtPosition

[RequireComponent(typeof(Rigidbody))]
public class DownforceSystem : MonoBehaviour
{
    // ----------------------------------
    //  INSPECTOR — Aero coefficients
    // ----------------------------------
    [Header("Aero Coefficients")]

    [Tooltip("Combined downforce coefficient (kg/m). " +
             "F_down = downforceCoeff * v². " +
             "Real F1 at 300 km/h ≈ 3× car weight.")]
    [Range(0.5f, 8f)] public float downforceCoeff = 3.5f;

    [Tooltip("Fraction of total downforce sent to front axle (0=all rear, 1=all front). " +
             "F1 neutral balance ≈ 0.36–0.40.")]
    [Range(0f, 1f)] public float frontBias = 0.38f;

    [Tooltip("Speed (m/s) at which full downforce is reached. " +
             "Scales the coefficient to avoid excessive downforce at low speed.")]
    [Range(20f, 100f)] public float fullDownforceSpeed = 70f;

    // ------------------------------------
    //  INSPECTOR — High-speed stability
    // ------------------------------------
    [Header("High-Speed Stability")]

    [Tooltip("Lateral drag coefficient. Resists sideways movement at speed. " +
             "Compensates for absent full aero model in Phase 1.")]
    [Range(0f, 5f)] public float lateralDragCoeff = 1.8f;

    [Tooltip("Speed (m/s) above which lateral drag ramps in fully.")]
    [Range(10f, 60f)] public float lateralDragOnsetSpeed = 20f;

    [Tooltip("Maximum lateral drag force (N) cap — prevents overcorrection.")]
    [Range(1000f, 40000f)] public float lateralDragCap = 18000f;

    // -----------------------------------
    //  INSPECTOR — Application points
    // -----------------------------------
    [Header("Force Application Points (local space)")]

    [Tooltip("Front wing downforce application point (local, relative to car root).")]
    public Vector3 frontDownforcePoint = new Vector3(0f, 0.1f, 1.6f);

    [Tooltip("Rear wing downforce application point (local, relative to car root).")]
    public Vector3 rearDownforcePoint = new Vector3(0f, 0.3f, -1.4f);

    // --------------------------------------------------------------
    //  PUBLIC OUTPUTS  (read by WeightTransfer.cs each FixedUpdate)
    // --------------------------------------------------------------
    [HideInInspector] public float DownforceTotal;
    [HideInInspector] public float FrontDownforce;
    [HideInInspector] public float RearDownforce;
    [HideInInspector] public float SpeedMs;          // cached for debug panel

    // ------------
    //  PRIVATE
    // ------------
    private Rigidbody _rb;

    // ================
    //  LIFECYCLE
    // ================
    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
#if UNITY_6000_0_OR_NEWER
        SpeedMs = _rb.linearVelocity.magnitude;
#else
        SpeedMs = _rb.velocity.magnitude;
#endif

        // 1. Downforce magnitude
        // F = downforceCoeff * v²
        // clamp the speed input so downforce grows realistically
        // from zero and saturates toward fullDownforceSpeed.
        float speedClamped = Mathf.Min(SpeedMs, fullDownforceSpeed);
        DownforceTotal = downforceCoeff * SpeedMs * SpeedMs;

        // 2. Front / rear split 
        FrontDownforce = DownforceTotal * frontBias;
        RearDownforce = DownforceTotal * (1f - frontBias);

        // 3. Apply at correct positions 
        Vector3 worldFrontPoint = transform.TransformPoint(frontDownforcePoint);
        Vector3 worldRearPoint = transform.TransformPoint(rearDownforcePoint);

        _rb.AddForceAtPosition(-transform.up * FrontDownforce, worldFrontPoint);
        _rb.AddForceAtPosition(-transform.up * RearDownforce, worldRearPoint);

        // 4. Lateral stability drag
        ApplyLateralDrag();
    }

    // =====================
    //  LATERAL STABILITY
    // =====================

    /// Applies a drag force opposing lateral (sideways) velocity.
    /// This simulates the aerodynamic resistance to sideslip that a real
    /// F1 car gets from its underbody and bodywork.
    /// Ramps in linearly from lateralDragOnsetSpeed to fullDownforceSpeed.
   
    private void ApplyLateralDrag()
    {
        // Lateral velocity in world space
#if UNITY_6000_0_OR_NEWER
        float lateralVel = Vector3.Dot(_rb.linearVelocity, transform.right);
#else
        float lateralVel = Vector3.Dot(_rb.velocity, transform.right);
#endif

        // Ramp factor: 0 at onset speed, 1 at fullDownforceSpeed
        float ramp = Mathf.InverseLerp(lateralDragOnsetSpeed, fullDownforceSpeed, SpeedMs);

        // INCREASE THIS: This is your "Anti-Skid" insurance.
        float stabilityFactor = 2.5f;

        // Drag force opposes lateral movement, scaled by speed²
        float dragMagnitude = Mathf.Clamp(
         lateralDragCoeff * lateralVel * lateralVel * stabilityFactor,
         -lateralDragCap,
         lateralDragCap);

        // Force direction: opposing the lateral velocity sign
        float dragSign = -Mathf.Sign(lateralVel);

        Vector3 dragForce = transform.right * dragSign * Mathf.Abs(dragMagnitude);
        _rb.AddForce(dragForce);
    }

    // ===============================================================
    //  UTILITY — called by PhysicsDebugPanel to get formatted stats
    // ===============================================================
    public DownforceStats GetStats()
    {
        return new DownforceStats
        {
            speedKmh = SpeedMs * 3.6f,
            totalDownforce = DownforceTotal,
            frontDownforce = FrontDownforce,
            rearDownforce = RearDownforce,
            frontBiasActual = (DownforceTotal > 0f) ? FrontDownforce / DownforceTotal : frontBias
        };
    }

    // ===============
    //  GIZMOS
    // ===============
    private void OnDrawGizmos()
    {
        // Front downforce arrow — cyan
        Gizmos.color = new Color(0f, 0.8f, 1f, 0.7f);
        Vector3 fwp = transform.TransformPoint(frontDownforcePoint);
        Gizmos.DrawSphere(fwp, 0.05f);
        Gizmos.DrawLine(fwp, fwp - transform.up * (FrontDownforce * 0.00005f));

        // Rear downforce arrow — magenta
        Gizmos.color = new Color(1f, 0.2f, 0.8f, 0.7f);
        Vector3 rwp = transform.TransformPoint(rearDownforcePoint);
        Gizmos.DrawSphere(rwp, 0.05f);
        Gizmos.DrawLine(rwp, rwp - transform.up * (RearDownforce * 0.00005f));
    }
}

/// Plain data struct — passed to PhysicsDebugPanel each frame.
[System.Serializable]
public struct DownforceStats
{
    public float speedKmh;
    public float totalDownforce;
    public float frontDownforce;
    public float rearDownforce;
    public float frontBiasActual;
}