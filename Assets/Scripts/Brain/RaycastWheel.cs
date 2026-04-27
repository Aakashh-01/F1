using UnityEngine;


/// Attach to each of the 4 wheel GameObjects (child of car root).
/// The car root must have a Rigidbody.  This script fires a suspension
/// ray each FixedUpdate, computes spring + damper forces, derives the
/// traction slip vector, and applies all forces directly to the Rigidbody.
///
/// Outputs read by TractionSystem.cs / WeightTransfer.cs:
///   IsGrounded, ContactPoint, ContactNormal, SuspensionTravel,
///   LocalSlipVector, NormalForce
[RequireComponent(typeof(WheelVisual))]   // swap for your rim mesh rotator
public class RaycastWheel : MonoBehaviour
{
    // ---------------------------
    //  INSPECTOR — Suspension
    // ---------------------------
    [Header("Suspension")]
    [Tooltip("Maximum suspension extension from rest (metres)")]
    [Range(0.05f, 0.5f)] public float suspensionLength = 0.25f;

    [Tooltip("Spring stiffness (N/m). " +
             "For 1000kg car / 4 wheels = 250kg per corner. " +
             "Natural freq = sqrt(k/m). At 80000: freq ≈ 17.9 rad/s = good F1 stiffness.")]
    [Range(10000f, 120000f)] public float springStiffness = 80000f;

    [Tooltip("Damper coefficient (N·s/m). " +
             "Critical damping = 2*sqrt(k*m_corner) = 2*sqrt(80000*250) = 8944. " +
             "Default 6000 = ~0.67x critical — slight underdamp for feel, no oscillation.")]
    [Range(1000f, 12000f)] public float damperStrength = 6000f;

    [Tooltip("Suspension rest position as a fraction of suspensionLength (0–1)")]
    [Range(0f, 1f)] public float restLengthRatio = 0.5f;

    // ---------------------
    //  INSPECTOR — Wheel
    // ---------------------
    [Header("Wheel")]
    [Range(0.25f, 0.55f)] public float wheelRadius = 0.34f;

    [Tooltip("Layers the suspension ray should hit (track surface only)")]
    public LayerMask groundLayers = ~0;

    // -------------------------------
    //  INSPECTOR — Safety limits
    // -------------------------------
    [Header("Safety Limits")]
    [Tooltip("Maximum upward velocity (m/s) the suspension force can produce in one tick. " +
             "Prevents first-frame collision explosion. Default 8 m/s is generous.")]
    [Range(2f, 20f)] public float maxSuspensionVelocity = 8f;

    [Tooltip("If true, suspension forces are disabled for the first N physics frames " +
             "after Awake to let the Rigidbody settle without exploding.")]
    public bool useSettleFrames = true;

    [Range(1, 20)] public int settleFrames = 8;

    [Header("Traction")]
    [Tooltip("Peak lateral grip coefficient (Pacejka Dy approximation)")]
    [Range(0.5f, 2.5f)] public float peakLateralGrip = 1.6f;

    [Tooltip("Peak longitudinal grip coefficient")]
    [Range(0.5f, 2.5f)] public float peakLongGrip = 1.8f;

    [Tooltip("Slip angle (degrees) at peak lateral grip")]
    [Range(2f, 12f)] public float peakSlipAngle = 6f;

    [Tooltip("Slip ratio at peak longitudinal grip")]
    [Range(0.05f, 0.3f)] public float peakSlipRatio = 0.12f;

    // ----------------------------------------------------------------------
    //  PUBLIC OUTPUTS  (read by TractionSystem / WeightTransfer each tick)
    // ----------------------------------------------------------------------
    [HideInInspector] public bool IsGrounded;
    [HideInInspector] public Vector3 ContactPoint;
    [HideInInspector] public Vector3 ContactNormal;
    [HideInInspector] public float SuspensionTravel;   // 0 = full droop, 1 = full bump
    [HideInInspector] public float NormalForce;        // Spring force magnitude (N)
    [HideInInspector] public Vector2 LocalSlipVector;    // x = lateral slip, y = longitudinal slip

    // --------------
    //  PRIVATE
    // --------------
    private Rigidbody _rb;
    private float _prevSuspTravel;
    private float _restLength;
    private int _frameCount;          // settle frame counter

    // Debug colours used by OnDrawGizmos
    private Color _gizmoSpring = new Color(0.2f, 1f, 0.4f, 0.8f);
    private Color _gizmoNoHit = new Color(1f, 0.3f, 0.3f, 0.5f);
    private Color _gizmoSlip = new Color(1f, 0.8f, 0f, 0.9f);

    // ================
    //  LIFECYCLE
    // ================
    private void Awake()
    {
        _rb = GetComponentInParent<Rigidbody>();
        _restLength = suspensionLength * restLengthRatio;
        _frameCount = 0;

        if (_rb == null)
        {
            Debug.LogError($"[RaycastWheel] No Rigidbody found in parent of {name}");
            return;
        }

        // Angular damping prevents the car tipping over at rest before
        // the anti-roll bars have enough force to counteract gravity torque.
        // 0.05 is Unity default — bump to 0.15 for stability.
        if (_rb.angularDamping < 0.15f)
            _rb.angularDamping = 0.15f;
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        _frameCount++;

        // 1. Suspension ray 
        Vector3 rayOrigin = transform.position;
        Vector3 rayDirection = -transform.up;
        float rayLength = suspensionLength + wheelRadius;

        IsGrounded = Physics.Raycast(
            rayOrigin, rayDirection, out RaycastHit hit, rayLength, groundLayers);

        if (!IsGrounded)
        {
            SuspensionTravel = 0f;
            NormalForce = 0f;
            LocalSlipVector = Vector2.zero;
            _prevSuspTravel = 0f;
            return;
        }

        // 2. Cache contact info 
        ContactPoint = hit.point;
        ContactNormal = hit.normal;

        float compression = (rayLength - hit.distance) / suspensionLength;
        SuspensionTravel = Mathf.Clamp01(compression);

        if (useSettleFrames && _frameCount <= settleFrames)
        {
            _prevSuspTravel = SuspensionTravel;
            NormalForce = 0f;
            return;
        }

        // 3. Spring + Damper force (REVERTED TO STABLE MATH) 
        float springDisplacement = (SuspensionTravel - restLengthRatio) * suspensionLength;
        float springForce = springStiffness * springDisplacement;

        // Convert normalized travel into physical meters before dividing by time
        float travelMetersDelta = (SuspensionTravel - _prevSuspTravel) * suspensionLength;
        float suspensionVelocity = travelMetersDelta / dt;

        // Reverted to simple, predictable damping (No weird multipliers)
        float damperForce = damperStrength * suspensionVelocity;

        float staticWeightPerWheel = (_rb.mass * Mathf.Abs(Physics.gravity.y)) / 4f;

        // mathematically prevents the suspension from exploding.
        float maxForcePerWheel = staticWeightPerWheel * 3f;
        NormalForce = Mathf.Clamp(springForce + damperForce, 0f, maxForcePerWheel);

        // 4. APPLY FORCE AT THE CORNER, NOT THE CENTER (FIXED TIPPING)
        //  push UP along the wheel's local up axis, and push AT the raycast origin
        Vector3 suspForce = transform.up * NormalForce;
        _rb.AddForceAtPosition(suspForce, transform.position);

        _prevSuspTravel = SuspensionTravel;

        //5. Wheel contact velocity & Slip 
        Vector3 contactVelWorld = _rb.GetPointVelocity(ContactPoint);
        float velForward = Vector3.Dot(contactVelWorld, transform.forward);
        float velRight = Vector3.Dot(contactVelWorld, transform.right);

        float speed = contactVelWorld.magnitude;
        float lateralSlipAngle = 0f;
        if (speed > 0.5f)
            lateralSlipAngle = Mathf.Rad2Deg * Mathf.Atan2(velRight, Mathf.Abs(velForward));

        float longitudinalSlip = -velForward / Mathf.Max(Mathf.Abs(velForward), 1f);

        LocalSlipVector = new Vector2(lateralSlipAngle, longitudinalSlip);
    }

    // ==============
    //  TRACTION
    // ==============

    /// <summary>
    /// Simplified Pacejka "Magic Formula" lateral + longitudinal force.
    /// Full Pacejka (B,C,D,E) coefficients will be exposed in TractionSystem.cs.
    /// This gives us grounded driving feel for Phase 1.1.
    /// </summary>
    

    /// <summary>
    /// Pacejka-inspired single-axis approximation.
    /// Returns a coefficient in [-peakGrip, +peakGrip] given a normalised slip input.
    /// Shape: rises to peak near |normalSlip| = 1, then falls slightly (combines well
    /// with the weight-transfer load sensitivity added in Phase 2).
    /// </summary>
    private float MagicFormula(float normSlip, float peakGrip)
    {
        // Simple sin-based Pacejka stand-in (C=1.9, E=0.97)
        float C = 1.9f;
        float E = 0.97f;
        float B = Mathf.PI / (2f * C);   // ensures peak at normSlip = 1
        float val = peakGrip * Mathf.Sin(C * Mathf.Atan(B * normSlip - E * (B * normSlip - Mathf.Atan(B * normSlip))));
        return val;
    }

    // ===============================
    //  VISUAL WHEEL POSITION SYNC
    // ===============================

    private void LateUpdate()
    {
        // Move the visual wheel mesh to match suspension travel.
        // Replace with your actual rim child reference.
        // (WheelVisual.cs handles mesh rotation in Phase 1.3)
        if (IsGrounded)
        {
            float suspOffset = (SuspensionTravel - restLengthRatio) * suspensionLength;
            // Push wheel down by suspension compression
            Vector3 localPos = transform.localPosition;
            localPos.y = -suspOffset;
            // Note: set on the *visual* child, not this transform
        }
    }

    // ==========================================
    //  DEBUG GIZMOS  (visible in Scene view)
    // ==========================================
    private void OnDrawGizmos()
    {
        float rayLength = suspensionLength + wheelRadius;

        if (IsGrounded)
        {
            // Spring force ray — green, length proportional to NormalForce
            Gizmos.color = _gizmoSpring;
            Gizmos.DrawLine(transform.position, ContactPoint);
            Gizmos.DrawWireSphere(ContactPoint, 0.04f);

            // Slip vector — yellow arrow
            if (LocalSlipVector.sqrMagnitude > 0.001f)
            {
                Gizmos.color = _gizmoSlip;
                Vector3 slipDir = (transform.right * LocalSlipVector.x * 0.01f +
                                   transform.forward * LocalSlipVector.y);
                Gizmos.DrawLine(ContactPoint, ContactPoint + slipDir * 0.5f);
            }
        }
        else
        {
            // No contact — red ray showing full extension
            Gizmos.color = _gizmoNoHit;
            Gizmos.DrawLine(transform.position, transform.position - transform.up * rayLength);
        }

        // Wheel radius circle
        Gizmos.color = new Color(1f, 1f, 1f, 0.2f);
        Gizmos.DrawWireSphere(
            transform.position - transform.up * (suspensionLength * (1f - SuspensionTravel) + wheelRadius),
            wheelRadius);
    }
}