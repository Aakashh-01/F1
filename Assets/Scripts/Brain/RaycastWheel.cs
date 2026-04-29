using UnityEngine;

[RequireComponent(typeof(WheelVisual))]
public class RaycastWheel : MonoBehaviour
{
    [Header("Suspension")]
    [Range(0.05f, 0.5f)] public float suspensionLength = 0.25f;
    [Range(10000f, 120000f)] public float springStiffness = 80000f;
    [Range(1000f, 12000f)] public float damperStrength = 6000f;
    [Range(0f, 1f)] public float restLengthRatio = 0.5f;
    [Range(0f, 1.5f)] public float rayStartOffsetRatio = 1f;
    [Range(1f, 12f)] public float groundProbeHeight = 6f;

    [Header("Wheel")]
    [Range(0.25f, 0.55f)] public float wheelRadius = 0.34f;
    public LayerMask groundLayers = ~0;

    [Header("Safety Limits")]
    [Range(2f, 20f)] public float maxSuspensionVelocity = 8f;
    public bool useSettleFrames = true;
    [Range(1, 20)] public int settleFrames = 8;

    [Header("Legacy Traction Values")]
    [Range(0.5f, 2.5f)] public float peakLateralGrip = 1.6f;
    [Range(0.5f, 2.5f)] public float peakLongGrip = 1.8f;
    [Range(2f, 12f)] public float peakSlipAngle = 6f;
    [Range(0.05f, 0.3f)] public float peakSlipRatio = 0.12f;

    [HideInInspector] public bool IsGrounded;
    [HideInInspector] public Vector3 ContactPoint;
    [HideInInspector] public Vector3 ContactNormal;
    [HideInInspector] public float SuspensionTravel;
    [HideInInspector] public float NormalForce;
    [HideInInspector] public Vector2 LocalSlipVector;
    [HideInInspector] public bool IsSettled;
    [HideInInspector] public float SettleProgress;

    public bool UseExternalSimulation { get; set; }

    private Rigidbody _rb;
    private float _prevSuspTravel;
    private int _frameCount;
    private bool _hasPreviousSuspensionTravel;
    private readonly RaycastHit[] _groundHits = new RaycastHit[8];

    private readonly Color _gizmoSpring = new Color(0.2f, 1f, 0.4f, 0.8f);
    private readonly Color _gizmoNoHit = new Color(1f, 0.3f, 0.3f, 0.5f);
    private readonly Color _gizmoSlip = new Color(1f, 0.8f, 0f, 0.9f);

    private void Awake()
    {
        _rb = GetComponentInParent<Rigidbody>();
        _frameCount = 0;
        IsSettled = !useSettleFrames;
        SettleProgress = IsSettled ? 1f : 0f;

        if (_rb == null)
            Debug.LogError($"[RaycastWheel] No Rigidbody found in parent of {name}");
    }

    private void FixedUpdate()
    {
        if (UseExternalSimulation)
            return;

        Simulate(null);
    }

    public void Simulate(VehiclePhysicsCoordinator coordinator)
    {
        if (_rb == null)
            return;

        float dt = Time.fixedDeltaTime;
        _frameCount++;

        IsGrounded = TryGetGroundHit(out RaycastHit hit, out float anchorToHitDistance);

        if (!IsGrounded)
        {
            SuspensionTravel = 0f;
            NormalForce = 0f;
            LocalSlipVector = Vector2.zero;
            _prevSuspTravel = 0f;
            _hasPreviousSuspensionTravel = false;
            SettleProgress = useSettleFrames ? Mathf.Clamp01((float)_frameCount / Mathf.Max(1, settleFrames)) : 1f;
            IsSettled = SettleProgress >= 1f;
            return;
        }

        ContactPoint = hit.point;
        ContactNormal = hit.normal;
        float maxAnchorDistance = suspensionLength + wheelRadius;
        SuspensionTravel = Mathf.Clamp01((maxAnchorDistance - anchorToHitDistance) / suspensionLength);

        float staticWeightPerWheel = (_rb.mass * Mathf.Abs(Physics.gravity.y)) / 4f;
        float springDisplacement = (SuspensionTravel - restLengthRatio) * suspensionLength;
        float springForce = staticWeightPerWheel + springStiffness * springDisplacement;
        float travelMetersDelta = (SuspensionTravel - _prevSuspTravel) * suspensionLength;
        float suspensionVelocity = _hasPreviousSuspensionTravel
            ? Mathf.Clamp(travelMetersDelta / Mathf.Max(dt, 0.001f), -maxSuspensionVelocity, maxSuspensionVelocity)
            : 0f;
        float damperForce = damperStrength * suspensionVelocity;
        float maxForcePerWheel = staticWeightPerWheel * 3f;

        SettleProgress = useSettleFrames ? Mathf.Clamp01((float)_frameCount / Mathf.Max(1, settleFrames)) : 1f;
        IsSettled = SettleProgress >= 1f;

        NormalForce = Mathf.Clamp(springForce + damperForce, 0f, maxForcePerWheel);

        Vector3 suspensionForce = transform.up * NormalForce;
        if (coordinator != null)
            coordinator.QueueForceAtPosition(suspensionForce, transform.position);
        else
            _rb.AddForceAtPosition(suspensionForce, transform.position);

        _prevSuspTravel = SuspensionTravel;
        _hasPreviousSuspensionTravel = true;

        Vector3 contactVelocityWorld = _rb.GetPointVelocity(ContactPoint);
        float velForward = Vector3.Dot(contactVelocityWorld, transform.forward);
        float velRight = Vector3.Dot(contactVelocityWorld, transform.right);
        float speed = contactVelocityWorld.magnitude;

        float lateralSlipAngle = speed > 0.5f ? Mathf.Rad2Deg * Mathf.Atan2(velRight, Mathf.Abs(velForward)) : 0f;
        float longitudinalSlip = -velForward / Mathf.Max(Mathf.Abs(velForward), 1f);

        LocalSlipVector = new Vector2(lateralSlipAngle, longitudinalSlip);
    }

    public bool TrySampleGround(out RaycastHit hit, out float anchorToHitDistance)
    {
        return TryGetGroundHit(out hit, out anchorToHitDistance);
    }

    public float GetRestAnchorDistance()
    {
        return wheelRadius + suspensionLength * (1f - restLengthRatio);
    }

    private bool TryGetGroundHit(out RaycastHit bestHit, out float bestAnchorDistance)
    {
        Vector3 up = transform.up;
        float maxAnchorDistance = suspensionLength + wheelRadius;
        float probeHeight = Mathf.Max(groundProbeHeight, maxAnchorDistance + 0.5f);
        float probeLength = probeHeight + maxAnchorDistance + 0.1f;

        bestHit = default;
        bestAnchorDistance = 0f;
        float bestScore = float.MaxValue;

        Vector3 topOrigin = transform.position + up * probeHeight;
        TryCollectGroundHit(topOrigin, -up, probeLength, maxAnchorDistance, ref bestHit, ref bestAnchorDistance, ref bestScore);

        // Some imported track meshes have reversed triangle winding. The reverse probe catches those surfaces.
        Vector3 bottomOrigin = transform.position - up * (maxAnchorDistance + 0.1f);
        TryCollectGroundHit(bottomOrigin, up, probeLength, maxAnchorDistance, ref bestHit, ref bestAnchorDistance, ref bestScore);

        return bestScore < float.MaxValue;
    }

    private void TryCollectGroundHit(
        Vector3 origin,
        Vector3 direction,
        float length,
        float maxAnchorDistance,
        ref RaycastHit bestHit,
        ref float bestAnchorDistance,
        ref float bestScore)
    {
        int hitCount = Physics.RaycastNonAlloc(origin, direction, _groundHits, length, groundLayers, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < hitCount; i++)
        {
            RaycastHit candidate = _groundHits[i];
            Rigidbody hitBody = candidate.rigidbody;
            if (hitBody != null && hitBody == _rb)
                continue;

            float anchorDistance = Vector3.Dot(transform.position - candidate.point, transform.up);
            if (anchorDistance < -0.05f || anchorDistance > maxAnchorDistance + 0.05f)
                continue;

            float score = Mathf.Abs(anchorDistance - GetRestAnchorDistance());
            if (score < bestScore)
            {
                bestScore = score;
                bestHit = candidate;
                bestAnchorDistance = Mathf.Max(0f, anchorDistance);
            }
        }
    }

    private void LateUpdate()
    {
        // WheelVisual owns the actual visual mesh movement. This remains for legacy prefab compatibility.
    }

    private void OnDrawGizmos()
    {
        float rayLength = suspensionLength + wheelRadius;

        if (IsGrounded)
        {
            Gizmos.color = _gizmoSpring;
            Gizmos.DrawLine(transform.position, ContactPoint);
            Gizmos.DrawWireSphere(ContactPoint, 0.04f);

            if (LocalSlipVector.sqrMagnitude > 0.001f)
            {
                Gizmos.color = _gizmoSlip;
                Vector3 slipDir = transform.right * LocalSlipVector.x * 0.01f + transform.forward * LocalSlipVector.y;
                Gizmos.DrawLine(ContactPoint, ContactPoint + slipDir * 0.5f);
            }
        }
        else
        {
            Gizmos.color = _gizmoNoHit;
            Gizmos.DrawLine(transform.position, transform.position - transform.up * rayLength);
        }

        Gizmos.color = new Color(1f, 1f, 1f, 0.2f);
        Gizmos.DrawWireSphere(
            transform.position - transform.up * (suspensionLength * (1f - SuspensionTravel) + wheelRadius),
            wheelRadius);
    }
}
