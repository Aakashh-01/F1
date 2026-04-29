using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DownforceSystem : MonoBehaviour
{
    [Header("Aero Coefficients")]
    [Range(0.5f, 8f)] public float downforceCoeff = 3.5f;
    [Range(0f, 1f)] public float frontBias = 0.38f;
    [Range(20f, 100f)] public float fullDownforceSpeed = 70f;

    [Header("High-Speed Stability")]
    [Range(0f, 5f)] public float lateralDragCoeff = 1.8f;
    [Range(10f, 60f)] public float lateralDragOnsetSpeed = 20f;
    [Range(1000f, 40000f)] public float lateralDragCap = 18000f;

    [Header("Force Application Points (local space)")]
    public Vector3 frontDownforcePoint = new Vector3(0f, 0.1f, 1.6f);
    public Vector3 rearDownforcePoint = new Vector3(0f, 0.3f, -1.4f);

    [HideInInspector] public float DownforceTotal;
    [HideInInspector] public float FrontDownforce;
    [HideInInspector] public float RearDownforce;
    [HideInInspector] public float SpeedMs;

    public bool UseExternalSimulation { get; set; }

    private Rigidbody _rb;
    private VehiclePhysicsCoordinator _coordinator;

    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();
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

#if UNITY_6000_0_OR_NEWER
        SpeedMs = _rb.linearVelocity.magnitude;
#else
        SpeedMs = _rb.velocity.magnitude;
#endif

        float effectiveSpeed = Mathf.Min(SpeedMs, fullDownforceSpeed);
        DownforceTotal = downforceCoeff * effectiveSpeed * effectiveSpeed;
        FrontDownforce = DownforceTotal * frontBias;
        RearDownforce = DownforceTotal * (1f - frontBias);

        AddForceAtPosition(-transform.up * FrontDownforce, transform.TransformPoint(frontDownforcePoint));
        AddForceAtPosition(-transform.up * RearDownforce, transform.TransformPoint(rearDownforcePoint));
        ApplyLateralDrag();
    }

    public void ApplyProfile(AeroProfile profile)
    {
        if (profile == null)
            return;

        downforceCoeff = profile.downforceCoeff;
        frontBias = profile.frontBias;
        fullDownforceSpeed = profile.fullDownforceSpeed;
        lateralDragCoeff = profile.lateralDragCoeff;
        lateralDragOnsetSpeed = profile.lateralDragOnsetSpeed;
        lateralDragCap = profile.lateralDragCap;
    }

    private void ApplyLateralDrag()
    {
#if UNITY_6000_0_OR_NEWER
        float lateralVelocity = Vector3.Dot(_rb.linearVelocity, transform.right);
#else
        float lateralVelocity = Vector3.Dot(_rb.velocity, transform.right);
#endif

        float ramp = Mathf.InverseLerp(lateralDragOnsetSpeed, fullDownforceSpeed, SpeedMs);
        float dragMagnitude = Mathf.Clamp(
            lateralDragCoeff * lateralVelocity * lateralVelocity * 2.5f,
            0f,
            lateralDragCap);

        Vector3 dragForce = transform.right * -Mathf.Sign(lateralVelocity) * dragMagnitude * ramp;
        AddForce(dragForce);
    }

    private void AddForce(Vector3 force)
    {
        if (_coordinator != null)
            _coordinator.QueueForce(force);
        else
            _rb.AddForce(force);
    }

    private void AddForceAtPosition(Vector3 force, Vector3 position)
    {
        if (_coordinator != null)
            _coordinator.QueueForceAtPosition(force, position);
        else
            _rb.AddForceAtPosition(force, position);
    }

    public DownforceStats GetStats()
    {
        return new DownforceStats
        {
            speedKmh = SpeedMs * 3.6f,
            totalDownforce = DownforceTotal,
            frontDownforce = FrontDownforce,
            rearDownforce = RearDownforce,
            frontBiasActual = DownforceTotal > 0f ? FrontDownforce / DownforceTotal : frontBias
        };
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = new Color(0f, 0.8f, 1f, 0.7f);
        Vector3 frontPoint = transform.TransformPoint(frontDownforcePoint);
        Gizmos.DrawSphere(frontPoint, 0.05f);
        Gizmos.DrawLine(frontPoint, frontPoint - transform.up * (FrontDownforce * 0.00005f));

        Gizmos.color = new Color(1f, 0.2f, 0.8f, 0.7f);
        Vector3 rearPoint = transform.TransformPoint(rearDownforcePoint);
        Gizmos.DrawSphere(rearPoint, 0.05f);
        Gizmos.DrawLine(rearPoint, rearPoint - transform.up * (RearDownforce * 0.00005f));
    }
}

[System.Serializable]
public struct DownforceStats
{
    public float speedKmh;
    public float totalDownforce;
    public float frontDownforce;
    public float rearDownforce;
    public float frontBiasActual;
}
