using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class TractionSystem : MonoBehaviour
{
    [Header("Wheel References")]
    [Tooltip("Assign in order: FL, FR, RL, RR")]
    public RaycastWheel[] wheels = new RaycastWheel[4];

    [Header("Pacejka Lateral")]
    [Range(1.0f, 2.5f)] public float lateralC = 1.5f;
    [Range(0.5f, 2.5f)] public float lateralD = 1.6f;
    [Range(4f, 20f)] public float lateralB = 10f;
    [Range(0.5f, 1.2f)] public float lateralE = 0.97f;

    [Header("Pacejka Longitudinal")]
    [Range(1.0f, 2.5f)] public float longC = 1.65f;
    [Range(0.5f, 2.5f)] public float longD = 1.8f;
    [Range(4f, 20f)] public float longB = 11f;
    [Range(0.5f, 1.2f)] public float longE = 0.97f;

    [Header("Load Sensitivity")]
    [Range(1000f, 8000f)] public float referenceLoad = 3500f;
    [Range(0.4f, 1.0f)] public float loadSensitivity = 0.68f;

    [Header("Friction Ellipse")]
    [Range(0.5f, 1.5f)] public float lateralRadius = 1.0f;
    [Range(0.5f, 1.5f)] public float longRadius = 1.0f;

    [Header("Speed Band Grip Multipliers")]
    [Range(0.6f, 1.0f)] public float lowSpeedGripMult = 0.82f;
    [Range(5f, 25f)] public float lowSpeedThreshold = 15f;
    [Range(0.8f, 1.2f)] public float midSpeedGripMult = 1.0f;
    [Range(40f, 90f)] public float highSpeedThreshold = 60f;
    [Range(0.7f, 1.0f)] public float highSpeedGripMult = 0.88f;

    [HideInInspector] public Vector3[] WheelGripForces = new Vector3[4];
    [HideInInspector] public float[] GripUtilisation = new float[4];
    [HideInInspector] public float[] LateralGripCoeff = new float[4];
    [HideInInspector] public float[] LongGripCoeff = new float[4];

    public bool UseExternalSimulation { get; set; }

    private Rigidbody _rb;
    private VehiclePhysicsCoordinator _coordinator;
    private float _speedMs;

    private const int FL = 0, FR = 1, RL = 2, RR = 3;

    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();

        if (wheels.Length != 4)
            Debug.LogError("[TractionSystem] Exactly 4 wheels must be assigned (FL, FR, RL, RR).");

        ResetAllWheelOutputs();
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
        _speedMs = _rb.linearVelocity.magnitude;
#else
        _speedMs = _rb.velocity.magnitude;
#endif

        for (int i = 0; i < wheels.Length; i++)
        {
            RaycastWheel wheel = wheels[i];
            if (wheel == null || !wheel.IsGrounded)
            {
                ResetWheelOutput(i);
                continue;
            }

            ProcessWheel(i, wheel);
        }
    }

    public void ApplyProfile(TireGripProfile profile)
    {
        if (profile == null)
            return;

        lateralC = profile.lateralC;
        lateralD = profile.lateralD;
        lateralB = profile.lateralB;
        lateralE = profile.lateralE;
        longC = profile.longC;
        longD = profile.longD;
        longB = profile.longB;
        longE = profile.longE;
        referenceLoad = profile.referenceLoad;
        loadSensitivity = profile.loadSensitivity;
        lateralRadius = profile.lateralRadius;
        longRadius = profile.longRadius;
        lowSpeedGripMult = profile.lowSpeedGripMult;
        lowSpeedThreshold = profile.lowSpeedThreshold;
        midSpeedGripMult = profile.midSpeedGripMult;
        highSpeedThreshold = profile.highSpeedThreshold;
        highSpeedGripMult = profile.highSpeedGripMult;
    }

    private void ProcessWheel(int index, RaycastWheel wheel)
    {
        float normalForce = wheel.NormalForce;
        if (normalForce <= 0f)
        {
            ResetWheelOutput(index);
            return;
        }

        if (_speedMs < 1.0f)
        {
            ResetWheelOutput(index);
            Vector3 contactVelocity = _rb.GetPointVelocity(wheel.transform.position);
            Vector3 dampingForce = -contactVelocity * (normalForce * 0.3f);
            dampingForce.y = 0f;
            AddForceAtPosition(dampingForce, wheel.transform.position);
            return;
        }

        float loadRatio = normalForce / referenceLoad;
        float gripScale = Mathf.Pow(loadRatio, loadSensitivity);
        float speedMult = GetSpeedBandMultiplier(_speedMs);
        float effectiveLateralD = lateralD * gripScale * speedMult;
        float effectiveLongD = longD * gripScale * speedMult;

        float lateralCoeff = Pacejka(wheel.LocalSlipVector.x, lateralB, lateralC, effectiveLateralD, lateralE);
        float longCoeff = Pacejka(wheel.LocalSlipVector.y, longB, longC, effectiveLongD, longE);

        LateralGripCoeff[index] = lateralCoeff;
        LongGripCoeff[index] = longCoeff;

        float maxLateral = effectiveLateralD * normalForce * lateralRadius;
        float maxLong = effectiveLongD * normalForce * longRadius;
        float rawLateral = lateralCoeff * normalForce;
        float rawLong = longCoeff * normalForce;

        float utilisation = 0f;
        if (maxLateral > 0f && maxLong > 0f)
        {
            float normLat = rawLateral / maxLateral;
            float normLong = rawLong / maxLong;
            utilisation = Mathf.Sqrt(normLat * normLat + normLong * normLong);
        }

        GripUtilisation[index] = utilisation;

        float ellipseScale = utilisation > 1f ? 1f / utilisation : 1f;
        Vector3 lateralForce = -wheel.transform.right * rawLateral * ellipseScale;
        Vector3 longForce = wheel.transform.forward * rawLong * ellipseScale;
        Vector3 totalForce = lateralForce + longForce;

        WheelGripForces[index] = totalForce;
        AddForceAtPosition(totalForce, wheel.transform.position);
    }

    private float Pacejka(float x, float b, float c, float d, float e)
    {
        float bx = b * x;
        float inner = bx - e * (bx - Mathf.Atan(bx));
        return d * Mathf.Sin(c * Mathf.Atan(inner));
    }

    private float GetSpeedBandMultiplier(float speed)
    {
        if (speed < lowSpeedThreshold)
            return Mathf.Lerp(lowSpeedGripMult, midSpeedGripMult, speed / lowSpeedThreshold);

        if (speed > highSpeedThreshold)
            return Mathf.Lerp(midSpeedGripMult, highSpeedGripMult, Mathf.InverseLerp(highSpeedThreshold, highSpeedThreshold * 1.5f, speed));

        return midSpeedGripMult;
    }

    private void AddForceAtPosition(Vector3 force, Vector3 position)
    {
        if (force.sqrMagnitude <= 0.000001f)
            return;

        if (_coordinator != null)
            _coordinator.QueueForceAtPosition(force, position);
        else
            _rb.AddForceAtPosition(force, position);
    }

    private void ResetAllWheelOutputs()
    {
        for (int i = 0; i < 4; i++)
            ResetWheelOutput(i);
    }

    private void ResetWheelOutput(int index)
    {
        WheelGripForces[index] = Vector3.zero;
        GripUtilisation[index] = 0f;
        LateralGripCoeff[index] = 0f;
        LongGripCoeff[index] = 0f;
    }

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

    private void OnDrawGizmos()
    {
        for (int i = 0; i < wheels.Length; i++)
        {
            if (wheels[i] == null || !wheels[i].IsGrounded) continue;

            float util = GripUtilisation[i];
            Gizmos.color = Color.Lerp(Color.green, Color.red, util);

            Vector3 from = wheels[i].ContactPoint;
            Vector3 to = from + Vector3.up * (util * 0.6f);
            Gizmos.DrawLine(from, to);
            Gizmos.DrawWireSphere(to, 0.04f);

            if (WheelGripForces[i].sqrMagnitude > 1f)
            {
                Gizmos.color = new Color(1f, 0.9f, 0.2f, 0.6f);
                Gizmos.DrawLine(from, from + WheelGripForces[i].normalized * 0.4f);
            }
        }
    }
}

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
