using UnityEngine;

public class F1CameraTargetRig : MonoBehaviour
{
    [Header("References")]
    public Transform car;
    public Rigidbody carRb;
    public SteeringSystem steeringSystem;

    [Header("Follow")]
    public Vector3 localOffset = new Vector3(0f, 0.65f, -0.35f);
    public float positionSharpness = 18f;
    public float rotationSharpness = 9f;

    [Header("Turn Feel")]
    [Range(0f, 12f)] public float maxYawLead = 4.5f;
    [Range(0f, 10f)] public float maxRollLean = 2.2f;
    [Range(10f, 250f)] public float fullEffectSpeedKmh = 140f;
    [Range(0f, 1f)] public float steeringInfluence = 0.45f;
    [Range(0f, 1f)] public float yawRateInfluence = 0.55f;
    [Range(0f, 1f)] public float lateralVelocityInfluence = 0.25f;

    private void Reset()
    {
        TryAutoAssign();
    }

    private void Awake()
    {
        TryAutoAssign();
        SnapToTarget();
    }

    private void LateUpdate()
    {
        if (car == null)
            return;

        if (carRb == null)
            carRb = car.GetComponent<Rigidbody>();

        float speedKmh = GetSpeedKmh();
        float speedT = Mathf.InverseLerp(20f, Mathf.Max(21f, fullEffectSpeedKmh), speedKmh);
        float turnSignal = Mathf.Clamp(GetSteeringSignal() + GetYawRateSignal() + GetLateralSignal(), -1f, 1f);

        Vector3 targetPosition = car.TransformPoint(localOffset);
        Quaternion baseRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(car.forward, Vector3.up).normalized, Vector3.up);
        Quaternion turnRotation = Quaternion.Euler(0f, turnSignal * maxYawLead * speedT, -turnSignal * maxRollLean * speedT);
        Quaternion targetRotation = baseRotation * turnRotation;

        float positionBlend = 1f - Mathf.Exp(-Mathf.Max(0.01f, positionSharpness) * Time.deltaTime);
        float rotationBlend = 1f - Mathf.Exp(-Mathf.Max(0.01f, rotationSharpness) * Time.deltaTime);
        transform.position = Vector3.Lerp(transform.position, targetPosition, positionBlend);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, rotationBlend);
    }

    public void SnapToTarget()
    {
        if (car == null)
            return;

        transform.position = car.TransformPoint(localOffset);
        transform.rotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(car.forward, Vector3.up).normalized, Vector3.up);
    }

    private void TryAutoAssign()
    {
        if (car == null)
        {
            GameObject carObject = GameObject.Find("F1_Body");
            if (carObject != null)
                car = carObject.transform;
        }

        if (car != null)
        {
            if (carRb == null)
                carRb = car.GetComponent<Rigidbody>();

            if (steeringSystem == null)
                steeringSystem = car.GetComponent<SteeringSystem>();
        }
    }

    private float GetSpeedKmh()
    {
        if (carRb == null)
            return 0f;

#if UNITY_6000_0_OR_NEWER
        return carRb.linearVelocity.magnitude * 3.6f;
#else
        return carRb.velocity.magnitude * 3.6f;
#endif
    }

    private float GetSteeringSignal()
    {
        if (steeringSystem == null || Mathf.Approximately(steeringSystem.maxSteerAngle, 0f))
            return 0f;

        return Mathf.Clamp(steeringSystem.CurrentSteerAngle / steeringSystem.maxSteerAngle, -1f, 1f) * steeringInfluence;
    }

    private float GetYawRateSignal()
    {
        if (carRb == null)
            return 0f;

        return Mathf.Clamp(carRb.angularVelocity.y / 1.5f, -1f, 1f) * yawRateInfluence;
    }

    private float GetLateralSignal()
    {
        if (carRb == null || car == null)
            return 0f;

#if UNITY_6000_0_OR_NEWER
        Vector3 velocity = carRb.linearVelocity;
#else
        Vector3 velocity = carRb.velocity;
#endif
        float lateralMs = Vector3.Dot(velocity, car.right);
        return Mathf.Clamp(lateralMs / 18f, -1f, 1f) * lateralVelocityInfluence;
    }
}
