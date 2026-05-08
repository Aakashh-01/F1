using UnityEngine;

public class F1CameraTargetRig : MonoBehaviour
{
    [Header("References")]
    public Transform car;
    public Rigidbody carRb;
    public SteeringSystem steeringSystem;

    [Header("Follow")]
    public Vector3 localOffset = new Vector3(0f, 0.65f, -0.35f);
    public float positionSharpness = 13f;
    public float rotationSharpness = 8f;
    [Range(0f, 2f)] public float highSpeedPullback = 0.7f;
    [Range(0f, 1f)] public float highSpeedLift = 0.14f;
    [Range(0f, 0.75f)] public float lateralFrameOffset = 0.28f;
    [Range(0f, 2f)] public float velocityLookAhead = 0.6f;
    [Range(1f, 25f)] public float lookAheadSharpness = 9f;

    [Header("Turn Feel")]
    [Range(0f, 12f)] public float maxYawLead = 4.4f;
    [Range(0f, 10f)] public float maxRollLean = 3.4f;
    [Range(0f, 8f)] public float maxBrakePitch = 1.8f;
    [Range(0f, 8f)] public float maxThrottlePitch = 1.1f;
    [Range(10f, 250f)] public float fullEffectSpeedKmh = 160f;
    [Range(0f, 1f)] public float steeringInfluence = 0.48f;
    [Range(0f, 1f)] public float yawRateInfluence = 0.58f;
    [Range(0f, 1f)] public float lateralVelocityInfluence = 0.22f;

    private Vector3 _lookAheadOffset;
    private Vector3 _previousLocalVelocity;
    private float _localAccelerationZ;
    private float _accelVelocity;

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
        float accelerationSignal = GetLongitudinalAccelerationSignal();

        Vector3 dynamicLocalOffset = localOffset;
        dynamicLocalOffset.z -= highSpeedPullback * speedT;
        dynamicLocalOffset.y += highSpeedLift * speedT;
        dynamicLocalOffset.x += -turnSignal * lateralFrameOffset * speedT;

        Vector3 targetLookAhead = GetVelocityLookAhead(speedT);
        float lookAheadBlend = 1f - Mathf.Exp(-Mathf.Max(0.01f, lookAheadSharpness) * Time.deltaTime);
        _lookAheadOffset = Vector3.Lerp(_lookAheadOffset, targetLookAhead, lookAheadBlend);

        Vector3 targetPosition = car.TransformPoint(dynamicLocalOffset) + _lookAheadOffset;
        Quaternion baseRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(car.forward, Vector3.up).normalized, Vector3.up);
        float pitch = accelerationSignal < 0f
            ? -accelerationSignal * maxBrakePitch
            : -accelerationSignal * maxThrottlePitch;
        Quaternion turnRotation = Quaternion.Euler(pitch, turnSignal * maxYawLead * speedT, -turnSignal * maxRollLean * speedT);
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
        _lookAheadOffset = Vector3.zero;
        _previousLocalVelocity = GetCurrentLocalVelocity();
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

    private Vector3 GetVelocityLookAhead(float speedT)
    {
        if (carRb == null)
            return Vector3.zero;

#if UNITY_6000_0_OR_NEWER
        Vector3 velocity = carRb.linearVelocity;
#else
        Vector3 velocity = carRb.velocity;
#endif
        Vector3 planarVelocity = Vector3.ProjectOnPlane(velocity, Vector3.up);
        if (planarVelocity.sqrMagnitude < 1f)
            return Vector3.zero;

        return planarVelocity.normalized * velocityLookAhead * speedT;
    }

    private float GetLongitudinalAccelerationSignal()
    {
        if (carRb == null || car == null)
            return 0f;

        Vector3 localVelocity = GetCurrentLocalVelocity();
        Vector3 localAcceleration = (localVelocity - _previousLocalVelocity) / Mathf.Max(Time.deltaTime, 0.001f);
        _previousLocalVelocity = localVelocity;

        _localAccelerationZ = Mathf.SmoothDamp(_localAccelerationZ, localAcceleration.z, ref _accelVelocity, 0.12f);
        return Mathf.Clamp(_localAccelerationZ / 35f, -1f, 1f);
    }

    private Vector3 GetCurrentLocalVelocity()
    {
        if (carRb == null || car == null)
            return Vector3.zero;

#if UNITY_6000_0_OR_NEWER
        return car.InverseTransformDirection(carRb.linearVelocity);
#else
        return car.InverseTransformDirection(carRb.velocity);
#endif
    }
}
