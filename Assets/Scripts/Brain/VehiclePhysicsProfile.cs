using UnityEngine;

[CreateAssetMenu(menuName = "F1/Vehicle Physics Profile", fileName = "VehiclePhysicsProfile")]
public class VehiclePhysicsProfile : ScriptableObject
{
    public SuspensionProfile suspension = new SuspensionProfile();
    public TireGripProfile tireGrip = new TireGripProfile();
    public AeroProfile aero = new AeroProfile();
    public SteeringAssistProfile steering = new SteeringAssistProfile();
    public DrivetrainBrakeProfile drivetrain = new DrivetrainBrakeProfile();
    public CameraSpeedProfile camera = new CameraSpeedProfile();
}

[System.Serializable]
public class SuspensionProfile
{
    [Range(0.05f, 0.5f)] public float suspensionLength = 0.3f;
    [Range(10000f, 120000f)] public float springStiffness = 60000f;
    [Range(1000f, 12000f)] public float damperStrength = 9000f;
    [Range(0f, 1f)] public float restLengthRatio = 0.5f;
    [Range(0.25f, 0.55f)] public float physicsWheelRadius = 0.34f;
}

[System.Serializable]
public class TireGripProfile
{
    [Range(1.0f, 2.5f)] public float lateralC = 1.5f;
    [Range(0.5f, 2.5f)] public float lateralD = 2.0f;
    [Range(4f, 20f)] public float lateralB = 10f;
    [Range(0.5f, 1.2f)] public float lateralE = 0.97f;
    [Range(1.0f, 2.5f)] public float longC = 1.65f;
    [Range(0.5f, 2.5f)] public float longD = 0.8f;
    [Range(4f, 20f)] public float longB = 11f;
    [Range(0.5f, 1.2f)] public float longE = 0.97f;
    [Range(1000f, 8000f)] public float referenceLoad = 3500f;
    [Range(0.4f, 1.0f)] public float loadSensitivity = 0.68f;
    [Range(0.5f, 1.5f)] public float lateralRadius = 1.0f;
    [Range(0.5f, 1.5f)] public float longRadius = 1.0f;
    [Range(0.6f, 1.0f)] public float lowSpeedGripMult = 0.82f;
    [Range(5f, 25f)] public float lowSpeedThreshold = 15f;
    [Range(0.8f, 1.2f)] public float midSpeedGripMult = 1.0f;
    [Range(40f, 90f)] public float highSpeedThreshold = 60f;
    [Range(0.7f, 1.0f)] public float highSpeedGripMult = 0.88f;
}

[System.Serializable]
public class AeroProfile
{
    [Range(0.5f, 8f)] public float downforceCoeff = 5f;
    [Range(0f, 1f)] public float frontBias = 0.38f;
    [Range(20f, 100f)] public float fullDownforceSpeed = 70f;
    [Range(0f, 5f)] public float lateralDragCoeff = 1.8f;
    [Range(10f, 60f)] public float lateralDragOnsetSpeed = 20f;
    [Range(1000f, 40000f)] public float lateralDragCap = 18000f;
}

[System.Serializable]
public class SteeringAssistProfile
{
    [Range(5f, 35f)] public float maxSteerAngle = 16f;
    public AnimationCurve speedSensitivityCurve = new AnimationCurve(
        new Keyframe(0f, 1f),
        new Keyframe(120f, 0.9f),
        new Keyframe(220f, 0.68f),
        new Keyframe(310f, 0.52f));
    [Range(0.01f, 0.8f)] public float lowSpeedSmoothTime = 0.05f;
    [Range(0.01f, 1.0f)] public float highSpeedSmoothTime = 0.1f;
    [Range(10f, 250f)] public float speedThresholdKmh = 50f;
    [Range(0f, 0.2f)] public float ackermannFactor = 0.15f;
    [Range(0f, 1f)] public float oversteerAssistStrength = 0.3f;
    [Range(0f, 1f)] public float understeerAssistStrength = 0.15f;
    [Range(1f, 20f)] public float slipThresholdDegrees = 8f;
}

[System.Serializable]
public class DrivetrainBrakeProfile
{
    [Range(1000f, 200000f)] public float motorForce = 92000f;
    [Range(1000f, 200000f)] public float brakeForce = 90000f;
    [Range(0.2f, 8f)] public float throttleSpoolSpeed = 1.5f;
    [Range(0.2f, 12f)] public float brakeSpoolSpeed = 6f;
    [Range(0f, 1f)] public float rearDriveBias = 1f;
    [Range(0f, 1f)] public float frontBrakeBias = 0.68f;
    [Range(0f, 1f)] public float brakeSteerStability = 0.75f;
    [Range(0f, 1f)] public float brakeSteerFrontBias = 0.72f;
    [Range(0f, 250f)] public float brakeSteerBlendSpeedKmh = 80f;
    [Range(0f, 25f)] public float reverseMaxSpeedKmh = 12f;
    public AnimationCurve motorForceBySpeed = new AnimationCurve(
        new Keyframe(0f, 1f),
        new Keyframe(180f, 0.78f),
        new Keyframe(260f, 0.46f),
        new Keyframe(310f, 0.2f),
        new Keyframe(350f, 0.08f));
    public AnimationCurve brakeForceBySpeed = AnimationCurve.Linear(0f, 0.65f, 320f, 0.86f);
}

[System.Serializable]
public class CameraSpeedProfile
{
    [Range(40f, 90f)] public float baseFov = 60f;
    [Range(50f, 110f)] public float maxFov = 72f;
    [Range(10f, 350f)] public float maxFovSpeedKmh = 260f;
    [Range(0.01f, 1f)] public float fovSmoothTime = 0.18f;
    [Range(0f, 2f)] public float shakeAmplitude = 0.15f;
    [Range(0f, 1f)] public float shakeAtSpeed = 0.35f;
    public bool enableShake;
}
