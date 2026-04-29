using NUnit.Framework;
using UnityEngine;

public class F1FoundationPlayModeTests
{
    [Test]
    public void Downforce_IsCappedByFullDownforceSpeed()
    {
        GameObject car = new GameObject("Downforce Test Car");
        Rigidbody rb = car.AddComponent<Rigidbody>();
        DownforceSystem downforce = car.AddComponent<DownforceSystem>();
        downforce.downforceCoeff = 5f;
        downforce.fullDownforceSpeed = 70f;

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.forward * 140f;
#else
        rb.velocity = Vector3.forward * 140f;
#endif
        downforce.Simulate(null);

        Assert.AreEqual(5f * 70f * 70f, downforce.DownforceTotal, 0.01f);
        Object.DestroyImmediate(car);
    }

    [Test]
    public void DrivetrainProfile_AppliesBrakeAndMotorTuning()
    {
        GameObject car = new GameObject("Drivetrain Test Car");
        BasicMotor drivetrain = car.AddComponent<BasicMotor>();
        DrivetrainBrakeProfile profile = new DrivetrainBrakeProfile
        {
            motorForce = 120000f,
            brakeForce = 130000f,
            throttleSpoolSpeed = 3f,
            brakeSpoolSpeed = 7f,
            frontBrakeBias = 0.62f
        };

        drivetrain.ApplyProfile(profile);

        Assert.AreEqual(120000f, drivetrain.motorForce);
        Assert.AreEqual(130000f, drivetrain.brakeForce);
        Assert.AreEqual(0.62f, drivetrain.frontBrakeBias);
        Object.DestroyImmediate(car);
    }

    [Test]
    public void SteeringAssist_UsesSlipAngleInsteadOfGripCoefficient()
    {
        GameObject car = new GameObject("Steering Test Car");
        Rigidbody rb = car.AddComponent<Rigidbody>();
        TractionSystem traction = car.AddComponent<TractionSystem>();
        SteeringSystem steering = car.AddComponent<SteeringSystem>();

        RaycastWheel[] wheels = new RaycastWheel[4];
        for (int i = 0; i < wheels.Length; i++)
        {
            GameObject wheel = new GameObject($"Wheel {i}");
            wheel.transform.SetParent(car.transform);
            wheel.AddComponent<WheelVisual>();
            wheels[i] = wheel.AddComponent<RaycastWheel>();
            wheels[i].IsGrounded = true;
            wheels[i].LocalSlipVector = new Vector2(i >= 2 ? 12f : 0f, 0f);
        }

        traction.wheels = wheels;
        steering.rb = rb;
        steering.tractionSystem = traction;
        steering.wheelFL = wheels[0].transform;
        steering.wheelFR = wheels[1].transform;
        steering.oversteerAssistStrength = 0.5f;
        steering.slipThreshold = 8f;

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = car.transform.forward * 30f;
#else
        rb.velocity = car.transform.forward * 30f;
#endif
        steering.Simulate(null);

        Assert.Less(steering.LastAssistAngle, 0f);
        Object.DestroyImmediate(car);
    }

    [Test]
    public void CameraSpeedPerception_IncreasesFovWithSpeed()
    {
        GameObject cameraObject = new GameObject("Speed Camera");
        Camera camera = cameraObject.AddComponent<Camera>();
        CameraSpeedPerception perception = cameraObject.AddComponent<CameraSpeedPerception>();
        GameObject target = new GameObject("Target Car");
        Rigidbody rb = target.AddComponent<Rigidbody>();

        perception.unityCamera = camera;
        perception.targetRigidbody = rb;
        perception.useProfileSettings = false;
        perception.baseFov = 60f;
        perception.maxFov = 80f;
        perception.maxFovSpeedKmh = 200f;
        perception.fovSmoothTime = 0.01f;

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.forward * 80f;
#else
        rb.velocity = Vector3.forward * 80f;
#endif
        perception.SendMessage("LateUpdate");

        Assert.Greater(camera.fieldOfView, 60f);
        Object.DestroyImmediate(cameraObject);
        Object.DestroyImmediate(target);
    }
}
