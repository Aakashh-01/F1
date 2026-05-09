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
    public void DrivetrainBrake_YawedCarBrakesAlongForwardAxisOnly()
    {
        GameObject car = new GameObject("Yawed Brake Test Car");
        BasicMotor drivetrain = car.AddComponent<BasicMotor>();
        var method = typeof(DrivetrainBrakeSystem).GetMethod(
            "CalculateBrakeForceVector",
            System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);

        Vector3 planarVelocity = (car.transform.forward + car.transform.right).normalized * 80f;
        Vector3 brakeForce = (Vector3)method.Invoke(drivetrain, new object[] { planarVelocity, 10000f });

        Assert.Less(Vector3.Dot(brakeForce, car.transform.forward), -9999f);
        Assert.AreEqual(0f, Vector3.Dot(brakeForce, car.transform.right), 0.001f);
        Object.DestroyImmediate(car);
    }

    [Test]
    public void DrivetrainBrake_UsesReverseOnlyAtLowForwardSpeed()
    {
        GameObject car = new GameObject("Reverse Selection Test Car");
        BasicMotor drivetrain = car.AddComponent<BasicMotor>();
        var method = typeof(DrivetrainBrakeSystem).GetMethod(
            "ShouldUseReverse",
            System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);

        bool atRestUsesReverse = (bool)method.Invoke(drivetrain, new object[] { 1f, 0f, 0f });
        bool forwardSpeedUsesBrake = (bool)method.Invoke(drivetrain, new object[] { 1f, 0f, 40f });

        Assert.IsTrue(atRestUsesReverse);
        Assert.IsFalse(forwardSpeedUsesBrake);
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
    public void AdvancedSteeringAssist_RearSlipProducesCountersteerAssist()
    {
        GameObject car = CreateAdvancedAssistTestCar(0f, 16f, out VehiclePhysicsCoordinator coordinator, out AdvancedSteeringAssist assist);
        RunAdvancedAssistFrames(coordinator, assist, 24);

        Assert.AreEqual(TractionLossState.RearOversteer, assist.CurrentTractionLossState);
        Assert.Less(assist.RawAssistAngle, 0f);
        Assert.Less(assist.SmoothedAssistAngle, 0f);
        Object.DestroyImmediate(car);
    }

    [Test]
    public void AdvancedSteeringAssist_UndersteerReducesSteeringDemand()
    {
        GameObject car = CreateAdvancedAssistTestCar(15f, 0f, out VehiclePhysicsCoordinator coordinator, out AdvancedSteeringAssist assist);
        MobileTouchControls.SetSteering(1f);
        coordinator.SendMessage("Update");
        RunAdvancedAssistFrames(coordinator, assist, 24);

        Assert.AreEqual(TractionLossState.FrontUndersteer, assist.CurrentTractionLossState);
        Assert.Less(assist.RawAssistAngle, 0f);
        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(car);
    }

    [Test]
    public void AdvancedSteeringAssist_AssistLevelsIncreaseCorrectionButStayCapped()
    {
        GameObject car = CreateAdvancedAssistTestCar(0f, 18f, out VehiclePhysicsCoordinator coordinator, out AdvancedSteeringAssist assist);

        assist.assistLevel = SteeringAssistLevel.Low;
        RunAdvancedAssistFrames(coordinator, assist, 1);
        float low = Mathf.Abs(assist.RawAssistAngle);

        assist.assistLevel = SteeringAssistLevel.Medium;
        RunAdvancedAssistFrames(coordinator, assist, 1);
        float medium = Mathf.Abs(assist.RawAssistAngle);

        assist.assistLevel = SteeringAssistLevel.High;
        RunAdvancedAssistFrames(coordinator, assist, 1);
        float high = Mathf.Abs(assist.RawAssistAngle);

        Assert.Greater(medium, low);
        Assert.Greater(high, medium);
        Assert.LessOrEqual(high, assist.maxAssistAngle * 1.35f + 0.001f);
        Object.DestroyImmediate(car);
    }

    [Test]
    public void AdvancedSteeringAssist_SkilledCountersteerSmoothlyReducesAssist()
    {
        GameObject neutralCar = CreateAdvancedAssistTestCar(0f, 16f, out VehiclePhysicsCoordinator neutralCoordinator, out AdvancedSteeringAssist neutralAssist);
        RunAdvancedAssistFrames(neutralCoordinator, neutralAssist, 40);
        float neutralMagnitude = Mathf.Abs(neutralAssist.SmoothedAssistAngle);

        GameObject countersteerCar = CreateAdvancedAssistTestCar(0f, 16f, out VehiclePhysicsCoordinator countersteerCoordinator, out AdvancedSteeringAssist countersteerAssist);
        MobileTouchControls.SetSteering(-1f);
        countersteerCoordinator.SendMessage("Update");
        RunAdvancedAssistFrames(countersteerCoordinator, countersteerAssist, 40);
        float countersteerMagnitude = Mathf.Abs(countersteerAssist.SmoothedAssistAngle);

        Assert.Greater(neutralMagnitude, 0.1f);
        Assert.Less(countersteerMagnitude, neutralMagnitude);
        Assert.Greater(countersteerAssist.PlayerOverrideFactor, 0f);

        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(neutralCar);
        Object.DestroyImmediate(countersteerCar);
    }

    [Test]
    public void AdvancedSteeringAssist_MobileTapDoesNotSnapAssistOff()
    {
        GameObject car = CreateAdvancedAssistTestCar(0f, 16f, out VehiclePhysicsCoordinator coordinator, out AdvancedSteeringAssist assist);
        RunAdvancedAssistFrames(coordinator, assist, 20);
        float beforeTap = Mathf.Abs(assist.SmoothedAssistAngle);

        MobileTouchControls.SetSteering(-1f);
        coordinator.SendMessage("Update");
        RunAdvancedAssistFrames(coordinator, assist, 1);
        float afterSingleTapFrame = Mathf.Abs(assist.SmoothedAssistAngle);

        Assert.Greater(beforeTap, 0.1f);
        Assert.Greater(afterSingleTapFrame, beforeTap * 0.65f);
        Assert.Less(assist.PlayerOverrideFactor, 0.25f);

        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(car);
    }

    [Test]
    public void AdvancedSteeringAssist_MobileReleaseSmoothsOverrideBackToZero()
    {
        GameObject car = CreateAdvancedAssistTestCar(0f, 16f, out VehiclePhysicsCoordinator coordinator, out AdvancedSteeringAssist assist);
        MobileTouchControls.SetSteering(-1f);
        coordinator.SendMessage("Update");
        RunAdvancedAssistFrames(coordinator, assist, 30);
        float heldOverride = assist.PlayerOverrideFactor;

        MobileTouchControls.SetSteering(0f);
        coordinator.SendMessage("Update");
        RunAdvancedAssistFrames(coordinator, assist, 1);
        float immediateReleaseOverride = assist.PlayerOverrideFactor;
        RunAdvancedAssistFrames(coordinator, assist, 35);

        Assert.Greater(heldOverride, 0.1f);
        Assert.Greater(immediateReleaseOverride, 0f);
        Assert.Less(assist.PlayerOverrideFactor, heldOverride);

        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(car);
    }

    [Test]
    public void AdvancedBraking_MobileBrakeRampsVirtualPedal()
    {
        GameObject car = CreateAdvancedBrakeTestCar(out VehiclePhysicsCoordinator coordinator, out AdvancedBrakingSystem braking, out _);
        braking.brakePressRate = 4f;
        MobileTouchControls.SetBrake(1f);
        coordinator.SendMessage("Update");

        RunAdvancedBrakeFrames(coordinator, braking, 1);
        float firstFramePressure = braking.VirtualBrakePressure;
        RunAdvancedBrakeFrames(coordinator, braking, 20);

        Assert.Greater(firstFramePressure, 0f);
        Assert.Less(firstFramePressure, 1f);
        Assert.Greater(braking.VirtualBrakePressure, firstFramePressure);
        Assert.LessOrEqual(braking.VirtualBrakePressure, 1f);

        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(car);
    }

    [Test]
    public void AdvancedBraking_BrakePressureCurveShapesEffectivePressure()
    {
        GameObject linearCar = CreateAdvancedBrakeTestCar(out VehiclePhysicsCoordinator linearCoordinator, out AdvancedBrakingSystem linearBraking, out _);
        ConfigureNoLockup(linearBraking);
        linearBraking.brakePressureCurve = AnimationCurve.Linear(0f, 0f, 1f, 1f);
        linearBraking.maxLateBrakeMultiplier = 1f;

        GameObject softCar = CreateAdvancedBrakeTestCar(out VehiclePhysicsCoordinator softCoordinator, out AdvancedBrakingSystem softBraking, out _);
        ConfigureNoLockup(softBraking);
        softBraking.brakePressureCurve = AnimationCurve.Linear(0f, 0f, 1f, 0.5f);
        softBraking.maxLateBrakeMultiplier = 1f;

        MobileTouchControls.SetBrake(1f);
        linearCoordinator.SendMessage("Update");
        softCoordinator.SendMessage("Update");
        RunAdvancedBrakeFrames(linearCoordinator, linearBraking, 2);
        RunAdvancedBrakeFrames(softCoordinator, softBraking, 2);

        Assert.Greater(linearBraking.EffectiveBrakePressure, softBraking.EffectiveBrakePressure);

        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(linearCar);
        Object.DestroyImmediate(softCar);
    }

    [Test]
    public void AdvancedBraking_LateBrakeCurveIncreasesHighSpeedAuthority()
    {
        GameObject slowCar = CreateAdvancedBrakeTestCar(out VehiclePhysicsCoordinator slowCoordinator, out AdvancedBrakingSystem slowBraking, out Rigidbody slowRb);
        GameObject fastCar = CreateAdvancedBrakeTestCar(out VehiclePhysicsCoordinator fastCoordinator, out AdvancedBrakingSystem fastBraking, out Rigidbody fastRb);
        ConfigureNoLockup(slowBraking);
        ConfigureNoLockup(fastBraking);
        slowBraking.brakePressureCurve = AnimationCurve.Linear(0f, 0f, 1f, 1f);
        fastBraking.brakePressureCurve = AnimationCurve.Linear(0f, 0f, 1f, 1f);
        slowBraking.lateBrakeMultiplierBySpeed = new AnimationCurve(new Keyframe(0f, 0.8f), new Keyframe(320f, 1.2f));
        fastBraking.lateBrakeMultiplierBySpeed = slowBraking.lateBrakeMultiplierBySpeed;

#if UNITY_6000_0_OR_NEWER
        slowRb.linearVelocity = Vector3.forward * 20f;
        fastRb.linearVelocity = Vector3.forward * 90f;
#else
        slowRb.velocity = Vector3.forward * 20f;
        fastRb.velocity = Vector3.forward * 90f;
#endif
        MobileTouchControls.SetBrake(1f);
        slowCoordinator.SendMessage("Update");
        fastCoordinator.SendMessage("Update");
        RunAdvancedBrakeFrames(slowCoordinator, slowBraking, 2);
        RunAdvancedBrakeFrames(fastCoordinator, fastBraking, 2);

        Assert.Greater(fastBraking.LateBrakeMultiplier, slowBraking.LateBrakeMultiplier);
        Assert.Greater(fastBraking.EffectiveBrakePressure, slowBraking.EffectiveBrakePressure);

        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(slowCar);
        Object.DestroyImmediate(fastCar);
    }

    [Test]
    public void AdvancedBraking_LockupThresholdsTriggerTelemetry()
    {
        GameObject car = CreateAdvancedBrakeTestCar(out VehiclePhysicsCoordinator coordinator, out AdvancedBrakingSystem braking, out _);
        braking.brakePressRate = 100f;
        braking.brakePressureCurve = AnimationCurve.Linear(0f, 0f, 1f, 1f);
        braking.frontLockupThreshold = 0.2f;
        braking.rearLockupThreshold = 0.2f;
        braking.lockupBlendRange = 0.2f;

        MobileTouchControls.SetBrake(1f);
        coordinator.SendMessage("Update");
        RunAdvancedBrakeFrames(coordinator, braking, 2);

        Assert.Greater(braking.FrontLockupAmount, 0f);
        Assert.Greater(braking.RearLockupAmount, 0f);

        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(car);
    }

    [Test]
    public void AdvancedBraking_LockupReducesBrakeEfficiency()
    {
        GameObject cleanCar = CreateAdvancedBrakeTestCar(out VehiclePhysicsCoordinator cleanCoordinator, out AdvancedBrakingSystem cleanBraking, out _);
        GameObject lockupCar = CreateAdvancedBrakeTestCar(out VehiclePhysicsCoordinator lockupCoordinator, out AdvancedBrakingSystem lockupBraking, out _);
        ConfigureNoLockup(cleanBraking);
        lockupBraking.brakePressRate = 100f;
        lockupBraking.brakePressureCurve = AnimationCurve.Linear(0f, 0f, 1f, 1f);
        lockupBraking.frontLockupThreshold = 0.2f;
        lockupBraking.rearLockupThreshold = 0.2f;
        lockupBraking.maxLockupEfficiencyLoss = 0.5f;

        MobileTouchControls.SetBrake(1f);
        cleanCoordinator.SendMessage("Update");
        lockupCoordinator.SendMessage("Update");
        RunAdvancedBrakeFrames(cleanCoordinator, cleanBraking, 2);
        RunAdvancedBrakeFrames(lockupCoordinator, lockupBraking, 2);

        Assert.Less(lockupBraking.BrakeEfficiency, cleanBraking.BrakeEfficiency);
        Assert.Less(lockupBraking.EffectiveBrakePressure, cleanBraking.EffectiveBrakePressure);

        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(cleanCar);
        Object.DestroyImmediate(lockupCar);
    }

    [Test]
    public void AdvancedBraking_TrailBrakingRequiresBrakeAndSteeringOverlap()
    {
        GameObject car = CreateAdvancedBrakeTestCar(out VehiclePhysicsCoordinator coordinator, out AdvancedBrakingSystem braking, out _);
        ConfigureNoLockup(braking);
        braking.brakePressRate = 100f;
        braking.brakePressureCurve = AnimationCurve.Linear(0f, 0f, 1f, 1f);
        braking.trailBrakeSupportStrength = 1f;
        braking.trailBrakeFrontBiasShift = 0.1f;

        MobileTouchControls.SetBrake(1f);
        MobileTouchControls.SetSteering(0f);
        coordinator.SendMessage("Update");
        RunAdvancedBrakeFrames(coordinator, braking, 2);
        float noSteerBlend = braking.TrailBrakeBlend;
        float baseBias = braking.DynamicFrontBrakeBias;

        MobileTouchControls.SetSteering(1f);
        coordinator.SendMessage("Update");
        RunAdvancedBrakeFrames(coordinator, braking, 2);

        Assert.AreEqual(0f, noSteerBlend, 0.001f);
        Assert.Greater(braking.TrailBrakeBlend, 0f);
        Assert.Greater(braking.DynamicFrontBrakeBias, baseBias);

        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(car);
    }

    [Test]
    public void AdvancedBraking_RearInstabilityIsCapped()
    {
        GameObject car = CreateAdvancedBrakeTestCar(out VehiclePhysicsCoordinator coordinator, out AdvancedBrakingSystem braking, out _);
        braking.brakePressRate = 100f;
        braking.brakePressureCurve = AnimationCurve.Linear(0f, 0f, 1f, 1f);
        braking.rearLockupThreshold = 0.1f;
        braking.rearInstabilityStrength = 1f;
        braking.maxRearInstabilityYawTorque = 1200f;

        MobileTouchControls.SetBrake(1f);
        MobileTouchControls.SetSteering(1f);
        coordinator.SendMessage("Update");
        RunAdvancedBrakeFrames(coordinator, braking, 2);

        Assert.Greater(braking.RearInstabilityAmount, 0f);
        Assert.LessOrEqual(Mathf.Abs(braking.RearInstabilityYawTorque), 1200.01f);

        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(car);
    }

    [Test]
    public void VehiclePhysicsCoordinator_ExternalInputOverridesMobileInput()
    {
        MobileTouchControls.ResetInputs();
        MobileTouchControls.SetSteering(-1f);
        MobileTouchControls.SetThrottle(1f);
        MobileTouchControls.SetBrake(1f);

        GameObject car = new GameObject("External Input Test Car");
        car.AddComponent<Rigidbody>();
        VehiclePhysicsCoordinator coordinator = car.AddComponent<VehiclePhysicsCoordinator>();
        coordinator.applyProfileOnAwake = false;
        coordinator.UseExternalInput = true;
        coordinator.SetExternalInput(0.35f, 0.6f, 0.2f);
        coordinator.SendMessage("Update");

        Assert.AreEqual(0.35f, coordinator.SteeringInput, 0.001f);
        Assert.AreEqual(0.6f, coordinator.ThrottleInput, 0.001f);
        Assert.AreEqual(0.2f, coordinator.BrakeInput, 0.001f);

        MobileTouchControls.ResetInputs();
        Object.DestroyImmediate(car);
    }

    [Test]
    public void AIDriver_SteersTowardLookaheadWaypoint()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out _, out Rigidbody rb);
        SetLineWaypoint(driver.racingLine, 1, new Vector3(12f, 0f, 28f), 120f);
#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.zero;
#else
        rb.velocity = Vector3.zero;
#endif
        coordinator.RefreshState();
        driver.Simulate();

        Assert.Greater(coordinator.SteeringInput, 0.05f);
        Assert.Greater(driver.LastSteeringInput, 0.05f);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_BrakesWhenSpeedExceedsTarget()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out _, out Rigidbody rb);
        SetAllLineSpeeds(driver.racingLine, 45f);
#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.forward * 35f;
#else
        rb.velocity = Vector3.forward * 35f;
#endif
        coordinator.RefreshState();
        driver.Simulate();

        Assert.Greater(coordinator.BrakeInput, 0.1f);
        Assert.Less(coordinator.ThrottleInput, 0.1f);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_CornerCurvatureLowersTargetSpeed()
    {
        GameObject straightRig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator straightCoordinator, out AIDriverController straightDriver, out _, out _);
        GameObject cornerRig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator cornerCoordinator, out AIDriverController cornerDriver, out _, out _);
        SetLineWaypoint(cornerDriver.racingLine, 2, new Vector3(22f, 0f, 28f), 160f);
        SetLineWaypoint(cornerDriver.racingLine, 3, new Vector3(44f, 0f, 28f), 160f);

        straightCoordinator.RefreshState();
        cornerCoordinator.RefreshState();
        straightDriver.Simulate();
        cornerDriver.Simulate();

        Assert.Greater(cornerDriver.LastCornerCurvature, straightDriver.LastCornerCurvature);
        Assert.Less(cornerDriver.LastTargetSpeedKmh, straightDriver.LastTargetSpeedKmh);
        Object.DestroyImmediate(straightRig);
        Object.DestroyImmediate(cornerRig);
    }

    [Test]
    public void AIDifficultyPresets_ChangePaceAndRisk()
    {
        AIDifficultyProfile easy = AIDifficultyProfile.CreateRuntimeProfile(AIDifficultyPreset.Easy);
        AIDifficultyProfile hard = AIDifficultyProfile.CreateRuntimeProfile(AIDifficultyPreset.Hard);

        Assert.Greater(hard.speedMultiplier, easy.speedMultiplier);
        Assert.Greater(hard.overtakeWillingness, easy.overtakeWillingness);
        Assert.Less(hard.brakingMargin, easy.brakingMargin);

        Object.DestroyImmediate(easy);
        Object.DestroyImmediate(hard);
    }

    [Test]
    public void AIPerception_UsesCachedDataBetweenStaggeredUpdates()
    {
        GameObject rig = CreateAIDriverTestRig(out _, out _, out AIPerceptionSensor sensor, out _);
        sensor.fixedFrameStride = 3;
        sensor.minimumUpdateInterval = 10f;
        GameObject obstacle = CreateObstacle("Cached Front Obstacle", new Vector3(0f, 0.8f, 8f));

        Physics.SyncTransforms();
        Assert.IsTrue(sensor.Tick(true));
        Assert.IsTrue(sensor.FrontBlocked);
        int firstSerial = sensor.SensorUpdateSerial;

        obstacle.transform.position = new Vector3(0f, 0.8f, 80f);
        Physics.SyncTransforms();
        Assert.IsFalse(sensor.Tick(false));
        Assert.AreEqual(firstSerial, sensor.SensorUpdateSerial);
        Assert.IsTrue(sensor.FrontBlocked);

        Assert.IsTrue(sensor.Tick(true));
        Assert.IsFalse(sensor.FrontBlocked);

        Object.DestroyImmediate(obstacle);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_ForwardObstacleIncreasesBraking()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out Rigidbody rb);
        GameObject obstacle = CreateObstacle("Forward AI Obstacle", new Vector3(0f, 0.8f, 4f));
#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.forward * 18f;
#else
        rb.velocity = Vector3.forward * 18f;
#endif
        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();

        Assert.Greater(coordinator.BrakeInput, 0.1f);
        Assert.Less(coordinator.ThrottleInput, 0.5f);

        Object.DestroyImmediate(obstacle);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_SideObstaclePreventsPreferredOvertakeLane()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        GameObject front = CreateObstacle("Front AI Obstacle", new Vector3(0f, 0.8f, 8f));
        GameObject right = CreateObstacle("Right AI Obstacle", new Vector3(4f, 0.8f, 0f));

        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.preferRightOvertake = true;
        driver.Simulate();

        Assert.IsTrue(sensor.FrontBlocked);
        Assert.IsTrue(sensor.RightBlocked);
        Assert.LessOrEqual(driver.DesiredLaneOffset, 0f);

        Object.DestroyImmediate(front);
        Object.DestroyImmediate(right);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_ChoosesOvertakeOffsetWhenBlockedAndSideClear()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        GameObject front = CreateObstacle("Clear Side Front Obstacle", new Vector3(0f, 0.8f, 8f));

        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.preferRightOvertake = true;
        driver.Simulate();

        Assert.IsTrue(sensor.FrontBlocked);
        Assert.IsFalse(sensor.RightBlocked);
        Assert.Greater(driver.DesiredLaneOffset, 0f);

        Object.DestroyImmediate(front);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_DoesNotRelockToFarWaypointAcrossTrack()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out _, out _);
        AIRacingLine line = driver.racingLine;
        SetLineWaypoint(line, 0, new Vector3(0f, 0f, 0f), 160f);
        SetLineWaypoint(line, 1, new Vector3(0f, 0f, 100f), 160f);
        SetLineWaypoint(line, 2, new Vector3(100f, 0f, 100f), 160f);
        SetLineWaypoint(line, 3, new Vector3(4f, 0f, 42f), 160f);
        driver.transform.position = new Vector3(3f, 0f, 42f);
        driver.CurrentWaypointIndex = 0;
        driver.forwardProgressSearchSteps = 1;
        typeof(AIDriverController)
            .GetField("_hasProgressIndex", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic)
            .SetValue(driver, true);

        coordinator.RefreshState();
        driver.Simulate();

        Assert.AreNotEqual(3, driver.CurrentWaypointIndex);
        Assert.LessOrEqual(driver.CurrentWaypointIndex, 2);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void RaycastWheel_ForwardVelocityDoesNotCreateLongitudinalSlip()
    {
        GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
        ground.name = "Longitudinal Slip Test Ground";
        ground.transform.position = Vector3.zero;
        ground.transform.localScale = new Vector3(4f, 0.02f, 4f);

        GameObject car = new GameObject("Longitudinal Slip Test Car");
        Rigidbody rb = car.AddComponent<Rigidbody>();

        GameObject wheelObject = new GameObject("FL");
        wheelObject.transform.SetParent(car.transform);
        wheelObject.transform.localPosition = new Vector3(0f, 0.49f, 0f);
        wheelObject.AddComponent<WheelVisual>();
        RaycastWheel wheel = wheelObject.AddComponent<RaycastWheel>();
        wheel.suspensionLength = 0.3f;
        wheel.wheelRadius = 0.34f;
        wheel.restLengthRatio = 0.5f;
        wheel.useSettleFrames = false;

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = car.transform.forward * 10f;
#else
        rb.velocity = car.transform.forward * 10f;
#endif
        Physics.SyncTransforms();
        wheel.Simulate(null);

        Assert.IsTrue(wheel.IsGrounded);
        Assert.AreEqual(0f, wheel.LocalSlipVector.y, 0.0001f);

        Object.DestroyImmediate(car);
        Object.DestroyImmediate(ground);
    }

    [Test]
    public void RaycastWheel_RecoversGroundContactWhenOverCompressed()
    {
        GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
        ground.name = "Overcompressed Suspension Test Ground";
        ground.transform.position = Vector3.zero;
        ground.transform.localScale = new Vector3(4f, 0.02f, 4f);

        GameObject car = new GameObject("Overcompressed Suspension Test Car");
        car.AddComponent<Rigidbody>();

        GameObject wheelObject = new GameObject("FL");
        wheelObject.transform.SetParent(car.transform);
        wheelObject.transform.localPosition = new Vector3(0f, -0.08f, 0f);
        wheelObject.AddComponent<WheelVisual>();
        RaycastWheel wheel = wheelObject.AddComponent<RaycastWheel>();
        wheel.suspensionLength = 0.3f;
        wheel.wheelRadius = 0.34f;
        wheel.restLengthRatio = 0.5f;
        wheel.useSettleFrames = false;

        Physics.SyncTransforms();
        wheel.Simulate(null);

        Assert.IsTrue(wheel.IsGrounded);
        Assert.AreEqual(1f, wheel.SuspensionTravel, 0.0001f);
        Assert.Greater(wheel.NormalForce, 0f);

        Object.DestroyImmediate(car);
        Object.DestroyImmediate(ground);
    }

    [Test]
    public void RaycastWheel_AeroLoadRaisesNormalForceClamp()
    {
        GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
        ground.name = "Aero Loaded Suspension Test Ground";
        ground.transform.position = Vector3.zero;
        ground.transform.localScale = new Vector3(4f, 0.02f, 4f);

        GameObject car = new GameObject("Aero Loaded Suspension Test Car");
        Rigidbody rb = car.AddComponent<Rigidbody>();
        DownforceSystem downforce = car.AddComponent<DownforceSystem>();
        downforce.RearDownforce = 12000f;

        GameObject wheelObject = new GameObject("RL");
        wheelObject.transform.SetParent(car.transform);
        wheelObject.transform.localPosition = new Vector3(0f, 0f, -1f);
        wheelObject.AddComponent<WheelVisual>();
        RaycastWheel wheel = wheelObject.AddComponent<RaycastWheel>();
        wheel.suspensionLength = 0.3f;
        wheel.wheelRadius = 0.34f;
        wheel.restLengthRatio = 0.5f;
        wheel.springStiffness = 60000f;
        wheel.useSettleFrames = false;

        Physics.SyncTransforms();
        wheel.Simulate(null);

        float oldClamp = rb.mass * Mathf.Abs(Physics.gravity.y) / 4f * 3f;
        Assert.IsTrue(wheel.IsGrounded);
        Assert.Greater(wheel.NormalForce, oldClamp);

        Object.DestroyImmediate(car);
        Object.DestroyImmediate(ground);
    }

    [Test]
    public void WheelVisual_AppliesSteeringYawToSteerableWheel()
    {
        GameObject car = new GameObject("Wheel Visual Steering Test Car");
        Rigidbody rb = car.AddComponent<Rigidbody>();
        SteeringSystem steering = car.AddComponent<SteeringSystem>();
        steering.CurrentSteerAngle = 12f;

        GameObject wheelObject = new GameObject("FL");
        wheelObject.transform.SetParent(car.transform);
        WheelVisual visual = wheelObject.AddComponent<WheelVisual>();
        RaycastWheel physicsWheel = wheelObject.AddComponent<RaycastWheel>();

        GameObject meshObject = new GameObject("Wheel Mesh");
        meshObject.transform.SetParent(wheelObject.transform);
        visual.wheelMesh = meshObject.transform;
        visual.carRb = rb;
        visual.physicsWheel = physicsWheel;
        visual.steeringSystem = steering;
        visual.steerable = true;
        visual.steeringYawScale = 1f;

        visual.SendMessage("Start");
        visual.SendMessage("LateUpdate");

        Assert.AreEqual(12f, meshObject.transform.localEulerAngles.y, 0.01f);
        Object.DestroyImmediate(car);
    }

    [Test]
    public void VehiclePhysicsCoordinator_UsesMobileInputWhenKeyboardIsIdle()
    {
        MobileTouchControls.ResetInputs();
        MobileTouchControls.SetSteering(0.75f);
        MobileTouchControls.SetThrottle(1f);
        MobileTouchControls.SetBrake(0.5f);

        GameObject car = new GameObject("Mobile Input Test Car");
        car.AddComponent<Rigidbody>();
        VehiclePhysicsCoordinator coordinator = car.AddComponent<VehiclePhysicsCoordinator>();
        coordinator.applyProfileOnAwake = false;

        coordinator.SendMessage("Update");

        Assert.AreEqual(0.75f, coordinator.SteeringInput, 0.001f);
        Assert.AreEqual(1f, coordinator.ThrottleInput, 0.001f);
        Assert.AreEqual(0.5f, coordinator.BrakeInput, 0.001f);

        MobileTouchControls.ResetInputs();
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

    private static GameObject CreateAdvancedAssistTestCar(
        float frontSlip,
        float rearSlip,
        out VehiclePhysicsCoordinator coordinator,
        out AdvancedSteeringAssist assist)
    {
        MobileTouchControls.ResetInputs();

        GameObject car = new GameObject("Advanced Steering Assist Test Car");
        Rigidbody rb = car.AddComponent<Rigidbody>();
        TractionSystem traction = car.AddComponent<TractionSystem>();
        assist = car.AddComponent<AdvancedSteeringAssist>();
        coordinator = car.AddComponent<VehiclePhysicsCoordinator>();

        RaycastWheel[] wheels = new RaycastWheel[4];
        for (int i = 0; i < wheels.Length; i++)
        {
            GameObject wheel = new GameObject(i == 0 ? "FL" : i == 1 ? "FR" : i == 2 ? "RL" : "RR");
            wheel.transform.SetParent(car.transform);
            wheel.AddComponent<WheelVisual>();
            wheels[i] = wheel.AddComponent<RaycastWheel>();
            wheels[i].IsGrounded = true;
            wheels[i].LocalSlipVector = new Vector2(i < 2 ? frontSlip : rearSlip, 0f);
        }

        traction.wheels = wheels;
        traction.GripUtilisation[0] = 0.96f;
        traction.GripUtilisation[1] = 0.96f;
        traction.GripUtilisation[2] = 0.96f;
        traction.GripUtilisation[3] = 0.96f;

        coordinator.rb = rb;
        coordinator.wheels = wheels;
        coordinator.traction = traction;
        coordinator.advancedSteeringAssist = assist;
        coordinator.applyProfileOnAwake = false;

        assist.tractionSystem = traction;
        assist.assistSmoothingTime = 0.08f;
        assist.mobileTapSmoothingTime = 0.14f;
        assist.overrideBlendInTime = 0.2f;
        assist.overrideBlendOutTime = 0.32f;

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = car.transform.forward * 35f;
#else
        rb.velocity = car.transform.forward * 35f;
#endif
        coordinator.RefreshState();
        return car;
    }

    private static void RunAdvancedAssistFrames(VehiclePhysicsCoordinator coordinator, AdvancedSteeringAssist assist, int frames)
    {
        for (int i = 0; i < frames; i++)
        {
            coordinator.RefreshState();
            assist.Simulate(coordinator);
        }
    }

    private static GameObject CreateAdvancedBrakeTestCar(
        out VehiclePhysicsCoordinator coordinator,
        out AdvancedBrakingSystem braking,
        out Rigidbody rb)
    {
        MobileTouchControls.ResetInputs();

        GameObject car = new GameObject("Advanced Braking Test Car");
        rb = car.AddComponent<Rigidbody>();
        TractionSystem traction = car.AddComponent<TractionSystem>();
        braking = car.AddComponent<AdvancedBrakingSystem>();
        DrivetrainBrakeSystem drivetrain = car.AddComponent<DrivetrainBrakeSystem>();
        coordinator = car.AddComponent<VehiclePhysicsCoordinator>();

        RaycastWheel[] wheels = new RaycastWheel[4];
        for (int i = 0; i < wheels.Length; i++)
        {
            GameObject wheel = new GameObject(i == 0 ? "FL" : i == 1 ? "FR" : i == 2 ? "RL" : "RR");
            wheel.transform.SetParent(car.transform);
            wheel.AddComponent<WheelVisual>();
            wheels[i] = wheel.AddComponent<RaycastWheel>();
            wheels[i].IsGrounded = true;
            wheels[i].NormalForce = 4200f;
            wheels[i].ContactPoint = wheel.transform.position;
            wheels[i].LocalSlipVector = new Vector2(i >= 2 ? 8f : 0f, 0f);
        }

        traction.wheels = wheels;
        drivetrain.rb = rb;
        drivetrain.wheels = wheels;
        drivetrain.advancedBraking = braking;

        braking.rb = rb;
        braking.tractionSystem = traction;
        braking.drivetrain = drivetrain;
        braking.brakeReleaseRate = 100f;
        braking.maxLateBrakeMultiplier = 1.2f;

        coordinator.rb = rb;
        coordinator.wheels = wheels;
        coordinator.traction = traction;
        coordinator.drivetrain = drivetrain;
        coordinator.advancedBraking = braking;
        coordinator.applyProfileOnAwake = false;

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = car.transform.forward * 70f;
#else
        rb.velocity = car.transform.forward * 70f;
#endif
        coordinator.RefreshState();
        return car;
    }

    private static void ConfigureNoLockup(AdvancedBrakingSystem braking)
    {
        braking.brakePressRate = 100f;
        braking.frontLockupThreshold = 5f;
        braking.rearLockupThreshold = 5f;
        braking.maxLockupEfficiencyLoss = 0f;
    }

    private static void RunAdvancedBrakeFrames(VehiclePhysicsCoordinator coordinator, AdvancedBrakingSystem braking, int frames)
    {
        for (int i = 0; i < frames; i++)
        {
            coordinator.RefreshState();
            braking.Simulate(coordinator);
        }
    }

    private static GameObject CreateAIDriverTestRig(
        out VehiclePhysicsCoordinator coordinator,
        out AIDriverController driver,
        out AIPerceptionSensor sensor,
        out Rigidbody rb)
    {
        MobileTouchControls.ResetInputs();

        GameObject root = new GameObject("AI Driver Test Rig");
        GameObject lineObject = new GameObject("Test Racing Line");
        lineObject.transform.SetParent(root.transform);
        AIRacingLine line = lineObject.AddComponent<AIRacingLine>();
        line.loop = false;

        CreateWaypoint(lineObject.transform, "WP_00", new Vector3(0f, 0f, 0f), 160f);
        CreateWaypoint(lineObject.transform, "WP_01", new Vector3(0f, 0f, 28f), 160f);
        CreateWaypoint(lineObject.transform, "WP_02", new Vector3(0f, 0f, 56f), 160f);
        CreateWaypoint(lineObject.transform, "WP_03", new Vector3(0f, 0f, 84f), 160f);
        line.RefreshWaypoints();

        GameObject car = new GameObject("AI Test Car");
        car.transform.SetParent(root.transform);
        car.transform.position = Vector3.zero;
        rb = car.AddComponent<Rigidbody>();
        car.AddComponent<BoxCollider>().size = new Vector3(2f, 1f, 4f);
        coordinator = car.AddComponent<VehiclePhysicsCoordinator>();
        sensor = car.AddComponent<AIPerceptionSensor>();
        driver = car.AddComponent<AIDriverController>();

        coordinator.applyProfileOnAwake = false;
        coordinator.rb = rb;
        coordinator.UseExternalInput = true;

        sensor.forwardDistance = 20f;
        sensor.sideDistance = 6f;
        sensor.fixedFrameStride = 3;
        sensor.minimumUpdateInterval = 0.07f;

        driver.coordinator = coordinator;
        driver.racingLine = line;
        driver.perception = sensor;
        driver.difficultyPreset = AIDifficultyPreset.Hard;
        driver.baseLookaheadDistance = 18f;
        driver.lookaheadPerKmh = 0f;
        driver.overtakeTrigger = 0.2f;

        coordinator.RefreshState();
        return root;
    }

    private static AIRacingWaypoint CreateWaypoint(Transform parent, string name, Vector3 position, float targetSpeedKmh)
    {
        GameObject waypointObject = new GameObject(name);
        waypointObject.transform.SetParent(parent);
        waypointObject.transform.position = position;
        AIRacingWaypoint waypoint = waypointObject.AddComponent<AIRacingWaypoint>();
        waypoint.targetSpeedKmh = targetSpeedKmh;
        waypoint.laneWidth = 7f;
        waypoint.overtakeWidth = 4f;
        return waypoint;
    }

    private static void SetLineWaypoint(AIRacingLine line, int index, Vector3 position, float targetSpeedKmh)
    {
        AIRacingWaypoint waypoint = line.GetWaypoint(index);
        waypoint.transform.position = position;
        waypoint.targetSpeedKmh = targetSpeedKmh;
        line.RefreshWaypoints();
    }

    private static void SetAllLineSpeeds(AIRacingLine line, float targetSpeedKmh)
    {
        for (int i = 0; i < line.Count; i++)
            line.GetWaypoint(i).targetSpeedKmh = targetSpeedKmh;
    }

    private static GameObject CreateObstacle(string name, Vector3 position)
    {
        GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.name = name;
        obstacle.transform.position = position;
        obstacle.transform.localScale = new Vector3(2f, 1.5f, 2f);
        return obstacle;
    }
}
