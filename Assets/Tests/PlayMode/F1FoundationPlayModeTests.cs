using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine.TestTools;
using UnityEngine;
using Object = UnityEngine.Object;

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
    public void DrivetrainBrake_DisablesReverseWhenReverseSpeedIsZero()
    {
        GameObject car = new GameObject("Reverse Disabled Test Car");
        BasicMotor drivetrain = car.AddComponent<BasicMotor>();
        drivetrain.reverseMaxSpeedKmh = 0f;
        var method = typeof(DrivetrainBrakeSystem).GetMethod(
            "ShouldUseReverse",
            System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);

        bool atRestUsesReverse = (bool)method.Invoke(drivetrain, new object[] { 1f, 0f, 0f });

        Assert.IsFalse(atRestUsesReverse);
        Object.DestroyImmediate(car);
    }

#if UNITY_EDITOR
    [Test]
    public void AIPhysicsProfile_DisablesReverseForGridLaunch()
    {
        VehiclePhysicsProfile profile = UnityEditor.AssetDatabase.LoadAssetAtPath<VehiclePhysicsProfile>("Assets/Profiles/AI_Profiles/F1_AI_Physics.asset");

        Assert.IsNotNull(profile);
        Assert.AreEqual(0f, profile.drivetrain.reverseMaxSpeedKmh, 0.001f);
    }
#endif

    [Test]
    public void SteeringWithoutAdvancedAssist_ProducesZeroAssistAngle()
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

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = car.transform.forward * 30f;
#else
        rb.velocity = car.transform.forward * 30f;
#endif
        steering.Simulate(null);

        Assert.AreEqual(0f, steering.LastAssistAngle,
            "Without AdvancedSteeringAssist there must be no assist.");
        Object.DestroyImmediate(car);
    }

    [Test]
    public void AdvancedSteeringAssist_IsSoleAssistSource()
    {
        GameObject car = CreateAdvancedAssistTestCar(0f, 16f, out VehiclePhysicsCoordinator coordinator, out AdvancedSteeringAssist assist);
        SteeringSystem steering = coordinator.GetComponent<SteeringSystem>();
        if (steering == null)
            steering = coordinator.gameObject.AddComponent<SteeringSystem>();
        steering.advancedAssist = assist;

        RunAdvancedAssistFrames(coordinator, assist, 4);

        steering.Simulate(coordinator);

        Assert.AreEqual(assist.SmoothedAssistAngle, steering.LastAssistAngle,
            "SteeringSystem must surface exactly the AdvancedSteeringAssist output.");
        Object.DestroyImmediate(car);
    }

    [Test]
    public void Coordinator_RequiresAdvancedSystems_DoesNotAutoAdd()
    {
        LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex("AdvancedBrakingSystem missing"));
        LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex("AdvancedSteeringAssist missing"));

        GameObject car = new GameObject("Coordinator Missing Systems Car");
        car.AddComponent<Rigidbody>();
        VehiclePhysicsCoordinator coordinator = car.AddComponent<VehiclePhysicsCoordinator>();
        coordinator.applyProfileOnAwake = false;

        Assert.IsNull(car.GetComponent<AdvancedBrakingSystem>(),
            "Coordinator must not silently AddComponent AdvancedBrakingSystem.");
        Assert.IsNull(car.GetComponent<AdvancedSteeringAssist>(),
            "Coordinator must not silently AddComponent AdvancedSteeringAssist.");
        Object.DestroyImmediate(car);
    }

    [Test]
    public void Coordinator_AppliesProfileToSerializedSystems()
    {
        MobileTouchControls.ResetInputs();
        GameObject car = new GameObject("Coordinator Profile Car");
        Rigidbody rb = car.AddComponent<Rigidbody>();
        TractionSystem traction = car.AddComponent<TractionSystem>();
        AdvancedSteeringAssist assist = car.AddComponent<AdvancedSteeringAssist>();
        AdvancedBrakingSystem braking = car.AddComponent<AdvancedBrakingSystem>();
        VehiclePhysicsCoordinator coordinator = car.AddComponent<VehiclePhysicsCoordinator>();

        RaycastWheel[] wheels = new RaycastWheel[4];
        for (int i = 0; i < wheels.Length; i++)
        {
            GameObject wheel = new GameObject(i == 0 ? "FL" : i == 1 ? "FR" : i == 2 ? "RL" : "RR");
            wheel.transform.SetParent(car.transform);
            wheel.AddComponent<WheelVisual>();
            wheels[i] = wheel.AddComponent<RaycastWheel>();
            wheels[i].IsGrounded = true;
        }

        assist.tractionSystem = traction;
        braking.rb = rb;
        braking.tractionSystem = traction;
        braking.drivetrain = null;
        braking.weightTransfer = null;

        coordinator.rb = rb;
        coordinator.wheels = wheels;
        coordinator.traction = traction;
        coordinator.advancedSteeringAssist = assist;
        coordinator.advancedBraking = braking;
        coordinator.applyProfileOnAwake = false;

        VehiclePhysicsProfile profile = ScriptableObject.CreateInstance<VehiclePhysicsProfile>();
        profile.advancedSteering.countersteerStrength = 0.61f;
        profile.advancedSteering.assistLevel = SteeringAssistLevel.Low;
        profile.advancedBrake.maxLateBrakeMultiplier = 0.93f;
        profile.advancedBrake.rearInstabilityStrength = 0.21f;

        coordinator.ApplyProfile(profile);

        Assert.AreEqual(0.61f, assist.countersteerStrength);
        Assert.AreEqual(SteeringAssistLevel.Low, assist.assistLevel);
        Assert.AreEqual(0.93f, braking.maxLateBrakeMultiplier);
        Assert.AreEqual(0.21f, braking.rearInstabilityStrength);

        Object.DestroyImmediate(profile);
        Object.DestroyImmediate(car);
    }

    [Test]
    public void WheelSlots_ResolvesByCanonicalNames()
    {
        GameObject car = new GameObject("WheelSlots Resolve Car");
        car.AddComponent<Rigidbody>();
        RaycastWheel[] found = new RaycastWheel[4];
        string[] names = { "RR", "FL", "RL", "FR" }; // deliberately unordered
        for (int i = 0; i < names.Length; i++)
        {
            GameObject wheel = new GameObject(names[i]);
            wheel.transform.SetParent(car.transform);
            wheel.AddComponent<WheelVisual>();
            found[i] = wheel.AddComponent<RaycastWheel>();
        }

        RaycastWheel[] slots = new RaycastWheel[4];
        WheelSlots.ResolveByName(found, slots);

        Assert.AreEqual("FL", slots[WheelSlots.FL].name);
        Assert.AreEqual("FR", slots[WheelSlots.FR].name);
        Assert.AreEqual("RL", slots[WheelSlots.RL].name);
        Assert.AreEqual("RR", slots[WheelSlots.RR].name);
        Object.DestroyImmediate(car);
    }

    [Test]
    public void WheelSlots_MissingSlot_FailsLoudly()
    {
        LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("wheel slot FL is not assigned"));
        LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("wheel slot FR is not assigned"));
        LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("wheel slot RL is not assigned"));
        LogAssert.Expect(LogType.Error, new System.Text.RegularExpressions.Regex("wheel slot RR is not assigned"));

        RaycastWheel[] slots = new RaycastWheel[4];
        bool valid = WheelSlots.Validate(slots, "Validation Car");

        Assert.IsFalse(valid, "Validation must fail when a slot is missing.");
    }

    [Test]
    public void Drivetrain_StandaloneBackfillsWheelsOnceByName()
    {
        GameObject car = new GameObject("Drivetrain Standalone Car");
        Rigidbody rb = car.AddComponent<Rigidbody>();
        DrivetrainBrakeSystem drivetrain = car.AddComponent<BasicMotor>();

        for (int i = 0; i < 4; i++)
        {
            GameObject wheel = new GameObject(i == 0 ? "FL" : i == 1 ? "FR" : i == 2 ? "RL" : "RR");
            wheel.transform.SetParent(car.transform);
            wheel.AddComponent<WheelVisual>();
            wheel.AddComponent<RaycastWheel>();
        }

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = car.transform.forward * 20f;
#else
        rb.velocity = car.transform.forward * 20f;
#endif

        // First Simulate resolves wheels by name; second must reuse the cache.
        drivetrain.Simulate(null);
        float firstBrakeForce = drivetrain.LastBrakeForce;
        drivetrain.Simulate(null);

        Assert.IsNotNull(drivetrain.wheels[0], "FL slot must be resolved by name.");
        Assert.IsNotNull(drivetrain.wheels[3], "RR slot must be resolved by name.");
        Assert.AreEqual(firstBrakeForce, drivetrain.LastBrakeForce);
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
    public void AIDriver_StuckWithoutProgress_EntersRecoveryAndRequestsReverse()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out _, out Rigidbody rb);
        SetAllLineSpeeds(driver.racingLine, 120f);
        driver.stuckDetectionSeconds = 0.2f;
        driver.recoveryReverseSeconds = 0.5f;

        // Wedge the car off to the side of the line so the stop is externally explainable.
        coordinator.transform.position = new Vector3(4f, 0f, 0f);

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.zero;
#else
        rb.velocity = Vector3.zero;
#endif

        int detectTicks = Mathf.CeilToInt(0.2f / Time.fixedDeltaTime) + 4;
        for (int i = 0; i < detectTicks && !driver.InStuckRecovery; i++)
        {
            coordinator.RefreshState();
            driver.Simulate();
        }

        Assert.IsTrue(driver.InStuckRecovery, "AI pinned without progress must enter recovery.");

        // One more tick so ApplyRecoveryInputs overrides the outputs.
        coordinator.RefreshState();
        driver.Simulate();

        Assert.AreEqual(0f, driver.LastThrottleInput, "Recovery must cut throttle.");
        Assert.GreaterOrEqual(driver.LastBrakeInput, 0.99f,
            "Recovery must request brake input (becomes reverse at standstill).");
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_MakingProgress_NeverEntersRecovery()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out _, out Rigidbody rb);
        SetAllLineSpeeds(driver.racingLine, 120f);
        driver.stuckDetectionSeconds = 0.2f;

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.forward * 20f;
#else
        rb.velocity = Vector3.forward * 20f;
#endif

        int ticks = Mathf.CeilToInt(1f / Time.fixedDeltaTime);
        for (int i = 0; i < ticks; i++)
        {
            coordinator.RefreshState();
            driver.Simulate();
        }

        Assert.IsFalse(driver.InStuckRecovery, "A moving AI must never enter recovery.");
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_RecoveryExitsAndCooldownPreventsImmediateRetrigger()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out _, out Rigidbody rb);
        SetAllLineSpeeds(driver.racingLine, 120f);
        driver.stuckDetectionSeconds = 0.1f;
        driver.recoveryReverseSeconds = 0.2f;
        driver.recoveryCooldownSeconds = 3f;

        // Wedge the car off-line so recovery is allowed to trigger.
        coordinator.transform.position = new Vector3(4f, 0f, 0f);

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.zero;
#else
        rb.velocity = Vector3.zero;
#endif

        int detectTicks = Mathf.CeilToInt(0.1f / Time.fixedDeltaTime) + 4;
        for (int i = 0; i < detectTicks && !driver.InStuckRecovery; i++)
        {
            coordinator.RefreshState();
            driver.Simulate();
        }
        Assert.IsTrue(driver.InStuckRecovery);

        int reverseTicks = Mathf.CeilToInt(0.2f / Time.fixedDeltaTime) + 4;
        for (int i = 0; i < reverseTicks && driver.InStuckRecovery; i++)
        {
            coordinator.RefreshState();
            driver.Simulate();
        }
        Assert.IsFalse(driver.InStuckRecovery, "Recovery must end after recoveryReverseSeconds.");

        // Still pinned, but inside cooldown window: must not re-enter yet.
        int cooldownProbeTicks = Mathf.CeilToInt(0.05f / Time.fixedDeltaTime);
        for (int i = 0; i < cooldownProbeTicks; i++)
        {
            coordinator.RefreshState();
            driver.Simulate();
        }
        Assert.IsFalse(driver.InStuckRecovery, "Cooldown must delay the next recovery attempt.");
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
        AIDifficultyProfile medium = AIDifficultyProfile.CreateRuntimeProfile(AIDifficultyPreset.Medium);
        SetLineWaypoint(cornerDriver.racingLine, 2, new Vector3(22f, 0f, 28f), 160f);
        SetLineWaypoint(cornerDriver.racingLine, 3, new Vector3(44f, 0f, 28f), 160f);
        SetAllLineCaution(cornerDriver.racingLine, 0.85f);
        straightDriver.difficultyProfile = medium;
        cornerDriver.difficultyProfile = medium;

        straightCoordinator.RefreshState();
        cornerCoordinator.RefreshState();
        straightDriver.Simulate();
        cornerDriver.Simulate();

        Assert.Greater(cornerDriver.LastCornerCurvature, straightDriver.LastCornerCurvature);
        Assert.Less(cornerDriver.LastTargetSpeedKmh, straightDriver.LastTargetSpeedKmh);
        Object.DestroyImmediate(medium);
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
    public void AIDriver_HardProfileCarriesMoreSpeedThroughCautionCorner()
    {
        GameObject easyRig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator easyCoordinator, out AIDriverController easyDriver, out _, out _);
        GameObject mediumRig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator mediumCoordinator, out AIDriverController mediumDriver, out _, out _);
        GameObject hardRig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator hardCoordinator, out AIDriverController hardDriver, out _, out _);
        AIDifficultyProfile easy = AIDifficultyProfile.CreateRuntimeProfile(AIDifficultyPreset.Easy);
        AIDifficultyProfile medium = AIDifficultyProfile.CreateRuntimeProfile(AIDifficultyPreset.Medium);
        AIDifficultyProfile hard = AIDifficultyProfile.CreateRuntimeProfile(AIDifficultyPreset.Hard);

        ConfigureCautionCorner(easyDriver.racingLine);
        ConfigureCautionCorner(mediumDriver.racingLine);
        ConfigureCautionCorner(hardDriver.racingLine);
        easyDriver.difficultyProfile = easy;
        mediumDriver.difficultyProfile = medium;
        hardDriver.difficultyProfile = hard;

        easyCoordinator.RefreshState();
        mediumCoordinator.RefreshState();
        hardCoordinator.RefreshState();
        easyDriver.Simulate();
        mediumDriver.Simulate();
        hardDriver.Simulate();

        Assert.Greater(mediumDriver.LastTargetSpeedKmh, easyDriver.LastTargetSpeedKmh);
        Assert.Greater(hardDriver.LastTargetSpeedKmh, mediumDriver.LastTargetSpeedKmh);

        Object.DestroyImmediate(easy);
        Object.DestroyImmediate(medium);
        Object.DestroyImmediate(hard);
        Object.DestroyImmediate(easyRig);
        Object.DestroyImmediate(mediumRig);
        Object.DestroyImmediate(hardRig);
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
    public void AIPerception_FrameOffsetStaggersFirstUpdate()
    {
        GameObject rig = CreateAIDriverTestRig(out _, out _, out AIPerceptionSensor sensor, out _);
        sensor.fixedFrameStride = 3;
        sensor.fixedFrameOffset = 1;
        sensor.minimumUpdateInterval = 0.03f;

        Assert.IsFalse(sensor.Tick(false));
        Assert.AreEqual(0, sensor.SensorUpdateSerial);

        Assert.IsTrue(sensor.Tick(false));
        Assert.AreEqual(1, sensor.SensorUpdateSerial);

        Object.DestroyImmediate(rig);
    }

    [Test]
    public void RaceGridManager_ConfiguresAIProfileDifficultyAndSensorOffset()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        GameObject managerObject = new GameObject("Race Grid Manager");
        RaceGridManager manager = managerObject.AddComponent<RaceGridManager>();
        VehiclePhysicsProfile physicsProfile = ScriptableObject.CreateInstance<VehiclePhysicsProfile>();
        AIDifficultyProfile difficultyProfile = AIDifficultyProfile.CreateRuntimeProfile(AIDifficultyPreset.Hard);
        RaceGridManager.RaceGridEntry entry = new RaceGridManager.RaceGridEntry
        {
            existingCar = driver.gameObject,
            physicsProfileOverride = physicsProfile,
            difficultyProfile = difficultyProfile,
            preferRightOvertake = false
        };

        manager.racingLine = driver.racingLine;
        manager.perceptionFrameStride = 4;
        manager.autoAssignCenteredLanes = false;
        manager.ConfigureCar(driver.gameObject, entry, 6);

        Assert.AreSame(physicsProfile, coordinator.physicsProfile);
        Assert.IsTrue(coordinator.UseExternalInput);
        Assert.AreSame(manager.racingLine, driver.racingLine);
        Assert.AreSame(difficultyProfile, driver.difficultyProfile);
        Assert.AreEqual(AIDifficultyPreset.Hard, driver.difficultyPreset);
        Assert.IsFalse(driver.preferRightOvertake);
        Assert.AreEqual(0f, driver.preferredLaneOffset01, 0.001f);
        Assert.AreEqual(4, sensor.fixedFrameStride);
        Assert.AreEqual(2, sensor.fixedFrameOffset);

        Object.DestroyImmediate(physicsProfile);
        Object.DestroyImmediate(difficultyProfile);
        Object.DestroyImmediate(managerObject);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void RaceGridManager_AutoAssignsCenteredEntriesToAlternatingRaceLanes()
    {
        GameObject rightRig = CreateAIDriverTestRig(out _, out AIDriverController rightDriver, out _, out _);
        GameObject leftRig = CreateAIDriverTestRig(out _, out AIDriverController leftDriver, out _, out _);
        GameObject managerObject = new GameObject("Race Grid Manager");
        RaceGridManager manager = managerObject.AddComponent<RaceGridManager>();
        manager.autoAssignCenteredLanes = true;
        manager.autoPreferredLaneOffset01 = 0.42f;
        manager.poleOnRight = true;

        RaceGridManager.RaceGridEntry entry = new RaceGridManager.RaceGridEntry();

        manager.ConfigureCar(rightDriver.gameObject, entry, 0);
        manager.ConfigureCar(leftDriver.gameObject, entry, 1);

        Assert.AreEqual(0.42f, rightDriver.preferredLaneOffset01, 0.001f);
        Assert.AreEqual(-0.42f, leftDriver.preferredLaneOffset01, 0.001f);

        Object.DestroyImmediate(managerObject);
        Object.DestroyImmediate(rightRig);
        Object.DestroyImmediate(leftRig);
    }

    [Test]
    public void RaceGridManager_AssignsPreferredLaneOffset()
    {
        GameObject rig = CreateAIDriverTestRig(out _, out AIDriverController driver, out _, out _);
        GameObject managerObject = new GameObject("Race Grid Manager");
        RaceGridManager manager = managerObject.AddComponent<RaceGridManager>();
        RaceGridManager.RaceGridEntry entry = new RaceGridManager.RaceGridEntry
        {
            existingCar = driver.gameObject,
            preferredLaneOffset01 = -0.55f
        };

        manager.ConfigureCar(driver.gameObject, entry, 1);

        Assert.AreEqual(-0.55f, driver.preferredLaneOffset01, 0.001f);

        Object.DestroyImmediate(managerObject);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void RaceGridManager_IncludesPlayerAndPlacesAIInAlternatingGridSlots()
    {
        GameObject player = new GameObject("Player Grid Car");
        player.transform.position = new Vector3(10f, 0f, 20f);
        player.AddComponent<Rigidbody>();
        player.AddComponent<VehiclePhysicsCoordinator>().applyProfileOnAwake = false;

        GameObject ai = new GameObject("AI Grid Car");
        ai.AddComponent<Rigidbody>();
        ai.AddComponent<VehiclePhysicsCoordinator>().applyProfileOnAwake = false;
        ai.AddComponent<AIPerceptionSensor>();
        ai.AddComponent<AIDriverController>();

        GameObject managerObject = new GameObject("Race Grid Manager");
        RaceGridManager manager = managerObject.AddComponent<RaceGridManager>();
        manager.playerCar = player;
        manager.includePlayerInGrid = true;
        manager.playerGridPosition = 0;
        manager.gridColumnSpacing = 6f;
        manager.gridRowSpacing = 8f;
        manager.gridVerticalOffset = 0f;
        manager.poleOnRight = true;
        manager.aiEntries = new[]
        {
            new RaceGridManager.RaceGridEntry
            {
                existingCar = ai,
                gridPosition = -1
            }
        };

        manager.SpawnGrid();

        Assert.AreEqual(new Vector3(10f, 0f, 20f), player.transform.position);
        Assert.AreEqual(new Vector3(4f, 0f, 20f), ai.transform.position);
        Assert.AreEqual(Quaternion.identity.eulerAngles.y, ai.transform.rotation.eulerAngles.y, 0.01f);

        Object.DestroyImmediate(managerObject);
        Object.DestroyImmediate(ai);
        Object.DestroyImmediate(player);
    }

    [Test]
    public void RaceGridManager_SpawnsWhenExistingCarFieldContainsPrefabAsset()
    {
#if UNITY_EDITOR
        GameObject rig = CreateAIDriverTestRig(out _, out AIDriverController prefabDriver, out _, out _);
        const string folderPath = "Assets/TempTests";
        const string prefabPath = "Assets/TempTests/RaceGridManager_AI_Test.prefab";
        if (!UnityEditor.AssetDatabase.IsValidFolder(folderPath))
            UnityEditor.AssetDatabase.CreateFolder("Assets", "TempTests");

        GameObject prefabSource = Object.Instantiate(prefabDriver.gameObject);
        GameObject prefab = UnityEditor.PrefabUtility.SaveAsPrefabAsset(prefabSource, prefabPath);
        Object.DestroyImmediate(prefabSource);

        GameObject managerObject = new GameObject("Race Grid Manager");
        RaceGridManager manager = managerObject.AddComponent<RaceGridManager>();
        manager.racingLine = prefabDriver.racingLine;
        manager.aiEntries = new[]
        {
            new RaceGridManager.RaceGridEntry
            {
                driverName = "Spawned AI",
                existingCar = prefab,
                fallbackDifficulty = AIDifficultyPreset.Medium
            }
        };

        manager.SpawnGrid();

        Assert.AreEqual(1, manager.SpawnedCars.Count);
        Assert.AreEqual("Spawned AI", manager.SpawnedCars[0].name);
        Assert.IsTrue(manager.SpawnedCars[0].scene.IsValid());
        Assert.AreSame(manager.racingLine, manager.SpawnedCars[0].GetComponent<AIDriverController>().racingLine);

        manager.DestroySpawnedGrid();
        Object.DestroyImmediate(managerObject);
        UnityEditor.AssetDatabase.DeleteAsset(prefabPath);
        if (UnityEditor.AssetDatabase.FindAssets(string.Empty, new[] { folderPath }).Length == 0)
            UnityEditor.AssetDatabase.DeleteAsset(folderPath);
        Object.DestroyImmediate(rig);
#else
        Assert.Ignore("Prefab asset regression requires the Unity Editor.");
#endif
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
        Assert.AreEqual(AISpeedClampReason.EmergencyBrake, driver.LastSpeedClampReason);

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
        Assert.IsTrue(driver.IsOvertaking);

        Object.DestroyImmediate(front);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_ClearOvertakeLaneAvoidsFullTrafficSpeedClamp()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        GameObject front = CreateObstacle("Overtake Speed Front Obstacle", new Vector3(0f, 0.8f, 8f));
        AIDifficultyProfile hard = AIDifficultyProfile.CreateRuntimeProfile(AIDifficultyPreset.Hard);
        driver.difficultyProfile = hard;
        driver.blockedTargetSpeedKmh = 45f;
        driver.preferRightOvertake = true;

        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();

        Assert.IsTrue(driver.IsOvertaking);
        Assert.AreEqual(AISpeedClampReason.TrafficCaution, driver.LastSpeedClampReason);
        Assert.Greater(driver.LastSpeedTargetKmh, driver.blockedTargetSpeedKmh);

        Object.DestroyImmediate(hard);
        Object.DestroyImmediate(front);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_OvertakeLaneCommitmentHoldsDirectionBriefly()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        GameObject front = CreateObstacle("Committed Overtake Front Obstacle", new Vector3(0f, 0.8f, 8f));
        driver.preferRightOvertake = true;

        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();
        float firstOffset = driver.DesiredLaneOffset;

        driver.preferRightOvertake = false;
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();

        Assert.Greater(firstOffset, 0f);
        Assert.Greater(driver.DesiredLaneOffset, 0f);
        Assert.IsTrue(driver.IsOvertaking);

        Object.DestroyImmediate(front);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_UsesPreferredLaneWhenTrackIsClear()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        driver.preferredLaneOffset01 = 0.5f;
        driver.freeTrackLaneUse = 0.6f;
        driver.laneVariationStrength = 0f;

        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();

        Assert.IsFalse(sensor.FrontBlocked);
        Assert.AreEqual(1.68f, driver.DesiredLaneOffset, 0.001f);

        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_HoldsLaneWhenSideTrafficBlocksDesiredLane()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        GameObject rightObstacle = CreateObstacle("Right Side Traffic", new Vector3(4f, 0.8f, 0f));
        driver.preferredLaneOffset01 = 0.8f;
        driver.freeTrackLaneUse = 0.7f;
        driver.laneVariationStrength = 0f;
        driver.sideTrafficLaneHoldBuffer = 0.1f;

        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();

        Assert.IsTrue(sensor.RightBlocked);
        Assert.AreEqual(0f, driver.DesiredLaneOffset, 0.001f);

        Object.DestroyImmediate(rightObstacle);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_UsesSideTrafficCautionInTurns()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        GameObject rightObstacle = CreateObstacle("Right Turn Side Traffic", new Vector3(4f, 0.8f, 0f));
        SetLineWaypoint(driver.racingLine, 1, new Vector3(12f, 0f, 28f), 160f);
        SetLineWaypoint(driver.racingLine, 2, new Vector3(22f, 0f, 56f), 160f);
        driver.preferredLaneOffset01 = 0f;
        driver.laneVariationStrength = 0f;
        driver.sideTrafficSteeringThreshold = 0.2f;
        driver.sideTrafficTurnBrake = 0.16f;

        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();

        Assert.IsTrue(sensor.RightBlocked);
        Assert.AreEqual(AISpeedClampReason.SideTraffic, driver.LastSpeedClampReason);
        Assert.Greater(driver.LastBrakeInput, 0f);

        Object.DestroyImmediate(rightObstacle);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_PackedSharpCornerScalesSpeedForTraffic()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        GameObject rightObstacle = CreateObstacle("Packed Corner Side Traffic", new Vector3(4f, 0.8f, 0f));
        SetLineWaypoint(driver.racingLine, 1, new Vector3(16f, 0f, 20f), 180f);
        SetLineWaypoint(driver.racingLine, 2, new Vector3(32f, 0f, 20f), 180f);
        driver.packCornerCurvatureThreshold = 0.1f;
        driver.packCornerSpeedScale = 0.72f;
        driver.laneVariationStrength = 0f;

        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();

        Assert.IsTrue(sensor.RightBlocked);
        Assert.AreEqual(AISpeedClampReason.SideTraffic, driver.LastSpeedClampReason);
        Assert.LessOrEqual(driver.LastSpeedTargetKmh, driver.LastTargetSpeedKmh * 0.72f + 0.001f);

        Object.DestroyImmediate(rightObstacle);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_BlendsWaypointLaneIntentWithDriverPreference()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        for (int i = 0; i < driver.racingLine.Count; i++)
            driver.racingLine.GetWaypoint(i).preferredLaneOffset01 = -0.5f;

        driver.preferredLaneOffset01 = 0.25f;
        driver.waypointLaneInfluence = 0.5f;
        driver.freeTrackLaneUse = 0.6f;
        driver.laneVariationStrength = 0f;

        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();

        Assert.AreEqual(-0.5f, driver.LastWaypointLaneIntent, 0.001f);
        Assert.AreEqual(0f, driver.DesiredLaneOffset, 0.001f);

        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_OvertakeOffsetKeepsSafetyMarginFromTrackEdge()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        GameObject front = CreateObstacle("Track Edge Safety Front Obstacle", new Vector3(0f, 0.8f, 8f));
        for (int i = 0; i < driver.racingLine.Count; i++)
        {
            AIRacingWaypoint waypoint = driver.racingLine.GetWaypoint(i);
            waypoint.laneWidth = 7f;
            waypoint.overtakeWidth = 10f;
        }
        driver.laneEdgeSafetyMargin = 1.4f;
        driver.preferRightOvertake = true;

        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();

        Assert.IsTrue(driver.IsOvertaking);
        Assert.AreEqual(5.6f, driver.DesiredLaneOffset, 0.001f);

        Object.DestroyImmediate(front);
        Object.DestroyImmediate(rig);
    }

    [Test]
    public void AIDriver_OutsideTrackRecentersAndCapsSpeedForRecovery()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out _);
        driver.transform.position = new Vector3(10f, 0f, 14f);
        driver.preferredLaneOffset01 = 0.8f;
        driver.laneVariationStrength = 0f;
        driver.trackRecoverySpeedKmh = 80f;
        SetAllLineSpeeds(driver.racingLine, 180f);

        Physics.SyncTransforms();
        sensor.Tick(true);
        coordinator.RefreshState();
        driver.Simulate();

        Assert.IsTrue(driver.IsRecoveringTrack);
        Assert.AreEqual(0f, driver.DesiredLaneOffset, 0.001f);
        Assert.AreEqual(80f, driver.LastSpeedTargetKmh, 0.001f);
        Assert.AreEqual(AISpeedClampReason.TrackRecovery, driver.LastSpeedClampReason);

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

    // ---------------------------------------------------------------------
    // AI cars must not drive into each other.
    //
    // The defect these cover: this drivetrain expresses reverse by holding the brake, so a
    // car that brakes hard because it cannot get past the car in front is, at a standstill,
    // indistinguishable from a car that has asked to reverse. Every car in a pack brakes at
    // the same moment, so one car easing off behind traffic used to reverse into the car
    // behind it, and that car then reversed into the one behind it. The separate issue is
    // that nothing modelled the car ahead's SPEED at all, so a pack had no way to hold a gap
    // and simply folded up on itself.
    // ---------------------------------------------------------------------

    /// <summary>
    /// A follower never travels more than the closing cap faster than the car ahead, and
    /// never faster at all once the gap is inside the desired headway. Without this the
    /// target is a function of distance alone and every car in the pack computes the same
    /// one.
    /// </summary>
    [Test]
    public void AIDriver_SlowerLeaderCapsFollowerSpeedByClosingAllowance()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out Rigidbody rb);
        SetAllLineSpeeds(driver.racingLine, 250f);

        // A lead car far enough away that the old distance ramp was barely awake, so this
        // test measures the following term rather than the blind speed clamp.
        sensor.forwardDistance = 60f;
        GameObject leader = CreateRigidCar("Lead Car", new Vector3(0f, 0f, 30f));
        Rigidbody leaderRb = leader.GetComponent<Rigidbody>();

        driver.baseLookaheadDistance = 18f;
        driver.lookaheadPerKmh = 0f;
        driver.overtakeTrigger = 0.99f; // never decide to pass; we want pure following.

        // The leader is doing 90 km/h, well below this car's target of 250.
#if UNITY_6000_0_OR_NEWER
        leaderRb.linearVelocity = new Vector3(0f, 0f, 25f); // 90 km/h
#else
        leaderRb.velocity = new Vector3(0f, 0f, 25f);
#endif
#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = new Vector3(0f, 0f, 50f);
#else
        rb.velocity = new Vector3(0f, 0f, 50f);
#endif

        coordinator.RefreshState();
        sensor.Tick(true);
        sensor.SampleLeaderState();
        driver.Simulate();

        float leaderKmh = sensor.LeaderSpeedKmh;
        Assert.Greater(leaderKmh, 1f, "The sensor must be reading the lead car's speed.");

        float ceiling = leaderKmh + driver.followMaxClosingKmh;
        Assert.LessOrEqual(driver.LastFollowTargetKmh, ceiling + 0.01f,
            $"A follower must not be allowed to travel more than the closing cap faster than the car " +
            $"ahead. Leader was {leaderKmh:F1} km/h, cap {driver.followMaxClosingKmh:F1} km/h, so the " +
            $"follower target must be at most {ceiling:F1} km/h but was {driver.LastFollowTargetKmh:F1} km/h.");

        Object.DestroyImmediate(leader);
        Object.DestroyImmediate(rig);
    }

    /// <summary>
    /// A gap at or inside the desired headway must command the leader's own speed — no
    /// closing allowance at all. This is the term that turns a pack into a convoy.
    /// </summary>
    [Test]
    public void AIDriver_GapInsideHeadwayCommandsLeaderSpeedNotMore()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out Rigidbody rb);
        SetAllLineSpeeds(driver.racingLine, 250f);

        // Place the leader so the gap is smaller than the desired headway even at speed.
        sensor.forwardDistance = 60f;
        float desiredGap = driver.followMinGap + driver.followTimeHeadway * (50f / 3.6f);
        GameObject leader = CreateRigidCar("Lead Car Close", new Vector3(0f, 0f, desiredGap - 1f));
        Rigidbody leaderRb = leader.GetComponent<Rigidbody>();

        driver.baseLookaheadDistance = 18f;
        driver.lookaheadPerKmh = 0f;
        driver.overtakeTrigger = 0.99f;

#if UNITY_6000_0_OR_NEWER
        leaderRb.linearVelocity = new Vector3(0f, 0f, 40f);
        rb.linearVelocity = new Vector3(0f, 0f, 50f);
#else
        leaderRb.velocity = new Vector3(0f, 0f, 40f);
        rb.velocity = new Vector3(0f, 0f, 50f);
#endif

        coordinator.RefreshState();
        sensor.Tick(true);
        sensor.SampleLeaderState();
        driver.Simulate();

        Assert.LessOrEqual(sensor.FrontDistance, desiredGap + 0.5f,
            "The test is only meaningful with the leader inside the desired headway.");
        Assert.LessOrEqual(driver.LastFollowTargetKmh, sensor.LeaderSpeedKmh + 0.01f,
            $"With no gap in hand the follower must match the leader's speed, not exceed it. " +
            $"Leader {sensor.LeaderSpeedKmh:F1} km/h, target {driver.LastFollowTargetKmh:F1} km/h.");

        Object.DestroyImmediate(leader);
        Object.DestroyImmediate(rig);
    }

    /// <summary>
    /// The regression itself: an AI stopped behind a slower car brakes, and that brake must
    /// be a hold. Before the fix the drivetrain read it as reverse and the car drove
    /// backwards into whatever was behind it.
    /// </summary>
    [Test]
    public void AIDriver_TrafficBrakeAtStandstillDoesNotCommandReverse()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out Rigidbody rb);
        SetAllLineSpeeds(driver.racingLine, 200f);

        DrivetrainBrakeSystem drivetrain = driver.gameObject.AddComponent<DrivetrainBrakeSystem>();
        drivetrain.reverseMaxSpeedKmh = 12f;
        coordinator.drivetrain = drivetrain;

        // A stationary lead car right in front, so the follower is up against it and at rest.
        GameObject leader = CreateRigidCar("Stationary Lead Car", new Vector3(0f, 0f, 3f));
        driver.overtakeTrigger = 0.99f;
        driver.stuckDetectionSeconds = 30f; // isolate the traffic brake from the recovery path.

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.zero;
#else
        rb.velocity = Vector3.zero;
#endif

        coordinator.RefreshState();
        sensor.Tick(true);
        driver.Simulate();

        Assert.Greater(driver.LastBrakeInput, 0.5f,
            "A car pinned against a slower car must be asking for brake.");
        Assert.AreEqual(0f, driver.LastThrottleInput, 0.001f,
            "A car pinned against a slower car must not be asking for throttle.");
        Assert.IsTrue(coordinator.SuppressReverse,
            "A brake used to hold position must be marked as a hold, not a reverse request.");

        // Now the part that actually bites: hand those inputs to the drivetrain and confirm
        // it brakes rather than reversing.
        coordinator.RefreshState();
        drivetrain.Simulate(coordinator);
        for (int i = 0; i < 20; i++)
            drivetrain.Simulate(coordinator);

        Assert.AreEqual(0f, drivetrain.CurrentReverse, 0.001f,
            "A car braking to hold position must not be driven backwards. CurrentReverse is the " +
            "force actually applied, so a non-zero value here is the car reversing into the car " +
            "behind it.");

        Object.DestroyImmediate(leader);
        Object.DestroyImmediate(rig);
    }

    /// <summary>
    /// The stuck-recovery manoeuvre is a real reverse, and it must not fire into the car
    /// behind. A wedged car with something in its rear must hold instead.
    /// </summary>
    [Test]
    public void AIDriver_RecoveryRefusesToReverseIntoObstacleBehind()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out AIPerceptionSensor sensor, out Rigidbody rb);
        SetAllLineSpeeds(driver.racingLine, 120f);
        driver.stuckDetectionSeconds = 0.2f;
        driver.recoveryReverseSeconds = 0.5f;
        driver.requireRearClearanceToReverse = true;

        // Wedge the car off the line so recovery is allowed to trigger at all.
        coordinator.transform.position = new Vector3(4f, 0f, 0f);
#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.zero;
#else
        rb.velocity = Vector3.zero;
#endif

        // A car sitting right behind, which is the thing the reverse would hit.
        CreateRigidCar("Car Behind", new Vector3(4f, 0f, -4f));

        int ticks = Mathf.CeilToInt(0.2f / Time.fixedDeltaTime) + 6;
        for (int i = 0; i < ticks; i++)
        {
            coordinator.RefreshState();
            sensor.Tick(true);
            driver.Simulate();
        }

        Assert.IsFalse(driver.LastRearClearToReverse,
            "A car with another car directly behind it has no reverse clearance.");
        Assert.IsFalse(driver.InStuckRecovery,
            "The recovery manoeuvre must be abandoned rather than reverse into the car behind.");
        Assert.IsTrue(coordinator.SuppressReverse,
            "Holding position behind must still be a hold, not a reverse request.");

        Object.DestroyImmediate(rig);
    }

    /// <summary>
    /// A car held at the start gate is not a stuck car. Without this the detector fires for
    /// the whole grid during the countdown and the entire field reverses when the lights go
    /// out.
    /// </summary>
    [Test]
    public void AIDriver_StartGateLockIsNotTreatedAsBeingStuck()
    {
        GameObject rig = CreateAIDriverTestRig(out VehiclePhysicsCoordinator coordinator, out AIDriverController driver, out _, out Rigidbody rb);
        SetAllLineSpeeds(driver.racingLine, 200f);
        driver.stuckDetectionSeconds = 0.1f;
        driver.recoveryReverseSeconds = 0.5f;

        // On its line, stationary, locked at the gate — which is every car in the field.
        coordinator.transform.position = Vector3.zero;
        coordinator.InputLocked = true;
#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.zero;
#else
        rb.velocity = Vector3.zero;
#endif

        int ticks = Mathf.CeilToInt(0.5f / Time.fixedDeltaTime) + 6;
        for (int i = 0; i < ticks; i++)
        {
            coordinator.RefreshState();
            driver.Simulate();
        }

        Assert.IsFalse(driver.InStuckRecovery,
            "A car parked at the start gate must never enter stuck recovery.");
        Object.DestroyImmediate(rig);
    }

    /// <summary>
    /// The end-to-end version of the reported defect, on the real car prefab with the real
    /// drivetrain and real physics: a grid of AI cars launches from a standing start and
    /// must not drive into each other.
    ///
    /// The specific failure being guarded is a REVERSE into the car behind. This drivetrain
    /// expresses reverse by holding the brake, so when a whole field brakes at once at the
    /// start every car in it reads its own brake as a reverse request and backs into the car
    /// it is following. The chain reaction walks backwards up the grid, which is what "the AI
    /// cars collide into themselves" looked like on screen.
    ///
    /// The unit tests above prove each mechanism in isolation. This proves the combination
    /// on the thing the demo actually runs.
    /// </summary>
    [UnityTest]
    public System.Collections.IEnumerator AIField_LaunchingGridNeverReversesIntoEachOther()
    {
        GameObject prefab = LoadAIPrefabOrSkip();
        if (prefab == null)
            yield break;

        GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
        ground.name = "AI Pack Test Ground";
        ground.layer = LayerMask.NameToLayer("Ground");
        ground.transform.position = new Vector3(0f, -0.5f, 0f);
        ground.transform.localScale = new Vector3(80f, 1f, 600f);
        Physics.SyncTransforms();

        // A long straight is where a launch is decided and where a pack is closest together.
        GameObject lineRoot = new GameObject("AI Pack Racing Line");
        AIRacingLine line = lineRoot.AddComponent<AIRacingLine>();
        line.loop = false;
        for (int i = 0; i < 24; i++)
        {
            GameObject wp = new GameObject("WP_" + i);
            wp.transform.SetParent(lineRoot.transform);
            wp.transform.position = new Vector3(0f, 0f, i * 28f);
            AIRacingWaypoint waypoint = wp.AddComponent<AIRacingWaypoint>();
            waypoint.targetSpeedKmh = 220f;
            waypoint.laneWidth = 16f;
        }
        line.RefreshWaypoints();

        // Six cars, real prefab, staggered two-by-two the way the grid does it.
        const int carCount = 6;
        const float rowSpacing = 18f;
        var cars = new List<AIDriverController>();
        for (int i = 0; i < carCount; i++)
        {
            int row = i / 2;
            float side = (i % 2 == 0) ? 6f : -6f;
            Vector3 position = new Vector3(side, 0.05f, -row * rowSpacing);
            GameObject car = Object.Instantiate(prefab, position, Quaternion.identity);
            car.name = "AI Pack Car " + i;

            AIDriverController driver = car.GetComponent<AIDriverController>();
            AIPerceptionSensor sensor = car.GetComponent<AIPerceptionSensor>();
            if (driver == null || sensor == null)
            {
                Object.DestroyImmediate(car);
                Object.DestroyImmediate(ground);
                Object.DestroyImmediate(lineRoot);
                Assert.Fail("The AI prefab must carry both AIDriverController and AIPerceptionSensor.");
                yield break;
            }

            driver.racingLine = line;
            driver.perception = sensor;
            driver.difficultyPreset = AIDifficultyPreset.Hard;
            sensor.forwardDistance = 32f;
            cars.Add(driver);
        }

        Physics.SyncTransforms();

        // Let the pack launch and run. Sample every frame for a reverse and for contact.
        float deadline = Time.time + 12f;
        float worstGap = float.MaxValue;
        string worstGapPair = null;
        var reverseOffenders = new List<string>();

        while (Time.time < deadline)
        {
            for (int i = 0; i < cars.Count; i++)
            {
                AIDriverController driver = cars[i];
                if (driver == null) continue;
                Rigidbody rb = driver.GetComponent<Rigidbody>();
                if (rb == null) continue;

                float forwardKmh = Vector3.Dot(rb.linearVelocity, driver.transform.forward) * 3.6f;
                var coordinator = driver.GetComponent<VehiclePhysicsCoordinator>();
                bool mayReverse = driver.InStuckRecovery || (coordinator != null && coordinator.InputLocked);

                if (!mayReverse && forwardKmh < -2f && reverseOffenders.Count < 5)
                {
                    reverseOffenders.Add(
                        $"{driver.name} rolled backwards at {forwardKmh:F1} km/h " +
                        $"(clamp={driver.LastSpeedClampReason}, front={driver.perception.FrontDistance:F1} m)");
                }

                for (int j = i + 1; j < cars.Count; j++)
                {
                    Rigidbody other = cars[j] != null ? cars[j].GetComponent<Rigidbody>() : null;
                    if (other == null) continue;
                    float gap = Vector3.Distance(rb.position, other.position);
                    if (gap < worstGap)
                    {
                        worstGap = gap;
                        worstGapPair = $"{i} and {j}";
                    }
                }
            }

            yield return null;
        }

        var detail = reverseOffenders.Count > 0
            ? " Cars that reversed: " + string.Join(" | ", reverseOffenders)
            : "";

        Assert.AreEqual(0, reverseOffenders.Count,
            "No AI car may drive backwards except as a deliberate, rear-checked recovery. " +
            "A car braking to hold position must not be read as a request to reverse, or the " +
            "whole field reverses into itself off the line." + detail);

        // The car is 4.48 m long and 2.02 m wide, so centres closer than this are overlapping.
        Assert.Greater(worstGap, 2.5f,
            $"AI cars {worstGapPair} came within {worstGap:F2} m of each other, which is " +
            "contact for a car this size. The pack must hold a gap, not fold up.");

        for (int i = 0; i < cars.Count; i++)
        {
            if (cars[i] != null) Object.DestroyImmediate(cars[i].gameObject);
        }
        Object.DestroyImmediate(ground);
        Object.DestroyImmediate(lineRoot);
    }

    private static GameObject LoadAIPrefabOrSkip()
    {
#if UNITY_EDITOR
        var prefab = UnityEditor.AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/AI_F1_Body.prefab");
        if (prefab == null)
            Assert.Ignore("The AI car prefab is not present at Assets/Prefabs/AI_F1_Body.prefab.");
        return prefab;
#else
        Assert.Ignore("The AI car prefab can only be loaded in the editor.");
        return null;
#endif
    }

    private static GameObject CreateRigidCar(string name, Vector3 position)
    {
        GameObject car = GameObject.CreatePrimitive(PrimitiveType.Cube);
        car.name = name;
        car.transform.position = position;
        car.transform.localScale = new Vector3(2f, 1f, 4f);
        Rigidbody body = car.AddComponent<Rigidbody>();
        body.useGravity = false;
        // Root-level on purpose: these must not share a parent with the car under test, or
        // the sensor's self-collider filter (IsChildOf) would treat the "other car" as part
        // of itself and the test would measure nothing.
        //
        // Tracked for teardown because a root-level object is NOT destroyed with the rig,
        // and one that survives a failing assert is enough to poison every later test in
        // the run — a stray car in the scene makes every subsequent AI look like it is
        // driving into traffic.
        _rigidCarsForCleanup.Add(car);
        return car;
    }

    private static readonly List<GameObject> _rigidCarsForCleanup = new List<GameObject>();

    [TearDown]
    public void TearDownRigidCars()
    {
        for (int i = 0; i < _rigidCarsForCleanup.Count; i++)
        {
            if (_rigidCarsForCleanup[i] != null)
                Object.DestroyImmediate(_rigidCarsForCleanup[i]);
        }

        _rigidCarsForCleanup.Clear();
    }

    private static readonly List<GameObject> _obstaclesForCleanup = new List<GameObject>();

    [TearDown]
    public void TearDownObstacles()
    {
        for (int i = 0; i < _obstaclesForCleanup.Count; i++)
        {
            if (_obstaclesForCleanup[i] != null)
                Object.DestroyImmediate(_obstaclesForCleanup[i]);
        }

        _obstaclesForCleanup.Clear();
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
    public void EngineAudio_MapsSpeedToRpmAndGearWithoutChangingPhysicsInputs()
    {
        GameObject car = new GameObject("Engine Audio Test Car");
        Rigidbody rb = car.AddComponent<Rigidbody>();
        VehiclePhysicsCoordinator coordinator = car.AddComponent<VehiclePhysicsCoordinator>();
        VehiclePhysicsProfile profile = ScriptableObject.CreateInstance<VehiclePhysicsProfile>();
        F1EngineAudioController engine = car.AddComponent<F1EngineAudioController>();

        coordinator.applyProfileOnAwake = false;
        coordinator.rb = rb;
        coordinator.UseExternalInput = true;
        coordinator.physicsProfile = profile;
        coordinator.SetExternalInput(0.2f, 1f, 0f);

        profile.engineAudio.gearCount = 4;
        profile.engineAudio.upshiftSpeedsKmh = new[] { 60f, 110f, 170f };
        engine.coordinator = coordinator;
        engine.physicsProfile = profile;

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.forward * (130f / 3.6f);
#else
        rb.velocity = Vector3.forward * (130f / 3.6f);
#endif
        coordinator.RefreshState();
        engine.RefreshTelemetry(true);

        Assert.AreEqual(3, engine.CurrentGear);
        Assert.Greater(engine.CurrentRpm, profile.engineAudio.idleRpm);
        Assert.AreEqual(1f, coordinator.ThrottleInput, 0.001f);
        Assert.AreEqual(0f, coordinator.BrakeInput, 0.001f);

        Object.DestroyImmediate(profile);
        Object.DestroyImmediate(car);
    }

    [Test]
    public void EngineAudio_GeneratesPlaceholderLoopWhenNoClipAssigned()
    {
        GameObject car = new GameObject("Generated Engine Audio Test Car");
        car.AddComponent<Rigidbody>();
        car.AddComponent<VehiclePhysicsCoordinator>().applyProfileOnAwake = false;
        F1EngineAudioController engine = car.AddComponent<F1EngineAudioController>();

        engine.SendMessage("Awake");

        Assert.IsNotNull(engine.engineSource);
        Assert.IsNotNull(engine.engineSource.clip);
        Assert.IsTrue(engine.engineSource.loop);

        Object.DestroyImmediate(car);
    }

    [Test]
    public void EngineAudio_GeneratesPlaceholderShiftBlipWhenNoClipAssigned()
    {
        GameObject car = new GameObject("Generated Shift Audio Test Car");
        car.AddComponent<Rigidbody>();
        car.AddComponent<VehiclePhysicsCoordinator>().applyProfileOnAwake = false;
        car.AddComponent<AudioSource>();
        car.AddComponent<AudioSource>();
        F1EngineAudioController engine = car.AddComponent<F1EngineAudioController>();

        engine.SendMessage("Awake");

        Assert.IsNotNull(engine.shiftSource);
        Assert.IsNotNull(engine.shiftBlipClip);
        Assert.IsFalse(engine.shiftSource.loop);
        Assert.LessOrEqual(engine.shiftBlipClip.length, 0.3f);

        Object.DestroyImmediate(car);
    }

#if UNITY_EDITOR
    [Test]
    public void PlayerPhysicsProfile_UsesClientBuildMotorPower()
    {
        VehiclePhysicsProfile profile = UnityEditor.AssetDatabase.LoadAssetAtPath<VehiclePhysicsProfile>("Assets/Profiles/F1_Player_Physics.asset");

        Assert.IsNotNull(profile);
        Assert.LessOrEqual(profile.drivetrain.motorForce, 100000f);
        Assert.LessOrEqual(profile.drivetrain.throttleSpoolSpeed, 1.35f);
    }
#endif

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

    [Test]
    public void CameraSpeedPerception_IncreasesCinemachineFovWithSpeed()
    {
        Type cinemachineType = Type.GetType("Unity.Cinemachine.CinemachineCamera, Unity.Cinemachine");
        if (cinemachineType == null)
            Assert.Ignore("Cinemachine package type is not available in this test environment.");

        GameObject cameraObject = new GameObject("Speed Cinemachine Camera");
        Component cinemachineCamera = cameraObject.AddComponent(cinemachineType);
        CameraSpeedPerception perception = cameraObject.AddComponent<CameraSpeedPerception>();
        GameObject target = new GameObject("Target Car");
        Rigidbody rb = target.AddComponent<Rigidbody>();

        SetCinemachineTestFov(cinemachineCamera, 60f);
        perception.cinemachineCamera = cinemachineCamera;
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

        Assert.Greater(GetCinemachineTestFov(cinemachineCamera), 60f);
        Object.DestroyImmediate(cameraObject);
        Object.DestroyImmediate(target);
    }

    [Test]
    public void F1CameraTargetRig_TracksPlayerWithoutUi()
    {
        GameObject car = new GameObject("Camera Target Car");
        Rigidbody rb = car.AddComponent<Rigidbody>();
        SteeringSystem steering = car.AddComponent<SteeringSystem>();
        GameObject target = new GameObject("Camera Target Rig");
        F1CameraTargetRig rig = target.AddComponent<F1CameraTargetRig>();
        rig.car = car.transform;
        rig.carRb = rb;
        rig.steeringSystem = steering;
        rig.positionSharpness = 30f;
        rig.rotationSharpness = 30f;
        rig.SnapToTarget();

        Vector3 initialPosition = target.transform.position;
        car.transform.position = new Vector3(0f, 0f, 12f);
#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.forward * 35f;
#else
        rb.velocity = Vector3.forward * 35f;
#endif
        rig.SendMessage("LateUpdate");

        Assert.Greater(target.transform.position.z, initialPosition.z);
        Object.DestroyImmediate(target);
        Object.DestroyImmediate(car);
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

    private static float GetCinemachineTestFov(Component cinemachineCamera)
    {
        object lens = GetCinemachineTestLens(cinemachineCamera, out _);
        FieldInfo fovField = lens.GetType().GetField("FieldOfView");
        return (float)fovField.GetValue(lens);
    }

    private static void SetCinemachineTestFov(Component cinemachineCamera, float fov)
    {
        object lens = GetCinemachineTestLens(cinemachineCamera, out FieldInfo lensField);
        FieldInfo fovField = lens.GetType().GetField("FieldOfView");
        fovField.SetValue(lens, fov);
        lensField.SetValue(cinemachineCamera, lens);
    }

    private static object GetCinemachineTestLens(Component cinemachineCamera, out FieldInfo lensField)
    {
        lensField = cinemachineCamera.GetType().GetField("Lens", BindingFlags.Instance | BindingFlags.Public);
        Assert.IsNotNull(lensField);
        object lens = lensField.GetValue(cinemachineCamera);
        Assert.IsNotNull(lens);
        return lens;
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

    private static void ConfigureCautionCorner(AIRacingLine line)
    {
        SetLineWaypoint(line, 2, new Vector3(22f, 0f, 28f), 120f);
        SetLineWaypoint(line, 3, new Vector3(44f, 0f, 28f), 120f);
        SetAllLineCaution(line, 0.85f);
    }

    private static void SetAllLineSpeeds(AIRacingLine line, float targetSpeedKmh)
    {
        for (int i = 0; i < line.Count; i++)
            line.GetWaypoint(i).targetSpeedKmh = targetSpeedKmh;
    }

    private static void SetAllLineCaution(AIRacingLine line, float brakingCaution)
    {
        for (int i = 0; i < line.Count; i++)
            line.GetWaypoint(i).brakingCaution = brakingCaution;
    }

    private static GameObject CreateObstacle(string name, Vector3 position)
    {
        GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.name = name;
        obstacle.transform.position = position;
        obstacle.transform.localScale = new Vector3(2f, 1.5f, 2f);
        // Root-level for the same reason CreateRigidCar is, and tracked for the same reason.
        // Several callers assert *after* creating this, so an assertion that throws skips
        // their own DestroyImmediate and leaves a 2x1.5x2 cube parked in the scene. The
        // sensor under test then finds that cube as traffic for the rest of the run, which
        // is why the AI failure count drifted between runs of identical code.
        _obstaclesForCleanup.Add(obstacle);
        return obstacle;
    }
}
