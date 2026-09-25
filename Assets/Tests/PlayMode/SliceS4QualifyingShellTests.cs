using System.Collections;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using F1.GameData;
using F1.GameFlow;

/// <summary>
/// Slice step S4: the exit gate, driven through the real route.
///
///     WingSetupScene -> PreRaceScene -> drive a lap -> qualifying record exists
///
/// PlayMode, because the shell only does its work with a live player loop: the flow
/// genuinely transitions scenes, the shell genuinely loads the track content additively,
/// and a real car is genuinely instantiated and configured. Asserting this in Edit mode
/// would test a mock of the thing that matters.
///
/// The lap itself is driven by teleporting the spawned car along the racing line and
/// ticking its tracker, the same technique the S3 tests use. That is deliberate: it tests
/// the *chain* — shell spawns a car, the car is timed, the timing reports to the flow, the
/// flow opens the gate — which is what S4 actually changed. Whether the car can be driven
/// by a human with a wheel is the pre-existing, already-demonstrated car, not this step.
/// </summary>
public class SliceS4QualifyingShellTests
{
    private GameObject _flowObject;

    [UnitySetUp]
    public IEnumerator SetUp()
    {
        GameDataRegistry.Initialize();
        F1.Progression.PlayerProfileManager.GrantStarterContent(
            F1.Progression.PlayerProfileManager.Current);

        _flowObject = new GameObject("S4Test_Flow");
        _flowObject.AddComponent<GameFlowManager>();
        _flowObject.AddComponent<FlowRoot>();

        // Let the flow settle before anything is asked of it, exactly as the boot route does.
        yield return Settle();
    }

    [UnityTearDown]
    public IEnumerator TearDown()
    {
        var sceneFlow = GameFlowManager.Instance?.SceneFlow;
        if (sceneFlow != null)
        {
            if (sceneFlow.ContentScenes.Count > 0)
            {
                var unload = sceneFlow.UnloadAllContent();
                if (unload != null) yield return unload;
            }

            var current = sceneFlow.CurrentFlowScene;
            if (current.IsValid() && current.isLoaded)
            {
                var op = SceneManager.UnloadSceneAsync(current);
                if (op != null) yield return op;
            }
        }

        if (_flowObject != null) Object.DestroyImmediate(_flowObject);
    }

    private GameFlowManager Flow => GameFlowManager.Instance;

    private static IEnumerator Settle()
    {
        yield return new WaitUntil(() =>
            GameFlowManager.Instance?.SceneFlow == null
            || !GameFlowManager.Instance.SceneFlow.IsBusy);
        yield return null;
    }

    private static IEnumerator WaitForScene(string sceneName, float timeout = 20f)
    {
        float deadline = Time.realtimeSinceStartup + timeout;
        while (SceneManager.GetActiveScene().name != sceneName &&
               Time.realtimeSinceStartup < deadline)
            yield return null;
    }

    /// <summary>Waits for the shell to finish bringing the session up, not just to load.</summary>
    private static IEnumerator WaitForSpawnedCar(float timeout = 20f)
    {
        float deadline = Time.realtimeSinceStartup + timeout;
        while (Time.realtimeSinceStartup < deadline)
        {
            var spawner = Object.FindAnyObjectByType<PlayerCarSpawner>();
            if (spawner != null && spawner.HasSpawned)
                yield break;
            yield return null;
        }
    }

    private IEnumerator EnterQualifying()
    {
        // Selected programmatically rather than by walking the hub, so the test exercises
        // the pre-race shell rather than re-testing S1's navigation. Navigating to wing setup
        // first would only add three scene transitions of waiting to every test.
        Flow.SelectCar(GameDataRegistry.FreeCars[0]);
        Flow.SelectTrack(GameDataRegistry.FreeTracks[0]);
        Flow.SelectWing(WingType.LowDownforce);

        Assert.IsTrue(Flow.Session.CanStartQualifying,
            "Precondition: a car, a track and a wing are selected.");

        Flow.StartQualifyingSession();
        yield return WaitForScene(FlowSceneNames.PreRace);
        yield return WaitForSpawnedCar();
    }

    /// <summary>
    /// Walks the spawned car around the circuit by teleporting it along the racing line.
    /// The tracker's own grounded check is bypassed for the same reason the S3 tests bypass
    /// it: a teleported car is not resting on the road, and the grounded probe would reject
    /// legitimate steps.
    /// </summary>
    private static void DriveOneLap(GameObject car, TrackPlacement track, float deltaTime)
    {
        var tracker = car.GetComponent<LapTracker>();
        var line = track.RacingLine;
        int count = line.Count;

        tracker.Tick(deltaTime); // establishes the start; contributes nothing to the clock

        int start = line.FindNearestIndex(car.transform.position);
        for (int i = 1; i <= count; i++)
        {
            car.transform.position = line.GetPosition((start + i) % count);
            tracker.Tick(deltaTime);
        }
    }

    private static void DisableGroundedCheck(GameObject car)
    {
        typeof(LapTracker)
            .GetField("_requireGrounded", BindingFlags.Instance | BindingFlags.NonPublic)
            .SetValue(car.GetComponent<LapTracker>(), false);
    }

    // --- The exit gate ---

    [UnityTest]
    public IEnumerator QualifyingSession_SpawnsACarOnTheTrackAndOpensTheGateOnALap()
    {
        yield return EnterQualifying();

        Assert.AreEqual(FlowSceneNames.PreRace, SceneManager.GetActiveScene().name,
            "Qualifying must land in its own shell, not the track content scene.");
        Assert.AreEqual(SessionType.Qualifying, Flow.CurrentSessionType);
        Assert.IsTrue(Flow.IsTrackContentLoaded,
            "The shell must load the shared track content additively (invariant 9).");

        var spawner = Object.FindAnyObjectByType<PlayerCarSpawner>();
        Assert.IsNotNull(spawner, "No car spawner in the pre-race scene.");
        Assert.IsTrue(spawner.HasSpawned, "The shell failed to spawn the player car.");

        var car = spawner.PlayerCar;
        Assert.IsNotNull(car.GetComponent<LapTracker>(),
            "The spawned car must carry the S3 timing stack, or laps are never measured.");
        Assert.IsNotNull(car.GetComponent<QualifyingLapReporter>(),
            "Without the reporter a completed lap never becomes a qualifying record.");
        Assert.IsNotNull(car.GetComponent<LapHudPresenter>(),
            "Without the presenter the lap clock is measured but never shown.");

        // The gate starts shut: the Phase 1 contract, unchanged by S4.
        Assert.IsFalse(Flow.CanStartRace,
            "Race entry must be blocked before any qualifying lap is driven.");

        DisableGroundedCheck(car);
        var track = Object.FindAnyObjectByType<TrackPlacement>();
        Assert.IsNotNull(track, "The loaded track content must provide a TrackPlacement.");

        DriveOneLap(car, track, 0.2f);
        yield return null;

        Assert.AreEqual(1, car.GetComponent<LapTracker>().CompletedLaps,
            "A lap driven on the spawned car should register.");
        Assert.IsTrue(Flow.CanStartRace,
            "A completed qualifying lap must open the race-entry gate.");
        Assert.Greater(Flow.BestQualifyingTime, 0f,
            "The qualifying record must carry a real time.");
    }

    // --- The attempt is a lap, not a menu ---

    /// <summary>
    /// The qualifying screen has two states, and the second one is reached by recording a
    /// lap rather than by arriving at the scene.
    ///
    /// This is asserted on the real prefab rather than on a fake screen, because the thing
    /// that regresses here is the prefab's own structure: the options must live inside the
    /// results panel (so showing the panel and offering "Go to Race" are the same act) and
    /// they must read Back, Retry, Go to Race in that order.
    /// </summary>
    [UnityTest]
    public IEnumerator QualifyingScreen_OffersTheOptionsOnlyAfterALapIsRecorded()
    {
        yield return EnterQualifying();

        var screen = Flow.QualifyingScreenInstance;
        Assert.IsNotNull(screen, "The pre-race shell must resolve a qualifying screen.");
        var root = ((Component)screen).gameObject;

        var panel = FindDeep(root.transform, "ResultsPanel");
        var liveClock = FindDeep(root.transform, "CurrentLapTimeText");
        Assert.IsNotNull(panel, "The qualifying screen must carry a results panel.");
        Assert.IsNotNull(liveClock, "The qualifying screen must carry the live lap clock.");

        Assert.IsFalse(panel.gameObject.activeSelf,
            "The first attempt is a lap, not a menu — the options must not be on screen yet.");
        Assert.IsTrue(liveClock.gameObject.activeSelf,
            "The live lap clock belongs to the driving state.");

        // The order is part of the contract the plan describes, so it is asserted rather
        // than left to whoever next edits the builder.
        var back = FindDeep(panel.transform, "BackButton");
        var retry = FindDeep(panel.transform, "RetryButton");
        var race = FindDeep(panel.transform, "GoToRaceButton");
        Assert.IsNotNull(back, "The results panel must offer Back.");
        Assert.IsNotNull(retry, "The results panel must offer Retry.");
        Assert.IsNotNull(race, "The results panel must offer Go to Race.");
        Assert.Less(back.GetComponent<RectTransform>().anchoredPosition.x,
                    retry.GetComponent<RectTransform>().anchoredPosition.x,
            "Back must come before Retry.");
        Assert.Less(retry.GetComponent<RectTransform>().anchoredPosition.x,
                    race.GetComponent<RectTransform>().anchoredPosition.x,
            "Retry must come before Go to Race.");

        var spawner = Object.FindAnyObjectByType<PlayerCarSpawner>();
        DisableGroundedCheck(spawner.PlayerCar);
        var track = Object.FindAnyObjectByType<TrackPlacement>();
        DriveOneLap(spawner.PlayerCar, track, 0.2f);
        yield return null;

        Assert.IsTrue(panel.gameObject.activeSelf,
            "Recording a qualifying lap must offer the session options.");
        Assert.IsFalse(liveClock.gameObject.activeSelf,
            "A live clock behind a results panel is two lap times at once.");

        var resultTime = FindDeep(panel.transform, "ResultLapTimeText");
        Assert.IsNotNull(resultTime, "The results panel must show the recorded lap time.");
        // Read the label through reflection rather than a TMPro reference: this assembly
        // does not reference TextMeshPro, and adding that reference for one assertion would
        // widen the test assembly's surface for no benefit.
        var label = System.Array.Find(resultTime.GetComponentsInChildren<Component>(true),
            c => c.GetType().GetProperty("text") != null);
        Assert.IsNotNull(label, "The lap time text must have a label carrying its value.");
        var shown = (string)label.GetType().GetProperty("text").GetValue(label);
        Assert.AreNotEqual("0:00.000", shown,
            "The results panel must show the lap that was actually recorded.");

        Flow.RetryQualifyingSession();
        yield return null;

        Assert.IsFalse(panel.gameObject.activeSelf,
            "A retry puts the player back out on track, so the options are not what the " +
            "screen is waiting on any more.");
        Assert.IsTrue(liveClock.gameObject.activeSelf,
            "The lap clock comes back for the retry.");
    }

    /// <summary>
    /// Finds a named descendant, so the test can assert on the prefab's real structure
    /// without the screen implementation having to expose its internals for testing.
    /// </summary>
    private static Transform FindDeep(Transform parent, string name)
    {
        if (parent.name == name) return parent;
        for (int i = 0; i < parent.childCount; i++)
        {
            var hit = FindDeep(parent.GetChild(i), name);
            if (hit != null) return hit;
        }
        return null;
    }

    // --- How and where the car is placed ---

    [UnityTest]
    public IEnumerator SpawnedCar_StartsAtTheCanonicalTrackStartPose()
    {
        yield return EnterQualifying();

        var spawner = Object.FindAnyObjectByType<PlayerCarSpawner>();
        var track = Object.FindAnyObjectByType<TrackPlacement>();
        var car = spawner.PlayerCar.transform;

        // Invariants 9 and 10: qualifying uses the track's own start pose, and the same
        // track content the race will use. The car is not placed relative to anything the
        // shell invented.
        track.GetStartPose(out var expectedPosition, out var expectedRotation);

        Assert.Less(
            Vector3.Distance(car.position, expectedPosition), 2f,
            $"The car should start at the canonical pose ({expectedPosition}) but is at " +
            $"{car.position}.");
        Assert.Less(
            Quaternion.Angle(car.rotation, expectedRotation), 5f,
            "The car should face down the track at the start line.");
    }

    [UnityTest]
    public IEnumerator SpawnedCar_UsesTheSelectedCarsWingProfile()
    {
        yield return EnterQualifying();

        var spawner = Object.FindAnyObjectByType<PlayerCarSpawner>();
        var coordinator = spawner.PlayerCar.GetComponent<VehiclePhysicsCoordinator>();
        Assert.IsNotNull(coordinator);

        // The prefab ships with a default profile so it is drivable in isolation. The wing
        // the player chose has to be what it actually drives with, or the wing screen is
        // decoration.
        var expected = Flow.CreatePlayerPhysicsProfile();
        Assert.IsNotNull(expected, "The selected car should produce a physics profile.");
        Assert.AreSame(expected, coordinator.physicsProfile,
            "The car is not running the selected car's wing profile.");
    }

    [UnityTest]
    public IEnumerator OnlyOnePlayerCarIsSpawned()
    {
        yield return EnterQualifying();

        var coordinators = Object.FindObjectsByType<VehiclePhysicsCoordinator>(FindObjectsSortMode.None);
        var playerCars = coordinators.Count(c => c.GetComponent<LapHudPresenter>() != null);

        Assert.AreEqual(1, playerCars,
            "Qualifying is player-only (invariant 6), so exactly one player car should exist. " +
            "Found " + playerCars + ".");
    }

    // --- The HUD ---

    [UnityTest]
    public IEnumerator QualifyingHud_IsHostedAndShowsTheTrack()
    {
        yield return EnterQualifying();

        var screen = Flow.QualifyingScreenInstance;
        Assert.IsNotNull(screen,
            "The flow must resolve the qualifying screen the shell hosts. Without it the " +
            "lap clock has nowhere to draw.");

        var host = Object.FindAnyObjectByType<FlowSceneHost>();
        Assert.IsNotNull(host);
        Assert.AreEqual(GameFlowManager.GameScreen.Qualifying, host.HostedScreen);
        Assert.IsTrue(host.HasScreenInstance);

        // The hosted screen must be the concrete implementation, not an abstract placeholder.
        // Checked by name: the screen implementations live under Assets/Scripts/UI, which has
        // no assembly definition and therefore compiles into the predefined Assembly-CSharp —
        // and an asmdef-based test assembly cannot reference a predefined one. The abstract
        // contract in F1.GameFlow is reachable, which is all the type check needs.
        Assert.AreEqual("QualifyingScreenImpl", screen.GetType().Name,
            "The hosted qualifying screen should be the concrete UI implementation.");
    }

    // --- Retry ---

    [UnityTest]
    public IEnumerator QualifyingRetry_ReturnsTheSameCarToTheStartPose()
    {
        yield return EnterQualifying();

        var spawner = Object.FindAnyObjectByType<PlayerCarSpawner>();
        var car = spawner.PlayerCar;
        var tracker = car.GetComponent<LapTracker>();
        var track = Object.FindAnyObjectByType<TrackPlacement>();

        DisableGroundedCheck(car);
        DriveOneLap(car, track, 0.2f);
        Assert.AreEqual(1, tracker.CompletedLaps, "Precondition: a lap was completed.");
        Assert.IsTrue(Flow.CanStartRace);

        Flow.RetryQualifyingSession();
        yield return null;
        yield return null;

        // A retry is the same car, track and wing back on the line (plan section 3.2), not a
        // fresh session. The lap state must be cleared or the retry starts a lap already
        // counted, and the car must be stopped or it launches off the line at race speed.
        Assert.AreSame(car, spawner.PlayerCar,
            "A retry should reuse the car, not respawn it.");

        track.GetStartPose(out var expectedPosition, out _);
        Assert.Less(Vector3.Distance(car.transform.position, expectedPosition), 2f,
            "A retry must put the car back on the canonical start pose.");

        var body = car.GetComponent<Rigidbody>();
        Assert.Less(body.linearVelocity.magnitude, 1f,
            "A retry must stop the car. Reusing it at the previous lap's speed would fling " +
            "it off the track.");

        // Attempt counting is the session layer's business; the shell only relays. What
        // matters here is that the shell reset the car rather than leaving it mid-lap.
        Assert.AreEqual(SessionType.Qualifying, Flow.CurrentSessionType);
    }
}
