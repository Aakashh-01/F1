using System.Collections;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using F1.GameData;
using F1.GameFlow;

/// <summary>
/// Slice step S3, part two: the flow's race-entry gate opening on a real qualifying lap.
///
/// These are PlayMode tests, unlike the pure lap-measurement tests. They need a real
/// <see cref="GameFlowManager"/>: MonoBehaviour <c>Awake</c> does not run in Edit mode, so
/// <c>GameFlowManager.Instance</c> stays null there and every navigation call would NRE.
/// </summary>
public class SliceS3RaceEntryGateTests
{
    private GameObject _flowObject;
    private GameObject _car;
    private GameObject _screenObject;
    private TrackPlacement _track;
    private Scene _trackScene;

    [UnitySetUp]
    public IEnumerator SetUp()
    {
        GameDataRegistry.Initialize();
        F1.Progression.PlayerProfileManager.GrantStarterContent(
            F1.Progression.PlayerProfileManager.Current);

        // The flow root must exist before the track content is loaded. Track_01 carries a
        // FlowSceneHost, and that host registers with the flow in Awake — loading the scene
        // first leaves it with nothing to register with, which it reports as a hard error.
        _flowObject = new GameObject("S3GateTest_Flow");
        _flowObject.AddComponent<GameFlowManager>();
        _flowObject.AddComponent<FlowRoot>();

        // These tests are about the session contract, not scene routing, and the pre-race
        // shell does not exist yet. Routing the qualifying and race screens to an empty
        // scene name keeps the flow resolving its screens in place instead of attempting
        // scene loads that have nothing to do with the gate.
        typeof(GameFlowManager)
            .GetField("_qualifyingScene", BindingFlags.Instance | BindingFlags.NonPublic)
            .SetValue(GameFlowManager.Instance, string.Empty);
        typeof(GameFlowManager)
            .GetField("_raceScene", BindingFlags.Instance | BindingFlags.NonPublic)
            .SetValue(GameFlowManager.Instance, string.Empty);

        var loadOp = SceneManager.LoadSceneAsync(
            FlowSceneNames.TrackContent, LoadSceneMode.Additive);
        while (!loadOp.isDone) yield return null;
        _trackScene = SceneManager.GetSceneByName(FlowSceneNames.TrackContent);

        _track = _trackScene.GetRootGameObjects()
            .SelectMany(go => go.GetComponentsInChildren<TrackPlacement>(true))
            .FirstOrDefault();

        Assert.IsNotNull(_track, "Track content must provide a TrackPlacement.");

        // Track_01 carries a FlowSceneHost in QualifyingOrRace mode with no screen prefab, so
        // the flow correctly resolves no qualifying screen from it and never falls through to
        // the legacy lookup. The pre-race shell that will own the HUD arrives with S4; until
        // then the test supplies the screen itself.
        var placeholderHost = Object.FindAnyObjectByType<FlowSceneHost>();
        if (placeholderHost != null)
            GameFlowManager.Instance.UnregisterSceneHost(placeholderHost);

        // The flow resolves its qualifying screen from the scene, and reports a hard error
        // when it cannot. That is correct behaviour — the HUD is not optional — so the test
        // supplies one rather than silencing the failure.
        _screenObject = new GameObject("S3GateTest_QualifyingScreen");
        _screenObject.AddComponent<RecordingQualifyingScreen>();

        _car = new GameObject("S3GateTest_Car");
        _car.transform.position = _track.RacingLine.GetPosition(_track.StartFinishIndex);

        yield return null; // let Awake and the first Tick run
    }

    [UnityTearDown]
    public IEnumerator TearDown()
    {
        if (_car != null) Object.DestroyImmediate(_car);
        if (_screenObject != null) Object.DestroyImmediate(_screenObject);
        if (_flowObject != null) Object.DestroyImmediate(_flowObject);

        if (_trackScene.IsValid() && _trackScene.isLoaded)
        {
            var op = SceneManager.UnloadSceneAsync(_trackScene);
            if (op != null) yield return op;
        }
    }

    private GameFlowManager Flow => GameFlowManager.Instance;

    private LapTracker NewTracker()
    {
        var tracker = _car.AddComponent<LapTracker>();
        var field = typeof(LapTracker).GetField("_requireGrounded",
            BindingFlags.Instance | BindingFlags.NonPublic);
        field.SetValue(tracker, false);
        return tracker;
    }

    private void DriveOneLap(LapTracker tracker, float deltaTime)
    {
        var line = _track.RacingLine;
        int count = line.Count;

        // The very first Tick only establishes the tracker's starting point and returns. It
        // contributes nothing to the lap clock, but without it the loop below spends its
        // first step on that initialisation and the car crosses the line one waypoint short
        // — which reads as "the lap did not register" rather than as a missing setup step.
        tracker.Tick(deltaTime);

        int start = line.FindNearestIndex(_car.transform.position);
        for (int i = 1; i <= count; i++)
        {
            _car.transform.position = line.GetPosition((start + i) % count);
            tracker.Tick(deltaTime);
        }
    }

    [Test]
    public void RaceEntry_IsBlockedUntilAQualifyingLapIsDriven()
    {
        Flow.SelectCar(GameDataRegistry.FreeCars[0]);
        Flow.SelectTrack(GameDataRegistry.FreeTracks[0]);
        Flow.StartQualifyingSession();

        // The Phase 1 contract: no valid qualifying record, so no race.
        Assert.IsFalse(Flow.CanStartRace,
            "Race entry must be blocked before any qualifying lap is driven.");

        var tracker = NewTracker();
        _car.AddComponent<QualifyingLapReporter>();

        // 0.2s x 90 waypoints = an 18s lap, comfortably above the plausibility floor.
        DriveOneLap(tracker, 0.2f);

        Assert.AreEqual(1, tracker.CompletedLaps, "The lap should have been measured.");
        Assert.IsTrue(Flow.CanStartRace,
            "A completed qualifying lap must open the race-entry gate.");
        Assert.Greater(Flow.BestQualifyingTime, 0f,
            "The qualifying record must carry a time.");
    }

    [Test]
    public void ImplausiblyShortLap_DoesNotOpenTheRaceEntryGate()
    {
        // A tracker reporting a lap one frame after the start would otherwise become the
        // player's permanent best and make the demo unwatchable.
        Flow.SelectCar(GameDataRegistry.FreeCars[0]);
        Flow.SelectTrack(GameDataRegistry.FreeTracks[0]);
        Flow.StartQualifyingSession();

        var tracker = NewTracker();
        _car.AddComponent<QualifyingLapReporter>();

        DriveOneLap(tracker, 0.001f); // 0.09s for a 3.2km lap: impossible

        Assert.AreEqual(1, tracker.CompletedLaps,
            "The tracker measures it regardless; filtering is the reporter's job.");
        Assert.IsFalse(Flow.CanStartRace,
            "An implausibly short lap must not become a qualifying record.");
    }

    [Test]
    public void LapReporter_IsQuietDuringTheRace()
    {
        // The race reuses LapTracker for per-car lap counts. If the reporter also fired
        // there, every car crossing the line would overwrite the player's qualifying
        // record - including the AI, and including a faster race lap.
        Flow.SelectCar(GameDataRegistry.FreeCars[0]);
        Flow.SelectTrack(GameDataRegistry.FreeTracks[0]);

        var tracker = NewTracker();
        _car.AddComponent<QualifyingLapReporter>();

        // Qualify legitimately first.
        Flow.StartQualifyingSession();
        DriveOneLap(tracker, 0.2f);
        float qualifyingTime = Flow.BestQualifyingTime;
        Assert.Greater(qualifyingTime, 0f, "Qualifying should have produced a record.");
        Assert.IsTrue(Flow.CanStartRace);

        // Now race, and drive a much faster lap. Entering the race resolves a race screen,
        // and there is not one yet — the race HUD arrives with S5. The flow reports that as
        // an error, so the test expects it rather than letting a message about a scene that
        // has not been built yet fail the run.
        LogAssert.Expect(LogType.Error,
            new Regex(@"Entered Race but found no screen controller"));
        Flow.StartRaceSession();
        Assert.AreEqual(SessionType.Race, Flow.CurrentSessionType);
        DriveOneLap(tracker, 0.001f);

        Assert.AreEqual(2, tracker.CompletedLaps, "The tracker should keep counting laps.");
        Assert.AreEqual(qualifyingTime, Flow.BestQualifyingTime, 0.001f,
            "A race lap must not overwrite the qualifying record.");
    }
}
