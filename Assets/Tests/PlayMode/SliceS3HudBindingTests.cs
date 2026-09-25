using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using F1.GameData;
using F1.GameFlow;

/// <summary>
/// Slice step S3, part three: the lap clock actually reaching the HUD.
///
/// <see cref="SliceS3RaceEntryGateTests"/> proves a driven lap opens the race-entry gate.
/// These prove the lap is *visible* while it is being driven — the third and last S3 task,
/// "feed the HUD", which is otherwise a statement nothing checks.
///
/// The HUD is asserted through the screen contract rather than through text in a TMP label.
/// That is deliberate: the label formatting belongs to <c>QualifyingScreenImpl</c>, whereas
/// what can silently regress is the binder reaching the wrong screen, or no screen at all,
/// or pushing a stale value. Those are all visible at the contract boundary.
///
/// Like the other S3 PlayMode tests, these need a real <see cref="GameFlowManager"/>:
/// MonoBehaviour Awake does not run in Edit mode, so the singleton stays null there.
/// </summary>
public class SliceS3HudBindingTests
{
    private GameObject _flowObject;
    private GameObject _car;
    private GameObject _screenObject;
    private LapTracker _tracker;
    private LapHudPresenter _presenter;
    private TrackPlacement _track;
    private Scene _trackScene;
    private RecordingQualifyingScreen _screen;

    private readonly List<string> _presenterLogs = new List<string>();

    [UnitySetUp]
    public IEnumerator SetUp()
    {
        Application.logMessageReceived += OnLog;
        _presenterLogs.Clear();

        GameDataRegistry.Initialize();
        F1.Progression.PlayerProfileManager.GrantStarterContent(
            F1.Progression.PlayerProfileManager.Current);

        // The flow root must exist before the track content is loaded. Track_01 carries a
        // FlowSceneHost, and that host registers with the flow in Awake — loading the scene
        // first leaves it with nothing to register with, which it reports as a hard error.
        _flowObject = new GameObject("S3HudTest_Flow");
        _flowObject.AddComponent<GameFlowManager>();
        _flowObject.AddComponent<FlowRoot>();

        // Take the overlay path instead of loading a scene. There is no 40_PreRaceScene yet
        // (that is S4), so the flow must be told there is no scene to route to; it then
        // resolves the screen directly, which is exactly the resolution path the binder
        // depends on. Without this the test would try to load a scene that does not exist.
        SetField(GameFlowManager.Instance, "_qualifyingScene", string.Empty);
        SetField(GameFlowManager.Instance, "_raceScene", string.Empty);

        var loadOp = SceneManager.LoadSceneAsync(
            FlowSceneNames.TrackContent, LoadSceneMode.Additive);
        while (!loadOp.isDone) yield return null;
        _trackScene = SceneManager.GetSceneByName(FlowSceneNames.TrackContent);

        _track = Object.FindAnyObjectByType<TrackPlacement>();
        Assert.IsNotNull(_track, "Track content must provide a TrackPlacement.");

        // Track_01 carries a FlowSceneHost in QualifyingOrRace mode with no screen prefab, so
        // the flow correctly resolves no qualifying screen from it and never falls through to
        // the legacy lookup. That is the real state of the project: the pre-race shell that
        // will own the qualifying HUD arrives with S4. This test stands in for that shell, so
        // it retires the placeholder host and lets the flow resolve the recording screen the
        // way it resolves any other screen.
        var placeholderHost = Object.FindAnyObjectByType<FlowSceneHost>();
        if (placeholderHost != null)
            GameFlowManager.Instance.UnregisterSceneHost(placeholderHost);

        _car = new GameObject("S3HudTest_Car");
        _car.transform.position = _track.RacingLine.GetPosition(_track.StartFinishIndex);

        _screenObject = new GameObject("S3HudTest_QualifyingScreen");
        _screen = _screenObject.AddComponent<RecordingQualifyingScreen>();

        yield return null; // let Awake run and the singleton settle
    }

    [UnityTearDown]
    public IEnumerator TearDown()
    {
        Application.logMessageReceived -= OnLog;

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

    private void OnLog(string condition, string stackTrace, LogType type)
    {
        if (type == LogType.Warning && condition.StartsWith("[LapHudPresenter]"))
            _presenterLogs.Add(condition);
    }

    private static void SetField(object target, string name, object value)
    {
        var field = target.GetType().GetField(name,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(field, $"No field '{name}' on {target.GetType().Name}.");
        field.SetValue(target, value);
    }

    /// <summary>Adds the timing stack the flow will spawn onto a car.</summary>
    private void AddTimingComponents()
    {
        _tracker = _car.AddComponent<LapTracker>();
        SetField(_tracker, "_track", _track);
        // The car is teleported between racing-line points here, so the grounded probe would
        // reject legitimate steps. Grounded behaviour belongs to the running car.
        SetField(_tracker, "_requireGrounded", false);
        _presenter = _car.AddComponent<LapHudPresenter>();
    }

    private void StartQualifyingWithScreen()
    {
        Flow.SelectCar(GameDataRegistry.FreeCars[0]);
        Flow.SelectTrack(GameDataRegistry.FreeTracks[0]);
        Flow.StartQualifyingSession();
        Assert.AreEqual(SessionType.Qualifying, Flow.CurrentSessionType);
        Assert.IsNotNull(Flow.QualifyingScreenInstance,
            "The flow should have resolved the recording screen.");
        Assert.AreSame(_screen, Flow.QualifyingScreenInstance);
    }

    /// <summary>Drives forward along the racing line, ticking the tracker deterministically.</summary>
    private void DriveSteps(int steps, float deltaTime)
    {
        var line = _track.RacingLine;
        int count = line.Count;
        int start = line.FindNearestIndex(_car.transform.position);
        for (int i = 1; i <= steps; i++)
        {
            _car.transform.position = line.GetPosition((start + i) % count);
            _tracker.Tick(deltaTime);
        }
    }

    // --- The binding ---

    [UnityTest]
    public IEnumerator LapHud_ReceivesTheRunningLapTime()
    {
        AddTimingComponents();
        StartQualifyingWithScreen();

        _tracker.Tick(0.1f); // establish the start
        DriveSteps(5, 0.2f);
        yield return null; // let the presenter's Update read the tracker

        Assert.Greater(_screen.LapTimePushCount, 0,
            "The presenter never pushed to the qualifying screen.");
        Assert.Greater(_screen.LastCurrentLapTime, 0f,
            "The HUD should show a running lap clock.");
        Assert.LessOrEqual(_screen.LastCurrentLapTime, _tracker.CurrentLapTime,
            "The HUD must never show a time ahead of the tracker's own.");
    }

    [UnityTest]
    public IEnumerator LapHud_ReceivesTheBestLapOnceOneIsCompleted()
    {
        AddTimingComponents();
        StartQualifyingWithScreen();

        _tracker.Tick(0.1f);
        DriveSteps(_track.RacingLine.Count, 0.2f); // one full circuit
        Assert.AreEqual(1, _tracker.CompletedLaps, "A lap should have been completed.");

        yield return null;

        Assert.Greater(_tracker.BestLapTime, 0f, "The tracker should hold a best lap.");
        Assert.Greater(_screen.LastBestLapTime, 0f,
            "A completed lap must reach the HUD's best field, or the player never sees it.");
    }

    [UnityTest]
    public IEnumerator LapHud_KeepsPushingAfterTheScreenIsResolved()
    {
        // The binder re-reads the screen handle every push rather than caching it, because
        // the flow replaces the screen on a transition. A cached handle would go stale and
        // silently freeze the clock at whatever the first screen last showed.
        AddTimingComponents();
        StartQualifyingWithScreen();

        DriveSteps(3, 0.2f);
        yield return null;
        int afterFirst = _screen.LapTimePushCount;
        Assert.Greater(afterFirst, 0);

        _tracker.Tick(0.2f);
        yield return null;

        Assert.Greater(_screen.LapTimePushCount, afterFirst,
            "The clock should keep updating, not push once and stop.");
    }

    [UnityTest]
    public IEnumerator LapHud_PushesProgressWhenItMovesAndSkipsIdleFrames()
    {
        // Progress is the raw material for the deferred sector timing, so it is measured and
        // handed over now — but only when it actually changes. Redrawing a progress bar
        // because a stationary car happens to be in a new frame is the kind of per-frame
        // waste that quietly costs more than the feature it supports.
        AddTimingComponents();
        StartQualifyingWithScreen();

        _tracker.Tick(0.1f);
        DriveSteps(5, 0.2f);
        yield return null;

        Assert.Greater(_presenter.ProgressPushCount, 0,
            "Progress should have been pushed while the car was moving.");
        float moved = _presenter.LastPushedProgress;
        Assert.Greater(moved, 0f, "Progress should be non-zero after driving forward.");

        // Sit still. The lap clock keeps running (that push is unconditional) but progress
        // must not be re-pushed for a value that has not moved.
        int afterDriving = _presenter.ProgressPushCount;
        for (int i = 0; i < 5; i++)
        {
            _tracker.Tick(0.2f);
            yield return null;
        }

        Assert.AreEqual(afterDriving, _presenter.ProgressPushCount,
            "A stationary car should not repaint unchanged progress every frame.");
        Assert.AreEqual(moved, _presenter.LastPushedProgress, 0.0001f,
            "The last pushed progress value should be unchanged.");
    }

    // --- Conditions under which it must stay quiet ---

    [UnityTest]
    public IEnumerator LapHud_DoesNotPushDuringTheRace()
    {
        AddTimingComponents();
        // The reporter is what turns a driven lap into a qualifying record, and a qualifying
        // record is what the race-entry gate requires — so it is part of arriving here.
        _car.AddComponent<QualifyingLapReporter>();
        StartQualifyingWithScreen();

        _tracker.Tick(0.1f);
        DriveSteps(_track.RacingLine.Count, 0.2f);
        Assert.IsTrue(Flow.CanStartRace,
            "A driven lap should have opened the race-entry gate.");
        yield return null;
        Assert.Greater(_screen.LapTimePushCount, 0, "Precondition: it pushed while qualifying.");

        // Entering the race resolves a race screen, and there is not one yet: the race HUD
        // arrives with S5, which also owns the lap counter this presenter deliberately
        // leaves alone. The flow reports that as an error, so the test expects it rather
        // than letting an unrelated message about a not-yet-built scene fail the run.
        LogAssert.Expect(LogType.Error,
            new Regex(@"Entered Race but found no screen controller"));
        Flow.StartRaceSession();
        Assert.AreEqual(SessionType.Race, Flow.CurrentSessionType);

        // The race reuses the same tracker for the player and every AI car. The qualifying
        // HUD must not be driven by race laps.
        int before = _screen.LapTimePushCount;
        DriveSteps(3, 0.2f);
        yield return null;
        yield return null;

        Assert.AreEqual(before, _screen.LapTimePushCount,
            "The qualifying HUD must not be driven once the session is a race.");
    }

    [UnityTest]
    public IEnumerator LapHud_MissingScreen_WarnsOnceAndKeepsMeasuring()
    {
        AddTimingComponents();
        StartQualifyingWithScreen();
        Assert.IsNotNull(Flow.QualifyingScreenInstance, "Precondition: a HUD was resolved.");

        // The HUD going away mid-session must not take the lap clock with it, and must not
        // spam the console once a frame. This is the real shape of the failure: the flow
        // holds a handle to a screen that has since been destroyed.
        Object.DestroyImmediate(_screenObject);
        _screenObject = null;
        _screen = null;

        _tracker.Tick(0.1f);
        DriveSteps(3, 0.2f);
        for (int i = 0; i < 5; i++) yield return null;

        // Checked through the UnityEngine.Object static type on purpose: the handle is an
        // interface, and a destroyed component behind an interface still compares equal by
        // reference. That gap is exactly what the presenter's own liveness check exists to
        // close, so the assertion has to look the same way the fix does.
        Assert.IsTrue(Flow.QualifyingScreenInstance as Object == null,
            "Precondition: the resolved screen no longer exists.");
        Assert.AreEqual(1, _presenterLogs.Count,
            "A missing HUD is a real problem, so it is reported — but once, not every frame. " +
            "Saw " + _presenterLogs.Count + " warnings.");

        // The measurement must be unaffected: losing the HUD loses the display, not the lap.
        Assert.Greater(_tracker.CurrentLapTime, 0f,
            "The lap clock should keep running with no HUD attached.");
    }
}
