using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using F1.GameData;
using F1.GameFlow;

/// <summary>
/// Slice step S3: lap timing, and the flow's race-entry gate opening on a real lap.
///
/// These drive <see cref="LapTracker.Tick"/> with controlled positions rather than relying
/// on physics or frame timing, so the lap-detection rules are tested exactly rather than
/// approximately.
public class SliceS3LapTimingTests
{
    private GameObject _car;
    private TrackPlacement _track;

    [SetUp]
    public void SetUp()
    {
        _track = OpenTrackContent();
        _car = new GameObject("LapTestCar");
        _car.transform.position = _track.RacingLine.GetPosition(_track.StartFinishIndex);
    }

    [TearDown]
    public void TearDown()
    {
        if (_car != null) Object.DestroyImmediate(_car);
    }

    private static TrackPlacement OpenTrackContent()
    {
        UnityEditor.SceneManagement.EditorSceneManager.OpenScene(
            "Assets/Scenes/Track_01.unity",
            UnityEditor.SceneManagement.OpenSceneMode.Single);
        var placement = Object.FindAnyObjectByType<TrackPlacement>();
        Assert.IsNotNull(placement, "Track_01 must contain a TrackPlacement.");
        return placement;
    }

    private static void SetField(object target, string name, object value)
    {
        var field = target.GetType().GetField(name,
            BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.IsNotNull(field, $"No field '{name}' on {target.GetType().Name}.");
        field.SetValue(target, value);
    }

    private LapTracker NewTracker()
    {
        var tracker = _car.AddComponent<LapTracker>();
        // Awake resolves the track from the scene; pin it anyway so the test is explicit
        // about which track it is measuring against.
        SetField(tracker, "_track", _track);
        // The car is teleported between positions in these tests, so the grounded probe
        // would reject legitimate steps. Grounded behaviour is exercised by the running car.
        SetField(tracker, "_requireGrounded", false);
        return tracker;
    }

    /// <summary>Places the car on racing-line waypoint i and ticks the tracker.</summary>
    private void StepTo(int waypointIndex, float deltaTime)
    {
        _car.transform.position = _track.RacingLine.GetPosition(waypointIndex);
        _car.GetComponent<LapTracker>().Tick(deltaTime);
    }

    /// <summary>
    /// Drives exactly one circuit forward from wherever the car currently is.
    ///
    /// Walking waypoints 0..Count-1 is only a lap if the car happens to start at waypoint
    /// 0; the car is placed at the start/finish index, which is not 0.
    /// </summary>
    private void DriveOneLap(float deltaTimePerWaypoint)
    {
        var line = _track.RacingLine;
        int count = line.Count;
        int start = line.FindNearestIndex(_car.transform.position);
        for (int i = 1; i <= count; i++)
            StepTo((start + i) % count, deltaTimePerWaypoint);
    }

    // --- Basic measurement ---

    [Test]
    public void LapTracker_StartsWithNoLapsAndNoTime()
    {
        var tracker = NewTracker();
        tracker.Tick(0.1f); // the first tick only establishes the starting point

        Assert.AreEqual(0, tracker.CompletedLaps);
        Assert.AreEqual(0f, tracker.LastLapTime);
        Assert.AreEqual(0f, tracker.BestLapTime);
        Assert.AreEqual(0f, tracker.TotalDistance, 0.001f);
    }

    [Test]
    public void LapTracker_AccumulatesDistanceWhileMovingForward()
    {
        var tracker = NewTracker();
        tracker.Tick(0.1f);

        for (int i = 1; i <= 10; i++) StepTo(i, 0.1f);

        Assert.Greater(tracker.TotalDistance, 50f,
            "Driving forward should accumulate distance along the track.");
        Assert.AreEqual(0, tracker.CompletedLaps,
            "A partial lap must not count as a lap.");
    }

    [Test]
    public void LapTracker_CompletesExactlyOneLapPerCircuit()
    {
        var tracker = NewTracker();
        tracker.Tick(0.1f);
        DriveOneLap(0.1f);

        Assert.AreEqual(1, tracker.CompletedLaps,
            "Driving a full circuit must complete exactly one lap.");
        Assert.Greater(tracker.LastLapTime, 0f, "A completed lap must have a time.");
        Assert.AreEqual(tracker.LastLapTime, tracker.BestLapTime, 0.001f);
    }

    [Test]
    public void LapTracker_KeepsTheFastestLap()
    {
        var tracker = NewTracker();
        tracker.Tick(0.1f);

        DriveOneLap(0.5f); // slow
        float slow = tracker.BestLapTime;

        DriveOneLap(0.01f); // fast

        Assert.AreEqual(2, tracker.CompletedLaps);
        Assert.Less(tracker.BestLapTime, slow, "The faster lap must become the best.");
        Assert.AreEqual(tracker.BestLapTime, tracker.LastLapTime, 0.001f);
    }

    [Test]
    public void LapTracker_TotalDistanceIsMonotonicAndLapDistanceWraps()
    {
        var tracker = NewTracker();
        tracker.Tick(0.1f);
        DriveOneLap(0.1f);

        // TotalDistance is a monotonic odometer, so after one lap it reads one lap - it is
        // not reset. LapDistance and LapProgress01 are the within-lap values that wrap.
        Assert.AreEqual(_track.LapLengthMeters, tracker.TotalDistance, 5f,
            "After one lap the odometer should read approximately one lap length.");
        Assert.AreEqual(1, tracker.CompletedLaps);

        Assert.Less(Mathf.Abs(tracker.LapDistance), 5f,
            "Lap progress should wrap back to near zero at the start line, but read "
            + tracker.LapDistance.ToString("0.0") + "m.");
        Assert.Less(Mathf.Abs(tracker.LapProgress01), 0.01f,
            "Lap progress should wrap to near zero at the start line.");

        // A second lap advances the odometer again.
        float afterOne = tracker.TotalDistance;
        DriveOneLap(0.1f);
        Assert.AreEqual(2, tracker.CompletedLaps);
        Assert.Greater(tracker.TotalDistance, afterOne,
            "The odometer must keep increasing across laps.");
    }

    // --- The anti-cheat rules, which are why this uses displacement ---

    [Test]
    public void LapTracker_StandingStillOnTheStartLine_CompletesNoLap()
    {
        var tracker = NewTracker();
        tracker.Tick(0.1f);

        for (int i = 0; i < 200; i++)
            StepTo(_track.StartFinishIndex, 0.1f);

        Assert.AreEqual(0, tracker.CompletedLaps,
            "Sitting on the start line must never complete a lap.");
        Assert.AreEqual(0f, tracker.TotalDistance, 0.001f);
    }

    [Test]
    public void LapTracker_IdlingOnAWaypointBoundary_AccumulatesNoDistance()
    {
        // The failure this guards against. With index-delta distance, a car oscillating
        // between two waypoint indices accumulates a whole waypoint of phantom progress per
        // frame and eventually completes a lap that was never driven.
        var tracker = NewTracker();
        var line = _track.RacingLine;

        StepTo(10, 0.1f);
        for (int i = 0; i < 300; i++)
            StepTo(i % 2 == 0 ? 10 : 11, 0.1f);

        Assert.AreEqual(0, tracker.CompletedLaps);
        Assert.Less(Mathf.Abs(tracker.TotalDistance), 60f,
            "Jittering between two waypoints accumulated " +
            tracker.TotalDistance.ToString("0.0") + " m of phantom progress.");
    }

    [Test]
    public void LapTracker_DrivingBackwards_DoesNotCompleteALap()
    {
        var tracker = NewTracker();
        tracker.Tick(0.1f);

        int count = _track.RacingLine.Count;
        for (int i = count - 1; i >= 0; i--)
            StepTo(i, 0.1f);

        Assert.AreEqual(0, tracker.CompletedLaps,
            "Driving the circuit in reverse must not complete a forward lap.");
    }

    [Test]
    public void LapTracker_OscillatingOverTheStartLine_CompletesNoExtraLaps()
    {
        var tracker = NewTracker();
        tracker.Tick(0.1f);
        DriveOneLap(0.1f);
        Assert.AreEqual(1, tracker.CompletedLaps);

        int start = _track.StartFinishIndex;
        int count = _track.RacingLine.Count;
        for (int i = 0; i < 60; i++)
        {
            StepTo(start, 0.1f);
            StepTo((start + 1) % count, 0.1f);
            StepTo(start, 0.1f);
        }

        Assert.AreEqual(1, tracker.CompletedLaps,
            "Oscillating over the start line must not manufacture extra laps.");
    }

    [Test]
    public void LapTracker_ATeleportIsNotCountedAsProgress()
    {
        var tracker = NewTracker();
        tracker.Tick(0.1f);
        for (int i = 1; i <= 5; i++) StepTo(i, 0.1f);
        float before = tracker.TotalDistance;

        int count = _track.RacingLine.Count;
        StepTo((_track.StartFinishIndex + count / 2) % count, 0.1f);

        Assert.AreEqual(before, tracker.TotalDistance, 0.001f,
            "A respawn must not be credited as distance travelled.");
        Assert.AreEqual(0, tracker.CompletedLaps);
    }

    [Test]
    public void LapTracker_SidewaysSliding_ContributesNoDistance()
    {
        var tracker = NewTracker();
        var line = _track.RacingLine;
        int index = 20;

        StepTo(index, 0.1f);

        Vector3 right = line.GetSegmentRight(index).normalized;
        for (int i = 0; i < 20; i++)
        {
            _car.transform.position += right * 5f;
            tracker.Tick(0.1f);
        }

        Assert.AreEqual(0f, tracker.TotalDistance, 0.001f,
            "Sliding sideways across the track is not progress.");
        Assert.AreEqual(0, tracker.CompletedLaps);
    }

    [Test]
    public void LapTracker_ResetClearsEverything()
    {
        var tracker = NewTracker();
        tracker.Tick(0.1f);
        DriveOneLap(0.1f);
        Assert.AreEqual(1, tracker.CompletedLaps);

        tracker.ResetTracker();

        Assert.AreEqual(0, tracker.CompletedLaps);
        Assert.AreEqual(0f, tracker.BestLapTime);
        Assert.AreEqual(0f, tracker.TotalDistance, 0.001f);
    }
}
