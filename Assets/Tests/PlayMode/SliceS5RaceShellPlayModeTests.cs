using System.Collections;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using F1.GameData;
using F1.GameFlow;

/// <summary>
/// Slice step S5b: the race shell, driven through the real route.
///
///     qualifying result -> 50_RaceScene -> same track content -> grid -> field
///
/// PlayMode because the shell only does its work with a live player loop: the flow genuinely
/// transitions scenes, the shell genuinely loads the track content additively, a real car is
/// genuinely instantiated and then genuinely moved onto a grid slot. Asserting this in Edit
/// mode would test a mock of the thing that matters.
///
/// The qualifying lap is injected rather than driven. Driving a lap takes a lap; what S5b
/// changed is what happens to the *result*, so the result is the input under test. The S3
/// tests already cover the drive.
/// </summary>
public class SliceS5RaceShellPlayModeTests
{
    private GameObject _flowObject;

    [UnitySetUp]
    public IEnumerator SetUp()
    {
        GameDataRegistry.Initialize();
        F1.Progression.PlayerProfileManager.GrantStarterContent(
            F1.Progression.PlayerProfileManager.Current);

        _flowObject = new GameObject("S5Test_Flow");
        _flowObject.AddComponent<GameFlowManager>();
        _flowObject.AddComponent<FlowRoot>();

        yield return null;
        yield return new WaitUntil(() =>
            GameFlowManager.Instance?.SceneFlow == null
            || !GameFlowManager.Instance.SceneFlow.IsBusy);
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

    /// <summary>
    /// Selects a car, track and wing, records a qualifying lap of the given time, and enters
    /// the race. The recorded time is the variable under test.
    /// </summary>
    private IEnumerator EnterRaceWithLap(float qualifyingSeconds)
    {
        Flow.SelectCar(GameDataRegistry.FreeCars[0]);
        Flow.SelectTrack(GameDataRegistry.FreeTracks[0]);
        Flow.SelectWing(WingType.LowDownforce);
        Assert.IsTrue(Flow.Session.CanStartQualifying, "Precondition: a car and a track.");

        Flow.SetQualifyingTime(qualifyingSeconds, null);
        Assert.IsTrue(Flow.CanStartRace, "Precondition: a qualifying record exists.");

        Flow.StartRaceSession();
        yield return WaitForScene(FlowSceneNames.Race, 30f);
        yield return WaitForRaceReady(30f);
    }

    private static IEnumerator WaitForScene(string sceneName, float timeout)
    {
        float deadline = Time.realtimeSinceStartup + timeout;
        while (SceneManager.GetActiveScene().name != sceneName &&
               Time.realtimeSinceStartup < deadline)
            yield return null;
    }

    private static IEnumerator WaitForRaceReady(float timeout)
    {
        float deadline = Time.realtimeSinceStartup + timeout;
        while (Time.realtimeSinceStartup < deadline)
        {
            var shell = Object.FindAnyObjectByType<RaceSceneController>();
            if (shell != null && shell.IsSessionReady)
                yield break;
            yield return null;
        }
    }

    private static RaceGridManager Grid() => Object.FindAnyObjectByType<RaceGridManager>();

    [UnityTest]
    public IEnumerator RaceShell_BringsUpTheFieldOnTheGrid()
    {
        yield return EnterRaceWithLap(84f);

        // The race gets its own scene. Before S5b this was the track content scene, so the
        // routing service was asked to load a scene that was already loaded as content.
        Assert.AreEqual(FlowSceneNames.Race, SceneManager.GetActiveScene().name,
            "The race must land in its own shell.");

        var shell = Object.FindAnyObjectByType<RaceSceneController>();
        Assert.IsNotNull(shell, "No race shell controller in the race scene.");
        Assert.IsTrue(shell.IsSessionReady, "The race shell did not finish bringing the session up.");
        Assert.AreEqual(SessionType.Race, Flow.CurrentSessionType);

        // Same track content as qualifying, loaded additively under the shell (invariant 9).
        var track = Object.FindAnyObjectByType<TrackPlacement>();
        Assert.IsNotNull(track, "The race must have the track content loaded.");
        Assert.IsTrue(track.IsValid);
        Assert.IsTrue(Flow.IsTrackContentLoaded);

        // The player car, on the grid and not on the start line.
        var spawner = Object.FindAnyObjectByType<PlayerCarSpawner>();
        Assert.IsNotNull(spawner);
        Assert.IsTrue(spawner.HasSpawned, "The race shell did not spawn the player car.");

        var car = spawner.PlayerCar;
        Assert.IsNotNull(car.GetComponent<LapTracker>(),
            "The car must carry the timing stack, or no lap is ever measured in a race.");

        // The field.
        var grid = Grid();
        Assert.IsNotNull(grid, "The race must drive the track's own grid manager.");
        int expectedAi = grid.aiFieldCount;
        Assert.Greater(expectedAi, 0, "The field is empty, so there is no race.");
        Assert.AreEqual(expectedAi, grid.SpawnedCars.Count,
            "Every car in the field should be on the grid.");

        foreach (var ai in grid.SpawnedCars)
        {
            var id = ai.GetComponent<DriverIdentifier>();
            Assert.IsNotNull(id, "An AI car with no board is indistinguishable from the rest.");
            Assert.Greater(id.GridSlot, -1, "An AI car that was never placed on the grid.");
        }
    }

    [UnityTest]
    public IEnumerator TheQualifyingResultDecidesThePlayersGridSlot()
    {
        // 84s against a field benchmarked 78/84/91 puts the player 6th, which is a slot in
        // the middle of the grid — visibly not pole, and not the back.
        yield return EnterRaceWithLap(84f);

        var spawner = Object.FindAnyObjectByType<PlayerCarSpawner>();
        var grid = Grid();
        var car = spawner.PlayerCar;

        int expected = GridPositionResolver.Resolve(
            Flow.Session.Qualifying.BestLapTime, grid.aiFieldCount, grid.aiField.GetBenchmarkLapSeconds);

        Assert.AreEqual(expected, Flow.RaceGridPosition,
            "The session's grid position must be the one the qualifying result earns.");
        Assert.AreEqual(expected - 1, grid.playerGridPosition,
            "The grid manager counts from zero; the conversion belongs at that boundary.");
        Assert.Greater(expected, 1,
            "An 84s lap should not win pole against this field. If it does, the field's " +
            "benchmark times are wrong, not the rule.");
        Assert.Less(expected, grid.aiFieldCount + 1,
            "The player cannot start behind the last car.");

        // And the car is physically standing in that slot, not merely labelled with it.
        Vector3 slot = grid.GetGridPosition(grid.playerGridPosition);
        Assert.Less(Vector3.Distance(car.transform.position, slot), 2f,
            $"The car should be standing in grid slot {grid.playerGridPosition} but is at " +
            $"{car.transform.position} rather than {slot}. A car whose label says P6 and whose " +
            "body sits on the start line is the failure this assertion exists for.");
    }
}
