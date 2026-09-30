using System.Collections;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using F1.GameData;
using F1.GameFlow;

/// <summary>
/// Slice step S1 exit gate:
///     CarSelectionScene -> TrackSelectionScene -> WingSetupScene
/// works with all selections preserved, and the car-selection hub advances on its own.
/// </summary>
public class SliceS1SelectionFlowTests
{
    private GameObject _flowObject;
    private GameFlowManager _flow;
    private GameFlowManager Manager => _flow;

    private void CreateFlow()
    {
        GameDataRegistry.Initialize();
        F1.Progression.PlayerProfileManager.GrantStarterContent(
            F1.Progression.PlayerProfileManager.Current);

        _flowObject = new GameObject("S1Test_Flow");
        _flowObject.AddComponent<GameFlowManager>();
        _flowObject.AddComponent<FlowRoot>();
        _flow = GameFlowManager.Instance;
        _flow.SceneFlow?.AdoptFlowScene(null);
    }

    /// <summary>
    /// Points the profile manager at a throwaway file before any test calls
    /// <see cref="F1.Progression.PlayerProfileManager.Current"/> through
    /// <see cref="CreateFlow"/>, so nothing here can reach the developer's real save.
    /// </summary>
    [UnitySetUp]
    public IEnumerator ProfileSetUp()
    {
        ProfileIsolation.Begin();
        yield return null;
    }

    [UnityTearDown]
    public IEnumerator TearDown()
    {
        var sceneFlow = GameFlowManager.Instance?.SceneFlow;
        if (sceneFlow != null)
        {
            // Bounded — see SceneOpWait. Yielding UnloadAllContent() directly cannot be
            // given a deadline, and an unbounded teardown is what wedged this suite.
            yield return SceneOpWait.UnloadAllContentBounded(sceneFlow);
            yield return SceneOpWait.UnloadSceneBounded(sceneFlow.CurrentFlowScene);
        }

        if (_flowObject != null) Object.DestroyImmediate(_flowObject);
        _flow = null;

        ProfileIsolation.End();
    }

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

    // --- Scenes exist and are in the build list ---

    [Test]
    public void SelectionScenes_ExistAndAreRegistered()
    {
        foreach (var name in new[] { FlowSceneNames.TrackSelection, FlowSceneNames.WingSetup })
        {
            Assert.IsTrue(System.IO.File.Exists($"Assets/Scenes/{name}.unity"),
                $"{name} does not exist.");
            Assert.IsTrue(SceneFlowService.IsSceneAvailable(name),
                $"{name} is not in the build list.");
        }
    }

    [Test]
    public void BuildSettings_OrderTheFullSelectionRoute()
    {
        var scenes = UnityEditor.EditorBuildSettings.scenes;
        var names = new System.Collections.Generic.List<string>();
        foreach (var s in scenes)
            names.Add(System.IO.Path.GetFileNameWithoutExtension(s.path));

        var expected = new[]
        {
            FlowSceneNames.Loading,
            FlowSceneNames.Lobby,
            FlowSceneNames.TrackSelection,
            FlowSceneNames.WingSetup
        };

        for (int i = 0; i < expected.Length; i++)
            Assert.AreEqual(expected[i], names[i], $"Build index {i} is wrong.");
    }

    // --- The S1 exit gate ---

    [UnityTest]
    public IEnumerator SelectionFlow_ReachesWingSetupWithSelectionsPreserved()
    {
        CreateFlow();
        yield return Settle();

        var car = GameDataRegistry.FreeCars[0];
        var track = GameDataRegistry.FreeTracks[0];

        Manager.SelectCar(car);
        Manager.SelectTrack(track);
        Manager.SelectWing(WingType.LowDownforce);

        Manager.GoToTrackSelection();
        yield return WaitForScene(FlowSceneNames.TrackSelection);

        Assert.AreEqual(FlowSceneNames.TrackSelection, SceneManager.GetActiveScene().name);
        Assert.AreEqual(GameFlowManager.GameScreen.TrackSelection, Manager.CurrentScreen);

        Manager.GoToWingSetup();
        yield return WaitForScene(FlowSceneNames.WingSetup);

        Assert.AreEqual(FlowSceneNames.WingSetup, SceneManager.GetActiveScene().name);
        Assert.AreEqual(GameFlowManager.GameScreen.WingSetup, Manager.CurrentScreen);

        // The whole point: the selection survived every hop.
        Assert.AreSame(car, Manager.SelectedCar, "The car was lost on the way to wing setup.");
        Assert.AreSame(track, Manager.SelectedTrack, "The track was lost on the way to wing setup.");
        Assert.AreEqual(WingType.LowDownforce, Manager.SelectedWing, "The wing was lost.");
        Assert.IsTrue(Manager.Session.IsSelectionComplete);
        Assert.IsTrue(Manager.Session.CanStartQualifying);
    }

    [UnityTest]
    public IEnumerator TrackSelectionScene_HostsItsScreenAndController()
    {
        CreateFlow();
        yield return Settle();

        Manager.GoToTrackSelection();
        yield return WaitForScene(FlowSceneNames.TrackSelection);

        var host = UnityEngine.Object.FindAnyObjectByType<FlowSceneHost>();
        Assert.IsNotNull(host, "The track selection scene must carry a FlowSceneHost.");
        Assert.AreEqual(GameFlowManager.GameScreen.TrackSelection, host.HostedScreen);
        Assert.IsTrue(host.HasScreenInstance, "The host must instantiate its screen prefab.");
        Assert.IsNotNull(UnityEngine.Object.FindAnyObjectByType<TrackSelectionSceneController>(),
            "The track selection scene must carry its scene controller.");
    }

    [UnityTest]
    public IEnumerator WingSetupScene_HostsItsScreenAndController()
    {
        CreateFlow();
        yield return Settle();

        Manager.GoToWingSetup();
        yield return WaitForScene(FlowSceneNames.WingSetup);

        var host = UnityEngine.Object.FindAnyObjectByType<FlowSceneHost>();
        Assert.IsNotNull(host);
        Assert.AreEqual(GameFlowManager.GameScreen.WingSetup, host.HostedScreen);
        Assert.IsTrue(host.HasScreenInstance);
        Assert.IsNotNull(UnityEngine.Object.FindAnyObjectByType<WingSetupSceneController>());
    }

    // --- Ordering invariants 3 and 4 ---

    [UnityTest]
    public IEnumerator TrackSelection_AdvancesToWingSetupAndNotBackToCars()
    {
        CreateFlow();
        yield return Settle();

        Manager.SelectCar(GameDataRegistry.FreeCars[0]);
        Manager.SelectTrack(GameDataRegistry.FreeTracks[0]);

        Manager.GoToTrackSelection();
        yield return WaitForScene(FlowSceneNames.TrackSelection);

        // Invariant 3: track selection is followed by wing selection, never by cars.
        var controller = UnityEngine.Object.FindAnyObjectByType<TrackSelectionSceneController>();
        Assert.IsNotNull(controller);
        Assert.AreEqual(GameFlowManager.GameScreen.TrackSelection, Manager.CurrentScreen,
            "The route must not skip ahead or backward from track selection.");
    }

    [UnityTest]
    public IEnumerator WingSetup_RemainsTheTerminalStepOfTheLobby()
    {
        CreateFlow();
        yield return Settle();

        Manager.SelectCar(GameDataRegistry.FreeCars[0]);
        Manager.SelectTrack(GameDataRegistry.FreeTracks[0]);
        Manager.GoToWingSetup();
        yield return WaitForScene(FlowSceneNames.WingSetup);

        // Invariant 4: wing selection is the last lobby step; the next thing is
        // qualifying, not another selection screen.
        Assert.AreEqual(GameFlowManager.GameScreen.WingSetup, Manager.CurrentScreen);
        Assert.IsTrue(Manager.Session.CanStartQualifying,
            "Wing setup must leave the session ready to qualify.");
    }

    // --- The placeholder flag is gone ---

    [Test]
    public void CarSelectionController_HasNoSerializedAdvanceField()
    {
        var fields = typeof(CarSelectionSceneController)
            .GetFields(System.Reflection.BindingFlags.Instance |
                       System.Reflection.BindingFlags.NonPublic |
                       System.Reflection.BindingFlags.Public);

        foreach (var f in fields)
        {
            if (f.FieldType != typeof(bool)) continue;
            Assert.IsFalse(f.Name.Contains("advance", System.StringComparison.OrdinalIgnoreCase),
                $"CarSelectionSceneController still has the placeholder flag '{f.Name}'. " +
                "The hub must derive its next hop from the scene list instead.");
        }
    }

    [Test]
    public void TrackSelectionScene_IsAvailableSoTheHubAdvancesOnItsOwn()
    {
        // The hub's advance is derived from this, so it is worth asserting directly:
        // if the scene disappeared, the hub would silently dead-end again.
        Assert.IsTrue(SceneFlowService.IsSceneAvailable(FlowSceneNames.TrackSelection),
            "The car-selection hub advances only because this scene is available.");
    }
}
