using System.Collections;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using F1.GameData;
using F1.GameFlow;

/// <summary>
/// Phase 3 exit gate:
///     Game launch -&gt; LoadingScene -&gt; CarSelectionScene
/// works with no main-menu screen.
/// </summary>
public class Phase3FlowSceneTests
{
    private GameObject _flowObject;
    private GameObject _controllerObject;
    private GameObject _hostUnderTest;
    private GameFlowManager _flow;

    private GameFlowManager CreateFlow()
    {
        _flowObject = new GameObject("Phase3Test_Flow");
        _flowObject.AddComponent<GameFlowManager>();
        _flowObject.AddComponent<FlowRoot>();
        _flow = GameFlowManager.Instance;

        // The flow is born in the test runner's scene, which Unity refuses to unload.
        // Drop the adopted birth scene so a transition does not try to unload it. The
        // real entry scene (00_LoadingScene) is unloadable and is loaded normally below.
        _flow.SceneFlow?.AdoptFlowScene(null);
        return _flow;
    }

    /// <summary>
    /// Creates the flow and lets its entry transition to Branding finish. Result is in
    /// <see cref="_flow"/>.
    ///
    /// GameFlowManager.Start() requests Branding the moment it exists, exactly as it
    /// does at game launch. A test that issues its own transition before that finishes
    /// is rejected as Busy, which is correct behaviour but not what these tests are
    /// about. Settling first also means the test's transition performs a real unload of
    /// 00_LoadingScene, which is the path the game actually takes.
    /// </summary>
    private IEnumerator SettleEntryTransition()
    {
        yield return WaitUntil(() => _flow.SceneFlow == null || !_flow.SceneFlow.IsBusy, 20f);
        yield return null;
    }

    /// <summary>
    /// Scenes are unloaded asynchronously. A plain TearDown that fires-and-forgets the
    /// unloads lets a scene outlive its test, so the next test's FlowSceneHost finds a
    /// destroyed GameFlowManager and logs. Await everything instead.
    /// </summary>
    [UnityTearDown]
    public IEnumerator TearDown()
    {
        if (_controllerObject != null) Object.DestroyImmediate(_controllerObject);
        if (_hostUnderTest != null) Object.DestroyImmediate(_hostUnderTest);

        var sceneFlow = GameFlowManager.Instance?.SceneFlow;
        if (sceneFlow != null)
        {
            var content = sceneFlow.ContentScenes;
            if (content.Count > 0)
            {
                var unloadContent = sceneFlow.UnloadAllContent();
                if (unloadContent != null)
                    yield return unloadContent;
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

    private static IEnumerator WaitUntil(System.Func<bool> condition, float timeoutSeconds)
    {
        float deadline = Time.realtimeSinceStartup + timeoutSeconds;
        while (!condition() && Time.realtimeSinceStartup < deadline)
            yield return null;
    }

    // --- Scene assets and the active route ---

    [Test]
    public void FlowScenes_AllExistOnDisk()
    {
        foreach (var name in new[]
                 {
                     FlowSceneNames.Loading,
                     FlowSceneNames.CarSelection,
                     FlowSceneNames.TrackContent
                 })
        {
            Assert.IsTrue(File.Exists(ToAbsolute($"Assets/Scenes/{name}.unity")),
                $"Flow scene '{name}' does not exist at Assets/Scenes/{name}.unity.");
        }
    }

    [Test]
    public void BuildSettings_StartWithLoadingThenCarSelection()
    {
        var scenes = UnityEditor.EditorBuildSettings.scenes;
        Assert.Greater(scenes.Length, 1, "The active route needs at least two scenes.");
        Assert.AreEqual(FlowSceneNames.Loading,
            Path.GetFileNameWithoutExtension(scenes[0].path));
        Assert.AreEqual(FlowSceneNames.CarSelection,
            Path.GetFileNameWithoutExtension(scenes[1].path));
    }

    [Test]
    public void BuildSettings_ExcludesTheLegacyLobby()
    {
        foreach (var entry in UnityEditor.EditorBuildSettings.scenes)
        {
            Assert.AreNotEqual(FlowSceneNames.LegacyLobby,
                Path.GetFileNameWithoutExtension(entry.path),
                "LobbyScene was replaced by the loading + car-selection route and must not ship.");
        }
    }

    [Test]
    public void FlowSceneNames_MatchTheScenesOnDisk()
    {
        // Guards against a constant drifting from the asset it names.
        foreach (var name in new[] { FlowSceneNames.Loading, FlowSceneNames.CarSelection })
        {
            Assert.IsTrue(File.Exists(ToAbsolute($"Assets/Scenes/{name}.unity")),
                $"FlowSceneNames.{name} points at a scene that does not exist.");
            Assert.IsTrue(SceneFlowService.IsSceneAvailable(name),
                $"FlowSceneNames.{name} is not in the build list.");
        }
    }

    [Test]
    public void FutureFlowScenes_AreNamedButNotYetBuilt()
    {
        // Recorded so it is explicit these are scheduled, not forgotten. Track selection
        // and wing setup were built by slice step S1; the pre-race shell (S4), the race
        // shell (S5) and the results scene (S6) are still to come.
        foreach (var name in new[]
                 {
                     FlowSceneNames.PreRace,
                     FlowSceneNames.Race,
                     FlowSceneNames.Results
                 })
        {
            Assert.IsFalse(File.Exists(ToAbsolute($"Assets/Scenes/{name}.unity")),
                $"{name} is scheduled for a later slice step and should not exist yet.");
        }
    }

    // --- The exit gate ---

    [UnityTest]
    public IEnumerator BrandingCompletesIntoCarSelectionNotAMainMenu()
    {
        CreateFlow();
        yield return SettleEntryTransition();
        var flow = _flow;
        flow.GoToCarSelection();

        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.CarSelection, 20f);

        Assert.AreEqual(GameFlowManager.GameScreen.CarSelection, flow.CurrentScreen,
            "The route after branding is car selection, never the main menu.");
        Assert.AreEqual(GameFlowState.CarSelection, flow.CurrentFlowState);
    }

    [UnityTest]
    public IEnumerator MainMenu_IsNotReachableFromTheEntryRoute()
    {
        CreateFlow();
        yield return SettleEntryTransition();
        var flow = _flow;
        flow.GoToCarSelection();

        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.CarSelection, 20f);

        Assert.AreNotEqual(GameFlowManager.GameScreen.MainMenu, flow.CurrentScreen,
            "There is no main menu between loading and car selection (invariant 1).");
        Assert.IsNotNull(flow.CurrentScreen == GameFlowManager.GameScreen.CarSelection
            ? (object)flow.CarSelectionScreenInstance
            : null,
            "Car selection must resolve to a real hosted screen.");
    }

    [UnityTest]
    public IEnumerator LoadingScene_AdvancesToCarSelectionWithNoMainMenu()
    {
        CreateFlow();
        yield return SettleEntryTransition();
        var flow = _flow;

        // Stand in for the loading scene's own controller. The real one lives in
        // 00_LoadingScene; instantiating it here exercises the same policy without
        // tearing down the test runner's scene.
        _controllerObject = new GameObject("LoadingSceneController_UnderTest");
        var controller = _controllerObject.AddComponent<LoadingSceneController>();
        SetMinDisplay(controller, 0f);

        // Wait on the loaded scene, not just the screen enum: TransitionToScreen sets
        // _currentScreen synchronously, before the load it kicks off has finished.
        float deadline = Time.realtimeSinceStartup + 20f;
        while (SceneManager.GetActiveScene().name != FlowSceneNames.CarSelection &&
               Time.realtimeSinceStartup < deadline)
            yield return null;

        Assert.AreEqual(GameFlowManager.GameScreen.CarSelection, flow.CurrentScreen,
            "The loading scene must advance to car selection.");
        Assert.AreNotEqual(GameFlowManager.GameScreen.MainMenu, flow.CurrentScreen,
            "There is no main menu between loading and car selection.");
        Assert.AreEqual(FlowSceneNames.CarSelection, SceneManager.GetActiveScene().name,
            "The car selection scene must be the active scene after the transition.");
    }

    [UnityTest]
    public IEnumerator EnteringCarSelectionScene_HostsItsScreen()
    {
        CreateFlow();
        yield return SettleEntryTransition();
        var flow = _flow;
        flow.GoToCarSelection();

        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.CarSelection, 20f);
        Assert.AreEqual(FlowSceneNames.CarSelection, SceneManager.GetActiveScene().name);

        var host = UnityEngine.Object.FindAnyObjectByType<FlowSceneHost>();
        Assert.IsNotNull(host, "10_CarSelectionScene must carry a FlowSceneHost.");
        Assert.AreEqual(GameFlowManager.GameScreen.CarSelection, host.HostedScreen);
        Assert.IsTrue(host.HasScreenInstance,
            "The host must have instantiated the car selection screen from its prefab.");
    }

    [UnityTest]
    public IEnumerator SceneHost_RegistersSoTheScreenResolvesWithoutSearching()
    {
        CreateFlow();
        yield return SettleEntryTransition();
        var flow = _flow;
        flow.GoToCarSelection();

        yield return WaitUntil(
            () => flow.CarSelectionScreenInstance != null, 20f);

        // Resolved through the host registry, not FindAnyObjectByType, so it still
        // resolves when the host instantiated the screen inactive (fixes finding L4).
        var host = UnityEngine.Object.FindAnyObjectByType<FlowSceneHost>();
        Assert.IsNotNull(host);
        Assert.IsNotNull(host.ScreenInstance);
        Assert.IsNotNull(flow.CarSelectionScreenInstance,
            "GameFlowManager must resolve the hosted car selection screen.");
    }

    [UnityTest]
    public IEnumerator LeavingCarSelection_UnregistersItsHost()
    {
        CreateFlow();
        yield return SettleEntryTransition();
        var flow = _flow;
        flow.GoToCarSelection();
        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.CarSelection, 20f);

        // Route back to the loading scene; the car selection host's scene unloads.
        // The active scene switches before the outgoing scene finishes unloading, so
        // wait for the unload too rather than asserting mid-transition.
        flow.GoToBranding();
        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.Loading, 20f);
        yield return WaitUntil(
            () => !SceneManager.GetSceneByName(FlowSceneNames.CarSelection).isLoaded, 20f);

        Assert.AreEqual(FlowSceneNames.Loading, SceneManager.GetActiveScene().name);
        Assert.IsNull(UnityEngine.Object.FindAnyObjectByType<CarSelectionSceneController>(),
            "No car selection controller should survive its scene unloading.");
    }

    // --- Host configuration ---

    [UnityTest]
    public IEnumerator HostScreenMode_QualifyingOrRace_FollowsTheSessionType()
    {
        GameDataRegistry.Initialize();
        F1.Progression.PlayerProfileManager.GrantStarterContent(
            F1.Progression.PlayerProfileManager.Current);

        CreateFlow();
        yield return SettleEntryTransition();
        var flow = _flow;
        flow.SelectCar(GameDataRegistry.FreeCars[0]);
        flow.SelectTrack(GameDataRegistry.FreeTracks[0]);

        _hostUnderTest = new GameObject("Host_QualifyingOrRace_UnderTest");
        var host = _hostUnderTest.AddComponent<FlowSceneHost>();
        host.Configure(GameFlowManager.GameScreen.Qualifying, null,
            HostScreenMode.QualifyingOrRace);

        // Drive the real flow so the session type is genuinely Qualifying, rather than
        // poking an internal setter the test assembly cannot reach. Waits on the *session
        // type*, not on a scene name: S4 moved qualifying out of the track content scene and
        // into its own shell, so asserting which scene is active here would test the route
        // rather than the mode. The route is covered by the slice tests.
        flow.StartQualifyingSession();
        yield return WaitUntil(
            () => flow.CurrentSessionType == SessionType.Qualifying, 20f);

        Assert.AreEqual(SessionType.Qualifying, flow.CurrentSessionType);
        Assert.AreEqual(GameFlowManager.GameScreen.Qualifying, host.HostedScreen,
            "A host in QualifyingOrRace mode must host the qualifying screen while qualifying.");

        // Qualifying blocks race entry until a valid lap exists (Phase 1 contract).
        Assert.IsFalse(flow.CanStartRace);
        flow.SetQualifyingTime(90f, "ghost-data");
        Assert.IsTrue(flow.CanStartRace);

        flow.StartRaceSession();
        Assert.AreEqual(SessionType.Race, flow.CurrentSessionType);
        Assert.AreEqual(GameFlowManager.GameScreen.Race, host.HostedScreen,
            "A host in QualifyingOrRace mode must host the race screen while racing.");
    }

    private static void SetMinDisplay(LoadingSceneController controller, float seconds)
    {
        var so = new UnityEditor.SerializedObject(controller);
        var prop = so.FindProperty("_minimumDisplaySeconds");
        if (prop != null) prop.floatValue = seconds;
        so.ApplyModifiedPropertiesWithoutUndo();
    }

    private static string ToAbsolute(string projectPath)
    {
        return Path.Combine(
            Application.dataPath.Replace("/Assets", "").Replace("\\Assets", ""),
            projectPath);
    }
}
