using System.Collections;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using F1.GameFlow;

public class SceneFlowServiceTests
{
    // Phase 3 replaced LobbyScene with the two flow scenes, so the routing tests point
    // at the scenes that are actually in the build list now.
    private const string FlowScene = F1.GameFlow.FlowSceneNames.Lobby;
    private const string ContentScene = F1.GameFlow.FlowSceneNames.Loading;

    private GameObject _host;
    private GameObject _flowObject;
    private SceneFlowService _service;

    [SetUp]
    public void SetUp()
    {
        _host = new GameObject("SceneFlowServiceTests_Host");
        _host.AddComponent<TestRunner>();
        _service = new SceneFlowService(_host.GetComponent<TestRunner>());
        // Do not adopt the test runner's own scene — these tests drive the transitions
        // explicitly and must not try to unload the scene the runner lives in.
        _service.AdoptFlowScene(null);

        // The flow scenes loaded below carry a FlowSceneHost, which requires the
        // persistent flow root to register with. Provide it so the test environment
        // matches the real runtime instead of loading a scene that logs an error.
        _flowObject = new GameObject("SceneFlowServiceTests_FlowRoot");
        _flowObject.AddComponent<GameFlowManager>();
        _flowObject.AddComponent<F1.GameFlow.FlowRoot>();
    }

    [UnityTearDown]
    public IEnumerator TearDown()
    {
        if (_service != null)
        {
            // Await the unloads; a fire-and-forget unload lets a scene outlive its
            // test and leak into the next one.
            // Bounded — see SceneOpWait. Yielding UnloadAllContent() directly cannot be
            // given a deadline, and an unbounded teardown is what wedged this suite.
            yield return SceneOpWait.UnloadAllContentBounded(_service);
            yield return SceneOpWait.UnloadSceneBounded(_service.CurrentFlowScene);
        }

        if (_flowObject != null)
            Object.DestroyImmediate(_flowObject);

        if (_host != null)
            Object.DestroyImmediate(_host);
    }

    // --- Validation: the check that catches build-settings entries pointing at
    //     files that no longer exist (finding B1). ---

    [Test]
    public void IsSceneAvailable_ScenesInBuildList_AreAvailable()
    {
        Assert.IsTrue(SceneFlowService.IsSceneAvailable(FlowScene),
            $"'{FlowScene}' is in the build list and must be reported as available.");
        Assert.IsTrue(SceneFlowService.IsSceneAvailable(ContentScene),
            $"'{ContentScene}' is in the build list and must be reported as available.");
    }

    [Test]
    public void IsSceneAvailable_UnknownScene_IsRejected()
    {
        Assert.IsFalse(SceneFlowService.IsSceneAvailable("ThisSceneDoesNotExist"));
        Assert.IsFalse(SceneFlowService.IsSceneAvailable(null));
        Assert.IsFalse(SceneFlowService.IsSceneAvailable(""));
        Assert.IsFalse(SceneFlowService.IsSceneAvailable("   "));
    }

    [Test]
    public void IsSceneAvailable_DeletedScene_IsRejected()
    {
        // These three were listed in the build settings but their files were deleted
        // from the working tree. IsSceneAvailable must not vouch for them.
        Assert.IsFalse(SceneFlowService.IsSceneAvailable("ClientDemo_Menu"),
            "A deleted scene must never be reported as available.");
        Assert.IsFalse(SceneFlowService.IsSceneAvailable("ClientDemo_Akash_Scene"));
        Assert.IsFalse(SceneFlowService.IsSceneAvailable("Akash_Scene"));
    }

    // --- Regression guards for finding B1, run against live project state. These
    //     read the build settings and mutate nothing. ---

    [Test]
    public void BuildSettings_EveryEnabledEntryResolvesToAnExistingFile()
    {
        var scenes = UnityEditor.EditorBuildSettings.scenes;
        Assert.Greater(scenes.Length, 0, "The build settings must not be empty.");

        foreach (var entry in scenes)
        {
            if (!entry.enabled)
                continue;

            var guid = UnityEditor.AssetDatabase.AssetPathToGUID(entry.path);
            Assert.IsNotEmpty(guid,
                $"Build-settings entry '{entry.path}' points at a file that does not exist (finding B1).");
        }
    }

    [Test]
    public void BuildSettings_IndexZeroIsTheEntryScene()
    {
        var scenes = UnityEditor.EditorBuildSettings.scenes;
        Assert.Greater(scenes.Length, 0, "The build settings must not be empty.");
        Assert.IsTrue(scenes[0].enabled, "Index 0 must be enabled to be the startup scene.");
        Assert.AreEqual(F1.GameFlow.FlowSceneNames.Loading,
            Path.GetFileNameWithoutExtension(scenes[0].path),
            "Index 0 must be the entry/loading scene.");
    }

    // --- Flow-scene routing ---

    [UnityTest]
    public IEnumerator LoadFlowScene_LoadsAndClaimsOwnership()
    {
        var result = default(SceneLoadResult);
        _service.LoadFlowScene(FlowScene, r => result = r);

        yield return WaitUntil(() => !_service.IsBusy, 10f);

        Assert.IsTrue(result.Success, $"Expected a successful load, got {result}");
        Assert.AreEqual(FlowScene, _service.CurrentFlowSceneName);
        Assert.IsTrue(_service.IsCurrentFlowScene(FlowScene));
        Assert.AreEqual(SceneManager.GetActiveScene().name, FlowScene);
    }

    [UnityTest]
    public IEnumerator LoadFlowScene_SameSceneTwice_DoesNotReload()
    {
        yield return RunFlowLoad(FlowScene);

        int sceneCountAfterFirst = SceneManager.sceneCount;

        // The old self-reference guard compared against gameObject.scene.name, which is
        // "DontDestroyOnLoad" for a persistent object and so never matched. This asserts
        // the handle-based guard (finding L2) actually short-circuits.
        yield return RunFlowLoad(FlowScene);

        Assert.AreEqual(FlowScene, _service.CurrentFlowSceneName);
        Assert.AreEqual(sceneCountAfterFirst, SceneManager.sceneCount,
            "Re-requesting the current flow scene must not load a second copy.");
    }

    [UnityTest]
    public IEnumerator LoadFlowScene_ReplacesARealOutgoingFlowScene()
    {
        // Guards the load-before-unload ordering. Unity refuses to unload the last
        // remaining scene, so routing that unloads first deadlocks when the outgoing
        // flow scene is the only one loaded - which is exactly the case at game boot.
        // Earlier tests called AdoptFlowScene(null), removing the outgoing scene from
        // consideration, which hid that defect.
        //
        // The literal "only scene in the process" case cannot be built here: the test
        // runner's own scene is never unloadable. What is reproducible is the ordering
        // with a real, unloadable outgoing scene, which is what this drives.
        yield return RunFlowLoad(ContentScene);
        Assert.AreEqual(ContentScene, _service.CurrentFlowSceneName,
            "Precondition: the loading scene is now the owned flow scene.");
        Assert.IsTrue(SceneManager.GetSceneByName(ContentScene).isLoaded);

        var result = default(SceneLoadResult);
        _service.LoadFlowScene(FlowScene, r => result = r);
        yield return WaitUntil(() => !_service.IsBusy, 20f);

        Assert.IsTrue(result.Success,
            $"Replacing a real outgoing flow scene must succeed, got {result}");
        Assert.AreEqual(FlowScene, _service.CurrentFlowSceneName);
        Assert.AreEqual(FlowScene, SceneManager.GetActiveScene().name);
        Assert.IsFalse(SceneManager.GetSceneByName(ContentScene).isLoaded,
            "The outgoing flow scene should have been unloaded.");
    }

    [UnityTest]
    public IEnumerator LoadFlowScene_UnknownScene_ReportsNotInBuild()
    {
        var result = default(SceneLoadResult);
        _service.LoadFlowScene("ThisSceneDoesNotExist", r => result = r);

        yield return WaitUntil(() => !_service.IsBusy, 10f);

        Assert.IsFalse(result.Success);
        Assert.AreEqual(SceneLoadStatus.NotInBuild, result.Status);
        Assert.IsNull(_service.CurrentFlowSceneName,
            "A failed load must not claim ownership of a flow scene.");
    }

    [UnityTest]
    public IEnumerator LoadFlowScene_EmptyName_ReportsEmptyName()
    {
        var result = default(SceneLoadResult);
        _service.LoadFlowScene("", r => result = r);

        yield return WaitUntil(() => !_service.IsBusy, 10f);

        Assert.IsFalse(result.Success);
        Assert.AreEqual(SceneLoadStatus.EmptyName, result.Status);
    }

    [UnityTest]
    public IEnumerator OnSceneLoadFailed_FiresForBadScene()
    {
        SceneLoadResult observed = default;
        bool fired = false;
        _service.OnSceneLoadFailed += r => { observed = r; fired = true; };

        yield return RunFlowLoad("ThisSceneDoesNotExist");

        Assert.IsTrue(fired, "OnSceneLoadFailed must fire when a load cannot proceed.");
        Assert.AreEqual(SceneLoadStatus.NotInBuild, observed.Status);
    }

    // --- Additive content lifetime (the track-content contract Phase 5 depends on) ---

    [UnityTest]
    public IEnumerator LoadContent_LoadsAdditively_AndIsTracked()
    {
        var result = default(SceneLoadResult);
        _service.LoadContent(ContentScene, r => result = r);

        yield return WaitUntil(() => !_service.IsBusy, 10f);

        Assert.IsTrue(result.Success, $"Expected a successful content load, got {result}");
        Assert.IsTrue(_service.IsContentLoaded(ContentScene));
        Assert.IsTrue(SceneManager.GetSceneByName(ContentScene).isLoaded);
    }

    [UnityTest]
    public IEnumerator FlowTransition_PreservesLoadedContent()
    {
        yield return RunContentLoad(ContentScene);
        Assert.IsTrue(_service.IsContentLoaded(ContentScene));

        yield return RunFlowLoad(FlowScene);

        Assert.AreEqual(FlowScene, _service.CurrentFlowSceneName);
        Assert.IsTrue(_service.IsContentLoaded(ContentScene),
            "Track content must survive a flow-scene transition (Phase 5 contract).");
        Assert.IsTrue(SceneManager.GetSceneByName(ContentScene).isLoaded,
            "The content scene must still be loaded after the flow transition.");
    }

    [UnityTest]
    public IEnumerator UnloadContent_RemovesTrackingAndUnloadsScene()
    {
        yield return RunContentLoad(ContentScene);
        Assert.IsTrue(_service.IsContentLoaded(ContentScene));

        bool done = false;
        _service.UnloadContent(ContentScene, () => done = true);
        yield return WaitUntil(() => done, 10f);

        Assert.IsFalse(_service.IsContentLoaded(ContentScene));
        Assert.IsFalse(SceneManager.GetSceneByName(ContentScene).isLoaded);
    }

    [UnityTest]
    public IEnumerator UnloadContent_UntrackedScene_IsANoOp()
    {
        bool done = false;
        _service.UnloadContent("SomeSceneNeverLoaded", () => done = true);
        yield return WaitUntil(() => done, 5f);

        Assert.IsTrue(done, "Unloading an untracked scene must still call back, not hang.");
    }

    [UnityTest]
    public IEnumerator Progress_ReachesOneOnSuccessfulLoad()
    {
        float lastProgress = -1f;
        _service.OnProgressChanged += p => lastProgress = p;

        yield return RunFlowLoad(FlowScene);

        Assert.AreEqual(1f, _service.Progress, 0.0001f);
        Assert.AreEqual(1f, lastProgress, 0.0001f);
    }

    // --- Helpers ---

    private IEnumerator RunFlowLoad(string sceneName)
    {
        var finished = false;
        _service.LoadFlowScene(sceneName, r => finished = true);
        yield return WaitUntil(() => finished && !_service.IsBusy, 10f);
    }

    private IEnumerator RunContentLoad(string sceneName)
    {
        var finished = false;
        _service.LoadContent(sceneName, r => finished = true);
        yield return WaitUntil(() => finished && !_service.IsBusy, 10f);
    }

    private static IEnumerator WaitUntil(System.Func<bool> condition, float timeoutSeconds)
    {
        float deadline = Time.realtimeSinceStartup + timeoutSeconds;
        while (!condition() && Time.realtimeSinceStartup < deadline)
            yield return null;
    }

    /// <summary>Minimal MonoBehaviour so the service has something to host coroutines on.</summary>
    private class TestRunner : MonoBehaviour
    {
    }
}
