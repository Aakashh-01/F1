using System.IO;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using F1.GameFlow;

/// <summary>
/// Scene-content checks that can only run in the Editor. They inspect scene assets
/// directly, which is forbidden during Play mode.
/// </summary>
public class SceneContentTests
{
    [SetUp]
    public void SetUp()
    {
        // Start each test from a known scene so an earlier test's open scene cannot
        // leak into the next.
        EditorSceneManager.OpenScene(
            $"Assets/Scenes/{FlowSceneNames.Loading}.unity", OpenSceneMode.Single);
    }

    [Test]
    public void EveryBuildScene_HasNoMissingScripts()
    {
        // FlowRoot originally shared FlowBootstrap.cs, and Unity serialized it into
        // 00_LoadingScene and LobbyScene as a bare `m_Script: {fileID: ...}` with no
        // GUID. Both scenes carried a missing script in that component slot and nothing
        // complained. One MonoBehaviour per file gives a stable .cs.meta GUID; this test
        // is the guard for that class of breakage.
        var scenes = EditorBuildSettings.scenes;
        Assert.Greater(scenes.Length, 0, "The build settings must not be empty.");

        foreach (var entry in scenes)
        {
            var scene = EditorSceneManager.OpenScene(entry.path, OpenSceneMode.Single);
            foreach (var root in scene.GetRootGameObjects())
            {
                foreach (var component in root.GetComponents<Component>())
                {
                    Assert.IsNotNull(component,
                        $"'{root.name}' in {entry.path} has a missing script component.");
                }
            }
        }
    }

    [Test]
    public void LoadingScene_CarriesThePersistentFlowRoot()
    {
        var scene = EditorSceneManager.OpenScene(
            $"Assets/Scenes/{FlowSceneNames.Loading}.unity", OpenSceneMode.Single);

        var flow = Object.FindAnyObjectByType<GameFlowManager>();
        Assert.IsNotNull(flow, "The entry scene must host the GameFlowManager.");
        Assert.IsNotNull(flow.GetComponent<FlowBootstrap>(),
            "The entry scene must host the FlowBootstrap.");
        Assert.IsNotNull(flow.GetComponent<FlowRoot>(),
            "The entry scene must host the FlowRoot marker.");
    }

    [Test]
    public void LoadingScene_HostIsWiredToTheBrandingScreen()
    {
        var scene = EditorSceneManager.OpenScene(
            $"Assets/Scenes/{FlowSceneNames.Loading}.unity", OpenSceneMode.Single);

        var host = Object.FindAnyObjectByType<FlowSceneHost>();
        Assert.IsNotNull(host, "The loading scene must carry a FlowSceneHost.");
        Assert.AreEqual(GameFlowManager.GameScreen.Branding, host.HostedScreen);

        var prefab = HostScreenPrefab(host);
        Assert.IsNotNull(prefab, "The loading scene's host must have a branding screen prefab.");

        // The GDD requires the app name and the Pearl-Lemon attribution.
        var texts = prefab.GetComponentsInChildren<TMPro.TMP_Text>(true);
        var all = string.Join("\n", System.Array.ConvertAll(texts, t => t.text));
        StringAssert.Contains("Developed by Pearl-Lemon", all,
            "The GDD requires 'Developed by Pearl-Lemon' on the branding screen.");
    }

    // CarSelectionScene_HostsOnlyItsOwnScreen and CarSelectionScene_DoesNotAlsoHostBranding
    // were removed with 10_CarSelectionScene. Car selection is now the lobby's second UI
    // state, and its coverage is carried by the scene-list sweep below plus
    // Phase3FlowSceneTests.EnteringLobbyScene_HostsItsScreen.

    // TrackScene_HostIsFixedToRace was removed because it asserted an arrangement two slices
    // ago. It demanded the track content scene carry a FlowSceneHost on Race; S5 then gave the
    // race a shell of its own, 50_RaceScene hosts Race, and the content scene's host went with
    // it. The track scene is loaded additively under whichever session is running, so a host
    // there can only compete with a shell for screen resolution, and the winner was decided by
    // load order. Two tests now assert the correct arrangement, and they contradict this one
    // directly: SliceS4PreRaceSceneTests.TrackScene_HostsNoScreenAtAllSoItCannotShadowAShell
    // and SliceS5RaceShellTests.TrackContentCarriesNoScreenHostAndNoRaceSession. The S4 test
    // says in its own comment that the assertion moved from "hosts Race" to "hosts nothing",
    // which is this test being left behind. It had never passed on a fresh checkout — the scene
    // it opens has carried no host since before the test was written.

    [Test]
    public void EveryFlowSceneInTheBuildList_ExistsAndLoads()
    {
        var scenes = EditorBuildSettings.scenes;
        foreach (var entry in scenes)
        {
            Assert.IsTrue(File.Exists(ToAbsolute(entry.path)),
                $"Build-settings entry '{entry.path}' points at a missing file.");
            var scene = EditorSceneManager.OpenScene(entry.path, OpenSceneMode.Single);
            Assert.IsTrue(scene.IsValid(), $"'{entry.path}' could not be opened.");
        }
    }

    [Test]
    public void TrackSelectionScene_IsWiredCorrectly()
    {
        var scene = EditorSceneManager.OpenScene(
            $"Assets/Scenes/{FlowSceneNames.TrackSelection}.unity", OpenSceneMode.Single);

        var host = Object.FindAnyObjectByType<FlowSceneHost>();
        Assert.IsNotNull(host, "The track selection scene must carry a FlowSceneHost.");
        Assert.AreEqual(GameFlowManager.GameScreen.TrackSelection, host.HostedScreen);

        var prefab = HostScreenPrefab(host);
        Assert.IsNotNull(prefab, "The host must have a screen prefab assigned.");
        Assert.IsNotNull(prefab.GetComponent<TrackSelectionScreen>(),
            "The hosted prefab must actually be a track selection screen.");

        Assert.IsNotNull(Object.FindAnyObjectByType<TrackSelectionSceneController>(),
            "The track selection scene must carry its scene controller.");
    }

    [Test]
    public void WingSetupScene_IsWiredCorrectly()
    {
        var scene = EditorSceneManager.OpenScene(
            $"Assets/Scenes/{FlowSceneNames.WingSetup}.unity", OpenSceneMode.Single);

        var host = Object.FindAnyObjectByType<FlowSceneHost>();
        Assert.IsNotNull(host, "The wing setup scene must carry a FlowSceneHost.");
        Assert.AreEqual(GameFlowManager.GameScreen.WingSetup, host.HostedScreen);

        var prefab = HostScreenPrefab(host);
        Assert.IsNotNull(prefab, "The host must have a screen prefab assigned.");
        Assert.IsNotNull(prefab.GetComponent<WingSetupScreen>(),
            "The hosted prefab must actually be a wing setup screen.");

        Assert.IsNotNull(Object.FindAnyObjectByType<WingSetupSceneController>(),
            "The wing setup scene must carry its scene controller.");
    }

    [Test]
    public void EveryFlowScene_HostsExactlyOneScreen()
    {
        // One screen per scene. Two hosts in a scene means one of them is unreachable.
        foreach (var name in new[]
                 {
                     FlowSceneNames.Loading,
                     FlowSceneNames.Lobby,
                     FlowSceneNames.TrackSelection,
                     FlowSceneNames.WingSetup
                 })
        {
            var scene = EditorSceneManager.OpenScene(
                $"Assets/Scenes/{name}.unity", OpenSceneMode.Single);

            int hosts = 0;
            foreach (var root in scene.GetRootGameObjects())
            {
                foreach (var component in root.GetComponents<Component>())
                {
                    if (component is FlowSceneHost) hosts++;
                }
            }

            Assert.AreEqual(1, hosts, $"{name} must host exactly one screen.");
        }
    }

    [Test]
    public void EveryFlowScene_HasExactlyOneEventSystem()
    {
        // The runtime logged "There can be only one active Event System" when a scene
        // carried a second one alongside a persistent one.
        foreach (var name in new[]
                 {
                     FlowSceneNames.Loading,
                     FlowSceneNames.Lobby,
                     FlowSceneNames.TrackSelection,
                     FlowSceneNames.WingSetup
                 })
        {
            var scene = EditorSceneManager.OpenScene(
                $"Assets/Scenes/{name}.unity", OpenSceneMode.Single);

            int systems = 0;
            foreach (var root in scene.GetRootGameObjects())
            {
                foreach (var component in root.GetComponents<Component>())
                {
                    if (component is UnityEngine.EventSystems.EventSystem) systems++;
                }
            }

            Assert.AreEqual(1, systems, $"{name} must have exactly one EventSystem.");
        }
    }

    private static GameObject HostScreenPrefab(FlowSceneHost host)
    {
        var so = new SerializedObject(host);
        var prop = so.FindProperty("_screenPrefab");
        Assert.IsNotNull(prop, "FlowSceneHost must have a _screenPrefab field.");
        return prop.objectReferenceValue as GameObject;
    }

    private static string ToAbsolute(string projectPath)
    {
        return Path.Combine(
            Application.dataPath.Replace("/Assets", "").Replace("\\Assets", ""),
            projectPath);
    }
}
