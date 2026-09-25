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

    [Test]
    public void CarSelectionScene_HostsOnlyItsOwnScreen()
    {
        var scene = EditorSceneManager.OpenScene(
            $"Assets/Scenes/{FlowSceneNames.CarSelection}.unity", OpenSceneMode.Single);

        var host = Object.FindAnyObjectByType<FlowSceneHost>();
        Assert.IsNotNull(host, "The car selection scene must carry a FlowSceneHost.");
        Assert.AreEqual(GameFlowManager.GameScreen.CarSelection, host.HostedScreen);

        var prefab = HostScreenPrefab(host);
        Assert.IsNotNull(prefab, "The car selection host must have a screen prefab.");
        // The concrete Impl lives in Assembly-CSharp, which an asmdef cannot reference,
        // so check against the abstract contract instead.
        Assert.IsNotNull(prefab.GetComponent<CarSelectionScreen>(),
            "The hosted prefab must actually be a car selection screen.");
    }

    [Test]
    public void CarSelectionScene_DoesNotAlsoHostBranding()
    {
        // One screen per scene. If a scene hosts two, one of them is unreachable.
        var scene = EditorSceneManager.OpenScene(
            $"Assets/Scenes/{FlowSceneNames.CarSelection}.unity", OpenSceneMode.Single);

        var hosts = scene.GetRootGameObjects();
        int count = 0;
        foreach (var root in hosts)
            if (root.GetComponent<FlowSceneHost>() != null) count++;

        Assert.AreEqual(1, count, "A flow scene hosts exactly one screen.");
    }

    [Test]
    public void TrackScene_HostIsFixedToRace()
    {
        var scene = EditorSceneManager.OpenScene(
            $"Assets/Scenes/{FlowSceneNames.TrackContent}.unity", OpenSceneMode.Single);

        var host = Object.FindAnyObjectByType<FlowSceneHost>();
        Assert.IsNotNull(host, "The shared track scene must carry a FlowSceneHost.");

        // This used to be HostScreenMode.QualifyingOrRace, asserting that the track scene
        // derived its screen from the session type and therefore hosted Qualifying while
        // qualifying. Slice step S4 gave qualifying its own shell, and with two hosts in play
        // that mode became ambiguous: the track host and the pre-race host both claimed
        // Qualifying, and which one the flow found depended on load order. The track scene is
        // content now and hosts Race only, until S5 gives the race a shell of its own.
        // SliceS4PreRaceSceneTests.TrackScene_HostsRaceOnlySoItCannotShadowTheQualifyingHost
        // asserts the serialized mode as well as the resolved screen.
        Assert.AreEqual(GameFlowManager.GameScreen.Race, host.HostedScreen);
    }

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
                     FlowSceneNames.CarSelection,
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
                     FlowSceneNames.CarSelection,
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
