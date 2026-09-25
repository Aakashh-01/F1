using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using F1.GameFlow;

/// <summary>
/// Slice step S4: the pre-race qualifying shell exists, is wired, and is unambiguous.
///
/// These inspect the scene as an asset. The shell's *behaviour* — loading the track,
/// spawning the car, driving a lap — needs a running player loop and lives in
/// <c>SliceS4QualifyingShellTests</c> (PlayMode). Splitting them this way is deliberate:
/// scene-content inspection needs <c>EditorSceneManager</c>, which is forbidden during play
/// mode, and the same mistake in S2's tests cost a full red run before it was corrected.
/// </summary>
public class SliceS4PreRaceSceneTests
{
    private const string PreRacePath = "Assets/Scenes/" + FlowSceneNames.PreRace + ".unity";

    private static Scene OpenPreRace()
    {
        Assert.IsTrue(File.Exists(ToAbsolute(PreRacePath)), $"{PreRacePath} does not exist.");
        return EditorSceneManager.OpenScene(PreRacePath, OpenSceneMode.Single);
    }

    private static string ToAbsolute(string projectPath) =>
        Path.Combine(Application.dataPath.Replace("/Assets", "").Replace("\\Assets", ""), projectPath);

    private static T Find<T>(Scene scene) where T : Component =>
        scene.GetRootGameObjects()
            .SelectMany(go => go.GetComponentsInChildren<T>(true))
            .FirstOrDefault();

    // --- The scene and its place in the route ---

    [Test]
    public void PreRaceScene_ExistsAndIsInTheBuildList()
    {
        Assert.IsTrue(File.Exists(ToAbsolute(PreRacePath)), $"{PreRacePath} does not exist.");

        var entry = EditorBuildSettings.scenes.FirstOrDefault(
            s => s.path == PreRacePath && s.enabled);
        Assert.IsNotNull(entry,
            "The pre-race shell must be an enabled build-settings entry, or a player build " +
            "cannot route into qualifying.");
    }

    [Test]
    public void BuildList_PutsPreRaceAfterWingSetupAndBeforeTrackContent()
    {
        var names = EditorBuildSettings.scenes.Select(s => Path.GetFileNameWithoutExtension(s.path)).ToList();

        int preRace = names.IndexOf(FlowSceneNames.PreRace);
        int wingSetup = names.IndexOf(FlowSceneNames.WingSetup);
        int trackContent = names.IndexOf(FlowSceneNames.TrackContent);

        Assert.Greater(preRace, -1, "PreRace is missing from the build list.");
        Assert.Greater(wingSetup, -1, "WingSetup is missing from the build list.");
        Assert.Greater(trackContent, -1, "The track content scene is missing from the build list.");

        // Invariant 4: wing selection happens before qualifying. The build order mirrors the
        // route so a player stepping through scenes in order meets them in order.
        Assert.Less(wingSetup, preRace, "Wing setup must precede the pre-race shell.");
        Assert.Less(preRace, trackContent, "Track content comes after the flow shells.");
    }

    [Test]
    public void Flow_RoutesQualifyingToThePreRaceShell()
    {
        var scene = EditorSceneManager.OpenScene(
            "Assets/Scenes/" + FlowSceneNames.Loading + ".unity", OpenSceneMode.Single);
        var flow = Find<GameFlowManager>(scene);
        Assert.IsNotNull(flow, "The loading scene must carry the persistent flow root.");

        var so = new SerializedObject(flow);
        Assert.AreEqual(FlowSceneNames.PreRace, so.FindProperty("_qualifyingScene").stringValue,
            "Qualifying must route to its own shell, not to the shared track content scene.");
    }

    // --- The shell's wiring ---

    [Test]
    public void PreRaceScene_HostsQualifyingOnAFixedHostWithAScreenPrefab()
    {
        var scene = OpenPreRace();
        var host = Find<FlowSceneHost>(scene);

        Assert.IsNotNull(host, "The pre-race scene must carry a FlowSceneHost.");
        Assert.AreEqual(GameFlowManager.GameScreen.Qualifying, host.HostedScreen);

        // The serialized prefab is the asset-level fact. The *instantiated* screen is a
        // runtime fact — the host builds it in Awake, which does not run in Edit mode — and
        // is asserted by the PlayMode test instead.
        var so = new SerializedObject(host);
        var prefab = so.FindProperty("_screenPrefab").objectReferenceValue as GameObject;
        Assert.IsNotNull(prefab,
            "The host has no screen prefab assigned. Without one there is no HUD, and the " +
            "lap clock S3 measures has nowhere to go.");
        Assert.AreEqual(0, so.FindProperty("_screenMode").enumValueIndex,
            "The pre-race host must be Fixed: it owns the qualifying screen outright.");
    }

    [Test]
    public void PreRaceScene_CarriesItsControllerAndSpawner()
    {
        var scene = OpenPreRace();

        Assert.IsNotNull(Find<PreRaceSceneController>(scene),
            "The pre-race scene must carry its scene controller.");
        Assert.IsNotNull(Find<PlayerCarSpawner>(scene),
            "The pre-race scene must carry the car spawner, or no car reaches the track.");

        // The controller resolves the spawner with GetComponentInChildren, so a spawner
        // parked on a separate root would never be found and the session would fail at
        // runtime with "no car can be placed on the track".
        var host = Find<FlowSceneHost>(scene);
        var spawner = Find<PlayerCarSpawner>(scene);
        Assert.IsTrue(
            spawner.transform.IsChildOf(host.transform) || spawner.transform == host.transform,
            "The spawner must live under the host root so the controller can find it.");
    }

    [Test]
    public void Spawner_HasACarPrefabThatCanActuallyBeConfigured()
    {
        var scene = OpenPreRace();
        var spawner = Find<PlayerCarSpawner>(scene);
        var so = new SerializedObject(spawner);
        var prefab = so.FindProperty("_playerCarPrefab").objectReferenceValue as GameObject;

        Assert.IsNotNull(prefab,
            "No car prefab is assigned. The spawner would refuse to spawn and qualifying " +
            "could never start.");

        // The spawner applies the selected car's wing profile through the coordinator, and
        // the position through the rigidbody. A prefab missing either would spawn a car
        // that ignores the wing choice, or one the spawner cannot place and settle.
        Assert.IsNotNull(prefab.GetComponent<VehiclePhysicsCoordinator>(),
            "The car prefab needs a VehiclePhysicsCoordinator to receive the wing profile.");
        Assert.IsNotNull(prefab.GetComponent<Rigidbody>(),
            "The car prefab needs a Rigidbody so the spawner can place it and zero its motion.");
    }

    [Test]
    public void PreRaceScene_HasNoMissingScripts()
    {
        // Guards the class of defect recorded as P6 in Phase 3, where a component that
        // shared a file with another class serialized as a bare missing-script slot and
        // nothing complained.
        var scene = OpenPreRace();
        foreach (var root in scene.GetRootGameObjects())
        {
            foreach (var component in root.GetComponents<Component>())
            {
                Assert.IsNotNull(component,
                    $"'{root.name}' has a missing script slot. A null component here means " +
                    "a serialized reference that silently does nothing.");
            }
        }
    }

    // --- The HUD prefab ---

    [Test]
    public void QualifyingHud_ExistsAndWiresTheLapClock()
    {
        const string path = "Assets/Prefabs/QualifyingScreen_Prefab.prefab";
        Assert.IsTrue(File.Exists(ToAbsolute(path)), $"{path} does not exist. Run Tools > BuildUIPrefabs.");

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);
        Assert.IsNotNull(prefab);

        var screen = prefab.GetComponentInChildren<QualifyingScreen>(true);
        Assert.IsNotNull(screen, "The HUD must carry a QualifyingScreen implementation.");

        var so = new SerializedObject(screen);
        foreach (var field in new[] { "_trackNameText", "_currentLapTimeText", "_bestLapText",
                                      "_raceButton", "_restartButton", "_backButton" })
        {
            var property = so.FindProperty(field);
            Assert.IsNotNull(property, $"QualifyingScreenImpl has no field '{field}'.");
            Assert.IsNotNull(property.objectReferenceValue,
                $"'{field}' is not wired. A HUD that renders but cannot display the lap clock " +
                "is exactly the gap S3 was measured against.");
        }
    }

    // --- The host ambiguity this step introduced and resolved ---

    [Test]
    public void TrackScene_HostsNoScreenAtAllSoItCannotShadowAShell()
    {
        // The track scene's host used to derive its screen from the session type, which made
        // it claim Qualifying during a qualifying session — the same screen the pre-race
        // shell's host claims. Resolution returned whichever registered first, so losing
        // meant no HUD at all, intermittently. Parking it on Race fixed that specific
        // collision; S5 then gave the race its own shell, at which point the track had no
        // screen left to host and the host went entirely.
        //
        // The invariant is unchanged and is the reason for the test: a scene loaded
        // additively underneath a session must never claim a screen that a shell claims. The
        // assertion moved from "hosts Race" to "hosts nothing" because that is now the only
        // safe answer, and a test left asserting the old arrangement would be asserting a bug
        // back into existence.
        var scene = EditorSceneManager.OpenScene(
            "Assets/Scenes/" + FlowSceneNames.TrackContent + ".unity", OpenSceneMode.Single);
        var host = Find<FlowSceneHost>(scene);
        Assert.IsNull(host,
            "The track scene is content loaded additively under whichever session is " +
            "running. A host here can only compete with a shell for screen resolution, and " +
            "which one wins depends on load order.");
    }
}
