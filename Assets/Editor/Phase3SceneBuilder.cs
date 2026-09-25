// Assets/Editor/Phase3SceneBuilder.cs - Builds the Phase 3 flow scenes and the active
// route. Run: Tools > BuildFlowScenes.
//
// Replaces the single monolithic LobbyScene with:
//   00_LoadingScene      - entry scene: persistent flow root, branding, loading progress
//   10_CarSelectionScene - the lobby hub
//
// Each scene carries a FlowSceneHost declaring the one screen it owns, so a new hub
// destination is a new scene plus a host - no change to GameFlowManager.
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem.UI;
using UnityEngine.SceneManagement;
using UnityEngine.Rendering.Universal;
using F1.GameFlow;

public class Phase3SceneBuilder
{
    private const string ScenesFolder = "Assets/Scenes";
    private const string PrefabFolder = "Assets/Prefabs";

    private static string ScenePath(string sceneName) => $"{ScenesFolder}/{sceneName}.unity";

    private const string LoadingScenePath = ScenesFolder + "/" + FlowSceneNames.Loading + ".unity";
    private const string CarSelectionScenePath = ScenesFolder + "/" + FlowSceneNames.CarSelection + ".unity";
    private const string TrackSelectionScenePath = ScenesFolder + "/" + FlowSceneNames.TrackSelection + ".unity";
    private const string WingSetupScenePath = ScenesFolder + "/" + FlowSceneNames.WingSetup + ".unity";
    private const string PreRaceScenePath = ScenesFolder + "/" + FlowSceneNames.PreRace + ".unity";
    private const string RaceScenePath = ScenesFolder + "/" + FlowSceneNames.Race + ".unity";

    private const string PlayerCarPrefab = PrefabFolder + "/F1_Body.prefab";

    private static readonly string[] OrderedRoute =
    {
        FlowSceneNames.Loading,
        FlowSceneNames.CarSelection,
        FlowSceneNames.TrackSelection,
        FlowSceneNames.WingSetup,
        FlowSceneNames.PreRace,
        FlowSceneNames.Race,
        FlowSceneNames.TrackContent
    };

    [MenuItem("Tools/BuildFlowScenes")]
    public static void BuildAll()
    {
        BuildLoadingScene();
        BuildCarSelectionScene();
        BuildTrackSelectionScene();
        BuildWingSetupScene();
        BuildPreRaceScene();
        BuildRaceScene();
        PrepareTrackContentScene();
        RegisterBuildSettings();
        Debug.Log("[Phase3SceneBuilder] Flow scenes built and build settings registered.");
    }

    /// <summary>
    /// Strips the track scene back to pure content.
    ///
    /// The track scene is loaded additively underneath whichever session is running, so it
    /// must not claim a screen: the moment a shell owns a screen that the track also claims,
    /// resolution returns whichever host registered first and the losing side silently gets
    /// no HUD. That is not hypothetical — it is what defect P16 was, and S4 fixed it by
    /// parking the track's host on Race. S5 removes the host instead, because the race has
    /// its own shell now and the track has no screen left to host.
    ///
    /// <c>TrackModeManager</c> goes with it. It held the race half of the session — race HUD
    /// info, lap count, AI opponents — and spawned those opponents at the world origin
    /// because the race had nowhere else to put them. The race shell owns all of that now and
    /// places the field on the grid instead, which is the whole point of having a grid.
    ///
    /// Idempotent, and additive in the other direction: nothing is created here, so running
    /// it on a track scene that is already clean changes nothing.
    /// </summary>
    [MenuItem("Tools/PrepareTrackContentScene")]
    public static void PrepareTrackContentScene()
    {
        string path = ScenePath(FlowSceneNames.TrackContent);
        if (!System.IO.File.Exists(ToAbsolute(path)))
        {
            Debug.LogError($"[Phase3SceneBuilder] {path} does not exist.");
            return;
        }

        var scene = EditorSceneManager.OpenScene(path, OpenSceneMode.Single);
        bool changed = false;

        // Only these roots may be deleted below. A general "delete any root with nothing but
        // a transform" sweep looks harmless and is not: the track's own root is exactly that
        // shape, because its geometry hangs off it as children. Destroying it takes the
        // circuit with it — which is what happened the first time this ran, taking a scene
        // from 98 GameObjects to 1. A root becomes a candidate for deletion here and nowhere
        // else.
        var candidatesForRemoval = new List<GameObject>();

        foreach (var root in scene.GetRootGameObjects())
        {
            var host = root.GetComponent<FlowSceneHost>();
            if (host != null)
            {
                Debug.Log(
                    "[Phase3SceneBuilder] Removing the track scene's FlowSceneHost. The race " +
                    "shell hosts Race now; a second host claiming it would make screen " +
                    "resolution depend on load order (defect P16).");
                Object.DestroyImmediate(host);
                candidatesForRemoval.Add(root);
                changed = true;
            }

            var modeManager = root.GetComponent<TrackModeManager>();
            if (modeManager != null)
            {
                Debug.Log(
                    "[Phase3SceneBuilder] Removing TrackModeManager from the track scene. The " +
                    "race shell owns the race session; this component lived in the content " +
                    "scene and spawned AI at the world origin.");
                Object.DestroyImmediate(modeManager);
                candidatesForRemoval.Add(root);
                changed = true;
            }
        }

        // A root this method just emptied is tidier gone than left as a bare GameObject.
        foreach (var root in candidatesForRemoval)
        {
            if (root != null && root.GetComponents<Component>().Length == 1)
            {
                Debug.Log($"[Phase3SceneBuilder] Removing '{root.name}', now empty in the track scene.");
                Object.DestroyImmediate(root);
            }
        }

        if (changed)
            Save(scene, path);
        else
            Debug.Log("[Phase3SceneBuilder] Track scene is already pure content.");
    }

    /// <summary>
    /// Kept as a menu item for muscle memory, and pointed at the method that does the work.
    /// </summary>
    [MenuItem("Tools/EnsureTrackSceneHost", true)]
    private static bool EnsureTrackSceneHostValidate() => false;

    // --- 00_LoadingScene ---

    private static void BuildLoadingScene()
    {
        var scene = CreateScene(LoadingScenePath, "00_LoadingScene");

        // The persistent flow root lives here, because this is the entry scene.
        var flowGo = EnsureRoot(scene, "GameFlowManager");
        var flow = EnsureComponent<GameFlowManager>(flowGo);
        EnsureComponent<FlowBootstrap>(flowGo);
        EnsureComponent<FlowRoot>(flowGo);

        // The scene hosts the branding screen. Branding is hosted by the loading scene
        // itself, so the serialized scene name points back at this scene; the routing
        // layer's self-reference guard makes that a no-op rather than a reload.
        var hostGo = EnsureRoot(scene, "FlowSceneHost");
        var host = EnsureComponent<FlowSceneHost>(hostGo);
        host.Configure(
            GameFlowManager.GameScreen.Branding,
            LoadPrefab("BrandingScreen_Prefab"),
            HostScreenMode.Fixed,
            hostGo.transform);

        // Scene controller owns the behaviour; the screen stays a pure view.
        EnsureComponent<LoadingSceneController>(hostGo);

        EnsureEventSystem(scene);

        WireFlowSceneNames(flow);
        Save(scene, LoadingScenePath);
    }

    // --- 10_CarSelectionScene ---

    private static void BuildCarSelectionScene()
    {
        var scene = CreateScene(CarSelectionScenePath, "10_CarSelectionScene");

        var hostGo = EnsureRoot(scene, "FlowSceneHost");
        var host = EnsureComponent<FlowSceneHost>(hostGo);
        host.Configure(
            GameFlowManager.GameScreen.CarSelection,
            LoadPrefab("CarSelectionScreen_Prefab"),
            HostScreenMode.Fixed,
            hostGo.transform);

        // The hop to 20_TrackSelectionScene is derived from the scene list, so the hub
        // advances on its own once that scene exists (slice step S1).
        EnsureComponent<CarSelectionSceneController>(hostGo);

        EnsureEventSystem(scene);
        Save(scene, CarSelectionScenePath);
    }

    // --- 20_TrackSelectionScene (slice step S1) ---

    private static void BuildTrackSelectionScene()
    {
        var scene = CreateScene(TrackSelectionScenePath, FlowSceneNames.TrackSelection);

        var hostGo = EnsureRoot(scene, "FlowSceneHost");
        var host = EnsureComponent<FlowSceneHost>(hostGo);
        host.Configure(
            GameFlowManager.GameScreen.TrackSelection,
            LoadPrefab("TrackSelectionScreen_Prefab"),
            HostScreenMode.Fixed,
            hostGo.transform);

        EnsureComponent<TrackSelectionSceneController>(hostGo);

        EnsureEventSystem(scene);
        Save(scene, TrackSelectionScenePath);
    }

    // --- 30_WingSetupScene (slice step S1) ---

    private static void BuildWingSetupScene()
    {
        var scene = CreateScene(WingSetupScenePath, FlowSceneNames.WingSetup);

        var hostGo = EnsureRoot(scene, "FlowSceneHost");
        var host = EnsureComponent<FlowSceneHost>(hostGo);
        host.Configure(
            GameFlowManager.GameScreen.WingSetup,
            LoadPrefab("WingSetupScreen_Prefab"),
            HostScreenMode.Fixed,
            hostGo.transform);

        EnsureComponent<WingSetupSceneController>(hostGo);

        EnsureEventSystem(scene);
        Save(scene, WingSetupScenePath);
    }

    // --- 40_PreRaceScene (slice step S4) ---

    /// <summary>
    /// The qualifying shell. A shell, not a copy of the track: it hosts the qualifying HUD
    /// and spawns the player's car, while the track content stays a separate additive scene
    /// that pre-race and race both share (invariant 9).
    ///
    /// It hosts the qualifying screen on a Fixed mode, unlike the track scene's
    /// QualifyingOrRace. The track scene is content and is no longer responsible for the
    /// qualifying HUD at all — that responsibility moved here with the rest of the session's
    /// behaviour, so the host has exactly one screen to declare.
    /// </summary>
    private static void BuildPreRaceScene()
    {
        var scene = CreateScene(PreRaceScenePath, FlowSceneNames.PreRace);

        var hostGo = EnsureRoot(scene, "FlowSceneHost");
        var host = EnsureComponent<FlowSceneHost>(hostGo);
        host.Configure(
            GameFlowManager.GameScreen.Qualifying,
            LoadPrefab("QualifyingScreen_Prefab"),
            HostScreenMode.Fixed,
            hostGo.transform);

        EnsureComponent<PreRaceSceneController>(hostGo);

        // The spawner is a child of the host root so the controller can find it with
        // GetComponentInChildren, matching how it finds the screen the host instantiated.
        var spawner = EnsureChild<PlayerCarSpawner>(hostGo.transform, "PlayerCarSpawner");
        var spawnerSo = new SerializedObject(spawner);
        SetObject(spawnerSo, "_playerCarPrefab", LoadPrefab("F1_Body"));
        spawnerSo.ApplyModifiedPropertiesWithoutUndo();

        // No camera rig here on purpose. The camera lives on the car prefab, so spawning a
        // car brings the camera and its behaviour with it — one setup, wherever a car is
        // placed, and the car remains a self-contained asset that can be dropped into any
        // scene. A scene-owned rig would mean every scene that spawns a car also has to
        // remember to build and aim a camera.
        RemoveChaseCameraRig(scene);

        EnsureEventSystem(scene);
        Save(scene, PreRaceScenePath);
    }

    /// <summary>
    /// Removes the scene-owned camera rig left behind by an earlier approach. The car
    /// prefab carries the camera instead, so the scene must not carry a second one: two
    /// cameras means two brains competing for the same frame.
    /// </summary>
    private static void RemoveChaseCameraRig(Scene scene)
    {
        var existing = GameObject.Find("ChaseCameraRig");
        if (existing != null)
        {
            Object.DestroyImmediate(existing);
            Debug.Log("[Phase3SceneBuilder] Removed the scene-owned camera rig; the car prefab carries the camera.");
        }
    }

    // --- 50_RaceScene (slice step S5) ---

    /// <summary>
    /// The race shell, built the same way the pre-race shell is: a scene that owns a screen,
    /// spawns the player's car, and leaves the track to the track.
    ///
    /// The car is spawned at the canonical start pose and then moved onto its grid slot by
    /// the track's own <c>RaceGridManager</c> — not placed on the grid from here. The grid
    /// anchor and the slot layout are track geometry, and this scene has none.
    ///
    /// No screen prefab yet: the race HUD is the next step. The host carries no prefab, which
    /// the flow reports as a warning rather than an error, so the race can be brought up and
    /// driven before the HUD exists.
    /// </summary>
    private static void BuildRaceScene()
    {
        var scene = CreateScene(RaceScenePath, FlowSceneNames.Race);

        var hostGo = EnsureRoot(scene, "FlowSceneHost");
        var host = EnsureComponent<FlowSceneHost>(hostGo);
        host.Configure(
            GameFlowManager.GameScreen.Race,
            null,
            HostScreenMode.Fixed,
            hostGo.transform);

        EnsureComponent<RaceSceneController>(hostGo);

        var spawner = EnsureChild<PlayerCarSpawner>(hostGo.transform, "PlayerCarSpawner");
        var spawnerSo = new SerializedObject(spawner);
        SetObject(spawnerSo, "_playerCarPrefab", LoadPrefab("F1_Body"));
        spawnerSo.ApplyModifiedPropertiesWithoutUndo();

        // Same reason as the pre-race shell: the camera is the car's own, so spawning a car
        // brings the camera with it and this scene has no camera rig to keep in step.
        RemoveChaseCameraRig(scene);

        EnsureEventSystem(scene);
        Save(scene, RaceScenePath);
    }

    // --- Helpers ---

    private static Scene CreateScene(string path, string rootNameHint)
    {
        if (System.IO.File.Exists(ToAbsolute(path)))
        {
            Debug.Log($"[Phase3SceneBuilder] {rootNameHint} already exists — opening it for rebuild.");
            return EditorSceneManager.OpenScene(path, OpenSceneMode.Single);
        }

        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        Debug.Log($"[Phase3SceneBuilder] Creating {path}");
        return scene;
    }

    /// <summary>
    /// Returns the scene's single root with this name, creating it if absent and
    /// removing extras left by an earlier non-idempotent run. Without the dedupe,
    /// re-running this builder silently produced a second FlowSceneHost and a second
    /// EventSystem in every scene it had already built — which is exactly what happened
    /// when the Phase 3 scenes were rebuilt for slice step S1.
    /// </summary>
    private static GameObject EnsureRoot(Scene scene, string name)
    {
        GameObject keep = null;
        var duplicates = new List<GameObject>();

        foreach (var root in scene.GetRootGameObjects())
        {
            if (root.name != name) continue;
            if (keep == null) keep = root;
            else duplicates.Add(root);
        }

        foreach (var duplicate in duplicates)
        {
            Debug.LogWarning(
                $"[Phase3SceneBuilder] Removing duplicate root '{name}' in {scene.name}.");
            Object.DestroyImmediate(duplicate);
        }

        return keep != null ? keep : new GameObject(name);
    }

    private static T EnsureComponent<T>(GameObject go) where T : Component
    {
        var existing = go.GetComponent<T>();
        return existing != null ? existing : go.AddComponent<T>();
    }

    /// <summary>Reuses the named component or adds it, for types reached by reflection.</summary>
    private static Component EnsureComponent(System.Type type, GameObject go)
    {
        var existing = go.GetComponent(type);
        return existing != null ? existing : go.AddComponent(type);
    }

    /// <summary>
    /// Returns the named child, adding it if absent, and guarantees the component is on it.
    /// Idempotent for the same reason <see cref="EnsureRoot"/> is: this builder is re-run
    /// whenever the route changes, and a second spawner would mean two cars on the grid.
    /// </summary>
    private static T EnsureChild<T>(Transform parent, string name) where T : Component
    {
        var existing = parent.Find(name);
        if (existing != null)
            return EnsureComponent<T>(existing.gameObject);

        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        return go.AddComponent<T>();
    }

    /// <summary>
    /// Reuses this builder's EventSystem if the scene already has one. A second one
    /// produces "There can be only one active Event System" at runtime, so this must be
    /// idempotent rather than additive.
    /// </summary>
    private static void EnsureEventSystem(Scene scene)
    {
        var es = EnsureRoot(scene, "EventSystem");
        EnsureComponent<EventSystem>(es);
        EnsureComponent<InputSystemUIInputModule>(es);
    }

    private static void WireFlowSceneNames(GameFlowManager flow)
    {
        var so = new SerializedObject(flow);
        Set(so, "_brandingScene", FlowSceneNames.Loading);
        Set(so, "_carSelectionScene", FlowSceneNames.CarSelection);
        Set(so, "_trackSelectionScene", FlowSceneNames.TrackSelection);
        Set(so, "_wingSetupScene", FlowSceneNames.WingSetup);
        Set(so, "_resultsScene", FlowSceneNames.Results);
        // Qualifying gets its own shell (S4) and the race gets one too (S5). The track scene
        // is shared *content* that both shells load additively, so both resolve the same live
        // track objects (invariant 9).
        Set(so, "_qualifyingScene", FlowSceneNames.PreRace);
        Set(so, "_raceScene", FlowSceneNames.Race);
        // No main menu in the active route (invariant 1).
        Set(so, "_mainMenuScene", "");
        so.ApplyModifiedPropertiesWithoutUndo();
    }

    private static void Set(SerializedObject so, string property, string value)
    {
        var sp = so.FindProperty(property);
        if (sp == null)
        {
            Debug.LogError($"[Phase3SceneBuilder] GameFlowManager has no field '{property}'.");
            return;
        }

        sp.stringValue = value;
    }

    /// <summary>Assigns an object-reference serialized field, e.g. a prefab.</summary>
    private static void SetObject(SerializedObject so, string property, UnityEngine.Object value)
    {
        var sp = so.FindProperty(property);
        if (sp == null)
        {
            Debug.LogError($"[Phase3SceneBuilder] {so.targetObject.GetType().Name} has no field '{property}'.");
            return;
        }

        sp.objectReferenceValue = value;
    }

    private static GameObject LoadPrefab(string prefabName)
    {
        string path = $"{PrefabFolder}/{prefabName}.prefab";
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);
        if (prefab == null)
            Debug.LogError(
                $"[Phase3SceneBuilder] Missing screen prefab '{path}'. Run Tools > BuildUIPrefabs first.");
        return prefab;
    }

    private static void Save(Scene scene, string path)
    {
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene, path);
        Debug.Log($"[Phase3SceneBuilder] Saved {path}");
    }

    // --- Build settings ---

    private static void RegisterBuildSettings()
    {
        var ordered = new System.Collections.Generic.List<EditorBuildSettingsScene>();
        foreach (var name in OrderedRoute)
        {
            string path = ScenePath(name);

            if (!System.IO.File.Exists(ToAbsolute(path)))
            {
                Debug.LogError($"[Phase3SceneBuilder] Cannot register '{path}': file does not exist.");
                continue;
            }

            ordered.Add(new EditorBuildSettingsScene(path, true));
        }

        EditorBuildSettings.scenes = ordered.ToArray();
        AssetDatabase.SaveAssets();

        var report = new System.Text.StringBuilder("[Phase3SceneBuilder] Build settings:\n");
        var scenes = EditorBuildSettings.scenes;
        for (int i = 0; i < scenes.Length; i++)
            report.AppendLine($"  [{i}] {scenes[i].path} enabled={scenes[i].enabled}");
        Debug.Log(report.ToString());
    }

    private static string ToAbsolute(string projectPath)
    {
        return System.IO.Path.Combine(
            Application.dataPath.Replace("/Assets", "").Replace("\\Assets", ""),
            projectPath);
    }
}
