// Assets/Editor/SceneBuilder.cs - Builds LobbyScene and Track_01, wires LobbyManager fields,
// registers both scenes in Build Settings. Run via Unity menu: Tools > BuildScenes.
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem.UI;
using UnityEngine.SceneManagement;
using F1.GameFlow;

public class SceneBuilder
{
    private static readonly string[] ScreenPrefabNames =
    {
        "BrandingScreen_Prefab", "MainMenuScreen_Prefab", "CarSelectionScreen_Prefab",
        "TrackSelectionScreen_Prefab", "WingSetupScreen_Prefab", "ModeSelectionScreen_Prefab"
    };

    private static readonly string[] LobbyFieldNames =
    {
        "_brandingPrefab", "_mainMenuPrefab", "_carSelectionPrefab",
        "_trackSelectionPrefab", "_wingSetupPrefab", "_modeSelectionPrefab"
    };

    [MenuItem("Tools/BuildScenes")]
    public static void BuildAll()
    {
        BuildLobbyScene();
        BuildTrack01();
        RegisterBuildSettings();
        Debug.Log("[SceneBuilder] Done.");
    }

    private static void BuildLobbyScene()
    {
        const string path = "Assets/Scenes/LobbyScene.unity";
        if (System.IO.File.Exists(Path(path)))
        {
            Debug.Log("[SceneBuilder] LobbyScene exists — delete it to rebuild.");
            return;
        }

        Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        // GameFlowManager (DontDestroyOnLoad singleton, takes effect at runtime)
        var flow = new GameObject("GameFlowManager");
        flow.AddComponent<GameFlowManager>();

        // LobbyManager: screen root = own transform (screens carry their own Canvases)
        var lobby = new GameObject("LobbyManager");
        var lm = lobby.AddComponent<LobbyManager>();

        var so = new SerializedObject(lm);
        so.FindProperty("_screenRoot").objectReferenceValue = lobby.transform;

        for (int i = 0; i < ScreenPrefabNames.Length; i++)
        {
            string prefabPath = "Assets/Prefabs/" + ScreenPrefabNames[i] + ".prefab";
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
            if (prefab == null)
            {
                Debug.LogError("[SceneBuilder] Missing prefab (run BuildUIPrefabs first): " + prefabPath);
                continue;
            }
            SerializedProperty sp = so.FindProperty(LobbyFieldNames[i]);
            if (sp == null)
                Debug.LogError("[SceneBuilder] Missing LobbyManager field: " + LobbyFieldNames[i]);
            else
                sp.objectReferenceValue = prefab;
        }
        so.ApplyModifiedPropertiesWithoutUndo();

        // EventSystem for uGUI input (Input System package installed)
        var es = new GameObject("EventSystem");
        es.AddComponent<EventSystem>();
        es.AddComponent<InputSystemUIInputModule>();

        EditorSceneManager.SaveScene(scene, path);
        Debug.Log("[SceneBuilder] LobbyScene saved");
    }

    private static void BuildTrack01()
    {
        const string path = "Assets/Scenes/Track_01.unity";
        if (System.IO.File.Exists(Path(path)))
        {
            Debug.Log("[SceneBuilder] Track_01 exists — delete it to rebuild.");
            return;
        }

        Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        var tm = new GameObject("TrackModeManager");
        var comp = tm.AddComponent<TrackModeManager>();
        var so = new SerializedObject(comp);
        so.FindProperty("_trackId").stringValue = "track_01";
        so.ApplyModifiedPropertiesWithoutUndo();

        EditorSceneManager.SaveScene(scene, path);
        Debug.Log("[SceneBuilder] Track_01 saved");
    }

    private static void RegisterBuildSettings()
    {
        // LobbyScene first (entry point), Track_01 second; keep any existing extra scenes after.
        var scenes = new System.Collections.Generic.List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
        AddScene(scenes, "Assets/Scenes/LobbyScene.unity");
        AddScene(scenes, "Assets/Scenes/Track_01.unity");
        EditorBuildSettings.scenes = scenes.ToArray();
        Debug.Log("[SceneBuilder] Build Settings now has " + EditorBuildSettings.scenes.Length + " scenes");
    }

    private static void AddScene(System.Collections.Generic.List<EditorBuildSettingsScene> scenes, string path)
    {
        foreach (var s in scenes)
            if (s.path == path) return;
        if (!System.IO.File.Exists(Path(path))) return;
        // Insert LobbyScene at the front (entry point); append others.
        if (path.EndsWith("LobbyScene.unity"))
            scenes.Insert(0, new EditorBuildSettingsScene(path, true));
        else
            scenes.Add(new EditorBuildSettingsScene(path, true));
    }

    private static string Path(string projectPath)
    {
        return System.IO.Path.Combine(Application.dataPath.Replace("/Assets", ""), projectPath);
    }
}
