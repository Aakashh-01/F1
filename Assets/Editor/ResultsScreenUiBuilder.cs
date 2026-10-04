// Assets/Editor/ResultsScreenUiBuilder.cs - builds the post-race results prefab and the
// 60_ResultsScene that hosts it.
//
// Run: Tools > Results/Build Results Screen, Tools > Results/Create Results Scene, or
// Tools > Results/Build All.
//
// Deliberately a SEPARATE, narrow builder, for the same reason RaceScreenUiBuilder and
// WingScreenUiBuilder are: UIBuilder.BuildAll (Tools > BuildUIPrefabs) rebuilds all nine
// prefabs from scratch and would throw away the hand-finished track-selection hero layout.
// Nothing here touches any other prefab or any other scene.
//
// THE RESULTS SCENE IS GENERATED. Re-running Create Results Scene deletes and rebuilds
// 60_ResultsScene.unity wholesale. That is safe while the scene is generated, and it is NOT
// safe once it has been hand-placed — the same warning the lobby scene carries, for the same
// reason. Do not tune this scene by hand and then re-run this builder.
using TMPro;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem.UI;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using F1.GameFlow;
using F1.UI;

public static class ResultsScreenUiBuilder
{
    private const string PrefabName = "ResultsScreen_Prefab";
    private const string PrefabPath = "Assets/Prefabs/" + PrefabName + ".prefab";
    private const string ScenePath = "Assets/Scenes/60_ResultsScene.unity";

    private static readonly Color Bright = new Color(0.96f, 0.98f, 1f, 1f);
    private static readonly Color Dim = new Color(0.72f, 0.84f, 0.94f, 0.96f);
    private static readonly Color Accent = SlimUiSkin.Accent;

    [MenuItem("Tools/Results/Build Results Screen")]
    public static void BuildResultsScreen()
    {
        // Opaque, deliberately. The track is still loaded behind this scene and the player's
        // car has just been locked mid-track, so a translucent panel would frame a frozen lap
        // rather than report a result. This is a menu screen, not a HUD.
        GameObject go = UIBuilder.ScreenRoot(PrefabName, opaqueBackground: true);
        var impl = go.AddComponent<ResultsScreenImpl>();

        // --- Title. ---
        var title = UIBuilder.AddText(go.transform, "TitleText",
            new Vector2(0.5f, 1f), new Vector2(900f, 60f),
            "RACE COMPLETE", 40, Bright, FontStyles.Bold);
        Nudge(title, new Vector2(0f, -70f));

        // --- The finishing position. The single number this screen exists to show, so it is
        // the largest thing on it, with the word that introduces it as a static caption
        // above rather than baked into the string. ---
        var position = UIBuilder.AddText(go.transform, "FinishPositionText",
            new Vector2(0.5f, 0.5f), new Vector2(760f, 200f),
            "P1", 150, Accent, FontStyles.Bold);
        Nudge(position, new Vector2(0f, 60f));

        var finishedCaption = UIBuilder.AddText(go.transform, "FinishedCaptionText",
            new Vector2(0.5f, 0.5f), new Vector2(600f, 34f),
            "FINISHED", 24, Dim, FontStyles.Bold);
        Nudge(finishedCaption, new Vector2(0f, 185f));

        // --- Points. ---
        var points = UIBuilder.AddText(go.transform, "PointsEarnedText",
            new Vector2(0.5f, 0.5f), new Vector2(760f, 56f),
            "+0 pts", 44, Bright, FontStyles.Bold);
        Nudge(points, new Vector2(0f, -60f));

        var total = UIBuilder.AddText(go.transform, "NewTotalPointsText",
            new Vector2(0.5f, 0.5f), new Vector2(760f, 34f),
            "Total: 0 pts", 22, Dim);
        Nudge(total, new Vector2(0f, -115f));

        // --- Unlock panel. Inactive at build time so the prefab opens in its common state,
        // and only shown when finishing the race actually put something within reach. ---
        GameObject unlockPanel = UIBuilder.NewUI("UnlockPanel");
        unlockPanel.transform.SetParent(go.transform, false);
        UIBuilder.Anchor(unlockPanel, new Vector2(0.5f, 0.5f), new Vector2(900f, 180f));
        // Sits BELOW the points block, and must stay there. uGUI draws siblings in hierarchy
        // order, and this panel is created after the position text — so centred, it painted
        // straight over the finishing position. That is invisible to every assertion and
        // obvious in a screenshot.
        Nudge(unlockPanel.transform, new Vector2(0f, -250f));
        unlockPanel.AddComponent<Image>();

        var unlockTitle = UIBuilder.AddText(unlockPanel.transform, "UnlockTitleText",
            new Vector2(0.5f, 1f), new Vector2(860f, 32f),
            "AVAILABLE TO UNLOCK", 20, Color.white, FontStyles.Bold);
        Nudge(unlockTitle, new Vector2(0f, -28f));

        var carUnlock = UIBuilder.AddText(unlockPanel.transform, "CarUnlockText",
            new Vector2(0.5f, 0.5f), new Vector2(860f, 34f), "", 22, Color.white);
        Nudge(carUnlock, new Vector2(0f, 14f));

        var trackUnlock = UIBuilder.AddText(unlockPanel.transform, "TrackUnlockText",
            new Vector2(0.5f, 0.5f), new Vector2(860f, 34f), "", 22, Color.white);
        Nudge(trackUnlock, new Vector2(0f, -24f));

        unlockPanel.SetActive(false);

        // --- Buttons. Continue right and primary, because it is the route the player takes
        // when they are done; retry left and neutral, because it is the one they take when
        // they are not. ---
        var retry = UIBuilder.AddButton(go.transform, "RetryButton",
            new Vector2(0f, 0f), new Vector2(240f, 72f), "RETRY", null, null);
        Nudge(retry, new Vector2(130f, 90f));

        var continueButton = UIBuilder.AddButton(go.transform, "ContinueButton",
            new Vector2(1f, 0f), new Vector2(240f, 72f), "CONTINUE", null, null);
        Nudge(continueButton, new Vector2(-130f, 90f));

        // No Main Menu button. There is no main menu: GameFlowManager._mainMenuScene is empty
        // and the lobby is the hub. A second button that did the same thing as Continue, or
        // that routed nowhere, would be worse than no button.

        // --- Skin ---
        //
        // The palette here is chosen by WHERE the text sits, not by what the SlimUI pack
        // supplies. SlimUI is a LIGHT theme: its Ink is near-black and is meant for a label on
        // a pale button or card. This screen's body sits on the dark screen-root panel, so
        // near-black ink there is invisible — it rendered that way on the first build, with
        // every EditMode assertion passing, and only the screenshot caught it.
        //
        // So: light ink for the body, dark ink for the button labels, which DO sit on pale
        // fills. Stock font material throughout — nothing here overlays live 3D, so none of it
        // needs the HUD shadow the race screen uses.
        SlimUiSkin.ApplyFlatPanel(unlockPanel.GetComponent<Image>(), SlimUiSkin.CardFill);

        title.color = SlimUiSkin.TileInk;
        finishedCaption.color = SlimUiSkin.TileInkMuted;
        position.color = Accent;
        points.color = SlimUiSkin.TileInk;
        total.color = SlimUiSkin.TileInkMuted;
        unlockTitle.color = SlimUiSkin.TileInk;
        carUnlock.color = SlimUiSkin.TileInk;
        trackUnlock.color = SlimUiSkin.TileInk;

        SlimUiSkin.ApplySecondaryButton(retry);
        SlimUiSkin.ApplyPrimaryButton(continueButton);
        ApplyButtonInk(retry);
        ApplyButtonInk(continueButton);

        UIBuilder.SetRefs(impl,
            ("_finishPositionText", position),
            ("_pointsEarnedText", points),
            ("_newTotalPointsText", total),
            ("_unlockPanel", unlockPanel),
            ("_unlockTitleText", unlockTitle),
            ("_carUnlockText", carUnlock),
            ("_trackUnlockText", trackUnlock),
            ("_continueButton", continueButton),
            ("_retryButton", retry));

        GameObject asset = UIBuilder.Save(PrefabName, go);
        if (asset == null)
        {
            Debug.LogError($"[ResultsScreenUiBuilder] Failed to build {PrefabPath}.");
            return;
        }

        // Wired AFTER the save, deliberately. UnityEventTools only writes a durable listener
        // when its target is already a prefab asset, so a listener added during construction
        // points at an object that no longer exists by the time the prefab is written, and the
        // button ships inert. Re-opening the prefab contents makes the impl a real asset, and
        // the listeners survive every later rebuild of this prefab.
        WireButtonListeners();

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log("[ResultsScreenUiBuilder] Built " + PrefabPath);
    }

    /// <summary>
    /// Adds the persistent onClick listeners to the saved prefab's two buttons.
    /// </summary>
    private static void WireButtonListeners()
    {
        var root = PrefabUtility.LoadPrefabContents(PrefabPath);
        try
        {
            var impl = root.GetComponent<ResultsScreenImpl>();
            if (impl == null)
            {
                Debug.LogError(
                    $"[ResultsScreenUiBuilder] {PrefabPath} has no ResultsScreenImpl to wire to.");
                return;
            }

            Wire(root.transform, "ContinueButton", impl.OnContinueClicked);
            Wire(root.transform, "RetryButton", impl.OnRetryClicked);

            PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    /// <summary>
    /// Writes one durable onClick listener onto a button in the open prefab.
    /// </summary>
    /// <remarks>
    /// This uses <c>UnityEventTools.AddVoidPersistentListener</c> and NOT
    /// <c>button.onClick.AddListener</c>, and the difference is the whole reason this method
    /// exists. AddListener registers a RUNTIME listener: it lives on this throwaway edit-mode
    /// copy of the prefab and evaporates when the contents are unloaded, so the button ships
    /// inert while looking entirely correct. Only the UnityEventTools call serialises a
    /// listener into the asset.
    ///
    /// Existing listeners are cleared first so that re-running the builder cannot stack a
    /// second copy of the same handler onto the button.
    /// </remarks>
    private static void Wire(Transform root, string buttonName, UnityAction onClick)
    {
        var buttonTransform = root.Find(buttonName);
        if (buttonTransform == null)
        {
            Debug.LogError($"[ResultsScreenUiBuilder] {buttonName} is missing from {PrefabPath}.");
            return;
        }

        var button = buttonTransform.GetComponent<Button>();
        if (button == null)
        {
            Debug.LogError($"[ResultsScreenUiBuilder] {buttonName} has no Button component.");
            return;
        }

        while (button.onClick.GetPersistentEventCount() > 0)
            UnityEditor.Events.UnityEventTools.RemovePersistentListener(button.onClick, 0);

        UnityEditor.Events.UnityEventTools.AddVoidPersistentListener(button.onClick, onClick);
    }

    private static void ApplyButtonInk(Button button)
    {
        var label = button.transform.Find("Label")?.GetComponent<TextMeshProUGUI>();
        if (label == null) return;
        label.color = SlimUiSkin.Ink;
        label.fontStyle = FontStyles.Bold;
    }

    /// <summary>
    /// Slides a point-anchored element without touching its size.
    ///
    /// The shared Anchor helper pins the pivot to the centre, so a corner-anchored element
    /// sits half off-screen until it is nudged inward.
    ///
    /// This writes anchoredPosition only and deliberately does NOT write offsetMin/offsetMax.
    /// For a fixed-anchor RectTransform the size is derived as offsetMax - offsetMin, so
    /// writing both offsets to the same value collapses sizeDelta to zero — and a zero-width
    /// TMP box wraps to one character per line. That bug shipped once already on the race HUD
    /// and every test still passed; only a screenshot caught it.
    /// </summary>
    private static void Nudge(Component ui, Vector2 position)
    {
        ((RectTransform)ui.transform).anchoredPosition = position;
    }

    [MenuItem("Tools/Results/Create Results Scene")]
    public static void CreateResultsScene()
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (prefab == null)
        {
            Debug.LogError(
                $"[ResultsScreenUiBuilder] {PrefabPath} does not exist. Run " +
                "Tools > Results > Build Results Screen first.");
            return;
        }

        // Idempotent by deletion. The alternative — opening an existing scene and clearing it —
        // has to guess at what to clear, and a half-cleared scene is worse than none. This
        // scene is generated, so regenerating it is the whole contract. See the header warning
        // about hand-tuning it.
        if (AssetDatabase.LoadAssetAtPath<SceneAsset>(ScenePath) != null)
            AssetDatabase.DeleteAsset(ScenePath);

        // Created ADDITIVELY and never made active. EditorSceneManager.NewScene in Single mode
        // closes whatever the developer has open, and this project has already lost a
        // hand-placed table and a hand-placed camera to exactly that.
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Additive);

        var root = new GameObject("ResultsFlow");
        SceneManager.MoveGameObjectToScene(root, scene);

        var host = root.AddComponent<FlowSceneHost>();
        host.Configure(GameFlowManager.GameScreen.Results, prefab);
        root.AddComponent<ResultsSceneController>();

        EnsureEventSystem(scene);

        if (!EditorSceneManager.SaveScene(scene, ScenePath))
        {
            Debug.LogError($"[ResultsScreenUiBuilder] Failed to save {ScenePath}.");
            EditorSceneManager.CloseScene(scene, true);
            return;
        }

        EditorSceneManager.CloseScene(scene, true);

        // The .meta has to exist before the scene can be put in the build list: Unity resolves
        // a build entry by GUID, and a scene written a moment ago has none until the asset
        // database has seen it.
        AssetDatabase.Refresh();

        AddToBuildSettings();
        Debug.Log($"[ResultsScreenUiBuilder] Created {ScenePath} and added it to the build list.");
    }

    /// <summary>
    /// Gives the results scene an EventSystem.
    ///
    /// Without one, uGUI buttons cannot be clicked — and a Canvas does not need one to DRAW.
    /// That combination is why this failure looked like anything other than a missing object:
    /// the results panel appeared complete and correct, and both its buttons did absolutely
    /// nothing. The screen rendered; nothing was listening.
    ///
    /// Every other flow scene carries one, which is exactly why the omission was easy to miss
    /// when reading them for a template. <c>FlowSceneHost</c> only instantiates the screen
    /// prefab, and no screen prefab in the project carries an EventSystem, so each scene has
    /// to bring its own.
    ///
    /// The InputSystemUIInputModule is what makes the module the project uses everywhere else
    /// — the actions it serialises are assigned on add, the same way Phase3SceneBuilder's own
    /// EnsureEventSystem does it.
    /// </summary>
    private static void EnsureEventSystem(Scene scene)
    {
        var eventSystem = new GameObject("EventSystem");
        SceneManager.MoveGameObjectToScene(eventSystem, scene);

        eventSystem.AddComponent<EventSystem>();
        eventSystem.AddComponent<InputSystemUIInputModule>();
    }

    /// <summary>
    /// Appends the results scene to the build list, if it is not already there.
    ///
    /// Appended rather than inserted in flow order on purpose: only the FIRST entry is the
    /// scene Unity loads at startup, so where a later entry sits changes nothing about how the
    /// game boots. Appending cannot displace the entry that does matter, which inserting
    /// could.
    /// </summary>
    private static void AddToBuildSettings()
    {
        var scenes = new System.Collections.Generic.List<EditorBuildSettingsScene>(
            EditorBuildSettings.scenes);

        foreach (var entry in scenes)
        {
            if (entry.path == ScenePath)
            {
                entry.enabled = true;
                EditorBuildSettings.scenes = scenes.ToArray();
                AssetDatabase.SaveAssets();
                return;
            }
        }

        scenes.Add(new EditorBuildSettingsScene(ScenePath, true));
        EditorBuildSettings.scenes = scenes.ToArray();

        // Flushed here on purpose. Assigning EditorBuildSettings.scenes updates the in-memory
        // list and logs as though it succeeded, but ProjectSettings is written lazily — so a
        // build made before Unity next saves would silently omit the scene the flow routes to
        // at the end of every race. The save is the part that makes it real.
        AssetDatabase.SaveAssets();
    }

    /// <summary>
    /// Forces the unlock panel visible with sample content, on the live instance only.
    ///
    /// Uses the real names the impl writes to, so this doubles as a check that those objects
    /// exist and are where the code expects them — a renamed child would leave the panel
    /// silently blank at runtime and nothing else would notice.
    /// </summary>
    private static void StandUpUnlockPanel(GameObject instance)
    {
        var panel = instance.transform.Find("UnlockPanel");
        if (panel == null)
        {
            Debug.LogError("[ResultsScreenUiBuilder] The results screen has no UnlockPanel.");
            return;
        }

        panel.gameObject.SetActive(true);

        // Looked up from the PANEL, not the screen root: Transform.Find only searches direct
        // children, and these two live inside the panel. Asking the root for them reports
        // "no such child" for elements that exist and are wired.
        SetSample(panel, "CarUnlockText", "Gen 2 — 75,000 pts");
        SetSample(panel, "TrackUnlockText", "Monza — 50,000 pts");
        SetSample(instance.transform, "PointsEarnedText", "+1,000 pts");
        SetSample(instance.transform, "NewTotalPointsText", "Total: 51,250 pts");
        SetSample(instance.transform, "FinishPositionText", "P2");
    }

    private static void SetSample(Transform parent, string childName, string text)
    {
        var child = parent.Find(childName);
        if (child == null)
        {
            Debug.LogError($"[ResultsScreenUiBuilder] No '{childName}' under '{parent.name}'.");
            return;
        }

        var label = child.GetComponent<TMPro.TMP_Text>();
        if (label == null)
        {
            Debug.LogError($"[ResultsScreenUiBuilder] {childName} has no TMP_Text.");
            return;
        }

        label.text = text;
    }

    [MenuItem("Tools/Results/Build All")]
    public static void BuildAll()
    {
        BuildResultsScreen();
        CreateResultsScene();
    }

    [MenuItem("Tools/Results/Capture Results Screen")]
    public static void CaptureResultsScreen() => CaptureResultsScreen(false);

    [MenuItem("Tools/Results/Capture Results Screen (with unlocks)")]
    public static void CaptureResultsScreenWithUnlocks() => CaptureResultsScreen(true);

    private static void CaptureResultsScreen(bool showUnlocks)
    {
        // Rendering this screen and looking at it is the only real check on it. The EditMode
        // tests prove the buttons are wired and the fields are assigned; they cannot prove the
        // layout is legible, and this project has already shipped a HUD that passed every
        // assertion while rendering as a column of one-letter-per-line text.
        var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Additive);
        GameObject camGO = null;
        RenderTexture target = null;
        Canvas canvas = null;
        RenderMode originalMode = RenderMode.ScreenSpaceOverlay;

        try
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
            if (prefab == null)
            {
                Debug.LogError($"[ResultsScreenUiBuilder] {PrefabPath} does not exist.");
                return;
            }

            // The host instantiates its screen at runtime, so in edit mode the scene holds no
            // UI at all. Stand one up the same way the host would, or there is nothing to look
            // at.
            var instance = (GameObject)Object.Instantiate(prefab);
            instance.name = prefab.name + " (Capture)";
            SceneManager.MoveGameObjectToScene(instance, scene);

            // The unlock panel only appears when finishing actually put something within
            // reach, so the default capture would never exercise it. Standing it up here is
            // the only way to see that branch without first playing a race that triggers it.
            // Editor-side only — nothing written back to the prefab.
            if (showUnlocks) StandUpUnlockPanel(instance);

            canvas = instance.GetComponent<Canvas>();
            if (canvas == null)
            {
                Debug.LogError("[ResultsScreenUiBuilder] The results screen has no Canvas.");
                return;
            }

            camGO = new GameObject("ResultsCaptureCamera");
            var cam = camGO.AddComponent<Camera>();
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = Color.black;
            cam.orthographic = false;
            cam.fieldOfView = 60f;
            camGO.transform.position = new Vector3(0f, 0f, -10f);

            // A Screen Space - OVERLAY canvas is composited by the UI system and is invisible
            // to any camera, so a camera render captures an empty frame. Switching to
            // ScreenSpaceCamera for the duration of the shot is the only way to get an
            // overlay canvas into a RenderTexture — and it is restored below, because leaving
            // it switched would silently change how the screen behaves in the real game.
            originalMode = canvas.renderMode;
            canvas.renderMode = RenderMode.ScreenSpaceCamera;
            canvas.worldCamera = cam;
            canvas.planeDistance = 1f;

            target = new RenderTexture(1920, 1080, 24, RenderTextureFormat.ARGB32);
            cam.targetTexture = target;

            Canvas.ForceUpdateCanvases();
            cam.Render();

            var previous = RenderTexture.active;
            RenderTexture.active = target;
            var shot = new Texture2D(1920, 1080, TextureFormat.RGB24, false);
            shot.ReadPixels(new Rect(0, 0, 1920, 1080), 0, 0);
            shot.Apply();
            RenderTexture.active = previous;

            byte[] png = shot.EncodeToPNG();
            string path = showUnlocks
                ? "Assets/Screenshots/results_screen_unlocks.png"
                : "Assets/Screenshots/results_screen.png";
            System.IO.File.WriteAllBytes(path, png);
            AssetDatabase.ImportAsset(path);
            Debug.Log($"[ResultsScreenUiBuilder] Wrote {path}");
        }
        finally
        {
            if (canvas != null) canvas.renderMode = originalMode;
            if (target != null) target.Release();
            if (camGO != null) Object.DestroyImmediate(camGO);
            EditorSceneManager.CloseScene(scene, true);
        }
    }
}