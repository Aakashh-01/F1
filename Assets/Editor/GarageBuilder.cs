// Assets/Editor/GarageBuilder.cs
//
// Builds the client-demo garage: screen prefab, scene, display stats, and routing.
// Run: Tools > Garage > Build All.
//
// The garage scene is the MENU HUB: one background, two hosted screens (car panel on
// GameScreen.Lobby, track panel on GameScreen.TrackSelection). The 3D framing is
// authored in code below — Assets/Scenes/05_LobbyScene.unity (the old rig source) and
// 20_TrackSelectionScene.unity are deleted, so there is nothing left to capture from.
// The room art is not built yet; the car stands alone against a solid backdrop until
// the new lobby environment asset lands (see Assets/Scenes/ReferenceImageLobby.png).
//
// WARNING: after FlowSceneNames.Lobby was repointed to 05_GarageScene, Phase3SceneBuilder's
// Tools > BuildFlowScenes and Tools > BuildLobbyScene now resolve their lobby path HERE and
// would overwrite this scene with the old tile-strip lobby. Use Tools > Garage instead.
//
// WARNING: Tools > Lobby > Build Wing Setup Lobby still captures its rig from the
// deleted 05_LobbyScene and will error if run. 30_WingSetupScene is already built; do
// not rebuild it here.
//
// The showcase swap deliberately reuses the existing baked CarShowcase_Prefab for all six
// cars: every CarDefinition points at the same F1_Body body today, so six identical bakes
// would be six copies of the same file. The per-car tint (ShowcaseCarSwapper) is what tells
// them apart until real liveries exist.
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem.UI;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;

public static class GarageBuilder
{
    private const string GaragePrefabPath = "Assets/Prefabs/GarageScreen_Prefab.prefab";
    private const string TrackPanelPrefabPath = "Assets/Prefabs/TrackPanelScreen_Prefab.prefab";
    private const string GarageScenePath = "Assets/Scenes/05_GarageScene.unity";
    private const string ShowcasePrefabPath = "Assets/Prefabs/CarShowcase_Prefab.prefab";

    [MenuItem("Tools/Garage/Build All", priority = 0)]
    public static void BuildAll()
    {
        AuthorCarDisplayStats();
        AuthorTrackMenuDisplay();
        BuildGarageScreenPrefab();
        BuildTrackPanelPrefab();
        BuildGarageScene();
        UpdateBuildSettingsAndFlowRoot();
        AssetDatabase.SaveAssets();
        Debug.Log("[Garage] Garage prefab, scene, stats and routing built.");
    }

    // ==================================================================================
    // Car display stats (one-shot authoring pass)
    // ==================================================================================

    private static readonly string[] EngineLabels =
    {
        "1.6L V6 Turbo Hybrid",
        "1.6L V6 Turbo Hybrid ERS+",
        "1.6L V6 Turbo Hybrid ERS+",
        "1.6L V6-H MK3",
        "1.6L V6-H MK4",
        "1.6L V6-H MK5",
    };

    private static readonly float[] PowerHP   = { 748f, 790f, 860f, 930f, 980f, 1020f };
    private static readonly float[] TorqueNm  = { 720f, 735f, 770f, 800f, 825f, 845f };
    private static readonly float[] TopSpeed  = { 320f, 328f, 338f, 345f, 350f, 352f };
    private static readonly float[] Accel0100 = { 3.4f, 3.3f, 3.1f, 2.8f, 2.6f, 2.5f };
    private static readonly float[] WeightKg  = { 748f, 745f, 740f, 733f, 729f, 726f };

    private static readonly Color[] Tints =
    {
        new Color(0.31f, 0.76f, 0.97f, 1f),  // light blue
        new Color(1.00f, 0.84f, 0.31f, 1f),  // amber
        new Color(0.51f, 0.78f, 0.52f, 1f),  // green
        new Color(0.90f, 0.45f, 0.45f, 1f),  // red
        new Color(0.73f, 0.41f, 0.78f, 1f),  // purple
        new Color(1.00f, 0.54f, 0.40f, 1f),  // orange
    };

    /// <summary>
    /// Fills the garage-display placeholder fields on every CarDefinition asset: engine
    /// label, the five stats, and a pairwise-distinct tint. Assignment is by asset order
    /// (sorted by name) across a gen1-to-gen3 spread — these are SHOWROOM numbers for the
    /// client demo, not physics; the driving car never reads them.
    /// </summary>
    [MenuItem("Tools/Garage/Author Car Display Stats", priority = 10)]
    public static void AuthorCarDisplayStats()
    {
        var cars = LoadAllCars();
        if (cars.Count == 0)
        {
            Debug.LogError("[Garage] No CarDefinition assets found.");
            return;
        }
        if (cars.Count > EngineLabels.Length)
        {
            Debug.LogWarning($"[Garage] {cars.Count} car assets but stat tables hold " +
                             $"{EngineLabels.Length}; the extras are left untouched.");
        }

        for (int i = 0; i < cars.Count && i < EngineLabels.Length; i++)
        {
            var so = new SerializedObject(cars[i]);
            so.FindProperty("_engineLabel").stringValue = EngineLabels[i];
            so.FindProperty("_powerHP").floatValue = PowerHP[i];
            so.FindProperty("_torqueNm").floatValue = TorqueNm[i];
            so.FindProperty("_topSpeedKph").floatValue = TopSpeed[i];
            so.FindProperty("_accel0100Seconds").floatValue = Accel0100[i];
            so.FindProperty("_weightKg").floatValue = WeightKg[i];
            so.FindProperty("_displayTint").colorValue = Tints[i];
            // Without this the writes live only on the SerializedObject copy — SetDirty
            // alone marked the asset dirty with its ORIGINAL values, which is why the
            // first two passes wrote nothing to disk.
            so.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(cars[i]);
        }

        Debug.Log($"[Garage] Authored display stats on {System.Math.Min(cars.Count, EngineLabels.Length)} car asset(s).");

        // SetDirty alone does not survive the domain reloads that follow any script edit —
        // the first version of this pass lost every value that way, and the panel rendered
        // zeros until this save was added.
        AssetDatabase.SaveAssets();
    }

    private static List<CarDefinition> LoadAllCars()
    {
        return AssetDatabase.FindAssets("t:CarDefinition")
            .Select(g => AssetDatabase.LoadAssetAtPath<CarDefinition>(AssetDatabase.GUIDToAssetPath(g)))
            .Where(c => c != null)
            .OrderBy(c => c.name)
            .ToList();
    }

    // ==================================================================================
    // Track menu display (one-shot authoring pass)
    // ==================================================================================

    // Asset order is sorted by name: monaco, monza, silverstone, spa, suzuka.
    private static readonly string[] TrackDifficulty =
        { "HARD", "MEDIUM", "MEDIUM", "HARD", "HARD" };

    /// <summary>
    /// One-shot authoring of the track panel's display data, same contract as
    /// AuthorCarDisplayStats: plausible placeholders, applied by sorted asset order.
    /// </summary>
    [MenuItem("Tools/Garage/Author Track Menu Display", priority = 14)]
    public static void AuthorTrackMenuDisplay()
    {
        var tracks = AssetDatabase.FindAssets("t:TrackDefinition")
            .Select(g => AssetDatabase.LoadAssetAtPath<TrackDefinition>(AssetDatabase.GUIDToAssetPath(g)))
            .Where(t => t != null)
            .OrderBy(t => t.name)
            .ToList();

        for (int i = 0; i < tracks.Count && i < TrackDifficulty.Length; i++)
        {
            var so = new SerializedObject(tracks[i]);
            so.FindProperty("_difficultyLabel").stringValue = TrackDifficulty[i];
            so.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(tracks[i]);
        }

        AssetDatabase.SaveAssets();
        Debug.Log($"[Garage] Authored difficulty labels on {System.Math.Min(tracks.Count, TrackDifficulty.Length)} track asset(s).");
    }

    // ==================================================================================
    // Garage screen prefab
    // ==================================================================================

    /// <summary>
    /// Builds the garage screen: profile top-left, dual currency top-right, stats panel
    /// mid-left, arrows flanking the car, Continue bottom-right, and a drag layer as the
    /// FIRST canvas child so every panel sits above it and keeps its own raycasts.
    ///
    /// No UnityEvent wiring and no second pass: GarageScreenImpl self-wires its buttons in
    /// Awake, so the inert-buttons trap the lobby builder had to route around does not
    /// apply here. Serialized references only.
    ///
    /// Re-runnable: edits the prefab in place.
    /// </summary>
    [MenuItem("Tools/Garage/Build Garage Screen Prefab", priority = 11)]
    public static void BuildGarageScreenPrefab()
    {
        SlimUiSkin.EnsureLoaded();
        EnsureFolder("Assets/Prefabs");

        var root = new GameObject("GarageScreen_Prefab",
            typeof(RectTransform), typeof(Canvas),
            typeof(UnityEngine.UI.CanvasScaler),
            typeof(UnityEngine.UI.GraphicRaycaster),
            typeof(F1.UI.GarageScreenImpl));

        var rt = (RectTransform)root.transform;
        root.GetComponent<Canvas>().renderMode = RenderMode.ScreenSpaceOverlay;
        var scaler = root.GetComponent<UnityEngine.UI.CanvasScaler>();
        scaler.uiScaleMode = UnityEngine.UI.CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);
        scaler.screenMatchMode = UnityEngine.UI.CanvasScaler.ScreenMatchMode.MatchWidthOrHeight;
        scaler.matchWidthOrHeight = 1f;

        var impl = root.GetComponent<F1.UI.GarageScreenImpl>();

        // --- Drag layer: FIRST child, invisible, takes what the panels miss. ---
        var dragLayer = NewUiObject("DragLayer", rt);
        Stretch(dragLayer);
        var dragImage = dragLayer.gameObject.AddComponent<UnityEngine.UI.Image>();
        dragImage.color = new Color(0f, 0f, 0f, 0f);
        dragImage.raycastTarget = true;
        dragLayer.gameObject.AddComponent<GarageDragLayer>();

        // --- Player profile, top-left. ---
        var profile = NewPanel("ProfilePanel", rt, new Vector2(0f, 1f), new Vector2(400f, 140f), new Vector2(24f, 24f));
        var playerName = NewText("PlayerName", profile, "Player", 24f, TextAlignmentOptions.Left, SlimUiSkin.TileInk);
        PlaceAt(playerName, new Vector2(0f, 1f), new Vector2(250f, 34f), new Vector2(20f, 16f));
        var team = NewText("Team", profile, "Team", 17f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted);
        PlaceAt(team, new Vector2(0f, 1f), new Vector2(250f, 26f), new Vector2(20f, 56f));
        var level = NewText("Level", profile, "LEVEL 1", 17f, TextAlignmentOptions.Right, SlimUiSkin.TileInk);
        PlaceAt(level, new Vector2(1f, 1f), new Vector2(130f, 26f), new Vector2(20f, 18f));
        var xpBg = NewBar("XpBar", profile, new Vector2(0f, 1f), new Vector2(360f, 12f), new Vector2(20f, 100f));
        var xpFill = NewBarFill(xpBg, SlimUiSkin.Accent);
        var profileWidget = profile.gameObject.AddComponent<F1.UI.ProfileWidget>();
        var profileSo = new SerializedObject(profileWidget);
        Set(profileSo, "_playerNameText", playerName.GetComponent<TMP_Text>());
        Set(profileSo, "_teamText", team.GetComponent<TMP_Text>());
        Set(profileSo, "_levelText", level.GetComponent<TMP_Text>());
        Set(profileSo, "_xpFill", xpFill);
        profileSo.ApplyModifiedPropertiesWithoutUndo();

        // --- Dual currency, top-right. Career points first: unlocks cost careerPoints. ---
        var currency = NewPanel("CurrencyPanel", rt, new Vector2(1f, 1f), new Vector2(330f, 140f), new Vector2(24f, 24f));
        NewText("CareerLabel", currency, "CAREER PTS", 14f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted)
            .AndPlace(new Vector2(0f, 1f), new Vector2(290f, 22f), new Vector2(20f, 16f));
        var careerValue = NewText("CareerValue", currency, "0", 30f, TextAlignmentOptions.Left, SlimUiSkin.TileInk);
        PlaceAt(careerValue, new Vector2(0f, 1f), new Vector2(290f, 42f), new Vector2(20f, 38f));
        NewText("PremiumLabel", currency, "PREMIUM", 14f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted)
            .AndPlace(new Vector2(0f, 1f), new Vector2(290f, 22f), new Vector2(20f, 86f));
        var premiumValue = NewText("PremiumValue", currency, "0", 22f, TextAlignmentOptions.Left, SlimUiSkin.TileInk);
        PlaceAt(premiumValue, new Vector2(0f, 1f), new Vector2(290f, 32f), new Vector2(20f, 104f));
        var currencyWidget = currency.gameObject.AddComponent<F1.UI.CurrencyWidget>();
        var currencySo = new SerializedObject(currencyWidget);
        Set(currencySo, "_careerPointsText", careerValue.GetComponent<TMP_Text>());
        Set(currencySo, "_premiumText", premiumValue.GetComponent<TMP_Text>());
        currencySo.ApplyModifiedPropertiesWithoutUndo();

        // --- Stats panel, mid-left. The CanvasGroup is the locked dimmer; it must not
        //     block raycasts or it would eat the drags that belong to the layer below. ---
        var stats = NewPanel("StatsPanel", rt, new Vector2(0f, 0.5f), new Vector2(400f, 470f), new Vector2(24f, 20f));
        var dimmer = stats.gameObject.AddComponent<CanvasGroup>();
        dimmer.blocksRaycasts = false;
        dimmer.interactable = false;

        var carName = NewText("CarName", stats, "Car", 28f, TextAlignmentOptions.Left, SlimUiSkin.TileInk);
        PlaceAt(carName, new Vector2(0f, 1f), new Vector2(280f, 40f), new Vector2(20f, 16f));
        var carGen = NewText("CarGen", stats, "GEN 1", 16f, TextAlignmentOptions.Right, SlimUiSkin.TileInkMuted);
        PlaceAt(carGen, new Vector2(1f, 1f), new Vector2(100f, 24f), new Vector2(20f, 24f));
        var carStatus = NewText("CarStatus", stats, "OWNED", 19f, TextAlignmentOptions.Left, SlimUiSkin.Accent);
        PlaceAt(carStatus, new Vector2(0f, 1f), new Vector2(360f, 28f), new Vector2(20f, 62f));

        NewText("EngineLabel", stats, "ENGINE", 13f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted)
            .AndPlace(new Vector2(0f, 1f), new Vector2(100f, 20f), new Vector2(20f, 100f));
        var engineValue = NewText("EngineValue", stats, "—", 15f, TextAlignmentOptions.Right, SlimUiSkin.TileInk);
        PlaceAt(engineValue, new Vector2(1f, 1f), new Vector2(240f, 20f), new Vector2(20f, 100f));

        string[] rowLabels = { "POWER", "TORQUE", "TOP SPEED", "0-100", "WEIGHT" };
        var rowValues = new TMP_Text[5];
        var rowFills = new Image[5];
        for (int i = 0; i < 5; i++)
        {
            float y = 136f + i * 56f;
            NewText($"Row{i}Label", stats, rowLabels[i], 13f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted)
                .AndPlace(new Vector2(0f, 1f), new Vector2(240f, 18f), new Vector2(20f, y));
            rowValues[i] = NewText($"Row{i}Value", stats, "—", 17f, TextAlignmentOptions.Right, SlimUiSkin.TileInk)
                .AndPlace(new Vector2(1f, 1f), new Vector2(150f, 20f), new Vector2(20f, y - 2f))
                .GetComponent<TMP_Text>();
            var barBg = NewBar($"Row{i}Bar", stats, new Vector2(0f, 1f), new Vector2(360f, 10f), new Vector2(20f, y + 26f));
            rowFills[i] = NewBarFill(barBg, SlimUiSkin.Accent);
        }

        // --- Arrows flanking the car. Both secondary: Continue is the primary action. ---
        // ASCII "<"/">": the default TMP SDF atlas only guarantees ASCII, and geometric
        // shapes like ◀ render as blank boxes there.
        var prev = NewButton("PrevButton", rt, "<", 40f, primary: false);
        PlaceAt(prev, new Vector2(0f, 0.5f), new Vector2(90f, 130f), new Vector2(430f, 0f));
        var next = NewButton("NextButton", rt, ">", 40f, primary: false);
        PlaceAt(next, new Vector2(1f, 0.5f), new Vector2(90f, 130f), new Vector2(430f, 0f));

        // --- Continue, bottom-right. Dark until the flow accepts a session car. ---
        var continueBtn = NewButton("ContinueButton", rt, "CONTINUE TO RACE", 26f, primary: true);
        PlaceAt(continueBtn, new Vector2(1f, 0f), new Vector2(320f, 80f), new Vector2(24f, 28f));
        continueBtn.GetComponent<UnityEngine.UI.Button>().interactable = false;

        // --- Serialized wiring. ---
        var so = new SerializedObject(impl);
        Set(so, "_carNameText", carName.GetComponent<TMP_Text>());
        Set(so, "_carGenText", carGen.GetComponent<TMP_Text>());
        Set(so, "_carStatusText", carStatus.GetComponent<TMP_Text>());
        Set(so, "_engineValueText", engineValue.GetComponent<TMP_Text>());
        var rows = so.FindProperty("_statRows");
        rows.arraySize = 5;
        for (int i = 0; i < 5; i++)
        {
            var row = rows.GetArrayElementAtIndex(i);
            row.FindPropertyRelative("Value").objectReferenceValue = rowValues[i];
            row.FindPropertyRelative("Fill").objectReferenceValue = rowFills[i];
        }
        Set(so, "_currency", currencyWidget);
        Set(so, "_profile", profileWidget);
        Set(so, "_continueButton", continueBtn.GetComponent<UnityEngine.UI.Button>());
        Set(so, "_prevButton", prev.GetComponent<UnityEngine.UI.Button>());
        Set(so, "_nextButton", next.GetComponent<UnityEngine.UI.Button>());
        Set(so, "_dragLayer", dragLayer.GetComponent<GarageDragLayer>());
        Set(so, "_lockedDimmer", dimmer);
        so.ApplyModifiedPropertiesWithoutUndo();

        PrefabUtility.SaveAsPrefabAsset(root, GaragePrefabPath);
        Object.DestroyImmediate(root);
        Debug.Log($"[Garage] Screen prefab written to {GaragePrefabPath}");
    }

    // ==================================================================================
    // Track panel prefab
    // ==================================================================================

    /// <summary>
    /// Builds the track panel: SELECT TRACK title, three carousel slots (sides dimmed),
    /// edge arrows, BACK and CONFIRM, over the shared profile/currency chrome. Same
    /// self-wiring rule as the garage screen — serialized references only.
    ///
    /// Re-runnable: edits the prefab in place.
    /// </summary>
    [MenuItem("Tools/Garage/Build Track Panel Prefab", priority = 15)]
    public static void BuildTrackPanelPrefab()
    {
        SlimUiSkin.EnsureLoaded();
        EnsureFolder("Assets/Prefabs");

        var root = new GameObject("TrackPanelScreen_Prefab",
            typeof(RectTransform), typeof(Canvas),
            typeof(UnityEngine.UI.CanvasScaler),
            typeof(UnityEngine.UI.GraphicRaycaster),
            typeof(F1.UI.TrackPanelScreenImpl));

        var rt = (RectTransform)root.transform;
        root.GetComponent<Canvas>().renderMode = RenderMode.ScreenSpaceOverlay;
        var scaler = root.GetComponent<UnityEngine.UI.CanvasScaler>();
        scaler.uiScaleMode = UnityEngine.UI.CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);
        scaler.screenMatchMode = UnityEngine.UI.CanvasScaler.ScreenMatchMode.MatchWidthOrHeight;
        scaler.matchWidthOrHeight = 1f;

        // No root Graphic: the garage behind is the point, exactly as on the car panel.

        var title = NewText("Title", rt, "SELECT TRACK", 34f, TextAlignmentOptions.Center, SlimUiSkin.TileInk);
        PlaceAt(title, new Vector2(0.5f, 1f), new Vector2(600f, 60f), new Vector2(0f, 36f));

        // Carousel slots. PlaceAt zeroes the offset on a middle anchor, so the sides get
        // their anchoredPosition set directly after.
        var centre = NewPanel("SlotCentre", rt, new Vector2(0.5f, 0.5f), new Vector2(430f, 560f), Vector2.zero);
        centre.anchoredPosition = new Vector2(0f, 20f);
        var left = NewPanel("SlotLeft", rt, new Vector2(0.5f, 0.5f), new Vector2(320f, 430f), Vector2.zero);
        left.anchoredPosition = new Vector2(-560f, 60f);
        var right = NewPanel("SlotRight", rt, new Vector2(0.5f, 0.5f), new Vector2(320f, 430f), Vector2.zero);
        right.anchoredPosition = new Vector2(560f, 60f);

        var slots = new F1.UI.TrackPanelScreenImpl.TrackSlot[3];
        slots[1] = BuildTrackSlot(centre);
        slots[0] = BuildTrackSlot(left);
        slots[2] = BuildTrackSlot(right);

        // ASCII "<"/">": the default TMP SDF atlas only guarantees ASCII.
        var prev = NewButton("PrevButton", rt, "<", 40f, primary: false);
        PlaceAt(prev, new Vector2(0f, 0.5f), new Vector2(90f, 130f), new Vector2(60f, 0f));
        var next = NewButton("NextButton", rt, ">", 40f, primary: false);
        PlaceAt(next, new Vector2(1f, 0.5f), new Vector2(90f, 130f), new Vector2(60f, 0f));

        var back = NewButton("BackButton", rt, "BACK", 26f, primary: false);
        PlaceAt(back, new Vector2(0f, 0f), new Vector2(240f, 80f), new Vector2(24f, 28f));
        var confirm = NewButton("ConfirmButton", rt, "CONFIRM", 26f, primary: true);
        PlaceAt(confirm, new Vector2(1f, 0f), new Vector2(320f, 80f), new Vector2(24f, 28f));
        confirm.GetComponent<UnityEngine.UI.Button>().interactable = false;

        // Shared chrome — identical placement to the car panel so the corner identity
        // holds across the swap.
        var profile = NewPanel("ProfilePanel", rt, new Vector2(0f, 1f), new Vector2(400f, 140f), new Vector2(24f, 24f));
        var playerName = NewText("PlayerName", profile, "Player", 24f, TextAlignmentOptions.Left, SlimUiSkin.TileInk);
        PlaceAt(playerName, new Vector2(0f, 1f), new Vector2(250f, 34f), new Vector2(20f, 16f));
        var team = NewText("Team", profile, "Team", 17f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted);
        PlaceAt(team, new Vector2(0f, 1f), new Vector2(250f, 26f), new Vector2(20f, 56f));
        var level = NewText("Level", profile, "LEVEL 1", 17f, TextAlignmentOptions.Right, SlimUiSkin.TileInk);
        PlaceAt(level, new Vector2(1f, 1f), new Vector2(130f, 26f), new Vector2(20f, 18f));
        var xpBg = NewBar("XpBar", profile, new Vector2(0f, 1f), new Vector2(360f, 12f), new Vector2(20f, 100f));
        var xpFill = NewBarFill(xpBg, SlimUiSkin.Accent);
        var profileWidget = profile.gameObject.AddComponent<F1.UI.ProfileWidget>();
        var profileSo = new SerializedObject(profileWidget);
        Set(profileSo, "_playerNameText", playerName.GetComponent<TMP_Text>());
        Set(profileSo, "_teamText", team.GetComponent<TMP_Text>());
        Set(profileSo, "_levelText", level.GetComponent<TMP_Text>());
        Set(profileSo, "_xpFill", xpFill);
        profileSo.ApplyModifiedPropertiesWithoutUndo();

        var currency = NewPanel("CurrencyPanel", rt, new Vector2(1f, 1f), new Vector2(330f, 140f), new Vector2(24f, 24f));
        NewText("CareerLabel", currency, "CAREER PTS", 14f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted)
            .AndPlace(new Vector2(0f, 1f), new Vector2(290f, 22f), new Vector2(20f, 16f));
        var careerValue = NewText("CareerValue", currency, "0", 30f, TextAlignmentOptions.Left, SlimUiSkin.TileInk);
        PlaceAt(careerValue, new Vector2(0f, 1f), new Vector2(290f, 42f), new Vector2(20f, 38f));
        NewText("PremiumLabel", currency, "PREMIUM", 14f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted)
            .AndPlace(new Vector2(0f, 1f), new Vector2(290f, 22f), new Vector2(20f, 86f));
        var premiumValue = NewText("PremiumValue", currency, "0", 22f, TextAlignmentOptions.Left, SlimUiSkin.TileInk);
        PlaceAt(premiumValue, new Vector2(0f, 1f), new Vector2(290f, 32f), new Vector2(20f, 104f));
        var currencyWidget = currency.gameObject.AddComponent<F1.UI.CurrencyWidget>();
        var currencySo = new SerializedObject(currencyWidget);
        Set(currencySo, "_careerPointsText", careerValue.GetComponent<TMP_Text>());
        Set(currencySo, "_premiumText", premiumValue.GetComponent<TMP_Text>());
        currencySo.ApplyModifiedPropertiesWithoutUndo();

        var impl = root.GetComponent<F1.UI.TrackPanelScreenImpl>();
        var so = new SerializedObject(impl);
        Set(so, "_titleText", title.GetComponent<TMP_Text>());
        var slotsProp = so.FindProperty("_slots");
        slotsProp.arraySize = 3;
        for (int i = 0; i < 3; i++)
        {
            var slot = slotsProp.GetArrayElementAtIndex(i);
            var s = slots[i];
            slot.FindPropertyRelative("Root").objectReferenceValue = s.Root;
            slot.FindPropertyRelative("Art").objectReferenceValue = s.Art;
            slot.FindPropertyRelative("ShortCode").objectReferenceValue = s.ShortCode;
            slot.FindPropertyRelative("Name").objectReferenceValue = s.Name;
            slot.FindPropertyRelative("Length").objectReferenceValue = s.Length;
            slot.FindPropertyRelative("Laps").objectReferenceValue = s.Laps;
            slot.FindPropertyRelative("Difficulty").objectReferenceValue = s.Difficulty;
            slot.FindPropertyRelative("SelectedBadge").objectReferenceValue = s.SelectedBadge;
            slot.FindPropertyRelative("Dim").objectReferenceValue = s.Dim;
        }
        Set(so, "_backButton", back.GetComponent<UnityEngine.UI.Button>());
        Set(so, "_prevButton", prev.GetComponent<UnityEngine.UI.Button>());
        Set(so, "_nextButton", next.GetComponent<UnityEngine.UI.Button>());
        Set(so, "_confirmButton", confirm.GetComponent<UnityEngine.UI.Button>());
        Set(so, "_profile", profileWidget);
        Set(so, "_currency", currencyWidget);
        so.ApplyModifiedPropertiesWithoutUndo();

        PrefabUtility.SaveAsPrefabAsset(root, TrackPanelPrefabPath);
        Object.DestroyImmediate(root);
        Debug.Log($"[Garage] Track panel prefab written to {TrackPanelPrefabPath}");
    }

    /// <summary>
    /// Fills one carousel slot: art well up top, short-code chip, name, then the
    /// LENGTH / LAPS / DIFFICULTY stat band. The CanvasGroup is the side-slot dimmer.
    /// </summary>
    private static F1.UI.TrackPanelScreenImpl.TrackSlot BuildTrackSlot(RectTransform parent)
    {
        var slot = new F1.UI.TrackPanelScreenImpl.TrackSlot { Root = parent.gameObject };

        var dim = parent.gameObject.AddComponent<CanvasGroup>();
        dim.blocksRaycasts = false;
        dim.interactable = false;
        slot.Dim = dim;

        var badge = NewText("SelectedBadge", parent, "SELECTED", 15f, TextAlignmentOptions.Right, SlimUiSkin.Accent);
        PlaceAt(badge, new Vector2(1f, 1f), new Vector2(140f, 26f), new Vector2(16f, 12f));
        slot.SelectedBadge = badge.gameObject;

        var art = NewBar("ArtWell", parent, new Vector2(0.5f, 1f), new Vector2(parent.sizeDelta.x - 32f, parent.sizeDelta.y * 0.42f), new Vector2(0f, 16f));
        slot.Art = art.GetComponent<UnityEngine.UI.Image>();

        var code = NewText("ShortCode", parent, "", 14f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted);
        PlaceAt(code, new Vector2(0f, 1f), new Vector2(120f, 22f), new Vector2(16f, 16f));
        slot.ShortCode = code.GetComponent<TMP_Text>();

        var name = NewText("Name", parent, "Track", 26f, TextAlignmentOptions.Center, SlimUiSkin.TileInk);
        PlaceAt(name, new Vector2(0.5f, 1f), new Vector2(parent.sizeDelta.x - 32f, 40f), new Vector2(0f, parent.sizeDelta.y * 0.42f + 28f));
        slot.Name = name.GetComponent<TMP_Text>();

        string[] labels = { "LENGTH", "LAPS", "DIFFICULTY" };
        var texts = new TMP_Text[3];
        for (int i = 0; i < 3; i++)
        {
            float y = parent.sizeDelta.y * 0.42f + 92f + i * 46f;
            NewText($"Row{i}Label", parent, labels[i], 13f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted)
                .AndPlace(new Vector2(0f, 1f), new Vector2(160f, 20f), new Vector2(24f, y));
            texts[i] = NewText($"Row{i}Value", parent, "—", 18f, TextAlignmentOptions.Right, SlimUiSkin.TileInk)
                .AndPlace(new Vector2(1f, 1f), new Vector2(160f, 24f), new Vector2(24f, y))
                .GetComponent<TMP_Text>();
        }
        slot.Length = texts[0];
        slot.Laps = texts[1];
        slot.Difficulty = texts[2];

        return slot;
    }

    // ==================================================================================
    // Menu hub scene
    // ==================================================================================

    // Framing constants, matched to Assets/Scenes/ReferenceImageLobby.png: head-on, low,
    // symmetric, car centred. The verification pass in the plan tunes these against a
    // real screenshot; change them here, never in the scene file.
    private static readonly Vector3 SocketPosition = new Vector3(0f, 0.05f, 0f);
    private const float SocketYawDegrees = 180f;   // nose toward the camera at -Z
    private const float SocketScale = 0.8f;
    private static readonly Vector3 CameraPosition = new Vector3(0f, 0.85f, -5.2f);
    private static readonly Vector3 CameraLookAt = new Vector3(0f, -0.55f, 0.2f);
    private const float CameraFov = 34f;
    private static readonly Color BackdropColor = new Color(0.055f, 0.065f, 0.08f, 1f);

    /// <summary>
    /// Builds 05_GarageScene as the menu hub: car socket + turntable + swapper, a camera
    /// and two lights framing the car head-on, a FlowSceneHost for the car panel
    /// (GameScreen.Lobby) and a second, start-inactive host for the track panel
    /// (GameScreen.TrackSelection), a GarageHubView to swap them, and one EventSystem.
    ///
    /// Always builds a fresh empty scene and saves over the path, so the old Simple
    /// Garage room cannot survive a rebuild.
    /// </summary>
    [MenuItem("Tools/Garage/Build Garage Scene", priority = 12)]
    public static void BuildGarageScene()
    {
        var garagePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(GaragePrefabPath);
        var trackPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(TrackPanelPrefabPath);
        if (garagePrefab == null)
            Debug.LogError($"[Garage] {GaragePrefabPath} missing; build the screen prefab first.");
        if (trackPrefab == null)
            Debug.LogError($"[Garage] {TrackPanelPrefabPath} missing; build the track panel prefab first.");

        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        // --- Car socket -------------------------------------------------------------
        var showcasePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(ShowcasePrefabPath);
        if (showcasePrefab == null)
            Debug.LogError($"[Garage] {ShowcasePrefabPath} missing; build it via Tools > Lobby.");

        // NewScene(Single) makes the new scene active, so everything created below lands
        // in it directly — no MoveGameObjectToScene round trips.
        var socket = new GameObject("CarSocket");
        socket.transform.localPosition = SocketPosition;
        socket.transform.localRotation = Quaternion.Euler(0f, SocketYawDegrees, 0f);
        socket.transform.localScale = new Vector3(SocketScale, SocketScale, SocketScale);
        socket.AddComponent<GarageTurntable>();
        var swapper = socket.AddComponent<ShowcaseCarSwapper>();
        WireSwapperEntries(swapper, showcasePrefab);

        // --- Camera, head-on and low -------------------------------------------------
        var camGo = new GameObject("LobbyCamera", typeof(Camera));
        camGo.tag = "MainCamera";
        var cam = camGo.GetComponent<Camera>();
        cam.fieldOfView = CameraFov;
        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 60f;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = BackdropColor;
        if (camGo.GetComponent<UnityEngine.Rendering.Universal.UniversalAdditionalCameraData>() == null)
            camGo.AddComponent<UnityEngine.Rendering.Universal.UniversalAdditionalCameraData>();
        camGo.transform.position = CameraPosition;
        camGo.transform.rotation = Quaternion.LookRotation(
            (CameraLookAt - CameraPosition).normalized, Vector3.up);

        // --- Key + fill ---------------------------------------------------------------
        // The reference's ceiling strips are art; until then a front-top key and a cool
        // rear fill give the car readable form against the backdrop.
        var keyGo = new GameObject("LobbyKeyLight", typeof(Light));
        var key = keyGo.GetComponent<Light>();
        key.type = LightType.Spot;
        key.color = new Color(1f, 0.98f, 0.94f);
        key.intensity = 1.4f;
        key.range = 35f;
        key.spotAngle = 62f;
        key.innerSpotAngle = 40f;
        key.shadows = LightShadows.Soft;
        keyGo.transform.position = new Vector3(0f, 5.5f, -4.5f);
        keyGo.transform.rotation = Quaternion.LookRotation(
            (SocketPosition + Vector3.up * 0.7f - new Vector3(0f, 5.5f, -4.5f)).normalized, Vector3.up);

        var fillGo = new GameObject("LobbyFillLight", typeof(Light));
        var fill = fillGo.GetComponent<Light>();
        fill.type = LightType.Directional;
        fill.color = new Color(0.75f, 0.82f, 0.95f);
        fill.intensity = 0.35f;
        fillGo.transform.rotation = Quaternion.Euler(38f, 145f, 0f);

        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
        RenderSettings.ambientLight = new Color(0.32f, 0.33f, 0.36f);

        // --- Hosts --------------------------------------------------------------------
        var carHostGo = EnsureRoot(scene, "FlowSceneHost");
        var carHost = EnsureComponent<FlowSceneHost>(carHostGo);
        carHost.Configure(GameFlowManager.GameScreen.Lobby, garagePrefab,
            HostScreenMode.Fixed, carHostGo.transform);
        EnsureComponent<GarageSceneController>(carHostGo);

        var trackHostGo = EnsureRoot(scene, "TrackPanelHost");
        var trackHost = EnsureComponent<FlowSceneHost>(trackHostGo);
        trackHost.Configure(GameFlowManager.GameScreen.TrackSelection, trackPrefab,
            HostScreenMode.Fixed, trackHostGo.transform);
        EnsureComponent<TrackPanelSceneController>(trackHostGo);
        // The car panel owns the stage on entry; GarageHubView swaps from here.
        trackHostGo.SetActive(false);

        var hubGo = EnsureRoot(scene, "MenuHub");
        EnsureComponent<GarageHubView>(hubGo);

        EnsureEventSystem(scene);

        EditorSceneManager.SaveScene(scene, GarageScenePath);
        Debug.Log($"[Garage] Menu hub scene saved to {GarageScenePath}");
    }

    private static void WireSwapperEntries(ShowcaseCarSwapper swapper, GameObject showcasePrefab)
    {
        var cars = LoadAllCars();
        var so = new SerializedObject(swapper);
        var entries = so.FindProperty("_entries");
        entries.arraySize = cars.Count;
        for (int i = 0; i < cars.Count; i++)
        {
            var entry = entries.GetArrayElementAtIndex(i);
            entry.FindPropertyRelative("Car").objectReferenceValue = cars[i];
            entry.FindPropertyRelative("ShowcasePrefab").objectReferenceValue = showcasePrefab;
        }
        so.ApplyModifiedPropertiesWithoutUndo();
    }

    // ==================================================================================
    // Routing: build settings + the flow root's serialized scene fields
    // ==================================================================================

    /// <summary>
    /// Routing: drop the retired scenes from the build list, keep the route order
    /// Loading -> Garage(hub) -> WingSetup -> PreRace -> Race -> Track_01 -> Results, and
    /// rewrite 00_LoadingScene's serialized scene fields, which would otherwise keep
    /// pointing at retired scenes and quietly load them.
    /// </summary>
    [MenuItem("Tools/Garage/Update Build Settings && Flow Root", priority = 13)]
    public static void UpdateBuildSettingsAndFlowRoot()
    {
        var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
        int removed = scenes.RemoveAll(s =>
            Path.GetFileNameWithoutExtension(s.path) == FlowSceneNames.RetiredGarageLobby ||
            Path.GetFileNameWithoutExtension(s.path) == FlowSceneNames.RetiredTrackSelection);
        if (removed > 0)
            Debug.Log($"[Garage] Removed {removed} retired scene entr(ies) from the build list.");

        bool hasHub = scenes.Exists(s =>
            Path.GetFileNameWithoutExtension(s.path) == FlowSceneNames.Lobby);
        if (!hasHub)
        {
            scenes.Insert(1, new EditorBuildSettingsScene(GarageScenePath, true));
            Debug.Log("[Garage] Inserted the hub scene after loading.");
        }
        EditorBuildSettings.scenes = scenes.ToArray();

        var loadingPath = "Assets/Scenes/" + FlowSceneNames.Loading + ".unity";
        if (!File.Exists(ToAbsolute(loadingPath)))
        {
            Debug.LogError($"[Garage] {loadingPath} does not exist; flow root not updated.");
            return;
        }

        var scene = EditorSceneManager.OpenScene(loadingPath, OpenSceneMode.Single);
        var flow = Object.FindAnyObjectByType<GameFlowManager>(FindObjectsInactive.Include);
        if (flow == null)
        {
            Debug.LogError("[Garage] 00_LoadingScene has no GameFlowManager; flow root not updated.");
            return;
        }

        var so = new SerializedObject(flow);
        var lobbyProp = so.FindProperty("_lobbyScene");
        var trackProp = so.FindProperty("_trackSelectionScene");
        if (lobbyProp == null || trackProp == null)
        {
            Debug.LogError("[Garage] GameFlowManager scene fields not found; flow root not updated.");
            return;
        }
        lobbyProp.stringValue = FlowSceneNames.Lobby;
        trackProp.stringValue = FlowSceneNames.TrackSelection;
        so.ApplyModifiedPropertiesWithoutUndo();
        EditorSceneManager.SaveScene(scene);
        Debug.Log($"[Garage] Flow root scenes -> lobby '{FlowSceneNames.Lobby}', " +
                  $"track selection '{FlowSceneNames.TrackSelection}'.");
    }

    // ==================================================================================
    // UI helpers
    // ==================================================================================

    private static RectTransform NewUiObject(string name, Transform parent)
    {
        var go = new GameObject(name, typeof(RectTransform));
        var r = (RectTransform)go.transform;
        r.SetParent(parent, false);
        return r;
    }

    private static void Stretch(RectTransform rt)
    {
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;
    }

    /// <summary>An opaque flat card. The pack's panel sprite is translucent mid-grey and
    /// would let the dark garage through under dark-on-dark text, so panels get solid
    /// fills and reserve the pack's frame art for overlays.</summary>
    private static RectTransform NewPanel(string name, Transform parent,
        Vector2 anchor, Vector2 size, Vector2 offset)
    {
        var rt = NewUiObject(name, parent);
        PlaceAt(rt, anchor, size, offset);
        var image = rt.gameObject.AddComponent<UnityEngine.UI.Image>();
        SlimUiSkin.ApplyFlatPanel(image, SlimUiSkin.CardFill);
        image.raycastTarget = false;
        return rt;
    }

    private static RectTransform NewText(string name, Transform parent, string content,
        float size, TextAlignmentOptions align, Color color)
    {
        var rt = NewUiObject(name, parent);
        var text = rt.gameObject.AddComponent<TextMeshProUGUI>();
        text.text = content;
        text.fontSize = size;
        text.alignment = align;
        text.color = color;
        text.raycastTarget = false;
        return rt;
    }

    private static RectTransform NewButton(string name, Transform parent, string label,
        float size, bool primary = false)
    {
        var rt = NewUiObject(name, parent);
        var image = rt.gameObject.AddComponent<UnityEngine.UI.Image>();
        image.raycastTarget = true;
        var button = rt.gameObject.AddComponent<UnityEngine.UI.Button>();
        button.targetGraphic = image;
        SlimUiSkin.ApplyButtonVisual(button, primary);

        var text = NewText("Label", rt, label, size, TextAlignmentOptions.Center, SlimUiSkin.Ink);
        Stretch(text);
        return rt;
    }

    private static RectTransform NewBar(string name, Transform parent,
        Vector2 anchor, Vector2 size, Vector2 offset)
    {
        var rt = NewUiObject(name, parent);
        PlaceAt(rt, anchor, size, offset);
        var image = rt.gameObject.AddComponent<UnityEngine.UI.Image>();
        image.color = SlimUiSkin.ArtWellFill;
        image.raycastTarget = false;
        return rt;
    }

    private static Image NewBarFill(RectTransform bar, Color color)
    {
        var rt = NewUiObject("Fill", bar);
        Stretch(rt);
        var image = rt.gameObject.AddComponent<UnityEngine.UI.Image>();
        image.color = color;
        image.type = UnityEngine.UI.Image.Type.Filled;
        image.fillMethod = UnityEngine.UI.Image.FillMethod.Horizontal;
        image.fillAmount = 0f;
        image.raycastTarget = false;
        return image;
    }

    /// <summary>Pins a box to a corner or edge; see LobbyBuilder.PlaceAt for the reasoning.
    /// Returns the rect so one-liners can chain a component lookup.</summary>
    private static void PlaceAt(RectTransform rt, Vector2 anchor, Vector2 size, Vector2 offset)
    {
        rt.anchorMin = anchor;
        rt.anchorMax = anchor;
        rt.pivot = anchor;
        rt.sizeDelta = size;

        float x = anchor.x < 0.5f ? offset.x : (anchor.x > 0.5f ? -offset.x : 0f);
        float y = anchor.y < 0.5f ? offset.y : (anchor.y > 0.5f ? -offset.y : 0f);
        rt.anchoredPosition = new Vector2(x, y);
    }

    private static RectTransform AndPlace(this RectTransform rt, Vector2 anchor,
        Vector2 size, Vector2 offset)
    {
        PlaceAt(rt, anchor, size, offset);
        return rt;
    }

    private static void Set(SerializedObject so, string field, Object value)
    {
        var prop = so.FindProperty(field);
        if (prop == null)
        {
            Debug.LogError($"[Garage] Field '{field}' not found on {so.targetObject.GetType().Name}.");
            return;
        }
        prop.objectReferenceValue = value;
    }

    // ==================================================================================
    // Scene helpers (idempotency, same pattern as Phase3SceneBuilder)
    // ==================================================================================

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
            Debug.LogWarning($"[Garage] Removing duplicate root '{name}'.");
            Object.DestroyImmediate(duplicate);
        }
        return keep != null ? keep : new GameObject(name);
    }

    private static T EnsureComponent<T>(GameObject go) where T : Component
    {
        var existing = go.GetComponent<T>();
        return existing != null ? existing : go.AddComponent<T>();
    }

    private static void EnsureEventSystem(Scene scene)
    {
        var es = EnsureRoot(scene, "EventSystem");
        EnsureComponent<EventSystem>(es);
        EnsureComponent<InputSystemUIInputModule>(es);
    }

    private static string ToAbsolute(string assetPath) =>
        Path.Combine(Directory.GetCurrentDirectory(), assetPath);

    private static void EnsureFolder(string path)
    {
        if (AssetDatabase.IsValidFolder(path)) return;
        var parent = Path.GetDirectoryName(path)?.Replace('\\', '/');
        var leaf = Path.GetFileName(path);
        if (!string.IsNullOrEmpty(parent) && !AssetDatabase.IsValidFolder(parent))
            EnsureFolder(parent);
        if (AssetDatabase.IsValidFolder(path)) return;
        AssetDatabase.CreateFolder(parent, leaf);
    }
}
