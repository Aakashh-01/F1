// Assets/Editor/RaceScreenUiBuilder.cs - builds the race HUD prefab and installs it into
// 50_RaceScene. Run: Tools > Race/Build Race Screen UI, or Tools > Race/Install Race Screen.
//
// Deliberately a SEPARATE, narrow builder. UIBuilder.BuildAll (Tools > BuildUIPrefabs)
// rebuilds all nine prefabs from scratch, which would throw away the hand-finished
// track-selection hero layout; WingScreenUiBuilder was split out for exactly that reason and
// this follows the same precedent. Nothing here touches any other prefab or scene.
using TMPro;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using F1.GameFlow;
using F1.UI;

public static class RaceScreenUiBuilder
{
    private const string PrefabName = "RaceScreen_Prefab";
    private const string PrefabPath = "Assets/Prefabs/" + PrefabName + ".prefab";
    private const string RaceScenePath = "Assets/Scenes/50_RaceScene.unity";

    // Same palette the qualifying HUD uses, so the two overlays read as one system.
    private static readonly Color Bright = new Color(0.96f, 0.98f, 1f, 1f);
    private static readonly Color Dim = new Color(0.72f, 0.84f, 0.94f, 0.96f);
    private static readonly Color Accent = SlimUiSkin.Accent;

    [MenuItem("Tools/Race/Build Race Screen UI")]
    public static void BuildRaceScreen()
    {
        GameObject go = UIBuilder.ScreenRoot(PrefabName, opaqueBackground: false);
        var impl = go.AddComponent<RaceScreenImpl>();

        // --- Top left: where you are. The single most-read number on a race HUD. ---
        var position = UIBuilder.AddText(go.transform, "PositionText",
            new Vector2(0f, 1f), new Vector2(300f, 76f),
            "P1 / 1", 56, Accent, FontStyles.Bold);
        // Anchor helper pins the pivot centre, so the top-left block needs a nudge to sit
        // in the corner rather than half off-screen.
        Nudge(position, new Vector2(170f, -50f));

        // --- Top centre: where you are, in the session's terms. ---
        var trackName = UIBuilder.AddText(go.transform, "TrackNameText",
            new Vector2(0.5f, 1f), new Vector2(700f, 34f),
            "Track", 22, Color.white, FontStyles.Bold);
        Nudge(trackName, new Vector2(0f, -28f));

        var totalLaps = UIBuilder.AddText(go.transform, "TotalLapsText",
            new Vector2(0.5f, 1f), new Vector2(700f, 28f),
            "Laps: 0 | Grid: 0", 18, Dim);
        Nudge(totalLaps, new Vector2(0f, -58f));

        // --- Top right: which lap, and the pause control. ---
        var currentLap = UIBuilder.AddText(go.transform, "CurrentLapText",
            new Vector2(1f, 1f), new Vector2(300f, 44f),
            "Lap 1 / 0", 30, Color.white, FontStyles.Bold);
        Nudge(currentLap, new Vector2(-190f, -40f));

        var pause = UIBuilder.AddButton(go.transform, "PauseButton",
            new Vector2(1f, 1f), new Vector2(64f, 64f), "II", impl, impl.OnPauseClicked);
        Nudge(pause, new Vector2(-46f, -46f));

        // --- Bottom right: the clock. Same place and weight as the qualifying lap time, so
        // the eye that learned to read one reads the other. ---
        var lapTime = UIBuilder.AddText(go.transform, "LapTimeText",
            new Vector2(1f, 0f), new Vector2(420f, 64f),
            "0:00.000", 44, Bright, FontStyles.Bold);
        Nudge(lapTime, new Vector2(-220f, 95f));

        var bestLap = UIBuilder.AddText(go.transform, "BestLapText",
            new Vector2(1f, 0f), new Vector2(420f, 32f),
            "Best: --:--.---", 20, Dim);
        Nudge(bestLap, new Vector2(-220f, 48f));

        // --- Bottom left: the three sectors. ---
        var sectorCaption = UIBuilder.AddText(go.transform, "SectorCaptionText",
            new Vector2(0f, 0f), new Vector2(300f, 26f),
            "SECTORS", 16, Dim);
        Nudge(sectorCaption, new Vector2(180f, 132f));

        var sector1 = UIBuilder.AddText(go.transform, "Sector1Text",
            new Vector2(0f, 0f), new Vector2(150f, 34f), "S1  --:--.---", 18, Dim);
        Nudge(sector1, new Vector2(95f, 105f));

        var sector2 = UIBuilder.AddText(go.transform, "Sector2Text",
            new Vector2(0f, 0f), new Vector2(150f, 34f), "S2  --:--.---", 18, Dim);
        Nudge(sector2, new Vector2(95f, 76f));

        var sector3 = UIBuilder.AddText(go.transform, "Sector3Text",
            new Vector2(0f, 0f), new Vector2(150f, 34f), "S3  --:--.---", 18, Dim);
        Nudge(sector3, new Vector2(95f, 47f));

        // --- Finish panel. Inactive at build time so the prefab opens in the driving
        // state, exactly as the qualifying results panel does. ---
        GameObject finishPanel = UIBuilder.NewUI("FinishPanel");
        finishPanel.transform.SetParent(go.transform, false);
        UIBuilder.Anchor(finishPanel, new Vector2(0.5f, 0.5f), new Vector2(900f, 400f));
        finishPanel.AddComponent<Image>().color = new Color(0.04f, 0.05f, 0.08f, 0.96f);

        UIBuilder.AddText(finishPanel.transform, "FinishTitleText",
            new Vector2(0.5f, 0.82f), new Vector2(700f, 44f),
            "RACE COMPLETE", 28, Color.white, FontStyles.Bold);

        var finishPosition = UIBuilder.AddText(finishPanel.transform, "FinishPositionText",
            new Vector2(0.5f, 0.58f), new Vector2(700f, 72f),
            "Finished: P1", 48, Accent, FontStyles.Bold);

        var points = UIBuilder.AddText(finishPanel.transform, "PointsEarnedText",
            new Vector2(0.5f, 0.38f), new Vector2(700f, 40f),
            "+0 pts", 26, Dim);

        finishPanel.SetActive(false);

        // --- Skin pass ---
        //
        // Split deliberately, and the split is the same one the qualifying HUD uses. Text
        // that floats directly over the 3D gets the shadowed HUD material, because without
        // it a white number over a bright sky or a pale kerb is unreadable at racing speed.
        // Text sitting on an opaque panel does not need it and keeps the font's stock
        // material. Folding the skin in here rather than shipping a separate pass is
        // deliberate: `Tools > BuildUIPrefabs` destroys the qualifying skin, and a builder
        // whose output depends on a second menu item is a builder that quietly rots.
        foreach (var overlayText in new[]
                 {
                     position, trackName, totalLaps, currentLap, lapTime, bestLap,
                     sectorCaption, sector1, sector2, sector3,
                 })
        {
            SlimUiSkin.ApplyHudTextMaterial(overlayText);
        }

        // Panel-internal text: ink palette on the card fill, stock material.
        finishPanel.GetComponent<Image>().color = SlimUiSkin.CardFill;
        foreach (var t in finishPanel.GetComponentsInChildren<TMPro.TMP_Text>(true))
        {
            t.color = t.fontStyle == TMPro.FontStyles.Bold ? SlimUiSkin.TileInk : SlimUiSkin.TileInkMuted;
        }

        SlimUiSkin.ApplyPrimaryButton(pause);
        var pauseLabel = pause.transform.Find("Label")?.GetComponent<TMPro.TextMeshProUGUI>();
        if (pauseLabel != null)
        {
            pauseLabel.color = SlimUiSkin.Ink;
            pauseLabel.fontStyle = TMPro.FontStyles.Bold;
        }

        UIBuilder.SetRefs(impl,
            ("_trackNameText", trackName),
            ("_totalLapsText", totalLaps),
            ("_positionText", position),
            ("_currentLapText", currentLap),
            ("_lapTimeText", lapTime),
            ("_bestLapText", bestLap),
            ("_sector1Text", sector1),
            ("_sector2Text", sector2),
            ("_sector3Text", sector3),
            ("_finishPanel", finishPanel),
            ("_finishPositionText", finishPosition),
            ("_pointsEarnedText", points),
            ("_pauseButton", pause));

        GameObject asset = UIBuilder.Save(PrefabName, go);
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log("[RaceScreenUiBuilder] Built " + PrefabPath + " -> " + (asset != null ? "ok" : "FAILED"));
    }

    /// <summary>
    /// Slides a point-anchored element without touching its size.
    ///
    /// The shared <c>Anchor</c> helper pins the pivot to the centre, so an element anchored
    /// to a screen corner sits half off-screen by default and needs sliding inward.
    ///
    /// This moves <c>anchoredPosition</c> and deliberately does NOT write offsetMin/offsetMax.
    /// For a fixed-anchor RectTransform the size is derived as offsetMax - offsetMin, so
    /// writing both offsets to the same value collapses sizeDelta to zero — and a zero-width
    /// TMP box wraps to one character per line. That is exactly what the first build of this
    /// HUD did, and it is why the position, lap and clock all rendered as vertical letter
    /// columns. Takes a Component so it works for both the TMP texts and the Button.
    ///
    /// The screenshot is the only thing that catches this: every assertion still passed.
    /// </summary>
    private static void Nudge(Component ui, Vector2 position)
    {
        var rt = (RectTransform)ui.transform;
        rt.anchoredPosition = position;
    }

    [MenuItem("Tools/Race/Install Race Screen")]
    public static void InstallIntoRaceScene()
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
        if (prefab == null)
        {
            Debug.LogError(
                $"[RaceScreenUiBuilder] {PrefabPath} does not exist. Run " +
                "Tools > Race > Build Race Screen UI first.");
            return;
        }

        // Opened additively and never made active. Opening it Single would tear down whatever
        // the developer has open, and a flat-clear camera in this project has already cost a
        // chase camera once (a cullingMask of 0 clears the shared render target).
        var scene = EditorSceneManager.OpenScene(RaceScenePath, OpenSceneMode.Additive);
        try
        {
            FlowSceneHost host = null;
            foreach (var root in scene.GetRootGameObjects())
            {
                host = root.GetComponentInChildren<FlowSceneHost>(true);
                if (host != null) break;
            }

            if (host == null)
            {
                Debug.LogError(
                    $"[RaceScreenUiBuilder] No FlowSceneHost in {RaceScenePath}. The race " +
                    "scene cannot host a screen without one.");
                return;
            }

            // Idempotent: re-running assigns the same reference, it does not add a second
            // screen instance. The host instantiates the prefab itself in Awake.
            var so = new SerializedObject(host);
            so.FindProperty("_screenPrefab").objectReferenceValue = prefab;
            so.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(host);

            if (!EditorSceneManager.SaveScene(scene))
                Debug.LogError($"[RaceScreenUiBuilder] Failed to save {RaceScenePath}.");

            Debug.Log($"[RaceScreenUiBuilder] {PrefabName} installed on the race host.");
        }
        finally
        {
            EditorSceneManager.CloseScene(scene, true);
        }
    }

    [MenuItem("Tools/Race/Build And Install Race Screen")]
    public static void BuildAndInstall()
    {
        BuildRaceScreen();
        InstallIntoRaceScene();
    }
}
