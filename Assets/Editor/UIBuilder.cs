// Assets/Editor/UIBuilder.cs - Builds and wires all 8 UI prefabs per the pd2/Hermes spec.
// Run: Tools > BuildUIPrefabs. Idempotent: rebuilds each prefab from scratch (GUID preserved by path).
using TMPro;
using UnityEditor;
using UnityEditor.Events;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;
using F1.UI;

public class UIBuilder
{
    private const string Folder = "Assets/Prefabs/";

    private static readonly Color PanelBg = new Color(0.04f, 0.05f, 0.08f, 1f);
    private static readonly Color ButtonBg = new Color(0.15f, 0.17f, 0.22f, 1f);
    private static readonly Color ToggleBg = new Color(0.13f, 0.15f, 0.20f, 1f);
    private static readonly Color CardBg = new Color(0.08f, 0.10f, 0.14f, 1f);
    private static readonly Color Green = new Color(0.3f, 1f, 0.3f, 1f);

    [MenuItem("Tools/BuildUIPrefabs")]
    public static void BuildAll()
    {
        // Cards first: selection screens hold prefab references to them.
        GameObject carCard = BuildCarCard();
        GameObject trackCard = BuildTrackCard();

        BuildBranding();
        BuildMainMenu();
        BuildCarSelection(carCard);
        BuildTrackSelection(trackCard);
        BuildWingSetup();
        BuildModeSelection();
        BuildQualifying();

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log("[UIBuilder] All 9 UI prefabs built.");
    }

    // --- Shared helpers ---

    private static GameObject NewUI(string name)
    {
        var go = new GameObject(name, typeof(RectTransform));
        go.layer = LayerMask.NameToLayer("UI");
        return go;
    }

    private static GameObject Save(string name, GameObject go)
    {
        string path = Folder + name + ".prefab";
        GameObject asset = PrefabUtility.SaveAsPrefabAsset(go, path);
        Object.DestroyImmediate(go);
        return asset;
    }

    private static void SetRefs(Component impl, params (string prop, Object value)[] refs)
    {
        var so = new SerializedObject(impl);
        foreach (var (prop, value) in refs)
        {
            SerializedProperty sp = so.FindProperty(prop);
            if (sp == null)
                Debug.LogError($"[UIBuilder] Missing field '{prop}' on {impl.GetType().Name}");
            else
                sp.objectReferenceValue = value;
        }
        so.ApplyModifiedPropertiesWithoutUndo();
    }

    private static RectTransform Anchor(GameObject go, Vector2 anchor, Vector2 size)
    {
        var rt = (RectTransform)go.transform;
        rt.anchorMin = anchor;
        rt.anchorMax = anchor;
        rt.pivot = new Vector2(0.5f, 0.5f);
        rt.sizeDelta = size;
        return rt;
    }

    private static TMP_Text AddText(Transform parent, string name, Vector2 anchor, Vector2 size,
        string content, int fontSize, Color color, FontStyles style = FontStyles.Normal)
    {
        GameObject go = NewUI(name);
        go.transform.SetParent(parent, false);
        Anchor(go, anchor, size);
        var tmp = go.AddComponent<TextMeshProUGUI>();
        tmp.text = content;
        tmp.fontSize = fontSize;
        tmp.color = color;
        tmp.fontStyle = style;
        tmp.alignment = TextAlignmentOptions.Center;
        return tmp;
    }

    // Full-rect centered label (used inside buttons/toggles).
    private static TMP_Text AddLabel(Transform parent, string content, int fontSize)
    {
        GameObject go = NewUI("Label");
        go.transform.SetParent(parent, false);
        var rt = (RectTransform)go.transform;
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;
        var tmp = go.AddComponent<TextMeshProUGUI>();
        tmp.text = content;
        tmp.fontSize = fontSize;
        tmp.color = Color.white;
        tmp.alignment = TextAlignmentOptions.Center;
        return tmp;
    }

    private static Button AddButton(Transform parent, string name, Vector2 anchor, Vector2 size,
        string label, Component impl, UnityAction onClick)
    {
        GameObject go = NewUI(name);
        go.transform.SetParent(parent, false);
        Anchor(go, anchor, size);
        var img = go.AddComponent<Image>();
        img.color = ButtonBg;
        var btn = go.AddComponent<Button>();
        btn.targetGraphic = img;
        if (!string.IsNullOrEmpty(label))
            AddLabel(go.transform, label, 18);
        if (impl != null && onClick != null)
            UnityEventTools.AddPersistentListener(btn.onClick, onClick);
        return btn;
    }

    private static Toggle AddToggle(Transform parent, string name, float anchorY,
        string label, Component impl, UnityAction<bool> onValueChanged)
    {
        GameObject go = NewUI(name);
        go.transform.SetParent(parent, false);
        Anchor(go, new Vector2(0.5f, anchorY), new Vector2(200f, 40f));
        var img = go.AddComponent<Image>();
        img.color = ToggleBg;
        var tog = go.AddComponent<Toggle>();
        tog.targetGraphic = img;
        AddLabel(go.transform, label, 16);
        if (impl != null && onValueChanged != null)
            UnityEventTools.AddPersistentListener(tog.onValueChanged, onValueChanged);
        return tog;
    }

    private static RectTransform AddColumn(Transform parent, string name, Vector2 min, Vector2 max)
    {
        GameObject go = NewUI(name);
        go.transform.SetParent(parent, false);
        var rt = (RectTransform)go.transform;
        rt.anchorMin = min;
        rt.anchorMax = max;
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;
        var lg = go.AddComponent<VerticalLayoutGroup>();
        lg.padding = new RectOffset(6, 6, 6, 6);
        lg.spacing = 10f;
        lg.childAlignment = TextAnchor.UpperCenter;
        lg.childControlWidth = true;
        lg.childControlHeight = false;
        lg.childForceExpandWidth = true;
        lg.childForceExpandHeight = false;
        return rt;
    }

    // Screen root: stretched RectTransform + Canvas/Scaler/Raycaster + full-screen Panel image.
    //
    // The opaque background is correct for a menu screen, which is the whole point of it. It
    // is wrong for an in-game HUD: a Screen Space Overlay canvas draws *after* the 3D, so an
    // opaque full-screen panel hides the driving entirely. That is not a subtle dimming — the
    // 3D stops being visible at all, while the camera underneath keeps rendering perfectly
    // and can still be captured to a render texture. Any screen that overlays gameplay must
    // pass opaqueBackground: false.
    private static GameObject ScreenRoot(string name, bool opaqueBackground = true)
    {
        GameObject go = NewUI(name);
        var rt = (RectTransform)go.transform;
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;
        var canvas = go.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        var scaler = go.AddComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);
        scaler.matchWidthOrHeight = 0.5f;
        go.AddComponent<GraphicRaycaster>();
        if (opaqueBackground)
        {
            var bg = go.AddComponent<Image>();
            bg.color = PanelBg;
        }
        return go;
    }

    private static GameObject AddIndicator(Transform parent)
    {
        GameObject go = NewUI("SelectedIndicator");
        go.transform.SetParent(parent, false);
        Anchor(go, new Vector2(0f, 0.5f), new Vector2(20f, 20f));
        go.AddComponent<Image>().color = Green;
        go.SetActive(false);
        return go;
    }

    // Card root: fixed height (width driven by the column's VerticalLayoutGroup), dark panel bg.
    private static GameObject CardRoot(string name)
    {
        GameObject go = NewUI(name);
        ((RectTransform)go.transform).sizeDelta = new Vector2(0f, 180f);
        go.AddComponent<Image>().color = CardBg;
        return go;
    }

    // --- Screen builders ---

    private static void BuildBranding()
    {
        GameObject go = ScreenRoot("BrandingScreen_Prefab");
        var impl = go.AddComponent<BrandingScreenImpl>();

        var title = AddText(go.transform, "TitleText", new Vector2(0.5f, 0.68f), new Vector2(600f, 120f),
            "F1", 72, new Color(0.96f, 0.98f, 1f, 1f), FontStyles.Bold);
        var sub = AddText(go.transform, "SubtitleText", new Vector2(0.5f, 0.55f), new Vector2(600f, 50f),
            "Select your race", 24, new Color(0.72f, 0.84f, 0.94f, 0.96f));

        // GDD: "Game Branding — Display game/app name. Developed by Pearl-Lemon."
        // The attribution was previously missing from the game entirely.
        var attribution = AddText(go.transform, "AttributionText", new Vector2(0.5f, 0.16f), new Vector2(600f, 40f),
            "Developed by Pearl-Lemon", 18, new Color(0.62f, 0.72f, 0.84f, 0.9f));

        // --- Loading progress ---
        var progressRoot = NewUI("LoadingProgress");
        progressRoot.transform.SetParent(go.transform, false);
        Anchor(progressRoot, new Vector2(0.5f, 0.38f), new Vector2(520f, 30f));
        var progressRect = (RectTransform)progressRoot.transform;

        var track = NewUI("Track");
        track.transform.SetParent(progressRoot.transform, false);
        var trackRt = (RectTransform)track.transform;
        trackRt.anchorMin = Vector2.zero;
        trackRt.anchorMax = new Vector2(1f, 0.35f);
        trackRt.offsetMin = Vector2.zero;
        trackRt.offsetMax = Vector2.zero;
        var trackImg = track.AddComponent<Image>();
        trackImg.color = new Color(0.12f, 0.14f, 0.18f, 1f);

        var fill = NewUI("Fill");
        fill.transform.SetParent(progressRoot.transform, false);
        var fillRt = (RectTransform)fill.transform;
        fillRt.anchorMin = Vector2.zero;
        fillRt.anchorMax = new Vector2(1f, 0.35f);
        fillRt.offsetMin = Vector2.zero;
        fillRt.offsetMax = Vector2.zero;
        var fillImg = fill.AddComponent<Image>();
        fillImg.color = new Color(0.3f, 1f, 0.3f, 1f);
        fillImg.type = Image.Type.Filled;
        fillImg.fillMethod = Image.FillMethod.Horizontal;
        fillImg.fillOrigin = (int)Image.OriginHorizontal.Left;
        fillImg.fillAmount = 0f;

        var progressLabel = AddText(progressRoot.transform, "ProgressText", new Vector2(0.5f, 0.7f),
            new Vector2(520f, 30f), "Loading", 16, new Color(0.72f, 0.84f, 0.94f, 0.96f));

        var errorLabel = AddText(go.transform, "ErrorText", new Vector2(0.5f, 0.30f), new Vector2(700f, 50f),
            "", 18, new Color(1f, 0.42f, 0.42f, 1f));
        errorLabel.text = "";

        SetRefs(impl,
            ("_titleText", title),
            ("_subtitleText", sub),
            ("_attributionText", attribution),
            ("_progressRoot", progressRoot),
            ("_progressFill", fillImg),
            ("_progressText", progressLabel),
            ("_errorText", errorLabel));
        Save("BrandingScreen_Prefab", go);
    }

    private static void BuildMainMenu()
    {
        GameObject go = ScreenRoot("MainMenuScreen_Prefab");
        var impl = go.AddComponent<MainMenuScreenImpl>();
        var start = AddButton(go.transform, "StartButton", new Vector2(0.5f, 0.6f), new Vector2(200f, 50f),
            "Start", impl, impl.OnStartClicked);
        var options = AddButton(go.transform, "OptionsButton", new Vector2(0.5f, 0.48f), new Vector2(200f, 50f),
            "Options", impl, impl.OnOptionsClicked);
        var quit = AddButton(go.transform, "QuitButton", new Vector2(0.5f, 0.36f), new Vector2(200f, 50f),
            "Quit", impl, impl.OnQuitClicked);
        SetRefs(impl, ("_startButton", start), ("_optionsButton", options), ("_quitButton", quit));
        Save("MainMenuScreen_Prefab", go);
    }

    private static void BuildCarSelection(GameObject cardAsset)
    {
        GameObject go = ScreenRoot("CarSelectionScreen_Prefab");
        var impl = go.AddComponent<CarSelectionScreenImpl>();
        AddText(go.transform, "TitleText", new Vector2(0.5f, 0.9f), new Vector2(600f, 70f),
            "Select Your Car", 36, Color.white);
        var owned = AddColumn(go.transform, "OwnedColumn", new Vector2(0.05f, 0.05f), new Vector2(0.35f, 0.85f));
        var rentable = AddColumn(go.transform, "RentableColumn", new Vector2(0.35f, 0.05f), new Vector2(0.65f, 0.85f));
        var locked = AddColumn(go.transform, "LockedColumn", new Vector2(0.65f, 0.05f), new Vector2(0.95f, 0.85f));
        SetRefs(impl,
            ("_ownedColumn", owned),
            ("_rentableColumn", rentable),
            ("_lockedColumn", locked),
            ("_carCardPrefab", cardAsset.GetComponent<CarCardImpl>()));
        Save("CarSelectionScreen_Prefab", go);
    }

    private static void BuildTrackSelection(GameObject cardAsset)
    {
        GameObject go = ScreenRoot("TrackSelectionScreen_Prefab");
        var impl = go.AddComponent<TrackSelectionScreenImpl>();
        AddText(go.transform, "TitleText", new Vector2(0.5f, 0.9f), new Vector2(600f, 70f),
            "Select Track", 36, Color.white);
        var owned = AddColumn(go.transform, "OwnedColumn", new Vector2(0.1f, 0.05f), new Vector2(0.4f, 0.85f));
        var locked = AddColumn(go.transform, "LockedColumn", new Vector2(0.6f, 0.05f), new Vector2(0.9f, 0.85f));
        SetRefs(impl,
            ("_ownedColumn", owned),
            ("_lockedColumn", locked),
            ("_trackCardPrefab", cardAsset.GetComponent<TrackCardImpl>()));
        Save("TrackSelectionScreen_Prefab", go);
    }

    private static void BuildWingSetup()
    {
        GameObject go = ScreenRoot("WingSetupScreen_Prefab");
        var impl = go.AddComponent<WingSetupScreenImpl>();
        var title = AddText(go.transform, "TitleText", new Vector2(0.5f, 0.8f), new Vector2(600f, 70f),
            "Wing Setup", 36, Color.white);
        var rec = AddText(go.transform, "RecommendationText", new Vector2(0.5f, 0.68f), new Vector2(700f, 40f),
            "Select wing configuration", 18, new Color(0.5f, 0.7f, 1f, 1f));
        var high = AddToggle(go.transform, "HighDownforceToggle", 0.5f, "High Downforce", impl, impl.OnHighDownforceToggled);
        var low = AddToggle(go.transform, "LowDownforceToggle", 0.4f, "Low Downforce", impl, impl.OnLowDownforceToggled);
        var cont = AddButton(go.transform, "ContinueButton", new Vector2(0.7f, 0.2f), new Vector2(200f, 50f),
            "Continue", impl, impl.OnContinueClicked);
        var back = AddButton(go.transform, "BackButton", new Vector2(0.3f, 0.2f), new Vector2(200f, 50f),
            "Back", impl, impl.OnBackClicked);
        SetRefs(impl,
            ("_titleText", title),
            ("_recommendationText", rec),
            ("_highDownforceToggle", high),
            ("_lowDownforceToggle", low),
            ("_continueButton", cont),
            ("_backButton", back));
        Save("WingSetupScreen_Prefab", go);
    }

    private static void BuildModeSelection()
    {
        GameObject go = ScreenRoot("ModeSelectionScreen_Prefab");
        var impl = go.AddComponent<ModeSelectionScreenImpl>();
        var title = AddText(go.transform, "TitleText", new Vector2(0.5f, 0.75f), new Vector2(600f, 70f),
            "Mode Selection", 36, Color.white);
        var quali = AddButton(go.transform, "QualifyingButton", new Vector2(0.5f, 0.5f), new Vector2(200f, 50f),
            "Qualifying", impl, impl.OnQualifyingClicked);
        var race = AddButton(go.transform, "RaceButton", new Vector2(0.5f, 0.38f), new Vector2(200f, 50f),
            "Race", impl, impl.OnRaceClicked);
        var back = AddButton(go.transform, "BackButton", new Vector2(0.5f, 0.2f), new Vector2(200f, 50f),
            "Back", impl, impl.OnBackClicked);
        SetRefs(impl,
            ("_titleText", title),
            ("_qualifyingButton", quali),
            ("_raceButton", race),
            ("_backButton", back));
        Save("ModeSelectionScreen_Prefab", go);
    }

    /// <summary>
    /// The qualifying HUD. This is the first screen in the project that draws live gameplay
    /// values rather than menu state, so it is laid out as an overlay on the driving view:
    /// timing top-left, the track name top-centre, actions along the bottom, leaving the
    /// middle of the screen clear.
    ///
    /// The ghost toggle and the three sector readouts are deliberately not built. Both are
    /// real client requirements and both are deferred past the slice — the ghost until after
    /// it, sector timing with it — and a control that renders but does nothing is worse than
    /// an absent one in a demo shown to a client. <c>QualifyingScreenImpl</c> null-checks
    /// these fields, so wiring them later needs no code change here, only a rebuild.
    /// </summary>
    private static void BuildQualifying()
    {
        // Transparent: this screen is an overlay on the driving, not a menu in front of it.
        GameObject go = ScreenRoot("QualifyingScreen_Prefab", opaqueBackground: false);
        var impl = go.AddComponent<QualifyingScreenImpl>();

        // --- Driving state: the clock, and nothing to click ---
        var trackName = AddText(go.transform, "TrackNameText",
            new Vector2(0.5f, 0.93f), new Vector2(700f, 44f),
            "Track", 24, Color.white, FontStyles.Bold);

        var lapTime = AddText(go.transform, "CurrentLapTimeText",
            new Vector2(0.5f, 0.86f), new Vector2(420f, 64f),
            "0:00.000", 44, new Color(0.96f, 0.98f, 1f, 1f), FontStyles.Bold);

        var best = AddText(go.transform, "BestLapText",
            new Vector2(0.5f, 0.80f), new Vector2(420f, 34f),
            "Best: --:--.---", 20, new Color(0.72f, 0.84f, 0.94f, 0.96f));

        // --- Results state: the recorded lap and the three options ---
        // The options live inside this panel rather than at the screen root so that showing
        // the panel is the same act as making them reachable: there is no way to end up
        // with "Go to Race" visible on a screen that is not offering it.
        GameObject panel = NewUI("ResultsPanel");
        panel.transform.SetParent(go.transform, false);
        Anchor(panel, new Vector2(0.5f, 0.5f), new Vector2(900f, 430f));
        panel.AddComponent<Image>().color = new Color(0.04f, 0.05f, 0.08f, 0.96f);

        AddText(panel.transform, "ResultsTitleText",
            new Vector2(0.5f, 0.84f), new Vector2(700f, 44f),
            "QUALIFYING COMPLETE", 26, Color.white, FontStyles.Bold);

        AddText(panel.transform, "ResultsCaptionText",
            new Vector2(0.5f, 0.66f), new Vector2(700f, 30f),
            "Lap time", 18, new Color(0.72f, 0.84f, 0.94f, 0.96f));

        var resultTime = AddText(panel.transform, "ResultLapTimeText",
            new Vector2(0.5f, 0.49f), new Vector2(700f, 84f),
            "0:00.000", 56, new Color(0.96f, 0.98f, 1f, 1f), FontStyles.Bold);

        // Order is deliberate: Back, Retry, Go to Race.
        var back = AddButton(panel.transform, "BackButton", new Vector2(0.22f, 0.16f),
            new Vector2(200f, 54f), "Back", impl, impl.OnBackClicked);
        var restart = AddButton(panel.transform, "RetryButton", new Vector2(0.5f, 0.16f),
            new Vector2(200f, 54f), "Retry", impl, impl.OnRestartClicked);
        var race = AddButton(panel.transform, "GoToRaceButton", new Vector2(0.78f, 0.16f),
            new Vector2(240f, 54f), "Go to Race", impl, impl.OnRaceClicked);

        // Inactive at build time so the prefab opens in the driving state, matching the
        // first attempt. ApplyState sets it explicitly from then on.
        panel.SetActive(false);

        SetRefs(impl,
            ("_trackNameText", trackName),
            ("_currentLapTimeText", lapTime),
            ("_bestLapText", best),
            ("_resultsPanel", panel),
            ("_resultLapTimeText", resultTime),
            ("_backButton", back),
            ("_restartButton", restart),
            ("_raceButton", race));
        Save("QualifyingScreen_Prefab", go);
    }

    // --- Card builders ---
    // Card buttons self-wire in Awake (impl adds its own onClick listener) => no persistent listener here.

    private static GameObject BuildCarCard()
    {
        GameObject go = CardRoot("CarCard_Prefab");
        var impl = go.AddComponent<CarCardImpl>();
        var name = AddText(go.transform, "NameText", new Vector2(0.5f, 0.7f), new Vector2(300f, 36f),
            "Car Name", 18, Color.white);
        var gen = AddText(go.transform, "GenText", new Vector2(0.5f, 0.55f), new Vector2(300f, 30f),
            "Gen 1", 14, Color.white);
        var cost = AddText(go.transform, "CostText", new Vector2(0.5f, 0.42f), new Vector2(300f, 30f),
            "FREE", 14, Color.white);
        var status = AddText(go.transform, "StatusText", new Vector2(0.5f, 0.3f), new Vector2(300f, 26f),
            "OWNED", 12, Green);
        var btn = AddButton(go.transform, "CardButton", new Vector2(0.5f, 0.1f), new Vector2(200f, 40f),
            "Select", null, null);
        GameObject ind = AddIndicator(go.transform);
        SetRefs(impl,
            ("_nameText", name),
            ("_genText", gen),
            ("_costText", cost),
            ("_statusText", status),
            ("_cardButton", btn),
            ("_selectedIndicator", ind));
        return Save("CarCard_Prefab", go);
    }

    private static GameObject BuildTrackCard()
    {
        GameObject go = CardRoot("TrackCard_Prefab");
        var impl = go.AddComponent<TrackCardImpl>();
        var name = AddText(go.transform, "NameText", new Vector2(0.5f, 0.7f), new Vector2(300f, 36f),
            "Track Name", 18, Color.white);
        var code = AddText(go.transform, "ShortCodeText", new Vector2(0.5f, 0.58f), new Vector2(300f, 30f),
            "TRK", 14, new Color(0.7f, 0.8f, 1f, 1f));
        var cost = AddText(go.transform, "CostText", new Vector2(0.5f, 0.46f), new Vector2(300f, 30f),
            "FREE", 14, Color.white);
        var status = AddText(go.transform, "StatusText", new Vector2(0.5f, 0.35f), new Vector2(300f, 26f),
            "AVAILABLE", 12, Green);
        var btn = AddButton(go.transform, "CardButton", new Vector2(0.5f, 0.12f), new Vector2(200f, 40f),
            "Select", null, null);
        GameObject ind = AddIndicator(go.transform);
        SetRefs(impl,
            ("_nameText", name),
            ("_shortCodeText", code),
            ("_costText", cost),
            ("_statusText", status),
            ("_cardButton", btn),
            ("_selectedIndicator", ind));
        return Save("TrackCard_Prefab", go);
    }
}
