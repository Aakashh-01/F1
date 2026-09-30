// Assets/Editor/WingScreenUiBuilder.cs
//
// Builds the wing screen's own UI: the wing tile, the tile strip on the screen, and the
// button wiring — plus the camera framing that keeps the car clear of the strip.
//
// Why this is a separate builder and not another Tools/BuildUIPrefabs run
// ----------------------------------------------------------------------
// UIBuilder.BuildAll rebuilds all nine prefabs from scratch, and three of them have since
// been hand-finished by other passes — the track screen in particular was re-laid out as a
// centred hero with flanking columns by ClientDemoUiBuilder.BuildTrackSelectionLayout.
// Running BuildAll would silently throw that away. This file therefore only ever touches
// WingCard_Prefab and WingSetupScreen_Prefab, and edits the screen prefab in place rather
// than regenerating it.
//
// The wiring happens in a second pass, against the saved prefab, for the reason recorded in
// LobbyBuilder: UnityEventTools only writes a persistent listener when the target is
// already a prefab asset, so listeners added to a throwaway scene object before the first
// save produce a prefab whose buttons are silently inert.
//
// Run via Tools/Wing/Build Wing Screen UI.
using System.IO;
using TMPro;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using F1.UI;

public class WingScreenUiBuilder
{
    private const string WingCardPath = "Assets/Prefabs/WingCard_Prefab.prefab";
    private const string WingScreenPath = "Assets/Prefabs/WingSetupScreen_Prefab.prefab";
    private const string WingScenePath = "Assets/Scenes/30_WingSetupScene.unity";

    // --- Tile geometry ---
    //
    // The car and track tiles are 250x280. This one is smaller, and deliberately so.
    //
    // Those screens are 2D over a background image, so tile height costs nothing but
    // wallpaper. This screen is a tight 3D shot of the rear wing, and a tall strip would
    // sit on top of the car. The size is the user's: their layout screenshot shows the strip
    // at 476x181, so two tiles plus 22 of spacing divide into 227 wide, and the height is
    // the strip's less a little breathing room.
    private const float TileWidth = 227f;
    private const float TileHeight = 170f;
    private const float TilePad = 8f;

    // The wing artwork is 512x279 (about 1.84:1). A 211x88 panel is 2.4:1, so preserveAspect
    // letterboxes it a little rather than smearing it.
    private const float ArtHeight = 88f;

    // The strip. Two tiles at 250 plus 22 of spacing, which is what actually holds them;
    // StripWidthMeasured is the rect the user's screenshot shows, and the two agree closely
    // enough that the layout group centres the pair inside it either way.
    private const float StripWidth = TileWidth * 2f + 22f;
    private const float StripHeight = TileHeight;

    // Where the strip sits, and the floor the buttons share.
    //
    // The strip was originally a bottom-centre bar, which worked while the camera was pulled
    // back far enough to leave floor under the car. The camera is now hand-placed as a close
    // side shot that fills the frame, so it moved to the RIGHT — the one region the car does
    // not reach.
    //
    // The strip's size and the two button placements below are the user's, laid out by hand
    // in the editor and captured in Assets/Scenes/Screenshot 2026-09-27 032811.png. They were
    // measured out of that screenshot rather than eyeballed: the light panels were found by
    // thresholding, identified by mean colour (#EAEAEC for BACK, #F3E1B1 for CONTINUE, a
    // pure #FFFFFF block at 1.01 fill for the strip), and converted from pixels to canvas
    // units. All three came out smaller than what this builder had produced, so the controls
    // were reduced to match.
    private const float StripCentreY = 0.44f;
    private const float StripRightMargin = 79f;
    private const float StripWidthMeasured = 476f;
    private const float StripHeightMeasured = 181f;
    private const float StripBottom = 68f;

    // The palette and sprite paths live in SlimUiSkin, shared with the car, track and
    // loading screen builders. They were local constants here until the SlimUI art landed;
    // keeping a second copy per builder is how four screens end up four different greys.

    [MenuItem("Tools/Wing/Build Wing Screen UI", priority = 60)]
    public static void BuildAll()
    {
        SlimUiSkin.EnsureLoaded();
        BuildWingCardPrefab();
        BuildWingScreenLayout();
        WireWingScreenClicks();
        AssetDatabase.SaveAssets();
        Debug.Log("[WingUI] Wing screen UI built: tile, strip, wiring.");
    }

    // ==================================================================================
    // The tile prefab
    // ==================================================================================

    private static void BuildWingCardPrefab()
    {
        var root = new GameObject("WingCard_Prefab",
                                  typeof(RectTransform), typeof(CanvasRenderer),
                                  typeof(Image), typeof(LayoutElement), typeof(WingCardImpl));
        var cardRt = (RectTransform)root.transform;
        cardRt.sizeDelta = new Vector2(TileWidth, TileHeight);

        var layout = root.GetComponent<LayoutElement>();
        layout.minWidth = layout.preferredWidth = TileWidth;
        layout.minHeight = layout.preferredHeight = TileHeight;
        layout.flexibleWidth = layout.flexibleHeight = 0f;

        var body = root.GetComponent<Image>();
        SlimUiSkin.ApplyFlatPanel(body, SlimUiSkin.CardFill);
        body.raycastTarget = false;

        // A faint light edge so the dark card is visible at all against a dark garage. It
        // sits at the very bottom of the draw order — see the sibling note at the end.
        var bodyFrame = NewUI("CardFrame", cardRt, false);
        Stretch(bodyFrame);
        SlimUiSkin.ApplyPanelOverlay(bodyFrame.GetComponent<Image>(), SlimUiSkin.CardFrameOverlay);

        // The whole tile is the button, and it goes low so the artwork and the labels draw
        // on top of it. Everything above sets raycastTarget = false so the press still lands
        // on the button rather than being swallowed by a label.
        //
        // The tile's press target carries no sprite on purpose — the tile already has a panel
        // frame around it, and a second frame nested inside the first reads as clutter. This
        // is a hit area, and its only job is to lighten very slightly when held, which on a
        // dark card is the direction that reads as "pressed".
        var buttonRt = NewUI("CardButton", cardRt, true);
        var button = buttonRt.gameObject.AddComponent<Button>();
        var buttonImage = buttonRt.GetComponent<Image>();
        buttonImage.color = new Color(0f, 0f, 0f, 0.05f);
        button.targetGraphic = buttonImage;
        button.transition = Selectable.Transition.ColorTint;
        var colors = button.colors;
        colors.normalColor = Color.white;
        colors.highlightedColor = new Color(1.6f, 1.6f, 1.7f, 1f);
        colors.pressedColor = new Color(2.2f, 2.2f, 2.4f, 1f);
        colors.selectedColor = Color.white;
        colors.disabledColor = new Color(1f, 1f, 1f, 0.5f);
        colors.fadeDuration = 0.06f;
        button.colors = colors;

        // A near-black well behind the picture. The wing art carries its own near-black
        // backdrop, so on the old pale tile it was a dark rectangle dropped into a bright
        // panel; against a matching well the backdrop runs off the edge and the wing floats.
        var artBacking = NewUI("WingArtBacking", cardRt, false);
        AnchorFromTop(artBacking, TilePad, TilePad, ArtHeight);
        SlimUiSkin.ApplyFlatPanel(artBacking.GetComponent<Image>(), SlimUiSkin.ArtWellFill);

        var art = NewUI("WingArt", cardRt, false);
        var artImage = art.GetComponent<Image>();
        artImage.type = Image.Type.Simple;
        artImage.preserveAspect = true;
        artImage.color = Color.white;
        AnchorFromTop(art, TilePad, TilePad, ArtHeight);

        // Light ink on the dark tile. The title was the pack's amber, which is unreadable at
        // this size on a pale panel — amber is kept for the selected border instead, where it
        // is a graphic and not something anyone has to read.
        //
        // The blurb is 14px in a 28-unit band, which is two lines. At 13 in a 22 band it was
        // one cramped line and effectively unreadable, and on a 227-wide tile this is the
        // only place the consequence of the choice is written down at all.
        var title = NewText("TitleText", cardRt, "HIGH DOWNFORCE", 19f, SlimUiSkin.TileInk, FontStyles.Bold);
        AnchorFromBottom(title, 40f, TileWidth - TilePad * 2f, 22f);

        var blurb = NewText("BlurbText", cardRt, "More grip, slower on the straights", 14f, SlimUiSkin.TileInkMuted);
        AnchorFromBottom(blurb, 8f, TileWidth - TilePad * 2f, 28f);

        // Named SelectedIndicator to match CarCardImpl, so the selected-state pattern the
        // other screens already use carries over.
        //
        // It is the pack's sliced frame, not uGUI's Outline component. Outline draws four
        // offset COPIES of the whole graphic, so on a full-card rect it covers the entire
        // card as a solid block; making the graphic transparent to open the centre up then
        // multiplies the border's alpha by that transparency and it disappears instead.
        // Both failures look correct in the inspector. A sliced frame sprite has an empty
        // middle, so it is a genuine border. See SlimUiSkin.ApplySelectedBorder.
        var indicator = NewUI("SelectedIndicator", cardRt, false);
        SlimUiSkin.ApplySliced(indicator.GetComponent<Image>(),
                               SlimUiSkin.ButtonFramePath, SlimUiSkin.SelectedBorder);
        indicator.GetComponent<Image>().raycastTarget = false;
        indicator.SetAsLastSibling();
        indicator.gameObject.SetActive(false);

        // Draw order, bottom to top: CardFrame (a faint edge so the dark card reads against
        // a dark garage), CardButton (hit area), WingArtBacking (the near-black well),
        // WingArt (the picture, washed by nothing), then the labels and the selected border.
        // A translucent frame drawn OVER the artwork is the haze this palette removes, so
        // the order is pinned here rather than left to the order things were created in.
        bodyFrame.SetSiblingIndex(0);
        buttonRt.SetSiblingIndex(1);
        artBacking.SetSiblingIndex(2);
        art.SetSiblingIndex(3);

        var impl = root.GetComponent<WingCardImpl>();
        Wire(impl, "_titleText", title);
        Wire(impl, "_blurbText", blurb);
        Wire(impl, "_wingImage", artImage);
        Wire(impl, "_cardButton", button);
        Wire(impl, "_selectedIndicator", indicator.gameObject);

        EnsureFolder("Assets/Prefabs");
        PrefabUtility.SaveAsPrefabAsset(root, WingCardPath);
        Object.DestroyImmediate(root);
    }

    // ==================================================================================
    // The screen prefab
    // ==================================================================================

    private static void BuildWingScreenLayout()
    {
        if (!File.Exists(WingScreenPath))
        {
            Debug.LogError($"[WingUI] Screen prefab missing at {WingScreenPath}.");
            return;
        }

        var root = PrefabUtility.LoadPrefabContents(WingScreenPath);
        try
        {
            var impl = FindImpl(root, "WingSetupScreenImpl");
            if (impl == null)
            {
                Debug.LogError($"[WingUI] WingSetupScreenImpl not found in {WingScreenPath}.");
                return;
            }

            // The two bare toggles are what this whole pass exists to replace: 200x40
            // controls sitting on top of the car, with no artwork and no way to tell which
            // setup is live beyond a tick. Deleted rather than hidden, so the impl's
            // removed toggle fields cannot keep a stale reference alive in the prefab.
            foreach (var dead in new[] { "HighDownforceToggle", "LowDownforceToggle" })
            {
                var existing = root.transform.Find(dead);
                if (existing != null) Object.DestroyImmediate(existing.gameObject);
            }

            // --- Tile strip, right of the car ---
            var strip = FindOrCreateUI("TileStrip", (RectTransform)root.transform);
            strip.anchorMin = new Vector2(1f, StripCentreY);
            strip.anchorMax = new Vector2(1f, StripCentreY);
            strip.pivot = new Vector2(1f, 0.5f);
            strip.anchoredPosition = new Vector2(-StripRightMargin, 0f);
            strip.sizeDelta = new Vector2(StripWidthMeasured, StripHeightMeasured);

            var group = strip.gameObject.GetComponent<HorizontalLayoutGroup>()
                        ?? strip.gameObject.AddComponent<HorizontalLayoutGroup>();
            group.spacing = 22f;
            group.childAlignment = TextAnchor.MiddleCenter;
            // The tile carries an explicit sizeDelta; letting the group control either axis
            // would override it and the tiles would drift off the 250x200 set above.
            group.childControlWidth = false;
            group.childControlHeight = false;
            group.childForceExpandWidth = false;
            group.childForceExpandHeight = false;

            // --- Title and recommendation, clear of the car ---
            var title = root.transform.Find("TitleText") as RectTransform;
            if (title != null)
            {
                title.anchorMin = title.anchorMax = new Vector2(0.5f, 1f);
                title.pivot = new Vector2(0.5f, 1f);
                title.anchoredPosition = new Vector2(0f, -46f);
                title.sizeDelta = new Vector2(1000f, 64f);
                var t = title.GetComponent<TMP_Text>();
                if (t != null) { t.fontSize = 40f; t.raycastTarget = false; }
            }

            var rec = root.transform.Find("RecommendationText") as RectTransform;
            if (rec != null)
            {
                rec.anchorMin = rec.anchorMax = new Vector2(0.5f, 1f);
                rec.pivot = new Vector2(0.5f, 1f);
                rec.anchoredPosition = new Vector2(0f, -112f);
                rec.sizeDelta = new Vector2(1100f, 38f);
                var t = rec.GetComponent<TMP_Text>();
                if (t != null) { t.fontSize = 22f; t.raycastTarget = false; }
            }

            // --- Back and Continue, in the bottom corners ---
            // Same arrangement as the lobby's selection panel, so the strip and the buttons
            // around it are learned once rather than twice.
            var back = root.transform.Find("BackButton") as RectTransform;
            if (back != null)
            {
                // Top-left in the user's layout. It was bottom-left, which in the close side
                // shot put it across the car's front wing.
                back.anchorMin = back.anchorMax = new Vector2(0f, 1f);
                back.pivot = new Vector2(0f, 1f);
                back.anchoredPosition = new Vector2(80f, -61f);
                back.sizeDelta = new Vector2(200f, 62f);
                StyleButton(back, "BACK", 24f, SlimUiSkin.Ink);
                SlimUiSkin.ApplySecondaryButton(back.GetComponent<Button>());
            }

            var cont = root.transform.Find("ContinueButton") as RectTransform;
            if (cont != null)
            {
                cont.anchorMin = cont.anchorMax = new Vector2(1f, 0f);
                cont.pivot = new Vector2(1f, 0f);
                cont.anchoredPosition = new Vector2(-79f, 97f);
                cont.sizeDelta = new Vector2(233f, 73f);
                StyleButton(cont, "CONTINUE", 27f, SlimUiSkin.Ink);
                SlimUiSkin.ApplyPrimaryButton(cont.GetComponent<Button>());
            }

            // --- Wire the impl ---
            Wire(impl, "_titleText", title != null ? title.GetComponent<TMP_Text>() : null);
            Wire(impl, "_recommendationText", rec != null ? rec.GetComponent<TMP_Text>() : null);
            Wire(impl, "_tileStrip", strip);
            Wire(impl, "_continueButton", cont != null ? cont.GetComponent<Button>() : null);
            Wire(impl, "_backButton", back != null ? back.GetComponent<Button>() : null);

            var cardPrefab = AssetDatabase.LoadAssetAtPath<WingCardImpl>(WingCardPath);
            if (cardPrefab == null)
                Debug.LogError($"[WingUI] {WingCardPath} not found; the strip will stay empty.");
            Wire(impl, "_wingCardPrefab", cardPrefab);

            PrefabUtility.SaveAsPrefabAsset(root, WingScreenPath);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    /// <summary>
    /// Reopens the saved screen prefab and attaches the Continue and Back handlers.
    ///
    /// Separate from the layout pass on purpose. UnityEventTools only writes a persistent
    /// listener when the target is already a prefab asset, and a freshly created scene
    /// object is not one — wiring here, after the prefab exists on disk, is what makes the
    /// references stick instead of producing a prefab whose buttons do nothing.
    /// </summary>
    private static void WireWingScreenClicks()
    {
        var contents = PrefabUtility.LoadPrefabContents(WingScreenPath);
        try
        {
            var impl = contents.GetComponent<WingSetupScreenImpl>();
            if (impl == null)
            {
                Debug.LogError("[WingUI] Saved screen prefab has no WingSetupScreenImpl; " +
                               "cannot wire buttons.");
                return;
            }

            Click(contents.transform, "ContinueButton", impl, nameof(WingSetupScreenImpl.OnContinueClicked));
            Click(contents.transform, "BackButton", impl, nameof(WingSetupScreenImpl.OnBackClicked));

            PrefabUtility.SaveAsPrefabAsset(contents, WingScreenPath);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(contents);
        }
    }

    // ==================================================================================
    // Camera framing — HAND-OWNED, DO NOT OVERWRITE
    // ==================================================================================

    /// <summary>
    /// The live wing screen camera is HAND-PLACED and belongs to the user.
    ///
    /// It was hand-tuned on 2026-09-27, after the measured pose below was applied. The
    /// values here are kept only as a record of where the framing came from and so the
    /// measurements below stay interpretable — they are NOT what the camera is set to, and
    /// the numbers are deliberately NOT synchronised with the scene, because syncing them
    /// is exactly the step that would overwrite the tuning on the next run.
    ///
    /// <see cref="AimWingCamera"/> is the only thing in this file that writes the camera,
    /// it is a separate menu item from <see cref="BuildAll"/>, and it now asks before it
    /// does anything. BuildAll — the one normally used to rebuild the tile and the screen
    /// layout — does not touch the camera at all, so it is safe to re-run freely.
    /// </summary>
    private static readonly Vector3 CameraPosition = new Vector3(0.57f, 2.00f, -3.98f);
    private static readonly Vector3 CameraLookAt = new Vector3(-0.04f, -0.45f, 0.38f);
    private const float CameraFov = 62f;

    /// <summary>
    /// Restores the MEASURED camera pose below, over whatever is in the scene.
    ///
    /// Destructive to hand-tuning by design, and gated on an explicit confirmation. The
    /// wing scene's camera is hand-owned; the values this writes are a record of the
    /// measured framing, not the current one, so running it undoes the user's work. It is
    /// kept — not deleted — because the measurements that produced it are the only record
    /// of why the framing is constrained the way it is, and reproducing them from scratch
    /// costs a great deal more than keeping a menu item behind a dialog.
    /// </summary>
    [MenuItem("Tools/Wing/Restore Measured Wing Camera (overwrites hand-placed camera)", priority = 61)]
    public static void AimWingCamera()
    {
        if (!EditorUtility.DisplayDialog(
                "Overwrite the hand-placed wing camera?",
                $"This replaces the LobbyCamera in {Path.GetFileName(WingScenePath)} with the "
                + "measured pose:\n\n"
                + $"  position {CameraPosition}\n"
                + $"  look at  {CameraLookAt}\n"
                + $"  fov      {CameraFov}\n\n"
                + "The camera in that scene has been hand-tuned. This cannot be undone from "
                + "here — only by re-tuning it.",
                "Overwrite", "Cancel"))
        {
            return;
        }

        var scene = EditorSceneManager.OpenScene(WingScenePath, OpenSceneMode.Single);
        var camera = FindInScene(scene, "LobbyCamera");
        if (camera == null)
        {
            Debug.LogError($"[WingUI] No LobbyCamera in {WingScenePath}.");
            return;
        }

        camera.transform.position = CameraPosition;
        camera.transform.rotation = Quaternion.LookRotation((CameraLookAt - CameraPosition).normalized, Vector3.up);
        var cam = camera.GetComponent<Camera>();
        if (cam != null) cam.fieldOfView = CameraFov;

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Debug.Log($"[WingUI] Wing camera restored to the measured pose {CameraPosition} / {CameraLookAt} / fov {CameraFov}.");
    }

    private static GameObject FindInScene(Scene scene, string name)
    {
        foreach (var root in scene.GetRootGameObjects())
        {
            if (root.name == name) return root;
            var found = root.transform.Find(name);
            if (found != null) return found.gameObject;
        }
        return null;
    }

    // ==================================================================================
    // Helpers
    // ==================================================================================

    private static RectTransform NewUI(string name, RectTransform parent, bool isButton)
    {
        var go = new GameObject(name, typeof(RectTransform), typeof(CanvasRenderer),
                                typeof(Image));
        var rt = (RectTransform)go.transform;
        rt.SetParent(parent, false);
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;

        var image = go.GetComponent<Image>();
        image.raycastTarget = isButton;
        return rt;
    }

    private static TMP_Text NewText(string name, RectTransform parent, string content,
                                   float fontSize, Color color, FontStyles style = FontStyles.Normal)
    {
        var go = new GameObject(name, typeof(RectTransform), typeof(CanvasRenderer),
                                typeof(TextMeshProUGUI));
        var rt = (RectTransform)go.transform;
        rt.SetParent(parent, false);
        var tmp = go.GetComponent<TextMeshProUGUI>();
        tmp.text = content;
        tmp.fontSize = fontSize;
        tmp.color = color;
        tmp.fontStyle = style;
        tmp.alignment = TextAlignmentOptions.Center;
        tmp.textWrappingMode = TextWrappingModes.Normal;
        // The tile's own button sits underneath; a raycast-hungry label would eat presses
        // in the middle of the tile and leave dead zones.
        tmp.raycastTarget = false;
        return tmp;
    }

    /// <summary>
    /// The rect of a component we just created. Takes a Component rather than a RectTransform
    /// because the callers hold the Graphic or the TMP_Text (the thing they need to wire a
    /// field to), not the transform, and casting twice at every call site is worse than
    /// casting once here.
    /// </summary>
    private static RectTransform RectOf(Component component) =>
        component == null ? null : component.transform as RectTransform;

    /// <summary>
    /// Stretches a rect across the top of its parent, inset left/right and `fromTop` down
    /// from the top edge. The height is not a parameter because a stretched rect's height
    /// comes from the two offsets; passing one in and not applying it would be a lie.
    /// </summary>
    private static void AnchorFromTop(Component component, float inset, float fromTop, float height)
    {
        var rt = RectOf(component);
        if (rt == null) return;
        rt.anchorMin = new Vector2(0f, 1f);
        rt.anchorMax = new Vector2(1f, 1f);
        rt.pivot = new Vector2(0.5f, 1f);
        rt.offsetMin = new Vector2(inset, -(fromTop + height));
        rt.offsetMax = new Vector2(-inset, -fromTop);
    }

    /// <summary>Anchors a rect to the bottom of its parent, centred horizontally.</summary>
    private static void AnchorFromBottom(Component component, float fromBottom, float width, float height)
    {
        var rt = RectOf(component);
        if (rt == null) return;
        rt.anchorMin = new Vector2(0.5f, 0f);
        rt.anchorMax = new Vector2(0.5f, 0f);
        rt.pivot = new Vector2(0.5f, 0f);
        rt.sizeDelta = new Vector2(width, height);
        rt.anchoredPosition = new Vector2(0f, fromBottom + height * 0.5f);
    }

    /// <summary>Fills the parent rect, so an overlay covers exactly its parent.</summary>
    private static void Stretch(RectTransform rt)
    {
        if (rt == null) return;
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.pivot = new Vector2(0.5f, 0.5f);
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;
    }

    private static RectTransform FindOrCreateUI(string name, RectTransform parent)
    {
        var existing = parent.Find(name) as RectTransform;
        if (existing != null) return existing;

        var go = new GameObject(name, typeof(RectTransform), typeof(CanvasRenderer), typeof(Image));
        var rt = (RectTransform)go.transform;
        rt.SetParent(parent, false);
        return rt;
    }

    /// <summary>
    /// Rebuilds a button's label. The Image is left alone — the caller skins it — because
    /// the sprite and the text colour have to be decided together: the pack's buttons are
    /// near-opaque light panels, so their labels take dark ink, while a title sitting
    /// straight on the dark garage takes light ink. Those are opposite colours and the
    /// difference is invisible to every test in the project.
    /// </summary>
    private static void StyleButton(RectTransform rt, string label, float fontSize, Color ink)
    {
        var image = rt.GetComponent<Image>();
        if (image != null) image.raycastTarget = true;

        var existing = rt.Find("Label");
        if (existing != null) Object.DestroyImmediate(existing.gameObject);

        var go = new GameObject("Label", typeof(RectTransform), typeof(CanvasRenderer),
                                typeof(TextMeshProUGUI));
        var labelRt = (RectTransform)go.transform;
        labelRt.SetParent(rt, false);
        labelRt.anchorMin = Vector2.zero;
        labelRt.anchorMax = Vector2.one;
        labelRt.offsetMin = Vector2.zero;
        labelRt.offsetMax = Vector2.zero;

        var tmp = go.GetComponent<TextMeshProUGUI>();
        tmp.text = label;
        tmp.fontSize = fontSize;
        tmp.color = ink;
        tmp.fontStyle = FontStyles.Bold;
        tmp.alignment = TextAlignmentOptions.Center;
        tmp.raycastTarget = false;
    }

    private static void Click(Transform root, string buttonName, Object target, string method)
    {
        var button = root.Find(buttonName)?.GetComponent<Button>();
        if (button == null)
        {
            Debug.LogError($"[WingUI] No button '{buttonName}' to wire to {method}.");
            return;
        }

        // Peels from the front until none are left, so a re-run cannot leave two listeners
        // on one button — a second Continue press would then fire the handler twice.
        while (button.onClick.GetPersistentEventCount() > 0)
            UnityEditor.Events.UnityEventTools.RemovePersistentListener(button.onClick, 0);

        var info = target.GetType().GetMethod(method,
            System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public);
        if (info == null)
        {
            Debug.LogError($"[WingUI] {target.GetType().Name}.{method} not found.");
            return;
        }

        // AddVoidPersistentListener takes a UnityAction, not (target, methodName); building
        // it from the reflected MethodInfo keeps the two in step, so a renamed handler fails
        // loudly here instead of quietly producing an unclickable button.
        var call = (UnityEngine.Events.UnityAction)System.Delegate.CreateDelegate(
            typeof(UnityEngine.Events.UnityAction), target, info);
        UnityEditor.Events.UnityEventTools.AddVoidPersistentListener(button.onClick, call);
    }

    private static void Wire(Object target, string field, Object value)
    {
        var so = new SerializedObject(target);
        var prop = so.FindProperty(field);
        if (prop == null)
        {
            Debug.LogError($"[WingUI] No field '{field}' on {target.GetType().Name}.");
            return;
        }
        prop.objectReferenceValue = value;
        so.ApplyModifiedPropertiesWithoutUndo();
    }

    private static MonoBehaviour FindImpl(GameObject root, string typeName)
    {
        foreach (var mb in root.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (mb != null && mb.GetType().Name == typeName) return mb;
        }
        return null;
    }

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
