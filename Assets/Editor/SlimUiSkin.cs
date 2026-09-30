// Assets/Editor/SlimUiSkin.cs
//
// The SlimUI "3D Modern Menu UI" art, applied to our own uGUI screens.
//
// Why this is a skin and not the pack's prefabs
// ----------------------------------------------
// The pack is a complete standalone main-menu system, not a sprite set. Its two template
// prefabs each carry ~90 MeshRenderers plus their own Camera, EventSystem,
// PostProcessVolume, AudioListener, Animator, AudioSource, and a UIMenuManager hardwired to
// named panels (mainMenu, playMenu, exitMenu, PanelControls, ...). Dropping one into a flow
// scene would put a second Camera in competition with the hand-tuned LobbyCamera, a second
// EventSystem (Unity rejects that outright), a PostProcessVolume that changes the garage's
// render, and a page router that has no relationship with GameFlowManager — and it would
// cover the garage, which is the entire point of the lobby design.
//
// So only the two-dimensional art is taken, and it is applied to the screens we already
// have. FlowSceneHost, the single EventSystem, the hand-placed cameras and the
// garage-visible-behind layout all survive untouched.
//
// What the art actually is
// ------------------------
// Greyscale and designed to be tinted. Sampled from the source PNGs:
//   Button Fram 256px        #7f7f7f   normal
//   Button Frame Hover 256px #a0a0a0   hover
//   Button Frame Press 256px #707070   pressed
// The three button states ship as separate sprites with 9-slice borders already set, so no
// importer work is needed. The pack's buttons are OUTLINES, not filled panels — that is
// why "Button Fram" reads as a hairline rectangle.
//
// FIVE of the pack's 22 sprites are advertisements for other SlimUI products, not UI art:
// CleanMenu, Cursor Controller Pro, Essence Menu and SciFi Menu (all 256x171) plus
// AssetLogo. They are deliberately absent from the paths below. Do not wire them in.
//
// Palette
// -------
// Taken from the pack's own Assets/SlimUI/Data/ThemeSettings_ModernMenuUI.asset, which
// defines three accents: amber, cyan and green. Amber is the pack's currentColor and is
// used here as the single accent.
using TMPro;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

public static class SlimUiSkin
{
    private const string G = "Assets/SlimUI/Modern Menu 1/Graphics/";

    // --- Sprites ---
    public const string ButtonFramePath = G + "Buttons/Button Fram 256px.png";
    public const string ButtonHoverPath = G + "Buttons/Button Frame Hover 256px.png";
    public const string ButtonPressPath = G + "Buttons/Button Frame Press 256px.png";
    public const string PanelFramePath = G + "Frames/Panel Frame 512px.png";
    public const string PanelFullPath = G + "Frames/Panel 1920x1080px.png";
    public const string CornerDetailPath = G + "Misc/Corner Detail 256px.png";
    public const string GearIconPath = G + "Icons/Gear 128px.png";
    public const string ArrowIconPath = G + "Icons/Arrow 128px.png";
    public const string ReturnIconPath = G + "Icons/Return 128px.png";

    // --- Palette ---
    // The pack's own accents, from ThemeSettings_ModernMenuUI.asset.
    public static readonly Color Accent = new Color(1f, 0.6862745f, 0f, 1f);      // #FFAF00
    public static readonly Color AccentCool = new Color(0f, 0.6677432f, 1f, 1f);  // #00AAFF

    /// <summary>
    /// Ink for text sitting on a light panel.
    ///
    /// This is the one thing the theme change forces and it is easy to get wrong. The UI was
    /// dark before, so every label was near-white (0.92, 0.93, 0.95). Carried over
    /// unchanged onto a light panel that text is invisible — and no test fails, because the
    /// colour is set, the string is set, and nothing checks whether either can be seen.
    /// </summary>
    public static readonly Color Ink = new Color(0.10f, 0.11f, 0.14f, 1f);        // #1A1D24

    /// <summary>Secondary labels on a LIGHT surface. About 5.6:1 against ButtonIdle.</summary>
    public static readonly Color InkMuted = new Color(0.20f, 0.22f, 0.27f, 1f);

    // --- The tiles, which are DARK ---
    //
    // The tiles went light in the first pass and that was wrong, for a reason the artwork
    // makes obvious. The car icons are a white car on a near-black studio backdrop, and the
    // wing art is the same. On a light card that dark backdrop became a hard dark rectangle
    // dropped into the middle of a pale panel: the two values fought, the card's edges read
    // as clutter around a hole, and the whole tile came out looking washed and blurry
    // rather than crisp. A near-black art well lets the artwork's own background run off
    // the edge of the well, so the subject floats instead of sitting in a box.
    //
    // The body is a deep indigo rather than neutral grey so the card has some colour in it
    // — that is where the "vibrant" comes from, since the artwork itself is fixed and
    // cannot be saturated any further without lying about how the car is painted.

    /// <summary>Card body: deep, slightly blue. Vibrant without competing with the art.</summary>
    public static readonly Color CardFill = new Color(0.105f, 0.125f, 0.205f, 1f);

    /// <summary>
    /// The well the artwork sits in, a shade under the body so it reads as a recess.
    /// Near-black, because the artwork's own backdrop is near-black.
    /// </summary>
    public static readonly Color ArtWellFill = new Color(0.045f, 0.050f, 0.072f, 1f);

    /// <summary>Title on a dark tile.</summary>
    public static readonly Color TileInk = new Color(0.95f, 0.96f, 1.00f, 1f);

    /// <summary>Secondary labels on a dark tile. About 7:1 against CardFill.</summary>
    public static readonly Color TileInkMuted = new Color(0.62f, 0.67f, 0.78f, 1f);

    /// <summary>
    /// How much of the pack's translucent panel art is laid over a card body.
    ///
    /// Kept low because it is a LIGHT overlay. At the 0.30 it started on, laid over a dark
    /// card it greys the body straight back out — the exact washed look this palette
    /// exists to remove. This is only here to give a dark card a visible edge against a
    /// dark garage, not to lighten it.
    /// </summary>
    public static readonly Color CardFrameOverlay = new Color(1f, 1f, 1f, 0.16f);

    /// <summary>Panel body for light surfaces. Kept for anything that is not a tile.</summary>
    public static readonly Color PanelFill = new Color(0.66f, 0.70f, 0.75f, 1f);

    public static readonly Color PanelEdge = new Color(0.45f, 0.49f, 0.55f, 1f);

    /// <summary>
    /// Button tints. The art is greyscale, so these drive its actual colour. The buttons sit
    /// directly on the dark garage rather than on a tile, so they stay light — a dark button
    /// out there would vanish. Primary is the pack's amber knocked back from full strength.
    /// </summary>
    public static readonly Color ButtonIdle = new Color(0.72f, 0.75f, 0.80f, 0.96f);
    public static readonly Color ButtonPrimaryIdle = new Color(0.78f, 0.63f, 0.30f, 0.97f);
    public static readonly Color ButtonPrimaryHover = new Color(0.86f, 0.70f, 0.34f, 1f);
    public static readonly Color ButtonPrimaryPress = new Color(0.68f, 0.54f, 0.25f, 1f);

    /// <summary>Accent used for the selected-tile border. Amber reads on a light panel.</summary>
    public static readonly Color SelectedBorder = new Color(1f, 0.6862745f, 0f, 1f);

    // --- Loading ---

    private static Sprite _cached;

    /// <summary>
    /// Loads the skin's sprites once. Every sprite is a sub-asset of its own PNG, so a path
    /// lookup needs the type filter; a null here means the pack is not imported.
    /// </summary>
    public static void EnsureLoaded()
    {
        if (_cached != null) return;
        _cached = AssetDatabase.LoadAssetAtPath<Sprite>(ButtonFramePath);
        if (_cached == null)
            Debug.LogError($"[SlimUi] Could not load {ButtonFramePath}. The SlimUI package does " +
                           "not appear to be imported, so the UI will fall back to flat colours.");
    }

    private static Sprite Load(string path) =>
        AssetDatabase.LoadAssetAtPath<Sprite>(path);

    // --- Applying ---

    /// <summary>
    /// Puts a sliced, tinted sprite on a Graphic.
    ///
    /// Sliced rather than Simple because these are frames: stretched flat, the corners and
    /// the 1px outline smear across the whole rect. The pack's own PNGs already carry 9-slice
    /// borders, so nothing needs setting in the importer.
    /// </summary>
    public static void ApplySliced(Image image, string spritePath, Color tint)
    {
        if (image == null) return;
        var sprite = Load(spritePath);
        if (sprite == null)
        {
            Debug.LogError($"[SlimUi] Missing sprite at {spritePath}; left unchanged.");
            return;
        }

        image.sprite = sprite;
        image.type = Image.Type.Sliced;
        image.color = tint;
        // preserveAspect is meaningless on a sliced frame and fights the borders.
        image.preserveAspect = false;
    }

    /// <summary>
    /// Applies a three-state sprite to a Button, replacing the flat colour and the
    /// ColorBlock tint that stood in for one.
    /// </summary>
    public static void ApplyButton(Button button, string idlePath, Color idleTint,
                                   Color normal, Color highlighted, Color pressed)
    {
        if (button == null) return;
        ApplySliced(button.targetGraphic as Image, idlePath, idleTint);

        var hover = Load(ButtonHoverPath);
        var press = Load(ButtonPressPath);
        button.transition = Selectable.Transition.SpriteSwap;
        button.spriteState = new SpriteState
        {
            highlightedSprite = hover,
            pressedSprite = press,
            selectedSprite = hover,
            disabledSprite = Load(ButtonPressPath),
        };

        // ColourTint underneath the sprite swap, kept near-white so it only tints rather
        // than recolours. A strong multiplier here would undo the sprite's own grey.
        var colors = button.colors;
        colors.normalColor = normal;
        colors.highlightedColor = highlighted;
        colors.pressedColor = pressed;
        colors.selectedColor = normal;
        colors.disabledColor = new Color(1f, 1f, 1f, 0.45f);
        colors.fadeDuration = 0.08f;
        button.colors = colors;
    }

    /// <summary>Applies the standard primary button (the pack's amber accent).</summary>
    public static void ApplyPrimaryButton(Button button)
    {
        ApplyButtonVisual(button, true);
    }

    /// <summary>Applies the standard secondary button (neutral outline).</summary>
    public static void ApplySecondaryButton(Button button)
    {
        ApplyButtonVisual(button, false);
    }

    /// <summary>
    /// Builds a button that is actually readable over the dark garage.
    ///
    /// The pack's Button Fram sprite is an OUTLINE: sampled, it is 99% transparent pixels
    /// with a one-pixel #7f7f7f border. Tinting it does not make a fill, because a tint
    /// multiplies whatever alpha is already there — tint a mostly-transparent sprite and it
    /// stays mostly transparent, whatever colour you ask for. So a button built from the
    /// frame sprite alone is an empty rectangle showing the 3D scene through it, and dark
    /// label ink on it is dark-on-dark and effectively invisible.
    ///
    /// The fix is three layers instead of one: the button's own Image carries an opaque
    /// light FILL, a child carries the pack's outline on top of it, and the label sits above
    /// both. State changes tint the fill, because SpriteState would swap the fill's sprite
    /// and there is no second fill sprite to swap to.
    /// </summary>
    public static void ApplyButtonVisual(Button button, bool primary)
    {
        if (button == null) return;

        var fill = button.targetGraphic as Image ?? button.GetComponent<Image>();
        if (fill != null)
        {
            fill.sprite = null;
            fill.type = Image.Type.Simple;
            fill.preserveAspect = false;
            fill.color = primary ? ButtonPrimaryIdle : ButtonIdle;
            fill.raycastTarget = true;
        }

        // The outline goes on a child, because a child draws over its parent's Image and
        // an outline behind an opaque fill would never be seen.
        var frame = button.transform.Find("ButtonFrame") as RectTransform;
        if (frame == null)
        {
            var go = new UnityEngine.GameObject("ButtonFrame", typeof(RectTransform),
                                                typeof(CanvasRenderer), typeof(Image));
            frame = (RectTransform)go.transform;
            frame.SetParent(button.transform, false);
            frame.anchorMin = Vector2.zero;
            frame.anchorMax = Vector2.one;
            frame.offsetMin = Vector2.zero;
            frame.offsetMax = Vector2.zero;
        }

        var frameImage = frame.GetComponent<Image>();
        ApplySliced(frameImage, ButtonFramePath,
                    primary ? ButtonPrimaryIdle : ButtonIdle);
        frameImage.raycastTarget = false;

        // Keep the frame above the fill but below the label, whatever order the label has.
        var label = button.transform.Find("Label");
        if (label != null) frame.SetSiblingIndex(label.GetSiblingIndex());
        else frame.SetAsLastSibling();

        button.transition = Selectable.Transition.ColorTint;
        var colors = button.colors;
        colors.normalColor = Color.white;
        colors.highlightedColor = primary ? new Color(1.02f, 1.02f, 1.02f, 1f)
                                          : new Color(0.97f, 0.97f, 0.98f, 1f);
        colors.pressedColor = primary ? new Color(0.86f, 0.80f, 0.68f, 1f)
                                       : new Color(0.84f, 0.86f, 0.89f, 1f);
        colors.selectedColor = colors.normalColor;
        colors.disabledColor = new Color(1f, 1f, 1f, 0.45f);
        colors.fadeDuration = 0.08f;
        button.colors = colors;
    }

    /// <summary>
    /// Recolours a TMP label for a light panel. Near-white text on a light panel is
    /// invisible, and nothing in the build will tell you so.
    /// </summary>
    public static void ApplyInk(TMPro.TMP_Text text, bool muted = false)
    {
        if (text == null) return;
        text.color = muted ? InkMuted : Ink;
    }

    /// <summary>
    /// An OPAQUE light fill for a surface that has to carry dark ink.
    ///
    /// Deliberately not the pack's "Panel Frame 512px" sprite. That sprite is a trap: opened
    /// in an image viewer it looks like a solid light blue-grey panel, because a viewer
    /// composites transparency over white. Its actual pixels are 50% alpha mid-grey
    /// (#959595, a=0.50), measured from the texture at runtime. Tinted and used as a card
    /// body it renders at about 48% opacity in mid grey, so the dark garage shows straight
    /// through it and the dark label sitting on top loses its contrast.
    ///
    /// The panel sprite is fine for what it actually is — a translucent overlay with a border
    /// and a faint diagonal sheen — and it is used that way for the button frames. It is
    /// simply not a background to put text on.
    /// </summary>
    public static void ApplyFlatPanel(Image image, Color fill)
    {
        if (image == null) return;
        image.sprite = null;
        image.type = Image.Type.Simple;
        image.preserveAspect = false;
        image.color = fill;
    }

    /// <summary>
    /// Lays the pack's translucent panel art OVER an already-opaque fill, as a border and
    /// sheen. Separate from ApplyFlatPanel so the two can be told apart at the call site.
    /// </summary>
    public static void ApplyPanelOverlay(Image image, Color tint)
    {
        ApplySliced(image, PanelFramePath, tint);
    }

    /// <summary>Where the HUD text material lives. See <see cref="HudTextMaterial"/>.</summary>
    public const string HudTextMaterialPath = "Assets/Art/UI/HudText.mat";

    /// <summary>
    /// A TextMeshPro material with a dark underlay, so HUD text reads over live 3D.
    ///
    /// The lap clock does not sit on a panel — it floats over whatever the player is driving,
    /// so the same glyph has to stay legible against a bright sky, pale tarmac and dark
    /// scenery within a few seconds of each other. A colour alone cannot do that: white reads
    /// against the dark garage and disappears against the sky.
    ///
    /// It has to be a real material ASSET, assigned as `fontSharedMaterial`. Enabling the
    /// keyword on `text.fontMaterial` does not survive authoring a prefab, for two reasons:
    /// `fontMaterial` throws in edit mode on a prefab ("m_sharedMaterial has not been
    /// assigned"), and even where it does not, a runtime material instance is not a project
    /// asset so it is not written to disk. The first attempt at this silently did nothing —
    /// the HUD looked acceptable only because the test sky happened to be dark enough.
    /// </summary>
    public static Material HudTextMaterial(Material source)
    {
        var existing = AssetDatabase.LoadAssetAtPath<Material>(HudTextMaterialPath);
        if (existing != null) return existing;

        if (source == null)
        {
            Debug.LogError("[SlimUi] No source material for the HUD text shadow, so it cannot " +
                           "be built. Pass a TMP label's fontSharedMaterial.");
            return null;
        }

        // CreateFolder takes ONE child name under ONE existing parent — passing a nested
        // path like "Art/UI" fails. Walk the segments instead.
        var segments = HudTextMaterialPath.Split('/');
        var parent = segments[0];
        for (int i = 1; i < segments.Length - 1; i++)
        {
            var child = parent + "/" + segments[i];
            if (!AssetDatabase.IsValidFolder(child)) AssetDatabase.CreateFolder(parent, segments[i]);
            parent = child;
        }

        var created = new Material(source) { name = "HudText" };
        created.EnableKeyword("UNDERLAY_ON");
        created.SetColor("_UnderlayColor", new Color(0f, 0f, 0f, 0.85f));
        created.SetFloat("_UnderlayOffsetX", 0.7f);
        created.SetFloat("_UnderlayOffsetY", -0.7f);
        created.SetFloat("_UnderlayDilate", 0.15f);
        created.SetFloat("_UnderlaySoftness", 0.08f);

        AssetDatabase.CreateAsset(created, HudTextMaterialPath);
        AssetDatabase.SaveAssets();
        Debug.Log($"[SlimUi] Built the HUD shadow material at {HudTextMaterialPath}.");
        return created;
    }

    /// <summary>
    /// Points a HUD label at the shadowed material, building the material from the label's
    /// own font material the first time so it always matches the project's font.
    /// </summary>
    public static void ApplyHudTextMaterial(TMPro.TMP_Text text)
    {
        if (text == null) return;
        // fontSharedMaterial, not fontMaterial: the latter throws in edit mode on a prefab.
        var mat = HudTextMaterial(text.fontSharedMaterial);
        if (mat != null) text.fontSharedMaterial = mat;
    }

    /// <summary>Convenience: a panel Image carrying the pack's frame art.</summary>
    public static void ApplyPanel(Image image, Color tint)
    {
        ApplySliced(image, PanelFramePath, tint);
    }

    /// <summary>
    /// Restyles a card's SelectedIndicator from the little corner dot it used to be into a
    /// full-card amber border.
    ///
    /// The dot was authored by UIBuilder.AddIndicator as a 20x20 green square in one corner.
    /// It is too quiet to be the thing that tells the player which of six cars they picked,
    /// and green-on-nothing has no relationship to the rest of the palette. The child keeps
    /// its name, because CarCardImpl, TrackCardImpl and WingCardImpl all toggle that name
    /// and none of them should have to know how it looks.
    ///
    /// It is drawn with the pack's own 9-sliced frame rather than uGUI's Outline component,
    /// and that took two attempts to get right. Outline is not a stroke: it draws four offset
    /// COPIES of the whole graphic, so on a full-card rect the union of them covers the
    /// entire card and you get a solid amber block, not a border. Making the graphic
    /// transparent to get the centre back does not help either, because with
    /// useGraphicAlpha = true the outline's alpha is multiplied by that transparent graphic
    /// and the border disappears entirely. Both failures look correct in the inspector — the
    /// component is enabled and the colour is right — and neither is caught by a test.
    ///
    /// A sliced frame sprite is a real border: the middle of the sprite is empty, so the
    /// centre stays clear and the amber only appears around the edge. The pack ships
    /// "Button Fram 256px" with a 9-slice border of 8/6/7/7 already set on the import.
    /// </summary>
    public static void ApplySelectedBorder(GameObject card, string indicatorName)
    {
        if (card == null) return;
        var indicator = card.transform.Find(indicatorName) as RectTransform;
        if (indicator == null) return;

        indicator.anchorMin = Vector2.zero;
        indicator.anchorMax = Vector2.one;
        indicator.pivot = new Vector2(0.5f, 0.5f);
        indicator.offsetMin = Vector2.zero;
        indicator.offsetMax = Vector2.zero;

        // A leftover Outline from an earlier run would draw a solid block over the card.
        var stale = indicator.GetComponent<Outline>();
        if (stale != null) Object.DestroyImmediate(stale, true);

        var image = indicator.GetComponent<Image>();
        if (image == null) image = indicator.gameObject.AddComponent<Image>();
        ApplySliced(image, ButtonFramePath, SelectedBorder);
        image.raycastTarget = false;
    }

    /// <summary>
    /// Colours a card's labels for the dark tile body, treating a named "status"-style line
    /// as muted.
    ///
    /// Uses the tile inks, not <see cref="Ink"/>. The cards are dark and the buttons are
    /// light, and the two sets of labels are on opposite surfaces — the dark ink is unreadable
    /// on a tile and the light ink is unreadable on a button, so the choice is per-surface
    /// rather than global.
    /// </summary>
    public static void ApplyCardInk(GameObject card, string[] plain, string[] muted = null)
    {
        if (card == null) return;
        foreach (var name in plain)
        {
            var t = card.transform.Find(name)?.GetComponent<TMPro.TMP_Text>();
            if (t != null) t.color = TileInk;
        }
        if (muted == null) return;
        foreach (var name in muted)
        {
            var t = card.transform.Find(name)?.GetComponent<TMPro.TMP_Text>();
            if (t != null) t.color = TileInkMuted;
        }
    }
}
