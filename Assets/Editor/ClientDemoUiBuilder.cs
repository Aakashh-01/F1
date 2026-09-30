// Assets/Editor/ClientDemoUiBuilder.cs
//
// Lays out the selection cards so they can carry artwork, and drops the generated
// backgrounds onto the screen prefabs.
//
// Two things were wrong with the cards before this. The root of each card had no
// preferred size, so the VerticalLayoutGroup that holds them had nothing to size
// against and gave every card zero width — the 300px-wide text was overflowing a
// zero-width card, which is why the screens read as unfinished. And there was no
// Image anywhere to put a picture in, because CarDefinition.Icon and
// TrackDefinition.Thumbnail were authored as Sprite fields and never filled.
//
// Re-runnable: it edits the existing prefabs in place, so running it twice is
// harmless. Run via Tools > BuildClientDemoUI.
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UI;

public class ClientDemoUiBuilder
{
    private const string CarCardPath = "Assets/Prefabs/CarCard_Prefab.prefab";
    private const string TrackCardPath = "Assets/Prefabs/TrackCard_Prefab.prefab";

    // The card is now a TILE, not a column entry. It lives in a horizontal strip along the
    // bottom of the lobby's selection panel, six across at the 1920x1080 reference:
    // 6 * 250 + 5 * 22 spacing = 1610, which clears the 80-unit strip insets either side.
    //
    // That makes it squarer than the 300x270 it used to be, which is what a strip wants -
    // a row of 1.1:1 cards reads as a shelf of cars, where a row of 300x270 read as a shelf
    // of letterboxes. The art panel grows to match so the picture still fills its frame.
    private const float CardWidth = 250f;
    private const float CardHeight = 280f;

    // The art panel and the text stack have to share 280 units between them. The text needs
    // 28 + 22 + 22 + 22 + 34 = 128 for name, gen, cost, status and the button, so the art
    // gets the remaining 152. At 168 the stack overflowed by 16 and the status line sat on
    // top of the Select button.
    //
    // 132 chosen for a 1.77:1 art panel against the ~1.83:1 generated car images, so the
    // picture still fills its frame without letterboxing.
    private const float ArtHeight = 132f;
    private const float CardCornerPad = 8f;

    [MenuItem("Tools/BuildClientDemoUI")]
    public static void Build()
    {
        SlimUiSkin.EnsureLoaded();
        BuildCarCard();
        BuildTrackCard();
        AssetDatabase.SaveAssets();
        Debug.Log("[ClientDemoUI] Card prefabs rebuilt with artwork slots and preferred sizes.");
    }

    // --- Screens: backgrounds and the missing loading screen ---

    private const string LoadingScenePath = "Assets/Scenes/00_LoadingScene.unity";
    private const string CarScreenPrefabPath = "Assets/Prefabs/CarSelectionScreen_Prefab.prefab";
    private const string TrackScreenPrefabPath = "Assets/Prefabs/TrackSelectionScreen_Prefab.prefab";
    private const string WingScreenPrefabPath = "Assets/Prefabs/WingSetupScreen_Prefab.prefab";
    private const string BrandingPrefabPath = "Assets/Prefabs/BrandingScreen_Prefab.prefab";
    private const string ArtRoot = "Assets/Art/";

    /// <summary>
    /// Puts the artwork behind the three selection screens and, more importantly, gives the
    /// loading scene a screen at all.
    ///
    /// 00_LoadingScene contained no Canvas whatsoever — only the flow root, the host and an
    /// EventSystem — so LoadingSceneController was logging "No BrandingScreen found" every
    /// run and the scene advanced through a blank frame. The branding prefab itself was fine
    /// and fully wired; it simply was never in the scene.
    /// </summary>
    /// <summary>
    /// Rewrites each generated background as a 16:9 image, in place.
    ///
    /// The image provider returns 1024x1024 whatever aspect is asked for. uGUI cannot do
    /// "cover" — a background Image is either stretched (cars go oval) or letterboxed (black
    /// bars top and bottom). Cropping the source to the screen's aspect is the only way to get
    /// a clean full-bleed result without a runtime fitter component.
    /// </summary>
    [MenuItem("Tools/CropBackgroundsToWidescreen")]
    public static void CropBackgroundsToWidescreen()
    {
        foreach (var guid in AssetDatabase.FindAssets("t:Texture2D", new[] { ArtRoot + "Backgrounds" }))
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            var source = AssetDatabase.LoadAssetAtPath<Texture2D>(path);
            if (source == null) continue;

            int w = source.width;
            int h = source.height;
            int targetH = Mathf.RoundToInt(w * 9f / 16f);
            if (targetH <= 0 || targetH >= h)
                continue; // already widescreen or taller than it

            // GetPixels needs a readable texture. Turn it on, do the crop, then turn it back
            // off — leaving Read/Write enabled costs GPU memory on a full-screen background.
            var importer = AssetImporter.GetAtPath(path) as TextureImporter;
            bool wasReadable = importer != null && importer.isReadable;
            if (importer != null && !wasReadable)
            {
                importer.isReadable = true;
                importer.SaveAndReimport();
                source = AssetDatabase.LoadAssetAtPath<Texture2D>(path);
            }

            // Bias the crop upward: skies and crowds live at the top, and the bottom of a
            // square generation is usually foreground clutter.
            int offsetY = (h - targetH) / 4;

            var pixels = source.GetPixels();
            var cropped = new Color[w * targetH];
            for (int y = 0; y < targetH; y++)
            {
                for (int x = 0; x < w; x++)
                    cropped[y * w + x] = pixels[(y + offsetY) * w + x];
            }

            var result = new Texture2D(w, targetH, TextureFormat.RGB24, false);
            result.SetPixels(cropped);
            result.Apply();
            File.WriteAllBytes(path, result.EncodeToPNG());
            UnityEngine.Object.DestroyImmediate(result);

            AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceUpdate);
            if (importer != null && !wasReadable)
            {
                importer.isReadable = false;
                importer.SaveAndReimport();
            }
            Debug.Log($"[ClientDemoUI] Cropped {Path.GetFileName(path)} to {w}x{targetH} (16:9).");
        }
    }

    [MenuItem("Tools/BuildClientDemoScreens")]
    public static void BuildScreens()
    {
        CropBackgroundsToWidescreen();
        ApplyBackground(CarScreenPrefabPath, ArtRoot + "Backgrounds/bg_carselect.png");
        // v2, not the original. The first track background was an aerial night photograph of a
        // circuit, and the track thumbnails are also warm, detailed circuit photographs — two
        // images of the same subject at the same level of detail, so they fought each other
        // and neither read. v2 is a dark abstract circuit-line graphic: it says "circuits"
        // without ever looking like another track photo, and its cool near-black field makes
        // the warm thumbnails the brightest thing on screen. The original bg_trackselect.png
        // is still on disk if it is ever wanted back.
        ApplyBackground(TrackScreenPrefabPath, ArtRoot + "Backgrounds/bg_trackselect_v2.png");
        ApplyBackground(WingScreenPrefabPath, ArtRoot + "Backgrounds/bg_wingsetup.png");
        ApplyBackground(BrandingPrefabPath, ArtRoot + "Backgrounds/bg_loading.png");
        EnsureLoadingScreenHasBranding();
        AssetDatabase.SaveAssets();
        Debug.Log("[ClientDemoUI] Screen backgrounds applied and loading screen ensured.");
    }

    /// <summary>
    /// Sets a screen prefab's root Image to the given sprite, stretched and aspect-filled, with
    /// a dark scrim behind the content so labels stay readable over artwork.
    /// </summary>
    private static void ApplyBackground(string prefabPath, string spritePath)
    {
        var sprite = AssetDatabase.LoadAssetAtPath<Sprite>(spritePath);
        var root = PrefabUtility.LoadPrefabContents(prefabPath);
        try
        {
            var image = root.GetComponent<Image>();
            if (image == null)
            {
                Debug.LogWarning($"[ClientDemoUI] {prefabPath} has no root Image; skipped.");
                return;
            }

            if (sprite != null)
            {
                image.sprite = sprite;
                // uGUI has no "cover" mode: Simple letterboxes, Filled only does directional
                // and radial fills. The artwork is therefore cropped to 16:9 at import time
                // (see Tools/CropBackgroundsToWidescreen) so Simple + preserveAspect lands
                // exactly on a 16:9 screen with no bars and no stretching.
                image.type = Image.Type.Simple;
                image.preserveAspect = true;
                image.color = Color.white;
            }
            else
            {
                // Art not generated yet. Keep the dark flat colour so the screen still reads
                // as designed rather than showing an untextured box.
                image.sprite = null;
                image.type = Image.Type.Simple;
                image.color = new Color(0.04f, 0.05f, 0.08f, 1f);
                Debug.LogWarning($"[ClientDemoUI] Missing sprite {spritePath}; kept flat colour.");
            }

            AddScrim(root.transform);
            PrefabUtility.SaveAsPrefabAsset(root, prefabPath);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    /// <summary>
    /// A translucent dark panel between the artwork and the text. Generated imagery is
    /// unpredictable in brightness, and without this the labels lose contrast on the light
    /// parts of a photo.
    /// </summary>
    private static void AddScrim(Transform root)
    {
        var existing = root.Find("BackdropScrim");
        if (existing != null) return;

        var go = new GameObject("BackdropScrim", typeof(RectTransform), typeof(CanvasRenderer), typeof(Image));
        var rt = (RectTransform)go.transform;
        rt.SetParent(root, false);
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;

        var img = go.GetComponent<Image>();
        img.color = new Color(0.02f, 0.03f, 0.05f, 0.55f);
        img.raycastTarget = false;

        // First child so it draws over the root background but under every label.
        rt.SetSiblingIndex(0);
    }

    /// <summary>
    /// Puts each screen prefab into the scene that is supposed to show it.
    ///
    /// None of the flow scenes contained any UI at all: 10, 20, 30 and 40 held only a
    /// FlowSceneHost and an EventSystem. The screen prefabs existed and were fully wired, but
    /// they had never been placed, so every one of those scenes rendered black. This is why
    /// "the car selection screen is black" — nothing was wrong with the prefab's styling,
    /// there was simply nothing in the scene to style.
    /// </summary>
    /// <summary>
    /// Rearranges the track screen into a centred hero tile with the unbuilt circuits
    /// flanking it.
    ///
    /// The two Owned/Locked columns produced a ragged 2x3 grid with a hole in it, and put the
    /// one playable circuit in a top corner where it read as just another option. The column
    /// split also duplicated information the per-card AVAILABLE / COMING SOON status already
    /// carries, so it goes entirely.
    ///
    /// Re-runnable and idempotent: existing columns are found and re-anchored rather than
    /// duplicated, and any left over from the old two-column layout are removed.
    /// </summary>
    [MenuItem("Tools/BuildTrackSelectionLayout")]
    public static void BuildTrackSelectionLayout()
    {
        var root = PrefabUtility.LoadPrefabContents(TrackScreenPrefabPath);
        try
        {
            var impl = FindImpl(root, "TrackSelectionScreenImpl");
            if (impl == null)
            {
                Debug.LogError($"[ClientDemoUI] TrackSelectionScreenImpl not found in {TrackScreenPrefabPath}.");
                return;
            }

            // The retired columns. They are deleted, not reused, so the two fields the impl
            // no longer has cannot keep a stale reference alive.
            foreach (var dead in new[] { "OwnedColumn", "LockedColumn" })
            {
                var existing = root.transform.Find(dead);
                if (existing != null) Object.DestroyImmediate(existing.gameObject);
            }

            var hero = EnsureColumn(root.transform, "HeroColumn");
            var left = EnsureColumn(root.transform, "LockedLeftColumn");
            var right = EnsureColumn(root.transform, "LockedRightColumn");

            // Hero centred, wide enough for a 430 tile with air either side. The flanking
            // columns take the outer thirds for 232-wide supporting tiles. The vertical band
            // stops short of the title and clears the bottom edge.
            Anchor(hero, new Vector2(0.30f, 0.06f), new Vector2(0.70f, 0.80f), 26f);
            Anchor(left, new Vector2(0.03f, 0.16f), new Vector2(0.27f, 0.74f), 18f);
            Anchor(right, new Vector2(0.73f, 0.16f), new Vector2(0.97f, 0.74f), 18f);

            var so = new SerializedObject(impl);
            Set(so, "_heroColumn", hero);
            Set(so, "_lockedLeftColumn", left);
            Set(so, "_lockedRightColumn", right);
            so.ApplyModifiedPropertiesWithoutUndo();

            PrefabUtility.SaveAsPrefabAsset(root, TrackScreenPrefabPath);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }

        // The screen is installed as an instance in the scene, so the rebuilt prefab has to
        // be pushed back out or the scene keeps the old two-column copy.
        PushIntoScene(TrackScreenPrefabPath, "Assets/Scenes/20_TrackSelectionScene.unity",
                      "TrackSelectionScreen");

        Debug.Log("[ClientDemoUI] Track screen laid out as a centred hero with flanking columns.");
    }

    private static RectTransform EnsureColumn(Transform root, string name)
    {
        var existing = root.Find(name) as RectTransform;
        if (existing != null) return existing;

        var go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(root, false);
        return (RectTransform)go.transform;
    }

    private static void Anchor(RectTransform rt, Vector2 min, Vector2 max, float spacing)
    {
        var group = rt.GetComponent<VerticalLayoutGroup>() ?? rt.gameObject.AddComponent<VerticalLayoutGroup>();
        group.spacing = spacing;
        group.childAlignment = TextAnchor.MiddleCenter;
        // The cards carry explicit sizeDeltas sized per role (hero vs supporting), so the
        // group must not control their dimensions.
        group.childControlWidth = false;
        group.childControlHeight = false;
        group.childForceExpandWidth = false;
        group.childForceExpandHeight = false;

        rt.anchorMin = min;
        rt.anchorMax = max;
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;
    }

    /// <summary>
    /// Replaces an installed screen instance in a scene with the current prefab, preserving
    /// its name. Screens are stored as prefab instances, so editing the prefab alone leaves
    /// the scene showing the old version.
    /// </summary>
    private static void PushIntoScene(string prefabPath, string scenePath, string objectName)
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
        if (prefab == null) return;

        var scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
        foreach (var root in scene.GetRootGameObjects())
        {
            if (root.name != objectName) continue;
            Object.DestroyImmediate(root);
            var instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab, scene);
            instance.name = objectName;
            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            Debug.Log($"[ClientDemoUI] Reinstalled {objectName} in {Path.GetFileName(scenePath)}.");
            return;
        }
        Debug.LogWarning($"[ClientDemoUI] No '{objectName}' root in {scenePath}; not reinstalled.");
    }

    /// <summary>Assigns an object-reference serialized field.</summary>
    private static void Set(SerializedObject so, string property, Object value)
    {
        var prop = so.FindProperty(property);
        if (prop == null)
        {
            Debug.LogError($"[ClientDemoUI] No field '{property}'.");
            return;
        }
        prop.objectReferenceValue = value;
    }

    [MenuItem("Tools/InstallScreensIntoScenes")]
    public static void InstallScreens()
    {
        Install("Assets/Scenes/10_CarSelectionScene.unity", "CarSelectionScreen", CarScreenPrefabPath);
        Install("Assets/Scenes/20_TrackSelectionScene.unity", "TrackSelectionScreen", TrackScreenPrefabPath);
        Install("Assets/Scenes/30_WingSetupScene.unity", "WingSetupScreen", WingScreenPrefabPath);
        Install("Assets/Scenes/40_PreRaceScene.unity", "QualifyingScreen", "Assets/Prefabs/QualifyingScreen_Prefab.prefab");
    }

    private static void Install(string scenePath, string objectName, string prefabPath)
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
        if (prefab == null)
        {
            Debug.LogError($"[ClientDemoUI] Missing prefab {prefabPath} for {scenePath}.");
            return;
        }

        var scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);

        foreach (var root in scene.GetRootGameObjects())
        {
            if (root.name == objectName) return; // already installed
        }

        var instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        instance.name = objectName;
        Undo.RegisterCreatedObjectUndo(instance, "Install screen prefab");
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Debug.Log($"[ClientDemoUI] Installed {objectName} into {Path.GetFileName(scenePath)}.");
    }

    /// <summary>
    /// Gives the loading scene a camera.
    ///
    /// 00_LoadingScene had none at all, and Unity logs a frame with nothing to render
    /// through — the "no camera available" complaint. The branding screen is a
    /// Screen Space - Overlay canvas, so uGUI itself does not need a camera and the UI drew
    /// fine; what was missing was anything for the engine to render the world with, which
    /// leaves the view behind the overlay undefined and produces a warning every run.
    ///
    /// The camera renders a flat dark clear and nothing else. The branding artwork is a
    /// full-screen Image on the overlay, so this exists to give the frame a surface, not to
    /// light or compose anything. It is tagged MainCamera so Camera.main resolves during the
    /// load, and it is destroyed with the scene, so the lobby's own LobbyCamera takes over
    /// the moment the flow advances.
    ///
    /// Idempotent, and additive: the scene is never made active, and a second run finds the
    /// camera and does nothing.
    /// </summary>
    /// <summary>
    /// Points the loading screen at a background and brings its progress bar into the theme.
    ///
    /// Narrow on purpose. `BuildScreens` also re-applies backgrounds to the car, track and
    /// WING screens, and the wing one would put an opaque picture back over the garage — the
    /// exact defect §4a of the handoff was about. This touches the branding screen only.
    ///
    /// The artwork is `bg_transition.png` — the user's own F1 key art, pasted into
    /// Assets/Scenes and copied here so BOTH loading screens share one file: this branding
    /// screen, and the transition card FlowOverlay shows between scenes. It is 1671x941,
    /// already 16:9, so it needs no crop.
    ///
    /// Every earlier candidate is still on disk and nothing was overwritten: bg_loading.png
    /// (the original stock night race), bg_loading_v2.png (the dark garage) and
    /// bg_trackselect_v2.png.
    ///
    /// The bar is recoloured in the same pass because it was the loudest thing on the new
    /// artwork: a hard RGBA(0.878, 0.024, 0.000) red sitting across a cool, dark image. It
    /// now uses the same amber accent as Start Race and Continue, so the loading screen is
    /// part of the same product rather than a red bar bolted onto a moody photograph.
    /// </summary>
    [MenuItem("Tools/UI/Apply Loading Screen Art", priority = 72)]
    public static void ApplyLoadingBackground()
    {
        ApplyBackground(BrandingPrefabPath, ArtRoot + "Backgrounds/bg_transition.png");
        SkinLoadingProgressBar();
    }

    /// <summary>Recolours the branding screen's progress track and fill to the theme.</summary>
    private static void SkinLoadingProgressBar()
    {
        var contents = PrefabUtility.LoadPrefabContents(BrandingPrefabPath);
        try
        {
            // The bar lives under "LoadingProgress", not a "ProgressRoot".
            var bar = contents.transform.Find("LoadingProgress");
            if (bar == null)
            {
                Debug.LogWarning("[ClientDemoUI] No LoadingProgress on the branding screen; " +
                                 "the bar was left alone.");
                return;
            }

            var trackImage = bar.Find("Track")?.GetComponent<Image>();
            var fillImage = bar.Find("Fill")?.GetComponent<Image>();
            if (trackImage == null || fillImage == null)
            {
                Debug.LogWarning("[ClientDemoUI] LoadingProgress is missing its Track or Fill; " +
                                 "the bar was left alone.");
                return;
            }

            // A translucent dark trough, so it reads as a container against both the dark
            // garage and the brighter car rather than as a solid bar.
            trackImage.color = new Color(0.10f, 0.11f, 0.14f, 0.75f);
            trackImage.raycastTarget = false;

            fillImage.color = SlimUiSkin.Accent;
            fillImage.raycastTarget = false;

            PrefabUtility.SaveAsPrefabAsset(contents, BrandingPrefabPath);
            Debug.Log("[ClientDemoUI] Loading bar recoloured to the theme accent (was hard red).");
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(contents);
        }
    }

    /// <summary>
    /// Removes the "Select your race" subtitle from the branding screen.
    ///
    /// The object is deleted rather than blanked, and the impl's field cleared, so nothing
    /// can put the text back by writing to it. BrandingScreenImpl only ever calls
    /// SetVisible on it and that is null-safe, so the screen is otherwise unaffected.
    /// </summary>
    [MenuItem("Tools/UI/Remove Loading Screen Subtitle", priority = 73)]
    public static void RemoveLoadingSubtitle()
    {
        var contents = PrefabUtility.LoadPrefabContents(BrandingPrefabPath);
        try
        {
            var subtitle = contents.transform.Find("SubtitleText");
            if (subtitle == null)
            {
                Debug.Log("[ClientDemoUI] No SubtitleText on the branding screen; already removed.");
                return;
            }

            Object.DestroyImmediate(subtitle.gameObject);

            var impl = FindImpl(contents, "BrandingScreenImpl");
            if (impl != null)
            {
                // One SerializedObject for both the read and the write: a second instance
                // would carry a different pending-change set and the clear would be dropped.
                var so = new SerializedObject(impl);
                var prop = so.FindProperty("_subtitleText");
                if (prop != null)
                {
                    prop.objectReferenceValue = null;
                    so.ApplyModifiedPropertiesWithoutUndo();
                }
                else
                {
                    Debug.LogWarning("[ClientDemoUI] BrandingScreenImpl has no _subtitleText field.");
                }
            }

            PrefabUtility.SaveAsPrefabAsset(contents, BrandingPrefabPath);
            Debug.Log("[ClientDemoUI] Removed the loading screen subtitle.");
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(contents);
        }
    }

    /// <summary>
    /// Gives the genuinely camera-less UI scenes a camera.
    ///
    /// 00_LoadingScene and 20_TrackSelectionScene had none, and Unity logs a frame with
    /// nothing to render through — the "no camera available" complaint. Their screens are
    /// Screen Space - Overlay canvases over a flat backdrop, so uGUI never needed a camera
    /// and the UI drew fine; what was missing was a surface for the engine to render the
    /// frame with.
    ///
    /// The camera clears to a flat dark colour and draws nothing (`cullingMask = 0`). It
    /// exists to give the frame something to render through, not to light or compose
    /// anything — the artwork behind each screen is a full-screen Image on the overlay.
    /// Tagged MainCamera so Camera.main resolves, and destroyed with its scene.
    ///
    /// **Three scenes are deliberately NOT in this list, and adding them breaks things.**
    ///
    /// - **40_PreRaceScene** was in an earlier version of this list and it was wrong. It is
    ///   not a UI scene: it is a driving scene. `PreRaceSceneController` loads the track
    ///   additively and calls `PlayerCarSpawner.Spawn`, and the car arrives carrying its own
    ///   `Main Camera`. A second camera that clears to near-black and renders nothing does
    ///   not sit harmlessly alongside it — it CLEARS the target and draws no replacement, so
    ///   it wipes the chase camera's output and the whole screen goes black. That is not a
    ///   cosmetic problem and no test would have caught it; the scene still "worked", it
    ///   simply rendered nothing.
    /// - **50_RaceScene** and **Track_01** have no camera in the scene and that is correct
    ///   for the same reason — the camera rides on the car prefab.
    ///
    /// Idempotent and additive: each scene is opened additively, never made active, and a
    /// scene that already has any camera is left alone.
    /// </summary>
    /// <summary>
    /// Brings the qualifying HUD into the same skin as the selection screens.
    ///
    /// It is a different kind of surface from everything else here, and the two halves need
    /// opposite treatment:
    ///
    /// - **The lap clock floats over live 3D.** No panel behind it by design — the whole
    ///   point is to watch the lap, and UIBuilder's own note says an opaque full-screen
    ///   background "would hide the driving entirely". So it cannot be given a dark tile to
    ///   sit on, and a colour alone will not carry it: white reads against dark scenery and
    ///   vanishes against a bright sky, which the track scenes have plenty of. It gets a dark
    ///   underlay instead, so the same glyph survives both.
    /// - **The results panel IS a tile**, and it is already a dark one. It moves onto the
    ///   same deep indigo as the selection cards, with the same light ink, and its three
    ///   buttons take the same three-layer treatment — light fill, the pack's frame, dark
    ///   label — so the panel does not read as a leftover from an older build.
    ///
    /// Go to Race is the primary of the three, so it gets the amber accent; Back and Retry
    /// stay neutral.
    /// </summary>
    [MenuItem("Tools/UI/Skin Qualifying Screen", priority = 74)]
    public static void SkinQualifyingScreen()
    {
        const string path = "Assets/Prefabs/QualifyingScreen_Prefab.prefab";
        var contents = PrefabUtility.LoadPrefabContents(path);
        try
        {
            // --- The lap clock, over the track ---
            var track = contents.transform.Find("TrackNameText")?.GetComponent<TMPro.TextMeshProUGUI>();
            var lap = contents.transform.Find("CurrentLapTimeText")?.GetComponent<TMPro.TextMeshProUGUI>();
            var best = contents.transform.Find("BestLapText")?.GetComponent<TMPro.TextMeshProUGUI>();

            foreach (var t in new[] { track, lap })
            {
                if (t == null) continue;
                t.color = Color.white;
                SlimUiSkin.ApplyHudTextMaterial(t);
            }
            if (best != null)
            {
                // Was 0.72/0.84/0.94 — a pale blue that lost all contrast the moment the
                // camera faced sky. Brighter, and now shadowed.
                best.color = new Color(0.90f, 0.93f, 0.97f, 1f);
                SlimUiSkin.ApplyHudTextMaterial(best);
            }

            // --- The results panel: a tile like any other ---
            var panelRt = contents.transform.Find("ResultsPanel") as RectTransform;
            if (panelRt == null)
            {
                Debug.LogError("[ClientDemoUI] QualifyingScreen_Prefab has no ResultsPanel.");
                return;
            }
            var panelImage = panelRt.GetComponent<Image>();
            if (panelImage != null)
                SlimUiSkin.ApplyFlatPanel(panelImage, SlimUiSkin.CardFill);

            var resultsTitle = panelRt.Find("ResultsTitleText")?.GetComponent<TMPro.TextMeshProUGUI>();
            var resultsCaption = panelRt.Find("ResultsCaptionText")?.GetComponent<TMPro.TextMeshProUGUI>();
            var resultsTime = panelRt.Find("ResultLapTimeText")?.GetComponent<TMPro.TextMeshProUGUI>();
            if (resultsTitle != null) resultsTitle.color = SlimUiSkin.TileInk;
            if (resultsCaption != null) resultsCaption.color = SlimUiSkin.TileInkMuted;
            if (resultsTime != null) resultsTime.color = Color.white;

            // --- Its three buttons ---
            SkinPanelButton(panelRt, "BackButton", primary: false);
            SkinPanelButton(panelRt, "RetryButton", primary: false);
            SkinPanelButton(panelRt, "GoToRaceButton", primary: true);

            PrefabUtility.SaveAsPrefabAsset(contents, path);
            Debug.Log("[ClientDemoUI] Qualifying HUD skinned: lap clock shadowed for the 3D " +
                      "backdrop, results panel moved onto the tile palette.");
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(contents);
        }
    }

    /// <summary>
    /// Gives a button inside the results panel the standard three-layer treatment and a dark
    /// label, since the fill is now light.
    /// </summary>
    private static void SkinPanelButton(RectTransform panel, string name, bool primary)
    {
        var rt = panel.Find(name) as RectTransform;
        var button = rt?.GetComponent<Button>();
        if (button == null)
        {
            Debug.LogWarning($"[ClientDemoUI] No button '{name}' in the results panel; skipped.");
            return;
        }

        if (primary) SlimUiSkin.ApplyPrimaryButton(button);
        else SlimUiSkin.ApplySecondaryButton(button);

        var label = rt.Find("Label")?.GetComponent<TMPro.TextMeshProUGUI>();
        if (label != null)
        {
            label.color = SlimUiSkin.Ink;
            label.fontStyle = TMPro.FontStyles.Bold;
        }
    }

    [MenuItem("Tools/UI/Ensure UI Scene Cameras", priority = 71)]
    public static void EnsureUiSceneCameras()
    {
        var targets = new (string scene, string cameraName, Color clear)[]
        {
            (LoadingScenePath, "LoadingCamera", new Color(0.035f, 0.040f, 0.050f, 1f)),
            ("Assets/Scenes/20_TrackSelectionScene.unity", "MenuCamera",
                new Color(0.020f, 0.030f, 0.050f, 1f)),
        };

        int added = 0, present = 0, missing = 0;
        foreach (var (scenePath, cameraName, clear) in targets)
        {
            if (!System.IO.File.Exists(scenePath))
            {
                Debug.LogWarning($"[ClientDemoUI] {scenePath} not found; skipped.");
                missing++;
                continue;
            }

            var scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Additive);
            try
            {
                bool alreadyHasOne = false;
                foreach (var root in scene.GetRootGameObjects())
                {
                    if (root.GetComponentInChildren<Camera>(true) != null) { alreadyHasOne = true; break; }
                }

                if (alreadyHasOne)
                {
                    Debug.Log($"[ClientDemoUI] {Path.GetFileName(scenePath)} already has a camera; " +
                              "nothing to do.");
                    present++;
                    continue;
                }

                var go = new GameObject(cameraName, typeof(Camera), typeof(AudioListener));
                UnityEngine.SceneManagement.SceneManager.MoveGameObjectToScene(go, scene);
                go.tag = "MainCamera";

                var cam = go.GetComponent<Camera>();
                cam.clearFlags = CameraClearFlags.SolidColor;
                cam.backgroundColor = clear;
                cam.nearClipPlane = 0.3f;
                cam.farClipPlane = 100f;
                // A UI-only scene: draw nothing, so the flat clear is all that sits behind
                // the overlay, and leave the mask clear for anything the scene grows later.
                cam.cullingMask = 0;

                go.transform.position = new Vector3(0f, 1f, -10f);
                go.transform.rotation = Quaternion.identity;

                EditorSceneManager.MarkSceneDirty(scene);
                EditorSceneManager.SaveScene(scene);
                added++;
                Debug.Log($"[ClientDemoUI] Added {cameraName} to {Path.GetFileName(scenePath)}; it had " +
                          "no camera, so Unity had nothing to render the frame through.");
            }
            finally
            {
                EditorSceneManager.CloseScene(scene, true);
            }
        }

        Debug.Log($"[ClientDemoUI] UI scene cameras: {added} added, {present} already present, " +
                  $"{missing} skipped. 40_PreRaceScene, 50_RaceScene and Track_01 are excluded on " +
                  "purpose — they are driving scenes and their camera rides on the car prefab. " +
                  "A flat-clear camera there wipes the chase camera's output and the screen goes black.");
    }

    /// <summary>Adds the branding prefab to the loading scene if it is not already there.</summary>
    private static void EnsureLoadingScreenHasBranding()
    {
        var scene = EditorSceneManager.OpenScene(LoadingScenePath, OpenSceneMode.Single);

        bool alreadyThere = false;
        foreach (var root in scene.GetRootGameObjects())
        {
            if (root.name == "BrandingScreen") { alreadyThere = true; break; }
        }
        if (alreadyThere) return;

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(BrandingPrefabPath);
        if (prefab == null)
        {
            Debug.LogError($"[ClientDemoUI] Branding prefab missing at {BrandingPrefabPath}.");
            return;
        }

        var instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        instance.name = "BrandingScreen";
        Undo.RegisterCreatedObjectUndo(instance, "Add branding to loading scene");
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Debug.Log("[ClientDemoUI] Added the missing BrandingScreen to the loading scene.");
    }
    private static void BuildCarCard() => BuildCard(CarCardPath, "CarCardImpl", "CarArt");

    private static void BuildTrackCard()
    {
        BuildCard(TrackCardPath, "TrackCardImpl", "TrackArt");
        AddComingSoonBadge(TrackCardPath);
    }

    /// <summary>
    /// Gives the track card a CanvasGroup to dim with and a badge to show when the circuit
    /// has no scene in the build. Both are referenced by TrackCardImpl; without them the
    /// "COMING SOON" state has nothing to act on.
    /// </summary>
    private static void AddComingSoonBadge(string path)
    {
        var root = PrefabUtility.LoadPrefabContents(path);
        try
        {
            var cardRect = (RectTransform)root.transform;

            var group = cardRect.GetComponent<CanvasGroup>();
            if (group == null) group = cardRect.gameObject.AddComponent<CanvasGroup>();
            group.alpha = 1f;
            group.interactable = true;
            group.blocksRaycasts = true;

            // A compact pill rather than a full-width banner. At 88 units tall on a 270
            // card it covered most of the artwork behind it, and a dimmed picture under a
            // dark banner is just a black rectangle — the thumbnail is the thing worth seeing.
            var badge = FindOrCreateChild(cardRect, "ComingSoonBadge");
            var badgeImage = badge.GetComponent<Image>();
            badgeImage.color = new Color(0f, 0f, 0f, 0.78f);
            badgeImage.raycastTarget = false;
            badgeRect(badge, ArtHeight);

            var label = FindOrCreateTextChild(badge, "ComingSoonText");
            var text = label.GetComponent<TMPro.TextMeshProUGUI>();
            text.text = "COMING SOON";
            text.fontSize = 18f;
            text.fontStyle = TMPro.FontStyles.Bold;
            text.alignment = TMPro.TextAlignmentOptions.Center;
            text.color = new Color(1f, 0.82f, 0.25f, 1f);
            text.raycastTarget = false;
            var labelRect = (RectTransform)label;
            labelRect.anchorMin = Vector2.zero;
            labelRect.anchorMax = Vector2.one;
            labelRect.offsetMin = Vector2.zero;
            labelRect.offsetMax = Vector2.zero;

            badge.gameObject.SetActive(false);

            // Written explicitly rather than left to the C# default, because a prefab
            // serialises whatever the field held when it was authored. Change the default
            // and an existing prefab keeps the old number, which is how the unavailable-card
            // dim stayed at 0.45 after the light theme landed.
            var impl = FindImpl(root, "TrackCardImpl");
            if (impl != null)
            {
                var implSo = new SerializedObject(impl);
                var alphaProp = implSo.FindProperty("_unavailableAlpha");
                if (alphaProp != null)
                {
                    alphaProp.floatValue = 0.75f;
                    implSo.ApplyModifiedPropertiesWithoutUndo();
                }
            }

            PrefabUtility.SaveAsPrefabAsset(root, path);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    private static void badgeRect(RectTransform badge, float artHeight)
    {
        // A pill across the lower part of the artwork, inset so the picture frames it.
        badge.anchorMin = new Vector2(0f, 1f);
        badge.anchorMax = new Vector2(1f, 1f);
        badge.pivot = new Vector2(0.5f, 1f);
        float top = -(CardCornerPad + artHeight - 46f);
        badge.offsetMin = new Vector2(38f, top - 30f);
        badge.offsetMax = new Vector2(-38f, top);
    }

    private static void BuildCard(string path, string implTypeName, string artName)
    {
        var root = PrefabUtility.LoadPrefabContents(path);
        try
        {
            var impl = FindImpl(root, implTypeName);
            if (impl == null)
            {
                Debug.LogError($"[ClientDemoUI] {implTypeName} not found in {path}.");
                return;
            }

            // --- Give the card a size for the layout group to use ---
            var layout = root.GetComponent<LayoutElement>();
            if (layout == null) layout = root.AddComponent<LayoutElement>();
            layout.minWidth = CardWidth;
            layout.minHeight = CardHeight;
            layout.preferredWidth = CardWidth;
            layout.preferredHeight = CardHeight;
            layout.flexibleWidth = 0f;
            layout.flexibleHeight = 0f;

            var cardRect = (RectTransform)root.transform;

            // The columns set childControlHeight = false, so the layout group keeps each
            // card's own rect height and ignores the LayoutElement's preferredHeight. Without
            // this the card stays 180 tall while the art panel is taller than that, and the
            // artwork spills over the card edges and onto its neighbours.
            cardRect.sizeDelta = new Vector2(CardWidth, CardHeight);

            // --- Card body: deep indigo, with the pack's panel art over it as an edge ---
            // Dark, not light: the car and track artwork carries its own dark backdrop, and
            // a pale card turned that into a dark rectangle dropped into a bright panel.
            // See SlimUiSkin for why the art well is near-black.
            var body = root.GetComponent<Image>();
            if (body != null)
            {
                SlimUiSkin.ApplyFlatPanel(body, SlimUiSkin.CardFill);
                body.raycastTarget = false;
            }

            var bodyFrame = FindOrCreateChild(cardRect, "CardFrame");
            SlimUiSkin.ApplyPanelOverlay(bodyFrame.GetComponent<Image>(),
                                         SlimUiSkin.CardFrameOverlay);
            bodyFrame.GetComponent<Image>().raycastTarget = false;
            Stretch((RectTransform)bodyFrame);

            // --- Artwork panel across the top ---
            var art = FindOrCreateChild(cardRect, artName);
            var artImage = art.GetComponent<Image>();
            if (artImage == null) artImage = art.gameObject.AddComponent<Image>();
            artImage.raycastTarget = false;
            // PreserveAspect so a square generated image is letterboxed inside the panel
            // rather than stretched into a smear.
            artImage.type = Image.Type.Simple;
            artImage.preserveAspect = true;
            artImage.color = Color.white;

            // A near-black well behind the picture. The artwork's own backdrop is near-black,
            // so the well disappears into it and the subject floats instead of sitting
            // inside a visible box. This is the change that stops the card looking hazy.
            var artWell = FindOrCreateChild(cardRect, artName + "Well");
            SlimUiSkin.ApplyFlatPanel(artWell.GetComponent<Image>(), SlimUiSkin.ArtWellFill);
            artWell.GetComponent<Image>().raycastTarget = false;
            var artWellRect = (RectTransform)artWell;
            artWellRect.anchorMin = new Vector2(0f, 1f);
            artWellRect.anchorMax = new Vector2(1f, 1f);
            artWellRect.pivot = new Vector2(0.5f, 1f);
            artWellRect.offsetMin = new Vector2(CardCornerPad, -(ArtHeight + CardCornerPad));
            artWellRect.offsetMax = new Vector2(-CardCornerPad, -CardCornerPad);
            artWellRect.SetAsFirstSibling();

            var artRect = (RectTransform)art;
            artRect.anchorMin = new Vector2(0f, 1f);
            artRect.anchorMax = new Vector2(1f, 1f);
            artRect.pivot = new Vector2(0.5f, 1f);
            artRect.offsetMin = new Vector2(CardCornerPad, -(ArtHeight + CardCornerPad));
            artRect.offsetMax = new Vector2(-CardCornerPad, -CardCornerPad);

            // Artwork must sit behind the text, so make it the first child to draw.
            art.SetSiblingIndex(0);

            // --- Stack the text below the artwork ---
            // The art panel is the top ArtHeight of the card, so on a 280 card everything
            // above 0.528 is picture and every label has to sit below that. Measured in
            // units from the card's bottom edge, the stack runs: button 1..33, status
            // 40..60, cost 68..88, gen 96..116, name 120..146, with the art starting at 148.
            SetAnchor(root.transform, "NameText", 0.475f, 26f);
            SetAnchor(root.transform, "GenText", 0.379f, 20f);
            SetAnchor(root.transform, "ShortCodeText", 0.379f, 20f);
            SetAnchor(root.transform, "CostText", 0.279f, 20f);
            SetAnchor(root.transform, "StatusText", 0.179f, 20f);
            SetAnchor(root.transform, "CardButton", 0.061f, 32f);

            // --- SlimUI skin ---
            // Labels go light on the now-dark tile. Gen/short-code/cost/status are secondary
            // lines and are muted so the name reads first, which is the hierarchy these cards
            // had before the theme changed.
            SlimUiSkin.ApplyCardInk(root,
                plain: new[] { "NameText" },
                muted: new[] { "GenText", "ShortCodeText", "CostText", "StatusText" });

            // The card's own press target gets the same three-layer button treatment as the
            // screen's Back and Continue: an opaque light fill, the pack's outline on top,
            // and dark ink. The button stays light even though the card is dark — it is the
            // one element that has to read against the dark garage as well.
            var cardButton = root.transform.Find("CardButton")?.GetComponent<Button>();
            if (cardButton != null) SlimUiSkin.ApplyButtonVisual(cardButton, primary: true);
            var buttonLabel = root.transform.Find("CardButton/Label")?.GetComponent<TMPro.TMP_Text>();
            SlimUiSkin.ApplyInk(buttonLabel);

            SlimUiSkin.ApplySelectedBorder(root, "SelectedIndicator");

            // Sibling order is settled last, because three separate passes each reorder this
            // hierarchy and the last one wins otherwise. Bottom to top it has to be:
            //   CardFrame  a translucent light edge, so the dark card is visible at all
            //              against a dark garage
            //   <art>Well  the near-black recess
            //   <art>      the picture itself, never washed by anything above it
            //   labels, button, selected indicator
            // Getting this wrong is invisible in the inspector and wrecks the card: a
            // translucent frame drawn OVER the artwork is exactly the haze this palette
            // exists to remove.
            var frameChild = cardRect.Find("CardFrame");
            var wellChild = cardRect.Find(artName + "Well");
            var artChild = cardRect.Find(artName);
            if (frameChild != null) frameChild.SetSiblingIndex(0);
            if (wellChild != null) wellChild.SetSiblingIndex(1);
            if (artChild != null) artChild.SetSiblingIndex(2);

            PrefabUtility.SaveAsPrefabAsset(root, path);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    private static MonoBehaviour FindImpl(GameObject root, string typeName)
    {
        foreach (var mb in root.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (mb != null && mb.GetType().Name == typeName) return mb;
        }
        return null;
    }

    private static RectTransform FindOrCreateChild(Transform parent, string name)
    {
        var existing = parent.Find(name);
        if (existing != null) return (RectTransform)existing;

        var go = new GameObject(name, typeof(RectTransform), typeof(CanvasRenderer), typeof(Image));
        var rt = (RectTransform)go.transform;
        rt.SetParent(parent, false);
        return rt;
    }

    /// <summary>Fills the parent rect, so an overlay covers exactly the card.</summary>
    private static void Stretch(RectTransform rt)
    {
        if (rt == null) return;
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.pivot = new Vector2(0.5f, 0.5f);
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;
    }

    /// <summary>
    /// Like <see cref="FindOrCreateChild"/> but for a text label. A GameObject can carry only
    /// one Graphic, so this must not add an Image as well — Unity refuses the second.
    /// </summary>
    private static RectTransform FindOrCreateTextChild(Transform parent, string name)
    {
        var existing = parent.Find(name);
        if (existing != null)
        {
            var rt = (RectTransform)existing;
            // A previous run may have left an Image on this object, which would block the
            // TextMeshProUGUI. Strip any Graphic that is not the text itself.
            foreach (var graphic in rt.GetComponents<UnityEngine.UI.Graphic>())
            {
                if (!(graphic is TMPro.TextMeshProUGUI))
                    Object.DestroyImmediate(graphic);
            }
            if (rt.GetComponent<TMPro.TextMeshProUGUI>() == null)
                rt.gameObject.AddComponent<TMPro.TextMeshProUGUI>();
            return rt;
        }

        var go = new GameObject(name, typeof(RectTransform), typeof(CanvasRenderer),
                                typeof(TMPro.TextMeshProUGUI));
        var created = (RectTransform)go.transform;
        created.SetParent(parent, false);
        return created;
    }

    /// <summary>Anchors a child to a fraction of the card's height, centred, with a fixed height.</summary>
    private static void SetAnchor(Transform parent, string childName, float heightFraction, float height)
    {
        var child = parent.Find(childName);
        if (child == null) return;

        var rt = (RectTransform)child;
        float half = height * 0.5f;
        rt.anchorMin = new Vector2(0.5f, heightFraction);
        rt.anchorMax = new Vector2(0.5f, heightFraction);
        rt.pivot = new Vector2(0.5f, 0.5f);
        rt.anchoredPosition = Vector2.zero;
        rt.offsetMin = new Vector2(-(CardWidth * 0.5f - 12f), -half);
        rt.offsetMax = new Vector2((CardWidth * 0.5f - 12f), half);
    }

    /// <summary>
    /// Wires the artwork Image on a card prefab to the matching serialized field on its
    /// impl component. Split out from the layout pass because it has to run after the
    /// Image component exists.
    /// </summary>
    [MenuItem("Tools/WireClientDemoCardArt")]
    public static void WireArt()
    {
        Wire(CarCardPath, "CarCardImpl", "_carImage", "CarArt");
        WireTrackExtras();
    }

    /// <summary>Track cards additionally need the badge and the group used to dim them.</summary>
    private static void WireTrackExtras()
    {
        var root = PrefabUtility.LoadPrefabContents(TrackCardPath);
        try
        {
            var impl = FindImpl(root, "TrackCardImpl");
            if (impl == null) return;

            var so = new SerializedObject(impl);
            var badge = root.transform.Find("ComingSoonBadge");
            var group = root.GetComponent<CanvasGroup>();

            var badgeProp = so.FindProperty("_comingSoonBadge");
            if (badgeProp != null && badge != null)
                badgeProp.objectReferenceValue = badge.gameObject;

            var groupProp = so.FindProperty("_contentGroup");
            if (groupProp != null && group != null)
                groupProp.objectReferenceValue = group;

            so.ApplyModifiedPropertiesWithoutUndo();
            PrefabUtility.SaveAsPrefabAsset(root, TrackCardPath);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    private static void Wire(string path, string implTypeName, string fieldName, string artName)
    {
        var root = PrefabUtility.LoadPrefabContents(path);
        try
        {
            var impl = FindImpl(root, implTypeName);
            var art = root.transform.Find(artName);
            if (impl == null || art == null) return;

            var image = art.GetComponent<Image>();
            if (image == null) return;

            var so = new SerializedObject(impl);
            var prop = so.FindProperty(fieldName);
            if (prop == null)
            {
                Debug.LogError($"[ClientDemoUI] {implTypeName}.{fieldName} not found.");
                return;
            }

            prop.objectReferenceValue = image;
            so.ApplyModifiedPropertiesWithoutUndo();
            PrefabUtility.SaveAsPrefabAsset(root, path);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }
}
