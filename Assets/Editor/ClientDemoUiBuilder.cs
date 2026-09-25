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

    // The columns are a third of the canvas wide and 80% of its height (864 units at the
    // 1920x1080 reference). A column can hold three cards, so the card has to fit three:
    // 3 * 270 + 2 * 10 spacing + 12 padding = 830, inside 864.
    // The width is chosen to match the art: the panel is 284 x 140, and the generated car
    // images are about 1.83:1, so they fill it closely instead of floating small in a wide
    // dark frame. A wider card just adds empty space either side of the picture.
    private const float CardWidth = 300f;
    private const float CardHeight = 270f;
    private const float ArtHeight = 140f;
    private const float CardCornerPad = 8f;

    [MenuItem("Tools/BuildClientDemoUI")]
    public static void Build()
    {
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
        ApplyBackground(TrackScreenPrefabPath, ArtRoot + "Backgrounds/bg_trackselect.png");
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

            // --- Darken the card body so the label text stays legible over artwork ---
            var body = root.GetComponent<Image>();
            if (body != null)
            {
                body.color = new Color(0.06f, 0.07f, 0.09f, 0.96f);
                body.raycastTarget = false;
            }

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

            var artRect = (RectTransform)art;
            artRect.anchorMin = new Vector2(0f, 1f);
            artRect.anchorMax = new Vector2(1f, 1f);
            artRect.pivot = new Vector2(0.5f, 1f);
            artRect.offsetMin = new Vector2(CardCornerPad, -(ArtHeight + CardCornerPad));
            artRect.offsetMax = new Vector2(-CardCornerPad, -CardCornerPad);

            // Artwork must sit behind the text, so make it the first child to draw.
            art.SetSiblingIndex(0);

            // --- Stack the text below the artwork ---
            // The art panel occupies the top ArtHeight of the card, which on a 270 card is
            // everything above anchor 0.452. Every label therefore has to sit below that, or
            // it renders on top of the picture.
            SetAnchor(root.transform, "NameText", 0.385f, 28f);
            SetAnchor(root.transform, "GenText", 0.290f, 22f);
            SetAnchor(root.transform, "ShortCodeText", 0.290f, 22f);
            SetAnchor(root.transform, "CostText", 0.205f, 22f);
            SetAnchor(root.transform, "StatusText", 0.150f, 20f);
            SetAnchor(root.transform, "CardButton", 0.055f, 38f);

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
