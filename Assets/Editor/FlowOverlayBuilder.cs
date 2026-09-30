// Assets/Editor/FlowOverlayBuilder.cs
//
// Builds the prefab FlowOverlay loads from Resources at runtime.
//
// It lives in Resources rather than being authored into a scene because the overlay has to
// exist before any scene transition and outlive the scenes on both sides of it. Anything
// parented to a scene root would be destroyed by exactly the transition it exists to cover.
//
// Run via Tools/UI/Build Flow Overlay.
using System.IO;
using TMPro;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

public class FlowOverlayBuilder
{
    private const string PrefabPath = "Assets/Resources/FlowOverlay_Prefab.prefab";

    // The transition artwork: the user's own F1 key art, shared with the branding loading
    // screen so the two loading moments read as one product. A bright progress bar and a
    // status line are drawn across the lower-middle of it, which is the dark road area, so
    // they read without a scrim.
    private const string TransitionImage = "Assets/Art/Backgrounds/bg_transition.png";
    private const string FallbackImage = "Assets/Art/Backgrounds/bg_loading_v2.png";

    [MenuItem("Tools/UI/Build Flow Overlay", priority = 75)]
    public static void Build()
    {
        string image = File.Exists(TransitionImage) ? TransitionImage : FallbackImage;
        if (!File.Exists(image))
        {
            Debug.LogError("[FlowOverlay] No transition background found at either " +
                           $"{TransitionImage} or {FallbackImage}.");
            return;
        }
        if (image == FallbackImage)
            Debug.LogWarning($"[FlowOverlay] {TransitionImage} is missing; falling back to " +
                             "the loading screen background for the transition card.");

        var sprite = AssetDatabase.LoadAssetAtPath<Sprite>(image);

        var root = new GameObject("FlowOverlay_Prefab",
                                  typeof(RectTransform), typeof(Canvas),
                                  typeof(CanvasScaler), typeof(GraphicRaycaster),
                                  typeof(F1.UI.FlowOverlay));

        var canvas = root.GetComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        // Above every flow screen, which all sit at the default 0. Without this the
        // transition card renders underneath the screen it is covering.
        canvas.sortingOrder = 100;

        var scaler = root.GetComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);
        scaler.matchWidthOrHeight = 1f;

        var rootRt = (RectTransform)root.transform;
        rootRt.anchorMin = Vector2.zero;
        rootRt.anchorMax = Vector2.one;
        rootRt.offsetMin = Vector2.zero;
        rootRt.offsetMax = Vector2.zero;

        var impl = root.GetComponent<F1.UI.FlowOverlay>();

        // --- Transition card ---
        var transition = NewUI("TransitionGroup", rootRt);
        Stretch(transition);
        var back = NewUI("TransitionBackground", transition);
        Stretch(back);
        var backImage = back.GetComponent<Image>();
        backImage.sprite = sprite;
        backImage.type = Image.Type.Simple;
        backImage.preserveAspect = true;
        backImage.color = Color.white;
        // Must stay a raycast target: while a transition is up, the player should not be
        // able to click a button belonging to the scene underneath.
        backImage.raycastTarget = true;

        var status = NewText("TransitionStatus", transition, "Loading", 30f,
                              new Color(0.92f, 0.94f, 0.97f, 1f));
        Place(status, new Vector2(0.5f, 0.36f), new Vector2(900f, 46f));

        var track = NewUI("TransitionTrack", transition);
        Place(track, new Vector2(0.5f, 0.30f), new Vector2(700f, 14f));
        track.GetComponent<Image>().color = new Color(1f, 1f, 1f, 0.18f);

        var fill = NewUI("TransitionFill", track);
        Stretch(fill);
        var fillImage = fill.GetComponent<Image>();
        fillImage.type = Image.Type.Filled;
        fillImage.fillMethod = Image.FillMethod.Horizontal;
        fillImage.fillOrigin = 0;
        fillImage.fillAmount = 0f;
        // The pack's amber, the same accent as every other call to action.
        fillImage.color = new Color(1f, 0.686f, 0f, 1f);
        fillImage.raycastTarget = false;

        // --- Start countdown ---
        var countdown = NewUI("CountdownGroup", rootRt);
        Stretch(countdown);
        var shade = countdown.GetComponent<Image>();
        // A very light vignette so the number reads over a bright sky without hiding the
        // track the player is about to drive.
        shade.color = new Color(0f, 0f, 0f, 0.18f);
        shade.raycastTarget = false;

        var number = NewText("CountdownNumber", countdown, "3", 190f, Color.white);
        number.GetComponent<TMP_Text>().fontStyle = FontStyles.Bold;
        Place(number, new Vector2(0.5f, 0.5f), new Vector2(700f, 230f), new Vector2(0f, 40f));

        var caption = NewText("CountdownCaption", countdown, "GET READY", 34f,
                               new Color(1f, 0.82f, 0.36f, 1f));
        Place(caption, new Vector2(0.5f, 0.5f), new Vector2(800f, 48f), new Vector2(0f, -110f));

        // The countdown text sits over live 3D, so it takes the same shadowed material the
        // qualifying lap clock uses rather than being plain white on a bright sky.
        var hudMat = AssetDatabase.LoadAssetAtPath<Material>("Assets/Art/UI/HudText.mat");
        if (hudMat != null)
        {
            number.GetComponent<TMP_Text>().fontSharedMaterial = hudMat;
            caption.GetComponent<TMP_Text>().fontSharedMaterial = hudMat;
        }

        Wire(impl, "_transitionGroup", transition.gameObject);
        Wire(impl, "_transitionBackground", backImage);
        Wire(impl, "_transitionFill", fillImage);
        Wire(impl, "_transitionStatus", status);
        Wire(impl, "_countdownGroup", countdown.gameObject);
        Wire(impl, "_countdownNumber", number);
        Wire(impl, "_countdownCaption", caption);

        // Both groups ship hidden. The overlay is created on the very first frame of the
        // game and must not cover the branding screen behind it.
        transition.gameObject.SetActive(false);
        countdown.gameObject.SetActive(false);

        Directory.CreateDirectory("Assets/Resources");
        PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
        Object.DestroyImmediate(root);
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();

        Debug.Log($"[FlowOverlay] Overlay prefab written to {PrefabPath} using {Path.GetFileName(image)}.");
    }

    // --- helpers ---

    private static RectTransform NewUI(string name, RectTransform parent)
    {
        var go = new GameObject(name, typeof(RectTransform), typeof(CanvasRenderer), typeof(Image));
        var rt = (RectTransform)go.transform;
        rt.SetParent(parent, false);
        return rt;
    }

    private static TMP_Text NewText(string name, RectTransform parent, string content,
                                    float size, Color color)
    {
        var go = new GameObject(name, typeof(RectTransform), typeof(CanvasRenderer),
                                typeof(TextMeshProUGUI));
        var rt = (RectTransform)go.transform;
        rt.SetParent(parent, false);
        var tmp = go.GetComponent<TextMeshProUGUI>();
        tmp.text = content;
        tmp.fontSize = size;
        tmp.color = color;
        tmp.alignment = TextAlignmentOptions.Center;
        tmp.raycastTarget = false;
        return tmp;
    }

    private static void Stretch(RectTransform rt)
    {
        rt.anchorMin = Vector2.zero;
        rt.anchorMax = Vector2.one;
        rt.pivot = new Vector2(0.5f, 0.5f);
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;
    }

    private static void Place(Component component, Vector2 anchor, Vector2 size, Vector2? offset = null)
    {
        var rt = (RectTransform)component.transform;
        rt.anchorMin = rt.anchorMax = anchor;
        rt.pivot = anchor;
        rt.sizeDelta = size;
        rt.anchoredPosition = offset ?? Vector2.zero;
    }

    private static void Wire(Object target, string field, Object value)
    {
        var so = new SerializedObject(target);
        var prop = so.FindProperty(field);
        if (prop == null)
        {
            Debug.LogError($"[FlowOverlay] No field '{field}' on {target.GetType().Name}.");
            return;
        }
        prop.objectReferenceValue = value;
        so.ApplyModifiedPropertiesWithoutUndo();
    }
}
