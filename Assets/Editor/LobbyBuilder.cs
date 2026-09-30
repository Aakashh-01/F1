// Assets/Editor/LobbyBuilder.cs
//
// Lobby build tooling: the car showcase prefab, and the scene assembly that drops the
// showcase into a selection scene inside the imported garage.
//
// The garage geometry is NOT authored here. It is the purchased "Simple Garage" asset
// (Assets/Simple Garage/), instantiated and scaled. What is authored here is the part that
// has to agree with our lobby: where the car sits, where the camera stands, and how the
// room is lit.
//
// Findings from the imported asset that shaped this file, all measured rather than assumed:
//
//   * All 84 of its materials are already URP/Lit. No conversion, no magenta surfaces.
//   * The interior measures 6.51 x 4.04 x 6.76 m. Our car is 6.3 m long, so the car would
//     fill the room wall to wall with ~10 cm to spare and the camera could not back off far
//     enough for a three-quarter shot. Hence GarageScale below.
//   * The asset ships NO lights - not in the scene, not across its 60 renderers. It reads at
//     all only because of dark ambient plus emissive ceiling panels. A glossy car drops into
//     that flat and floats, so the lighting rig here is not optional dressing.
//   * The floor top sits at y = -0.03 at native scale, so the garage is nudged up to put
//     the floor exactly at y = 0 and the car stands on it rather than hovering.
//
// Re-runnable via Tools > Lobby > ...
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using UnityEngine.SceneManagement;
using TMPro;

public class LobbyBuilder
{
    private const string CarShowcasePath = "Assets/Prefabs/CarShowcase_Prefab.prefab";
    private const string CarBodyPath = "Assets/Prefabs/F1_Body.prefab";
    private const string GaragePrefabPath = "Assets/Simple Garage/Prefabs/Garage.prefab";
    private const string LobbyScreenPrefabPath = "Assets/Prefabs/LobbyHubScreen_Prefab.prefab";

    /// <summary>
    /// The scene the garage rig is READ from, and the only hand-authored flow scene.
    ///
    /// It used to be 10_CarSelectionScene, which was deleted when car selection became a
    /// UI state of the lobby. The lobby now holds the one tuned garage rig, and the wing
    /// scene copies from it so the two never drift.
    ///
    /// Opening this scene is safe — it is a normal scene load and does not discard unsaved
    /// changes — but it IS the user's hand-placed scene, so nothing here may save over it.
    /// Only ever read from it.
    /// </summary>
    private const string RigSourceScenePath = "Assets/Scenes/05_LobbyScene.unity";

    private const string WingScenePath = "Assets/Scenes/30_WingSetupScene.unity";
    private const string LobbyScenePath = "Assets/Scenes/05_LobbyScene.unity";

    /// <summary>
    /// 1.45 takes the interior from 6.5 x 6.8 m to roughly 9.4 x 9.8 m, which is what a
    /// 6.3 m car needs to have air around it and still let the camera stand off far enough
    /// for a three-quarter composition. The cost is that the workbench, skateboard and
    /// lockers scale up with the room; at 45% they are only wrong if you walk right up to them.
    /// </summary>
    private const float GarageScale = 1.45f;

    /// <summary>
    /// Camera framing, sized against the real interior rather than the asset's outer shell.
    ///
    /// The walkable room is 8.3 x 8.8 m, so the usable half-extents are about 4.1 x 4.4 m
    /// and the longest clear sightline — corner to centre — is roughly 6.0 m. The camera
    /// takes ~5.1 m of that, leaving enough clearance that the near plane is nowhere near
    /// the plaster, and 42 degrees of vertical FOV is what puts the whole car in frame at
    /// that distance with air around it. The first attempt used the outer shell's half-
    /// extents and landed 2 cm from the inside of a wall.
    ///
    /// A corner is used deliberately: it is the longest sightline available, and it
    /// presents the car three-quarter without having to angle it as hard.
    /// </summary>
    private static readonly Vector3 CameraPosition = new Vector3(-3.45f, 1.62f, 3.70f);
    private static readonly Vector3 CameraLookAt = new Vector3(0f, 0.30f, 0f);
    private const float CameraFov = 42f;

    /// <summary>
    /// Builds the showcase prefab and the wing scene. Deliberately does NOT touch the
    /// lobby scene — that one is hand-authored, and this builder is a scaffold for scenes
    /// that have never been built.
    /// </summary>
    [MenuItem("Tools/Lobby/Build All", priority = 0)]
    public static void BuildAll()
    {
        BuildCarShowcasePrefab();
        BuildWingLobbyScene();
        AssetDatabase.SaveAssets();
        Debug.Log("[Lobby] Built showcase and wing lobby scene.");
    }

    // ==================================================================================
    // Lobby screen prefab
    // ==================================================================================

    /// <summary>
    /// Builds the lobby screen: the currency readout, the car tile strip and Next, over
    /// whatever 3D scene hosts them.
    ///
    /// The root deliberately carries NO Image. The other screens paint an opaque background
    /// sprite, but here the garage behind is the point - an opaque root would curtain it.
    /// With no root Graphic the GraphicRaycaster still hits the buttons (they have their own
    /// Images) and everything else falls through to the 3D, which is the behaviour a lobby
    /// wants.
    ///
    /// Re-runnable: it edits the prefab in place.
    /// </summary>
    [MenuItem("Tools/Lobby/Build Lobby Hub Screen Prefab", priority = 10)]
    public static void BuildLobbyHubScreenPrefab()
    {
        SlimUiSkin.EnsureLoaded();
        var root = new GameObject("LobbyHubScreen_Prefab",
                                  typeof(RectTransform), typeof(Canvas),
                                  typeof(UnityEngine.UI.CanvasScaler),
                                  typeof(UnityEngine.UI.GraphicRaycaster),
                                  typeof(F1.UI.LobbyHubScreenImpl));

        var rt = (RectTransform)root.transform;
        var canvas = root.GetComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;

        var scaler = root.GetComponent<UnityEngine.UI.CanvasScaler>();
        scaler.uiScaleMode = UnityEngine.UI.CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);
        scaler.screenMatchMode = UnityEngine.UI.CanvasScaler.ScreenMatchMode.MatchWidthOrHeight;
        scaler.matchWidthOrHeight = 1f;

        var impl = root.GetComponent<F1.UI.LobbyHubScreenImpl>();

        var selection = BuildSelectionPanel(rt);

        Wire(impl, "_currencyText", selection.transform.Find("CurrencyText")?.GetComponent<TMPro.TextMeshProUGUI>());
        Wire(impl, "_nextButton", selection.transform.Find("NextButton")?.GetComponent<UnityEngine.UI.Button>());

        // The strip and the card that fills it.
        Wire(impl, "_tileStrip", selection.transform.Find("TileStrip") as RectTransform);
        var cardPrefab = AssetDatabase.LoadAssetAtPath<F1.UI.CarCardImpl>("Assets/Prefabs/CarCard_Prefab.prefab");
        if (cardPrefab == null)
            Debug.LogError("[Lobby] CarCard_Prefab not found; the tile strip will stay empty.");
        Wire(impl, "_carCardPrefab", cardPrefab);

        // Clicks are wired after the first save, in WireClicks().

        // One panel, always up. The lobby opens straight onto the car tiles.
        selection.SetActive(true);

        EnsureFolder("Assets/Prefabs");
        PrefabUtility.SaveAsPrefabAsset(root, LobbyScreenPrefabPath);
        Object.DestroyImmediate(root);

        // Clicks are wired in a SECOND pass, against the saved prefab rather than the
        // throwaway scene object. UnityEventTools only writes a persistent listener when its
        // target is itself a persistent asset; adding them to a temp GameObject before the
        // first save silently produces a prefab whose buttons are all inert. Saving first and
        // reopening the contents is what makes the references stick.
        WireClicks();

        Debug.Log($"[Lobby] Hub screen prefab written to {LobbyScreenPrefabPath}");
    }

    /// <summary>Reopens the saved prefab and attaches the button handler.</summary>
    private static void WireClicks()
    {
        var contents = PrefabUtility.LoadPrefabContents(LobbyScreenPrefabPath);
        try
        {
            var impl = contents.GetComponent<F1.UI.LobbyHubScreenImpl>();
            if (impl == null)
            {
                Debug.LogError("[Lobby] Saved hub prefab has no LobbyHubScreenImpl; " +
                               "cannot wire buttons.");
                return;
            }

            var selection = contents.transform.Find("SelectionPanel");
            if (selection == null)
            {
                Debug.LogError("[Lobby] Saved hub prefab is missing its panel root.");
                return;
            }

            ClearListeners(selection, "NextButton");

            Click(selection, "NextButton", impl, nameof(F1.UI.LobbyHubScreenImpl.OnNextClicked));

            PrefabUtility.SaveAsPrefabAsset(contents, LobbyScreenPrefabPath);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(contents);
        }
    }

    /// <summary>
    /// Strips existing listeners so re-running the builder does not stack duplicates — a
    /// second Next press would otherwise fire the handler twice. RemovePersistentListener
    /// only takes an index, so this peels from the front until none are left.
    /// </summary>
    private static void ClearListeners(Transform panel, params string[] buttonNames)
    {
        foreach (var name in buttonNames)
        {
            var button = panel.Find(name)?.GetComponent<UnityEngine.UI.Button>();
            if (button == null) continue;
            while (button.onClick.GetPersistentEventCount() > 0)
                UnityEditor.Events.UnityEventTools.RemovePersistentListener(button.onClick, 0);
        }
    }

    private static readonly Color PanelColor = new Color(0.04f, 0.05f, 0.08f, 0.82f);
    private static readonly Color ButtonColor = new Color(0.10f, 0.12f, 0.17f, 0.95f);
    private static readonly Color AccentColor = new Color(1f, 0.82f, 0.25f, 1f);
    private static readonly Color TextColor = new Color(0.92f, 0.93f, 0.95f, 1f);

    /// <summary>
    /// The lobby screen: the currency readout, the car tile strip and Next.
    ///
    /// There used to be two panels here — this one, and a hub carrying currency, settings,
    /// tasks and a START RACE button that swapped to it. The hub is gone, so there is a
    /// single panel now and it is the first thing the player sees in the lobby.
    /// </summary>
    private static GameObject BuildSelectionPanel(RectTransform root)
    {
        var panel = NewUiObject("SelectionPanel", root);
        Stretch(panel);
        var body = panel.gameObject.AddComponent<UnityEngine.UI.Image>();
        body.color = new Color(0f, 0f, 0f, 0f); // transparent: the garage is the backdrop
        body.raycastTarget = false;

        // The currency readout keeps the exact placement it had on the hub panel — top right,
        // 34pt, amber. That placement is the user's, laid out by hand in the editor and
        // captured in Assets/Scenes/Screenshot 2026-09-27 032721.png: the light panels were
        // found by thresholding, then identified by their mean colour (#EAEBEC for the white
        // pair, #F3E2B1 for the amber one) and converted from pixels to canvas units, rather
        // than eyeballed off the image. Moving it here rather than re-laying it out is what
        // keeps the screen from shifting under a change that should be invisible.
        var currency = NewText("CurrencyText", panel, "0", 34f, TextAlignmentOptions.Right);
        PlaceAt(currency, new Vector2(1f, 1f), new Vector2(280f, 48f), new Vector2(60f, 70f));
        currency.GetComponent<TMPro.TextMeshProUGUI>().color = AccentColor;

        // The tile strip. 300 tall to clear the 280-tall card, and inset 80 either side,
        // which leaves 1760 of run for six 250-wide tiles at 22 spacing (1610). Lifted to 56
        // off the floor so the Select buttons are not sitting on the screen edge.
        var strip = NewUiObject("TileStrip", panel);
        var stripRt = (RectTransform)strip.transform;
        stripRt.anchorMin = new Vector2(0f, 0f);
        stripRt.anchorMax = new Vector2(1f, 0f);
        stripRt.pivot = new Vector2(0.5f, 0f);
        stripRt.offsetMin = new Vector2(80f, 56f);
        stripRt.offsetMax = new Vector2(-80f, 356f);
        var group = strip.gameObject.AddComponent<UnityEngine.UI.HorizontalLayoutGroup>();
        group.spacing = 22f;
        group.childAlignment = TextAnchor.MiddleCenter;
        // The card carries an explicit sizeDelta. Letting the group control width or height
        // would override it and the tiles would drift away from the 250x280 the builder set.
        group.childControlWidth = false;
        group.childControlHeight = false;
        group.childForceExpandWidth = false;
        group.childForceExpandHeight = false;

        // Bottom-right, matching where START RACE sat, so the primary action stays in the
        // corner the camera is hand-framed around — the car sits low and forward and a
        // centred button would land on its nose.
        var next = NewButton("NextButton", panel, "NEXT", 30f, primary: true);
        PlaceAt(next, new Vector2(1f, 0f), new Vector2(240f, 72f), new Vector2(60f, 316f));

        // Disabled until a tile is chosen. interactable is what actually blocks the click;
        // the disabledColor in the button's ColorBlock is what says so.
        next.GetComponent<UnityEngine.UI.Button>().interactable = false;

        return panel.gameObject;
    }

    // --- small UI helpers ---

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

    private static RectTransform NewText(string name, Transform parent, string content,
                                         float size, TextAlignmentOptions align)
    {
        var rt = NewUiObject(name, parent);
        var text = rt.gameObject.AddComponent<TMPro.TextMeshProUGUI>();
        text.text = content;
        text.fontSize = size;
        text.alignment = align;
        text.color = TextColor;
        text.raycastTarget = false;
        return rt;
    }

    /// <summary>
    /// Creates one of the lobby's buttons, already wearing the SlimUI skin.
    ///
    /// `primary` picks the amber accent for the two calls to action (Start Race, Next) and
    /// leaves the rest neutral. The button is three layers rather than one — an opaque light
    /// fill, the pack's outline sprite on top of it, and the label above both — because the
    /// pack's button sprite is an OUTLINE with a transparent interior. Tinting that alone
    /// leaves an empty rectangle over the dark garage with dark label ink on it, which is
    /// dark on dark and unreadable.
    /// </summary>
    private static RectTransform NewButton(string name, Transform parent, string label, float size,
                                           bool primary = false)
    {
        var rt = NewUiObject(name, parent);
        var image = rt.gameObject.AddComponent<UnityEngine.UI.Image>();
        image.raycastTarget = true;

        var button = rt.gameObject.AddComponent<UnityEngine.UI.Button>();
        button.targetGraphic = image;
        SlimUiSkin.ApplyButtonVisual(button, primary);

        // A GameObject may carry only one Graphic, so the label is a child, not a second
        // Image on the same object. Unity refuses the second Graphic outright.
        var text = NewText("Label", rt, label, size, TextAlignmentOptions.Center);
        Stretch(text);
        // Dark ink, because the button now has a light fill behind it.
        text.GetComponent<TMPro.TextMeshProUGUI>().color = SlimUiSkin.Ink;

        return rt;
    }

    /// <summary>
    /// Pins a box to a corner or edge, inset from it.
    ///
    /// anchor picks the corner or edge, size is the box, and offset is how far in from that
    /// edge to sit. pivot is set equal to the anchor so anchoredPosition measures cleanly:
    /// with pivot == anchor, a negative value pulls the box back toward the edge and a
    /// positive one pushes it into the screen interior. Setting offsetMin/offsetMax instead
    /// would mean authoring the rect by its two opposite corners, where a typo silently
    /// produces a zero-size box.
    /// </summary>
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

    // ==================================================================================
    // Garage rig copy
    // ==================================================================================

    /// <summary>Everything the lobby needs to reproduce the car scene's garage rig.</summary>
    public struct GarageRig
    {
        public Vector3 GaragePos, CarPos, CamPos, KeyPos, FillPos;
        public Quaternion GarageRot, CarRot, CamRot, KeyRot, FillRot;
        public Vector3 GarageScale, CarScale;

        public float CamFov, CamNear, CamFar;
        public Color CamBackground;
        public CameraClearFlags CamClear;

        public LightType KeyType, FillType;
        public Color KeyColor, FillColor;
        public float KeyIntensity, KeyRange, KeyAngle, KeyInnerAngle, KeyShadowStrength, KeyShadowBias;
        public float FillIntensity;
        public LightShadows KeyShadows;

        public AmbientMode AmbientMode;
        public Color AmbientSky, AmbientEquator, AmbientGround;
    }

    /// <summary>
    /// Reads the hand-tuned garage rig out of the car selection scene.
    ///
    /// Reads the live values rather than re-deriving them, because the car, camera and key
    /// light in 10_CarSelectionScene were placed and adjusted by hand; re-deriving them from
    /// the constants in this file would quietly undo that. Reading at build time also means
    /// re-running after a re-tune keeps the two scenes in agreement instead of letting them
    /// drift apart.
    ///
    /// Capture is deliberately separate from application. Opening the source scene makes it
    /// the active one, so anything that writes scene-level settings — RenderSettings among
    /// them — has to happen after the caller has re-opened the target as active, or the
    /// write lands on the wrong scene.
    ///
    /// The source is saved before being opened, so this can never discard unsaved work.
    /// </summary>
    public static GarageRig CaptureGarageRig()
    {
        if (EditorSceneManager.GetActiveScene().isDirty)
            EditorSceneManager.SaveOpenScenes();

        var source = EditorSceneManager.OpenScene(RigSourceScenePath, OpenSceneMode.Single);

        Transform Find(string name)
        {
            foreach (var root in source.GetRootGameObjects())
                if (root.name == name) return root.transform;
            return null;
        }

        var garage = Find("Garage");
        var car = Find("CarShowcase");
        if (garage == null || car == null)
        {
            Debug.LogError($"[Lobby] {RigSourceScenePath} has no Garage/CarShowcase. " +
                           "Build the car lobby scene first.");
            return default;
        }

        var rig = new GarageRig
        {
            GaragePos = garage.localPosition,
            GarageRot = garage.localRotation,
            GarageScale = garage.localScale,
            CarPos = car.localPosition,
            CarRot = car.localRotation,
            CarScale = car.localScale,

            // Ambient is scene-level, not an object, so it has to be read explicitly or the
            // lobby renders with the default skybox ambient instead of the tuned low one.
            AmbientMode = RenderSettings.ambientMode,
            AmbientSky = RenderSettings.ambientSkyColor,
            AmbientEquator = RenderSettings.ambientEquatorColor,
            AmbientGround = RenderSettings.ambientGroundColor,
        };

        var camT = Find("LobbyCamera");
        if (camT != null)
        {
            var c = camT.GetComponent<Camera>();
            if (c != null)
            {
                rig.CamPos = camT.localPosition;
                rig.CamRot = camT.localRotation;
                rig.CamFov = c.fieldOfView;
                rig.CamNear = c.nearClipPlane;
                rig.CamFar = c.farClipPlane;
                rig.CamBackground = c.backgroundColor;
                rig.CamClear = c.clearFlags;
            }
        }

        var keyT = Find("LobbyKeyLight");
        if (keyT != null)
        {
            var l = keyT.GetComponent<Light>();
            if (l != null)
            {
                rig.KeyPos = keyT.localPosition;
                rig.KeyRot = keyT.localRotation;
                rig.KeyType = l.type;
                rig.KeyColor = l.color;
                rig.KeyIntensity = l.intensity;
                rig.KeyRange = l.range;
                rig.KeyAngle = l.spotAngle;
                rig.KeyInnerAngle = l.innerSpotAngle;
                rig.KeyShadows = l.shadows;
                rig.KeyShadowStrength = l.shadowStrength;
                rig.KeyShadowBias = l.shadowBias;
            }
        }

        var fillT = Find("LobbyFillLight");
        if (fillT != null)
        {
            var l = fillT.GetComponent<Light>();
            if (l != null)
            {
                rig.FillPos = fillT.localPosition;
                rig.FillRot = fillT.localRotation;
                rig.FillType = l.type;
                rig.FillColor = l.color;
                rig.FillIntensity = l.intensity;
            }
        }

        return rig;
    }

    /// <summary>
    /// Instantiates the captured rig into the given scene and applies its settings.
    ///
    /// Idempotent. Phase3SceneBuilder.CreateScene OPENS an existing scene rather than
    /// replacing it, so re-running this builder lands in a scene that already holds a rig.
    /// Without the clear below, each run added another Garage, CarShowcase, camera and pair
    /// of lights — which is exactly what happened: five stacked copies. EnsureRoot's
    /// by-name dedupe does not help here because the rig is created with InstantiatePrefab
    /// rather than through EnsureRoot.
    /// </summary>
    public static void ApplyGarageRig(Scene target, GarageRig rig,
                                      CameraPlacement? camera = null)
    {
        ClearOwnedRig(target);

        var garagePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(GaragePrefabPath);
        var showcasePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(CarShowcasePath);
        if (garagePrefab == null || showcasePrefab == null)
        {
            Debug.LogError("[Lobby] Garage or showcase prefab missing.");
            return;
        }

        var garage = (GameObject)PrefabUtility.InstantiatePrefab(garagePrefab, target);
        garage.name = "Garage";
        garage.transform.localPosition = rig.GaragePos;
        garage.transform.localRotation = rig.GarageRot;
        garage.transform.localScale = rig.GarageScale;

        var car = (GameObject)PrefabUtility.InstantiatePrefab(showcasePrefab, target);
        car.name = "CarShowcase";
        car.transform.localPosition = rig.CarPos;
        car.transform.localRotation = rig.CarRot;
        car.transform.localScale = rig.CarScale;

        if (rig.CamFov > 0f || camera.HasValue)
        {
            var go = new GameObject("LobbyCamera", typeof(Camera));
            SceneManager.MoveGameObjectToScene(go, target);
            go.tag = "MainCamera";

            var cam = go.GetComponent<Camera>();
            cam.nearClipPlane = rig.CamNear > 0f ? rig.CamNear : 0.1f;
            cam.farClipPlane = rig.CamFar > 0f ? rig.CamFar : 60f;
            cam.clearFlags = rig.CamClear;
            cam.backgroundColor = rig.CamBackground;
            if (go.GetComponent<UnityEngine.Rendering.Universal.UniversalAdditionalCameraData>() == null)
                go.AddComponent<UnityEngine.Rendering.Universal.UniversalAdditionalCameraData>();

            // An override replaces the captured camera wholesale rather than nudging it: the
            // wing screen needs a different vantage, not a corrected version of the same one.
            if (camera.HasValue)
            {
                cam.fieldOfView = camera.Value.Fov;
                go.transform.position = camera.Value.Position;
                go.transform.rotation = Quaternion.LookRotation(
                    (camera.Value.LookAt - camera.Value.Position).normalized, Vector3.up);
            }
            else
            {
                cam.fieldOfView = rig.CamFov;
                go.transform.localPosition = rig.CamPos;
                go.transform.localRotation = rig.CamRot;
            }
        }

        if (rig.KeyType != default || rig.KeyIntensity > 0f)
            ApplyLight(target, "LobbyKeyLight", rig.KeyPos, rig.KeyRot, rig.KeyType,
                       rig.KeyColor, rig.KeyIntensity, rig.KeyRange, rig.KeyAngle,
                       rig.KeyInnerAngle, rig.KeyShadows, rig.KeyShadowStrength, rig.KeyShadowBias);

        if (rig.FillIntensity > 0f)
            ApplyLight(target, "LobbyFillLight", rig.FillPos, rig.FillRot, rig.FillType,
                       rig.FillColor, rig.FillIntensity, 0f, 0f, 0f,
                       LightShadows.None, 0f, 0f);

        // Only safe now: the caller has the target scene open and active.
        RenderSettings.ambientMode = rig.AmbientMode;
        RenderSettings.ambientSkyColor = rig.AmbientSky;
        RenderSettings.ambientEquatorColor = rig.AmbientEquator;
        RenderSettings.ambientGroundColor = rig.AmbientGround;
    }

    private static void ApplyLight(Scene target, string name, Vector3 pos, Quaternion rot,
                                   LightType type, Color color, float intensity, float range,
                                   float angle, float innerAngle, LightShadows shadows,
                                   float shadowStrength, float shadowBias)
    {
        var go = new GameObject(name, typeof(Light));
        SceneManager.MoveGameObjectToScene(go, target);

        var l = go.GetComponent<Light>();
        l.type = type;
        l.color = color;
        l.intensity = intensity;
        l.range = range;
        l.spotAngle = angle;
        l.innerSpotAngle = innerAngle;
        l.shadows = shadows;
        l.shadowStrength = shadowStrength;
        l.shadowBias = shadowBias;

        go.transform.localPosition = pos;
        go.transform.localRotation = rot;
    }

    /// <summary>
    /// Connects a button to a public method on the screen impl, as a persistent listener.
    ///
    /// The listener has to be persistent, not a runtime AddListener: it is authored here and
    /// has to survive being written into the prefab asset. A runtime listener is lost the
    /// moment the prefab is saved and reloaded, which is exactly what left every button on
    /// this screen inert the first time round.
    /// </summary>
    /// <summary>
    /// Deletes every previously-applied rig object in the scene, however many rounds of the
    /// builder produced them. Only names this builder owns are touched, so the screen prefab,
    /// the FlowSceneHost and the EventSystem are left alone.
    /// </summary>
    private static void ClearOwnedRig(Scene target)
    {
        var owned = new HashSet<string>
        {
            "Garage", "CarShowcase", "LobbyCamera", "LobbyKeyLight", "LobbyFillLight"
        };

        var doomed = new List<GameObject>();
        foreach (var root in target.GetRootGameObjects())
            if (owned.Contains(root.name)) doomed.Add(root);

        foreach (var go in doomed) Object.DestroyImmediate(go);

        if (doomed.Count > 0)
            Debug.Log($"[Lobby] Cleared {doomed.Count} previously-applied rig object(s) before rebuild.");
    }

    private static void Click(Transform panel, string buttonName, UnityEngine.Object target, string method)
    {
        var button = panel.Find(buttonName)?.GetComponent<UnityEngine.UI.Button>();
        if (button == null)
        {
            Debug.LogError($"[Lobby] No button '{buttonName}' to wire to {method}.");
            return;
        }

        System.Reflection.MethodInfo info = target.GetType().GetMethod(method,
            System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public);
        if (info == null)
        {
            Debug.LogError($"[Lobby] {target.GetType().Name}.{method} not found.");
            return;
        }

        // AddVoidPersistentListener takes a UnityAction rather than (target, methodName);
        // building it from the reflected MethodInfo keeps the two in step, so a renamed
        // handler fails loudly here instead of silently producing an unclickable button.
        var call = (UnityEngine.Events.UnityAction)System.Delegate.CreateDelegate(
            typeof(UnityEngine.Events.UnityAction), target, info);
        UnityEditor.Events.UnityEventTools.AddVoidPersistentListener(button.onClick, call);
    }

    private static void EnsureFolder(string path)
    {
        if (AssetDatabase.IsValidFolder(path)) return;
        var parent = Path.GetDirectoryName(path)?.Replace('\\', '/');
        var leaf = Path.GetFileName(path);
        if (!string.IsNullOrEmpty(parent) && !AssetDatabase.IsValidFolder(parent))
            EnsureFolder(parent);
        // Re-check: the recursive call may have been the one that created it, and
        // CreateFolder on an existing folder logs "Failed to create folder".
        if (AssetDatabase.IsValidFolder(path)) return;
        AssetDatabase.CreateFolder(parent, leaf);
    }

    private static void Wire(Object target, string field, Object value)
    {
        var so = new SerializedObject(target);
        var prop = so.FindProperty(field);
        if (prop == null)
        {
            Debug.LogError($"[Lobby] {target.GetType().Name}.{field} not found.");
            return;
        }
        prop.objectReferenceValue = value;
        so.ApplyModifiedPropertiesWithoutUndo();
    }

    // ==================================================================================
    // Scene assembly
    // ==================================================================================

    [MenuItem("Tools/Lobby/Build Wing Setup Lobby", priority = 41)]
    public static void BuildWingLobbyScene() =>
        BuildLobbyScene(WingScenePath, "CarShowcase", WingCamera);

    /// <summary>
    /// The wing screen's camera: a close rear three-quarter on the rear wing.
    ///
    /// It cannot reuse the lobby's camera. That one frames the whole car from the front,
    /// where the rear wing is small and self-occluded — and a wing-angle change is only
    /// about 0.19 m of travel on a 4.7 m car, so at that framing the two setups look
    /// identical. Getting close is what makes a plausible 16-degree difference read.
    ///
    /// It must sit INSIDE the room. The interior is only 8.3 x 8.8 m, so a camera at
    /// z = -4.6 ends up behind the roller door and renders the outside of the wall; the
    /// prototype position below is about 2 m behind the car, inside the shell.
    /// </summary>
    private static readonly CameraPlacement WingCamera = new CameraPlacement(
        position: new Vector3(1.55f, 1.32f, -3.15f),
        lookAt: new Vector3(0.50f, 0.92f, -1.64f),
        fov: 46f);

    /// <summary>Where a lobby camera stands and what it points at.</summary>
    public struct CameraPlacement
    {
        public readonly Vector3 Position;
        public readonly Vector3 LookAt;
        public readonly float Fov;

        public CameraPlacement(Vector3 position, Vector3 lookAt, float fov)
        {
            Position = position;
            LookAt = lookAt;
            Fov = fov;
        }
    }

    /// <summary>
    /// Assembles a lobby scene — but ONLY if it is not already assembled.
    ///
    /// This used to delete and rebuild the garage, car, camera and lights every run, which
    /// was fine while those values came from constants in this file. They do not any more:
    /// the lobby's car, camera and key light are hand-placed and tuned, and re-running
    /// would silently destroy that work. So the rig is READ from the lobby and applied to a
    /// different scene, and this refuses to touch one that is already built.
    /// </summary>
    private static void BuildLobbyScene(string scenePath, string showcaseName,
                                        CameraPlacement? cameraOverride = null)
    {
        var garagePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(GaragePrefabPath);
        var showcasePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(CarShowcasePath);
        if (garagePrefab == null)
        {
            Debug.LogError($"[Lobby] Garage prefab missing at {GaragePrefabPath}.");
            return;
        }
        if (showcasePrefab == null)
        {
            Debug.LogError("[Lobby] Run 'Build Car Showcase Prefab' first.");
            return;
        }

        // Read the tuned rig BEFORE opening the target, because opening the source makes it
        // the active scene and any scene-level write would then land there instead.
        var rig = CaptureGarageRig();
        var scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);

        foreach (var root in scene.GetRootGameObjects())
        {
            if (root.name == "Garage" || root.name == showcaseName || root.name == "LobbyCamera")
            {
                Debug.Log($"[Lobby] {Path.GetFileName(scenePath)} is already assembled " +
                          $"(found '{root.name}'). Leaving it alone. Delete those objects by " +
                          "hand to rebuild.");
                return;
            }
        }

        // ApplyGarageRig already creates the key and fill lights from the captured rig, so
        // BuildLighting must NOT run here as well — it used to, which left the scene with
        // two of each and double the light.
        ApplyGarageRig(scene, rig, cameraOverride);
        BuildAmbient();
        OpenUpTheBackdrop(scene);

        // Only the wing scene gets the driver. The lobby's car is a static showcase, and
        // rigging the wing there would add a pivot to a scene that has no wing UI.
        if (cameraOverride.HasValue)
        {
            DisableScreenBackdrop("Assets/Prefabs/WingSetupScreen_Prefab.prefab");
            AddWingDriver(scene, showcaseName);
            RemoveDuplicateScreen(scene, "WingSetupScreen");
        }

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Debug.Log($"[Lobby] {Path.GetFileName(scenePath)} assembled.");
    }

    /// <summary>
    /// Drops the garage in and centres it on the origin, floor exactly at y = 0.
    ///
    /// Centring uses the FLOOR, not the overall bounds, and that distinction matters. The
    /// asset's outermost renderer ("Exterior", 9.45 x 9.81 m) is the whole shell including
    /// the outside of the walls and the roof; the walkable interior is the "Floor" at
    /// 8.32 x 8.81 m, and the ceiling is at 3.58 m, not the 5.86 m the outer bounds imply.
    /// Sizing the camera off the outer bounds put it 2 cm from the inside of the wall.
    ///
    /// The room also is not centred on its own origin — its floor centre sits at (-0.15,
    /// 0.97) — so an uncentred drop leaves the camera addressing a corner.
    ///
    /// Every offset is derived from the instantiated bounds rather than hard-coded, so
    /// changing GarageScale cannot quietly reintroduce a floating car or a buried camera.
    /// </summary>
    private static Bounds PlaceGarage(Scene scene, GameObject garagePrefab)
    {
        var garage = (GameObject)PrefabUtility.InstantiatePrefab(garagePrefab, scene);
        garage.name = "Garage";
        garage.transform.localPosition = Vector3.zero;
        garage.transform.localRotation = Quaternion.identity;
        garage.transform.localScale = Vector3.one * GarageScale;

        var renderers = garage.GetComponentsInChildren<Renderer>(true);
        if (renderers.Length == 0)
        {
            Debug.LogError("[Lobby] Garage has no renderers; cannot derive its interior.");
            return new Bounds();
        }

        Bounds floor = default;
        bool haveFloor = false;
        foreach (var r in renderers)
        {
            if (r.name != "Floor") continue;
            if (!haveFloor) { floor = r.bounds; haveFloor = true; }
            else floor.Encapsulate(r.bounds);
        }

        if (!haveFloor)
        {
            Debug.LogWarning("[Lobby] No renderer named 'Floor'; falling back to overall bounds.");
            floor = renderers[0].bounds;
            foreach (var r in renderers) floor.Encapsulate(r.bounds);
        }

        // Shift so the floor's centre is the origin and its top surface is y = 0.
        garage.transform.localPosition = new Vector3(
            -floor.center.x, -floor.max.y, -floor.center.z);

        Debug.Log($"[Lobby] Garage centred. Interior {floor.size.x:F1} x {floor.size.z:F1} m, "
                  + $"half-extents {floor.size.x * 0.5f:F1} x {floor.size.z * 0.5f:F1} m.");
        return floor;
    }

    /// <summary>
    /// The car, standing still at a fixed three-quarter angle.
    ///
    /// There is no turntable and no pivot. The car is presented, not operated: it sits at a
    /// chosen yaw and the player looks at it. That is also why the framing can be composed
    /// for this one pose rather than for the worst case — a spinning car has to be framed so
    /// it still fits when it is side-on and at its widest, which forces a wide lens and
    /// leaves the car small in frame. Fixed, it can be shot the way the reference is.
    /// </summary>
    private const float CarYaw = -34f;

    private static void PlaceCar(Scene scene, GameObject showcasePrefab, string name)
    {
        var car = (GameObject)PrefabUtility.InstantiatePrefab(showcasePrefab, scene);
        car.name = name;
        car.transform.localPosition = Vector3.zero;
        car.transform.localRotation = Quaternion.Euler(0f, CarYaw, 0f);
        car.transform.localScale = Vector3.one;
        Undo.RegisterCreatedObjectUndo(car, "Lobby car showcase");

        // Stand the car ON the floor rather than at y = 0.
        //
        // The showcase's Car_Model sits at local y = 0.337, but its mesh geometry reaches
        // down to y = -0.891 — the model origin is up inside the bodywork, not at the
        // contact patch. Dropping the root at zero therefore buries the car up to its
        // shoulder and leaves a sliver showing, which reads as a speck on the floor rather
        // than a car. The racing scenes never hit this because PlayerCarSpawner positions
        // the body; a lobby has to stand it up itself.
        //
        // Derived from bounds rather than hard-coded, so a re-export of the model cannot
        // quietly reintroduce a buried car. Yaw is applied first, but a rotation about Y
        // cannot change the vertical extent, so the order does not matter here.
        var renderers = car.GetComponentsInChildren<Renderer>(true);
        if (renderers.Length == 0)
        {
            Debug.LogError("[Lobby] Car showcase has no renderers; cannot stand it on the floor.");
            return;
        }

        var bounds = renderers[0].bounds;
        foreach (var r in renderers) bounds.Encapsulate(r.bounds);

        var p = car.transform.localPosition;
        car.transform.localPosition = new Vector3(p.x, p.y - bounds.min.y, p.z);
        Debug.Log($"[Lobby] Car raised {bounds.min.y:F3} m to stand on the floor; "
                  + $"car is {bounds.size.z:F1} m long, roof at {bounds.max.y - bounds.min.y:F1} m.");
    }

    /// <summary>
    /// Puts the wing driver on the car. The rig itself is deferred to Awake because
    /// Transform.SetParent is silently refused on these objects in the editor.
    /// </summary>
    private static void AddWingDriver(Scene scene, string showcaseName)
    {
        var car = scene.GetRootGameObjects().FirstOrDefault(r => r.name == showcaseName);
        if (car == null) return;
        if (car.GetComponent<F1.Lobby.WingAngleDriver>() != null) return;
        car.AddComponent<F1.Lobby.WingAngleDriver>();
        Debug.Log("[Lobby] WingAngleDriver added to the wing scene's car (rigs on Awake).");
    }

    /// <summary>
    /// Disables a screen prefab's opaque backdrop, so a 3D scene behind it shows through.
    ///
    /// This has to be done on the PREFAB, not on an instance in the scene. The FlowSceneHost
    /// instantiates its own copy at runtime, so editing the scene's copy only ever affected
    /// a duplicate that was not the one actually being displayed — which is how the wing
    /// screen ended up painting a flat dark rectangle over the whole garage.
    /// </summary>
    private static void DisableScreenBackdrop(string prefabPath)
    {
        if (!File.Exists(ToAbsolute(prefabPath)))
        {
            Debug.LogError($"[Lobby] Screen prefab missing at {prefabPath}.");
            return;
        }

        var contents = PrefabUtility.LoadPrefabContents(prefabPath);
        try
        {
            int disabled = 0;
            foreach (var canvas in contents.GetComponentsInChildren<Canvas>(true))
            {
                foreach (var graphic in canvas.GetComponentsInChildren<UnityEngine.UI.Graphic>(true))
                {
                    bool isBackdrop = graphic.gameObject == canvas.gameObject
                                   || graphic.gameObject.name == "BackdropScrim";
                    if (!isBackdrop) continue;
                    if (!graphic.enabled) continue;
                    graphic.enabled = false;
                    disabled++;
                }
            }

            if (disabled > 0) PrefabUtility.SaveAsPrefabAsset(contents, prefabPath);
            Debug.Log($"[Lobby] Disabled {disabled} backdrop graphic(s) on {Path.GetFileName(prefabPath)}.");
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(contents);
        }
    }

    private static string ToAbsolute(string assetPath)
    {
        var root = System.IO.Directory.GetParent(Application.dataPath).FullName;
        return Path.Combine(root, assetPath).Replace('\\', '/');
    }

    /// <summary>
    /// Deletes a hand-placed screen root that duplicates the one its FlowSceneHost already
    /// instantiates.
    ///
    /// Four flow scenes ended up with both: `Tools > InstallScreensIntoScenes` drops a screen
    /// in as a scene ROOT, while the host instantiates its own copy parented to itself. Two
    /// copies then exist at runtime, both get Initialize and Show called on them, and the
    /// one that renders on top is not the one anything was configured on. The host is the
    /// single owner, so the stray root goes.
    /// </summary>
    private static void RemoveDuplicateScreen(Scene scene, string name)
    {
        foreach (var root in scene.GetRootGameObjects())
        {
            if (root.name != name) continue;
            if (root.GetComponent<F1.GameFlow.FlowSceneHost>() != null) continue;
            Object.DestroyImmediate(root);
            Debug.Log($"[Lobby] Removed duplicate '{name}' root; the FlowSceneHost owns it.");
        }
    }

    /// <summary>
    /// Removes the duplicate screen roots from the four flow scenes that still have them.
    ///
    /// The duplication came from two tools disagreeing about who owns a screen.
    /// Tools/InstallScreensIntoScenes installs the screen prefab as a scene ROOT, while
    /// Phase3SceneBuilder configures the FlowSceneHost to instantiate its OWN copy parented
    /// to itself. Any scene touched by both therefore runs two live copies at once, both
    /// receive Initialize and Show, and the one drawn on top is not the one anything was
    /// configured on. It is also why the wing screen once "showed nothing": edits to the
    /// hand-placed copy never reached the host's runtime copy.
    ///
    /// The wing scene was already fixed. These three were left, and this clears them.
    ///
    /// Each scene is opened ADDITIVELY and closed again, never made active, so whatever the
    /// user has open — including unsaved work in the hand-tuned lobby — is untouched. And a
    /// root is only removed when the scene's FlowSceneHost actually carries a non-null
    /// screen prefab, because that is the proof the host will instantiate a replacement. If
    /// the prefab were null, the scene root would be the only copy and deleting it would
    /// leave the scene with no UI at all.
    /// </summary>
    [MenuItem("Tools/UI/Remove Duplicate Screen Roots", priority = 70)]
    public static void RemoveDuplicateScreenRoots()
    {
        var targets = new (string scene, string screen)[]
        {
            ("Assets/Scenes/00_LoadingScene.unity",        "BrandingScreen"),
            ("Assets/Scenes/20_TrackSelectionScene.unity", "TrackSelectionScreen"),
            ("Assets/Scenes/30_WingSetupScene.unity",      "WingSetupScreen"),
            ("Assets/Scenes/40_PreRaceScene.unity",        "QualifyingScreen"),
        };

        int removed = 0, skipped = 0;
        foreach (var (scenePath, screenName) in targets)
        {
            if (!File.Exists(ToAbsolute(scenePath)))
            {
                Debug.LogWarning($"[Lobby] {scenePath} not found; skipped.");
                continue;
            }

            var scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Additive);
            try
            {
                var host = scene.GetRootGameObjects()
                    .Select(r => r.GetComponent<F1.GameFlow.FlowSceneHost>())
                    .FirstOrDefault(h => h != null);

                bool hostWillInstantiate = false;
                if (host != null)
                {
                    var so = new SerializedObject(host);
                    var prop = so.FindProperty("_screenPrefab");
                    hostWillInstantiate = prop != null && prop.objectReferenceValue != null;
                }

                bool hasRoot = scene.GetRootGameObjects().Any(r => r.name == screenName);
                if (!hasRoot)
                {
                    Debug.Log($"[Lobby] {Path.GetFileName(scenePath)}: no '{screenName}' root; already clean.");
                    continue;
                }

                if (!hostWillInstantiate)
                {
                    Debug.LogError($"[Lobby] {Path.GetFileName(scenePath)}: '{screenName}' root left in " +
                                   "place because its FlowSceneHost has no screen prefab. Deleting the " +
                                   "only copy would leave the scene with no UI.");
                    skipped++;
                    continue;
                }

                RemoveDuplicateScreen(scene, screenName);
                EditorSceneManager.MarkSceneDirty(scene);
                EditorSceneManager.SaveScene(scene);
                removed++;
            }
            finally
            {
                EditorSceneManager.CloseScene(scene, true);
            }
        }

        Debug.Log($"[Lobby] Duplicate screen roots removed from {removed} scene(s); {skipped} left alone. " +
                  "Each now runs the single copy its FlowSceneHost instantiates.");
    }

    private static void BuildCamera(Scene scene)
    {
        var go = new GameObject("LobbyCamera", typeof(Camera));
        SceneManager.MoveGameObjectToScene(go, scene);
        go.tag = "MainCamera";

        var cam = go.GetComponent<Camera>();
        cam.fieldOfView = CameraFov;
        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 60f;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color(0.04f, 0.045f, 0.055f);
        go.AddComponent<UniversalAdditionalCameraData>();

        go.transform.position = CameraPosition;
        go.transform.rotation = Quaternion.LookRotation(
            (CameraLookAt - CameraPosition).normalized, Vector3.up);
    }

    /// <summary>
    /// Two lights, and the split between them is the whole point.
    ///
    /// The key is a spot aimed at the car with shadows on. Shadows matter more than usual
    /// here: with no contact shadow under it, a car in a room reads as a sticker floating in
    /// mid-air, and that is the single cheapest way to make a 3D lobby look unfinished.
    ///
    /// Its range is the number that matters, not its intensity. The lamp hangs 4.5 m up and
    /// the walls are 4.7 m out, so at the 11 m range first tried it lit the whole room and
    /// left the pegboard brighter than the car - the opposite of the brief. Pulling the
    /// range to 6.5 m means the light dies before it reaches the walls, so the car sits in a
    /// pool of light and the room falls away behind it. Intensity rises to keep the car
    /// itself just as bright.
    ///
    /// The fill is deliberately weak. It exists so the pegboard walls, workbench and lockers
    /// stay readable rather than crushing to black, which is the "dim but keep it legible"
    /// brief - the background recedes by contrast with the lit car, not by being switched off.
    /// </summary>
    private static void BuildLighting(Scene scene)
    {
        var key = new GameObject("LobbyKeyLight", typeof(Light));
        SceneManager.MoveGameObjectToScene(key, scene);
        var keyLight = key.GetComponent<Light>();
        keyLight.type = LightType.Spot;
        keyLight.color = new Color(1f, 0.97f, 0.92f);
        keyLight.intensity = 26f;
        keyLight.range = 4.8f;
        keyLight.spotAngle = 72f;
        keyLight.innerSpotAngle = 34f;
        keyLight.shadows = LightShadows.Soft;
        keyLight.shadowStrength = 0.9f;
        keyLight.shadowBias = 0.02f;
        // Inside the room, just under the 3.58 m ceiling. The 4.5 m first tried was ABOVE
        // the ceiling, so it lit the roof slab instead of the car. Range 4.8 is chosen so
        // the light dies before the walls: the car edges are ~4.3 m away, the walls ~5.2 m.
        key.transform.position = new Vector3(-0.8f, 3.20f, 0.6f);
        key.transform.rotation = Quaternion.LookRotation(
            (new Vector3(0f, 0.3f, 0f) - key.transform.position).normalized, Vector3.up);

        var fill = new GameObject("LobbyFillLight", typeof(Light));
        SceneManager.MoveGameObjectToScene(fill, scene);
        var fillLight = fill.GetComponent<Light>();
        fillLight.type = LightType.Directional;
        fillLight.color = new Color(0.82f, 0.86f, 0.95f);
        fillLight.intensity = 0.15f;
        fillLight.shadows = LightShadows.None;
        fill.transform.rotation = Quaternion.Euler(38f, 28f, 0f);
    }

    /// <summary>
    /// Ambient: low and cool.
    ///
    /// The asset's own scene ships a skybox ambient around 0.21 that flatters an empty room,
    /// but the pegboard is a near-white material, so at that level the walls render BRIGHTER
    /// than the car — a dark car on a bright wall, which is the opposite of the brief. Pulling
    /// ambient down is what lets the key spot actually model the bodywork and lets the room
    /// sit behind the car instead of competing with it. It stays non-zero so the shelves and
    /// lockers remain readable, per "dim but keep it legible".
    /// </summary>
    private static void BuildAmbient()
    {
        RenderSettings.ambientMode = AmbientMode.Trilight;
        RenderSettings.ambientSkyColor = new Color(0.105f, 0.112f, 0.130f);
        RenderSettings.ambientEquatorColor = new Color(0.082f, 0.084f, 0.090f);
        RenderSettings.ambientGroundColor = new Color(0.040f, 0.040f, 0.042f);
    }

    /// <summary>
    /// Stops the screen's own opaque background painting over the 3D garage.
    ///
    /// CarSelectionScreen_Prefab's root Image carries bg_carselect.png stretched across the
    /// whole canvas. That was correct when the screen was a flat UI page; with a real garage
    /// behind it, it is an opaque curtain over the entire lobby. Disabled here rather than
    /// deleted, so the prefab stays intact and the flat look is one un-tick away if the 3D
    /// lobby is ever backed out.
    /// </summary>
    private static void OpenUpTheBackdrop(Scene scene)
    {
        int disabled = 0;
        foreach (var root in scene.GetRootGameObjects())
        {
            foreach (var canvas in root.GetComponentsInChildren<Canvas>(true))
            {
                foreach (var graphic in canvas.GetComponentsInChildren<UnityEngine.UI.Graphic>(true))
                {
                    // The screen root's own Image, and the scrim the background pass added.
                    bool isScreenBackground = graphic.gameObject == canvas.gameObject
                                           || graphic.gameObject.name == "BackdropScrim";
                    if (!isScreenBackground) continue;
                    graphic.enabled = false;
                    disabled++;
                }
            }
        }

        Debug.Log(disabled > 0
            ? $"[Lobby] Disabled {disabled} opaque backdrop graphic(s) so the garage shows through."
            : "[Lobby] No opaque backdrop found; the garage may still be hidden.");
    }
    // ==================================================================================
    // Car showcase — a stripped, static, visual-only clone
    // ==================================================================================

    [MenuItem("Tools/Lobby/Build Car Showcase Prefab", priority = 20)]
    public static void BuildCarShowcasePrefab()
    {
        var root = PrefabUtility.LoadPrefabContents(CarBodyPath);
        try
        {
            RemoveCameras(root.transform);
            StripNonVisualComponents(root.transform);

            root.name = "CarShowcase";
            ReportLeftovers(root.transform);
            PrefabUtility.SaveAsPrefabAsset(root, CarShowcasePath);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
        Debug.Log($"[Lobby] Car showcase (visual only) written to {CarShowcasePath}");
    }

    /// <summary>
    /// Drops anything that was a camera, GameObject and all. F1_Body carries its own
    /// Main Camera and a CinemachineCamera, and a lobby needs exactly one camera that it
    /// controls — a second one silently taking over would be very hard to spot from a
    /// screenshot.
    ///
    /// The transform list is snapshotted before any mutation, so entries that were children
    /// of a destroyed camera are fake-null by the time this finishes. Hence the null checks.
    /// </summary>
    private static void RemoveCameras(Transform root)
    {
        foreach (var t in AllTransforms(root))
        {
            if (t == null) continue;
            if (PrefabUtility.IsPartOfPrefabInstance(t)) continue;
            if (t.GetComponent<Camera>() == null) continue;
            Object.DestroyImmediate(t.gameObject);
        }
    }

    /// <summary>
    /// Strips the driving scripts, the Rigidbody, the colliders and the audio sources,
    /// leaving Transform, MeshFilter and MeshRenderer so the car looks identical to the
    /// racing prefab.
    ///
    /// Iterates to a fixed point rather than running once, because Unity refuses to remove a
    /// component that something else still depends on: a single pass leaves the Rigidbody
    /// behind (DownforceSystem, TractionSystem, WeightTransfer and VehiclePhysicsCoordinator
    /// all hold it) and leaves WheelVisual on each of the four wheels (RaycastWheel holds
    /// it). Removing the dependents first is what makes the dependency removable, so the
    /// next pass clears what this one could not. The loop means a future script added to
    /// F1_Body cannot quietly survive into the showcase.
    ///
    /// Nested prefab instances are skipped throughout. Car_Model is a glTF import instance
    /// and its internals are not ours to edit; only the hierarchy F1_Body itself authored is.
    /// </summary>
    private static void StripNonVisualComponents(Transform root)
    {
        for (int pass = 0; pass < 6; pass++)
        {
            bool removedAny = false;

            foreach (var t in AllTransforms(root))
            {
                if (t == null) continue;
                if (PrefabUtility.IsPartOfPrefabInstance(t)) continue;

                foreach (var component in t.GetComponents<Component>())
                {
                    if (component == null) continue;
                    if (component is Transform || component is MeshFilter || component is MeshRenderer)
                        continue;

                    Object.DestroyImmediate(component);
                    removedAny = true;
                }
            }

            if (!removedAny) break;
        }
    }

    /// <summary>
    /// Names anything non-visual that survived the strip, with counts. Silent leftovers are
    /// how a showcase ends up quietly running VehiclePhysicsCoordinator in a menu scene, so
    /// this reports rather than trusts.
    /// </summary>
    private static void ReportLeftovers(Transform root)
    {
        var counts = new Dictionary<string, int>();
        foreach (var t in AllTransforms(root))
        {
            if (t == null) continue;
            if (PrefabUtility.IsPartOfPrefabInstance(t)) continue;

            foreach (var c in t.GetComponents<Component>())
            {
                if (c == null) continue;
                if (c is Transform || c is MeshFilter || c is MeshRenderer) continue;

                var n = c.GetType().Name;
                counts.TryGetValue(n, out var k);
                counts[n] = k + 1;
            }
        }

        if (counts.Count == 0)
        {
            Debug.Log("[Lobby] Showcase strip clean: visual components only.");
            return;
        }

        var parts = new List<string>();
        foreach (var kv in counts) parts.Add($"{kv.Key} x{kv.Value}");
        Debug.LogWarning($"[Lobby] Showcase still carries {counts.Count} non-visual component type(s): "
                         + string.Join(", ", parts));
    }

    /// <summary>Depth-first over live transforms, collecting before mutating.</summary>
    private static List<Transform> AllTransforms(Transform root)
    {
        var found = new List<Transform>();
        var stack = new Stack<Transform>();
        stack.Push(root);
        while (stack.Count > 0)
        {
            var t = stack.Pop();
            found.Add(t);
            for (int i = 0; i < t.childCount; i++) stack.Push(t.GetChild(i));
        }
        return found;
    }
}
