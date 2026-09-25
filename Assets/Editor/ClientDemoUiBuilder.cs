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
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using F1.UI;

public class ClientDemoUiBuilder
{
    private const string CarCardPath = "Assets/Prefabs/CarCard_Prefab.prefab";
    private const string TrackCardPath = "Assets/Prefabs/TrackCard_Prefab.prefab";

    // The columns are a third of the canvas wide each, so 320 leaves comfortable gutters
    // on a 16:9 screen while still filling the column.
    private const float CardWidth = 320f;
    private const float CardHeight = 460f;
    private const float ArtHeight = 250f;
    private const float CardCornerPad = 6f;

    [MenuItem("Tools/BuildClientDemoUI")]
    public static void Build()
    {
        BuildCarCard();
        BuildTrackCard();
        AssetDatabase.SaveAssets();
        Debug.Log("[ClientDemoUI] Card prefabs rebuilt with artwork slots and preferred sizes.");
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

            // A banner across the artwork rather than a small label, so it reads at a glance
            // from across a room during a demo.
            var badge = FindOrCreateChild(cardRect, "ComingSoonBadge");
            var badgeImage = badge.GetComponent<Image>();
            badgeImage.color = new Color(0f, 0f, 0f, 0.72f);
            badgeImage.raycastTarget = false;
            badgeRect(badge, ArtHeight);

            var label = FindOrCreateTextChild(badge, "ComingSoonText");
            var text = label.GetComponent<TMPro.TextMeshProUGUI>();
            text.text = "COMING SOON";
            text.fontSize = 30f;
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
        // Sits over the middle of the artwork panel.
        badge.anchorMin = new Vector2(0f, 1f);
        badge.anchorMax = new Vector2(1f, 1f);
        badge.pivot = new Vector2(0.5f, 1f);
        float top = -(CardCornerPad + artHeight * 0.5f);
        badge.offsetMin = new Vector2(CardCornerPad, top - 44f);
        badge.offsetMax = new Vector2(-CardCornerPad, top + 44f);
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
            SetAnchor(root.transform, "NameText", 0.80f, 34f);
            SetAnchor(root.transform, "GenText", 0.70f, 30f);
            SetAnchor(root.transform, "ShortCodeText", 0.70f, 30f);
            SetAnchor(root.transform, "CostText", 0.60f, 30f);
            SetAnchor(root.transform, "StatusText", 0.51f, 26f);
            SetAnchor(root.transform, "CardButton", 0.38f, 44f);

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
