using System;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class ClientDemoMenuController : MonoBehaviour
{
    [Header("Scene Flow")]
    [SerializeField] private string gameplaySceneName = "ClientDemo_Akash_Scene";

    [Header("Presentation")]
    [SerializeField] private string titleText = "F1";
    [SerializeField] private string subtitleText = "Playable review build";
    [SerializeField] private string backgroundResourcePath = "ClientDemo/MenuBackground";

    private GameObject _entryPanel;
    private GameObject _racePanel;
    private Font _font;
    private readonly Color _accentColor = new Color(0.95f, 0.08f, 0.08f, 1f);

    private void Awake()
    {
        EnsureEventSystem();
        BuildMenu();
        ShowEntryPanel();
    }

    public void ShowEntryPanel()
    {
        if (_entryPanel != null)
            _entryPanel.SetActive(true);

        if (_racePanel != null)
            _racePanel.SetActive(false);
    }

    public void ShowRacePanel()
    {
        if (_entryPanel != null)
            _entryPanel.SetActive(false);

        if (_racePanel != null)
            _racePanel.SetActive(true);
    }

    public void StartRace()
    {
        if (string.IsNullOrWhiteSpace(gameplaySceneName))
        {
            Debug.LogWarning("Client demo gameplay scene name is empty.");
            return;
        }

        SceneManager.LoadScene(gameplaySceneName);
    }

    public void Quit()
    {
#if UNITY_EDITOR
        Debug.Log("Client demo quit requested.");
#else
        Application.Quit();
#endif
    }

    private void BuildMenu()
    {
        _font = ResolveFont();

        Canvas canvas = CreateCanvas("ClientDemo_MenuCanvas", 0);
        CreateBackground(canvas.transform);

        CreateHorizontalFadeOverlay(canvas.transform);

        GameObject vignette = CreatePanel("Vignette", canvas.transform, new Color(0f, 0f, 0f, 0.28f));
        Stretch(vignette.GetComponent<RectTransform>());

        GameObject accent = CreatePanel("AccentRule", canvas.transform, _accentColor);
        RectTransform accentRect = accent.GetComponent<RectTransform>();
        accentRect.anchorMin = new Vector2(0.085f, 0.575f);
        accentRect.anchorMax = new Vector2(0.25f, 0.582f);
        accentRect.offsetMin = Vector2.zero;
        accentRect.offsetMax = Vector2.zero;

        Text eyebrow = CreateText("Eyebrow", canvas.transform, "PREVIEW BUILD // WINDOWS", 18, FontStyle.Bold, TextAnchor.MiddleLeft, new Color(0.64f, 0.82f, 1f, 0.92f));
        RectTransform eyebrowRect = eyebrow.rectTransform;
        eyebrowRect.anchorMin = new Vector2(0.085f, 0.76f);
        eyebrowRect.anchorMax = new Vector2(0.45f, 0.81f);
        eyebrowRect.offsetMin = Vector2.zero;
        eyebrowRect.offsetMax = Vector2.zero;

        Text title = CreateText("Title", canvas.transform, titleText, 72, FontStyle.Bold, TextAnchor.MiddleLeft, new Color(0.96f, 0.98f, 1f, 1f));
        RectTransform titleRect = title.rectTransform;
        titleRect.anchorMin = new Vector2(0.082f, 0.62f);
        titleRect.anchorMax = new Vector2(0.5f, 0.76f);
        titleRect.offsetMin = Vector2.zero;
        titleRect.offsetMax = Vector2.zero;

        Text subtitle = CreateText("Subtitle", canvas.transform, subtitleText, 24, FontStyle.Normal, TextAnchor.MiddleLeft, new Color(0.72f, 0.84f, 0.94f, 0.96f));
        RectTransform subtitleRect = subtitle.rectTransform;
        subtitleRect.anchorMin = new Vector2(0.085f, 0.53f);
        subtitleRect.anchorMax = new Vector2(0.44f, 0.59f);
        subtitleRect.offsetMin = Vector2.zero;
        subtitleRect.offsetMax = Vector2.zero;

        Text footer = CreateText("Footer", canvas.transform, "Phase 2.3 playable demo", 16, FontStyle.Normal, TextAnchor.LowerLeft, new Color(0.62f, 0.7f, 0.78f, 0.76f));
        RectTransform footerRect = footer.rectTransform;
        footerRect.anchorMin = new Vector2(0.085f, 0.055f);
        footerRect.anchorMax = new Vector2(0.48f, 0.095f);
        footerRect.offsetMin = Vector2.zero;
        footerRect.offsetMax = Vector2.zero;

        _entryPanel = CreateMenuPanel("EntryPanel", canvas.transform);
        CreateButton("StartButton", _entryPanel.transform, "START", ShowRacePanel, 0);
        CreateButton("QuitButton", _entryPanel.transform, "QUIT", Quit, 1);

        _racePanel = CreateMenuPanel("RacePanel", canvas.transform);
        CreateButton("StartRaceButton", _racePanel.transform, "START RACE", StartRace, 0);
        CreateButton("BackButton", _racePanel.transform, "BACK", ShowEntryPanel, 1);
    }

    private GameObject CreateMenuPanel(string name, Transform parent)
    {
        GameObject panel = new GameObject(name, typeof(RectTransform));
        panel.transform.SetParent(parent, false);
        RectTransform rect = panel.GetComponent<RectTransform>();
        rect.anchorMin = new Vector2(0.085f, 0.275f);
        rect.anchorMax = new Vector2(0.36f, 0.455f);
        rect.sizeDelta = Vector2.zero;
        rect.anchoredPosition = Vector2.zero;
        return panel;
    }

    private Button CreateButton(string name, Transform parent, string label, UnityEngine.Events.UnityAction action, int index)
    {
        GameObject buttonObject = new GameObject(name, typeof(RectTransform), typeof(Image), typeof(Button));
        buttonObject.transform.SetParent(parent, false);

        RectTransform rect = buttonObject.GetComponent<RectTransform>();
        rect.anchorMin = new Vector2(0f, 1f);
        rect.anchorMax = new Vector2(1f, 1f);
        rect.pivot = new Vector2(0.5f, 1f);
        rect.sizeDelta = new Vector2(0f, 66f);
        rect.anchoredPosition = new Vector2(0f, -index * 82f);

        Image image = buttonObject.GetComponent<Image>();
        image.color = index == 0 ? new Color(0.82f, 0.02f, 0.03f, 0.95f) : new Color(0.035f, 0.055f, 0.075f, 0.92f);

        Button button = buttonObject.GetComponent<Button>();
        button.targetGraphic = image;
        button.onClick.AddListener(action);

        ColorBlock colors = button.colors;
        colors.highlightedColor = index == 0 ? new Color(1f, 0.12f, 0.12f, 1f) : new Color(0.1f, 0.15f, 0.19f, 1f);
        colors.pressedColor = new Color(0.45f, 0.01f, 0.02f, 1f);
        button.colors = colors;

        Text text = CreateText("Label", buttonObject.transform, label, 25, FontStyle.Bold, TextAnchor.MiddleLeft, Color.white);
        RectTransform textRect = text.rectTransform;
        Stretch(textRect);
        textRect.offsetMin = new Vector2(28f, 0f);
        textRect.offsetMax = new Vector2(-28f, 0f);

        GameObject marker = CreatePanel("Marker", buttonObject.transform, index == 0 ? Color.white : _accentColor);
        RectTransform markerRect = marker.GetComponent<RectTransform>();
        markerRect.anchorMin = new Vector2(1f, 0.34f);
        markerRect.anchorMax = new Vector2(1f, 0.66f);
        markerRect.pivot = new Vector2(1f, 0.5f);
        markerRect.sizeDelta = new Vector2(5f, 0f);
        markerRect.anchoredPosition = new Vector2(-22f, 0f);
        return button;
    }

    private void CreateBackground(Transform parent)
    {
        Texture2D backgroundTexture = Resources.Load<Texture2D>(backgroundResourcePath);
        GameObject background = new GameObject("GeneratedBackground", typeof(RectTransform), typeof(RawImage));
        background.transform.SetParent(parent, false);
        Stretch(background.GetComponent<RectTransform>());

        RawImage image = background.GetComponent<RawImage>();
        image.texture = backgroundTexture;
        image.color = backgroundTexture != null ? Color.white : new Color(0.02f, 0.025f, 0.03f, 1f);
        image.uvRect = new Rect(0f, 0f, 1f, 1f);

        GameObject blueWash = CreatePanel("CoolWash", parent, new Color(0.01f, 0.035f, 0.06f, 0.18f));
        Stretch(blueWash.GetComponent<RectTransform>());
    }

    private void CreateHorizontalFadeOverlay(Transform parent)
    {
        const int width = 96;
        Texture2D fadeTexture = new Texture2D(width, 1, TextureFormat.RGBA32, false)
        {
            wrapMode = TextureWrapMode.Clamp,
            filterMode = FilterMode.Bilinear
        };

        for (int x = 0; x < width; x++)
        {
            float t = x / (float)(width - 1);
            float alpha = Mathf.SmoothStep(0.82f, 0f, t);
            fadeTexture.SetPixel(x, 0, new Color(0f, 0.004f, 0.01f, alpha));
        }

        fadeTexture.Apply(false, true);

        GameObject fade = new GameObject("LeftCinematicFade", typeof(RectTransform), typeof(RawImage));
        fade.transform.SetParent(parent, false);
        Stretch(fade.GetComponent<RectTransform>());

        RawImage image = fade.GetComponent<RawImage>();
        image.texture = fadeTexture;
        image.raycastTarget = false;
    }

    private Canvas CreateCanvas(string name, int sortingOrder)
    {
        GameObject canvasObject = new GameObject(name, typeof(RectTransform), typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
        canvasObject.transform.SetParent(transform, false);

        Canvas canvas = canvasObject.GetComponent<Canvas>();
        canvas.sortingOrder = sortingOrder;
        Camera mainCamera = Camera.main;
        if (mainCamera != null)
        {
            canvas.renderMode = RenderMode.ScreenSpaceCamera;
            canvas.worldCamera = mainCamera;
            canvas.planeDistance = 1f;
        }
        else
        {
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        }

        CanvasScaler scaler = canvasObject.GetComponent<CanvasScaler>();
        scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);
        scaler.matchWidthOrHeight = 0.5f;

        return canvas;
    }

    private GameObject CreatePanel(string name, Transform parent, Color color)
    {
        GameObject panel = new GameObject(name, typeof(RectTransform), typeof(Image));
        panel.transform.SetParent(parent, false);
        panel.GetComponent<Image>().color = color;
        return panel;
    }

    private Text CreateText(string name, Transform parent, string value, int size, FontStyle style, TextAnchor alignment, Color color)
    {
        GameObject textObject = new GameObject(name, typeof(RectTransform), typeof(Text));
        textObject.transform.SetParent(parent, false);

        Text text = textObject.GetComponent<Text>();
        text.font = _font;
        text.text = value;
        text.fontSize = size;
        text.fontStyle = style;
        text.alignment = alignment;
        text.color = color;
        text.horizontalOverflow = HorizontalWrapMode.Wrap;
        text.verticalOverflow = VerticalWrapMode.Truncate;
        return text;
    }

    private static void Stretch(RectTransform rectTransform)
    {
        rectTransform.anchorMin = Vector2.zero;
        rectTransform.anchorMax = Vector2.one;
        rectTransform.offsetMin = Vector2.zero;
        rectTransform.offsetMax = Vector2.zero;
    }

    private static Font ResolveFont()
    {
        Font font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        if (font == null)
            font = Resources.GetBuiltinResource<Font>("Arial.ttf");
        return font;
    }

    private static void EnsureEventSystem()
    {
        if (FindAnyObjectByType<EventSystem>() != null)
            return;

        GameObject eventSystem = new GameObject("EventSystem", typeof(EventSystem));
        Type inputSystemModule = Type.GetType("UnityEngine.InputSystem.UI.InputSystemUIInputModule, Unity.InputSystem");
        if (inputSystemModule != null)
            eventSystem.AddComponent(inputSystemModule);
        else
            eventSystem.AddComponent<StandaloneInputModule>();
    }
}
