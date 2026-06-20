using UnityEngine;
using UnityEngine.UI;

public class ClientDemoHudController : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private VehiclePhysicsCoordinator player;
    [SerializeField] private ClientDemoRacePositionTracker racePositionTracker;

    [Header("Display")]
    [SerializeField, Min(0.03f)] private float refreshInterval = 0.08f;

    private Text _speedText;
    private Text _positionText;
    private Font _font;
    private float _nextRefreshTime;

    private void Awake()
    {
        ResolveReferences();
        BuildHud();
    }

    private void Start()
    {
        ResolveReferences();
        RefreshHud();
    }

    private void Update()
    {
        if (Time.unscaledTime < _nextRefreshTime)
            return;

        ResolveReferences();
        RefreshHud();
        _nextRefreshTime = Time.unscaledTime + refreshInterval;
    }

    private void ResolveReferences()
    {
        if (player == null)
            player = FindPlayerCoordinator();

        if (racePositionTracker == null)
            racePositionTracker = FindAnyObjectByType<ClientDemoRacePositionTracker>();

        if (racePositionTracker == null)
            racePositionTracker = gameObject.AddComponent<ClientDemoRacePositionTracker>();
    }

    private VehiclePhysicsCoordinator FindPlayerCoordinator()
    {
        VehiclePhysicsCoordinator[] vehicles = FindObjectsByType<VehiclePhysicsCoordinator>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < vehicles.Length; i++)
        {
            if (vehicles[i] != null && !vehicles[i].UseExternalInput)
                return vehicles[i];
        }

        for (int i = 0; i < vehicles.Length; i++)
        {
            if (vehicles[i] == null)
                continue;

            string vehicleName = vehicles[i].name;
            if (vehicleName.Contains("F1_Body") && !vehicleName.Contains("AI"))
                return vehicles[i];
        }

        return vehicles.Length > 0 ? vehicles[0] : null;
    }

    private void RefreshHud()
    {
        if (_speedText != null)
        {
            float speedKmh = player != null ? player.SpeedKmh : 0f;
            _speedText.text = $"{Mathf.RoundToInt(speedKmh):000} KM/H";
        }

        if (_positionText != null)
        {
            string position = racePositionTracker != null ? racePositionTracker.PositionText : "P1 / 1";
            _positionText.text = position;
        }
    }

    private void BuildHud()
    {
        _font = ResolveFont();

        Canvas canvas = CreateCanvas("ClientDemo_HudCanvas", 20);

        GameObject speedPanel = CreatePanel("SpeedPanel", canvas.transform, new Color(0.01f, 0.015f, 0.018f, 0.68f));
        RectTransform speedRect = speedPanel.GetComponent<RectTransform>();
        speedRect.anchorMin = new Vector2(0f, 0f);
        speedRect.anchorMax = new Vector2(0f, 0f);
        speedRect.pivot = new Vector2(0f, 0f);
        speedRect.sizeDelta = new Vector2(250f, 82f);
        speedRect.anchoredPosition = new Vector2(36f, 34f);

        _speedText = CreateText("SpeedText", speedPanel.transform, "000 KM/H", 34, FontStyle.Bold, TextAnchor.MiddleCenter, Color.white);
        Stretch(_speedText.rectTransform);

        Text speedLabel = CreateText("SpeedLabel", speedPanel.transform, "SPEED", 14, FontStyle.Bold, TextAnchor.UpperLeft, new Color(0.46f, 0.75f, 0.94f, 1f));
        speedLabel.rectTransform.anchorMin = new Vector2(0f, 1f);
        speedLabel.rectTransform.anchorMax = new Vector2(0f, 1f);
        speedLabel.rectTransform.pivot = new Vector2(0f, 1f);
        speedLabel.rectTransform.sizeDelta = new Vector2(120f, 24f);
        speedLabel.rectTransform.anchoredPosition = new Vector2(14f, -8f);

        GameObject positionPanel = CreatePanel("PositionPanel", canvas.transform, new Color(0.01f, 0.015f, 0.018f, 0.68f));
        RectTransform positionRect = positionPanel.GetComponent<RectTransform>();
        positionRect.anchorMin = new Vector2(1f, 1f);
        positionRect.anchorMax = new Vector2(1f, 1f);
        positionRect.pivot = new Vector2(1f, 1f);
        positionRect.sizeDelta = new Vector2(180f, 76f);
        positionRect.anchoredPosition = new Vector2(-36f, -34f);

        _positionText = CreateText("PositionText", positionPanel.transform, "P1 / 1", 32, FontStyle.Bold, TextAnchor.MiddleCenter, Color.white);
        Stretch(_positionText.rectTransform);

        Text positionLabel = CreateText("PositionLabel", positionPanel.transform, "POSITION", 14, FontStyle.Bold, TextAnchor.UpperLeft, new Color(0.46f, 0.75f, 0.94f, 1f));
        positionLabel.rectTransform.anchorMin = new Vector2(0f, 1f);
        positionLabel.rectTransform.anchorMax = new Vector2(0f, 1f);
        positionLabel.rectTransform.pivot = new Vector2(0f, 1f);
        positionLabel.rectTransform.sizeDelta = new Vector2(130f, 24f);
        positionLabel.rectTransform.anchoredPosition = new Vector2(12f, -8f);
    }

    private Canvas CreateCanvas(string name, int sortingOrder)
    {
        GameObject canvasObject = new GameObject(name, typeof(RectTransform), typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
        canvasObject.transform.SetParent(transform, false);

        Canvas canvas = canvasObject.GetComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvas.sortingOrder = sortingOrder;

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
}
