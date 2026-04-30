using UnityEngine;

public class MobileTouchControls : MonoBehaviour
{
    [Header("Layout")]
    [Range(0.12f, 0.35f)] public float buttonSizeNormalized = 0.2f;
    [Range(0.01f, 0.08f)] public float screenMarginNormalized = 0.03f;
    [Range(0.01f, 0.08f)] public float buttonGapNormalized = 0.02f;

    [Header("Behavior")]
    public bool showInEditor = true;

    public static bool IsActive { get; private set; }
    public static float Steering { get; private set; }
    public static float Throttle { get; private set; }
    public static float Brake { get; private set; }

    private Rect _leftRect;
    private Rect _rightRect;
    private Rect _throttleRect;
    private Rect _brakeRect;

    private GUIStyle _buttonStyle;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void EnsureInstance()
    {
        if (FindFirstObjectByType<MobileTouchControls>() != null)
            return;

        GameObject controlsObject = new GameObject("MobileTouchControls");
        controlsObject.AddComponent<MobileTouchControls>();
    }

    private void OnEnable()
    {
        IsActive = true;
        RecalculateRects();
    }

    private void OnDisable()
    {
        Steering = 0f;
        Throttle = 0f;
        Brake = 0f;
        IsActive = false;
    }

    private void Update()
    {
        RecalculateRects();
        Steering = 0f;
        Throttle = 0f;
        Brake = 0f;

        for (int i = 0; i < Input.touchCount; i++)
        {
            Touch touch = Input.GetTouch(i);
            if (touch.phase == TouchPhase.Ended || touch.phase == TouchPhase.Canceled)
                continue;

            Vector2 point = touch.position;
            point.y = Screen.height - point.y;
            ApplyPoint(point);
        }

#if UNITY_EDITOR
        if (showInEditor && Input.GetMouseButton(0))
        {
            Vector2 point = Input.mousePosition;
            point.y = Screen.height - point.y;
            ApplyPoint(point);
        }
#endif
    }

    private void OnGUI()
    {
        if (!_ShouldDraw())
            return;

        if (_buttonStyle == null)
        {
            _buttonStyle = new GUIStyle(GUI.skin.button);
            _buttonStyle.fontSize = Mathf.RoundToInt(Mathf.Min(Screen.width, Screen.height) * 0.045f);
            _buttonStyle.alignment = TextAnchor.MiddleCenter;
        }

        Color oldColor = GUI.color;
        DrawButton(_leftRect, "LEFT", Steering < 0f);
        DrawButton(_rightRect, "RIGHT", Steering > 0f);
        DrawButton(_throttleRect, "GAS", Throttle > 0f);
        DrawButton(_brakeRect, "BRAKE", Brake > 0f);
        GUI.color = oldColor;
    }

    private void DrawButton(Rect rect, string text, bool pressed)
    {
        GUI.color = pressed ? new Color(0.2f, 0.9f, 0.3f, 0.95f) : new Color(0f, 0f, 0f, 0.55f);
        GUI.Box(rect, text, _buttonStyle);
    }

    private void ApplyPoint(Vector2 point)
    {
        if (_leftRect.Contains(point))
            Steering = -1f;
        else if (_rightRect.Contains(point))
            Steering = 1f;

        if (_throttleRect.Contains(point))
            Throttle = 1f;
        else if (_brakeRect.Contains(point))
            Brake = 1f;
    }

    private bool _ShouldDraw()
    {
#if UNITY_EDITOR
        return showInEditor;
#else
        return true;
#endif
    }

    private void RecalculateRects()
    {
        float minDim = Mathf.Min(Screen.width, Screen.height);
        float buttonSize = minDim * buttonSizeNormalized;
        float margin = minDim * screenMarginNormalized;
        float gap = minDim * buttonGapNormalized;

        float y = Screen.height - margin - buttonSize;
        _leftRect = new Rect(margin, y, buttonSize, buttonSize);
        _rightRect = new Rect(margin + buttonSize + gap, y, buttonSize, buttonSize);

        float rightX = Screen.width - margin - buttonSize;
        _throttleRect = new Rect(rightX, y, buttonSize, buttonSize);
        _brakeRect = new Rect(rightX - buttonSize - gap, y, buttonSize, buttonSize);
    }
}
