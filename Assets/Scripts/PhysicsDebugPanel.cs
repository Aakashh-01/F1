using UnityEngine;
using System.Collections.Generic;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif


/// Uses Unity's immediate-mode OnGUI — no Canvas, no prefab, no RectTransform setup.
/// Shows:
///   - Live telemetry (speed, downforce, axle loads, weight transfer G-forces)
///   - Per-wheel grip utilisation bars with colour coding
///   - Runtime sliders for all key physics parameters (live write-back every frame)

public class PhysicsDebugPanel : MonoBehaviour
{
    // -------------------------------------------------------------------------
    //  INSPECTOR
    // -------------------------------------------------------------------------
    [Header("Car Reference")]
    public GameObject carRoot;

    [Header("Panel")]
    public KeyCode toggleKey = KeyCode.Tab;
    [Range(260f, 440f)] public float panelWidth = 320f;
    [Range(0.3f, 1f)] public float panelAlpha = 0.88f;

    // -------------------------------------------------------------------------
    //  PRIVATE — system refs
    // -------------------------------------------------------------------------
    private RaycastWheel[] _wheels = new RaycastWheel[4];
    private TractionSystem _traction;
    private DownforceSystem _downforce;
    private WeightTransfer _weightXfer;
    private Rigidbody _rb;

    // -------------------------------------------------------------------------
    //  PRIVATE — UI state
    // -------------------------------------------------------------------------
    private bool _visible = true;
    private Vector2 _scroll = Vector2.zero;
    private GUIStyle _labelStyle;
    private GUIStyle _headerStyle;
    private GUIStyle _valueStyle;
    private GUIStyle _panelStyle;
    private bool _stylesBuilt;

    private class SliderParam
    {
        public string label;
        public string group;
        public float min, max, value;
        public System.Action<float> apply;
    }
    private List<SliderParam> _sliders = new List<SliderParam>();
    private bool _slidersBuilt;

    private static readonly string[] WheelNames = { "FL", "FR", "RL", "RR" };

    // =========================================================================
    //  LIFECYCLE
    // =========================================================================
    private void Start()
    {
        if (carRoot == null) { Debug.LogError("[DebugPanel] carRoot not assigned!"); return; }
        ResolveRefs();
        BuildSliders();
    }

    private void Update()
    {
        if (WasTogglePressed())
            _visible = !_visible;

        if (_visible && _slidersBuilt)
            foreach (var s in _sliders)
                s.apply(s.value);
    }

    // =========================================================================
    //  ONGUI — entire panel drawn here, no Canvas required
    // =========================================================================
    private void OnGUI()
    {
        if (!_visible || carRoot == null) return;
        EnsureStyles();

        float panH = Screen.height - 20f;
        Rect panRect = new Rect(10, 10, panelWidth, panH);

        GUI.Box(panRect, GUIContent.none, _panelStyle);

        Rect inner = new Rect(panRect.x + 8, panRect.y + 8,
                               panRect.width - 16, panRect.height - 16);

        GUILayout.BeginArea(inner);
        _scroll = GUILayout.BeginScrollView(_scroll, false, false,
                    GUIStyle.none, GUIStyle.none);

        // Title
        SetColor(0.20f, 0.85f, 1.00f);
        GUILayout.Label("F1 PHYSICS DEBUG   [TAB]", _headerStyle);
        ResetColor();
        Divider();

        // Telemetry
        SectionLabel("TELEMETRY");
        DrawTelemetry();
        Divider();

        // Grip bars
        SectionLabel("GRIP UTILISATION");
        DrawGripBars();
        Divider();

        // Sliders
        SectionLabel("TUNING");
        DrawSliders();

        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    // =========================================================================
    //  TELEMETRY SECTION
    // =========================================================================
    private void DrawTelemetry()
    {
        if (_rb != null)
        {
#if UNITY_6000_0_OR_NEWER
            float kmh = _rb.linearVelocity.magnitude * 3.6f;
#else
            float kmh = _rb.velocity.magnitude * 3.6f;
#endif
            Row("Speed", $"{kmh:F0} km/h");
        }

        if (_downforce != null)
        {
            Row("Downforce", $"{_downforce.DownforceTotal:F0} N");
            Row("Front / Rear", $"{_downforce.FrontDownforce:F0} / {_downforce.RearDownforce:F0} N");
        }

        if (_weightXfer != null)
        {
            Row("Front load", $"{_weightXfer.FrontAxleLoad:F0} N");
            Row("Rear load", $"{_weightXfer.RearAxleLoad:F0} N");
            float lg = _weightXfer.LongTransferG * 5f;
            float lat = _weightXfer.LatTransferG * 5f;
            Row("Long G", $"{lg:+0.00;-0.00} G");
            Row("Lat G", $"{lat:+0.00;-0.00} G");
        }
    }

    private void Row(string label, string val)
    {
        GUILayout.BeginHorizontal();
        SetColor(0.60f, 0.65f, 0.72f);
        GUILayout.Label(label, _labelStyle, GUILayout.Width(130f));
        SetColor(0.92f, 0.95f, 1.00f);
        GUILayout.Label(val, _valueStyle);
        ResetColor();
        GUILayout.EndHorizontal();
    }

    // =========================================================================
    //  GRIP BAR SECTION
    // =========================================================================
    private void DrawGripBars()
    {
        if (_traction == null)
        {
            SetColor(1f, 0.4f, 0.4f);
            GUILayout.Label("TractionSystem not found on carRoot", _labelStyle);
            ResetColor();
            return;
        }

        for (int i = 0; i < 4; i++)
        {
            float util = _traction.GripUtilisation[i];
            Color fill = util < 0.70f ? new Color(0.18f, 0.72f, 0.96f, 1f)
                       : util < 0.90f ? new Color(1.00f, 0.60f, 0.10f, 1f)
                                      : new Color(0.95f, 0.22f, 0.22f, 1f);

            GUILayout.BeginHorizontal();

            // Label
            SetColor(0.75f, 0.78f, 0.84f);
            GUILayout.Label(WheelNames[i], _labelStyle, GUILayout.Width(26f));

            // Bar track + fill using a horizontal slider styled as a bar
            Rect trackRect = GUILayoutUtility.GetRect(panelWidth - 90f, 12f);
            GUI.DrawTexture(trackRect, Texture2D.whiteTexture, ScaleMode.StretchToFill,
                            true, 0, new Color(1f, 1f, 1f, 0.10f), 0, 0);
            Rect fillRect = new Rect(trackRect.x, trackRect.y,
                                     trackRect.width * Mathf.Clamp01(util), trackRect.height);
            GUI.DrawTexture(fillRect, Texture2D.whiteTexture, ScaleMode.StretchToFill,
                            true, 0, fill, 0, 0);

            // Percent
            SetColor(0.88f, 0.91f, 0.96f);
            GUILayout.Label($"{util * 100f:F0}%", _valueStyle, GUILayout.Width(36f));

            ResetColor();
            GUILayout.EndHorizontal();
            GUILayout.Space(4f);
        }
    }

    // =========================================================================
    //  SLIDER SECTION
    // =========================================================================
    private void DrawSliders()
    {
        if (!_slidersBuilt) return;

        string lastGroup = "";
        foreach (var s in _sliders)
        {
            if (s.group != lastGroup)
            {
                GUILayout.Space(6f);
                SetColor(0.55f, 0.60f, 0.68f);
                GUILayout.Label(s.group, _labelStyle);
                ResetColor();
                lastGroup = s.group;
            }

            GUILayout.BeginHorizontal();
            SetColor(0.72f, 0.75f, 0.82f);
            GUILayout.Label(s.label, _labelStyle, GUILayout.Width(162f));
            SetColor(0.92f, 0.95f, 1.00f);
            GUILayout.Label(FormatVal(s.value, s.min, s.max), _valueStyle, GUILayout.Width(50f));
            ResetColor();
            GUILayout.EndHorizontal();

            s.value = GUILayout.HorizontalSlider(s.value, s.min, s.max);
            GUILayout.Space(2f);
        }
    }

    // =========================================================================
    //  BUILD SLIDERS
    // =========================================================================
    private void AddSlider(string group, string label, float min, float max,
                            float init, System.Action<float> apply)
    {
        _sliders.Add(new SliderParam
        {
            group = group,
            label = label,
            min = min,
            max = max,
            value = init,
            apply = apply
        });
    }

    private void BuildSliders()
    {
        _sliders.Clear();

        if (_wheels[0] != null)
        {
            AddSlider("Suspension", "Spring stiffness",
                10000f, 120000f, _wheels[0].springStiffness,
                v => { foreach (var w in _wheels) if (w) w.springStiffness = v; });
            AddSlider("Suspension", "Damper strength",
                500f, 12000f, _wheels[0].damperStrength,
                v => { foreach (var w in _wheels) if (w) w.damperStrength = v; });
            AddSlider("Suspension", "Rest length ratio",
                0.1f, 0.9f, _wheels[0].restLengthRatio,
                v => { foreach (var w in _wheels) if (w) w.restLengthRatio = v; });
        }

        if (_downforce != null)
        {
            AddSlider("Downforce", "Downforce coeff",
                0.5f, 8f, _downforce.downforceCoeff,
                v => _downforce.downforceCoeff = v);
            AddSlider("Downforce", "Front bias",
                0f, 1f, _downforce.frontBias,
                v => _downforce.frontBias = v);
            AddSlider("Downforce", "Lateral drag",
                0f, 5f, _downforce.lateralDragCoeff,
                v => _downforce.lateralDragCoeff = v);
        }

        if (_traction != null)
        {
            AddSlider("Traction", "Lateral D (peak grip)",
                0.5f, 2.5f, _traction.lateralD,
                v => _traction.lateralD = v);
            AddSlider("Traction", "Lateral B (stiffness)",
                4f, 20f, _traction.lateralB,
                v => _traction.lateralB = v);
            AddSlider("Traction", "Load sensitivity",
                0.4f, 1f, _traction.loadSensitivity,
                v => _traction.loadSensitivity = v);
        }

        if (_weightXfer != null)
        {
            AddSlider("Weight Transfer", "CoM height",
                0.1f, 0.6f, _weightXfer.comHeight,
                v => _weightXfer.comHeight = v);
            AddSlider("Weight Transfer", "Long transfer rate",
                1f, 15f, _weightXfer.longTransferRate,
                v => _weightXfer.longTransferRate = v);
            AddSlider("Weight Transfer", "Anti-roll front",
                0f, 150000f, _weightXfer.frontAntiRollStiffness,
                v => _weightXfer.frontAntiRollStiffness = v);
            AddSlider("Weight Transfer", "Anti-roll rear",
                0f, 150000f, _weightXfer.rearAntiRollStiffness,
                v => _weightXfer.rearAntiRollStiffness = v);
        }

        _slidersBuilt = true;
    }

    // =========================================================================
    //  HELPERS
    // =========================================================================
    private void ResolveRefs()
    {
        _rb = carRoot.GetComponent<Rigidbody>();
        _traction = carRoot.GetComponent<TractionSystem>();
        _downforce = carRoot.GetComponent<DownforceSystem>();
        _weightXfer = carRoot.GetComponent<WeightTransfer>();

        var found = carRoot.GetComponentsInChildren<RaycastWheel>();
        for (int i = 0; i < Mathf.Min(found.Length, 4); i++)
            _wheels[i] = found[i];

        if (_traction == null) Debug.LogWarning("[DebugPanel] TractionSystem not found on carRoot.");
        if (_downforce == null) Debug.LogWarning("[DebugPanel] DownforceSystem not found on carRoot.");
        if (_weightXfer == null) Debug.LogWarning("[DebugPanel] WeightTransfer not found on carRoot.");
    }

    private bool WasTogglePressed()
    {
#if ENABLE_INPUT_SYSTEM
        return Keyboard.current != null &&
               Keyboard.current[UnityEngine.InputSystem.Key.Tab].wasPressedThisFrame;
#else
        return Input.GetKeyDown(toggleKey);
#endif
    }

    private void SectionLabel(string text)
    {
        SetColor(0.20f, 0.85f, 1.00f, 0.90f);
        GUILayout.Label(text, _labelStyle);
        ResetColor();
        GUILayout.Space(2f);
    }

    private void Divider()
    {
        GUILayout.Space(4f);
        Rect r = GUILayoutUtility.GetRect(panelWidth - 20f, 1f);
        GUI.DrawTexture(r, Texture2D.whiteTexture, ScaleMode.StretchToFill,
                        true, 0, new Color(1f, 1f, 1f, 0.12f), 0, 0);
        GUILayout.Space(4f);
    }

    private void SetColor(float r, float g, float b, float a = 1f) =>
        GUI.color = new Color(r, g, b, a);

    private void ResetColor() => GUI.color = Color.white;

    private string FormatVal(float v, float min, float max)
    {
        float range = max - min;
        if (range >= 1000f) return $"{v:F0}";
        if (range >= 10f) return $"{v:F1}";
        return $"{v:F3}";
    }

    private void EnsureStyles()
    {
        if (_stylesBuilt) return;

        // Panel background — semi-transparent dark box
        var bgTex = new Texture2D(1, 1);
        bgTex.SetPixel(0, 0, new Color(0.05f, 0.07f, 0.10f, panelAlpha));
        bgTex.Apply();

        _panelStyle = new GUIStyle(GUI.skin.box)
        {
            border = new RectOffset(4, 4, 4, 4),
            padding = new RectOffset(0, 0, 0, 0),
            normal = { background = bgTex }
        };

        _labelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            fontStyle = FontStyle.Normal,
            padding = new RectOffset(0, 0, 1, 1),
            normal = { textColor = Color.white }
        };

        _headerStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 13,
            fontStyle = FontStyle.Bold,
            padding = new RectOffset(0, 0, 2, 4),
            normal = { textColor = Color.white }
        };

        _valueStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 11,
            alignment = TextAnchor.MiddleRight,
            padding = new RectOffset(0, 2, 1, 1),
            normal = { textColor = Color.white }
        };

        _stylesBuilt = true;
    }
}