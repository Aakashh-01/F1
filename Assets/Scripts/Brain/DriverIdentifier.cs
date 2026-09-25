using TMPro;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Puts a driver's grid slot and name on a board above an AI car, and turns that board to
/// face the camera.
///
/// This exists because the project owns one AI car prefab. Every car in the field is that
/// same prefab, so the cars are visually identical and there is nothing to tell them apart
/// with. The board is what gives each one an identity: the big label is the grid slot the car
/// started from, which is where the qualifying result put it, and the name underneath is who
/// is driving. The car also carries its race number, because that is the identity the HUD
/// standings use — so "P5" on a board and "5" in the standings are the same car, and neither
/// display has to invent its own numbering.
///
/// It is a board and not a livery on purpose. The car's paint is baked into textures in the
/// imported glTF, with no colour property a material instance could tint, so per-car liveries
/// are not available without reworking the model. A board works with any prefab, and number
/// boards are what a real grid looks like anyway.
/// </summary>
[DisallowMultipleComponent]
public class DriverIdentifier : MonoBehaviour
{
    [Header("Identity")]
    [Tooltip("Shown under the number. Set through SetIdentity, not usually by hand.")]
    [SerializeField] private string _driverName = "AI Driver";

    [Tooltip("Race number, shown on the board when the car was never placed on a grid. Zero " +
             "leaves the board with no headline at all, just the name.")]
    [SerializeField] private int _number;

    [Tooltip("Grid slot the car started from, zero-based as the grid manager counts them, so " +
             "0 is pole. Shown as the big label ('P1') when set. -1 means the car was never " +
             "placed on a grid. Set by the grid manager at spawn.")]
    [SerializeField] private int _gridSlot = NoGridSlot;

    [Header("Presentation")]
    [Tooltip("How far above the car the board floats, in the car's local space.")]
    [SerializeField] private float _heightAboveCar = 2.1f;

    [Tooltip("World size of the board. A race car is about 5 m long, so this reads as a board " +
             "rather than a billboard.")]
    [SerializeField] private float _boardWidthMetres = 1.6f;

    [Tooltip("Show the board at all. Off gives a clean car for screenshots or a replay.")]
    [SerializeField] private bool _visible = true;

    [Tooltip("Hide the board when the camera is closer than this, so it does not fill the " +
             "screen when the car is right in front of the chase camera.")]
    [SerializeField] private float _hideWithinMetres = 3.5f;

    private GameObject _board;
    private Canvas _canvas;
    private TMP_Text _numberText;
    private TMP_Text _nameText;
    private Camera _cachedCamera;

    /// <summary>Driver name shown on the board.</summary>
    public string DriverName => _driverName;

    /// <summary>
    /// Sentinel for "this car was never placed on a grid".
    ///
    /// It has to be -1 rather than 0, because 0 is a real slot: the grid manager counts from
    /// zero and slot 0 is pole. Treating 0 as "unset" made the pole car fall back to its race
    /// number, so the front of the grid read "2" while the row behind it read "P1" and "P2" —
    /// a field whose labels disagreed with its order by exactly one car.
    /// </summary>
    public const int NoGridSlot = -1;

    /// <summary>Race number shown on the board, and the identity the HUD standings use.</summary>
    public int Number => _number;

    /// <summary>
    /// Grid slot the car started from, zero-based (0 is pole), or <see cref="NoGridSlot"/>.
    /// </summary>
    public int GridSlot => _gridSlot;

    /// <summary>
    /// The big label on the board: the grid slot when the car has one, otherwise the race
    /// number, otherwise nothing.
    ///
    /// The slot is displayed one-based because that is how a grid is labelled in every
    /// context a viewer brings to it — P1 is pole, not P0. The conversion lives here, on the
    /// one place that shows a slot, so the flow's own 1-based <c>RaceGridPosition</c> and the
    /// grid manager's 0-based slots can both stay in their natural units without either
    /// having to know about the other.
    /// </summary>
    public string BoardHeadline => _gridSlot >= 0
        ? $"P{_gridSlot + 1}"
        : (_number > 0 ? _number.ToString() : string.Empty);

    private void Awake()
    {
        BuildBoard();
        RefreshText();
    }

    private void OnDestroy()
    {
        if (_board != null)
            Destroy(_board);
    }

    /// <summary>
    /// Names the car, gives it a race number and a grid slot. Safe to call before or after
    /// Awake, so a grid manager can add the component and configure it in one step.
    ///
    /// The slot is optional because not every car on a grid is placed by one — a car spawned
    /// outside a race has a number and a name but nowhere it started from.
    /// </summary>
    public void SetIdentity(string driverName, int number, int gridSlot = NoGridSlot)
    {
        if (!string.IsNullOrWhiteSpace(driverName))
            _driverName = driverName;

        _number = number;
        _gridSlot = gridSlot < 0 ? NoGridSlot : gridSlot;
        RefreshText();
    }

    private void LateUpdate()
    {
        if (_board == null)
            return;

        Camera cam = ResolveCamera();
        if (cam == null)
        {
            _board.SetActive(false);
            return;
        }

        float distance = Vector3.Distance(cam.transform.position, _board.transform.position);
        if (!_visible || distance < _hideWithinMetres)
        {
            if (_board.activeSelf) _board.SetActive(false);
            return;
        }

        if (!_board.activeSelf) _board.SetActive(true);

        // The canonical world-space-canvas billboard. Looking at the camera and then
        // spinning 180 is what puts the front face of the canvas toward the viewer; without
        // the spin the board faces away and renders as nothing.
        Transform board = _board.transform;
        board.LookAt(cam.transform.position, Vector3.up);
        board.Rotate(0f, 180f, 0f);
    }

    private Camera ResolveCamera()
    {
        // The chase camera is driven by Cinemachine, so the tag holder is the real camera
        // rather than any virtual one. Re-resolved only when it goes away, because a car can
        // outlive a camera during a transition.
        if (_cachedCamera == null)
            _cachedCamera = Camera.main;

        return _cachedCamera;
    }

    private void BuildBoard()
    {
        // Authored in canvas units, scaled to metres at the end.
        //
        // This is the whole trick to a world space canvas, and getting it backwards is
        // spectacular rather than subtle: TMP measures a glyph in the canvas's own units, so
        // a board whose sizeDelta is 1.6 units wide and which asks for font size 44 renders
        // text 44 times taller than the entire board. The board is therefore laid out at a
        // resolution where a font size means something, and then scaled down to the metres
        // it should occupy in the world.
        const float boardWidthUnits = 512f;
        float width = boardWidthUnits;
        float height = boardWidthUnits * 0.42f;

        _board = new GameObject("DriverBoard", typeof(RectTransform));
        _board.transform.SetParent(transform, false);
        _board.transform.localPosition = new Vector3(0f, _heightAboveCar, 0f);

        _canvas = _board.AddComponent<Canvas>();
        _canvas.renderMode = RenderMode.WorldSpace;

        var rect = (RectTransform)_board.transform;
        rect.sizeDelta = new Vector2(width, height);
        rect.localScale = Vector3.one * (_boardWidthMetres / boardWidthUnits);

        _canvas.sortingOrder = 50;

        // The board is decoration. It must never intercept a click meant for a UI button
        // underneath it, which is exactly what a world space canvas would otherwise do.
        var group = _board.AddComponent<CanvasGroup>();
        group.blocksRaycasts = false;
        group.interactable = false;

        // A plate behind the text. Without it the board is white glyphs over whatever is
        // behind it — sky, tarmac, grandstand — and over a bright sky it is simply not
        // readable, which defeats the point of a board that exists to be read.
        CreatePlate(rect, width, height);

        _numberText = CreateText(rect, "Number", new Vector2(0.5f, 0.62f),
            new Vector2(width * 0.9f, height * 0.55f), 44, TextAlignmentOptions.Center);
        _nameText = CreateText(rect, "Name", new Vector2(0.5f, 0.2f),
            new Vector2(width * 0.95f, height * 0.34f), 22, TextAlignmentOptions.Center);
    }

    /// <summary>
    /// The dark backing panel. Added first so it sits behind the labels in the hierarchy.
    /// </summary>
    private void CreatePlate(RectTransform parent, float width, float height)
    {
        var go = new GameObject("Plate", typeof(RectTransform));
        go.transform.SetParent(parent, false);

        var rect = (RectTransform)go.transform;
        rect.anchorMin = Vector2.zero;
        rect.anchorMax = Vector2.one;
        rect.offsetMin = Vector2.zero;
        rect.offsetMax = Vector2.zero;

        var image = go.AddComponent<UnityEngine.UI.Image>();
        // A plain Image with no sprite draws a solid quad, which is all a plate needs.
        image.color = new Color(0.06f, 0.06f, 0.08f, 0.85f);
        image.raycastTarget = false;
    }

    private TMP_Text CreateText(RectTransform parent, string name, Vector2 anchor,
        Vector2 size, int fontSize, TextAlignmentOptions alignment)
    {
        var go = new GameObject(name, typeof(RectTransform));
        go.transform.SetParent(parent, false);

        var rect = (RectTransform)go.transform;
        rect.anchorMin = anchor;
        rect.anchorMax = anchor;
        rect.pivot = new Vector2(0.5f, 0.5f);
        rect.sizeDelta = size;
        rect.anchoredPosition = Vector2.zero;

        var text = go.AddComponent<TextMeshProUGUI>();
        text.fontSize = fontSize;
        text.alignment = alignment;
        text.color = Color.white;
        text.enableWordWrapping = false;
        text.raycastTarget = false;
        return text;
    }

    private void RefreshText()
    {
        if (_numberText == null || _nameText == null)
            return;

        _numberText.text = BoardHeadline;
        _nameText.text = _driverName ?? string.Empty;
    }
}
