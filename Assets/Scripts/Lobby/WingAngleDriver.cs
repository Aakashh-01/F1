using UnityEngine;

namespace F1.Lobby
{
    /// <summary>
    /// Drives the rear wing's moving element between a high-downforce and a low-downforce
    /// pose. This is the prototype that answers one question: does an angle change on this
    /// part actually read on screen, or is it too small and too far from the car to matter?
    ///
    /// Why a pivot has to be inserted rather than rotating the DRS node directly
    /// --------------------------------------------------------------------
    /// The imported model names its wing pivots (`DRS1_165_250`) but they carry IDENTITY
    /// transforms, so their origin sits at the model origin in the middle of the car, not at
    /// the wing. Rotating one swings the whole flap through a huge arc around the car's
    /// centre instead of pivoting it on its hinge.
    ///
    /// The hinge is derived from the flap's own world bounds rather than hard-coded, because
    /// the car's scale and position are set per-scene and a fixed number would drift.
    ///
    /// A note on the parent scale
    /// --------------------------
    /// `Car_Model` carries a non-uniform scale of (1.8, 0.6, 4), and Unity applies a parent's
    /// scale BEFORE a child's rotation. Rotating under it therefore shears the flap rather
    /// than pivoting it rigidly. How badly that shows is the thing this prototype is here to
    /// find out, so the shear is left in and judged by eye rather than pre-compensated on
    /// faith.
    /// </summary>
    [DisallowMultipleComponent]
    public class WingAngleDriver : MonoBehaviour
    {
        [Header("Target")]
        [Tooltip("The DRS flap — the small moving element on the trailing edge.")]
        [SerializeField] private string _flapNodeName = "DRS1_165_250";

        [Tooltip("The main rear wing plane. This is the element that actually reads on " +
                 "screen; the flap alone travels only about 6 cm and is invisible at lobby " +
                 "framing. Both are driven together so the combined travel is legible.")]
        [SerializeField] private string _mainWingNodeName = "rearwing_top_62_90";

        [Tooltip("How far along the flap, front to back, the hinge sits. 0 = front edge, " +
                 "1 = back edge. A DRS flap is hinged at its leading edge.")]
        [SerializeField] [Range(0f, 1f)] private float _hingeAlongDepth = 0.06f;

        [Tooltip("Lift the hinge to the top of the flap, where a real flap pivots from.")]
        [SerializeField] [Range(0f, 1f)] private float _hingeAcrossHeight = 0.75f;

        [Tooltip("Same two measurements for the main plane's hinge.")]
        [SerializeField] [Range(0f, 1f)] private float _mainHingeAlongDepth = 0.06f;
        [SerializeField] [Range(0f, 1f)] private float _mainHingeAcrossHeight = 0.8f;

        [Header("Poses")]
        [Tooltip("Downforce setup: the wing sits steep, pushing the car down.")]
        [SerializeField] private float _highDownforceAngle = 16f;

        [Tooltip("Low-drag setup: the wing lies much flatter. The pair is deliberately " +
                 "narrow — see the note on readings below.")]
        [SerializeField] private float _lowDownforceAngle = 3f;

        [Header("Motion")]
        [SerializeField] private float _approach = 6f;

        private Transform _mainPivot;
        private Transform _flapPivot;
        private Transform _flap;
        private float _angle;
        private float _target;

        /// <summary>True when both pivots were found and the wing is under control.</summary>
        public bool IsRigged => _mainPivot != null && _flapPivot != null && _flap != null;

        public float CurrentAngle => _angle;
        public float TargetAngle => _target;

        private void Awake()
        {
            Rig();
        }

        /// <summary>
        /// Finds both wing elements, measures them, and inserts a pivot on each hinge line.
        /// Safe to call twice; a second call finds the existing pivots and does nothing.
        ///
        /// MUST run at runtime. In the editor, Transform.SetParent on these objects is
        /// silently refused — the glTF sits inside a nested prefab instance — with no
        /// exception and nothing in the console, so the rig appears to succeed and the wing
        /// then does nothing when rotated.
        /// </summary>
        public void Rig()
        {
            if (IsRigged) return;

            _flap = FindDeep(transform, _flapNodeName);
            if (_flap == null)
            {
                Debug.LogError($"[WingAngleDriver] No node named '{_flapNodeName}' under " +
                               $"{name}. The wing cannot be driven.", this);
                return;
            }

            // Refuse to rig twice: a second pivot under the same element would fight the first.
            _flapPivot = FindExistingHinge(_flap);
            if (_flapPivot == null) _flapPivot = InsertHinge(_flap, _hingeAlongDepth, _hingeAcrossHeight);

            var mainWing = FindDeep(transform, _mainWingNodeName);
            if (mainWing == null)
            {
                Debug.LogError($"[WingAngleDriver] No node named '{_mainWingNodeName}' under " +
                               $"{name}.", this);
                return;
            }
            _mainPivot = FindExistingHinge(mainWing);
            if (_mainPivot == null) _mainPivot = InsertHinge(mainWing, _mainHingeAlongDepth, _mainHingeAcrossHeight);
        }

        /// <summary>Returns an already-inserted hinge above this element, or null.</summary>
        private static Transform FindExistingHinge(Transform element)
        {
            for (var p = element.parent; p != null; p = p.parent)
            {
                if (p.name == "WingHinge" || p.name == "MainWingHinge") return p;
            }
            return null;
        }

        /// <summary>
        /// Inserts a pivot at the element's hinge line and parents the element under it,
        /// leaving its world pose untouched.
        /// </summary>
        private Transform InsertHinge(Transform element, float alongDepth, float acrossHeight)
        {
            var renderers = element.GetComponentsInChildren<Renderer>(true);
            if (renderers.Length == 0)
            {
                Debug.LogError($"[WingAngleDriver] '{element.name}' has no renderers, so " +
                               "there is nothing to measure a hinge from.", this);
                return null;
            }

            var bounds = renderers[0].bounds;
            foreach (var r in renderers) bounds.Encapsulate(r.bounds);

            // Hinge on the leading edge, lifted to the top. "Leading" is toward the nose;
            // the car's nose is +Z, so that is the max-Z face of the bounds.
            var hinge = new Vector3(
                bounds.center.x,
                Mathf.Lerp(bounds.min.y, bounds.max.y, acrossHeight),
                Mathf.Lerp(bounds.max.z, bounds.min.z, alongDepth));

            var pivotGo = new GameObject(element == _flap ? "WingHinge" : "MainWingHinge");
            pivotGo.transform.SetParent(element.parent, true);
            pivotGo.transform.position = hinge;

            // worldPositionStays keeps the element exactly where it was, so rigging is
            // invisible until something actually rotates the pivot.
            element.SetParent(pivotGo.transform, true);
            return pivotGo.transform;
        }

        private static Transform FindDeep(Transform root, string name)
        {
            var stack = new System.Collections.Generic.Stack<Transform>();
            stack.Push(root);
            while (stack.Count > 0)
            {
                var t = stack.Pop();
                if (t.name == name) return t;
                for (int i = 0; i < t.childCount; i++) stack.Push(t.GetChild(i));
            }
            return null;
        }

        /// <summary>True for the steep, downforce-producing pose.</summary>
        public void SetHighDownforce() => _target = _highDownforceAngle;

        /// <summary>True for the open, low-drag pose.</summary>
        public void SetLowDownforce() => _target = _lowDownforceAngle;

        /// <summary>Snaps straight to a pose with no animation, for framing a screenshot.</summary>
        public void SetAngleImmediate(float degrees)
        {
            _angle = _target = degrees;
            Apply();
        }

        public void SetHighDownforceImmediate() => SetAngleImmediate(_highDownforceAngle);
        public void SetLowDownforceImmediate() => SetAngleImmediate(_lowDownforceAngle);

        private void Update()
        {
            if (!IsRigged) return;
            if (Mathf.Approximately(_angle, _target)) return;

            _angle = Mathf.Lerp(_angle, _target, 1f - Mathf.Exp(-_approach * Time.deltaTime));
            Apply();
        }

        private void Apply()
        {
            // Guarded, because not every caller goes through Update. Update checks IsRigged
            // itself, but SetAngleImmediate is public and does not — and the wing screen
            // calls the Immediate variants directly to pose the wing to the session's
            // current setup. A driver whose nodes were not found (wrong model, renamed
            // parts) or that has not run Awake yet — which is every driver outside play
            // mode, since rigging is deferred to Awake because SetParent is refused in the
            // editor — would otherwise null-dereference both pivots.
            //
            // The pose is still recorded in _angle and _target above, so if the rig does
            // appear later, Update eases the wing to the pending target on its own.
            if (!IsRigged) return;

            // About local X: the hinge runs across the car, so the wing swings up and down.
            // The main plane takes the full angle and the flap a little more, so the two
            // together give a combined travel that reads at a close framing.
            _mainPivot.localRotation = Quaternion.Euler(_angle, 0f, 0f);
            _flapPivot.localRotation = Quaternion.Euler(_angle * 1.6f, 0f, 0f);
        }
    }
}
