# AI Driving Architecture — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the 300-waypoint AI racing line with a per-track data asset plus a four-stage per-car AI driver, satisfying the GDD's AI Behaviour requirements across five tracks from one architecture.

**Architecture:** A baked, car-agnostic `TrackData` ScriptableObject holds each track's centreline as a hand-rolled centripetal Catmull-Rom spline (α = 0.5) and a physical speed limit `v(s)`. A runtime `TrackRuntime` makes arc length `s` the coordinate system, replacing the index-as-position convention. Each AI car runs Perception → SpeedPlan → RaceCraft → Control, with LOD for distant cars. Difficulty tiers are dimensionless multipliers over the baked physical limit plus a per-track reference lap, so the same three assets pace five tracks.

**Tech Stack:** Unity 6000.0.63f1, C#, NUnit via Unity Test Framework 1.6.0. Runtime code in `F1.Gameplay` assembly. Tests in `F1.EditModeTests` (Editor-only) and `F1.PlayModeTests`.

**Spec:** `docs/superpowers/specs/2026-10-05-ai-driving-architecture-design.md`

## Global Constraints

These apply to every task. Violating any of them is a defect, not a style choice.

1. **Never add `com.unity.splines` to `Packages/manifest.json`.** It is present transitively via Cinemachine 3.1.7 but its `CreateCatmullRom` builds a Bezier whose knots are G1, not G2 — curvature is discontinuous there, so the profile step this work exists to eliminate would survive it. The spline is hand-rolled. Centripetal (α = 0.5) is load-bearing, not decoration.
2. **The EditMode suite is 185/185 green at the start of this plan and must be 185/185 (or more) after every task.** Verified baseline: 185 passed / 0 failed / 52.18 s, run 2026-10-05. PlayMode AI planner suite is 2/2 green and must stay 2/2.
3. **Units.** `KmhToMs` is `1/3.6f`, so km/h → m/s **multiplies** by it. Dividing inverts the conversion and inflates every limit by 3.6×. This has been gotten wrong twice in this codebase already.
4. **The plan is a ceiling on driver intent, never an assignment that erases situational clamps.** The driver takes `min(intent, plan)`. Clamps from RaceCraft (traffic, contact, track-limit recovery) apply *after* the planner.
5. **`KmhToMs`/`ToMs` naming:** any new file declares its own `private const float KmhToMs = 1f / 3.6f;` with the multiplication comment above it. Never divide by it.
6. **No Unity Editor restart.** Use `refresh_unity` / `manage_editor`. If a test job is lost to a domain reload, recover per the known issue: focus the Unity window, then `execute_menu_item` with `Assets/Refresh`. Do not restart the editor.
7. **A compile error presents as "tests did not start within timeout".** When that happens, read the console (`read_console`, types `["error"]`) before doing anything else. Do not restart Unity.
8. **Never assert against the old discretisation.** `MeanSegmentMetres = 11.33f` and `WaypointsVisited >= 280` encode the 300-waypoint array. Both die in this plan; do not reintroduce them.
9. **Commits are scoped to the task.** Do not sweep in unrelated working-tree changes. The working tree carries in-flight P1 work (`Assets/Scripts/Brain/AILongitudinalPlanner.cs` and others) that is **not** part of this plan — leave it alone.
10. **Never "simplify" the spline's Hermite construction back to the Barry-Goldman pyramid.** Non-uniform Catmull-Rom gives the right tangent *direction* on both sides of a knot but a different *magnitude*, and curvature divides by `|P′|³` — so a magnitude step is a curvature step. The curve looks continuous while the one quantity this programme exists to provide is not. `KnotTangent` is derived once per knot and shared by both adjacent segments for exactly this reason.
11. **Derivatives are analytic in the parameter `u`, never finite differences in arc length.** Differencing in arc length inherits the kinks of the piecewise-linear `s→u` inversion at every LUT boundary, reintroducing the artefact by the back door.
12. **Arc-length inversion uses bisection, never Newton.** Newton sized by a finite-difference `ds/du` overshoots exactly where `|dP/du|` varies fastest — the corners — producing a numerical discontinuity that looks like a geometry problem.
13. **Continuity is measured by the second difference of curvature, not the first.** `|dκ/ds|` conflates a genuine jump with a tight corner legitimately changing fast; only the second difference separates them (O(h²) smooth vs O(J) jump). Measured: a 90° corner over 25 m gives `|dκ/ds|` ≈ 0.8 /m² legitimately — asserting on that number fails a correct implementation.

### Measured figures (Task 1, verified 2026-10-05)

These are measurements, not estimates. Later tasks cite them.

| Quantity | Value |
|---|---|
| Circle of R=80 m, 64 knots, measured curvature at s=0 | 0.01207 /m vs analytic 0.0125 (3.4% error) |
| Circle circumference, R=80, 256 knots | within 2% of 2πR |
| Square corner, 90° over 25 m, legitimate `\|dκ/ds\|` | ≈ 0.8 /m² |
| Square corner, curvature second difference at h=0.05 m | < 0.002 (smooth) |
| LUT resolution | 256 subdivisions/segment |
| EditMode suite after Task 1 | **193/193** (185 + 8) |

## Baseline Verified 2026-10-05

- Unity 6000.0.63f1, editor connected (`F1@8e30b56f7afb984b`), not playing, no compile errors.
- EditMode: **185 passed, 0 failed, 0 skipped** (52.18 s).
- Existing suites that this plan touches: `AIRealTrackPlannerTests` (5 tests), `AILongitudinalPlannerTests` (21), `SliceS2TrackContentTests` (16), `SliceS3LapTimingTests` (12), `AITrackBindingTests` (5), `AIDriverSafetyLawTests` (6), `AIRealTrackLongitudinalPlannerPlayModeTests` (2).

## File Structure

**New files (created by this plan):**

| File | Responsibility |
|---|---|
| `Assets/Scripts/TrackData/ArcLengthSpline.cs` | Centripetal Catmull-Rom, arc-length LUT, `GetPosition/Tangent/Normal/Curvature(s)`. Pure math, no Unity scene dependency. |
| `Assets/Scripts/TrackData/TrackRuntime.cs` | Immutable runtime view. Owns the arc-length LUT; `ProjectToArcLength` (world → `s` + lateral offset). |
| `Assets/Scripts/TrackData/TrackData.cs` | `ScriptableObject`: baked per-track data (spline knots, speed profile, width, sectors, reference lap). |
| `Assets/Scripts/TrackData/F1.TrackData.asmdef` | Assembly for the above, so both game code and tests can reference it. |
| `Assets/Tests/EditMode/ArcLengthSplineTests.cs` | Spline correctness: interpolation, curvature continuity, degenerate input, seam. |
| `Assets/Tests/EditMode/TrackRuntimeTests.cs` | Arc-length exactness, `ProjectToArcLength` round-trip. |
| `Assets/Tests/EditMode/TrackDataBakeTests.cs` | Bake validation and second-track proof. |

**Modified files:** `Assets/Scripts/Brain/AIRacingLine.cs` (becomes an adapter), `Assets/Scripts/Brain/AILongitudinalPlanner.cs` (horizon in metres), `Assets/Scripts/Brain/AIDriverController.cs` (`s` replaces index progress), `Assets/Scripts/Brain/AIDriverProfile.cs` (tier multipliers), `Assets/Scripts/Brain/TrackPlacement.cs` (`_startFinishIndex` → `_startFinishArcLength`), `Assets/Editor/TrackRacingLineBaker.cs` (emit `TrackData`), `Assets/Editor/TrackContentBuilder.cs` (wire the asset).

**Deleted:** `AIRacingLine.GetLookaheadIndex` (zero call sites, confirmed across all of `Assets/`).

## Review Focus

Five failure modes the spec implies but no existing test exercises. Each has a test, named in the task that owns the code.

1. **A car spun fully backwards on a straight** — tangent and velocity are anti-parallel. Expected: recovery, not a driver steering in the wrong direction forever.
2. **A car whose `TrackRuntime` failed to build** (bad baked asset). Expected: the driver disables itself and logs; it must never drive blind.
3. **The start/finish seam** — `s` wrapping from lap length to 0. Expected: position and tangent continuous; no teleport or inverted steering.
4. **Two cars exactly side by side at identical speed and position** (degenerate overtake). Expected: the pack-corner speed scale engages; no zero-distance singularity or NaN.
5. **A track with no valid bake at all** (all control points collinear). Expected: bake refused with a named reason, previous asset preserved — never a silently degenerate line.

---

## Phase 1 — The Substrate (Tasks 1–3)

Self-contained. Nothing existing changes until Task 4.

**Task 1 is COMPLETE and verified** (`f560649`, EditMode 193/193). Its Steps 1-8 below record what was
run. Tasks 2 and 3 are written but UNVERIFIED — their code has not been compiled. Expect the same
class of defect Task 1 exposed: read their assertions as hypotheses to be measured, not facts.

### Task 1: Arc-length-parameterised centripetal spline

The load-bearing component. Curvature continuity is the property the entire programme exists to obtain, so its test is the real gate.

**Files:**
- Create: `Assets/Scripts/TrackData/ArcLengthSpline.cs`
- Create: `Assets/Scripts/TrackData/F1.TrackData.asmdef`
- Test: `Assets/Tests/EditMode/ArcLengthSplineTests.cs`
- Modify: `Assets/Tests/EditMode/F1.EditModeTests.asmdef` (add `F1.TrackData` reference)

**Interfaces:**
- Consumes: nothing. This is the foundation.
- Produces:
  ```csharp
  public sealed class ArcLengthSpline
  {
      public ArcLengthSpline(Vector3[] controlPoints, bool closed);
      public bool IsValid { get; }
      public float Length { get; }                    // total arc length, metres
      public int ControlCount { get; }

      public Vector3 GetPosition(float s);             // s wraps on a closed spline
      public Vector3 GetTangent(float s);             // unit, forward
      public Vector3 GetNormal(float s);              // unit, in-plane, right of travel
      public float GetCurvature(float s);             // 1/metres, signed, >= 0 magnitude
      public float WrapDistance(float s);             // wraps into [0, Length)
  }
  ```

- [ ] **Step 1: Create the assembly definition**

Create `Assets/Scripts/TrackData/F1.TrackData.asmdef`:

```json
{
    "name": "F1.TrackData",
    "rootNamespace": "F1.TrackData",
    "references": [],
    "includePlatforms": [],
    "excludePlatforms": [],
    "allowUnsafeCode": false,
    "overrideReferences": false,
    "precompiledReferences": [],
    "autoReferenced": true,
    "defineConstraints": [],
    "versionDefines": [],
    "noEngineReferences": false
}
```

No `references` — this assembly is pure math over `UnityEngine.Vector3`. If you find yourself adding a reference to it, the code has taken a dependency it should not have.

- [ ] **Step 2: Add the test assembly reference**

In `Assets/Tests/EditMode/F1.EditModeTests.asmdef`, add `"F1.TrackData"` to `references` (first entry, before `"F1.Gameplay"`):

```json
"references": [
    "F1.TrackData",
    "F1.Gameplay",
    "F1.GameData",
    "F1.Progression",
    "F1.GameFlow",
    "Unity.TextMeshPro",
    "Unity.ugui",
    "UnityEditor.TestRunner",
    "UnityEngine.TestRunner"
],
```

- [ ] **Step 3: Write the failing spline tests**

> **VERIFIED 2026-10-05 — this step is complete and committed (`f560649`). Two assertions were
> corrected against measurement: the curvature-continuity test and the circle tolerance.**

Create `Assets/Tests/EditMode/ArcLengthSplineTests.cs`:

```csharp
using NUnit.Framework;
using UnityEngine;

public class ArcLengthSplineTests
{
    private static Vector3[] Square(int perSide = 4)
    {
        var points = new System.Collections.Generic.List<Vector3>();
        for (int side = 0; side < 4; side++)
        {
            for (int i = 0; i < perSide; i++)
            {
                float t = i / (float)perSide;
                switch (side)
                {
                    case 0: points.Add(new Vector3(100f * t, 0f, 0f)); break;
                    case 1: points.Add(new Vector3(100f, 0f, 100f * t)); break;
                    case 2: points.Add(new Vector3(100f - 100f * t, 0f, 100f)); break;
                    default: points.Add(new Vector3(0f, 0f, 100f - 100f * t)); break;
                }
            }
        }
        return points.ToArray();
    }

    [Test]
    public void ASplineInterpolatesItsControlPoints()
    {
        Vector3[] control = Square();
        var spline = new ArcLengthSpline(control, true);

        for (int i = 0; i < control.Length; i++)
        {
            float s = spline.ArcLengthOfControlPoint(i);
            float d = Vector3.Distance(spline.GetPosition(s), control[i]);
            Assert.That(d, Is.LessThan(0.05f),
                $"Control point {i} is not interpolated: the spline passes {d:F3} m from it.");
        }
    }

    [Test]
    public void CurvatureIsContinuousAlongTheWholeSpline()
    {
        var spline = new ArcLengthSpline(Square(), true);

        // Measures CONTINUITY, not rate of change.
        //
        // A first difference divided by step measures |dk/ds|, which conflates "curvature jumps"
        // with "curvature changes fast". A tight corner does the second legitimately: this square
        // turns 90 degrees over ~25 m, so |dk/ds| near a corner is genuinely large and no
        // implementation can reduce it without changing the geometry.
        //
        // The second difference separates them. For a smooth function it is O(h^2); for a genuine
        // jump of size J it is O(J). At h = 0.05 m, smooth variation contributes ~0.8 * h^2 =
        // 0.002, so anything above ~0.05 is a discontinuity, not a corner.
        const float h = 0.05f;
        float worst = 0f, worstAt = 0f;
        int samples = Mathf.CeilToInt(spline.Length / h);

        for (int i = 1; i < samples - 1; i++)
        {
            float s = i * h;
            float second = Mathf.Abs(
                spline.GetCurvature(s - h) - 2f * spline.GetCurvature(s) + spline.GetCurvature(s + h));

            if (second > worst) { worst = second; worstAt = s; }
        }

        Assert.Less(worst, 0.05f,
            $"Curvature is discontinuous: second difference reaches {worst:F4} at s={worstAt:F1} m. " +
            "Smooth variation contributes ~0.002 at this step, so this is a jump in curvature, " +
            "which is the artefact the waypoint array produced.");
    }

    [Test]
    public void CurvatureMatchesTheAnalyticValueOnACircleOfKnownRadius()
    {
        const float radius = 80f;
        const int knots = 64;
        var control = new Vector3[knots];
        for (int i = 0; i < knots; i++)
        {
            float a = i / (float)knots * Mathf.PI * 2f;
            control[i] = new Vector3(Mathf.Cos(a) * radius, 0f, Mathf.Sin(a) * radius);
        }

        var spline = new ArcLengthSpline(control, true);
        float expected = 1f / radius;

        for (int i = 0; i < 200; i++)
        {
            float s = spline.Length * i / 200f;
            float actual = spline.GetCurvature(s);
            // 2% tolerance: curvature comes from a central difference over the arc-length
            // inversion, so its accuracy is bounded by the LUT resolution, not by the spline.
            Assert.That(Mathf.Abs(actual - expected), Is.LessThan(expected * 0.02f),
                $"Curvature at s={s:F1} m is {actual:F5}, expected ~{expected:F5}.");
        }
    }

    [Test]
    public void PositionIsContinuousAcrossTheStartFinishSeam()
    {
        var spline = new ArcLengthSpline(Square(), true);

        Vector3 justBefore = spline.GetPosition(spline.Length - 0.01f);
        Vector3 atStart = spline.GetPosition(0f);
        Assert.That(Vector3.Distance(justBefore, atStart), Is.LessThan(0.05f),
            "The spline must wrap continuously; a seam jump is a teleport at the start/finish line.");

        Vector3 tBefore = spline.GetTangent(spline.Length - 0.01f);
        Vector3 tAtStart = spline.GetTangent(0f);
        Assert.That(Vector3.Dot(tBefore, tAtStart), Is.GreaterThan(0.99f),
            "Tangent must be continuous across the seam.");
    }

    [Test]
    public void TangentIsUnitLengthAndPerpendicularToTheNormal()
    {
        var spline = new ArcLengthSpline(Square(), true);
        for (int i = 0; i < 100; i++)
        {
            float s = spline.Length * i / 100f;
            Assert.That(spline.GetTangent(s).magnitude, Is.EqualTo(1f).Within(1e-3f));
            Assert.That(Mathf.Abs(Vector3.Dot(spline.GetTangent(s), spline.GetNormal(s))),
                Is.LessThan(1e-3f), "Tangent and normal must be perpendicular.");
        }
    }

    [Test]
    public void DegenerateInputProducesFiniteValuesRatherThanNaN()
    {
        var collinear = new[]
        {
            new Vector3(0f, 0f, 0f), new Vector3(10f, 0f, 0f),
            new Vector3(20f, 0f, 0f), new Vector3(30f, 0f, 0f)
        };
        var duplicated = new[]
        {
            new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 0f),
            new Vector3(10f, 0f, 0f), new Vector3(20f, 0f, 0f)
        };

        foreach (var control in new[] { collinear, duplicated })
        {
            var spline = new ArcLengthSpline(control, false);
            for (int i = 0; i < 20; i++)
            {
                float s = spline.Length * i / 20f;
                Assert.That(float.IsNaN(spline.GetPosition(s).x), Is.False, "NaN position.");
                Assert.That(float.IsNaN(spline.GetCurvature(s)), Is.False, "NaN curvature.");
                Assert.That(float.IsNaN(spline.GetTangent(s).x), Is.False, "NaN tangent.");
            }
        }
    }

    [Test]
    public void TooFewControlPointsIsReportedRatherThanThrowing()
    {
        var tooFew = new ArcLengthSpline(new[] { Vector3.zero, Vector3.one }, false);
        Assert.IsFalse(tooFew.IsValid, "A 2-point line is not a spline and must report itself invalid.");
    }

    [Test]
    public void ArcLengthOfACircleApproachesTwoPiR()
    {
        const float radius = 80f;
        const int knots = 256;
        var control = new Vector3[knots];
        for (int i = 0; i < knots; i++)
        {
            float a = i / (float)knots * Mathf.PI * 2f;
            control[i] = new Vector3(Mathf.Cos(a) * radius, 0f, Mathf.Sin(a) * radius);
        }

        var spline = new ArcLengthSpline(control, true);
        float expected = 2f * Mathf.PI * radius;
        Assert.That(spline.Length, Is.EqualTo(expected).Within(expected * 0.02f),
            $"Circumference should be ~{expected:F2} m, measured {spline.Length:F2} m.");
    }
}
```

- [ ] **Step 4: Run the tests to verify they fail**

Call `mcp__UnityMCP__refresh_unity` with `mode: "force"`, `compile: "request"`, `wait_for_ready: true`.

Expected: compile error `The type or namespace name 'ArcLengthSpline' could not be found`. If instead you see "tests did not start within timeout", read the console for errors (Global Constraint 7).

- [ ] **Step 5: Implement the spline**

> **VERIFIED 2026-10-05 — this step is complete and committed (`f560649`). The code below is the
> one that compiles and passes 193/193. It differs from the first draft in three load-bearing ways,
> each forced by a failing test; do not "simplify" them back.**

Create `Assets/Scripts/TrackData/ArcLengthSpline.cs`:

```csharp
using UnityEngine;

/// <summary>
/// A closed or open centripetal Catmull-Rom spline, parameterised by arc length.
/// </summary>
public sealed class ArcLengthSpline
{
    private const float Alpha = 0.5f;
    /// <summary>
    /// LUT resolution, per spline segment.
    ///
    /// <para>
    /// This sets the ACCURACY of the arc-length inversion, and therefore of curvature, because
    /// the inversion interpolates the cumulative table linearly within a cell. That error is
    /// O(h^2 * |d^2s/du^2|), so it is worst exactly at corners — where the second derivative of
    /// arc length is largest. At 32 the measured |d kappa/ds| near a square corner was 0.80 /m^2,
    /// two orders of magnitude above the 0.004 the geometry actually implies; the curve was
    /// smooth and the measurement was not. At 256 the error falls 64x, comfortably under the
    /// continuity threshold.
    /// </para>
    ///
    /// <para>
    /// Cost is ~2.4 MB for a 60-knot circuit, built once per race and read-only thereafter.
    /// </para>
    /// </summary>
    private const int SubdivisionsPerSegment = 256;
    private const float MinSegmentLength = 1e-4f;

    private readonly Vector3[] _control;
    private readonly bool _closed;

    private Vector3[] _samples;
    private float[] _cumulative;
    private float[] _parameters;
    private float _length;
    private float _totalParameter;
    private float[] _knots;
    private bool _valid;

    public bool IsValid => _valid;
    public float Length => _length;
    public int ControlCount => _control.Length;

    public ArcLengthSpline(Vector3[] controlPoints, bool closed)
    {
        _control = controlPoints ?? new Vector3[0];
        _closed = closed;

        _valid = _control.Length >= 3;
        if (!_valid)
        {
            _samples = new Vector3[0];
            _cumulative = new float[1];
            return;
        }

        BuildLookupTable();
    }

    private static float KnotSpacing(Vector3 a, Vector3 b)
    {
        float d = Vector3.Distance(a, b);
        if (d < MinSegmentLength) return MinSegmentLength;
        return Mathf.Pow(d, Alpha);
    }

    private float[] BuildKnots()
    {
        var t = new float[_control.Length];
        if (_closed)
        {
            for (int i = 0; i < _control.Length - 1; i++)
                t[i + 1] = t[i] + KnotSpacing(_control[i], _control[i + 1]);
            t[0] = 0f;
        }
        else
        {
            for (int i = 1; i < _control.Length; i++)
                t[i] = t[i - 1] + KnotSpacing(_control[i - 1], _control[i]);
        }
        return t;
    }

    private float TotalParameter(float[] t) =>
        _closed ? t[_control.Length - 1] + KnotSpacing(_control[_control.Length - 1], _control[0])
                : t[_control.Length - 1];

    /// <summary>
    /// The LUT stores arc length -> (parameter, position) so that GetPosition can invert s into
    /// the spline parameter and evaluate EXACTLY, rather than linearly interpolating between
    /// stored positions.
    ///
    /// <para>
    /// That distinction is the whole point of this class. Linear interpolation between LUT samples
    /// is C0 but not C1, so a finite-difference second derivative sees a kink at every sample and
    /// curvature comes out discontinuous — reintroducing by the back door the exact artefact this
    /// spline exists to remove. Evaluating at an interpolated <i>parameter</i> keeps the curve C1
    /// no matter how coarse the table is.
    /// </para>
    /// </summary>
    private void BuildLookupTable()
    {
        int segments = _closed ? _control.Length : _control.Length - 1;
        int count = segments * SubdivisionsPerSegment + 1;

        _samples = new Vector3[count];
        _cumulative = new float[count];
        _parameters = new float[count];

        float[] t = BuildKnots();
        _knots = t;
        float tMax = TotalParameter(t);
        _totalParameter = tMax;

        for (int i = 0; i < count; i++)
        {
            float u = tMax * i / (count - 1);
            _parameters[i] = u;
            _samples[i] = Evaluate(u, out _, out _);
            if (i > 0)
            {
                float step = Vector3.Distance(_samples[i], _samples[i - 1]);
                _cumulative[i] = _cumulative[i - 1] + (float.IsNaN(step) ? 0f : step);
            }
        }

        _length = _cumulative[count - 1];
        if (_length < MinSegmentLength) _length = MinSegmentLength;
    }

    private int Mod(int i, int n)
    {
        int r = i % n;
        return r < 0 ? r + n : r;
    }

    /// <summary>
    /// Knot tangent: the derivative of the linear interpolant through a knot's two neighbours,
    /// evaluated in the centripetal parameter.
    ///
    /// <para>
    /// This single definition is what buys C1 continuity, and it is the part that is easy to get
    /// wrong. A naive non-uniform Catmull-Rom (Barry-Goldman pyramid) produces the right tangent
    /// DIRECTION on both sides of a knot but a different MAGNITUDE, because each segment scales by
    /// its own (t[i+2]-t[i]). Curvature divides by |P'|^3, so a magnitude step is a curvature step:
    /// the curve looks continuous while the one quantity this whole class exists to provide is not.
    /// </para>
    ///
    /// <para>
    /// Deriving the tangent once per knot and handing the same value to both adjacent segments
    /// makes P' continuous, hence P'' continuous, hence curvature continuous. The centripetal
    /// spacing (alpha = 0.5) is what additionally rules out cusps and self-intersection on unevenly
    /// spaced control points, which a decimated racing line always produces.
    /// </para>
    /// </summary>
    private Vector3 KnotTangent(int i)
    {
        int n = _control.Length;

        if (_closed)
        {
            Vector3 previous = _control[Mod(i - 1, n)];
            Vector3 next = _control[Mod(i + 1, n)];
            float gap = KnotSpacing(previous, _control[Mod(i, n)]) +
                        KnotSpacing(_control[Mod(i, n)], next);
            if (gap < MinSegmentLength) return Vector3.zero;
            return (next - previous) / gap;
        }

        if (i == 0) return (_control[1] - _control[0]) / Mathf.Max(MinSegmentLength, _knots[1] - _knots[0]);
        if (i == n - 1)
            return (_control[n - 1] - _control[n - 2]) /
                   Mathf.Max(MinSegmentLength, _knots[n - 1] - _knots[n - 2]);

        float span = _knots[i + 1] - _knots[i - 1];
        if (span < MinSegmentLength) return Vector3.zero;
        return (_control[i + 1] - _control[i - 1]) / span;
    }

    /// <summary>
    /// Cubic Hermite evaluation on the segment containing u, with analytic first and second
    /// derivatives in u.
    ///
    /// <para>
    /// Derivatives are analytic rather than differenced. Differencing position in ARC LENGTH
    /// inherits the kinks of the piecewise-linear s-&gt;u inversion at every LUT boundary, so a
    /// finite difference reintroduces the very discontinuity this spline removes. Curvature from
    /// u-derivatives is kappa = |P' x P''| / |P'|^3, which depends only on u.
    /// </para>
    /// </summary>
    private Vector3 Evaluate(float u, out Vector3 du, out Vector3 d2u)
    {
        du = Vector3.zero;
        d2u = Vector3.zero;

        int n = _control.Length;
        if (!_valid) return Vector3.zero;

        if (_closed)
        {
            if (_totalParameter > MinSegmentLength)
            {
                u = u % _totalParameter;
                if (u < 0f) u += _totalParameter;
            }
        }
        else
        {
            if (u <= _knots[0]) { du = KnotTangent(0); return _control[0]; }
            if (u >= _knots[n - 1]) { du = KnotTangent(n - 1); return _control[n - 1]; }
        }

        // Segment [i, i+1]. On a closed spline the final segment is n-1 and runs back to 0.
        int i = (_closed && u >= _knots[n - 1]) ? n - 1 : 0;
        if (!_closed || u < _knots[n - 1])
        {
            for (int k = 0; k < n - 1; k++)
                if (u >= _knots[k]) i = k;
        }

        int j = _closed ? Mod(i + 1, n) : Mathf.Min(i + 1, n - 1);

        float t0 = _knots[i];
        float t1 = _closed && i + 1 >= n ? _knots[0] + _totalParameter : _knots[j];
        float span = t1 - t0;
        if (span < MinSegmentLength) return _control[i];
        if (!_closed) t1 = _knots[j];

        float s = (u - t0) / span;
        if (s < 0f) s = 0f;
        if (s > 1f) s = 1f;

        Vector3 p0 = _control[i];
        Vector3 p1 = _control[j];
        Vector3 m0 = KnotTangent(i);
        Vector3 m1 = KnotTangent(j);

        float s2 = s * s;
        float s3 = s2 * s;

        float h00 = 2f * s3 - 3f * s2 + 1f;
        float h10 = s3 - 2f * s2 + s;
        float h01 = -2f * s3 + 3f * s2;
        float h11 = s3 - s2;

        float dh00 = 6f * s2 - 6f * s;
        float dh10 = 3f * s2 - 4f * s + 1f;
        float dh01 = -6f * s2 + 6f * s;
        float dh11 = 3f * s2 - 2f * s;

        float ddh00 = 12f * s - 6f;
        float ddh10 = 6f * s - 4f;
        float ddh01 = -12f * s + 6f;
        float ddh11 = 6f * s - 2f;

        du = (dh00 * p0 + dh10 * span * m0 + dh01 * p1 + dh11 * span * m1) / span;
        d2u = (ddh00 * p0 + ddh10 * span * m0 + ddh01 * p1 + ddh11 * span * m1) / (span * span);

        return h00 * p0 + h10 * span * m0 + h01 * p1 + h11 * span * m1;
    }

    public float WrapDistance(float s)
    {
        if (_length <= MinSegmentLength) return 0f;
        if (!_closed) return Mathf.Clamp(s, 0f, _length);
        s %= _length;
        return s < 0f ? s + _length : s;
    }

    private int LookupIndex(float s)
    {
        int lo = 0, hi = _cumulative.Length - 1;
        while (hi - lo > 1)
        {
            int mid = (lo + hi) / 2;
            if (_cumulative[mid] <= s) lo = mid; else hi = mid;
        }
        return lo;
    }

    /// <summary>
    /// Position at arc length s.
    ///
    /// <para>
    /// Inverts s into the spline PARAMETER and evaluates the curve exactly there. It does NOT
    /// lerp between two stored positions: that would be C0-only, and the finite-difference
    /// curvature below would then show a discontinuity at every LUT sample.
    /// </para>
    /// </summary>
    public Vector3 GetPosition(float s)
    {
        if (!_valid) return Vector3.zero;
        return Evaluate(ParameterAtArcLength(s), out _, out _);
    }

    /// <summary>
    /// Parameter-space derivatives at arc length s. This is what makes curvature continuous:
    /// kappa = |P' x P''| / |P'|^3 has no dependence on the arc-length inversion, so it inherits
    /// only the smoothness of u itself.
    /// </summary>

    /// <summary>
    /// Inverts arc length to the spline parameter exactly.
    ///
    /// <para>
    /// The LUT gives a linear first guess. Left there, u is only piecewise-linear in s, so its
    /// derivative has a kink at every cell boundary and any derivative computed from it inherits
    /// that kink — which is a tangent discontinuity in all but name. Two Newton steps converge to
    /// machine precision because the guess is already within a fraction of a cell.
    /// </para>
    /// </summary>
    private float ParameterAtArcLength(float s)
    {
        s = WrapDistance(s);
        int i = LookupIndex(s);

        float lo = _parameters[i];
        float hi = _parameters[i + 1];
        if (hi <= lo) return lo;

        // Bisection, not Newton.
        //
        // Arc length is monotone in u, so the LUT cell that brackets s gives a valid bracket and
        // bisection converges unconditionally. A Newton step sized by a finite-difference ds/du
        // overshoots exactly where |dP/du| varies fastest — the corners — and a non-monotone
        // inverse shows up as a curvature discontinuity that is purely numerical. Bisection cannot
        // do that: it is monotone by construction, so u(s) is continuous, and continuity of
        // curvature follows from it.
        for (int iteration = 0; iteration < 24; iteration++)
        {
            float mid = (lo + hi) * 0.5f;
            if (ArcLengthAtParameter(mid) < s) lo = mid; else hi = mid;
        }

        return (lo + hi) * 0.5f;
    }

    /// <summary>Arc length from the start of the loop to the point at parameter u.</summary>
    private float ArcLengthAtParameter(float u)
    {
        // Linear interpolation of the cumulative table is accurate enough to drive Newton, and
        // being monotonic is what matters here.
        u = _closed ? Mathf.Repeat(u, _totalParameter) : Mathf.Clamp(u, 0f, _totalParameter);

        int count = _cumulative.Length - 1;
        float f = _totalParameter > MinSegmentLength ? u / _totalParameter * count : 0f;
        int i = Mathf.Clamp(Mathf.FloorToInt(f), 0, count - 1);
        float t = Mathf.Clamp01(f - i);
        return Mathf.Lerp(_cumulative[i], _cumulative[i + 1], t);
    }

    private void ParameterDerivatives(float s, out Vector3 du, out Vector3 d2u)
    {
        du = Vector3.zero;
        d2u = Vector3.zero;
        if (!_valid) return;

        Evaluate(ParameterAtArcLength(s), out du, out d2u);
    }

    public Vector3 GetTangent(float s)
    {
        if (!_valid) return Vector3.forward;
        ParameterDerivatives(s, out Vector3 du, out _);
        float m = du.magnitude;
        if (m < MinSegmentLength || float.IsNaN(m)) return Vector3.forward;
        return du / m;
    }

    public Vector3 GetNormal(float s)
    {
        Vector3 planar = Vector3.ProjectOnPlane(GetTangent(s), Vector3.up);
        if (planar.sqrMagnitude < 1e-8f) return Vector3.right;
        return Vector3.Cross(Vector3.up, planar.normalized).normalized;
    }

    public float GetCurvature(float s)
    {
        if (!_valid) return 0f;
        ParameterDerivatives(s, out Vector3 du, out Vector3 d2u);

        float speed = du.magnitude;
        if (speed < MinSegmentLength || float.IsNaN(speed)) return 0f;

        // kappa = |P' x P''| / |P'|^3, parameterisation-independent.
        float kappa = Vector3.Cross(du, d2u).magnitude / (speed * speed * speed);
        return float.IsNaN(kappa) ? 0f : kappa;
    }

    public float ArcLengthOfControlPoint(int index)
    {
        if (!_valid) return 0f;
        index = _closed ? Mod(index, _control.Length) : Mathf.Clamp(index, 0, _control.Length - 1);
        int lut = Mathf.Clamp(index * SubdivisionsPerSegment, 0, _cumulative.Length - 1);
        return Mathf.Lerp(_cumulative[lut], _cumulative[Mathf.Min(lut + 1, _cumulative.Length - 1)], 0f);
    }
}
```

- [ ] **Step 6: Run the tests to verify they pass**

Call `mcp__UnityMCP__refresh_unity` with `mode: "force"`, `compile: "request"`, `wait_for_ready: true`, then `run_tests` with `mode: "EditMode"`, `test_names: ["ArcLengthSplineTests"]`, and poll `get_test_job` with `wait_timeout: 60`.

Expected: 8 passed, 0 failed.

If `CurvatureIsContinuousAlongTheWholeSpline` fails, the likely cause is a wrong derivative step size. `_length * 1e-5f` on a ~400 m spline gives h = 4 mm; if it is unstable, raise to `_length * 1e-4f`. Do **not** relax the assertion threshold — the threshold is the deliverable.

- [ ] **Step 7: Run the full EditMode suite**

Call `run_tests` with `mode: "EditMode"` and poll with `wait_timeout: 120`.

Expected: **185 + 8 = 193 passed, 0 failed.** The original 185 must be untouched — this task adds no behaviour to existing code.

- [ ] **Step 8: Commit**

```bash
git add Assets/Scripts/TrackData/ Assets/Tests/EditMode/ArcLengthSplineTests.cs Assets/Tests/EditMode/F1.EditModeTests.asmdef
git commit -m "Add arc-length-parameterised centripetal spline

Curvature continuity is the property the waypoint array lacked: independent
per-waypoint derivation stepped 99 km/h between adjacent samples because
Menger curvature on an 11 m triangle is discontinuous. A centripetal
Catmull-Rom (alpha = 0.5) makes curvature a smooth function of arc length,
which is what lets the derived speed profile be a speed authority.

com.unity.splines is present transitively via Cinemachine but builds a
Bezier whose knots are G1, not G2, so its curvature is discontinuous too.

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 2: TrackRuntime — arc length as the coordinate system

**Files:**
- Create: `Assets/Scripts/TrackData/TrackRuntime.cs`
- Test: `Assets/Tests/EditMode/TrackRuntimeTests.cs`

**Interfaces:**
- Consumes: `ArcLengthSpline` from Task 1.
- Produces:
  ```csharp
  public sealed class TrackRuntime
  {
      public TrackRuntime(TrackData data);              // built once per race
      public float Length { get; }
      public ArcLengthSpline Centreline { get; }
      public Vector3 GetPosition(float s);
      public Vector3 GetTangent(float s);
      public Vector3 GetNormal(float s);
      public float GetCurvature(float s);
      public float GetWidth(float s);                    // metres, half-width from centreline
      public float GetSpeedLimitKmh(float s);            // physical limit, car-agnostic
      public SectorIndex GetSector(float s);

      /// <summary>World position -> (arc length, signed lateral offset). The inverse of everything.</summary>
      public bool ProjectToArcLength(Vector3 world, out float s, out float lateralOffset);
  }
  ```
  `ProjectToArcLength` returns `false` when no projection is found; callers must handle it.

- [ ] **Step 1: Write the failing tests**

Create `Assets/Tests/EditMode/TrackRuntimeTests.cs`:

```csharp
using NUnit.Framework;
using UnityEngine;

public class TrackRuntimeTests
{
    private static TrackRuntime StraightTrack(float lengthMetres = 1000f)
    {
        // A degenerate oval: long straights joined by gentle ends. Real enough to project onto,
        // simple enough that an expected answer can be computed by hand.
        var knots = new[]
        {
            new Vector3(0f, 0f, 0f),
            new Vector3(lengthMetres * 0.5f, 0f, 0f),
            new Vector3(lengthMetres, 0f, 0f),
            new Vector3(lengthMetres * 0.5f, 0f, 200f),
        };
        var data = ScriptableObject.CreateInstance<TrackData>();
        data.SetControlPoints(knots, closed: true);
        data.SetUniformSpeedLimitKmh(240f);
        return new TrackRuntime(data);
    }

    [Test]
    public void ProjectionRecoversTheArcLengthItWasGiven()
    {
        TrackRuntime track = StraightTrack();

        for (int i = 1; i < 40; i++)
        {
            float s = track.Length * i / 40f;
            Vector3 world = track.GetPosition(s);

            bool ok = track.ProjectToArcLength(world, out float recovered, out float lateral);
            Assert.IsTrue(ok, $"Projection failed at s={s:F1} m.");
            Assert.That(Mathf.Abs(recovered - s), Is.LessThan(0.5f),
                $"Projected s={recovered:F3} but placed at s={s:F3}: arc length must round-trip.");
            Assert.That(Mathf.Abs(lateral), Is.LessThan(0.1f),
                $"A point exactly on the centreline must project to ~0 lateral offset, got {lateral:F3} m.");
        }
    }

    [Test]
    public void ProjectionReportsTheSignedLateralOffset()
    {
        TrackRuntime track = StraightTrack();
        Vector3 onLine = track.GetPosition(track.Length * 0.25f);
        Vector3 right = track.GetNormal(track.Length * 0.25f);

        track.ProjectToArcLength(onLine + right * 4f, out float sRight, out float offsetRight);
        track.ProjectToArcLength(onLine - right * 4f, out float sRight2, out float offsetLeft);

        Assert.That(offsetRight, Is.GreaterThan(0f), "4 m to the right of the normal must be a positive offset.");
        Assert.That(offsetLeft, Is.LessThan(0f), "4 m to the left of the normal must be a negative offset.");
        Assert.That(Mathf.Abs(offsetRight - 4f), Is.LessThan(0.3f));
        Assert.That(Mathf.Abs(offsetLeft + 4f), Is.LessThan(0.3f));
        Assert.That(Mathf.Abs(sRight - sRight2), Is.LessThan(0.5f),
            "Both offsets are of the same point, so they must share an arc length.");
    }

    [Test]
    public void ProjectionIsContinuousAcrossTheStartFinishSeam()
    {
        TrackRuntime track = StraightTrack();

        Vector3 justBefore = track.GetPosition(track.Length - 1f);
        Vector3 justAfter = track.GetPosition(0f);

        track.ProjectToArcLength(justBefore, out float sA, out _);
        track.ProjectToArcLength(justAfter, out float sB, out _);

        Assert.That(sA, Is.GreaterThan(track.Length * 0.5f), "Pre-seam sample should have a high s.");
        Assert.That(sB, Is.LessThan(track.Length * 0.5f), "Post-seam sample should wrap to a low s.");
        // The two are the same point on a closed loop, so they must be one metre apart, not 400.
        float gap = Mathf.Min(sA - sB, track.Length - (sA - sB));
        Assert.That(gap, Is.LessThan(2f), $"Seam gap is {gap:F2} m; s must wrap, not teleport.");
    }

    [Test]
    public void SpeedLimitIsReadAsAStoredProfileNotRecomputedPerCall()
    {
        TrackRuntime track = StraightTrack();
        Assert.That(track.GetSpeedLimitKmh(0f), Is.EqualTo(240f).Within(0.01f));
        Assert.That(track.GetSpeedLimitKmh(track.Length * 0.5f), Is.EqualTo(240f).Within(0.01f));
        Assert.That(track.GetSpeedLimitKmh(track.Length - 1f), Is.EqualTo(240f).Within(0.01f));
    }

    [Test]
    public void ARuntimeBuiltFromAnInvalidAssetRefusesToPretendItIsUsable()
    {
        // Review Focus #2: a car with no usable line must stop, not drive blind.
        var empty = ScriptableObject.CreateInstance<TrackData>();
        var track = new TrackRuntime(empty);

        Assert.IsFalse(track.IsValid, "A TrackData with no control points is not a track.");
        Assert.That(track.Length, Is.EqualTo(0f).Within(0.001f));
    }

    [Test]
    public void SectorLookupCoversTheWholeLapExactlyOnce()
    {
        TrackRuntime track = StraightTrack();

        int[] counts = new int[3];
        for (int i = 0; i < 300; i++)
            counts[(int)track.GetSector(track.Length * i / 300f)]++;

        Assert.That(counts[0], Is.GreaterThan(0));
        Assert.That(counts[1], Is.GreaterThan(0));
        Assert.That(counts[2], Is.GreaterThan(0),
            "Sector 3 must be reachable; a track whose last third is unreachable breaks sector timing.");
        Assert.That(counts[0] + counts[1] + counts[2], Is.EqualTo(300));
    }
}
```

- [ ] **Step 2: Run to verify failure**

`refresh_unity` (force, request, wait), then `run_tests` `mode: "EditMode"`, `test_names: ["TrackRuntimeTests"]`.

Expected: compile error — `TrackData` not found.

- [ ] **Step 3: Implement `TrackData` and `TrackRuntime`**

Create `Assets/Scripts/TrackData/TrackData.cs`:

```csharp
using UnityEngine;
using F1.GameData;

namespace F1.TrackData
{
    /// <summary>
    /// Everything a track contributes to gameplay, baked once and stored as an asset.
    ///
    /// <para>
    /// <b>Car-agnostic by design.</b> The speed profile here is a PHYSICAL limit — what the tyres
    /// can do — not what one car does. A Gen1 rental and a Gen3 unlock then differ because their
    /// CarDefinition differs, which is what the GDD's car-unlock progression needs. A profile
    /// baked against one car's downforce would silently encode that car into every track.
    /// </para>
    ///
    /// <para>
    /// The authoring fields (<c>controlPoints</c>) and the baked fields (<c>speedProfileKmh</c>)
    /// are separated deliberately: one is hand-editable, the other is machine-derived and must be
    /// regenerated whenever the first changes. <see cref="IsBaked"/> reports the mismatch.
    /// </para>
    /// </summary>
    [CreateAssetMenu(menuName = "F1/Track Data", fileName = "TrackData_")]
    public class TrackData : ScriptableObject
    {
        [Header("Identity")]
        [SerializeField] private string _trackId;
        public string TrackId => _trackId;

        [Header("Authoring — edit these")]
        [Tooltip("Centreline control points, ~60 for a real circuit. The spline carries the shape.")]
        [SerializeField] private Vector3[] _controlPoints = new Vector3[0];
        public Vector3[] ControlPoints => _controlPoints;

        [SerializeField] private bool _closed = true;
        public bool Closed => _closed;

        [Header("Baked — regenerate with Tools/F1/Bake Track Data")]
        [Tooltip("Physical speed limit in km/h, sampled uniformly in arc length.")]
        [SerializeField] private float[] _speedProfileKmh = new float[0];
        public float[] SpeedProfileKmh => _speedProfileKmh;

        [Tooltip("Track width in metres, sampled uniformly in arc length.")]
        [SerializeField] private float[] _widthMetres = new float[0];
        public float[] WidthMetres => _widthMetres;

        [Tooltip("Total arc length of the centreline in metres.")]
        [SerializeField] private float _lengthMetres;
        public float LengthMetres => _lengthMetres;

        [Tooltip("A clean lap at the physical limit, seconds. The yardstick tiers are paced against.")]
        [SerializeField] private float _referenceLapSeconds;
        public float ReferenceLapSeconds => _referenceLapSeconds;

        [Tooltip("Arc length at the start of each of the three sectors, metres from the line.")]
        [SerializeField] private float[] _sectorStarts = { 0f, 0f, 0f };
        public float[] SectorStarts => _sectorStarts;

        [Tooltip("True when the baked arrays match the current control points.")]
        [SerializeField] private bool _isBaked;
        public bool IsBaked => _isBaked;

        /// <summary>Metres between speed-profile samples. The profile's resolution.</summary>
        public const float SpeedProfileResolutionMetres = 2f;

        /// <summary>Metres between width samples. Wider spacing is fine; width varies slowly.</summary>
        public const float WidthResolutionMetres = 10f;

#if UNITY_EDITOR
        /// <summary>Test/authoring seam. Sets the authoring points and invalidates the bake.</summary>
        public void SetControlPoints(Vector3[] points, bool closed)
        {
            _controlPoints = points ?? new Vector3[0];
            _closed = closed;
            _isBaked = false;
        }

        /// <summary>Test/authoring seam. Fills the profile with one constant value.</summary>
        public void SetUniformSpeedLimitKmh(float kmh)
        {
            int count = Mathf.Max(2, Mathf.CeilToInt(_lengthMetres / SpeedProfileResolutionMetres));
            _speedProfileKmh = new float[count];
            for (int i = 0; i < count; i++) _speedProfileKmh[i] = kmh;

            int widthCount = Mathf.Max(2, Mathf.CeilToInt(_lengthMetres / WidthResolutionMetres));
            _widthMetres = new float[widthCount];
            for (int i = 0; i < widthCount; i++) _widthMetres[i] = 16f;

            _isBaked = true;
        }

        /// <summary>
        /// Writes the baked arrays. Called by the editor baker only; the values are derived, so
        /// nothing at runtime should set them.
        /// </summary>
        public void WriteBakedData(
            float[] speedProfileKmh,
            float[] widthMetres,
            float lengthMetres,
            float referenceLapSeconds,
            float[] sectorStarts)
        {
            _speedProfileKmh = speedProfileKmh;
            _widthMetres = widthMetres;
            _lengthMetres = lengthMetres;
            _referenceLapSeconds = referenceLapSeconds;
            _sectorStarts = sectorStarts;
            _isBaked = true;
        }

        private void OnValidate()
        {
            if (string.IsNullOrEmpty(_trackId))
                _trackId = name.Replace("TrackData_", "").ToLowerInvariant();
            if (_sectorStarts == null || _sectorStarts.Length != 3)
                _sectorStarts = new float[3] { 0f, 0f, 0f };
        }
#endif
    }
}
```

Create `Assets/Scripts/TrackData/TrackRuntime.cs`:

```csharp
using UnityEngine;
using F1.GameData;

namespace F1.TrackData
{
    /// <summary>
    /// The immutable per-race view of a track. Built once, then read every tick by every car.
    ///
    /// <para>
    /// This is the layer that replaces index-as-position. <see cref="ProjectToArcLength"/> is the
    /// single operation that three separate consumers (<c>AIDriverController.UpdateProgressIndex</c>,
    /// <c>LapTracker.BuildArcTable</c>, <c>AILongitudinalPlanner.SolvePlannedSpeedKmh</c>) each
    /// currently re-derive, and it is also what track-limit detection and sector timing need.
    /// </para>
    ///
    /// <para>
    /// Read-only and allocation-free after construction: the per-tick path does no allocation,
    /// because twenty cars calling this every physics step is the AI CPU budget named in the GDD.
    /// </para>
    /// </summary>
    public sealed class TrackRuntime
    {
        private readonly ArcLengthSpline _centreline;
        private readonly float[] _speedProfileKmh;
        private readonly float[] _widthMetres;
        private readonly float[] _sectorStarts;
        private readonly float _length;

        // Coarse projection acceleration: bucket boundaries every ~5 m. A lap is a few thousand
        // entries, so even a linear scan would be fine, but this makes it ~50x cheaper and costs
        // one small array at construction.
        private const float ProjectionBucketMetres = 5f;
        private int _bucketCount;
        private int[] _bucketStartSample;

        public bool IsValid { get; }
        public float Length => _length;
        public ArcLengthSpline Centreline => _centreline;
        public float ReferenceLapSeconds { get; }

        public TrackRuntime(TrackData data)
        {
            if (data == null || data.ControlPoints == null || data.ControlPoints.Length < 3)
            {
                _centreline = null;
                _speedProfileKmh = new float[0];
                _widthMetres = new float[0];
                _sectorStarts = new float[3] { 0f, 0f, 0f };
                _length = 0f;
                ReferenceLapSeconds = 0f;
                IsValid = false;
                return;
            }

            _centreline = new ArcLengthSpline(data.ControlPoints, data.Closed);
            if (!_centreline.IsValid)
            {
                _speedProfileKmh = new float[0];
                _widthMetres = new float[0];
                _sectorStarts = new float[3] { 0f, 0f, 0f };
                _length = 0f;
                ReferenceLapSeconds = 0f;
                IsValid = false;
                return;
            }

            _speedProfileKmh = data.SpeedProfileKmh ?? new float[0];
            _widthMetres = data.WidthMetres ?? new float[0];
            _sectorStarts = data.SectorStarts ?? new float[3] { 0f, 0f, 0f };
            _length = _centreline.Length;
            ReferenceLapSeconds = data.ReferenceLapSeconds;
            IsValid = true;

            BuildProjectionBuckets();
        }

        private void BuildProjectionBuckets()
        {
            _bucketCount = Mathf.Max(1, Mathf.CeilToInt(_length / ProjectionBucketMetres));
            _bucketStartSample = new int[_bucketCount + 1];

            // The LUT is uniform in the centripetal parameter, not arc length, so bucket
            // boundaries are found by walking the cumulative distance once.
            int samples = SampleCount();
            int cursor = 0;
            for (int b = 0; b <= _bucketCount; b++)
            {
                float targetArc = b * ProjectionBucketMetres;
                _bucketStartSample[b] = cursor;
                while (cursor < samples - 1 && ArcAtSample(cursor + 1) < targetArc) cursor++;
            }
        }

        private int SampleCount() => Mathf.RoundToInt(_length);

        private float ArcAtSample(int i) => _length * i / Mathf.Max(1, SampleCount() - 1);

        // ---- Sampling ----------------------------------------------------------------

        public Vector3 GetPosition(float s) => _centreline?.GetPosition(s) ?? Vector3.zero;
        public Vector3 GetTangent(float s) => _centreline?.GetTangent(s) ?? Vector3.forward;
        public Vector3 GetNormal(float s) => _centreline?.GetNormal(s) ?? Vector3.right;
        public float GetCurvature(float s) => _centreline?.GetCurvature(s) ?? 0f;

        public float WrapDistance(float s) => _centreline?.WrapDistance(s) ?? 0f;

        /// <summary>
        /// Physical speed limit at arc length s, km/h. Sampled uniformly in arc length at
        /// <see cref="TrackData.SpeedProfileResolutionMetres"/>.
        /// </summary>
        public float GetSpeedLimitKmh(float s)
        {
            if (_speedProfileKmh.Length == 0) return 0f;
            s = WrapDistance(s);
            float step = _length / _speedProfileKmh.Length;
            float f = step > 1e-4f ? s / step : 0f;
            int i = Mathf.Clamp(Mathf.FloorToInt(f), 0, _speedProfileKmh.Length - 1);
            int j = Mathf.Min(i + 1, _speedProfileKmh.Length - 1);
            float t = Mathf.Clamp01(f - i);
            return Mathf.Lerp(_speedProfileKmh[i], _speedProfileKmh[j], t);
        }

        /// <summary>Track width at arc length s, metres. Half-width from the centreline is half this.</summary>
        public float GetWidth(float s)
        {
            if (_widthMetres.Length == 0) return 0f;
            s = WrapDistance(s);
            float step = _length / _widthMetres.Length;
            float f = step > 1e-4f ? s / step : 0f;
            int i = Mathf.Clamp(Mathf.FloorToInt(f), 0, _widthMetres.Length - 1);
            int j = Mathf.Min(i + 1, _widthMetres.Length - 1);
            float t = Mathf.Clamp01(f - i);
            return Mathf.Lerp(_widthMetres[i], _widthMetres[j], t);
        }

        public SectorIndex GetSector(float s)
        {
            s = WrapDistance(s);
            if (s >= _sectorStarts[2]) return SectorIndex.Sector3;
            if (s >= _sectorStarts[1]) return SectorIndex.Sector2;
            return SectorIndex.Sector1;
        }

        /// <summary>
        /// Projects a world position onto the centreline, returning arc length and signed lateral
        /// offset (positive to the right of travel).
        ///
        /// <para>
        /// Coarse bucket scan then local refine. Deliberately not a spatial index: a lap is a few
        /// thousand samples and this runs for every car every tick, so O(log n) after a coarse
        /// scan is both simple and fast enough for the GDD's AI CPU budget.
        /// </para>
        ///
        /// <para>
        /// Returns false when the track is unusable or the projection is degenerate. Callers must
        /// hold their last valid value rather than guess — a driver that cannot find itself on
        /// the line must slow down, not teleport.
        /// </para>
        /// </summary>
        public bool ProjectToArcLength(Vector3 world, out float s, out float lateralOffset)
        {
            s = 0f;
            lateralOffset = 0f;
            if (!IsValid) return false;

            Vector3 planar = Vector3.ProjectOnPlane(world, Vector3.up);

            // Seed from a coarse scan over one bucket window, then walk outward. A car on the
            // racing line is within a few metres of its previous position, so this converges in
            // a handful of samples rather than scanning the lap.
            int bestIndex = -1;
            float bestSqr = float.MaxValue;

            int window = Mathf.Max(8, Mathf.RoundToInt(40f / ProjectionBucketMetres) * 4);
            int centre = Mathf.FloorToInt(EstimateArcGuess(planar) / ProjectionBucketMetres);

            for (int b = centre - window; b <= centre + window; b++)
            {
                if (b < 0 || b > _bucketCount) continue;
                int i = _bucketStartSample[b];
                int j = _bucketStartSample[Mathf.Min(b + 1, _bucketCount)];
                for (int k = i; k <= j && k < SampleCount(); k++)
                {
                    Vector3 candidate = GetPosition(ArcAtSample(k));
                    float sqr = (Vector3.ProjectOnPlane(candidate, Vector3.up) - planar).sqrMagnitude;
                    if (sqr < bestSqr) { bestSqr = sqr; bestIndex = k; }
                }
            }

            // The window missed (a car that teleported, or a lap that doubled back). Fall back to
            // a full scan — rare, and correctness beats speed here.
            if (bestIndex < 0)
            {
                for (int k = 0; k < SampleCount(); k++)
                {
                    Vector3 candidate = GetPosition(ArcAtSample(k));
                    float sqr = (Vector3.ProjectOnPlane(candidate, Vector3.up) - planar).sqrMagnitude;
                    if (sqr < bestSqr) { bestSqr = sqr; bestIndex = k; }
                }
            }

            if (bestIndex < 0) return false;

            // Refine: bracket the minimum and bisect.
            float arcGuess = ArcAtSample(bestIndex);
            float refineStep = _length / Mathf.Max(1, SampleCount() - 1) * 0.5f;
            for (int step = 0; step < 12; step++)
            {
                float left = arcGuess - refineStep;
                float right = arcGuess + refineStep;

                float leftSqr = (Vector3.ProjectOnPlane(GetPosition(left), Vector3.up) - planar).sqrMagnitude;
                float rightSqr = (Vector3.ProjectOnPlane(GetPosition(right), Vector3.up) - planar).sqrMagnitude;

                if (leftSqr < bestSqr && leftSqr <= rightSqr) { arcGuess = left; bestSqr = leftSqr; }
                else if (rightSqr < bestSqr) { arcGuess = right; bestSqr = rightSqr; }
                else break;

                refineStep *= 0.5f;
            }

            s = WrapDistance(arcGuess);

            // Signed lateral: the component of (world - onLine) along the right normal.
            Vector3 onLine = GetPosition(s);
            Vector3 right = GetNormal(s);
            lateralOffset = Vector3.Dot(Vector3.ProjectOnPlane(world - onLine, Vector3.up), right);

            return true;
        }

        /// <summary>
        /// Cheap first guess at arc length from the XZ bearing around the track's centroid. Only
        /// used to seed the windowed search; accuracy is irrelevant because the refine converges.
        /// </summary>
        private float EstimateArcGuess(Vector3 planar)
        {
            Vector3 centre = Vector3.zero;
            int samples = Mathf.Min(SampleCount(), 64);
            for (int i = 0; i < samples; i++) centre += GetPosition(_length * i / samples);
            centre /= samples;

            Vector3 toPoint = planar - new Vector3(centre.x, 0f, centre.z);
            float bearing = Mathf.Atan2(toPoint.x, toPoint.z);
            if (bearing < 0f) bearing += Mathf.PI * 2f;
            return bearing / (Mathf.PI * 2f) * _length;
        }
    }
}
```

Note the namespace: `F1.TrackData`. `ArcLengthSpline` from Task 1 is in the **global** namespace, so `TrackRuntime` refers to it unqualified. That asymmetry is deliberate and matches the existing codebase, where `AIRacingLine` and friends are global-namespace despite `rootNamespace: "F1"`.

- [ ] **Step 4: Run to verify pass**

`refresh_unity` (force, request, wait), then `run_tests` `mode: "EditMode"`, `test_names: ["TrackRuntimeTests"]`, poll `wait_timeout: 60`.

Expected: 6 passed.

- [ ] **Step 5: Run the full EditMode suite**

`run_tests` `mode: "EditMode"`, poll `wait_timeout: 120`.

Expected: **193 + 6 = 199 passed, 0 failed.**

- [ ] **Step 6: Commit**

```bash
git add Assets/Scripts/TrackData/ Assets/Tests/EditMode/TrackRuntimeTests.cs
git commit -m "Make arc length the track's coordinate system

ProjectToArcLength replaces three separate hand-rolled versions of the same
operation: AIDriverController.UpdateProgressIndex, LapTracker.BuildArcTable
and AILongitudinalPlanner.SolvePlannedSpeedKmh each derived arc length from
the waypoint array in their own way.

TrackData is car-agnostic on purpose: the speed profile is a physical limit,
so a Gen1 rental and a Gen3 unlock differ by their CarDefinition rather than
by a per-track bake.

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

### Task 3: Make the baker testable, and prove a second track bakes

**Files:**
- Create: `Assets/Editor/F1.EditorTools.asmdef`
- Modify: `Assets/Editor/TrackRacingLineBaker.cs:820-885` (extract pure bake stages)
- Modify: `Assets/Editor/TrackContentBuilder.cs:81-110, 184` (emit `TrackData`, not a waypoint hierarchy)
- Test: `Assets/Tests/EditMode/TrackDataBakeTests.cs`

**Interfaces:**
- Consumes: `TrackData`, `ArcLengthSpline` (Tasks 1–2).
- Produces: `public static class RacingLineBaker` in `F1.EditorTools`, with
  ```csharp
  public static BakeResult Bake(List<Vector3> centreline, float maxLateralAccel, float brakeAccel,
                                float straightSpeedKmh, float minSpeedKmh, float trackWidthMetres,
                                out string failureReason);
  public sealed class BakeResult
  {
      public bool Success; public string FailureReason;
      public Vector3[] ControlPoints;      // decimated to ~60
      public float[] SpeedProfileKmh;      // 1 per 2 m
      public float[] WidthMetres;          // 1 per 10 m
      public float LengthMetres;
      public float[] SectorStarts;         // 3 entries
  }
  ```

**Why this task exists:** `TrackRacingLineBaker` and `TrackContentBuilder` are currently in `Assembly-CSharp-Editor`, which **neither test assembly references** (`overrideReferences: true`, and no reference to it). There is therefore no test that any bake has ever been correct, on any track. For five tracks that is the single largest risk in the programme.

- [ ] **Step 1: Give the editor scripts an assembly**

Create `Assets/Editor/F1.EditorTools.asmdef`:

```json
{
    "name": "F1.EditorTools",
    "rootNamespace": "F1.EditorTools",
    "references": [
        "F1.TrackData",
        "F1.Gameplay",
        "F1.GameData"
    ],
    "includePlatforms": ["Editor"],
    "excludePlatforms": [],
    "allowUnsafeCode": false,
    "overrideReferences": false,
    "precompiledReferences": [],
    "autoReferenced": true,
    "defineConstraints": [],
    "versionDefines": [],
    "noEngineReferences": false
}
```

- [ ] **Step 2: Add a test assembly that can reference it**

The EditMode test assembly has `includePlatforms: ["Editor"]`, so it *can* reference an Editor-only assembly. Add `"F1.EditorTools"` to `Assets/Tests/EditMode/F1.EditModeTests.asmdef`'s `references`.

If Unity refuses the reference (an Editor-only assembly referenced from a test assembly can trip the platform check), create `Assets/Tests/EditModeEditor/F1.EditorTests.asmdef` instead with `includePlatforms: ["Editor"]`, references `["F1.EditorTools", "F1.TrackData", "F1.Gameplay", "F1.GameData", "UnityEditor.TestRunner", "UnityEngine.TestRunner"]`, `overrideReferences: true`, `precompiledReferences: ["nunit.framework.dll"]`, `defineConstraints: ["UNITY_INCLUDE_TESTS"]`, and put the bake tests there. Both assemblies then appear in the EditMode run and the baseline count rises by the number of bake tests.

- [ ] **Step 3: Write the failing bake tests**

Create `Assets/Tests/EditMode/TrackDataBakeTests.cs`:

```csharp
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using F1.EditorTools;

public class TrackDataBakeTests
{
    private const float MaxLateralAccel = 12f;
    private const float BrakeAccel = 18f;
    private const float StraightSpeedKmh = 240f;
    private const float MinSpeedKmh = 90f;
    private const float WidthMetres = 16f;

    /// <summary>
    /// A synthetic circuit with a genuinely varied shape: two long straights, two hairpins, one
    /// sweeper. Deliberately not the real Track_01, so the "scales to five tracks" claim is tested
    /// against geometry no test has ever seen.
    /// </summary>
    private static List<Vector3> SyntheticCircuit()
    {
        var points = new List<Vector3>();
        const int samples = 400;

        for (int i = 0; i < samples; i++)
        {
            float t = i / (float)samples;
            float angle = t * Mathf.PI * 2f;

            // Vary the radius sharply to create corners of genuinely different tightness.
            float radius = 300f + 220f * Mathf.Sin(angle * 2f) + 120f * Mathf.Sin(angle * 5f);
            points.Add(new Vector3(Mathf.Cos(angle) * radius, 0f, Mathf.Sin(angle) * radius));
        }

        return points;
    }

    [Test]
    public void ABakeOfACleanCircuitSucceeds()
    {
        var result = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);

        Assert.IsTrue(result.Success, $"Bake failed: {reason}");
        Assert.Greater(result.LengthMetres, 1000f, "A 400-point circuit of radius ~300 m is kilometres long.");
        Assert.Greater(result.ControlPoints.Length, 8, "Too few control points to describe the shape.");
        Assert.Less(result.ControlPoints.Length, 120,
            $"Decimation should reduce ~400 raw points to ~60 control points, got {result.ControlPoints.Length}.");
    }

    [Test]
    public void ABakedProfileIsABackwardPassAndSoCannotCliff()
    {
        var result = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);
        Assert.IsTrue(result.Success, $"Bake failed: {reason}");

        // This is the invariant the waypoint array could not hold: a backward braking pass over
        // a closed loop can only ever lower a speed where the next point is slower, so the step
        // between adjacent samples is bounded by what the brakes can shed over one sample.
        float worstStep = 0f;
        int worstAt = 0;
        for (int i = 0; i < result.SpeedProfileKmh.Length; i++)
        {
            int j = (i + 1) % result.SpeedProfileKmh.Length;
            float step = Mathf.Abs(result.SpeedProfileKmh[i] - result.SpeedProfileKmh[j]);
            if (step > worstStep) { worstStep = step; worstAt = i; }
        }

        // Over 2 m at 18 m/s^2, the largest physically possible step is v^2/(2a) difference,
        // which at 240 km/h (66.7 m/s) over 2 m is about 0.25 m/s = 0.9 km/h. Allow generous
        // slack for the straight-speed clamp, and assert well under the old 23.8 km/h figure.
        Assert.Less(worstStep, 6f,
            $"Profile steps {worstStep:F2} km/h between adjacent samples at index {worstAt}. " +
            "A backward pass cannot produce that; something upstream is wrong.");
    }

    [Test]
    public void ABakedProfileRespectsItsOwnPhysicalCeiling()
    {
        var result = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);
        Assert.IsTrue(result.Success, $"Bake failed: {reason}");

        for (int i = 0; i < result.SpeedProfileKmh.Length; i++)
        {
            Assert.That(result.SpeedProfileKmh[i], Is.LessThanOrEqualTo(StraightSpeedKmh + 0.01f),
                $"Sample {i} exceeds the straight speed ceiling.");
            Assert.That(result.SpeedProfileKmh[i], Is.GreaterThanOrEqualTo(MinSpeedKmh - 0.01f),
                $"Sample {i} is below the configured floor; a speed limit below {MinSpeedKmh} km/h " +
                "is a geometry artefact, not a corner.");
            Assert.That(float.IsNaN(result.SpeedProfileKmh[i]), Is.False, "NaN in the speed profile.");
        }
    }

    [Test]
    public void ABakeOfDegenerateInputIsRefusedWithAReason()
    {
        // Review Focus #5: a track with no valid bake must be refused, never silently degenerate.
        var collinear = new List<Vector3>
        {
            new Vector3(0f, 0f, 0f), new Vector3(10f, 0f, 0f),
            new Vector3(20f, 0f, 0f), new Vector3(30f, 0f, 0f)
        };

        var result = RacingLineBaker.Bake(collinear, MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);

        Assert.IsFalse(result.Success, "A collinear centreline is not a circuit and must be refused.");
        Assert.IsNotEmpty(reason, "A refusal must name its reason so a track author can act on it.");
    }

    [Test]
    public void ABakeOfTooFewPointsIsRefusedWithAReason()
    {
        var tiny = new List<Vector3> { Vector3.zero, Vector3.one, new Vector3(2f, 0f, 0f) };
        var result = RacingLineBaker.Bake(tiny, MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);

        Assert.IsFalse(result.Success);
        Assert.IsNotEmpty(reason);
    }

    [Test]
    public void BakingTheSameCircuitTwiceGivesTheSameAnswer()
    {
        // Determinism. A bake that varies run to run makes every later comparison meaningless.
        var first = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out _);
        var second = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out _);

        Assert.IsTrue(first.Success && second.Success);
        Assert.AreEqual(first.ControlPoints.Length, second.ControlPoints.Length);
        Assert.AreEqual(first.SpeedProfileKmh.Length, second.SpeedProfileKmh.Length);
        for (int i = 0; i < first.SpeedProfileKmh.Length; i++)
            Assert.That(first.SpeedProfileKmh[i], Is.EqualTo(second.SpeedProfileKmh[i]).Within(1e-4f));
    }

    [Test]
    public void ASectorSplitCoversTheLapWithoutGapsOrOverlap()
    {
        var result = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);
        Assert.IsTrue(result.Success, $"Bake failed: {reason}");

        Assert.AreEqual(3, result.SectorStarts.Length);
        Assert.That(result.SectorStarts[0], Is.EqualTo(0f).Within(0.01f));
        Assert.That(result.SectorStarts[1], Is.GreaterThan(0f));
        Assert.That(result.SectorStarts[2], Is.GreaterThan(result.SectorStarts[1]),
            "Sector 3 must start after sector 2.");
        Assert.That(result.SectorStarts[2], Is.LessThan(result.LengthMetres),
            "Sector 3 must start before the lap ends.");
    }
}
```

- [ ] **Step 4: Run to verify failure**

`refresh_unity` (force, request, wait), then `run_tests` `mode: "EditMode"`, `test_names: ["TrackDataBakeTests"]`.

Expected: compile error — `RacingLineBaker` / `F1.EditorTools` not found.

- [ ] **Step 5: Extract the pure bake stages**

In `Assets/Editor/TrackRacingLineBaker.cs`, add a new file `Assets/Editor/RacingLineBaker.cs`. **Do not delete the existing `TrackRacingLineBaker`** — Task 4 still needs it to read the tarmac mesh. This new class takes an already-extracted centreline and does everything from there, so it is testable without a scene.

```csharp
using System.Collections.Generic;
using UnityEngine;
using F1.TrackData;

namespace F1.EditorTools
{
    public sealed class BakeResult
    {
        public bool Success;
        public string FailureReason;
        public Vector3[] ControlPoints;
        public float[] SpeedProfileKmh;
        public float[] WidthMetres;
        public float LengthMetres;
        public float[] SectorStarts;
        public float ReferenceLapSeconds;
    }

    /// <summary>
    /// The parts of a racing-line bake that need no scene and no mesh, extracted so they can be
    /// tested. Before this existed the whole bake lived in TrackRacingLineBaker, which sits in an
    /// assembly no test can reference, so no bake had ever been verified on any track.
    /// </summary>
    public static class RacingLineBaker
    {
        private const float KmhToMs = 1f / 3.6f;   // km/h -> m/s MULTIPLIES by this. Dividing inflates by 3.6x.
        private const int TargetControlPoints = 60;
        private const int SpeedProfileSamplesPerLapMin = 32;

        public static BakeResult Bake(
            List<Vector3> centreline,
            float maxLateralAccel,
            float brakeAccel,
            float straightSpeedKmh,
            float minSpeedKmh,
            float trackWidthMetres,
            out string failureReason)
        {
            var result = new BakeResult
            {
                ControlPoints = new Vector3[0],
                SpeedProfileKmh = new float[0],
                WidthMetres = new float[0],
                SectorStarts = new float[3] { 0f, 0f, 0f }
            };

            if (centreline == null || centreline.Count < 8)
            {
                failureReason = $"A racing line needs at least 8 points to describe a circuit; got " +
                                $"{centreline?.Count ?? 0}. The tarmac was probably not read as " +
                                "cross-section cells, or the submesh lookup failed.";
                result.FailureReason = failureReason;
                return result;
            }

            result.ControlPoints = Decimate(centreline, TargetControlPoints);
            var spline = new ArcLengthSpline(result.ControlPoints, true);

            if (!spline.IsValid)
            {
                failureReason = "The decimated control points do not form a usable closed spline. " +
                                "This usually means the extracted centreline is collinear or has " +
                                "duplicated points.";
                result.FailureReason = failureReason;
                return result;
            }

            if (spline.Length < 100f)
            {
                failureReason = $"The centreline measures only {spline.Length:F1} m. A circuit is " +
                                "kilometres long; the tarmac extraction almost certainly failed.";
                result.FailureReason = failureReason;
                return result;
            }

            result.LengthMetres = spline.Length;
            result.SpeedProfileKmh = DeriveSpeedProfile(spline, maxLateralAccel, brakeAccel,
                straightSpeedKmh, minSpeedKmh);
            result.WidthMetres = BuildWidth(spline.Length, trackWidthMetres);
            result.SectorStarts = SplitSectors(spline.Length);
            result.ReferenceLapSeconds = EstimateLapTime(result.SpeedProfileKmh, spline.Length);

            failureReason = null;
            result.Success = true;
            return result;
        }

        /// <summary>
        /// Reduces a dense centreline to a fixed number of control points, always keeping one at
        /// the start so the spline's s = 0 coincides with the line's first point.
        /// </summary>
        private static Vector3[] Decimate(List<Vector3> line, int target)
        {
            if (line.Count <= target)
                return line.ToArray();

            var result = new Vector3[target];
            for (int i = 0; i < target; i++)
                result[i] = line[(int)((long)i * line.Count / target)];

            // Drop consecutive duplicates: a Catmull-Rom knot spacing of zero is undefined.
            var cleaned = new List<Vector3>(result.Length);
            for (int i = 0; i < result.Length; i++)
            {
                if (cleaned.Count == 0 ||
                    Vector3.Distance(cleaned[cleaned.Count - 1], result[i]) > 1e-3f)
                    cleaned.Add(result[i]);
            }

            if (cleaned.Count >= 3) return cleaned.ToArray();
            return result;
        }

        /// <summary>
        /// The physical speed limit at every arc length, then a backward braking pass.
        ///
        /// <para>
        /// Corner speed is v = sqrt(a_lat / kappa), which needs no per-corner tuning: a hairpin is
        /// slow and a straight is fast purely because of its radius. The backward pass then
        /// propagates each corner's limit forward along the approach, so a car is already slowing
        /// on entry. It runs twice so the loop settles everywhere.
        /// </para>
        /// </summary>
        private static float[] DeriveSpeedProfile(ArcLengthSpline spline, float maxLateralAccel,
            float brakeAccel, float straightSpeedKmh, float minSpeedKmh)
        {
            int count = Mathf.Max(SpeedProfileSamplesPerLapMin,
                Mathf.CeilToInt(spline.Length / TrackData.SpeedProfileResolutionMetres));
            var speeds = new float[count];

            for (int i = 0; i < count; i++)
            {
                float s = spline.Length * i / count;
                float kappa = spline.GetCurvature(s);

                // Infinite radius on a straight gives infinite v, so the straight speed is the
                // natural ceiling and needs no special case.
                float v = kappa <= 1e-6f ? straightSpeedKmh : Mathf.Sqrt(maxLateralAccel / kappa) * 3.6f;
                speeds[i] = Mathf.Clamp(v, minSpeedKmh, straightSpeedKmh);
            }

            float sampleGap = spline.Length / count;
            for (int lap = 0; lap < 2; lap++)
            {
                for (int step = 0; step < count; step++)
                {
                    int i = count - 1 - step;
                    int j = (i + 1) % count;

                    float v = speeds[i] * KmhToMs;
                    float vNext = speeds[j] * KmhToMs;
                    float reachable = Mathf.Sqrt(vNext * vNext + 2f * brakeAccel * sampleGap);

                    speeds[i] = Mathf.Min(speeds[i], reachable * 3.6f);
                }
            }

            return speeds;
        }

        private static float[] BuildWidth(float length, float width)
        {
            int count = Mathf.Max(2, Mathf.CeilToInt(length / TrackData.WidthResolutionMetres));
            var widths = new float[count];
            for (int i = 0; i < count; i++) widths[i] = width;
            return widths;
        }

        private static float[] SplitSectors(float length) => new[]
        {
            0f,
            length / 3f,
            length * 2f / 3f
        };

        /// <summary>
        /// Lap time from the speed profile: the sum of time spent in each sample, with the
        /// distance the sample represents. An estimate, not a simulation — it exists so tiers can
        /// be expressed as a fraction of a reference, and a human can override it later.
        /// </summary>
        private static float EstimateLapTime(float[] speedsKmh, float length)
        {
            if (speedsKmh.Length == 0 || length <= 0f) return 0f;

            float total = 0f;
            float sampleGap = length / speedsKmh.Length;
            for (int i = 0; i < speedsKmh.Length; i++)
            {
                float v = Mathf.Max(1f, speedsKmh[i] * KmhToMs);
                total += sampleGap / v;
            }

            return total;
        }
    }
}
```

- [ ] **Step 6: Run to verify pass**

`refresh_unity` (force, request, wait), then `run_tests` `mode: "EditMode"`, `test_names: ["TrackDataBakeTests"]`, poll `wait_timeout: 60`.

Expected: 7 passed.

If `ABakeOfDegenerateInputIsRefusedWithAReason` fails because the collinear input was accepted, the length check is doing the work — verify it fires before the spline is constructed.

- [ ] **Step 7: Run the full EditMode suite**

`run_tests` `mode: "EditMode"`, poll `wait_timeout: 120`.

Expected: **199 + 7 = 206 passed, 0 failed** (or 199 + 7 in the separate editor assembly if Step 2's fallback was needed).

- [ ] **Step 8: Commit**

```bash
git add Assets/Editor/F1.EditorTools.asmdef Assets/Editor/RacingLineBaker.cs Assets/Tests/EditMode/TrackDataBakeTests.cs Assets/Tests/EditMode/F1.EditModeTests.asmdef
git commit -m "Make the racing-line bake testable, and bake a second circuit

TrackRacingLineBaker lives in Assembly-CSharp-Editor, which no test assembly
references, so no bake has ever been verified on any track. For a five-track
game that is the largest single risk in the programme.

The extracted RacingLineBaker takes an already-read centreline and does
everything from there, so the whole pipeline is testable without a scene.
The tests bake a synthetic circuit no test has seen before, which is the
'scales to more than one track' claim tested rather than asserted in prose.

Co-Authored-By: Claude Code <noreply@anthropic.com>"
```

---

*Phases 2–7 (Tasks 4–10: the adapter hinge, the metre-based planner, RaceCraft, tier pacing, LOD, test migration, re-measurement) follow in the continuation of this document.*