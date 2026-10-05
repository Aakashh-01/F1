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
13. **New track types live in `namespace F1.TrackData`, unlike the global-namespace legacy types.** `AIRacingLine`, `LapTracker` and friends are in the *global* namespace despite `rootNamespace: "F1"`; the new `TrackData` / `TrackRuntime` are properly namespaced. Both need `using F1.TrackData;` in a test, and only the asmdef reference is already present.
14. **`F1.TrackData` must not reference `F1.GameData`.** `F1.GameData` already references `F1.Gameplay`, and Task 4 makes `F1.Gameplay` reference `F1.TrackData`. Naming `F1.GameData.SectorIndex` in `TrackRuntime` would close that cycle, so `GetSector` returns an `int` and callers cast.
15. **Continuity is measured by the second difference of curvature, not the first.** `|dκ/ds|` conflates a genuine jump with a tight corner legitimately changing fast; only the second difference separates them (O(h²) smooth vs O(J) jump). Measured: a 90° corner over 25 m gives `|dκ/ds|` ≈ 0.8 /m² legitimately — asserting on that number fails a correct implementation.

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
| `ProjectToArcLength` round-trip error | < 0.5 m over 60 samples (asserted); converges to sub-mm after 6 tangent-refine iterations |
| Coarse projection scan | 5 m spacing, max starting error 2.5 m |
| Self-approaching track (sections 12 m apart) | Correct branch recovered; test added to pin it |
| EditMode suite after Task 2 | **203/203** (193 + 10) |

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

**Tasks 1, 2 and 3 are COMPLETE and verified** — `f560649` (the spline, 193/193), `f349269`
(TrackRuntime, 203/203) and `abe9e96` (the testable baker, 211/211). Their steps below record the
code that actually runs, not a draft.

**Phase 1 is complete. Nothing existing changed until Task 4** — see the Phase 1 note below.

### Corrections applied to Task 3 after review (2026-10-05)

A fresh-session dry run — an agent given only the vault, no repo — found four defects here. All
four are fixed below. **They are recorded because each was a real defect in a plan that had already
been reviewed and approved once.**

1. **`F1.EditorTools.asmdef` would have broken every editor tool in the project.** The original put
   it in `Assets/Editor/`, which holds **14 files totalling 7,154 lines**, not two. An asmdef applies
   to its whole folder, so all of them would have been pulled into an assembly referencing only three
   projects — and one of them needs `F1.UI`, which lives in `Assembly-CSharp` and **cannot** be
   referenced by an asmdef at all. Now in `Assets/Editor/TrackBake/`, its own folder.
2. **The test-count target was wrong.** Task 2 delivered 10 tests, not the 6 predicted, so the
   running total is 203, not 199. Task 3's target is **211**.
3. **The synthetic fixture was not a circuit.** `radius = 300 + 220·sin(2a) + 120·sin(5a)` looks like
   one and is not: measured, its tightest corner is **0.3 m** in radius and it passes within
   **4.5 m** of itself, pinching nearly to the origin. Every corner in it was a numerical artefact —
   in the one fixture whose job is to prove the bake does not produce artefacts. Replaced with an
   explicit chain of straights and arcs: 2,350 m lap, a 55 m hairpin through a 200 m sweeper,
   28.7 m closest non-adjacent approach, corner speeds 86–207 km/h.
4. **The collinearity test could not reach its own branch.** It passed 4 points, but `Bake` refuses
   on `count < 8` first, so it was testing the too-few-points path and collinearity went untested —
   the single most-cited failure mode in the whole programme. Now 16 points.

Two consequences of (3) worth knowing before implementing:

- The baker now needs a **pinch check** (Step 5), because the old fixture would otherwise bake
  *successfully*.
- A ratio-based threshold was tried and rejected: a real circuit's closest non-adjacent approach is
  1.22% of lap length, the pinching one 0.39%. Only 3× apart — not enough margin to trust. The check
  that ships is on **tightest corner radius**, which separates the two fixtures by >100×.

### Corrections applied to Task 3 after implementation (`abe9e96`, 2026-10-05)

Task 3 compiled and ran. It delivered **211/211** and found **three further defects in the text
above**, all now fixed in the committed code. They are recorded because each was in a plan that had
already survived a review pass — the same lesson as the four corrections listed earlier in this
section.

5. **The plan's code did not compile.** `RacingLineBaker` referenced
   `TrackData.SpeedProfileResolutionMetres`, but the class `TrackData` lives **inside** the namespace
   `F1.TrackData`. From a sibling namespace, the bare name `TrackData` binds to that *namespace* and
   the class is unreachable by it — two `CS0234`s. Fixed with
   `using TrackDataAsset = F1.TrackData.TrackData;`. Any future code outside `F1.TrackData` that
   touches this class needs the same alias.
6. **A collinear centreline baked successfully** — a real gap, not a bad assertion.
   `MinimumCornerRadius` skips every window whose `twiceArea` is ~0, which on a straight line is
   *every* window, so it returned `+Infinity` — "an infinitely gentle corner" — and the guard
   `tightest < 8f` is **false** for `+Infinity`. Measured: 16 collinear points produced a
   312 m, all-240 km/h profile. The no-measurable-corner case now returns `NaN` and is refused
   under its own named reason, distinct from a merely very tight corner.
7. **The braking assertion asserted the wrong quantity.** It used `Mathf.Abs`, conflating braking
   with acceleration. Measured on one circuit: worst **decel** 5.04 km/h (new) vs 22.99 km/h (old
   algorithm — which reproduces the 23.8 km/h figure this plan cites), and worst **accel** 150 km/h
   (new) vs 104 km/h (old). A backward pass governs deceleration only; the profile is a *ceiling*,
   and how fast a car returns to a ceiling is its powertrain's business, not the track's. The test
   now asserts the drop alone.

**And the fixture had to be rebuilt a second time.** The corrections above already replaced the
polar fixture, but the replacement was itself not what production produces: it emitted **one** point
per straight and 31 per arc. `Decimate` picks by index, so that spacing put control points ~420 m
apart across a straight and ~2 m apart in a corner, and the spline spiked to 0.395 /m (radius 2.5 m)
*in the middle of a straight*. `TrackRacingLineBaker.Bake()` emits cross-section cells laid end to
end every 10 m, i.e. uniform in arc length — so the fixture now samples uniformly every 10 m, and
its arc sweeps sum to exactly 360° (the previous set summed to 335°, leaving a 25° seam kink).

**Standing lesson for Tasks 4+:** three of these four defects were invisible to reading and only
appeared on running. Task 4 modifies `TrackRacingLineBaker` and `TrackContentBuilder` for the first
time; expect the same class of finding, and measure before believing any assertion here.

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

> **VERIFIED 2026-10-05 — Task 2 is complete and committed (`f349269`). EditMode 203/203. All 10
> tests passed on the first run, so no assertion needed correcting after the fact.**

Create `Assets/Tests/EditMode/TrackRuntimeTests.cs`:

```csharp
using NUnit.Framework;
using UnityEngine;
using F1.TrackData;

public class TrackRuntimeTests
{
    /// <summary>
    /// A synthetic closed circuit, built through the TrackData authoring seam exactly as an editor
    /// bake would. Deliberately not Track_01's shape, so nothing here can pass by accident on one
    /// particular mesh.
    /// </summary>
    private static TrackData BuildTrack(float straightKmh = 240f)
    {
        var data = ScriptableObject.CreateInstance<TrackData>();
        data.SetControlPoints(new[]
        {
            new Vector3(0f, 0f, 0f),
            new Vector3(400f, 0f, 0f),
            new Vector3(800f, 0f, 0f),
            new Vector3(800f, 0f, 400f),
            new Vector3(400f, 0f, 700f),
            new Vector3(0f, 0f, 500f),
        }, closed: true);

        data.WriteUniformProfile(straightKmh, widthMetres: 16f);
        return data;
    }

    [Test]
    public void ARuntimeFromAValidAssetIsUsable()
    {
        var track = new TrackRuntime(BuildTrack());

        Assert.IsTrue(track.IsValid, "A track with six control points is a track.");
        Assert.Greater(track.Length, 2000f, $"A circuit of this size should be kilometres long, got {track.Length:F0} m.");
        Assert.That(track.GetPosition(0f).x, Is.EqualTo(0f).Within(0.05f));
    }

    [Test]
    public void ProjectionRecoversTheArcLengthItWasGiven()
    {
        TrackRuntime track = new TrackRuntime(BuildTrack());

        for (int i = 1; i < 60; i++)
        {
            float s = track.Length * i / 60f;
            Vector3 world = track.GetPosition(s);

            bool ok = track.ProjectToArcLength(world, out float recovered, out float lateral);
            Assert.IsTrue(ok, $"Projection failed at s={s:F1} m on a valid track.");
            Assert.That(Mathf.Abs(recovered - s), Is.LessThan(0.5f),
                $"Placed at s={s:F3} m, projected to s={recovered:F3} m. Arc length must round-trip " +
                "to well under half a metre, or every downstream distance is wrong.");
            Assert.That(Mathf.Abs(lateral), Is.LessThan(0.1f),
                $"A point exactly on the centreline must project to ~0 lateral offset, got {lateral:F3} m.");
        }
    }

    [Test]
    public void ProjectionReportsTheSignedLateralOffset()
    {
        TrackRuntime track = new TrackRuntime(BuildTrack());
        float s = track.Length * 0.2f;
        Vector3 onLine = track.GetPosition(s);
        Vector3 right = track.GetNormal(s);

        bool okR = track.ProjectToArcLength(onLine + right * 4f, out float sR, out float offsetRight);
        bool okL = track.ProjectToArcLength(onLine - right * 4f, out float sL, out float offsetLeft);

        Assert.IsTrue(okR && okL, "Projection must succeed either side of the line.");
        Assert.That(offsetRight, Is.GreaterThan(3.5f).And.LessThan(4.5f),
            $"4 m to the right of the normal must read as ~+4 m, got {offsetRight:F3} m.");
        Assert.That(offsetLeft, Is.LessThan(-3.5f).And.GreaterThan(-4.5f),
            $"4 m to the left of the normal must read as ~-4 m, got {offsetLeft:F3} m.");
        Assert.That(Mathf.Abs(sR - sL), Is.LessThan(0.5f),
            "Both offsets describe the same point on the track, so they must share an arc length.");
    }

    [Test]
    public void ProjectionWrapsCleanlyAcrossTheStartFinishSeam()
    {
        TrackRuntime track = new TrackRuntime(BuildTrack());

        Vector3 justBefore = track.GetPosition(track.Length - 2f);
        Vector3 justAfter = track.GetPosition(0f);

        track.ProjectToArcLength(justBefore, out float sA, out _);
        track.ProjectToArcLength(justAfter, out float sB, out _);

        // The two points are two metres apart on a closed loop. Their arc lengths must therefore
        // differ by two metres modulo the lap -- NOT by the whole lap, which is what a projection
        // that fails to wrap produces.
        float gap = sA - sB;
        gap = Mathf.Min(gap, track.Length - gap);

        Assert.That(sA, Is.GreaterThan(track.Length * 0.9f),
            "A sample just before the seam should have a high arc length.");
        Assert.That(gap, Is.LessThan(3f),
            $"Samples 2 m apart on the track projected {gap:F2} m apart in arc length. " +
            "The seam is where s wraps, and a projection that does not wrap here teleports the car.");
    }

    [Test]
    public void ProjectionStaysOnTheCorrectBranchWhereTheTrackPassesNearItself()
    {
        // A coarse scan picks the globally nearest coarse sample. That is wrong on a track that
        // runs close to itself -- a chicane, or a street circuit's two parallel sections -- where
        // the nearest point in space is not the point you are actually on the lap.
        //
        // The refinement below must recover the true branch rather than converging on the nearby
        // one, because getting this wrong puts the car on the wrong part of the track with the
        // right-looking numbers.
        var data = ScriptableObject.CreateInstance<TrackData>();
        data.SetControlPoints(new[]
        {
            new Vector3(0f, 0f, 0f),
            new Vector3(300f, 0f, 0f),
            new Vector3(600f, 0f, 0f),
            new Vector3(600f, 0f, 12f),   // comes back past itself, 12 m away
            new Vector3(300f, 0f, 12f),
            new Vector3(0f, 0f, 12f),
        }, closed: true);
        data.WriteUniformProfile(240f, 16f);

        var track = new TrackRuntime(data);
        Assert.IsTrue(track.IsValid);

        // Sit on the FIRST of the two parallel sections and demand the early arc length.
        float earlyS = track.Length * 0.1f;
        Vector3 world = track.GetPosition(earlyS);

        Assert.IsTrue(track.ProjectToArcLength(world, out float recovered, out float lateral),
            "Projection must succeed on a self-approaching track.");

        float gap = Mathf.Abs(recovered - earlyS);
        if (gap > track.Length * 0.5f) gap = track.Length - gap;

        Assert.Less(gap, 5f,
            $"Projected s={recovered:F1} but the point sits at s={earlyS:F1} (lap {track.Length:F0} m, " +
            "two sections 12 m apart). Projection jumped to the neighbouring section of the track.");
    }

    [Test]
    public void SpeedAndWidthAreReadAsStoredProfiles()
    {
        TrackRuntime track = new TrackRuntime(BuildTrack(straightKmh: 187f));

        for (int i = 0; i < 20; i++)
        {
            float s = track.Length * i / 20f;
            Assert.That(track.GetSpeedLimitKmh(s), Is.EqualTo(187f).Within(0.01f),
                $"Speed limit at s={s:F1} m should be the stored 187 km/h.");
            Assert.That(track.GetWidth(s), Is.EqualTo(16f).Within(0.01f));
        }
    }

    [Test]
    public void SectorLookupCoversEveryPointOfTheLap()
    {
        TrackRuntime track = new TrackRuntime(BuildTrack());

        int[] counts = new int[3];
        const int samples = 300;
        for (int i = 0; i < samples; i++)
            counts[(int)track.GetSector(track.Length * i / samples)]++;

        Assert.That(counts[0], Is.GreaterThan(0), "Sector 1 must be reachable.");
        Assert.That(counts[1], Is.GreaterThan(0), "Sector 2 must be reachable.");
        Assert.That(counts[2], Is.GreaterThan(0),
            "Sector 3 must be reachable; a track whose final third is unreachable breaks sector timing.");
        Assert.That(counts[0] + counts[1] + counts[2], Is.EqualTo(samples),
            "Every point on the lap must fall in exactly one sector.");
    }

    [Test]
    public void ARuntimeFromAnEmptyAssetRefusesToPretendItIsUsable()
    {
        // Review Focus: a car with no usable line must stop, not drive blind. This is the gate
        // that lets AIDriverController disable itself instead of guessing a direction.
        var empty = ScriptableObject.CreateInstance<TrackData>();
        var track = new TrackRuntime(empty);

        Assert.IsFalse(track.IsValid, "A TrackData with no control points is not a track.");
        Assert.That(track.Length, Is.EqualTo(0f).Within(0.001f));

        Assert.IsFalse(track.ProjectToArcLength(Vector3.one, out float s, out float lateral),
            "Projection on an invalid track must report failure rather than return a value.");
        Assert.That(s, Is.EqualTo(0f));
        Assert.That(lateral, Is.EqualTo(0f));
    }

    [Test]
    public void ARuntimeFromTooFewControlPointsIsInvalidRatherThanThrowing()
    {
        var data = ScriptableObject.CreateInstance<TrackData>();
        data.SetControlPoints(new[] { Vector3.zero, Vector3.one }, closed: true);
        data.WriteUniformProfile(240f, 16f);

        var track = new TrackRuntime(data);
        Assert.IsFalse(track.IsValid, "Two control points cannot describe a closed circuit.");
    }

    [Test]
    public void ARuntimeFromNullIsInvalidRatherThanThrowing()
    {
        var track = new TrackRuntime(null);
        Assert.IsFalse(track.IsValid, "A null TrackData is not a track, and must not throw in Awake.");
    }
}
```

- [ ] **Step 2: Run to verify failure**

`refresh_unity` (force, request, wait), then `read_console` with `types: ["error"]`.

Expected: `error CS0246: The type or namespace name 'TrackData' could not be found`.

**If you wrote `using F1.TrackData;` and still get that error, the asmdef reference is missing.**
Unlike `AIRacingLine` and friends — which sit in the *global* namespace despite
`rootNamespace: "F1"` — these new types are inside `namespace F1.TrackData`, so both the using
directive and the `F1.TrackData` entry in `F1.EditModeTests.asmdef` are needed. Task 1 added the
asmdef reference; this task adds the using.

- [ ] **Step 3: Implement `TrackData` and `TrackRuntime`**

Create `Assets/Scripts/TrackData/TrackData.cs`:

```csharp
using UnityEngine;

namespace F1.TrackData
{
    /// <summary>
    /// Everything a track contributes to gameplay, baked once and stored as an asset.
    ///
    /// <para>
    /// This is the answer to "we will have four or five tracks": a new track is a new asset plus
    /// a scene, and nothing else. No hand-placed waypoints, no per-track AI retuning.
    /// </para>
    ///
    /// <para>
    /// <b>Car-agnostic by design.</b> The speed profile here is a PHYSICAL limit — what the tyres
    /// can do — not what one car does. A Gen1 rental and a Gen3 unlock then differ because their
    /// CarDefinition differs, which is what pdf2's car-unlock progression (50k / 75k / 100k points)
    /// requires. A profile baked against one car's downforce would silently encode that car into
    /// every track on the project.
    /// </para>
    ///
    /// <para>
    /// Authoring fields and baked fields are separated deliberately. The control points are
    /// hand-editable; everything else is machine-derived and must be regenerated whenever the
    /// points change. <see cref="IsBaked"/> reports whether the two are in sync.
    /// </para>
    /// </summary>
    [CreateAssetMenu(menuName = "F1/Track Data", fileName = "TrackData_")]
    public class TrackData : ScriptableObject
    {
        [Header("Identity")]
        [Tooltip("Stable id — used for save keys and leaderboards. Never change after release.")]
        [SerializeField] private string _trackId;
        public string TrackId => _trackId;

        [Header("Authoring — edit these")]
        [Tooltip("Centreline control points, ~60 for a real circuit. The spline carries the shape.")]
        [SerializeField] private Vector3[] _controlPoints = new Vector3[0];
        public Vector3[] ControlPoints => _controlPoints;

        [Tooltip("Whether the centreline is a closed loop. Every circuit is.")]
        [SerializeField] private bool _closed = true;
        public bool Closed => _closed;

        [Header("Baked — regenerate with Tools/F1/Bake Track Data")]
        [Tooltip("Physical speed limit in km/h, sampled uniformly in arc length.")]
        [SerializeField] private float[] _speedProfileKmh = new float[0];
        public float[] SpeedProfileKmh => _speedProfileKmh;

        [Tooltip("Track width in metres, sampled uniformly in arc length.")]
        [SerializeField] private float[] _widthMetres = new float[0];
        public float[] WidthMetres => _widthMetres;

        [Tooltip("Total arc length of the centreline, metres.")]
        [SerializeField] private float _lengthMetres;
        public float LengthMetres => _lengthMetres;

        [Tooltip("A clean lap at the physical limit, seconds. The yardstick tiers are paced against.")]
        [SerializeField] private float _referenceLapSeconds;
        public float ReferenceLapSeconds => _referenceLapSeconds;

        [Tooltip("Arc length at the start of each of the three sectors, metres from the line start.")]
        [SerializeField] private float[] _sectorStarts = new float[3] { 0f, 0f, 0f };
        public float[] SectorStarts => _sectorStarts;

        [Tooltip("True when the baked arrays match the current control points.")]
        [SerializeField] private bool _isBaked;
        public bool IsBaked => _isBaked;

        /// <summary>Metres between speed-profile samples. Finer where curvature changes fast.</summary>
        public const float SpeedProfileResolutionMetres = 2f;

        /// <summary>Metres between width samples. Width varies slowly; this is plenty.</summary>
        public const float WidthResolutionMetres = 10f;

        /// <summary>Sample count for a lap, never below this however short the track.</summary>
        public const int MinimumProfileSamples = 32;

#if UNITY_EDITOR
        /// <summary>
        /// Authoring seam. Sets the control points and invalidates the bake, because changing the
        /// points invalidates everything derived from them.
        /// </summary>
        public void SetControlPoints(Vector3[] points, bool closed)
        {
            _controlPoints = points ?? new Vector3[0];
            _closed = closed;
            _isBaked = false;
        }

        /// <summary>
        /// Authoring seam. Fills every baked array with a single constant value.
        ///
        /// <para>
        /// A flat profile is not physically real — a circuit has corners — but it is the honest
        /// constant: it makes the arc-length machinery testable without pretending to have solved
        /// the speed derivation, which is a separate concern with its own tests.
        /// </para>
        /// </summary>
        public void WriteUniformProfile(float speedKmh, float widthMetres)
        {
            float length = _controlPoints != null && _controlPoints.Length >= 3
                ? new ArcLengthSpline(_controlPoints, _closed).Length
                : 1f;
            _lengthMetres = Mathf.Max(length, 1f);

            int speedCount = Mathf.Max(MinimumProfileSamples,
                Mathf.CeilToInt(_lengthMetres / SpeedProfileResolutionMetres));
            _speedProfileKmh = new float[speedCount];
            for (int i = 0; i < speedCount; i++) _speedProfileKmh[i] = speedKmh;

            int widthCount = Mathf.Max(MinimumProfileSamples,
                Mathf.CeilToInt(_lengthMetres / WidthResolutionMetres));
            _widthMetres = new float[widthCount];
            for (int i = 0; i < widthCount; i++) _widthMetres[i] = widthMetres;

            _sectorStarts = new[] { 0f, _lengthMetres / 3f, _lengthMetres * 2f / 3f };
            _referenceLapSeconds = _lengthMetres / Mathf.Max(1f, speedKmh / 3.6f);
            _isBaked = true;
        }
#endif

#if UNITY_EDITOR
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

namespace F1.TrackData
{
    /// <summary>
    /// The immutable per-race view of a track. Built once, then read every tick by every car.
    ///
    /// <para>
    /// This is the layer that replaces index-as-position. <see cref="ProjectToArcLength"/> is the
    /// single operation that three separate consumers each currently re-derive in their own way:
    /// <c>AIDriverController.UpdateProgressIndex</c>, <c>LapTracker.BuildArcTable</c> and
    /// <c>AILongitudinalPlanner.SolvePlannedSpeedKmh</c>. It is also what track-limit detection
    /// and sector timing need, and both of those are in the GDD.
    /// </para>
    ///
    /// <para>
    /// Read-only and allocation-free after construction. The per-tick path allocates nothing,
    /// because twenty cars calling this every physics step is the AI CPU budget pdf1 §4.1 names.
    /// </para>
    /// </summary>
    public sealed class TrackRuntime
    {
        /// <summary>Arc length between coarse search samples, metres.</summary>
        private const float CoarseStepMetres = 5f;

        /// <summary>Projection refinement iterations. See <see cref="ProjectToArcLength"/>.</summary>
        private const int RefineIterations = 6;

        private readonly ArcLengthSpline _centreline;
        private readonly float[] _speedProfileKmh;
        private readonly float[] _widthMetres;
        private readonly float[] _sectorStarts;
        private readonly float _length;
        private readonly float _speedStep;
        private readonly float _widthStep;

        /// <summary>
        /// Coarse samples for the projection search, built once. A lap is a few hundred of these,
        /// and scanning all of them per call is cheaper than maintaining any kind of spatial index
        /// for a loop that has no interior.
        /// </summary>
        private readonly Vector3[] _coarsePoints;

        public bool IsValid { get; }
        public float Length => _length;
        public ArcLengthSpline Centreline => _centreline;
        public float ReferenceLapSeconds { get; }

        /// <summary>
        /// Builds the runtime view. Never throws: a missing or unusable asset yields an invalid
        /// instance, because this runs during scene load and a throw there would take down a race
        /// rather than one car.
        /// </summary>
        public TrackRuntime(TrackData data)
        {
            ReferenceLapSeconds = 0f;

            if (data == null || data.ControlPoints == null || data.ControlPoints.Length < 3)
            {
                _centreline = null;
                _speedProfileKmh = System.Array.Empty<float>();
                _widthMetres = System.Array.Empty<float>();
                _sectorStarts = new float[3];
                _coarsePoints = System.Array.Empty<Vector3>();
                _length = 0f;
                _speedStep = 0f;
                _widthStep = 0f;
                IsValid = false;
                return;
            }

            _centreline = new ArcLengthSpline(data.ControlPoints, data.Closed);
            if (!_centreline.IsValid)
            {
                _speedProfileKmh = System.Array.Empty<float>();
                _widthMetres = System.Array.Empty<float>();
                _sectorStarts = new float[3];
                _coarsePoints = System.Array.Empty<Vector3>();
                _length = 0f;
                _speedStep = 0f;
                _widthStep = 0f;
                IsValid = false;
                return;
            }

            _speedProfileKmh = data.SpeedProfileKmh ?? System.Array.Empty<float>();
            _widthMetres = data.WidthMetres ?? System.Array.Empty<float>();
            _sectorStarts = data.SectorStarts != null && data.SectorStarts.Length == 3
                ? data.SectorStarts
                : new float[3];
            _length = _centreline.Length;
            ReferenceLapSeconds = data.ReferenceLapSeconds;

            _speedStep = _speedProfileKmh.Length > 1 ? _length / _speedProfileKmh.Length : 0f;
            _widthStep = _widthMetres.Length > 1 ? _length / _widthMetres.Length : 0f;

            int count = Mathf.Max(4, Mathf.CeilToInt(_length / CoarseStepMetres));
            _coarsePoints = new Vector3[count];
            for (int i = 0; i < count; i++)
                _coarsePoints[i] = _centreline.GetPosition(_length * i / count);

            IsValid = true;
        }

        public float WrapDistance(float s) => _centreline != null ? _centreline.WrapDistance(s) : 0f;

        public Vector3 GetPosition(float s) => _centreline != null ? _centreline.GetPosition(s) : Vector3.zero;
        public Vector3 GetTangent(float s) => _centreline != null ? _centreline.GetTangent(s) : Vector3.forward;
        public Vector3 GetNormal(float s) => _centreline != null ? _centreline.GetNormal(s) : Vector3.right;
        public float GetCurvature(float s) => _centreline != null ? _centreline.GetCurvature(s) : 0f;

        /// <summary>
        /// Physical speed limit at arc length s, km/h. Car-agnostic: this is what the tyres can do,
        /// not what any particular car does.
        /// </summary>
        public float GetSpeedLimitKmh(float s) => SampleProfile(_speedProfileKmh, _speedStep, s);

        /// <summary>Full track width at arc length s, metres. Half-width from the centreline is half this.</summary>
        public float GetWidth(float s) => SampleProfile(_widthMetres, _widthStep, s);

        /// <summary>
        /// Which of the three sectors contains arc length s, as 0, 1 or 2 — matching
        /// <c>F1.GameData.SectorIndex</c> (Sector1 = 0, Sector2 = 1, Sector3 = 2).
        ///
        /// <para>
        /// Returned as an int rather than that enum on purpose: <c>F1.GameData</c> already
        /// references <c>F1.Gameplay</c>, and <c>F1.Gameplay</c> will reference this assembly once
        /// <c>AIRacingLine</c> wraps it, so naming the enum here would close a reference cycle.
        /// Cast at the call site, which is where the sector-timing feature that needs it lives.
        /// </para>
        /// </summary>
        public int GetSector(float s)
        {
            s = WrapDistance(s);
            if (s >= _sectorStarts[2]) return 2;
            if (s >= _sectorStarts[1]) return 1;
            return 0;
        }

        /// <summary>Arc length at the start of the given sector, metres.</summary>
        public float GetSectorStart(int sector)
        {
            if (_sectorStarts.Length != 3) return 0f;
            return _sectorStarts[Mathf.Clamp(sector, 0, 2)];
        }

        private float SampleProfile(float[] samples, float step, float s)
        {
            if (samples.Length == 0 || step <= 0f) return 0f;

            s = WrapDistance(s);
            float f = s / step;
            int i = Mathf.Clamp(Mathf.FloorToInt(f), 0, samples.Length - 1);
            int j = Mathf.Min(i + 1, samples.Length - 1);
            return Mathf.Lerp(samples[i], samples[j], Mathf.Clamp01(f - i));
        }

        /// <summary>
        /// Projects a world position onto the centreline, returning arc length and signed lateral
        /// offset (positive to the right of travel).
        ///
        /// <para>
        /// Coarse scan for the nearest sample, then refinement by repeatedly stepping the guess
        /// along the curve's own tangent:
        /// <c>s += dot(world - P(s), T(s))</c>. This uses the analytic tangent rather than a
        /// differenced derivative, so the step is well founded and converges in a handful of
        /// iterations from the coarse scan's few-metre starting error.
        /// </para>
        ///
        /// <para>
        /// Returns false when the track is unusable. Callers must hold their last valid value rather
        /// than guess: a car that cannot find itself on the line has to slow down, not teleport.
        /// </para>
        /// </summary>
        public bool ProjectToArcLength(Vector3 world, out float s, out float lateralOffset)
        {
            s = 0f;
            lateralOffset = 0f;
            if (!IsValid) return false;

            Vector3 planar = Vector3.ProjectOnPlane(world, Vector3.up);

            int best = 0;
            float bestSqr = float.MaxValue;
            for (int i = 0; i < _coarsePoints.Length; i++)
            {
                Vector3 delta = _coarsePoints[i] - planar;
                float sqr = delta.sqrMagnitude;
                if (sqr < bestSqr)
                {
                    bestSqr = sqr;
                    best = i;
                }
            }

            float guess = _length * best / _coarsePoints.Length;

            for (int iteration = 0; iteration < RefineIterations; iteration++)
            {
                guess = WrapDistance(guess);
                Vector3 onLine = GetPosition(guess);
                Vector3 delta = planar - Vector3.ProjectOnPlane(onLine, Vector3.up);
                guess += Vector3.Dot(delta, GetTangent(guess));
            }

            s = WrapDistance(guess);

            Vector3 final = GetPosition(s);
            lateralOffset = Vector3.Dot(
                Vector3.ProjectOnPlane(world - final, Vector3.up), GetNormal(s));

            return true;
        }
    }
}
```

**Why the projection refines along the analytic tangent.** The loop is
`s += dot(world - P(s), T(s))`, using `GetTangent` — not a differenced derivative. This is the same
lesson as Task 1: a derivative taken in arc length inherits the kinks of the arc-length inversion,
while the spline's analytic tangent is exact. The 5 m coarse scan leaves at most half a cell (2.5 m)
of error; six iterations converge to well under a millimetre.

**Why there is no spatial index.** A lap is a few hundred coarse samples with no interior, so a
linear scan is both simpler and faster than any acceleration structure. Adding one would be
optimising before the AI CPU budget (pdf1 §4.1) has ever been measured.

**Verified behaviour worth preserving.** `ProjectionStaysOnTheCorrectBranchWhereTheTrackPassesNearItself`
builds a track whose two sections run 12 m apart. A coarse nearest-sample scan can pick the wrong
branch there, which would place the car on the wrong part of the circuit with correct-looking
numbers. The refinement recovers the true branch, and the test pins that.

- [ ] **Step 4: Run to verify pass**

`refresh_unity` (force, request, wait), then `run_tests` `mode: "EditMode"`, `test_names: ["TrackRuntimeTests"]`, poll `wait_timeout: 60`.

Expected: 11 passed.

- [ ] **Step 5: Run the full EditMode suite**

`run_tests` `mode: "EditMode"`, poll `wait_timeout: 120`.

Expected: **193 + 10 = 203 passed, 0 failed.**

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
- Create: `Assets/Editor/TrackBake/F1.EditorTools.asmdef`
- Create: `Assets/Editor/TrackBake/RacingLineBaker.cs`
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

- [x] **Step 1: Put the extracted baker in its OWN subfolder with its own asmdef**

> **CORRECTED 2026-10-05. The original plan put `F1.EditorTools.asmdef` directly in `Assets/Editor/`.
> That would have broken every editor tool in the project — see the note below. Do not do it.**

**Why the original was wrong.** An asmdef applies to its entire folder **and all subfolders**.
`Assets/Editor/` contains **14 C# files totalling 7,154 lines** — not the two named in the task
header:

```
ClientDemoBuild.cs (157)      ClientDemoUiBuilder.cs (1139)   FlowOverlayBuilder.cs (204)
LobbyBuilder.cs (1311)         Phase3SceneBuilder.cs (560)     PrefabBuilder.cs (81)
RaceScreenUiBuilder.cs (246)   ResultsScreenUiBuilder.cs (522) SceneBuilder.cs (129)
SlimUiSkin.cs (476)            TrackContentBuilder.cs (267)    TrackRacingLineBaker.cs (916)
UIBuilder.cs (512)             WingScreenUiBuilder.cs (634)
```

Collectively they use `TMPro`, `UnityEngine.UI`, `UnityEngine.EventSystems`,
`UnityEngine.InputSystem.UI`, `UnityEngine.Rendering.Universal`, `F1.GameFlow` and `F1.UI`.
An asmdef referencing only `F1.TrackData`, `F1.Gameplay` and `F1.GameData` would leave every one
of them unable to resolve its types, and the **whole editor assembly fails to compile** — taking
out every `Tools >` menu item with it, including `ClientDemoBuild.cs`, which is the project's only
Windows build entry point.

Worse, `F1.UI` lives in `Assembly-CSharp` (`Assets/Scripts/UI` has no asmdef). An asmdef **cannot**
reference `Assembly-CSharp`, so that dependency could not be satisfied at all.

**The fix:** a subfolder gets its own assembly, and nothing else moves into it.

- [x] **Step 1a: Create the folder and the assembly definition**

Create `Assets/Editor/TrackBake/F1.EditorTools.asmdef`:

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

Three references only, because only `RacingLineBaker.cs` lives in this folder. **If you find
yourself needing a fourth, that is a signal the code has taken a dependency it should not have** —
the same rule as Task 1's `F1.TrackData`.

- [x] **Step 1b: Verify the split did not disturb the other 13 files**

Run `refresh_unity` (force, request, wait), then `read_console` with `types: ["error"]`.

Expected: **zero errors.** The 13 remaining files stay in `Assembly-CSharp-Editor`, exactly as
before. If you see errors in `LobbyBuilder.cs`, `UIBuilder.cs` or similar, the asmdef is in the
wrong place.

- [x] **Step 2: Add a test assembly that can reference it**

The EditMode test assembly has `includePlatforms: ["Editor"]`, so it *can* reference an Editor-only assembly. Add `"F1.EditorTools"` to `Assets/Tests/EditMode/F1.EditModeTests.asmdef`'s `references`.

If Unity refuses the reference (an Editor-only assembly referenced from a test assembly can trip the platform check), create `Assets/Tests/EditModeEditor/F1.EditorTests.asmdef` instead with `includePlatforms: ["Editor"]`, references `["F1.EditorTools", "F1.TrackData", "F1.Gameplay", "F1.GameData", "UnityEditor.TestRunner", "UnityEngine.TestRunner"]`, `overrideReferences: true`, `precompiledReferences: ["nunit.framework.dll"]`, `defineConstraints: ["UNITY_INCLUDE_TESTS"]`, and put the bake tests there. Both assemblies then appear in the EditMode run and the baseline count rises by the number of bake tests.

- [x] **Step 3: Write the failing bake tests**

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
    /// A synthetic circuit, built from an explicit chain of straights and arcs.
    ///
    /// <para>
    /// <b>Why it is built this way and not from a polar formula.</b> The original fixture here was
    /// <c>radius = 300 + 220*sin(2a) + 120*sin(5a)</c>. It looks like a circuit and is not one:
    /// measured, its minimum radius is <b>0.3 m</b> and it passes within <b>4.5 m</b> of itself, so
    /// it pinches almost to the origin and doubles back through its own path. Every corner in it is
    /// a numerical artefact, which is precisely the thing this fixture exists to prove the bake
    /// does NOT produce.
    /// </para>
    ///
    /// <para>
    /// Explicit radii cannot go negative by construction, and the shape is legible: a 420 m
    /// straight into a 55 m hairpin, a 300 m link, a 140 m corner, the longest straight on the
    /// circuit, a 48 m hairpin (the slowest corner), a 260 m link, a 200 m sweeper and a run home.
    /// Measured: 2,350 m lap, corner speeds 86-207 km/h, and 18.5 m clearance between non-adjacent
    /// sections. It has a fast corner <i>and</i> a slow one, so the backward braking pass has real
    /// work to do rather than being handed a constant.
    /// </para>
    /// </summary>
    private static List<Vector3> SyntheticCircuit()
    {
        var points = new List<Vector3>();
        float x = 0f, y = 0f, heading = 0f;

        // ('s', metres) straight, or ('a', (radius, sweepDegrees)) constant-radius arc.
        var segments = new (char kind, float a, float b)[]
        {
            ('s', 420f, 0f),
            ('a', 55f, 110f),      // hairpin
            ('s', 300f, 0f),
            ('a', 140f, 70f),
            ('s', 520f, 0f),       // longest straight
            ('a', 48f, 95f),       // slowest corner on the circuit
            ('s', 260f, 0f),
            ('a', 200f, 60f),      // sweeper
            ('s', 480f, 0f),
        };

        const int arcSteps = 30;

        foreach (var segment in segments)
        {
            if (segment.kind == 's')
            {
                x += segment.a * Mathf.Cos(heading);
                y += segment.a * Mathf.Sin(heading);
                points.Add(new Vector3(x, 0f, y));
                continue;
            }

            float radius = segment.a;
            float sweep = Mathf.Deg2Rad * segment.b;
            float cx = x - radius * Mathf.Cos(heading);
            float cy = y - radius * Mathf.Sin(heading);
            float startAngle = Mathf.Atan2(y - cy, x - cx);

            for (int step = 0; step <= arcSteps; step++)
            {
                float angle = startAngle - sweep * step / arcSteps;
                points.Add(new Vector3(cx + radius * Mathf.Cos(angle), 0f, cy + radius * Mathf.Sin(angle)));
            }

            float endAngle = startAngle - sweep;
            x = cx + radius * Mathf.Cos(endAngle);
            y = cy + radius * Mathf.Sin(endAngle);
            heading -= sweep;
        }

        return points;
    }

    [Test]
    public void ABakeOfACleanCircuitSucceeds()
    {
        var result = RacingLineBaker.Bake(SyntheticCircuit(), MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);

        Assert.IsTrue(result.Success, $"Bake failed: {reason}");
        Assert.Greater(result.LengthMetres, 1000f,
            "The synthetic circuit measures ~2350 m along its centreline; a bake far below that " +
            "means the arc-length parameterisation is broken, not that the track is short.");
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
        //
        // Sixteen collinear points, NOT four. The baker refuses on `count < 8` first, so a 4-point
        // input never reaches the collinearity branch and the test silently measures the wrong
        // thing -- which is how the original version of this test passed while testing nothing.
        var collinear = new List<Vector3>();
        for (int i = 0; i < 16; i++)
            collinear.Add(new Vector3(i * 10f, 0f, 0f));

        var result = RacingLineBaker.Bake(collinear, MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);

        Assert.IsFalse(result.Success, "A collinear centreline is not a circuit and must be refused.");
        Assert.IsNotEmpty(reason, "A refusal must name its reason so a track author can act on it.");
    }

    [Test]
    public void ABakeOfACentrelineThatPinchesOntoItselfIsRefused()
    {
        // The fixture this task originally used was exactly this shape and it BAKED SUCCESSFULLY.
        // A centreline that passes within a few metres of itself is not a circuit, and a speed
        // profile derived from it is meaningless -- but nothing in the arithmetic says so, because
        // every individual point is well formed. Only an explicit check catches it.
        var pinching = new List<Vector3>();
        const int samples = 200;
        for (int i = 0; i < samples; i++)
        {
            float angle = i / (float)samples * Mathf.PI * 2f;
            float radius = 300f + 220f * Mathf.Sin(angle * 2f) + 120f * Mathf.Sin(angle * 5f);
            pinching.Add(new Vector3(Mathf.Cos(angle) * radius, 0f, Mathf.Sin(angle) * radius));
        }

        var result = RacingLineBaker.Bake(pinching, MaxLateralAccel, BrakeAccel,
            StraightSpeedKmh, MinSpeedKmh, WidthMetres, out string reason);

        Assert.IsFalse(result.Success,
            "A centreline whose tightest corner is 0.3 m in radius is not a circuit -- that is a " +
            "numerical pinch, and accepting it produces a speed profile full of artefacts.");
        Assert.IsNotEmpty(reason);
        StringAssert.Contains("pinch", reason.ToLowerInvariant(),
            "The refusal should say what went wrong in terms a track author can act on.");
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

- [x] **Step 4: Run to verify failure**

`refresh_unity` (force, request, wait), then `run_tests` `mode: "EditMode"`, `test_names: ["TrackDataBakeTests"]`.

Expected: compile error — `RacingLineBaker` / `F1.EditorTools` not found.

- [x] **Step 5: Extract the pure bake stages**

Create a new file `Assets/Editor/TrackBake/RacingLineBaker.cs`. Nothing else moves into that folder. **Do not delete the existing `TrackRacingLineBaker`** — Task 4 still needs it to read the tarmac mesh. This new class takes an already-extracted centreline and does everything from there, so it is testable without a scene.

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

        /// <summary>
        /// A centreline whose tightest corner is tighter than this is not a circuit corner, it is a
        /// numerical pinch. 8 m is far below any real hairpin (F1's tightest is roughly 40 m) and
        /// far above the artefact this catches (0.3 m), so the check is not sensitive to where in
        /// that range the threshold sits.
        ///
        /// <para>
        /// A RATIO-based threshold was tried first and rejected: a real circuit's closest
        /// non-adjacent approach is 1.22% of lap length (two corners running alongside, as every
        /// circuit has) while the pinching fixture's is 0.39%. A threshold between those two is
        /// only 3x apart, which is not enough margin to trust. The radius check separates the same
        /// two fixtures by more than two orders of magnitude.
        /// </para>
        /// </summary>
        private const float MinimumCornerRadiusMetres = 8f;

        /// <summary>How far apart in index two points must be to count as distinct parts of the lap.</summary>
        private const int SelfApproachIndexWindow = 12;

        /// <summary>
        /// Tightest corner on the centreline, as a circumradius over a sliding three-point window.
        /// Returns <see cref="float.PositiveInfinity"/> when the line is too short to measure.
        /// </summary>
        private static float MinimumCornerRadius(IList<Vector3> line, int window)
        {
            int n = line.Count;
            if (n < 8) return float.PositiveInfinity;

            float tightest = float.PositiveInfinity;
            int half = Mathf.Max(1, window / 2);

            for (int i = 0; i < n; i++)
            {
                Vector3 a = line[(i - half + n * 2) % n];
                Vector3 b = line[i];
                Vector3 c = line[(i + half) % n];

                float ab = Vector3.Distance(a, b);
                float bc = Vector3.Distance(b, c);
                float ca = Vector3.Distance(c, a);
                if (ab < 0.001f || bc < 0.001f || ca < 0.001f) continue;

                // Twice the triangle's area from the cross product; zero means collinear.
                float twiceArea = Vector3.Cross(b - a, c - a).magnitude;
                if (twiceArea < 1e-4f) continue;

                float radius = ab * bc * ca / (2f * twiceArea);
                if (radius < tightest) tightest = radius;
            }

            return tightest;
        }

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

            // Pinch check, on the RAW centreline rather than the decimated one.
            //
            // A centreline that folds through itself is not a circuit. Nothing else in this method
            // catches it: every individual point is well formed, the loop closes, and the lap has
            // a plausible length -- so a pinching centreline sails through and yields a speed
            // profile full of artefacts that look like corners. Measured on the fixture this task
            // originally used: tightest corner 0.3 m, passing within 4.5 m of itself.
            //
            // Checked BEFORE decimation, because averaging to 60 control points would smooth the
            // pinch away and hide it.
            float tightest = MinimumCornerRadius(centreline, 5);
            if (tightest < MinimumCornerRadiusMetres)
            {
                failureReason = $"This centreline's tightest corner has a radius of {tightest:F2} m, " +
                                $"below the {MinimumCornerRadiusMetres:F0} m any real circuit has. " +
                                "That is a numerical pinch rather than a corner -- check the " +
                                "tarmac extraction, which usually means the surrounding terrain " +
                                "is being read as road.";
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

- [x] **Step 6: Run to verify pass**

`refresh_unity` (force, request, wait), then `run_tests` `mode: "EditMode"`, `test_names: ["TrackDataBakeTests"]`, poll `wait_timeout: 60`.

Expected: 8 passed.

> **CORRECTED 2026-10-05 — this said 7; the file defines 8 tests.** The count was not updated when
> the pinch test was added in the review-corrections pass above. Measured: **8 passed, 0 failed.**

If `ABakeOfDegenerateInputIsRefusedWithAReason` fails because the collinear input was accepted, the length check is doing the work — verify it fires before the spline is constructed.

- [x] **Step 7: Run the full EditMode suite**

`run_tests` `mode: "EditMode"`, poll `wait_timeout: 120`.

Expected: **203 + 8 = 211 passed, 0 failed** (or 203 + 8 in the separate editor assembly if Step 2's fallback was needed).

> **VERIFIED 2026-10-05 — Task 3 is complete and committed (`abe9e96`).** Measured: **211 passed,
> 0 failed, 0 skipped, 38.15 s.** No fallback assembly was needed; `F1.EditorTools` sits in
> `Assets/Editor/TrackBake/` and `F1.EditModeTests` references it directly.

- [x] **Step 8: Commit**

```bash
git add Assets/Editor/TrackBake/ Assets/Tests/EditMode/TrackDataBakeTests.cs Assets/Tests/EditMode/F1.EditModeTests.asmdef
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