# AI Driving System — Architecture Design

Written 2026-10-05. Supersedes `Assets/Docs/SESSION-HANDOFF-SPLINE-RACING-LINE.md` and the
waypoint-array approach it describes. The AI driving system is being rebuilt from first
principles against the two GDDs, not patched.

Source requirements: `Assets/Assets/pdf1.pdf` (§1.1–§1.4 physics and steering, §2.3 AI
Behaviour V1, §3.3 AI Behaviour Final, §4.1 optimisation) and `Assets/Assets/pdf2.pdf`
(racing line, track limits, slipstream, sectors, qualifying/ghost, progression).

## 1. Intent and success criteria

Build an AI racing driver that satisfies the GDD's AI Behaviour requirements on **five tracks
from one architecture**, not one track from a hand-tuned array.

The GDD's AI requirements, quoted:

| Requirement | Source | Status today |
|---|---|---|
| Racing line system | pdf1 §2.3, pdf2 | 300 GameObjects baked from the tarmac mesh |
| Corner speed adaptation | pdf1 §2.3 | Derived per waypoint from Menger curvature |
| Collision avoidance logic | pdf1 §2.3 | Exists, single-stage |
| Basic overtaking logic | pdf1 §2.3 | Exists as a speed boost, not a manoeuvre |
| Difficulty scaling presets | pdf1 §2.3 | Three assets, **identical values** |
| Adaptive racing logic | pdf1 §3.3 | Not built |
| Clean defensive driving | pdf1 §3.3 | Not built |
| Consistent lap time targeting | pdf1 §3.3 | Not built |
| Competitive across tiers | pdf1 §3.3 | Cannot pass — tiers are identical |
| AI CPU load optimisation | pdf1 §4.1 | Not addressed |

**Success is measured, not asserted.** The gate is a new EditMode + PlayMode suite where:

1. A track's geometry is **one data asset**, and building track #2 costs a bake, not a
   re-tune. Proven by baking a second track's asset in a test.
2. **Curvature is continuous**, so the derived speed profile cannot step between adjacent
   samples. The handoff measured a 99 km/h step between adjacent waypoints on Track_01; the
   new architecture must show a worst-step figure an order of magnitude smaller, and the
   test must **assert** it (today it is only logged, deliberately).
3. **Tiers differ by a real, measured margin** on one line and pace consistently, so §3.3's
   "competitive across difficulty tiers" stops being unpassable.
4. The AI laps its own track without track-limit incidents, which is pdf2's
   "anti-collision system minimises track-limit incidents".

### Non-goals

- A **racing-line optimiser** (Frenet frame minimising the curvature integral). This design
  consumes a centreline; it does not search for the optimal one. Revisit after the AI drives
  acceptably.
- Slipstream and electric-boost AI coupling — pdf2 marks slipstream `tbd` and the handoff
  defers it. The architecture leaves a slot; it does not implement it.
- Rewriting the vehicle physics. That is pdf1 §1.1–§2.2 and is a separate programme. The AI
  reads the car's *measured* limits rather than assuming them, which is what makes it survive
  a physics change.

## 2. Why the current substrate is being replaced

Not because it is ugly — because it cannot express what the GDD asks for.

- **Index is the coordinate system.** `AIRacingLine` is a 300-entry array; eight runtime
  consumers plus eight test files read it by index. `AIDriverController.cs:834`
  (`CurrentWaypointIndex = WrapIndex(CurrentWaypointIndex + 1)`) is a progress *counter* built
  on an array *subscript*. `TrackPlacement._startFinishIndex` is a serialised slot in
  `Track_01.unity`. Every one of these is a place where adding a control point silently
  renumbers physical reality.
- **Three of five waypoint fields have exactly one distinct value** across all 300 waypoints.
  The representation carries almost no information and reads as if it does.
- **`RefreshWaypoints()` rebuilds the array from hierarchy order**, so the hierarchy — an
  editor convenience — is the authority on the racing line.
- **The line is 300 GameObjects in a scene**, not an asset. Tooling that reads project data
  (leaderboards, sector times, the ghost, minimaps) cannot see it.
- **The baker is untestable and Editor-only.** `TrackRacingLineBaker` and `TrackContentBuilder`
  live in `Assembly-CSharp-Editor`, which neither test assembly references. There is no test
  that a bake is correct, on any track.

None of that is fixable while the line is an array of scene objects. It is the wrong shape for
a five-track game.

## 3. Architecture

Five layers. Each is independently testable and knows nothing about the layer above it.

```
  TrackRuntimeData        (per track, baked ONCE, an asset)
    ├── centreline        arc-length-parameterised spline, ~60 knots
    ├── speedProfile      v_physical(s)   -- physical limit, car-agnostic
    ├── referenceLapTime  seconds         -- for tier pacing
    ├── width(s)          metres          -- track limits, overtaking room
    ├── sectors           3 x arc length
    └── startPose, gridSlots, kerbs, runoff
            │
            ▼
  TrackRuntime            (immutable runtime view, built once per race)
    · projects world position -> (s, lateralOffset)
    · gives continuous tangent / normal / curvature at any s
    · owns the arc-length LUT that makes s exact
            │
   ┌────────┴────────┬──────────────┬──────────────┐
   ▼                 ▼              ▼              ▼
 Driver (per car)  TrafficModel   Ghost/Sector   TrackLimits
   4 stages, LOD     (per car)     Timing         (per car)
```

### 3.1 `TrackRuntimeData` — the asset

A `ScriptableObject` (`TrackData`) holding everything a track contributes to gameplay. This is
the answer to "4–5 tracks": **a new track is a new asset plus a scene**, and nothing else.

Fields are *functions of arc length `s`*, sampled into fixed-resolution arrays:

| Field | Resolution | Why it is a function of `s` |
|---|---|---|
| `centreline` | ~60 control knots | Sparse; the spline carries the shape |
| `speedProfileKmh` | 1 per ~2 m | Dense; this is the physical speed limit |
| `widthMetres` | 1 per ~10 m | Varies; overtaking room and track limits |
| `sectorStarts` | 3 values | pdf2's three sectors |
| `referenceLapSeconds` | 1 float | tier pacing (§3.5) |

The asset is **car-agnostic**. It stores a *physical* limit — what the tyres can do — not what
this car does. A Gen1 rental car and a Gen3 unlock car then differ because `CarDefinition`
differs, which is exactly what pdf2's car-unlock progression wants. Baking a speed limit that
already assumed one car's downforce would break that.

### 3.2 The spline

Hand-rolled, **not** `com.unity.splines`. Two reasons, the second decisive:

1. `SplineFactory.CreateCatmullRom` builds a **Bezier** with auto-smoothed tangents
   `(next−prev)/√|next−prev|·0.5`. Unity's own source comments it: *"this is not a correct
   uniform catmull rom parameterization."*
2. That output is **G1** (tangent-continuous), not **G2**. Curvature is *discontinuous* at
   knots — so the 99 km/h profile step the handoff set out to eliminate would survive the
   package. Curvature continuity in Catmull-Rom comes specifically from non-uniform knot
   spacing (centripetal, α = 0.5; Yuksel et al., *On the curvature of Catmull-Rom curves*,
   2011), which that API does not expose.

(Note: the package *is* present in this project, pulled in transitively by Cinemachine 3.1.7,
though absent from `manifest.json`. Its presence is not a reason to use it.)

Requirements on the implementation:

- **Centripetal Catmull-Rom**, α = 0.5, closed-loop, uniform in the centripetal parameter.
- **Arc-length parameterised** by a LUT (dense sample, cumulative distance, binary search +
  local refinement). `s` is exact to <1 cm; this is what makes "how far to the corner" a real
  question rather than a waypoint count.
- **`GetPosition(s)`, `GetTangent(s)`, `GetNormal(s)`, `GetCurvature(s)`**, all continuous.
  Curvature from the analytic second derivative, so it is a smooth function of `s` by
  construction.
- Handles degenerate input honestly: a 4-point or collinear control polygon must not produce
  NaN tangents. Several existing tests build exactly such lines.

### 3.3 `TrackRuntime` — the projection

The layer that replaces index-as-coordinate. Its core operation is the inverse of everything
the old array made awkward:

```
  ProjectToArcLength(worldPosition) -> (s, lateralOffset)
  GetCurvatureAt(s), GetTangentAt(s), GetWidthAt(s)
  GetPositionAtDistance(s), GetLateralAt(s, offset)
```

`ProjectToArcLength` is what `AIDriverController.UpdateProgressIndex`, `LapTracker.BuildArcTable`
and `AILongitudinalPlanner.SolvePlannedSpeedKmh` each currently re-derive badly. It is also
what pdf2's track-limit detection needs (offset vs half-width) and what sector timing needs
(`s` vs sector bounds).

Implementation: coarse scan over the LUT then a local refine — no spatial index, because a lap
is a few thousand entries and this is O(log n).

### 3.4 The driver — four stages, per car, with LOD

Each AI car runs the full stack every physics tick. Chosen over a central race director
because pdf1 §3.3 asks for adaptive racing logic and clean defending, which are per-car
decisions; and chosen with LOD because pdf1 §4.1 makes AI CPU load a named deliverable.

| Stage | Job | Input | Output |
|---|---|---|---|
| **Perception** | What's around me, in track space | World query, OBB overlap | Cars ahead/behind, gaps, closing rate |
| **SpeedPlan** | Fastest speed at every `s` in the braking horizon | `v_physical(s)`, measured brake budget | `targetSpeed(s)` curve |
| **RaceCraft** | Am I attacking or defending? | Perception + track position | Lateral offset target, overtake intent |
| **Control** | Steer / throttle / brake | Aim point + target speed | Driver inputs |

**SpeedPlan** is where the horizon inversion gets fixed properly. The horizon is a *braking
distance* (`v²/2a`), which is **shortest exactly where the car is fastest** — the budget peaks
on a straight and the corner it must see is furthest away. Measured on Track_01: at 255 km/h
the plan looked 48 m ahead against ~200 m needed. Because `s` is now exact, the horizon is a
metre value with no waypoint-count translation, and it is sized against
`min(measured, pessimistic ceiling)`. Lengthening it can only pull the minimum down, so
sizing it generously is safe in the direction that matters.

**The plan is a ceiling, never an assignment.** Clamps from RaceCraft (traffic, contact,
track-limit recovery) apply *after* the planner, and the driver takes
`min(intent, plan)`. Making the planner sole authority is a known reverted mistake — it erased
the 40 km/h side-overlap floor.

**RaceCraft** is a real state machine, not a speed boost:

```
  Following ──pass available──▶ Committing ──side chosen──▶ Alongside ──▶ Clear
      ▲                            │                            │          │
      └────────── Abort ◀──────────┴────── no room / unsafe ─────────┘
```

Defending chooses a side from *track position and space* and never defends into the inside.
With a continuous normal at any `s`, lane offset stops being a per-waypoint lookup.

**LOD.** Distant cars run Perception + Control only, on a decimated schedule; they rejoin full
update as they come into range. Traffic behind the player still matters for the mirror and for
pack racing, so this is a budget decision, not a visibility one.

### 3.5 Difficulty tiers

`AIDifficultyProfile` becomes **a small multiplier set over the physical limit**, not a second
copy of the physics:

| Tier knob | Effect |
|---|---|
| `cornerSpeedScale` | Fraction of `v_physical` the driver will use in a corner |
| `brakeConfidenceScale` | Fraction of the measured brake budget it plans against |
| `aggression` | Overtake willingness and commit duration |
| `errorAllowance` | Target-speed tracking tolerance (models a human's imperfection) |

Because these are dimensionless, **the same three assets pace five tracks** — a new track
changes the baked profile, not the tier tuning. That is the property that makes track #2
cheap, and it is what §3.3's "competitive across tiers" actually requires.

`TrackPlacement._startFinishIndex` and `AIDriverController.CurrentWaypointIndex` both stop
existing as serialised integers; they become arc length, or transient state.

## 4. Data flow

**Authoring (editor, once per track):**
`tarmac mesh → cross-section cells → closed centreline → smooth → decimate to ~60 knots →
centripetal spline → curvature κ(s) → v = sqrt(a_lat/κ) → backward braking pass → sample to
1-per-2m → referenceLapSeconds`

The GDD's cross-section-cell insight is retained and generalised: the tarmac is a chain of
cross-section cells, so the centreline is read directly rather than inferred. This is what
made Track_01's line track the road closely.

**Runtime (per race):**
`TrackData → TrackRuntime (builds arc-length LUT once) → per car { Perception → SpeedPlan →
RaceCraft → Control → inputs }`

**Bake validation.** Because the baker becomes testable (it moves into an assembly tests can
reference), the bake asserts: closed loop, no NaN, lap length within an expected band, no
sample off the tarmac, sector coverage complete, curvature within a plausible band. A bad bake
fails loudly instead of producing a car that drives into a wall.

## 5. Error handling and failure modes

Failure modes worth naming, because a racing AI fails *quietly*:

| Failure | Detection | Response |
|---|---|---|
| Bake produced an open loop or NaN | Bake-time assertion | Refuse the bake; keep the previous asset |
| Car cannot find itself on the line | `ProjectToArcLength` finds no minimum | Hold last valid `s`, clamp recovery speed, log once |
| Car spun, facing backwards | Tangent vs velocity dot < 0 | Enter track-recovery: reduce speed, re-aim, reverse only if rear is clear |
| Car wedged by another | Speed below threshold while demanding throttle | Recovery FSM (already exists, ported) |
| Perception sees nothing at speed | Gap exceeds sensor reach | Fall back to leader-speed matching, never to full throttle |
| `TrackRuntime` missing entirely | Guard at driver init | Driver disables itself and logs; **never** drives blind |

The last row matters: a driver with no line must stop, not guess.

## 6. Testing

The existing 185 EditMode / 2 PlayMode tests are written against the array. This design's
migration strategy is explicit about that:

**Kept working by preserving the public surface.** `AIRacingLine`'s methods become thin
adapters over `TrackRuntime` (`GetPosition(i)` → sample `i`). ~15 test loops that walk
`0..Count-1` keep running, which is what buys room to land the change incrementally.

**Must be rewritten, because they assert properties of the array itself:**

| Test | Why it cannot survive |
|---|---|
| `SliceS2TrackContentTests.RacingLine_SpacingIsEvenAndTheLoopIsClosed` | Asserts uniformity of *waypoint spacing* — meaningless under a spline |
| `SliceS2TrackContentTests.RacingLine_TargetSpeedsArePlausible` | Reads `line.waypoints[i].targetSpeedKmh` directly |
| `AILongitudinalPlannerTests.MeanSegmentMetres = 11.33f` | Encodes the discretisation as a physical constant — the question the spline exists to delete |
| `AIRealTrackPlannerTests.IndependentCornerSpeedMs` | A **second** Menger-curvature implementation; left alone it becomes the yardstick the spline is measured by, and it would disagree for the right reason |
| PlayMode `WaypointsVisited >= 280` | Hardcoded against 300 |

**New tests are the real gate:**

1. **Curvature continuity** — the load-bearing one. Sample κ(s) densely; assert max
   |dκ/ds| is bounded. This is the property the whole redesign exists to obtain, and it is
   currently only *logged*.
2. **Speed-profile smoothness** — worst step between adjacent profile samples, asserted
   against a threshold, replacing today's log-only clause.
3. **Arc-length exactness** — `ProjectToArcLength` round-trips; `s` recovered from a point
   placed at known `s` matches to <1 cm; position is continuous across the start/finish seam.
4. **Tier pacing** — with three distinct tier assets on one line, lap times differ by a real
   margin and are ordered Easy > Medium > Hard. This is §3.3's acceptance criterion, currently
   unpassable.
5. **Bake correctness** — run the baker in a test on the real tarmac mesh; assert the loop
   closes, nothing leaves the road, lap length is in band.
6. **Second-track proof** — bake a second track's `TrackData` in a test from synthetic input
   and assert the same invariants hold. This is the "scales to 5 tracks" claim, tested rather
   than asserted in prose.
7. **Degenerate geometry** — 4-point, collinear, and duplicate-knot lines produce finite
   tangents and curvature. Several existing fixtures are exactly these.

**Test assembly change required:** the baker moves into a referenceable assembly (or gains a
runtime counterpart). This is why it is currently untestable.

## 7. Migration, in order

Each step is independently landable and independently verifiable.

| # | Step | Gate |
|---|---|---|
| 1 | Hand-rolled centripetal spline + `TrackRuntime` as a **new** assembly, no scene changes | New spline tests green; existing 185 untouched |
| 2 | Move the baker into a testable assembly; add bake validation + tests | Bake tests green on Track_01 |
| 3 | New `TrackData` asset type; bake Track_01 into it; compare against the old line | New asset's lap length and profile within a stated band of the old |
| 4 | `AIRacingLine` becomes an adapter over `TrackRuntime` | 185 EditMode + 2 PlayMode still green |
| 5 | `SpeedPlan` re-expressed over `s`; horizon becomes metres | Planner tests green; **the profile-step test now asserts** |
| 6 | `RaceCraft` state machine replaces the overtake boost | Craft tests green; no ramming |
| 7 | Tier multipliers; three distinct assets | **Tier-pacing test green** (unpassable today) |
| 8 | LOD tiers for distant cars | CPU budget met |
| 9 | Rewrite the array-shaped tests listed in §6 | Suite green on the new representation |
| 10 | Re-measure the ~44 km/h overspeed gap | Explained, or documented as open with the new baseline |

Step 4 is the hinge: before it, nothing existing has moved; after it, everything can.

**The ~44 km/h overspeed defect is re-measured at step 10, not before.** The handoff's own
note is right that it is measured against a substrate that step 3 replaces. Chasing it now
would be chasing an artefact of the old representation — the same class of mistake as
smoothing a discretisation artefact instead of removing the discretisation.

## 8. Open questions

1. **Wing setup coupling (pdf2).** "Incorrect wing choice significantly impacts lap times."
   Wings change downforce, hence grip, hence `v_physical`. Should `TrackData.speedProfile`
   carry a per-wing variant, or should the AI read live downforce from `CarDefinition` and
   derive the limit each race? The second is more correct (downforce is speed-dependent, so a
   baked variant goes stale at the wrong moment) but costs a re-derive per race.
2. **Track-limit penalty (pdf2).** "Drifting away from the line causes small speed reductions";
   vibration when all four tyres leave. Needs `width(s)` and the car's ground contact — the
   latter the AI does not currently have. Cheaper substitute: penalty on |lateralOffset| vs
   half-width, which is an approximation of the same rule.
3. **Sector times (pdf2, purple/green/yellow).** `LapTracker` explicitly defers this
   ("deliberately not here"). `s` vs sector bounds makes it straightforward — but it is
   player-facing UI, so it is its own piece of work rather than part of the AI core.
4. **Reference lap authoring.** §3.5 assumes a `referenceLapSeconds` per track. Either the AI
   drives a clean lap during the bake (self-consistent, always available) or a human sets it
   (better quality, needs a per-track pass). Recommend the former, with the latter as an
   override.
5. **Second track's provenance.** The design proves a second track can be *baked* in a test.
   Producing tracks 2–5 needs real tarmac meshes; nothing in this design generates them.