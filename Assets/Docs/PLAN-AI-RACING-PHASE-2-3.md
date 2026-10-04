# Plan — AI racing to pdf1 Phase 3

Written 2026-10-04, before any code change. Every claim below was read out of the source or
**measured** in the live editor. Nothing here is inferred from a symptom.

Reference: `pdf1.pdf` (the phased development programme). `pdf2.pdf` is the gameplay/feature
spec and is out of scope for this plan.

---

## 1. What the reference actually asks for

| Ref | Item | Requirement |
|---|---|---|
| **2.3** | AI Behaviour v1 | racing line system · corner speed adaptation · collision avoidance · basic overtaking · difficulty scaling presets |
| 2.3 AC | | follows racing line consistently · no aggressive ramming · competitive but fair overtakes |
| **3.3** | AI Behaviour Final | adaptive racing logic · clean defensive driving · slipstream (optional) · consistent lap time targeting |
| 3.3 AC | | behaves like confident F1 drivers · **no erratic braking** · competitive across difficulty tiers |

Phase 1 (1.1 physics, 1.2 steering, 1.3 camera) and 2.1/2.2 (steering assist, braking) are
already implemented and are **not** in scope — except where the AI depends on their output.
The vehicle physics the AI drives is real: custom raycast-wheel model, downforce, a
Pacejka-style traction curve, longitudinal + lateral weight transfer, brake bias, countersteer
assist. The physics is not the problem. The **driver** is.

---

## 2. Honest inventory — what is built

Built and working:

- Custom vehicle physics (`VehiclePhysicsCoordinator`, `RaycastWheel`, `TractionSystem`,
  `DownforceSystem`, `WeightTransfer`, `DrivetrainBrakeSystem`, `AdvancedSteeringAssist`) —
  Phase 1.1/1.2/2.1/2.2 satisfied.
- A baked racing line: **1 line, 300 waypoints, 3397.8 m loop, 11.17–11.33 m spacing**,
  with a genuinely smooth authored speed profile (worst single-waypoint step is **23.8 km/h**).
- AI perception on a dedicated `Traffic` layer (mask = `1 << 6`, verified) with forward, rear
  and two side corridors, leader speed and closing rate reported.
- A car-following law, side-overlap escape, stuck recovery, timed grid-launch exemption.
- An overtake path with lane offset, commit timer and a correctly-signed speed ease.
- Difficulty presets `Easy/Medium/Hard` driving speed, braking margin, corner confidence,
  reaction delay, overtake willingness and avoidance caution.
- EditMode suite **112/112 green** (measured this session).

Not built (Phase 3.3): **slipstream 0 hits, defensive driving 0 hits, lap-time targeting
0 hits.** These are not partially done; they do not exist.

---

## 3. Measured defects, ranked
## 3. Measured defects, ranked

### STOP — two inverted safety laws must be fixed before anything else

Both verified by reading the code directly. Both are *upstream* of every other item: a braking
planner (P1) cannot be evaluated while the emergency brake overrides its output at 0.85 and the
overtake branch raises the very clamp the planner depends on.

**S-A — the overtake branch RAISES the car-ahead speed clamp.** `AIDriverController.cs:755-759`

```csharp
float eased = Mathf.Lerp(cautionTarget, targetSpeedKmh, overtakeTrafficSlowdownScale);
cautionTarget = Mathf.Min(eased, targetSpeedKmh);
```

`Lerp(a, b, t)` with `t` in `[0,1]` always returns a value inside `[a, b]`. In traffic
`targetSpeedKmh > cautionTarget`, so `eased <= targetSpeedKmh` and **`Mathf.Min(eased, targetSpeedKmh)`
returns `eased` unchanged — the `Min` is a no-op.** The assignment then stores
`cautionTarget + (targetSpeedKmh - cautionTarget) * 0.4`, i.e. **up to 40% of the way back up to
the unblocked racing-line speed.**

Worked through on a Hard car (`speedMultiplier` 1.18) at 250 km/h, leader doing 36 km/h at 15 m:

```
cautionTarget (not overtaking)  = Lerp(36, 0, 0.586^2)          =  23.6 km/h
eased         (overtaking)      = Lerp(23.6, 283, 0.4)           = 114.4 km/h
cautionTarget                   = Min(114.4, 283)                = 114.4 km/h
```

A committed overtaker targets **114 km/h into a car doing 36 km/h** — 4.8x the caution the
identical car gets when it declines to pass. At 44 m/s of closing speed that is contact in ~0.34 s.

The guard that used to prevent this, `Mathf.Min(cautionTarget, eased)`, was removed as "a no-op".
It *was* a no-op — but in the safe direction. The comment at `:751-754` justifies the removal by
claiming "the cap that actually matters is the one at the speedTarget assignment below, which
already holds the result to targetSpeedKmh." That is `speedTarget = Mathf.Min(speedTarget,
cautionTarget)` at `:779`, which caps against the **already-raised** `cautionTarget` and therefore
cannot undo the raise.

*This is a regression.* The project memory records this exact defect as fixed; a later change
removed the guard and restored the bug. Treat the memory as stale.

**S-B — the emergency brake is a tautology above 40.7 km/h.** `:883-891` + `:842`

```csharp
float distance = emergencyMinGap + emergencyTimeHeadway * speedMs + emergencySpeedMargin * speedMs;
return Mathf.Min(distance, sensorReach);
```

With prefab values that is `min(6 + 2.3 * v_ms, 32)`. The cap binds above `26 / 2.3 = 11.3 m/s =
40.7 km/h`, so above that `emergencyDistance == 32 == perception.forwardDistance`. And
`FrontDistance` is a cast *travel* distance that is `<= forwardDistance`, so
`perception.FrontDistance < emergencyDistance` holds on essentially **every** `FrontBlocked` tick.
The distance test is vacuous.

The only surviving gate is `risky = !needsRiskCheck || perception.ClosingSpeedKmh >
emergencyClosingKmh` (`:839-840`), i.e. **a closing rate above 4 km/h.**

*Failure:* at 250 km/h with a car 30 m ahead doing 246 km/h — a 4 km/h closing rate — the AI
applies throttle 0 and brake 0.85. Any follower that is closing at all triggers it, every tick it
persists.

**Together these are the erratic-braking acceptance criterion failing**, and they explain each
other: the overtake branch raises the target, the car closes, closing speed exceeds 4 km/h, the
emergency brake fires, speed drops, closing goes negative, the brake releases, throttle returns —
a repeating full-throttle/full-brake transient for the duration of every pass.


### D1 — The AI sees one waypoint of speed profile and cannot brake in time. **(root cause)**

`AIDriverController.cs:268`

```csharp
LookaheadWaypointIndex = racingLine.WrapIndex(lookaheadSegmentIndex + 1);
```

The target speed is the speed of **the single next waypoint** (`:270-278`). `curvatureLookaheadSteps`
is used only for curvature, never for the speed target.

Braking from 240 km/h to 90 km/h at ~45 m/s² needs **≈42 m ≈ 3.8 waypoints**. The AI looks
**11.2 m**. It cannot see the corner until it is already inside the braking zone, so the only
way it can respond is to blow past the target and saturate. Every corner is a wall.

This is the mechanism behind "erratic braking" in the 3.3 acceptance criteria.

### D2 — Longitudinal control is a proportional speed-error controller with a coast band

`AIDriverController.cs:793-794`

```csharp
LastThrottleInput = Mathf.Clamp01( speedError / throttleFullErrorKmh);                    // /16
LastBrakeInput    = Mathf.Clamp01(-speedError / (brakeFullErrorKmh * brakingMargin));      // /42 * margin
```

No feed-forward, no integral, no brake-distance term. On Hard (`brakingMargin` 0.74) brake
saturates at 31 km/h of error, so any error in **[−31, +16] km/h produces zero throttle and
zero brake** — a ~47 km/h-wide coast band. The controller then oscillates between the two
rails. `throttleFullErrorKmh: 16` and `brakeFullErrorKmh: 42` are the *prefab* values.

### D3 — The AI's tuning lives in two places and they have silently diverged

AIDriverController declares **53** config fields. The prefab `AI_F1_Body.prefab` serialises
**30 of them with values that differ from the code defaults.** Unity deserialisation prefers
the prefab, so **every code-side tuning change from the last three sessions is inert in play
mode.**

Concretely, measured:

| field | code default | prefab (live) |
|---|---|---|
| `trackRecoverySpeedKmh` | **45** | **95** |
| `baseLookaheadDistance` | 12 | 8 |
| `lookaheadPerKmh` | 0.08 | 0.035 |
| `brakeFullErrorKmh` | 34 | 42 |
| `throttleFullErrorKmh` | 22 | 16 |
| `overtakeTrigger` | 0.35 | 0.18 |
| `laneVariationStrength` | 0.18 | 0.05 |
| `sideTrafficTurnThrottleScale` | 0.74 | 0.62 |
| `sideTrafficTurnBrake` | 0.12 | 0.18 |

`trackRecoverySpeedKmh` is the sharpest case: the code carries a tooltip explaining it "used
to be 95 km/h" and was changed to 45 — but the prefab still serialises **95**, so that fix is
**not in effect**. The last documented recovery fix is silently dead.

Only two fields escape this — `gridLaunchWindowSeconds` and `overlapEscapeMinSpeedKmh` — and
those are the only two because they were *added* after the prefab was last saved, so they fall
through to the code default. Fixes that add a field work; fixes that change a value do not.

### D4 — Three difficulty `.asset` files are decorative

`F1_AI_Easy/Medium/Hard.asset` are referenced by **no prefab and no scene**. The AI prefab has
`difficultyProfile: {fileID: 0}` and resolves instead from the enum via
`CreateRuntimeProfile(difficultyPreset)` (`AIDriverController.cs:213-217`), so difficulty *does*
work — from the enum, Hard(2) on the prefab. But anyone who tunes `F1_AI_Hard.asset` will see
nothing happen. Phase 2.3's "difficulty scaling presets" is half-real.

### D5 — Three of five waypoint fields are uniform constants

Measured over all 300 waypoints: `laneWidth` has **1 distinct value**, `brakingCaution` **1**,
`preferredLaneOffset01` **1** (zero). Only `targetSpeedKmh` carries real data, and 162 of 300
waypoints (54%) sit exactly on the 240 km/h ceiling. There is no per-corner strategic
information — no braking markers, no apex/lane preference — for adaptive or defensive logic
to build on.

### D6 — The AI is never told it was hit

There is no `OnCollision*` anywhere in the project. A shunt is invisible to the driver model
except as a sensor reading, so contact recovery is guesswork.

### D7 — Perception is a 2.6 m forward-only corridor

`forwardHalfExtents (1.3, 0.8, 0.8)`, `forwardDistance 32`. A car more than ~1.3 m off the nose
is invisible, so the whole following law switches off. Widening it naively would make a car
legitimately alongside register as "ahead" and brake mid-overtake — this needs a shape change,
not a tuning change.

### D8 — The racing line reference is resolved by a scene-wide search

Prefab `racingLine: {fileID: 0}`; `AIDriverController.cs:946` does
`FindAnyObjectByType<AIRacingLine>()`. A proper reference exists on `TrackDefinition`
(`_racingLineReference`) and is not used here.

### D9 — There are **three** override layers, not two — verified

D3 described code-vs-prefab. A physics audit found the third: **`VehiclePhysicsProfile` assets
are applied in `Awake` and beat the prefab.**

```csharp
// VehiclePhysicsCoordinator.cs:152-155  Awake
ResolveReferences();
ApplyProfile(physicsProfile);        // -> advancedSteeringAssist.ApplyProfile(...)  :348
```

```csharp
// AdvancedSteeringAssist.cs:108
assistLevel = profile.assistLevel;
```

Measured:

| | value |
|---|---|
| `AI_F1_Body.prefab:772 assistLevel` | **1** (Medium) — the authored intent |
| `F1_AI_Physics.asset:107 assistLevel` | **2** (High) — wins at runtime |

So the AI runs **1.35× steering correction**, not the Medium the prefab asks for. Any P0 that
only reconciles code-vs-prefab leaves this live.

### D10 — There is no longitudinal tyre model, so there is no traction limit to respect

`RaycastWheel.cs:132-135` (verified)

```csharp
// Longitudinal slip needs wheel angular velocity. Body velocity alone turns normal
// forward motion into a fake braking slip, so drivetrain/brakes own this axis for now.
float longitudinalSlip = 0f;
LocalSlipVector = new Vector2(lateralSlipAngle, longitudinalSlip);
```

`LocalSlipVector.y ≡ 0` ⇒ `TractionSystem.cs:142` evaluates `Pacejka(0,…) = 0` ⇒
`LongGripCoeff` ≡ 0 on all four wheels, forever. The friction-circle ellipse
(`TractionSystem.cs:152-165`) therefore degenerates to a lateral-only magnitude cap, and the
drivetrain's drive/brake force is clamped against `NormalForce × tireForceLimitMultiplier`
(`DrivetrainBrakeSystem.cs:252`) with **no reference to lateral usage** — so the two axes do not
share a budget.

**Consequence for the AI:** there is no wheelspin signal and no longitudinal traction ceiling to
respect. Any braking planner must derive its deceleration budget from **measured brake
capability**, not from a tyre traction limit, because one does not exist.

### D11 — The physics runs at 100 Hz, not Unity's 50 Hz — verified

`ProjectSettings/TimeManager.asset` — `fixedDeltaTime = 1411199 / 141120000 = `**`0.01 s`**.
Unchanged since the initial commit; a stock Unity project on the same disk is 0.02.

Every per-second tuning constant therefore runs at twice its nominal rate
(`throttleSpoolSpeed 3.5`, `brakeSpoolSpeed 6` reach target in half the wall-clock seconds), and
the AI's decision rate is 100 Hz. The planner's deceleration budget must be expressed per-second
and sampled per-step consistently with this, not against the Unity default.

### D12 — The AI reads exactly one number from the vehicle

`AIDriverController` reads **only** `coordinator.SpeedKmh` (`:264`). A grep for
`traction.` / `downforce.` / `weightTransfer.` in the AI returns **zero hits**.

Everything a corner-speed model needs is already computed every frame and simply not read:

- `traction.GetStats().speedBandMult` — the exact multiplier applied to peak grip, derived from
  the same speed the AI already has
- `traction.GripUtilisation[4]` — per-wheel grip usage
- `wheels[i].NormalForce` — newtons, **including downforce**, and the same value the drivetrain
  is clamped against, so it is the true available-grip figure
- `AverageFrontSlipAngle` / `AverageRearSlipAngle`, `TractionLossState`,
  `FrontBrakeLockup` / `RearBrakeLockup`, `weightTransfer.FrontAxleLoad` / `RearAxleLoad`

The AI is not forced to guess grip. It is forced to ignore grip.

---

### D13 — No test drives an AI car along the real racing line

All 39 AI behaviour tests build a **synthetic** line from primitive GameObjects
(`CreateAIDriverTestRig` hand-places `AIRacingWaypoint`s). The `Track_01` tests are geometry
arithmetic on an *unopened* scene. `SliceS2TrackContentTests.RacingLine_SpacingIsEvenAndTheLoopIsClosed`
only bounds `longest < avg*3` — loose enough that a line twice as coarse as intended passes.

**This is the risk to the whole plan.** P1 replaces the braking planner. A planner verified only on
synthetic geometry can be correct on a straight and fall apart on the real hairpin, with all 39
tests still green. The baked line is 300 waypoints at 11.17-11.33 m over a 3397.8 m loop with
peaks as low as 90 km/h, and its apex chords are exactly where an 11 m lookahead is wrong.

### D14 — AI lap tracking survives only on a hard-coded layer number and a scene-load ordering nobody controls

- `PlayerCarSpawner.cs:144-149` sets `tracker.TrackLayer` **for the player only**. Nothing does it
  for AI cars; they rely on `_trackLayer: 3` serialized in `AI_F1_Body.prefab:933`.
- `BindTrack` has exactly one caller — `PlayerCarSpawner.cs:184`. AI trackers resolve their track
  in `Awake` via `FindAnyObjectByType<TrackPlacement>()`, which works today only because
  `RaceSceneController.BringUpRace` loads track content before spawning the grid.

**Failure:** renumber the layers in TagManager, or spawn the grid before the track exists, and
every AI's `IsOnTrackSurface()` misses the road — lap distance stays 0 forever, the field ranks
level, and the race HUD position is meaningless. Silent, no console output. This is the exact
failure `LapTracker.cs:391-407` documents; the fix reached one spawn path and not the other.

P5 (lap-time targeting) is not worth starting before this is fixed.

### D15 — `AIRacingLine.RefreshWaypoints()` overwrites the serialized array from hierarchy order

`AIRacingLine.cs:20-23`, called from `Awake` and `OnValidate`. Reorder the `WP_*` children, or
re-bake and get a different order, and the 300-entry array is silently replaced —
`TrackPlacement._startFinishIndex = 2` then points at a different physical place, moving
start/finish, the grid basis and sector progress with no error. Currently latent: hierarchy order
and array order were verified equal.

### D16 — `Pit_Road`'s collider is on layer 0, not `Ground`

AI wheels cast on `groundLayers = 8` and `LapTracker` on `_trackLayer = 3`; `Pit_Road` is on
layer 0. A car that ends up on the pit road finds no ground, drops through, and freezes its lap
distance. Low severity — the racing line never enters the pit road — but reachable after a shunt
toward the pits.

### Further confirmed defects (verified, lower priority than S-A/S-B)

| # | Defect | Evidence |
|---|---|---|
| a | **The AI is blind to all static geometry.** Walls, kerbs, guardrail and the track edge are not on `Traffic` and are never cast. The only off-track detection is `LastLateralError > 16 + 1.25 = 17.25 m`, against a `laneWidth` that is the literal `16f` in `TrackRacingLineBaker.cs:908`, **never measured from the tarmac mesh** | mask is `1<<6`; `:449`; baker `:908` |
| b | **`reactionDelay` is declared, ranged, written by all three preset branches, present in all three `.asset` files, and read by nothing.** An entire advertised difficulty axis has zero effect | `AIDifficultyProfile.cs:17` |
| c | **`avoidanceCaution` and `blockedTargetSpeedKmh` are unreachable in the shipped config.** Both are read only inside the `!LeaderIsMovable` branch, which cannot be taken when the mask is `Traffic`-only and every `Traffic` object is a Rigidbody car | `AIDifficultyProfile.cs:19`; `AIDriverController.cs:36,726` |
| d | **Curvature is under-scaled, so corner adaptation is inert below a hairpin.** `InverseLerp(8deg, 90deg, angle)` over `curvatureLookaheadSteps=3` (~32 m) reads ~0.13 at a 100 m radius and ~0.35 at 50 m. `packCornerCurvatureThreshold` = 0.38 needs ~47 m before it engages. In a 50 m corner `cornerScale` moves 2.4% on a Hard car | `AIRacingLine.cs:194`; prefab `:718` |
| e | **`IsOvertaking` is recomputed every tick and the commit timer guards only the *side*.** Reset to false at `:472`, re-set at `:503` from a condition sampled every 3rd fixed tick. The speed target swings between the raised and follow values on a 3-tick cadence | `:472,491,503`; `fixedFrameStride 3` |
| f | **Curvature is sampled at `CurrentWaypointIndex` while the target speed comes from `LookaheadWaypointIndex`** — corner confidence scales a waypoint ~17 m ahead using the geometry where the car currently is | `:274` vs `:270` |
| g | **The side-overlap escape RAISES `cautionTarget` to 40 km/h while the car-ahead case lowers it.** Side-by-side raises the target; car ahead lowers it | `:776-777` |
| h | **`ApplyPreset(Easy)` contradicts `F1_AI_Easy.asset`** — code writes `0.78 / 1.45 / 0.18`, the asset holds `0.92 / 1.18 / 0.35`. Two disagreeing sources for the same preset | `AIDifficultyProfile.cs:44-50` vs the asset |
| i | **Sensor unit mismatch.** `LeftDistance`/`RightDistance` are `BoxCastNonAlloc` *travel* distances and the box already extends `sideHalfExtents.x = 1.15 m` before the cast starts, so `< sideOverlapEscapeMetres (1.8)` is really a ~2.95 m true gap, not the 1.8 m the tooltip claims | `AIPerceptionSensor.cs:209`; prefab `:644` |
| j | **Dead code**, all confirmed zero-read: `private LapTracker _tracker` (its doc comment claims a role the code does not have — the grid exemption is driven by `Time.timeSinceLevelLoad`), `LastUnblockedTargetSpeedKmh`, `AIPerceptionSensor.ClosestObstacle`, `AIRacingLine.GetLookaheadIndex`, and an orphan prefab field `gridLaunchDistanceMetres: 60` with no member behind it | `:207/227`, `:183`, `AIRacingLine.cs:62`, prefab `:724` |


### Confirmed NOT defects (checked, so they are not re-investigated)

- **Racing-line seam and wrap indexing** — correct at every call site. `WrapIndex` is
  double-modulo, `GetWaypoint`/`GetPosition` wrap, the `WP_299 -> WP_000` gap is 11.33 m
  (identical to the mean), and `LapTracker.BuildArcTable` adds the closing segment explicitly.
- **Grid spawn double-lift** — fixed. `VehicleGroundSnap` measures the raycast-suspension
  equilibrium, not the collider, and writes both `transform.position` and `body.position` so it
  survives `SyncTransforms`.
- **AI perception seeing the road** — fixed. Mask is `m_Bits: 64` = Traffic only, asserted by
  `AIPerceptionLayerTests`, and `MainTrack_Object` is correctly on `Ground`.
- **Overtake easing undoing the caution** — fixed at `AIDriverController.cs:755-759`.
- **`PlayerCarSpawner.PutOnTrafficLayer`** puts the player on `Traffic` at spawn, so AI see it.


## 4. What I propose to build

Sequenced so that each phase is verifiable on its own. **P0 first** — nothing else can be
judged until the AI can brake for a corner and the tuning it is judged on is the tuning we
think we are running.

### P0 — One source of truth for AI tuning *(fixes D3, D4, D8, D9)*

**Decision (2026-10-04, developer): a ScriptableObject is the single source of truth.** Not
hardcoded script values, not one-off prefab overrides.

- New **`AIDriverProfile : ScriptableObject`** carrying the ~53 AI behaviour knobs that today
  live as public fields on `AIDriverController`. Three assets: Easy / Medium / Hard.
- `AIDriverController` keeps **one** serialized reference (`driverProfile`) and reads every
  value from it. The 53 public fields come off the component entirely, which is what makes the
  prefab structurally incapable of overriding them.
- The existing orphan `F1_AI_Easy/Medium/Hard.asset` (`AIDifficultyProfile`) are folded in —
  they are already the three tiers, they are just referenced by nothing. `ApplyPreset` becomes
  an authoring helper that generates the three profiles rather than a runtime path.
- `AIDriverProfile` also carries the **physics-relevant budget** the planner needs, so D10 is
  handled in data: `brakeDecelBudget`, `brakeResponseTime`, `tractionCeilingHint`.
- **D9 specifically:** add a test asserting that a prefab's `physicsProfile` does not carry
  values that silently contradict the prefab's own component settings — that is the class of
  bug where `assistLevel` reads 1 in the prefab and 2 at runtime.
- Delete `FindAnyObjectByType<AIRacingLine>()`; assign `racingLine` from `TrackDefinition`.

**Verify:** EditMode green; a test asserting `AIDriverController` exposes no tunable public
fields; a test that the three tiers produce measurably different profiles; live telemetry
matches the asset values.

### P0b — Decouple `CarDefinition` from AI physics

**Status correction (2026-10-04): the *wiring* is already fixed.** Commit `c82b3f5` flipped all
six `CarDefinition` assets from `F1_AI_Physics` to `F1_Player_Physics` — verified by GUID:

```
_basePhysicsProfile: guid c9ba836a...  =  Assets/Profiles/F1_Player_Physics.asset   (all six)
F1_AI_Physics (b2d1af3f...)  referrers:  AI_F1_Body.prefab:577   (exactly one)
```

An earlier draft of this plan, and the memory note `f1-ai-physics-profile-mis-wiring`, were both
stale on this. `SESSION-FIX-AI-AND-CONTROLS.md` §2A is stale on the same point.

**Unintended consequence to be aware of:** the player's car now drives on `F1_Player_Physics` as
authored — `motorForce` 98000 (was 128000), `throttleSpoolSpeed` 1.35 (was 3.5),
`rearInstabilityStrength` 0.06 (was 0.36), `maxSteerAngle` 16 (was 17.5). **The player's handling
changed at that commit.** Nobody appears to have checked whether that is acceptable.

What the developer's directive still asks for, and is *not* yet done:

- **`CarDefinition` does not provide read-only limits.** `CarDefinition.BasePhysicsProfile`
  (`CarDefinition.cs:36`) hands over the entire mutable `VehiclePhysicsProfile`, and
  `CreateProfileWithWing` (`:106-118`) deep-clones it, swapping only the aero. The car
  definition is currently a mutable physics profile in disguise.
- **`F1_AI_Physics` is still a full physics profile** (7.7 KB) carrying drivetrain, downforce,
  steering, traction and brake values that a player's profile also owns — so it *can* express a
  player-facing value even though nothing references it for a player today.
- **`CarDefinition.CarPrefab` is dead and points every car at the AI prefab.** All six assets
  resolve `_carPrefab` to `AI_F1_Body.prefab`, and no code reads `CarPrefab`. The first person who
  wires "spawn the selected car" to it hands the **player** a body with `UseExternalInput: 1` and
  the AI profile already applied — an uncontrollable car.

Target shape:

- `CarDefinition` exposes **read-only physical limits** — mass, wheelbase, track width, downforce
  coefficient, brake force, motor force, reference load. Properties of the *car*, not a mutable
  profile handed to a clone path.
- `F1_AI_Physics` is reduced to **AI-only concerns** — trajectory following, brake planning,
  virtual steering aid tuning — applied additively to AI cars, and structurally unable to express
  a value a player's profile owns.
- `CarPrefab` is repointed at `F1_Body.prefab` or deleted.

**Verify:** a test asserting no `CarDefinition` references an AI profile — this would have caught
the original defect, since the existing
`GameFlowPhase2GateTests.Registry_EveryCarHasAPhysicsProfileAndBothWings` asserts only non-null
and so passed the whole time the player was on the AI profile; a test asserting the AI overlay
cannot write a player profile's fields; a test asserting `CarPrefab` never resolves to an AI
body.

### P0c — Make AI lap tracking explicit *(fixes D14 — do this before P5)*

Small, and it de-risks everything downstream of it.

- `RaceGridManager` sets `tracker.TrackLayer` and calls `BindTrack` for every AI car it spawns,
  mirroring what `PlayerCarSpawner` already does for the player. Nothing should depend on a
  number serialized into a prefab or on `FindAnyObjectByType` finding the track in `Awake`.
- **Verify:** a test that spawns the grid and asserts every AI tracker reports a bound track and
  a non-zero `TrackLayer`, and that lap distance advances. Write the test first and watch it fail
  against the current wiring — per the project's own note, a fix that passes by hitting the thing
  under test is not a fix.

### P0d — Fix the two inverted safety laws *(S-A, S-B — do this FIRST)*

Two surgical fixes, both with a regression test each, before any planner work.

- **S-A.** Make the overtake ease genuinely bounded. The correct shape is to relax the clamp
  toward the target the car is chasing **and never above the safe following speed** — i.e. compute
  the follow target, ease toward it, and floor the result at that follow target, not at the
  unblocked racing-line speed. Do not reintroduce a bare `Min(cautionTarget, eased)`, which is
  a no-op in the safe direction but leaves the documented "eases the clamp" behaviour unimplemented.
  Add an EditMode test: with a leader at 36 km/h at 15 m, `LastTargetSpeedKmh` while overtaking
  must be `<=` the value it holds when declining to pass, and must stay below the leader's speed
  plus a bounded allowance.
- **S-B.** The emergency trigger must not saturate at the sensor reach. Either raise the
  `Mathf.Min(distance, sensorReach)` ceiling so the time-headway term stays live at speed, or
  bound the cap by the *follow* distance rather than the sensor reach, so that above ~40 km/h the
  trigger still tightens with closing speed. Add an EditMode test asserting
  `GetEmergencyDistanceMetres` is strictly increasing with speed across the whole speed range
  rather than flat above 40 km/h.

**Why first:** P1's braking planner writes `LastBrakeInput`, which `:846` then overrides with 0.85
whenever S-B fires, and it plans against a speed target which S-A can raise underneath it. A planner
tuned while these are live is being measured through two broken safety nets.


### P1 — Longitudinal speed planner *(fixes D1, D2, D10, D12 — the core of 2.3 + 3.3)*

Replace the reactive P-controller with a real speed profile the car can actually follow.

1. **Backward-pass speed profile.** For each waypoint within a braking horizon, compute the
   fastest speed the car may carry there given its braking budget:
   `v_allowed² ≤ v_next² + 2·a_brake·d`. Take the **minimum** over the horizon — never a single
   waypoint (D1).
2. **Horizon from braking distance, not waypoint count**, so it is long at 240 km/h and short in
   slow corners.
3. **`a_brake` must be measured, not assumed.** D10/D12: per-wheel force is capped at
   `NormalForce × tireForceLimitMultiplier`, and `NormalForce` carries an unconditional `mg/4`.
   So derive the budget from the car's own `NormalForce`, brake efficiency and mass, sampled per
   step at the **100 Hz** timestep (D11). Add a learning term from observed deceleration so it
   self-corrects for a wet line or a downforce-limited car.
4. **Feed the planner grip** from `traction.GetStats().speedBandMult` and `GripUtilisation`
   (D12) instead of letting it be a pure speed table.
5. **Replace the P-controller** with feed-forward + trim: throttle from the *planned* target,
   brake ramped by remaining braking distance, and a narrow proportional trim only to correct
   residual error. This removes the 47 km/h coast band.

**Verify:**
- (a) EditMode tests on the profile maths — monotonic non-increasing speed into a corner,
  respects `a_brake`, never exceeds the authored waypoint speed.
- (b) **A new PlayMode test that drives a car along the real baked `Track_01` line** (D13). This
  is the gate that does not exist today and the one P1 most needs. It must assert the car
  completes a lap without exceeding the authored target speed by more than a tolerance, and that
  brake input does not chatter — on the actual geometry, not a synthetic rig.
- (c) A **live race on Track_01** where the telemetry trace of `LastThrottleInput` /
  `LastBrakeInput` shows one braking event per corner instead of chatter. Per project practice,
  visible AI work is not done until it has been watched.

### P2 — Overtaking and race craft *(completes 2.3)*

- Replace the `overtakeTrigger` / `sideTrafficLaneHoldBuffer` instinct values with explicit
  state: `Following → Committing → Alongside → Clear → Complete/Abort`, with an abort that
  actually returns the car behind the leader rather than merely dropping the lane offset.
- Make the side-traffic guard *opportunistic*: it currently forbids lane changes toward a
  side car, which is right until contact, and the overlap escape is a bolt-on for the contact
  case. One rule that handles both is better than two that disagree.
- `overtakeWidth` is per-waypoint data and is uniform (D5) — give the bake a real value.

**Verify:** head-to-head runs at each difficulty; no contact at a defined pass distance; every
pass completes or aborts, none are abandoned mid-manoeuvre.

### P3 — Defensive driving *(3.3, currently 0 hits)*

When a leader is caught, choose a side from track position and space, defend once, yield if the
challenger has a materially better exit. Must never defend into the inside of a corner.

**Verify:** EditMode state-machine tests; a live race where the leader yields when it has no
line, and defends when it does.

### P4 — Difficulty presets that are real *(closes 2.3 properly)*

The three tiers exist as assets from P0 but are not yet differentiated enough to matter.
`AIDifficultyProfile` currently cannot reach the steering assist at all — `applyPreset` touches
six numbers and none of them is the assist level, so Easy/Medium/Hard cars differ only in target
speed and reaction delay while all running High assist (D9). Fold assist level into
`AIDriverProfile` and make the tiers differ in it.

Assert it: an EditMode test that paces the three profiles on the same racing line and fails if
their lap targets are within a few percent of each other. "Competitive across difficulty tiers"
is an acceptance criterion, so it should be a test, not an opinion.

### P5 — Lap time targeting *(3.3)*

Give each AI a target lap time derived from its difficulty, and a small controller that trims
throttle within the safe envelope to hit it. This is also the natural place for rubber-banding
to go, if it is wanted.

**Verify:** measured lap-time spread across the field and across presets.

### P6 — Collision feedback *(D6)*

`OnCollisionEnter` → a short "contact" state on the AI that unlocks the escape for a bounded
time. Cheap, and removes the last unexplained behaviour.

**Slipstream is deferred** (decided 2026-10-04). It is the one Phase 3.3 item with no existing
groundwork — it is a new mechanic rather than a fix, and it should not compete for attention
with the braking planner. Noted here so it is not lost.

---

## 5. Explicitly not in this plan

- Vehicle physics retuning (Phase 1.1/3.2). The AI must drive the physics we have.
- Camera, sectors, ghost, ERS, track limits — other unbuilt Phase 3 items, separate sessions.
- The `CarDefinition` → `F1_AI_Physics` mis-wiring. It affects the **player's** car, not the
  AI, and changing it alters how the car feels. Flagged, not touched.

---

## 6. Decisions taken 2026-10-04

| Question | Decision |
|---|---|
| Where AI tuning lives | **A ScriptableObject** (`AIDriverProfile`, Easy/Medium/Hard). Not script defaults, not prefab overrides. P0. |
| CarDefinition mis-wiring | **Decouple.** `CarDefinition` = read-only physical limits; `F1_AI_Physics` becomes an AI-only, additive overlay that never alters player-facing values. P0b. |
| P0b scope | **Decouple.** The asset *wiring* was already fixed in `c82b3f5`. What remains: make `CarDefinition` expose read-only limits, strip `F1_AI_Physics` to AI-only concerns, fix the dead `CarPrefab`. |
| P0b and player feel | **Moot — superseded by the above.** The player already drives `F1_Player_Physics` as authored. The "preserve felt behaviour" choice was made on a premise that turned out to be false. The handling change it was protecting against has *already* happened; whether the new numbers are acceptable is an open question, not something this work decides. |
## 7. Found by the physics audit but deliberately out of scope

Real defects, verified, in the **vehicle** rather than the AI. Listed so they are not lost; a
separate ticket should own them. Decision 2026-10-04: **not** in this work.

- **Stale one-frame reads in `VehiclePhysicsCoordinator.FixedUpdate`.** `RefreshState` reads
  `LocalSlipVector` before `wheels[i].Simulate` writes it (`:221` vs `:228`), so
  `AdvancedSteeringAssist` corrects against last frame's slip. `AdvancedBrakingSystem` reads
  axle loads written seven lines later, and those loads are themselves missing the current
  frame's downforce — so lockup prediction is wrong by roughly the amount downforce is growing.
- **`WeightTransfer` smooths on `Time.deltaTime` inside `FixedUpdate`** (`WeightTransfer.cs:145`)
  while every comparable smoother passes `Time.fixedDeltaTime`. Weight-transfer lag is therefore
  a function of render framerate: the same car transfers weight differently at 60 Hz and 240 Hz.
- **`centerOfMass` is written after every force that depends on it** (`:238`, before
  `FlushForces` at `:240`), so all drive/brake/tyre forces resolve torque about the previous
  frame's CoM — and the lag is largest exactly when transfer is largest.
- **Downforce is flat above 70 m/s** (`fullDownforceSpeed`), and the game runs to ~282 km/h, so
  there is no downforce growth across the top ~25% of the speed range.
- **Below 1 m/s the tyre model is replaced by isotropic velocity damping** with a magic `0.3f`
  (`TractionSystem.cs:124-131`). `launchStandingSpeedKmh = 5` means every grid start lives
  inside that branch.
- **`QueueForceAtPosition` silently drops forces** past 64 slots (`VehiclePhysicsCoordinator.cs:255`)
  — no warning, no counter. Not reached today (~19 per step), but any future system that queues
  more loses grip with nothing in the console.

Each of these affects AI *behaviour* indirectly. None is an AI defect, and fixing them would
change how every car drives — which is the thing the project has been careful to protect.

---

## 8. Implementation log

Started 2026-10-04 from commit `c82b3f5`. EditMode baseline **112/112 green**.

> **Session note (2026-10-04, later).** The first implementation session was killed by a
> daily rate limit partway through. It had written **every test** for P0c and P0d but
> **not one line of the production fixes** — `AIDriverController.cs` and
> `RaceGridManager.cs` were byte-identical to `HEAD` afterwards even though their
> modification times looked minutes old. The "Landed" list below was written from intent,
> not from the tree. Everything in it is now verified by a green test run.

### Landed and verified

**P0b (safe parts) — `CarDefinition` isolation**
- All six `CarDefinition` assets pointed `_carPrefab` at **`AI_F1_Body.prefab`**, the AI body,
  which carries `UseExternalInput: 1` and the AI physics profile already applied. The field is
  read by nothing, so nothing broke — but the first person to wire "spawn the selected car" to it
  would have handed the player an uncontrollable car. Repointed at `F1_Body.prefab`
  (root GameObject fileID resolved from the prefab, not assumed).
- Documented the invariant where it can actually be broken: `CreateProfileWithWing` deep-copies
  the whole base profile, so every field the base carries reaches the player's car.
- `Assets/Tests/EditMode/CarDefinitionIsolationTests.cs` — six tests, green. The load-bearing
  one is `NoCarDefinitionUsesAnAiPhysicsProfile`, which asserts *which* profile; the existing
  `GameFlowPhase2GateTests.Registry_EveryCarHasAPhysicsProfileAndBothWings` asserts only
  non-null, which is why the original mis-wiring passed every gate it had.

**P0d — the two inverted safety laws.** Both fixes applied; all six
`AIDriverSafetyLawTests` green.

- **S-A.** The overtake branch now eases toward `followCeilingKmh` — the speed the following
  law itself chose — and is floored by it, instead of blending toward `targetSpeedKmh` and
  wrapping the result in a `Min` that could never bind. Worked example on the rig (leader
  36 km/h, 15 m gap, Hard car): the committed overtaker went from **136.3 km/h to 28.6 km/h**,
  against a ceiling of 56.0 (leader + `followMaxClosingKmh`), and it still sits above the
  23.6 km/h a car that declined to pass holds — so the ease survives and is bounded, rather than
  one or the other.
- The guard is `followCeilingKmh > cautionTarget`, and it is load-bearing in both directions:
  a leader reading zero puts the ceiling at zero too, so there is nothing to ease toward and the
  caution is left exactly where the follow law put it.
- **S-B.** `GetEmergencyDistanceMetres` no longer caps at the sensor's reach, so the trigger
  grows all the way up (31.6 m at 40 km/h, 197.7 m at 300 km/h) instead of being a constant
  above ~41 km/h. A trigger that outgrows the sensor cannot be "unreached", so the caller pairs
  it with two terms: **contact** (inside `emergencyMinGap`, whatever the closing rate) and
  **imminent** (closing above the deadband AND time-to-contact under `emergencyTimeHeadway`).
  A closing *rate* on its own is not enough: 6 km/h sounds alarming until you notice the gap is
  30 m and lasts eighteen seconds.

**P0c — AI lap tracking is explicit.** `RaceGridManager.ConfigureCar` now binds every spawned
car's `LapTracker` to the grid's `TrackPlacement` and sets `TrackLayer` from the surface
collider's actual layer. All five `AITrackBindingTests` green.
- The binding used to come from `LapTracker.Awake`'s own `FindAnyObjectByType<TrackPlacement>()`,
  which works only because `RaceSceneController` loads track content before spawning the grid —
  an ordering nobody owns. The layer came from a number serialised into `AI_F1_Body.prefab`,
  which was right only by coincidence of layer numbering.
- When no track is found the car is **skipped, not `BindTrack(null)`** — `BindTrack` assigns
  unconditionally, so calling it with null would erase a binding the car inherited.
  `AITrackBindingTests.GridSpawn_WithNoTrackPresent_LeavesATrackerThatAlreadyResolvedOneAlone`
  exists to catch exactly that.
- `ResolveReferences` now asks the `TrackPlacement` for the racing line before falling back to a
  scene-wide search, so the line a car drives and the track it measures are the same track.

**P1 (solver only, not yet wired) — `AISpeedProfile`**
- `Assets/Scripts/Brain/AISpeedProfile.cs` — the braking-horizon arithmetic as a pure static
  class, so the maths that decides whether a car brakes in time is testable directly instead of
  inferred from a race.
  - `SolveAllowedSpeedKmh` — the speed a car may carry now such that `v^2 <= v_limit^2 + 2*a*d`
    holds at every point ahead. Rejects negative braking capability rather than reading it as
    thrust.
  - `SolveForwardProfileKmh` — whole-line backward pass over the loop.
  - `MeasureBrakeDecel` — learns the car's real braking capability from observed deceleration,
    because there is no longitudinal tyre model to read it from (`RaycastWheel.cs:132-135`
    hard-zeroes longitudinal slip, so `LongGripCoeff` is 0 on every wheel forever).
- `Assets/Tests/EditMode/AISpeedProfileTests.cs` — 17 tests, green.
- **Two of those tests were wrong and have been corrected.** Both were mine, and both failed
  against a solver that was right:
  - `TheNearestConstraintWinsOnlyWhenItIsTheTightest` used 240 km/h at 10 m as its "near loose"
    constraint. Braking at 25 m/s^2 barely changes speed over a few metres, so that constraint
    permits 253 km/h — *tighter* than the 90 km/h corner 300 m away (450 km/h). The fixture
    contradicted the test's own name. Replaced with 300 km/h at 10 m against 90 km/h at 60 m,
    where the distant tight corner genuinely is the binding one.
  - `AddingAConstraintCanNeverRaiseTheAllowedSpeed` solved a fresh two-constraint problem each
    pass and asserted the *sequence* of answers was non-increasing — a claim about the generated
    fixture, not about the solver, since the previous pass's constraint was discarded. It now
    appends to one growing horizon, so the answer really is a minimum over a growing set.
- `EachWaypointIsConstrainedByTheGapAheadOfItNotTheGapBehindIt` exists because a pass that reads
  the segment on the wrong side of each point looks correct on Track_01, where every segment is
  11.2-11.33 m.

**P0 — `AIDriverProfile` is wired and is now the only place AI tuning is authored.** EditMode
**159/159 green** (147 prior + 12 new).
- The 52 `[Range] public` fields are gone from `AIDriverController`. They are now **properties**
  that read through `driverProfile`, which is a property rather than a field precisely because
  Unity cannot serialise an override for one. That is the structural part of the fix: the
  prefab is no longer *able* to shadow the tuning, rather than merely being asked not to.
- Each property keeps a **per-instance override** behind a nullable backing field. Reading
  returns the override when one was set and the profile's value otherwise. This is not
  convenience — roughly 50 call sites in `F1FoundationPlayModeTests` and two in
  `RaceGridManager` assign these fields directly, and a shared asset a test could mutate would
  leak one test's scenario into every other car in the session and dirty the asset on disk.
- `AIDriverProfile.CreateRuntimeProfile()` mirrors `AIDifficultyProfile.CreateRuntimeProfile`,
  and the controller caches the fallback in `_runtimeTuning` so an unassigned car costs one
  `CreateInstance` for the session rather than one per read at 100 Hz.
- **All 50 serialised overrides were stripped from `AI_F1_Body.prefab`** and replaced with a
  single `driverProfile` reference to `AIDriver_Hard` — which matches the `difficultyPreset: 2`
  the prefab already carried. Without that strip the migration would have been inert: Unity
  would have kept deserialising the old keys. Verified: **50 of the 52 prefab values matched
  the profile defaults already**, so the cutover changes no driving behaviour. The one
  deliberate exception is `trackRecoverySpeedKmh`, 95 → 45, the value the field's own tooltip
  identifies as the bug.
- `RaceGridManager` gained `defaultDriverProfile` and a per-entry `driverProfile`, assigned in
  `ConfigureCar` with the same null-coalescing idiom as `difficultyProfile`.
- `Assets/Tests/EditMode/AIDriverProfileTests.cs` — 12 tests. The load-bearing ones assert
  structurally rather than by value: no tunable is a public field, every tunable is a readable
  *and writable* property, the fallback is cached, and **the prefab serialises no tunable keys**.

### Known gap — the three tiers are identical
`AIDriver_Easy`, `AIDriver_Medium` and `AIDriver_Hard` currently hold **the same 59 values**;
measured, 0 keys differ. They exist and are assignable, so P0's requirement is met, but picking
a tier changes nothing. That is exactly the work **P4** exists to do, and P4's acceptance test
("paces the three profiles on the same racing line and fails if their lap targets are within a
few percent of each other") will not pass until they differ. Do not read the presence of three
assets as three difficulties.

### In progress
- **P1** — `AISpeedProfile` is written and tested but **not wired into the per-frame loop**. The
  controller still targets the speed of the single waypoint ahead (D1) through a proportional
  speed-error controller with a ~47 km/h coast band (D2).

---

## 9. P1 landed (commits `e79547a`, `0a6b6c5`)

EditMode **175/175 green** (159 at session start, +16 new). Production code compiles with no new
warnings.

### What was built

**`Assets/Scripts/Brain/AILongitudinalPlanner.cs`** — the missing half of P1. `AISpeedProfile` was
pure arithmetic and correct; nothing called it. This class is the part that needs a car: it walks
the real line, measures the budget, and holds the learning state.

- `SolvePlannedSpeedKmh` takes the **minimum over every waypoint inside a braking horizon**, not
  the next waypoint. The horizon is `v² / (2·a) · 1.25`, floored at `minimumBrakingHorizonMetres`
  and capped at `maxBrakingHorizonWaypoints` entries — a *distance*, so it is ~175 m at 240 km/h
  and ~45 m in a hairpin. A waypoint *count* would get one of those two wrong.
- `MeasureBrakeDecelMs2` derives the budget from the clamp the simulation **actually enforces**:
  `Σ_grounded (NormalForce × tireForceLimitMultiplier) / mass`, scaled by measured
  `advancedBraking.BrakeEfficiency`. This follows D10 — there is no longitudinal tyre model to
  read (`RaycastWheel.cs:135` hard-zeroes `LocalSlipVector.y`, so `LongGripCoeff` ≡ 0), so the
  budget comes from `DrivetrainBrakeSystem.cs:252`'s force cap rather than from a traction limit
  that does not exist.
- Learning from `MeasureBrakeDecel` is gated on the brake having been **≥ 0.8 applied**, and the
  result is a `Min` against the measured clamp. Both directions are load-bearing: a soft brake
  sheds speed for reasons unrelated to capability, and observation must never raise the budget
  above a force the physics will refuse.

**Wiring** (`AIDriverController.Simulate`) — the plan is applied **last**, after the overtake
boost and the recovery clamp, not before. The boost raises the target by up to 7%; applying the
minimum earlier would let a committed pass ask for a corner entry speed the car cannot brake for.

**The longitudinal law** — feed-forward plus a narrow trim replaces the proportional band. The
brake term is `excess / horizonSeconds / budget`, so it ramps in continuously as a corner
approaches. The old law had a dead zone of ~47 km/h (any error in [−31, +16] km/h commanded
neither pedal), which is *why* the oscillation was the corner-independent rhythm it was.

### Two defects the new tests caught

Both were in my own work, and both are the original defect wearing a different costume:

1. **The horizon recorded segment length, not distance from the car.** Every constraint was
   recorded ~11 m away, so the solver granted corners the speed a car would have after braking
   over one segment — and never slowed for them. `EachWaypointIsConstrainedByTheGapAheadOfItNot
   TheGapBehindIt` already existed for this class of error in the pure solver; the same care was
   needed one level up.
2. **`SolveAllowedSpeedKmh` read the whole buffer.** The planner reuses per-tick arrays sized to
   the largest horizon, so the zero-filled tail was fed in as real constraints — a zero target
   speed at zero distance pinned every result to a standstill. It now takes an explicit `count`.
   Caught as two safety-law failures reading `"Line target 0 km/h, clamped to 0 km/h"`.

### Verification

- `Assets/Tests/EditMode/AILongitudinalPlannerTests.cs` — **12 tests**. Horizon length against real
  speed; a slow car getting a *short* horizon (catching a horizon that silently degenerated to
  scanning the whole line); the minimum beating the next waypoint; the budget formula pinned as a
  *formula* (sum over grounded wheels / mass) rather than a value, so it does not rot when the car's
  mass changes; ungrounded wheels excluded; the three learning gates.
- `Assets/Tests/EditMode/AIRealTrackPlannerTests.cs` — **4 tests**, and the answer to **D13**. Solves
  the plan from **every one of the 300 real waypoints** and independently recomputes the tightest
  constraint in that horizon, asserting the permitted speed never exceeds what the brakes allow.
  Recomputing separately keeps it a check *on* the planner rather than of it against its own
  output. Two further tests guard the first from passing vacuously — the line must genuinely carry
  a spread of corner speeds, and the planner must see more than one waypoint.

### Still open

- **The PlayMode gate.** No test yet drives a car along the real line in motion and watches the
  brake telemetry. The static geometry check is a real improvement on D13 but it is not the same
  thing: it proves the plan is *feasible*, not that the controller *chases* it. P1's own
  verification (c) — a live race showing one braking event per corner — is also unrun.
- **The coast band is gone but unmeasured on track.** The dead zone is closed in code; nothing yet
  asserts the resulting lap behaviour.

