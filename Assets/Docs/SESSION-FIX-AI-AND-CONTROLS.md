# Fix plan — AI field deadlock and race-mode controls

Written 2026-09-28 after an inspection pass. Nothing in this plan has been applied yet except
the two items marked DONE. Every claim below was verified by reading the source or by
measuring the live editor, not inferred.

## 0. What I changed, and whether it caused this

My only gameplay edit this session is `Assets/Scripts/Brain/VehicleGroundSnap.cs`
(the staged file). Everything else I touched was tests and docs. No scene file was modified —
`git status Assets/Scenes/` is byte-identical to the pre-session snapshot.

**It did not cause either symptom.** It changes spawn **Y** only, and measured against a
parked car it now moves the body **0.0007 m** — the XZ gap that triggers the AI brakes is
untouched. Direct evidence: the first race run of this session, before I edited anything,
already had 5 of 8 AI stuck at 0 km/h on the grid.

**But it removed an accidental escape hatch, and I want that on the record.** The old
collider-based snap put the AI car 0.97 m under its suspension equilibrium; the bump stop
answered with ~3 g, and that launch scattered some cars out of the sensor's emergency range.
Removing it made the deadlock *more consistent*, not new. So the honest summary is: I did not
create this bug, but I removed the thing that was accidentally hiding it. The response is to
fix the AI properly — **not** to restore a 3 g launch.

## 1. Symptom: "AI cars are still reacting to other AI cars"

### Measured, live, on `Track_01`

6 of 8 AI stationary at 0.0 km/h, still in grid formation 3.3–3.7 m apart. Their state:

| field | stuck cars | the 2 that escaped |
|---|---|---|
| `LastSpeedTargetKmh` | **0.000** | 224.3 |
| `LastUnblockedTargetSpeedKmh` | 283.2 | 224.3 |
| `LastSpeedClampReason` | **EmergencyBrake** | None |
| `LastThrottleInput` / `LastBrakeInput` | 0.000 / **0.850** | — |
| `LastLeaderSpeedKmh` | 0.003 | 125.2 |

One car was found **upside down** (`transform.up.y == -1`) and headings were scattered across
±130°. This is not a subtle handling issue; the field does not race.

### Root cause — four compounding defects, all in uncommitted earlier work

**A. The overtake speed-ease is a mathematical no-op.** `AIDriverController.cs:563-567`

```csharp
float eased = Mathf.Lerp(cautionTarget, targetSpeedKmh, overtakeTrafficSlowdownScale);
cautionTarget = Mathf.Min(cautionTarget, eased);
```

`Lerp(caution, target, t)` with t∈[0,1] returns a value in `[caution, target]`. When
`target > caution` (always, in traffic) `eased > cautionTarget`, so `Min` returns
`cautionTarget` — the branch cannot raise the clamp and can only lower it. A committed
overtaker gets **exactly the same** speed as a car that gave up. The comment at :548-562
states the intent ("eases the clamp, but never RAISE it"); the operand order is inverted.
The `Min` at :569 already caps the result at `targetSpeedKmh`, so the clamp cannot exceed the
target even if the ease is applied.

**B. The emergency brake judges distance only — never leader speed.** `AIDriverController.cs:601-622`
and `GetEmergencyDistanceMetres` :659-665 give `6 + 2.3·v` capped at the sensor reach (32 m
on the prefab). Above ~40.7 km/h the cap binds, so the emergency fires on **any** object
anywhere inside the full 32 m regardless of how fast it is going. It never reads
`LeaderSpeedKmh`. `ClosingSpeedKmh` is computed at :647, stored in `LastClosingSpeedKmh`,
and **never used anywhere in the control law**.

**C. The follow law has no acceleration authority, so it cannot launch.** `CalculateFollowTargetKmh`
:641-657 returns `leaderSpeed + max(0, surplus) × 1 km/h-per-m`, capped at 20 km/h. With a
stationary leader that is a target of a few km/h. It is then collapsed further at :538:
`cautionTarget = Lerp(followTarget, 0, distanceBlend²)`. On the grid, spacing is 8 m against a
32 m sensor reach, so `distanceBlend` is already **0.83** and the target lands near **1 km/h**.
A car cannot leave the line at 1 km/h. (This is the "AI car crawling at ~1 km/h" symptom
already in my notes — same defect.)

**D. The grid guarantees the trigger fires, at t=0.** `RaceGridManager.cs:43-44`:
`gridRowSpacing = 8`, `gridColumnSpacing = 5`; prefab `sideDistance = 8.5`. The car ahead is
8 m — inside the 6 m emergency trigger even at a standstill. The side partner is 5 m lateral —
inside side range — so **every car sees a permanent side block for the whole race** on a
2-wide grid, which suppresses lane changes, halves the overtake options, and trips the
side-traffic throttle cut. And `obstacleLayers = ~0`, so kerbs and walls register as traffic
too.

Chain: on the line each car sees a stopped neighbour inside emergency range → brake 0.85,
throttle 0, target ~1 km/h → nobody moves → nobody's leader ever moves → permanent deadlock.
The two that escape are the ones whose sensors happened to find no car in range.

### Also masking everything: a test leak

`CreateObstacle` (`F1FoundationPlayModeTests.cs:2353-2360`) makes root-level primitives that
are cleaned up only by `Object.DestroyImmediate` lines *after* the final assert. An assert
failure throws, so the cube leaks and every later sensor in the run sees a phantom car ahead.
`CreateRigidCar` objects do have a teardown; `CreateObstacle` does not. This is why the AI
failure count drifts between runs (17 then 21) with the same code.

## 2. Symptom: "the controls are changed in race mode"

**A. The player is handed the AI's physics profile.** All six
`Assets/Resources/GameData/Cars/CarDefinition_car_gen*.asset` set
`_basePhysicsProfile` → `F1_AI_Physics`. `PlayerCarSpawner.ApplyWingProfile:122-123` then
overwrites the prefab's `F1_Player_Physics` with it via
`GameFlowManager.CreatePlayerPhysicsProfile` → `CarDefinition.CreateProfileWithWing`.
`F1_Player_Physics` is referenced by the prefab and **nothing else** — it is dead in-game.
45 fields differ; the felt ones:

| field | AI (what race gives you) | Player (what the prefab intends) |
|---|---|---|
| `drivetrain.throttleSpoolSpeed` | **3.5** | **1.35** — throttle takes 2.6× longer to reach full |
| `steering.maxSteerAngle` | 17.5 | 16 |
| `advancedBrake.rearInstabilityStrength` | 0.36 | 0.06 — 6× the rear-instability yaw |
| `steering.oversteerAssistStrength` | 0.3 | 0.42 |
| `advancedSteering.countersteerStrength` | 0.42 | 0.52 |
| `drivetrain.motorForce` | 128000 | 98000 |
| `engineAudio.*` (11 fields) | **absent** | present — the wing clone silently discards audio tuning |

This is literally "the controls change when I enter race mode", and it only appears in race
because that is the only place the overwrite path runs.

**B. The start gate can freeze the entire field.** `RaceSceneController:162` locks the player
**and every `SpawnedCars` AI** via `coordinator.InputLocked`. The only release is
`FlowOverlay.cs` — **untracked, never committed**. Three independent ways it never fires:
- `StartGateRoutine` times out at 30 s (:198, :216-217) and only logs, leaving the field locked.
- `PlayCountdown:245` does `if (_countdown != null) StopCoroutine(_countdown);` — a second call
  **discards the previous `onComplete`**, so that gate is never released.
- `FindStartGate:220-228` scans `FindObjectsByType<MonoBehaviour>(Include, None)` across
  additively-loaded scenes, order unspecified. `SceneFlowService` unloads the outgoing scene
  *after* the new one loads, so there is a real window where both a `PreRaceSceneController`
  and a `RaceSceneController` implement `ISessionStartGate` and the wrong one can be latched.

While locked, `VehiclePhysicsCoordinator:179-185` zeroes all input and
`DrivetrainBrakeSystem:89-93` hard-writes velocity to zero every step — inert, not braked.

**C. Brake-as-reverse still applies to the player.** `SuppressReverse` is set only by
`AIDriverController:252`; the player never sets it. `ShouldUseReverse:351-360` reads brake-held
+ no-throttle + ≤2 km/h as a reverse request, and `reverseMaxSpeedKmh` is 12. In the garage you
barely stop; in a race you stop constantly, so this fires far more.

## 3. Execution order

P0 — the field must race at all
1. `:563-567` apply the ease for real (`cautionTarget = eased`, already capped by :569).
2. `:601-622` require actual risk before emergency braking — consult `ClosingSpeedKmh`, and
   keep the distance-only rule only for immovable scenery.
3. `CalculateFollowTargetKmh` give the follower a speed the gap can actually support, so a car
   with room accelerates instead of crawling; stop collapsing the target toward zero on raw
   sensor fraction.
4. `AI_F1_Body.prefab` narrow `obstacleLayers` so kerbs/walls stop reading as traffic.

P1 — controls
5. Repoint the six CarDefinitions at `F1_Player_Physics`.
6. Make the start gate always release: unlock on timeout, do not discard `onComplete` in
   `PlayCountdown`, and prefer the active scene in `FindStartGate`.

P2 — test hygiene
7. Register `CreateObstacle` objects for teardown so a failed assert stops poisoning the run.

P3 — verification
8. EditMode green. Live race: 8 AI moving, no upside-down car, player controllable, no
   `EmergencyBrake` deadlock at the line.

## 4. Open question for the developer

`_startPlayerAtBackOfGrid` is ticked on `50_RaceScene`, so the player starts P9 with the whole
field ahead. That was a deliberate demo choice, but combined with defect D it means the
player's grid partner is permanently "side blocked". Worth revisiting once the AI reacts
correctly — a front-row start may simply look and play better.
