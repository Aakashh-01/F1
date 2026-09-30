# Session handoff — phantom lap, spawn bump, player at back of grid

**Status: ALL THREE IMPLEMENTED AND VERIFIED, 2026-09-28. Read §0b before the rest.**

Written 2026-09-27 as a plan. The follow-up session implemented it and re-diagnosed §2; the
original plan below is kept for its evidence, but two of its conclusions were wrong and are
marked. Do not implement from this document.

Branch at the time: `feature/client-demo-ui`.

## 0b. What actually happened

- **§1 (17.999 lap) — fixed as planned.** `FilePathOverride` plus `ProfileIsolation` in all
  7 profile-touching PlayMode fixtures. Confirmed by re-reading `player_profile.json` after
  two full race runs: `ghostLapData` empty, no 17.999. Verification step 2 passes.
- **§2 (bump) — the transform+body fix was done, but the DIAGNOSIS was wrong.** Writing both
  is kept because it is correct. The bump was not `Physics.SyncTransforms()`: the cars hang
  on a `RaycastWheel` suspension that self-centres the body at **0.99736 m**, and the
  collider-based snap was placing the AI car **0.97 m below** that. The AI's collider is a
  small box that by design floats a metre above the road, so `colliderBottom = 1.0104` was
  never a fault and never the bump. The wheel anchors went half a metre into the tarmac and
  the bump stop answered with ~3 g. `Snap` now asks the suspension's own rest geometry
  (`GetRestAnchorDistance`) and moves a parked car by **0.0007 m** instead of −0.97 m. The
  collider path is kept as the fallback for a wheel-less body, which is what the 6 EditMode
  snap tests exercise, so all 6 still pass.
- **§3 (back of the grid) — fixed as planned.** Verified live: `Player starts P9 of 9`, and
  the grid renders with the field ahead of the player.
- Verification step 5 passes: the log now reads `Spawned player car at (25.16, 0.96, -0.35)`
  rather than the stale `(0.00, 1.00, 0.00)`.

**EditMode is 94/94 green.** `SceneContentTests.TrackScene_HostIsFixedToRace` was deleted: it
required a `FlowSceneHost` on `Track_01` that `SliceS4PreRaceSceneTests` and
`SliceS5RaceShellTests` both require to be absent. It had never passed on a fresh checkout.

**Still open, all pre-existing:** the PlayMode suite hangs reproducibly at 146/155 on
`SliceS4QualifyingShellTests.OnlyOnePlayerCarIsSpawned` and then latches `tests_running`,
blocking compilation — stopping play mode clears it, `clear_stuck` alone does not. ~21
PlayMode failures remain (AI overtaking, cross-test leakage). `50_RaceScene` hosts Race but
has no screen prefab assigned, so the race has no HUD.

**Nothing was committed.** `VehicleGroundSnap.cs` is `git add`-ed so it cannot be lost.

---

## 0. Read this first — two traps in the tree

- **`Assets/Scripts/Brain/VehicleGroundSnap.cs` is UNTRACKED** (`??` in `git status`) along with
  its `.meta`. It is the file the spawn fix lives in. It has never been committed. Do not lose
  it, and `git add` it as part of this work.
- **`Physics.autoSyncTransforms` is `False`** in this project. Every manual placement therefore
  has to end in a `Physics.SyncTransforms()` or the physics engine never sees it. This is the
  direct cause of bug 2.

---

## 1. Lap time always shows 17.999

**Not a broken timer.** The `LapTracker` measurement bug is already fixed — its own comment at
`LapTracker.cs:280-284` describes this exact 18-second phantom lap, and the
`_minimumProgressMetres` guard that stops it is in place.

**Root cause:** the value is read from disk, not recomputed.
`GameFlowManager.StartQualifyingSession()` seeds the session with

```csharp
_session.RestoreSavedBestGhost(profile.GetBestLapTime(track.TrackId),
                               profile.GetGhostData(track.TrackId));
```

and `player_profile.json` currently holds:

```json
"bestLapTimes": { "track_monaco": 17.999996185302736 },
"ghostLapData": { "track_monaco": "ghost-data" }
```

`"ghost-data"` is the literal string `Phase3FlowSceneTests.cs:364` passes, so this is
test pollution, not a player result. See [[the recurrence]] below.

**Fix (approved by the user):**
1. **Surgically clear** only `bestLapTimes` and `ghostLapData` in
   `C:/Users/aakas/AppData/LocalLow/DefaultCompany/F1/player_profile.json`. Leave
   `ownedCarIds`, `ownedTrackIds`, `wingPreferencePerTrack` and the volume settings alone.
2. Stop it recurring — add a `public static string FilePathOverride` to `PlayerProfileManager`
   (consumed by the existing `FilePath` when non-null, `PlayerProfile.cs:214`) and point the
   PlayMode fixtures at a temp file, reset in teardown.

**Flag for the demo:** after clearing, `GameSessionContext.CanStartRace` requires
`BestLapTime > 0`. Someone must actually drive a real qualifying lap in PreRace before the race
unlocks. That is correct behaviour, but it is the first thing you will hit.

## 1b. The recurrence

PlayMode tests drive the real flow singleton and a real `LapTracker`, and `SetQualifyingTime`
(`Phase3FlowSceneTests.cs:364`, `SliceS5RaceShellPlayModeTests.cs:81`) writes through to the
real save. Until the path is injectable, **any test run that touches the session re-poisons the
save.** Re-read the JSON after running tests.

---

## 2. Player car spawns with a bump

Reproduced and measured on a live race. The ground snap works — and is then silently undone.

`VehicleGroundSnap.Snap()` lifts the **body only** (`body.position += new Vector3(0, lift, 0)`)
and leaves the Transform at the un-snapped grid height. `RaceGridManager.SpawnGrid()` ends with
`Physics.SyncTransforms()` (`RaceGridManager.cs:140`). Because the car's Transform genuinely
moved during placement, that sync pushes the stale height back into the body and discards the
lift.

Measured, immediately after `SpawnGrid()` on a live race:

```
bodyY = 0.0500    colliderBottom = -0.8707    <- 0.87 m INSIDE the road
```

PhysX ejects it on the next step (observed rising to y ≈ 0.997 — *that* is the bump). Calling
`Snap` by hand at the same instant gives `bodyY = 0.9607`, `colliderBottom = 0.0400` — correct.
The snap is not wrong; it is reverted.

**Fix** — in `VehicleGroundSnap.Snap`, write the corrected position to the Transform as well as
the body, so the two cannot disagree and no later sync can undo it:

```csharp
Vector3 corrected = body.position + new Vector3(0f, lift, 0f);
body.position = corrected;
body.transform.position = corrected;
```

This is the single root fix — `PlayerCarSpawner.PlaceAtStartPose` and
`RaceGridManager.PositionCarOnGrid` both route through `Snap`.

**Also:** `PlayerCarSpawner.PlaceAtStartPose` writes only `body.position`, which is why the
console prints `Spawned player car at (0.00, 1.00, 0.00)` — the Transform is still at the world
origin. Write the transform there too so the log tells the truth.

**Do not re-introduce a lift constant.** The height is a measurement; see the existing
`VehicleGroundSnap` design notes.

---

## 3. Start the player at the back of the grid

**Current behaviour:** `RaceSceneController.PlaceOnGrid()` calls `_flow.ResolveRaceGridPosition(field)`
→ `GridPositionResolver.Resolve(bestLap, fieldCount, benchmark)`. The bogus 17.999 lap beats every
AI benchmark, so the player takes pole. Confirmed live: `Player starts P1 of 9, field of 8`.

**This is also why the car gets bumped.** With the player on pole, all 8 AI start behind and
accelerate into a stationary car. Observed after ~30 s of an unattended race: the player was
knocked **162 m forward**, and 3 AI cars ended up *behind* the start line. From the back,
nothing is behind the player to hit them.

**Fix (approved):** a serialized toggle on `RaceSceneController` — the class already documents
that "which slot the player starts from" belongs to the race scene, so this introduces no new
concept:

```csharp
[Tooltip("Start the player at the back of the grid regardless of the qualifying result. " +
         "For a client demo, where the whole AI field ahead of the player reads better.")]
[SerializeField] private bool _startPlayerAtBackOfGrid;
```

In `PlaceOnGrid()`, when set, use the field size + 1 instead of the qualifying result. Tick it
on `50_RaceScene` so it is an Inspector switch — no recompile to turn off.

Verified all slots 0–8 have tarmac beneath them (road at y = 0.000 under slots 0, 4 and 8), so
slot 8 (4 rows back, 72 m) is on the road.

`GridPositionResolver` is left untouched — its policy stays intact and tested; the override sits
above it in the race shell.

---

## Files to change

| File | Change |
|---|---|
| `Assets/Scripts/Brain/VehicleGroundSnap.cs` | write transform + body together (root fix) — **untracked, see §0** |
| `Assets/Scripts/GameFlow/PlayerCarSpawner.cs` | write transform in `PlaceAtStartPose` |
| `Assets/Scripts/GameFlow/RaceSceneController.cs` | add `_startPlayerAtBackOfGrid` + use it |
| `Assets/Scenes/50_RaceScene.unity` | tick the new flag |
| `Assets/Scripts/Progression/PlayerProfile.cs` | add the `FilePathOverride` seam |
| `Assets/Tests/PlayMode/*.cs` | point the profile at a temp file, reset in teardown |
| `player_profile.json` (LocalLow) | clear the two poisoned entries |

---

## Verification

1. **Tests** — run EditMode and PlayMode suites. `SliceS5RaceShellTests` asserts on
   `GridPositionResolver` directly and is unaffected; confirm it still passes.
2. **Re-read `player_profile.json` after the test run** — it must not gain a `bestLapTimes`
   entry. This is the actual proof the pollution is fixed.
3. **Play mode, driven through the UI** — Lobby → Track → Wing → PreRace. Drive a real lap,
   confirm the HUD shows a real time. Enter the race.
4. **Screenshot the grid.** Gate: cars sit level on the tarmac, no settle or jump, and the
   player is at the **back** with the field ahead. Re-read `colliderBottom` — it must be ≈ +0.04,
   never negative.
5. Confirm the console no longer prints `Spawned player car at (0.00, 1.00, 0.00)`.

---

## Driving the flow from MCP — traps that cost time

- **Select the track in a SEPARATE tool call from `GoToRace()`.** Doing both in one call races
  the async screen transition; `GetSelectedTrackSceneName()` comes back empty and the console
  shows `EmptyName(): No content scene name was supplied` → `Track content load failed` → the
  race never spawns a car.
- **Load the track with** `Resources.LoadAll<TrackDefinition>("GameData/Tracks")` and pick by
  `TrackId`. `Resources.Load<TrackDefinition>("GameData/Tracks/track_monaco")` returns **null**.
- **Jumping straight to the race leaves the start gate unlocked** (`InputLocked == False`),
  because no overlay is driving the countdown — the console says *"No session start gate became
  ready within 30s; skipping the countdown"*. An unattended car is then immediately rear-ended.
  Not a product bug; it makes the race look broken when you are only inspecting.
- **`execute_code` loses project types after entering Play mode** until names are fully
  qualified — use `F1.GameFlow.GameFlowManager`, not `GameFlowManager`.
- **`TrackDefinition` has `TrackId`, not `Id`.**
- Opening `Track_01` in the editor to measure is safe, but check `isDirty` on the current scene
  first — the flow scenes are hand-tuned and unsaved edits would be lost.

---

## Out of scope (flagging, not fixing)

`SESSION-HANDOFF-LOBBY-AND-WING.md` §9 item 5b: `Builds/F1Client.exe` builds but **has never
been run**, and the build log warns *"No RuntimePipelineConfig asset found — Pipeline will be
disabled in Player builds"*, so URP may behave differently in the built game than in the
editor. Worth launching the exe once before the client demo.
