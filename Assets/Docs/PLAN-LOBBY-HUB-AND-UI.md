# Plan — lobby hub, and the UI pass that follows it

Written 2026-09-26, after the 3D car lobby was built and the flow was changed to route
through a hub. **This supersedes `SESSION-HANDOFF-3D-LOBBY-AND-UI-LAYOUT.md`**, which was
written before any of the lobby work and is now stale in several places.

---

## 1. What is already done

- **3D car lobby is live** in `10_CarSelectionScene`. Real garage geometry (the free
  "Simple Garage" asset, scaled 1.45), the car standing on the floor, a dimmed background,
  a key spot with shadows, and the screen's opaque backdrop disabled so the garage shows
  through.
- **`CarShowcase_Prefab`** — a stripped, visual-only clone of the car (0 script references:
  no Rigidbody, no colliders, none of the driving scripts, no cameras). `F1_Body.prefab` is
  untouched.
- **The car is static** at a fixed three-quarter yaw. Drag-to-spin was decided and then
  explicitly reversed; `ShowcaseTurntable` was written and deleted.
- **The car, camera and key light are hand-placed by the user** and tuned. The lobby builder
  is scaffold-only and refuses to touch an already-assembled scene.
- **Phase A DONE — the lobby.** `05_LobbyScene` exists and is in the build list at index
  1. Loading routes to it. It carries the hand-tuned garage rig, copied live out of
  `10_CarSelectionScene` at build time, plus `LobbyHubScreen_Prefab`: the car tile strip,
  Next, and the currency readout.
  Verified in play: the car tiles are up on arrival with the car unmoved, and Next is
  disabled until a car is chosen.

- **The hub was removed 2026-09-30.** The lobby used to open on a hub panel (currency,
  Settings, Tasks, START RACE) that had to be clicked through to reach the tiles. That
  click is gone: `05_LobbyScene` now opens directly on car selection. The hub panel,
  Start Race, Settings, Tasks and Back are deleted from the prefab and the builder, and
  `ShowHubState`/`ShowSelectionState`/`IsShowingSelection` are gone from `ILobbyScreen`
  and `LobbyScreen`. The currency readout moved onto the selection panel at its original
  top-right anchors, so the screen does not shift. Settings and Tasks were already dead
  ends — nothing subscribed to `OnSettingsPressed` or `OnTasksPressed` — so no working
  behaviour was lost. This reverses the 2026-09-26 hub decision.

### Verified end to end
```
00_Loading -> 05_Lobby (car tiles) -> pick a car -> Next -> 20_TrackSelection
```
Next is disabled and `LobbySceneController.OnNextPressed` refuses to navigate while
`_flow.SelectedCar` is null, so a premature press cannot load a track selection with no car.

## 2. The flow change

```
00_Loading
   |
   v
05_Lobby  -- the 3D garage, car standing in it, car selection over it:
              currency readout, horizontal car tiles, NEXT
   |
   |  [pick a car -> Next]
   v
20_TrackSelection  ->  30_WingSetup  ->  40_PreRace  ->  50_Race
```

**The lobby IS the garage.** It is not a menu in front of the garage, and there is no
"click Car Selection to navigate somewhere" step. The car stands in the garage and does not
move; what changes is the UI laid over it. This matches the Traxion reference in
`Assets/Scenes/img.jpeg`, where the car stays put and the furniture around it changes.

**One scene, two UI states** - not two scenes each loading the garage. Decided so the garage
is built once and the car does not blink or re-appear on the transition. The flow moves on
to a *different* scene only at the Next button, going to track selection.

Car Selection becomes a real chooser rather than a waypoint:

1. Player taps a car tile in the bottom strip.
2. That tile shows its selected state, and the Select button on it becomes active.
3. A separate Next button advances to Track Selection.

**Selecting a car must stop auto-advancing.** Today `CarSelectionSceneController.OnCarSelected`
calls `GoToTrackSelection()` the moment a car is chosen. That is the single behavioural change
the new flow requires, and it is what makes the Select/Next pair meaningful.

### 2a. This reverses a documented client requirement

The flow was originally built so that there is **no main menu** — loading went straight to
car selection. That is encoded in the code and enforced by tests, so the following are
expected to change deliberately, not to be worked around:

- `FlowSceneNames.CarSelection` — its comment says "there is no main menu between loading and
  car selection". Update it.
- `LobbyManager` — "Auto-advance: spec has no Continue button on car/track screens". Stale.
- `Phase3FlowSceneTests.BuildSettings_StartWithLoadingThenCarSelection` — asserts build
  settings index 1 is CarSelection.
- `Phase3FlowSceneTests.BrandingCompletesIntoCarSelectionNotAMainMenu` — exists purely to
  enforce "not a main menu".
- `Phase3FlowSceneTests.BuildSettings_ExcludesTheLegacyLobby` — asserts the dead
  pre-Phase-3 `LobbyScene` must not ship.

**Name collision to watch:** that dead `LobbyScene.unity` + `LobbyManager.cs` pair still
exists on disk. The new hub must use a distinct name (`05_LobbyScene`) or the legacy pair
should be deleted first.

## 3. Work items, in order

### Phase A — the hub — **DONE**, with two traps already paid for
0. **Append** `Lobby` to the end of `GameFlowManager.GameScreen`, never insert it. The enum
   is serialized as an integer by `FlowSceneHost._hostedScreen`, and the existing scenes
   carry `0, 2, 3, 4, 5, 6`. Inserting a member at index 2 would shift every one of them
   and make each flow scene host the wrong screen with no error anywhere.
1. `FlowSceneNames`: add `Lobby = "05_LobbyScene"`; update the stale "no main menu" comment.
2. `GameFlowManager`: add `GameScreen.Lobby`, a `_lobbyScene` field, `GoToLobby()`, and route
   branding completion to the hub instead of car selection.
3. New `ILobbyScreen` + `LobbyScreen` + `LobbyHubScreenImpl`. The screen sits over the
   garage and shows the tile strip, Next and the currency readout; the garage, car, camera
   and lights are untouched by anything it does.
4. Reuse the hand-tuned garage rig from `10_CarSelectionScene` rather than rebuilding it.
   `LobbyBuilder.CaptureGarageRig()` reads the live values and `ApplyGarageRig()` re-creates
   them, so re-running after a re-tune keeps the two scenes in agreement.
5. Route: Lobby -> Next -> `20_TrackSelectionScene`.
6. Update the five stale tests above to assert the hub route. **DONE.** EditMode is back to
   its baseline: 10 of 11 pass, the one failure being the pre-existing
   `TrackScene_HostIsFixedToRace`. PlayMode `Phase3FlowSceneTests` is 10 of 13; the three
   failures are all pre-existing and none involve the lobby —
   `FutureFlowScenes_AreNamedButNotYetBuilt` (asserts `40_PreRaceScene` does not exist, and
   it does), `LeavingCarSelection_UnregistersItsHost` (the FlowSceneHost Awake-ordering
   error, which was already in the console before any test was touched), and
   `HostScreenMode_QualifyingOrRace_FollowsTheSessionType` (a scene-operation collision on
   `50_RaceScene`). The last two have not been proven against a clean pre-change baseline.

**Two traps this phase hit, so they are not re-learned:**
- `Phase3SceneBuilder.CreateScene` OPENS an existing scene rather than replacing it. The rig
  is created with `InstantiatePrefab`, which bypasses `EnsureRoot`'s by-name dedupe, so every
  re-run stacked another copy — five Garages before it was caught. `ApplyGarageRig` now calls
  `ClearOwnedRig` first. **Do not run `Tools > BuildFlowScenes`** either: it rebuilds every
  flow scene from empty and would wipe the hand-tuned car selection scene. Use
  `Tools > BuildLobbyScene`.
- `UnityEventTools.AddPersistentListener` only persists when its target is already a
  prefab asset. Adding listeners to a throwaway scene GameObject before the first
  `SaveAsPrefabAsset` produced a prefab whose buttons were all inert — Start Race did
  nothing. The builder now saves, reopens the prefab contents, and wires in a second pass.

### Phase B — car selection becomes a chooser — **DONE**
7. Stripped auto-advance out of `CarSelectionSceneController.OnCarSelected`. Selecting a car
   no longer navigates anywhere.
8. The tiles live in the **lobby's** `TileStrip`, not in `10_CarSelectionScene`. One
   horizontal strip along the bottom, six across, ordered owned → rentable → locked. The
   tier that used to be expressed by which of three columns a card sat in is now carried by
   the ordering plus each tile's own status line (OWNED / RENTAL / LOCKED).
9. Choosing a tile marks it (its `SelectedIndicator` lights) and enables Next. Next goes to
   track selection. Verified: 0 marked and Next dead on entry; a **rejected** tap leaves all
   three clear; an **accepted** tap marks exactly one and enables Next.
10. The card is now a **tile**: 250 x 280 with a 132 art panel, replacing 300 x 270. A row of
    squarer cards reads as a shelf of cars; the old 300x270 read as a shelf of letterboxes.
    Layout still goes through `ClientDemoUiBuilder`, never by hand-editing the prefab.

**The subtle bug Phase B caught.** The screen marked a tile optimistically on tap, then asked
the flow whether to accept it. For a rental the player has no rental for, `SelectCar`
rejected it — but the tile stayed lit and Next stayed enabled, so the player could select a
car the session would never carry, press Next, and get nothing. Fixed by having the controller
push the flow's verdict back through `ILobbyScreen.SetChosenCar`, so a rejected tap clears
the mark and disables Next. Anything that asks the flow a question and then keeps its own
optimistic answer is the shape to watch for here.

### Phase C — track selection — **DONE**
11. Monaco is a centred hero tile at 430x380 with an AVAILABLE status; the other four flank it
    two-and-two at 250x280. The Owned/Locked column split is gone — it produced a ragged 2x3
    grid with a hole, and it duplicated what the per-card status already says.
12. **New background.** The old one was an aerial night photograph of a circuit, and the
    thumbnails are also warm, detailed circuit photographs — two images of the same subject at
    the same level of detail, so they fought and neither read. `bg_trackselect_v2.png` is a
    dark abstract circuit-line graphic: cool near-black, so the warm thumbnails are the
    brightest thing on screen. The original `bg_trackselect.png` is still on disk.

**The bug Phase C caught.** The hero was chosen from `ProgressionRegistry.GetOwnedTracks`,
which returns what the profile *owns* — and both Monaco and Spa are free, so both arrived.
But Spa is flagged `UnderConstruction` and points at the same corner as Monaco, so it cannot
be raced, and it was being given a supporting tile in the *centre column* beside Monaco. That
read as two playable options when there is only one. The split now uses
`SceneAvailability.IsAvailable`, the same test the card uses to draw its badge, so the layout
and the badge can no longer disagree about what is playable. Partitioning a UI by ownership
when the thing that matters is availability is the trap here.

Also: an unavailable circuit's status line used to say COMING SOON *and* carry a COMING SOON
badge, so every locked tile said the same thing twice in two colours. The badge owns that
message now and the status line stays empty.

### Phase D — wing selection
12. Still open: reuse the garage for a 3D wing lobby, or keep wing selection 2D.
13. Either way the screen must be fixed — it currently shows **nothing**. It has no card or
    tile system at all, just two bare `Toggle`s, and no code path ever reads
    `WingAeroProfile.icon`, so the wing art is invisible by construction. Its background
    `bg_wingsetup.png` was also never generated.
14. The car has addressable wing objects, so a visible difference between the two setups is
    achievable with no new asset — but note the DRS pivot nodes have identity transforms, so
    a pivot has to be introduced on the hinge line before rotating them, or the whole flap
    swings around the car origin. And `Car_Model` carries a non-uniform scale
    `(1.8, 0.6, 4)`, which skews a naive child rotation.

## 4. 10_CarSelectionScene — REMOVED

Car selection is the lobby's second UI state, so its own scene was retired. Deleted
`Assets/Scenes/10_CarSelectionScene.unity` and dropped it from the build list (it now runs
Loading -> Lobby -> Track -> Wing). `GoToCarSelection()` and `_carSelectionScene` are gone
from `GameFlowManager`, as is `Phase3SceneBuilder.BuildCarSelectionScene`.

**`GameScreen.CarSelection` was KEPT at index 2, and must stay there.** FlowSceneHost
serializes its hosted screen as this enum's integer; the remaining scenes carry 3, 4, 5 and
6. Deleting or moving that member shifts all of them down by one and every scene silently
hosts the wrong screen. During this work the member was briefly moved to the end by mistake
and had to be put back — it is the single most dangerous edit in the enum.

**A latent bug this surfaced:** `TrackSelectionSceneController.OnBackPressed` called
`GoToCarSelection()`, so the track screen's Back button pointed at the scene being deleted.
It now goes to the lobby. (The track screen has no Back button in the prefab today, so the
handler was unreachable — but it would have become a live route the moment one was added.)

### Still on disk, deliberately
- `Assets/Prefabs/CarSelectionScreen_Prefab.prefab` — nothing in the build uses it now. It
  was not deleted, only the scene was authorised for removal.
- `Assets/Scenes/LobbyScene.unity` + `LobbyManager.cs` — the dead pre-Phase-3 single-scene
  lobby, still referenced by that prefab. `LobbyManager` had to be repointed at the lobby
  because it called the now-removed `GoToCarSelection()`. Worth deleting as a pair.
- `PrefabBuilder` / `SceneBuilder` / `UIBuilder` — legacy builders that would recreate the
  car selection prefab if run. Superseded, not deleted.

## 5. Still deferred, by agreement
- Livery swap, so each tier shows a differently-coloured car. All six `CarDefinition`s point
  at the same prefab.
- `SceneContentTests.TrackScene_HostIsFixedToRace` — pre-existing failure, belongs to Phase 5.
- `50_RaceScene` has no screen prefab at all.

## 5. Verification habit
Render it and look; never assert a UI change works. A 3D screen that passes every test can
still be 40x too large or buried in the floor. Screen-space-overlay canvases cannot be
captured by a camera render without temporarily switching the canvas to `ScreenSpaceCamera`
and restoring it.
