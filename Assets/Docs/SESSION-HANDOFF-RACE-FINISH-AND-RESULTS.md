# Session handoff — race finish and results

Written at the end of the session that made a race end. **Read this before touching the finish.**

Companion: `SESSION-HANDOFF-LOBBY-AND-WING.md` (the lobby/wing session) and
`SESSION-HANDOFF-3D-LOBBY-AND-UI-LAYOUT.md` (now stale — describes a flow that no longer exists).

---

## 0. Read this first — the results scene is GENERATED

**`Assets/Scenes/60_ResultsScene.unity` is produced by `Tools > Results/Build All`, which deletes
and rebuilds it wholesale.** Do not hand-tune that scene and then re-run the builder. The same
warning `05_LobbyScene` carries, for the same reason.

Safe to re-run freely: `Tools/Results/Build All`, `Tools/Results/Capture Results Screen`.

---

## 1. What was broken

A race could not end. Not "ended badly" — there was no end at all. Three separate breaks, each
of which had to be fixed before any of them could show:

| Break | Where |
|---|---|
| Nothing detected the finish | No finish-line code anywhere in `Assets/Scripts` |
| The flow's results entry points had no callers | `GoToResults()`, `SetRaceResult()` — defined, never called |
| The scene the flow routes to did not exist | `60_ResultsScene` referenced at `FlowSceneNames.cs:42`, absent from disk and from the build list |

Everything *downstream* of the flag already existed and was coherent: `IResultsScreen`, the
abstract `ResultsScreen`, `ResultsScreenImpl`, the flow-manager wiring, `OnRaceFinished`,
`profile.RecordRaceFinish()`. This was a wiring gap, not greenfield — which is why it survived
so long. A race that never ends still looks like a game that is still loading.

## 2. What was built

New:

- **`Assets/Scripts/GameFlow/RacePointsTable.cs`** — the GDD's points table as a pure function.
- **`Assets/Scripts/GameFlow/ResultsSceneController.cs`** — wires the results screen's events to
  the flow. Mirrors `TrackSelectionSceneController`.
- **`Assets/Editor/ResultsScreenUiBuilder.cs`** — builds the prefab and the scene.
- **`Assets/Scenes/60_ResultsScene.unity`**, **`Assets/Prefabs/ResultsScreen_Prefab.prefab`**.
- **`Assets/Tests/EditMode/RaceFinishTests.cs`** — 10 tests.

Modified:

- **`RaceSceneController`** — subscribes to the player's `LapTracker.OnLapCompleted`, and
  `FinishRace()` classifies, holds the field, scores, and routes to results.
- **`GameFlowManager`** — `CompleteRace(position)`, `HasRaceResult`, `PresentRaceResult()`, and
  a result reset in `GoToRace`.
- **`ResultsScreenImpl`** — the unlock panel now announces rather than claims (see §5).

### The decoupling rule used

Policy is pure and separate; orchestration joins it. `RacePointsTable` holds the rule,
`GameFlowManager.CompleteRace` is the seam. This is the same split `GridPositionResolver` /
`ResolveRaceGridPosition` already use, and it is why the points table is testable without
standing up the flow singleton, the registry and the save file.

## 3. THE FINISH BUG — read this before changing how the race ends

**The first implementation did not work and the race ran forever.** Found by driving it, not by
any test — the entire EditMode suite was green while it was broken.

`LapTracker` raises `OnLapCompleted` from inside `CompleteLap()`, which it calls in a loop
**before** it assigns `CompletedLaps = lapsDone`:

```csharp
for (int i = CompletedLaps; i < lapsDone; i++)
    CompleteLap();          // <-- event fires in here
CompletedLaps = lapsDone;  // <-- count committed only after
```

So during the event, `tracker.CompletedLaps` still holds the value from *before* that update.
Finishing lap 2 of 2 raises the event while the count reads 1. An event handler testing that
count against the lap total reads it **one lap stale, forever** — so `CompletedLaps < TotalLaps`
was true on every single event the race ever produced, and the finish never fired.

**The fix:** poll `CompletedLaps` from `Update()` instead of subscribing to the event. Polling
reads the count after the assignment has committed, so it sees the true value. It is also the
right shape for this particular test — "the player has completed the race" is a *level*, not an
edge, so a frame missed costs nothing; the next frame still sees it.

**Do not "optimise" this back to the event.** If `LapTracker` is ever reordered to assign the
count first, the comment on `CheckForFinish` becomes wrong, and the reasoning has to be
re-examined rather than assumed.

## 4. THE DEAD BUTTONS — the second bug, and the same lesson

**Both buttons did nothing.** The screen rendered perfectly, so nothing about it looked wrong.

**Cause: the results scene had no EventSystem.** A Canvas does not need one to DRAW, only to
be *interactable*. Without it uGUI routes no pointer or click events at all — so the panel came
up complete and every control on it was inert.

What made this easy to miss: **every other flow scene has one.** `60_ResultsScene` was
generated from scratch and `FlowSceneHost` only instantiates the screen prefab — and no screen
prefab in the project carries an EventSystem, so each scene brings its own. Reading the other
scenes for a template is what hid it, because they all had the thing the new one lacked.

Fixed by `ResultsScreenUiBuilder.EnsureEventSystem`, which adds `EventSystem` +
`InputSystemUIInputModule` exactly as `Phase3SceneBuilder.EnsureEventSystem` does. The
module's actions are assigned on add and serialise against the project's
`InputSystem_Actions.inputactions`, matching every other flow scene.

**A new flow scene needs three things, not two:** a `FlowSceneHost` with its screen prefab, the
scene controller that wires the screen's events to the flow, **and an EventSystem**. The first
two were obvious; the third is invisible until someone clicks.

Guarded by `TheResultsSceneCanReceiveAClickAtAll`, which asserts the module's point AND
left-click actions are assigned — a present-but-unassigned module routes nothing either, so a
presence-only check would sit green over the same failure.

## 5. Decisions taken (by the developer, 2026-10-04)

| Question | Answer |
|---|---|
| Race length | `_raceLapsOverride` on `50_RaceScene` set to **2**, so a race is drivable to its finish in a sitting for repeated testing. Set it back to `-1` for the real 78-lap Monaco distance. |
| Points | The **full GDD table** — 1st 1250 … 10th 100 — included in this pass. This is what first makes career points earnable. |
| After results | **Continue → the lobby.** The points just earned become spendable there. |
| At the finish | **Classify and show results immediately.** AI are classified by standing, not by crossing the line. |

## 6. Verified

- **EditMode: 112 / 112 green**, including the 11 new ones. The `TrackScene_HostIsFixedToRace`
  failure the older handoffs record is no longer failing.
- **Driven by the developer**: the race now ends at the flag and the results panel appears.
  Both of the first two bugs below were found this way, with the whole suite green.
- **Rendered and looked at**, twice: `Assets/Screenshots/results_screen.png` (no unlocks) and
  `results_screen_unlocks.png` (panel populated). Both states are legible.
- **`player_profile.json` untouched** — still 0 points, last modified before this session.

### Still to confirm by driving

The buttons have never been clicked in play. The chain is verified statically — persistent
listeners present and pointing at the right methods, no event shadowing between
`ResultsScreenImpl` and its base, an EventSystem whose point and left-click actions are both
assigned — but the click itself has not been exercised. See §8.

### Four bugs found this session, none of which any assertion caught

1. **The race never ended.** `LapTracker` raises `OnLapCompleted` before committing
   `CompletedLaps`, so an event handler reads the count one lap stale forever. See §3.
2. **Both buttons were dead.** No EventSystem in the results scene, so uGUI routed no clicks
   while drawing the panel perfectly. See §4.
3. **The body text was invisible.** `SlimUiSkin.Ink` is near-black and belongs on the pack's
   *pale* cards; this screen sits on the dark screen-root panel, so near-black-on-near-black.
   Body text now uses `TileInk` / `TileInkMuted`; only button labels keep dark ink, because
   those genuinely sit on pale fills.
4. **The position wrapped, and the unlock panel painted over it.** `Finished: P1` at 120pt
   overflowed its 700-unit box onto two lines; and because uGUI draws siblings in hierarchy
   order and the panel is created after the position text, centred it covered the number. The
   word moved to a static `FINISHED` caption, the field carries just `P1` at 150pt, and the
   panel sits below the points block at a fixed offset.

3 and 4 were found by `Tools/Results/Capture Results Screen`. 1 and 2 by driving the game.

## 7. Things that will waste time if forgotten

- **`EditorBuildSettings.scenes` is written lazily.** Assigning it logs success and updates
  memory, but `ProjectSettings` is not flushed, so a build made before Unity next saves
  silently omits `60_ResultsScene` and the flow routes to a scene that will not load. The
  builder now calls `SaveAssets` itself; do not remove that.
- **`AddListener` is not `AddPersistentListener`.** `Button.onClick.AddListener` registers a
  *runtime* listener on the throwaway edit-mode copy of the prefab, and the button ships inert.
  Only `UnityEditor.Events.UnityEventTools.AddVoidPersistentListener` serialises one. This is
  why wiring happens in a second pass over `LoadPrefabContents`.
- **The results scene has no camera, on purpose.** The track is still loaded underneath and
  carries no camera; the player's camera came from the spawned car in the race scene. A
  `cullingMask = 0` camera here clears the shared target and renders no replacement, which is
  what blanked the qualifying screen once. The opaque overlay needs no camera. There is a test
  asserting the absence — do not "fix" the console warning by adding one.
- **No Main Menu button, deliberately.** `_mainMenuScene` is empty and the lobby is the hub, so
  the button would route nowhere. `ResultsScreenImpl` keeps the serialized field; leave it null.
- **`Transform.Find` only searches direct children.** The unlock labels are children of
  `UnlockPanel`, not of the screen root. Asking the root reports them as missing when they are
  present and wired.
- **`F1.UI` is in `Assembly-CSharp`,** which no test assembly can reference. Tests reach it by
  class name, as `WingScreenUiTests` does. An asmdef cannot reference the predefined assembly.
- **`CompleteRace` is not unit-tested against the real profile.** It writes to
  `player_profile.json` in the editor. The double-award guard is tested by flagging a result as
  already recorded and calling in, which never reaches the write. Closing this properly needs
  an injectable profile path — a known gap, unchanged.

## 8. NOT verified — needs a real drive

Everything automated is green, but the finish **trigger** has not been exercised in play:

1. Start from `00_LoadingScene` (entering a flow scene directly gives `flow == NULL`).
2. Lobby → Next → track (Monaco) → wing → qualifying → race.
3. Confirm the HUD says **Lap 1 / 3**.
4. Drive 3 laps. On the third crossing the car should stop accepting input and the results
   screen should appear showing your position, `+N pts`, and the new total.
5. Press **RETRY** — the race should rebuild and award points on its own merit, not repeat the
   first race's award.
6. Press **CONTINUE** — back to the lobby, with the new balance in the currency readout.

PlayMode was not run: the suite hangs on `SliceS4QualifyingShellTests`, and that stall is known
to be the harness rather than the game.

## 9. What is still missing from the GDD

Not in this pass. In rough order of how much they unblock:

1. **Sector timing** — rules and helpers are written (`TrackDefinition.GetSectorColor`,
   `LapTracker.LapProgress01`, `QualifyingScreen.UpdateSectorTimes`), zero call sites.
2. **Ghost car** — storage and session logic exist; `SetQualifyingTime` is always called with
   a null ghost, the prefab is unassigned, `OnGhostToggled` is an empty method.
3. **Racing line** — the AI line is built; there is no visible line, no off-line penalty, and
   `RacingLineReference` is `{fileID: 0}` on all five tracks.
4. **Auto-throttle** — the GDD specifies auto-throttle and a centrally placed brake button. The
   build has manual W/S throttle, and mobile currently has no throttle button at all.
5. **ERS boost, slipstream, track-limit vibration** — absent outright; slipstream has zero hits
   anywhere in `Assets/`.
6. **Rental is free.** `ProgressionRegistry.TryRentCar` has a comment saying it will check
   currency and then charges nothing.