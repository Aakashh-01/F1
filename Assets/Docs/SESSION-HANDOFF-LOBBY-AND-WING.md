# Session handoff — lobby hub, track screen, and the wing prototype

Written mid-session 2026-09-27, at the user's request, so the work can resume in a fresh
session. **Read this before touching anything.**

Companion documents: `PLAN-LOBBY-HUB-AND-UI.md` (the plan) and
`SESSION-HANDOFF-3D-LOBBY-AND-UI-LAYOUT.md` (the original handoff, now stale — it was
written before any of the lobby work and describes a flow that no longer exists).

---

## 0. Read this first — the lobby scene is hand-owned

**`Assets/Scenes/05_LobbyScene.unity` is hand-authored and must not be regenerated,
reloaded casually, or entered into play mode.**

The user has hand-placed and tuned the garage, the car, the camera and the key light, and
has re-tweaked the camera more than once. Two things have already been destroyed by
automation that should not have touched it:

- a table the user had placed behind the car, lost when play mode was entered on the scene
  with unsaved changes (play mode discards them)
- earlier, the user's own camera placement

**Rules for the rest of this work:**
1. Never enter play mode on `05_LobbyScene` without asking.
2. Never run a scene builder against it. `Tools > BuildFlowScenes` is destructive to every
   flow scene; `Tools > BuildLobbyScene` overwrites the rig.
3. Ask before any action that could discard unsaved work.
4. At the time of writing the scene had **unsaved changes** (the user's camera tweak). If it
   still reports `dirty=True`, flag it rather than saving or reloading over it.

The wing work goes in `30_WingSetupScene` and does not need the lobby scene at all.

---

## 1. Where the flow is now

```
00_Loading  ->  05_LobbyScene  --  car tiles straight away, no click first
                                        --  Next     -->  20_TrackSelection
            20_TrackSelection  --  pick track  -->  30_WingSetup  ->  40_PreRace ->  50_Race
```

The lobby **is** the 3D garage, and car selection is its only screen — the tile strip, Next
and the currency readout. The car never moves. `10_CarSelectionScene` was deleted and
removed from the build list.

**Changed 2026-09-30:** the lobby used to open on a hub panel (currency, Settings, Tasks,
START RACE) that had to be clicked through to reach the tiles. That click is gone and the
lobby now opens directly on car selection. The hub panel, Start Race, Settings, Tasks and
Back are deleted. Settings and Tasks were already dead ends, so nothing functional was lost.

`GameScreen.CarSelection` **must stay at enum index 2** — see §6.

## 2. Verified working (rendered and looked at, not just asserted)

- `00_Loading -> 05_LobbyScene`, car tiles up on arrival, car in the garage
- Tile click marks that tile and enables Next; Next goes to `20_TrackSelectionScene` with the
  session car carried over
- Back returns to the hub without navigating
- Track screen: Monaco hero centre, four circuits flanking, selecting Monaco reaches
  `30_WingSetup`

A **rejected** car pick (a rental with no active rental) leaves nothing marked and Next
disabled. This was a real bug: the screen marked a tile optimistically and the flow then
rejected it. Fixed by pushing the flow's verdict back through `ILobbyScreen.SetChosenCar`.

## 3. Test status

- EditMode: 8 of 9. The one failure is the pre-existing `TrackScene_HostIsFixedToRace`,
  which the original handoff says belongs to Phase 5 and must not be "fixed" early.
- PlayMode (`Phase3FlowSceneTests`, `SliceS1SelectionFlowTests`, `SceneFlowServiceTests`):
  34 of 37. Three pre-existing failures, none involving the lobby:
  `FutureFlowScenes_AreNamedButNotYetBuilt` (asserts `40_PreRaceScene` does not exist; it
  does), `HostScreenMode_QualifyingOrRace_FollowsTheSessionType` (a scene-operation
  collision on `50_RaceScene`), and `LeavingTheLobby_UnregistersItsHost` (the FlowSceneHost
  Awake-ordering error, present in the console before any test was touched).
  The last two have **not** been proven against a clean pre-change baseline.

## 4. The wing prototype — findings, and what was decided

**Decision taken by the user: tighter camera on the wing, with BOTH the main rear wing plane
and the DRS flap moving. BUILT AND VERIFIED — the difference reads clearly at 16° vs 3°.**

`30_WingSetupScene` now has the garage rig read live from the lobby, a rear three-quarter
camera at `(1.55, 1.32, -3.15)` looking at `(0.5, 0.92, -1.64)` fov 46, and a
`WingAngleDriver` that rigs both hinges in `Awake`. Verified in play: one canvas, backdrop
off, both hinges rigged, and the two poses are visibly different.

Findings that drove it, all measured rather than assumed:

| Element | Size | Travel | Reads on screen? |
|---|---|---|---|
| `DRS1_165_250` (DRS flap) | 0.78 × 0.12 × 0.29 m | 5.7 cm alone | **No** alone — ~7 px, dark on dark |
| `rearwing_top_62_90` (main plane) | 1.12 × 0.49 × 0.98 m | 0.10 m | **Yes**, and clearly at close framing |

- The DRS flap exists on **one side only** (centred at x=+0.51). The fixed wing is
  symmetric; this moving element is not. So the handoff's recommended "animate the existing
  DRS flap" does **not** work on this model.
- The main plane reads on its own; adding the flap at 1.6x the main angle widens the gap
  further without making the motion silly.
- `DRS2_75_118` is a 3 cm part, **not** the flap. Do not animate it.
- Final angles: high 16°, low 3°. Earlier 6°/30° read but 30° made the endplates stand
  vertically and look broken.

Screenshots: `wing_scene_high.png` and `wing_scene_low.png` (the working pair), plus
`wing*.png` from the prototype stage.

## 4a. Systemic defect found — duplicate screens in four flow scenes — FIXED 2026-09-27

`Tools > InstallScreensIntoScenes` installs a screen prefab as a scene ROOT, while
`Phase3SceneBuilder` configures the `FlowSceneHost` to instantiate its OWN copy parented to
itself. Any scene touched by both tools therefore had **two live copies of its screen at
runtime**, both receiving `Initialize` and `Show`, and the one drawn on top was not the one
anything was configured on.

**This was the likely cause of the long-standing "the wing screen shows nothing" defect**:
`OpenUpTheBackdrop` only ever edited the hand-placed copy, so the host's runtime copy kept
its opaque `RGBA(0.04, 0.05, 0.08)` background and painted a flat rectangle over
everything behind it.

**All four are now fixed** by `Tools/UI/Remove Duplicate Screen Roots`
(`LobbyBuilder.RemoveDuplicateScreenRoots`). Every flow scene now holds only its
`FlowSceneHost` and the garage rig, and runs the single copy the host instantiates:

| Scene | Removed duplicate root |
|---|---|
| `00_LoadingScene` | `BrandingScreen` |
| `20_TrackSelectionScene` | `TrackSelectionScreen` |
| `30_WingSetupScene` | already clean |
| `40_PreRaceScene` | `QualifyingScreen` |

The pass opens each scene **additively** and closes it again, never making it active, so it
cannot disturb the hand-tuned lobby. It also refuses to delete a root unless that scene's
`FlowSceneHost._screenPrefab` is non-null — the proof the host will instantiate a
replacement. Without that check, a scene whose host had no prefab would be left with no UI
at all.

**Do not re-run `Tools/InstallScreensIntoScenes`.** It is the tool that creates the
duplicate in the first place.


## 5. Hard-won facts about rigging the wing — do not rediscover these

- **`SetParent` silently no-ops in edit mode** on the car's hierarchy. The glTF sits inside
  `Car_Model` inside a prefab instance, and structural edits to those objects are refused:
  no exception, no log, `parent` simply does not change. It works normally in play mode, so
  the rig must happen in `Awake()` at runtime. `WingAngleDriver` already does this.
- **The non-uniform scale is not a problem**, contrary to the original handoff's warning.
  `Car_Model` has scale `(1.8, 0.6, 4)`, but `SetParent(..., worldPositionStays: true)` gives
  the new pivot a **uniform** world scale of (1,1,1) and pushes the non-uniform scale down
  onto the flap, so the pivot's rotation applies *after* the flap's scale. The result is a
  rigid rotation. Verified by the bounding box shrinking in Z as the part tilts.
- The car hierarchy is **793 transforms**; there is exactly one node of each wing name, so a
  failure to find one is never a duplicate-name problem.
- The car model's transform nodes all sit at the model origin with geometry baked into mesh
  vertices. **Measure with `Renderer.bounds`, never with `transform.position`.**

`Assets/Scripts/Lobby/WingAngleDriver.cs` exists and its rigging is correct. It currently
drives the DRS flap only; extend it to also drive the main plane, and add a camera override.

## 6. Two edits that will bite anyone who is not careful

1. **`GameScreen.CarSelection` must stay at index 2.** `FlowSceneHost` serializes its hosted
   screen as this enum's integer and the remaining scenes carry 3, 4, 5, 6. The member is
   retained purely so those integers keep their meaning. It was briefly moved to the end
   during this work and had to be put back.
2. **`Tools > BuildFlowScenes` is destructive** — it rebuilds every flow scene from empty.
   Never run it now that a scene is hand-authored.

## 8. The wing screen's own UI — BUILT 2026-09-27

**Done.** The full plan, now updated with what was actually built and where it departed
from the original design, is in `PLAN-WING-SCREEN-UI.md`. Summary:

- `WingCardImpl` (new) — the 250x200 tile, mirroring `CarCardImpl`. Carries the wing
  artwork, a title, a plain-English consequence line, and a `SelectedIndicator` under the
  same child name the car card uses.
- `WingSetupScreenImpl` — the two bare `Toggle`s are gone, replaced by a `TileStrip` that
  populates one tile per `WingType` from the chosen car's two `WingAeroProfile`s. Tapping
  a tile records the choice *and* eases `WingAngleDriver` to that pose.
- `WingScreenUiBuilder` (new) — `Tools/Wing/Build Wing Screen UI` and
  `Tools/Wing/Aim Wing Camera`. A **separate** builder on purpose: `Tools/BuildUIPrefabs`
  rebuilds all nine prefabs from scratch and would throw away the hand-laid-out track
  screen.
- 8 new EditMode tests, all passing.

**The camera moved**, and this is the one thing to know before touching it again:
`(0.57, 2.00, -3.98)` looking at `(-0.04, -0.45, 0.38)`, fov 62. The room is the limit —
a sweep over distance, bearing, height and fov found exactly one pose where the car clears
a 200-unit bottom strip *and* the header. Measure with **per-renderer** bounds, never a
single AABB around the car: the car is yawed, so its box corners project ~0.2 of the frame
below the bodywork and report overflows that are not there.

`bg_wingsetup.png` was never needed — the garage is the backdrop.

## 8a. A latent crash fixed in passing

`WingAngleDriver.Apply()` dereferenced both pivots with no guard. `Update()` checked
`IsRigged` first, but the public `SetAngleImmediate` path did not — and the wing screen now
calls `SetHighDownforceImmediate()` directly to pose the wing to the session's current
setup. An unrigged driver (wrong model, renamed parts, or any driver outside play mode,
since rigging is deferred to `Awake`) would have thrown. `Apply()` now returns early when
`!IsRigged`; the pose is still recorded in `_angle`/`_target`, so if the rig appears later
`Update` eases to the pending target on its own.

## 8c. The UI is now skinned with SlimUI — the camera is HAND-OWNED

**The user hand-placed the wing scene's camera on 2026-09-27 and it is theirs.** It is a
close side shot that fills the frame, replacing the measured pose this file previously
recorded. Consequences:

- `WingScreenUiBuilder.AimWingCamera` is now behind a confirmation dialog and renamed
  `Tools/Wing/Restore Measured Wing Camera (overwrites hand-placed camera)`. It restores the
  old measured values and is kept only as a record. **`BuildAll` does not touch the camera**,
  so `Tools/Wing/Build Wing Screen UI` is safe to re-run freely.
- The tile strip moved from bottom-centre to the **right side**, because the car now fills
  the frame and a bottom-centre strip sat on its flank. The right side, over the dark
  cabinets, is the only region the car does not reach.
- The old note in §8b about the wing being less dramatic no longer applies — the framing is
  the user's choice and reads well.

**Skin:** `Assets/Editor/SlimUiSkin.cs` holds the palette and sprite paths for the SlimUI
"3D Modern Menu UI" pack, shared by the wing, car, track and lobby builders. Applied to the
wing screen, both card prefabs, and the lobby hub. The track screen's own text stays light
because it sits on its dark backdrop; only its cards were skinned.

Five findings about that pack that cost time and are worth not rediscovering — see
`f1-slimui-art-quirks` in the project memory, or the long comment at the top of
`SlimUiSkin.cs`. The short version: it is a whole menu system rather than a sprite kit, its
panel sprite is 50% alpha, its button sprite is an outline with a transparent interior, and
it is a **light** theme, so every label that was near-white is now dark ink.

**Safe to run:** `Tools/Wing/Build Wing Screen UI`, `Tools/BuildClientDemoUI`,
`Tools/Lobby/Build Lobby Hub Screen Prefab`, `Tools/Wing/Build Track Selection Layout`,
`Tools/UI/Remove Duplicate Screen Roots`.
**Never run:** `Tools/BuildUIPrefabs` (rebuilds all nine prefabs from scratch),
`Tools/BuildFlowScenes`, `Tools/InstallScreensIntoScenes` (it is what creates the duplicate
screen roots), `Tools/Lobby/Build Wing Setup Lobby` and `Tools/Lobby/Build Lobby Scene`
(both rebuild a garage rig, and the camera is now hand-tuned).

## 8d. Button placement and palette — the user's, measured from their screenshots

Two screenshots in `Assets/Scenes/` (`Screenshot 2026-09-27 032721.png` for the lobby,
`032811.png` for the wing scene) are the authority on layout. They were not eyeballed: the
light panels were found by thresholding each image, identified by mean colour (`#EAEBEC`
white secondary, `#F3E2B1` amber primary, a pure `#FFFFFF` block at 1.01 fill for the wing
strip), and converted from pixels to canvas units.

Lobby — the currency readout keeps its measured top-right placement (280x48 at 60,70). The
hub's TASKS, SETTINGS and START RACE buttons were removed on 2026-09-30 along with the hub
panel itself; NEXT stays bottom-right at 240x72.

Wing screen — BACK moved to the **top left** (200x62; it was bottom-left, where the close
side shot put it across the car's front wing), CONTINUE 233x73 bottom-right, tile strip
476x181 on the right. Tiles are sized to that strip: 227x170 each, artwork panel 88, blurb
14px in a two-line band.

**Palette muted after feedback that the panels and buttons were too bright and competed with
the card artwork** — the picture is the thing worth looking at. Panel fill 0.80 -> 0.66, art
backing 0.88 -> 0.76, secondary button 0.98 -> 0.72, primary amber knocked back to
0.78/0.63/0.30, white panel overlay over each card 55% -> 30%. `InkMuted` was darkened at the
same time (#4A5260 -> #343842), because muting the panel without muting the secondary labels
breaks 14px text quietly; it now sits at about 5.6:1.

## 8e. Scene cameras — what needs one, and the trap in adding one

**`00_LoadingScene` and `20_TrackSelectionScene` had zero cameras**, and Unity logged a
frame with nothing to render through — the "no camera available" report. Their screens are
Screen Space - Overlay canvases over a flat backdrop, so uGUI never needed a camera and the
UI drew fine; what was missing was a surface for the engine to render the frame with.
`Tools/UI/Ensure UI Scene Cameras` adds a flat-clear camera (`cullingMask = 0`) to those two
and to nothing else. Idempotent and additive; each scene is opened additively, never active.

**Three scenes are deliberately excluded, and adding to them breaks the screen:**

| Scene | Camera source |
|---|---|
| `40_PreRaceScene` | the spawned car — `PlayerCarSpawner.Spawn` brings `Main Camera` |
| `50_RaceScene` | the spawned car |
| `Track_01` | the spawned car |

**A flat-clear camera alongside another camera erases it.** `40_PreRaceScene` was wrongly
included in an earlier version of the pass and the whole qualifying screen went black. A
camera that clears its target and then draws nothing is not a neutral bystander: it clears
the shared target and, with `cullingMask = 0`, renders no replacement, so whichever camera
renders last wins — and this one wins with an empty frame.

The mistake was **classification, not the camera**. `40_PreRaceScene` looks like a UI scene
because `QualifyingScreen_Prefab` is a bare overlay HUD with no background, but it is a
driving scene: `PreRaceSceneController` loads the track additively and spawns the car. **A
scene having no camera is not by itself evidence that it needs one — find out where its
current one comes from first.** `F1_Body.prefab` carries a camera.

Every test passed while that screen was black. Only a screenshot caught it.

## 8f. The loading screen's artwork

**New artwork.** `Assets/Art/Backgrounds/bg_loading_v2.png` replaces `bg_loading.png` — a
dark low-key garage shot with the car lit on the right and a large empty area on the left.
The old night-race photograph was judged too generic, and its hazy centre competed with the
title. As with `bg_trackselect_v2.png`, **the original is still on disk**; nothing was
overwritten. The provider returns 1024x1024, so it was cropped to 16:9 by
`Tools/CropBackgroundsToWidescreen`, which skips anything already widescreen.

**The progress bar was recoloured in the same pass** and was the loudest thing on the new
image: a hard `RGBA(0.878, 0.024, 0.000)` red across a cool, dark photograph. It now uses
the same amber accent as Start Race and Continue, so the bar and the car's rear-wing accent
lights read as one thing. Note the bar lives under **`LoadingProgress`**, not `ProgressRoot`.
`Tools/UI/Apply Loading Screen Art` does both the background and the bar, and
`Tools/UI/Remove Loading Screen Subtitle` deletes the "Select your race" line outright —
object and serialized field both — so nothing can write it back.

**Two console messages on entering play are transient, not defects.** "There can be only one
active Event System" appears during the load-to-lobby handover, when both scenes are briefly
loaded at once; the lobby settles at exactly one EventSystem and one camera with
`Camera.main` resolving. The `FlowSceneHost cannot register: no GameFlowManager exists` line
is the known Awake-ordering issue and predates this work.

## 8g. The qualifying HUD is skinned — and the two halves need opposite treatment

`Tools/UI/Skin Qualifying Screen`. It is a different kind of surface from the rest, and
splitting it is the whole point:

- **The lap clock floats over live 3D.** No panel behind it by design — UIBuilder's own note
  says a full-screen background "would hide the driving entirely", and the point of the
  screen is watching the lap. So it cannot be given a dark tile, and **a colour alone will
  not carry it**: white reads against the dark garage and vanishes against the sky the
  track scenes have plenty of. It uses a real material asset, `Assets/Art/UI/HudText.mat`
  (TMP underlay, 0.85 black, 0.7 offset), assigned as `fontSharedMaterial`.
- **The results panel IS a tile**, and was already a dark one. It moved onto the same deep
  indigo as the selection cards, with the same light ink, and its three buttons took the
  same three-layer treatment. Go to Race is the primary and carries the amber accent; Back
  and Retry stay neutral.

**Two TMP traps hit here, both of which fail silently:**

1. **You cannot author a prefab TMP shadow through `text.fontMaterial`.** It throws in edit
   mode on a prefab — *"The variable m_sharedMaterial of TextMeshProUGUI has not been
   assigned"* — and even where it does not, a runtime material instance is not a project
   asset, so nothing is written to disk. The first attempt enabled the keyword this way and
   the HUD looked fine **only because the test sky happened to be dark enough**. The underlay
   has to be a material asset assigned to `fontSharedMaterial`.
2. **`AssetDatabase.CreateFolder` takes ONE child name under ONE existing parent.** Passing
   a nested path like `"Art/UI"` fails; walk the segments instead.

Also worth knowing: on a re-run the button children came back as `Label[0] ButtonFrame[1]`
— the frame's art drawing over its own label — because the first run had created them in
the wrong order before the ordering fix landed. Re-running healed it. The sibling order in
`SlimUiSkin.ApplyButtonVisual` and in both card builders is load-bearing and is now pinned
explicitly rather than left to creation order.

## 8b. Known trade-off, for the user

The wing is less dramatic in this framing than in the original tight three-quarter. 16° vs
3° still reads when compared side by side and the easing motion is unmistakable, but the
wing is a smaller part of the frame. Clearing a bottom strip in an 8.3 x 8.8 m room forces
the camera back. If a punchier wing matters more than an unobstructed strip, the tiles can
be shortened (the 2:1 artwork sets the floor at 200 tall) or allowed to overlap the car's
lower bodywork.


## 9. Remaining work, in order

1. ~~Fix `CarScenePath`~~ **DONE** — now `RigSourceScenePath`, pointing at
   `05_LobbyScene.unity`, and read-only.
2. ~~Build the wing lobby in `30_WingSetupScene`~~ **DONE** — garage rig, tight rear camera,
   `WingAngleDriver` on the car.
3. ~~Extend the driver to both parts, pick final angles~~ **DONE** — 16° / 3°, verified.
4. ~~**Wing screen UI**~~ **DONE 2026-09-27** — see §8. Tiles, artwork, selected state,
   buttons clear of the car, wired to `WingAngleDriver`.
5. ~~**Decide about the other three duplicated screens**~~ **DONE 2026-09-27** — see §4a.
   All four flow scenes now run a single screen copy.
5a. **ONE ACTION LEFT FOR THE USER, 2026-09-27.** The qualifying HUD kept showing a
   phantom `17.999` lap *after* the tracker fix, because the value is **persisted, not
   recomputed**. `player_profile.json` holds
   `"bestLapTimes": { "track_monaco": 17.999996185302736 }` and
   `"ghostLapData": { "track_monaco": "ghost-data" }`. The ghost string is the literal
   argument the PlayMode tests pass, so **the tests have been writing to the real save**.

   **Fix:** delete `C:/Users/aakas/AppData/LocalLow/DefaultCompany/F1/player_profile.json`.
   `PlayerProfileManager.Load` recreates it via `CreateNewProfile` -> `GrantStarterContent`,
   granting the one free starter car and every free track — identical to what the file
   already had, so nothing is lost. Writing to the save needs the user's authorisation, so
   it was deliberately left to them.

   **This will recur** after any test run that touches the session. The real fix is an
   injectable profile path so tests use a temp file; not done.

5b. **The build is fixed but untested at runtime.** `SceneAvailability` called
   `Application.CanStreamedLevelUnlocked`, which does not exist — the API is
   `CanStreamedLevelBeLoaded`. It sat in a `#else` arm, so the editor compiled the *other*
   arm and all 89 tests passed while the player build had never compiled. `Builds/F1Client.exe`
   now builds. **It has not been run.** The build log also warns
   *"No RuntimePipelineConfig asset found — Pipeline will be disabled in Player builds"*,
   so URP may behave differently in the built game than in the editor. Worth launching the
   exe once before the demo.
6. Optional, waiting on the user: delete `CarSelectionScreen_Prefab.prefab` (unused, not
   authorised for deletion); delete the dead `LobbyScene.unity` + `LobbyManager.cs` pair.

## 9. Things that will waste time if forgotten

- Do not overwrite the user's art in place. Work on copies; the original
  `bg_trackselect.png` is still on disk beside the replacement `bg_trackselect_v2.png`.
- `Phase3SceneBuilder.CreateScene` **opens** an existing scene rather than replacing it, and
  `PrefabUtility.InstantiatePrefab` bypasses `EnsureRoot`'s dedupe, so re-running a builder
  stacks duplicate objects. The lobby scene builder already clears its own rig first.
- `UnityEventTools` only writes a persistent listener when the target is already a prefab
  asset. Wiring buttons before the first `SaveAsPrefabAsset` yields inert buttons. The hub
  builder saves, reopens the prefab contents, wires, and saves again.
- The card is now a **tile** (250 × 280, art panel 132) shared by the car lobby and the
  track screen. Track cards are resized per role at runtime by
  `TrackSelectionScreenImpl.ResizeCard`, which relays out the artwork *and* all five labels —
  the builder's label positions are fractions of card height and slide into the picture on a
  taller card.
- Entering a flow scene directly gives `flow == NULL` and no cards populate. Start from
  `00_LoadingScene` when testing screens.
- `execute_code` runs on the main thread; `Thread.Sleep` inside it blocks async scene loads
  from progressing. Let Unity tick between MCP calls instead.
