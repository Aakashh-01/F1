# Plan — the wing screen's own UI

Written 2026-09-27 at the user's request. **No development has been done on any of this** —
it is a plan for a fresh session to pick up. Read
`SESSION-HANDOFF-LOBBY-AND-WING.md` first for the state of everything around it.

This is item 5 of the original five, and the last one outstanding.

---

## 1. What is wrong today

`30_WingSetupScene` now correctly shows the 3D garage with the car, and the wing genuinely
animates between High and Low downforce. The 3D half works and is verified.

The UI on top of it does not:

- The screen has **two 200x40 `Toggle`s and nothing else**. No artwork, no selected state,
  no indication of which setup is active beyond a toggle tick.
- They are positioned **over the car**, so the controls sit on top of the thing they control.
- The generated wing art is **invisible by construction**. `WingAeroProfile.icon` is a public
  field with the tooltip "Artwork shown on the wing setup tile", and both profiles have a
  sprite assigned — and **no line of code anywhere reads it**. Verified by grep, not assumed.
- There is no `WingCardImpl` or equivalent, unlike the car and track screens which both have
  one.

So the tiles were always the intent; they were simply never built.

## 2. What already exists to build on

| Thing | Where | Note |
|---|---|---|
| `WingType` enum | `CarDefinition.cs` | `HighDownforce = 0`, `LowDownforce = 1` |
| `CarDefinition.HighDownforceAero` / `.LowDownforceAero` | `CarDefinition.cs` | the two `WingAeroProfile`s |
| `CarDefinitionExtensions.GetWingProfile(wing)` | `CarDefinition.cs:99` | already resolves either profile from a `WingType` |
| `WingAeroProfile.icon` | `WingAeroProfile.cs` | sprite, assigned on both profiles |
| Wing artwork | `Assets/Art/Wings/wing_{high,low}downforce.png` | visibly distinct per the original inspection |
| `WingAngleDriver` | `Assets/Scripts/Lobby/` | rigs in `Awake`, 16° / 3°, drives both wing parts |
| `IWingSetupScreen.OnWingSelected` | `ScreenInterfaces.cs` | already exists, already fires |
| Recommendation text | `WingSetupScreenImpl.GetWingRecommendation` | works, based on track length |

`WingSetupScreenImpl` already has the right *behaviour* — `ApplyWing` calls
`_flow.SelectWing(wing)` and raises `OnWingSelected`. Only the *presentation* is missing.
Keep `ApplyWing`; replace what calls it.

## 3. The design

**Two wing tiles, and the 3D wing responds to them.** The car is the preview; the tiles are
the control. Selecting a tile both marks it and re-poses the wing, so the player sees the
consequence rather than reading a stat line.

### 3a. Layout — DECIDED: bottom strip, car lifted slightly

The camera is deliberately tight on the rear wing, and the car fills the middle of frame
with floor visible at the bottom corners.

**Chosen (user, 2026-09-27): lift the car slightly and run the two tiles side by side along
the bottom edge**, with Continue and Back below them. This is the Traxion pattern and it
matches the bottom tile strip the player has already learned in the car screen.

Implementation notes:

- Lift the car by raising the camera's look-at point, **not** by moving the car. The car is
  positioned by the garage rig and its height was derived from bounds so it stands on the
  floor; moving the object risks the same buried-or-floating class of bug the car lobby
  already had to fix twice.
- The lift has to be small. This is a close shot on the wing, and a large vertical
  adjustment will lose the rear wing off the top of the frame — the thing the screen exists
  to show. Judge it from a screenshot, not from arithmetic.
- Two 250-wide tiles plus Continue and Back fit the 1920x1080 reference comfortably.
- The tiles must not overlap the car. Overlap is the specific defect being fixed, so check
  it explicitly in the verification screenshots rather than assuming.

**Rejected alternative:** tiles in the bottom corners with the car untouched. Less code and
no camera change, but the two tiles sit far apart and read as unrelated to each other.

### 3b. The tile

Model it on `CarCardImpl` and `TrackCardImpl` rather than inventing a third pattern:

- an `Image` for `WingAeroProfile.icon`, `preserveAspect` on (both images are wide, so a
  square-ish tile letterboxes them rather than smearing — same as the car cards)
- a title: **HIGH DOWNFORCE** / **LOW DOWNFORCE**
- one line of consequence, not raw numbers. The player is choosing a feel, not a coefficient.
  Suggested: *"More grip, slower on the straights"* / *"Faster, less grip through corners"*.
  The existing recommendation text already frames it this way.
- a `SelectedIndicator`, the same child name `CarCardImpl` uses, so the existing selected
  state pattern carries over unchanged.

Size: the car and track cards are 250 x 280 tiles. Two of those side by side is 522 wide,
which fits comfortably, and matches the strip the player already knows.

### 3c. Background — one thing gets simpler

`bg_wingsetup.png` was never generated and is still listed as missing. **On this screen it
is no longer needed.** The garage is the backdrop, exactly as in the car lobby, and the
opaque screen background has already been disabled. Do not generate it; delete it from the
"not generated yet" list in the plan.

## 4. Code changes

### 4a. New `Assets/Scripts/UI/WingCardImpl.cs`

Mirrors `CarCardImpl`. Needs `SetWing(WingType, WingAeroProfile)`, `SetSelected(WingType)`
toggling `_selectedIndicator`, an `OnCardClicked` event, and an `Image` field wired to the
artwork. The `ClientDemoUiBuilder` wiring helpers (`Wire`, `SetAnchor`) are the pattern to
copy — and note the lesson from the hub: **wire persistent button listeners in a second
pass against the saved prefab**, never against a throwaway scene object, or the buttons
silently do nothing.

### 4b. `WingSetupScreenImpl`

- Replace `_highDownforceToggle` / `_lowDownforceToggle` with a tile strip RectTransform and
  a `WingCardImpl` prefab reference.
- `Setup(car, track, currentWing)` already receives the car, so it can resolve
  `car.HighDownforceAero` / `car.LowDownforceAero` and read their icons. Preserve the
  existing `SetIsOnWithoutNotify` reasoning in a comment-worthy form: the screen is being
  *configured*, not recording a choice, and a callback that fires before the flow has wired
  the screen dereferences a null flow.
- `OnTileClicked` replaces the two toggle handlers and calls the existing `ApplyWing`.

### 4c. Reaching the wing driver

`WingAngleDriver` lives in `Assets/Scripts/Lobby/`, which is Assembly-CSharp — the same
assembly as the UI scripts, so the screen **can** reference it directly. It cannot be a
serialized field though, because the driver is on a scene object and the screen is a
prefab. Resolve it at runtime instead:

```csharp
_driver = FindFirstObjectByType<F1.Lobby.WingAngleDriver>();
```

and call `_driver.SetHighDownforce()` / `SetLowDownforce()`. Keep these **null-tolerant** —
if the driver is missing the screen must still work as a plain 2D chooser, because that is
the fallback if the 3D ever gets backed out.

Note `WingSetupSceneController` is in `F1.GameFlow`, which is a separate asmdef and **cannot**
see `F1.Lobby`. That is the same boundary that forced `RefreshFromProfile` onto the
`ILobbyScreen` interface rather than a cast. Going through the screen is the consistent
choice.

### 4d. Poses

`WingAngleDriver` already exposes `SetHighDownforce` / `SetLowDownforce`, which ease rather
than snap. Use those, not the `Immediate` variants, so the wing visibly swings when the
tile is tapped. That motion is the whole point of the screen.

## 5. Verification

Per the standing rule: **render it and look.** A UI change that passes every test can still
be 40x too large or sitting on top of the thing it controls.

- Start from `00_LoadingScene`, not the wing scene directly — entering a flow scene
  standalone gives `flow == NULL` and nothing populates.
- Drive: Lobby -> car tile -> Next -> Monaco -> wing screen.
- Confirm, with screenshots: the garage is visible behind, the tiles are clear of the car,
  the artwork is showing, tapping a tile visibly re-poses the wing, and the selected state
  is obvious.
- Add EditMode coverage: each profile's `icon` is non-null, and the screen populates two
  tiles from a car definition.

## 6. Open, for the user

- ~~Layout~~ — **decided**, see §3a: bottom strip, car lifted slightly.
- Whether the tile should show a raw number (`downforceCoeff`) as well as the plain-English
  line. Recommend no — the recommendation text already frames it, and the player is choosing
  a feel. Still open, but low stakes and easy to add later.
- **The three other duplicated screens** (`00_LoadingScene`, `20_TrackSelectionScene`,
  `40_PreRaceScene`) are still doubled and unfixed. Unrelated to this work, but it is the
  same class of defect and cheap to clear in one pass with the helpers already written.

---

## 7. Status — BUILT 2026-09-27

- 3D wing: built and verified (16° / 3°, both parts moving).
- Wing screen UI: **built, rendered and looked at.** All of §4 is done.

### What was built

| File | Note |
|---|---|
| `Assets/Scripts/UI/WingCardImpl.cs` | new — the tile, mirroring `CarCardImpl` |
| `Assets/Scripts/UI/WingSetupScreenImpl.cs` | toggles replaced by a tile strip; drives `WingAngleDriver` |
| `Assets/Editor/WingScreenUiBuilder.cs` | new — builds the tile and the screen layout, wires the buttons, aims the camera |
| `Assets/Tests/EditMode/WingScreenUiTests.cs` | new — 8 tests, all passing |
| `Assets/Editor/UIBuilder.cs` | `BuildWingSetup` updated; the old toggle fields no longer exist |
| `Assets/Scripts/Lobby/WingAngleDriver.cs` | `Apply()` now guards on `IsRigged` (see below) |
| `Assets/Prefabs/WingCard_Prefab.prefab` | new |
| `Assets/Prefabs/WingSetupScreen_Prefab.prefab` | relaid out |
| `Assets/Scenes/30_WingSetupScene.unity` | camera only |

Run with `Tools/Wing/Build Wing Screen UI`, then `Tools/Wing/Aim Wing Camera`.

### Where this departed from the plan, and why

**§3a asked for 250x280 tiles, only a small camera lift, and no overlap with the car.**
Those three cannot all hold. The car is 4.67 m long in a room whose walkable interior is
8.3 x 8.8 m; at the original tight framing it filled 80% of the frame height and ran off
the bottom edge, so a 280-unit strip would have needed a lift large enough to push the rear
wing — the entire reason the screen exists — off the top. Settled with the user on
**250x200 tiles plus a camera pull-back**, which is what the mockup that was chosen depicts.

**The lift is a camera change, not a car move,** as §3a required. The camera is now
`(0.57, 2.00, -3.98)` looking at `(-0.04, -0.45, 0.38)` at fov 62, chosen by sweeping
distance/bearing/height/fov and scoring each candidate by where the car's silhouette landed
in the viewport against the strip and the header. Two things worth recording:

- **A single AABB around the car is a useless proxy here.** The car is yawed, so its box
  corners project ~0.2 of the frame below the bodywork and report an overflow that is not
  there. Per-renderer bounds are what hug the real outline.
- **The room, not the composition, is the limit.** A sweep of every combination of bearing,
  height, distance and fov produced exactly one pose that clears the strip with margin *and*
  keeps the car clear of the header. Broader is not available; do not spend time re-deriving it.

**§3c was right and cost nothing** — `bg_wingsetup.png` was never generated and is not
needed. The garage is the backdrop.

### What the visual pass caught that no test would have

- The first selected-state wash was gold at 0.13 alpha, strong enough that **the selected
  tile read as a larger tile than its neighbour**. They were the same 250x200 throughout.
  Selection is now carried by a border over a 0.07 wash.
- The generated wing art is a near-black wing on a near-black background, so on the card's
  own `0.06` body it rendered as an indistinct smudge. It needed a separate lighter
  `WingArtBacking` — the `Image` holding the sprite cannot supply this, because its colour
  multiplies the sprite and tinting it lighter would darken an already dark picture.

### Known trade-off, for the user

The wing is less dramatic in this framing than in the original tight three-quarter. 16°
versus 3° still reads clearly when the two are compared side by side, and the motion when
easing between them is unmistakable — but the wing is a smaller part of the frame than it
was. The cause is geometric: clearing a bottom strip in this room forces the camera back.
The escape, if a punchier wing matters more than an unobstructed strip, is to shorten the
tiles further (the art is what sets the floor at 200) or to accept the tiles overlapping the
car's lower bodywork.

### Still open from §6

- Whether to show a raw `downforceCoeff`. **Now settled by evidence: no.** Both profiles
  have `downforceCoeff = 5`, so a number would render as *no difference at all* on a screen
  whose whole purpose is showing a difference. The plain-English line stays.
- The three other duplicated screens (`00`, `20`, `40`) — untouched, still needs a
  decision.

### Test status

- New: 8 EditMode tests, all passing.
- Full EditMode: 89 run, 1 failure — `SceneContentTests.TrackScene_HostIsFixedToRace`, the
  pre-existing Phase 5 one the handoff says must not be fixed early.

