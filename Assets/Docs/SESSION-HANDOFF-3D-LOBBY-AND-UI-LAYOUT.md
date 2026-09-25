# Session handoff — 3D lobby and UI layout pass

Written at the end of the client-demo UI session, before the 3D lobby work starts.
**No development has been done on anything below — this is a plan, not a changelog.**

Reference image for the lobby concept: `Assets/Scenes/img.jpeg` (Traxion-style car
garage: full-bleed 3D garage, real car angled at centre, tile strip along the bottom).

---

## 1. Where the project stands

**Branch `feature/client-demo-ui`, 6 commits ahead of `main`, NOT pushed.** `main` is
already pushed and is at `52f4770`. The user has been told repeatedly that the branch
is unpushed and has not asked for it to land yet — confirm before pushing anything.

Commits on the branch:

| Commit | What |
|---|---|
| `636576c` | Card artwork slots, card sizing, track availability |
| `e1a968a` | Screen prefabs placed into their scenes + backgrounds |
| `965fb2f` | Gemini generation brief + target folders |
| `2caede7` | Generated art wired into car/track/wing data |

Before that, on `main`: the AI racing line fix (`0364bf3`) and the whole 00–50 game
flow (`52f4770`).

### The big UI finding

Scenes 10/20/30/40 contained **no UI at all** — only a `FlowSceneHost` and an
`EventSystem`. The screen prefabs existed and were fully wired but had never been
placed in a scene, which is why the car selection screen was black. Fixed by
`Tools > InstallScreensIntoScenes`.

### The big track finding

`TrackDefinition` names five circuits; the project contains one. Monaco and Spa both
point at `Track_01`, and Monza/Silverstone/Suzuka point at scenes that do not exist.
Handled by `SceneAvailability` (reads the build settings, so it cannot drift) plus an
explicit `TrackDefinition.UnderConstruction` flag for Spa, which exists but is the
same corner as Monaco. Currently: **Monaco playable, four dimmed with COMING SOON.**

---

## 2. What the user wants next

Five items, in their words:

1. **3D lobby for car selection** — a 3D garage, car 3D asset at the centre, player can
   rotate it
2. **3D lobby for wing selection** — same treatment
3. **Car selection layout** — tiles move to a **horizontal strip along the bottom**, like
   the reference. Current canvas layout is "not good enough".
4. **Track selection** — 5 tracks, arranged so they "look good" and align properly
5. **Wing screen has no visuals** — tiles and background not visible

They also said the gameplay itself is good; this is a **presentation** pass.

---

## 3. Item 1+3 — the 3D car lobby

### The concept

Full-bleed 3D garage environment. The real car model at the centre, angled three-quarter
toward camera, large in frame. Player rotates it. A horizontal tile strip runs along the
**bottom** for car selection — the reference has a strip of slots along the bottom edge.

The user recalls building this before in another project using a **4K/8K panorama
material that turned a whole skybox scene into a 3D garage**. That is the equirectangular
skybox technique: one 2:1 panorama image, `RenderSettings.skybox` set to a Panorama
material, and the "garage" is the image wrapping around the camera. No geometry needed.

### Hard constraint you must know before designing

**All six `CarDefinition` assets point at the same prefab** — guid
`d0603a6a646e83d43b49066eaba6ba0a`, 230 renderers, 6.3 m long. Verified earlier this
session.

That means: pick Gen 1, Gen 2 or Gen 3 in the lobby, and **the same 3D car appears**.
The tile art differentiates the tiers; the 3D model does not. The 6.3 m car is
`Assets/Prefabs/F1_Body.prefab`. `AI_F1_Body.prefab` is the same model rotated 90°
(bounds 6.3 x 1.7 x 2.2 vs 2.2 x 1.7 x 6.3).

So there is a real decision here (see §8) — either accept one model for all tiers, or
add a livery/material swap so picking Gen 2 actually changes the car's colour on the
turntable. Given the tile art already went to the trouble of colour-coding the three
generations, a livery swap is probably worth it.

### What does not exist yet

- **No lobby/garage geometry.** Searched all prefabs for lobby/garage/showroom/hangar —
  none. Must be built or faked with a panorama.
- **No skybox asset.** `RenderSettings.skybox` is `Default-Skybox`. No cubemap or
  render texture assets in the project.
- No turntable/rotation component exists.

### Build notes

- `F1_Body` has a **Rigidbody** and a collider. For a static lobby display it should
  probably be kinematic with gravity off, or have its physics disabled, so it does not
  fall through the floor while the player spins it.
- The car is 6.3 m long — the camera needs real framing, and any fake garage floor needs
  to be at road level (y ≈ 0) so it does not float.
- Rotating: drag-to-spin on the model root is the usual pattern; auto-slow-rotate when
  idle is a cheap alternative and reads well in a demo.
- The lobby needs a **camera**, and the selection scenes currently have none — that is
  also why screen-space-overlay canvases could not be captured earlier without a
  temporary camera.

### Layout change this implies

The car screen's three `VerticalLayoutGroup` columns (Owned / Rentable / Locked,
anchored 0.05–0.35 / 0.35–0.65 / 0.65–0.95, 0.05–0.85) need to become a **single
horizontal strip along the bottom**. That means:
- the columns' `childControlWidth`/`childForceExpandWidth` behaviour has to be inverted
  to a `HorizontalLayoutGroup`
- the card prefab's artwork panel may want to become a square-ish tile rather than the
  current 300x270 wide format
- the "tier" grouping (Owned/Rentable/Locked) is a vertical-column concept; a bottom
  strip has to express it differently — probably by ordering tiles and letting the
  status text on each tile carry the tier, as it already does

---

## 4. Item 2 — the 3D wing lobby

Same treatment. Two choices only: `HighDownforce_Aero` and `LowDownforce_Aero`.

Note there is **no 3D wing model in the project** — only the two `WingAeroProfile`
ScriptableObjects, which are pure data (`downforceCoeff`, `frontBias`, aero numbers).
A wing lobby needs a 3D asset: either a front-wing model, or a cutaway of the car
showing the wing change, or a 3D representation of the aero difference some other way.
This is a genuine gap and a decision to make (see §8).

The generated wing art is good and can be used for the bottom strip tiles:
`Assets/Art/Wings/wing_highdownforce.png`, `wing_lowdownforce.png`. They are visibly
distinct (different endplate structures) though neither reads as a "slim low-drag
blade" — both are multi-element.

---

## 5. Item 4 — track selection layout

Current state: two `VerticalLayoutGroup` columns, `OwnedColumn` anchored 0.1–0.4 and
`LockedColumn` 0.6–0.9, both 0.05–0.85, `UpperCenter`, spacing 10.

With 5 tracks that produces a ragged 2×3 grid with one hole — which is what the user is
reacting to. Captured screenshot showed Monaco top-left, Monza top-right, Spa mid-left,
Silverstone mid-right, Suzuka bottom-right.

The ask: "if we have 5 tracks for now put them in a way that they look good."

Reasonable approaches:
- one centred row of 5 equal tiles
- 3 + 2 centred, second row centred under the first
- keep the two-column split but centre both columns and make the rows line up

Whatever is chosen, the current 2-column "owned vs locked" split is what creates the
hole, so it likely has to go or be rebalanced. **Worth asking the user** rather than
guessing — see §8.

---

## 6. Item 5 — the wing screen is genuinely broken (diagnosed, not fixed)

Two separate defects, both confirmed by inspecting the prefab:

### 6a. No background

`WingSetupScreen_Prefab` root `Image.sprite` is **NULL**. The builder logged
*"Missing sprite Assets/Art/Backgrounds/bg_wingsetup.png; kept flat colour"* — that
image was the one item in the generation brief that never got made (it was the 14th,
and the provider ran out of credit). The screen therefore renders flat dark
`RGBA(0.04, 0.05, 0.08)`. **A `BackdropScrim` child exists but has no sprite either.**

Fix: generate `bg_wingsetup.png` (the prompt is in `Assets/Art/GEMINI-PROMPT.md`,
section 1) and re-run `Tools > BuildClientDemoScreens`.

### 6b. Nothing ever displays the wing artwork

This is the bigger one. The wing screen **has no card or tile system at all**. Its
hierarchy is:

```
WingSetupScreen_Prefab [Canvas, CanvasScaler, GraphicRaycaster, Image, WingSetupScreenImpl]
  BackdropScrim, TitleText, RecommendationText
  HighDownforceToggle  [Image, Toggle]   <- 200x40, no sprite
  LowDownforceToggle   [Image, Toggle]   <- 200x40, no sprite
  ContinueButton, BackButton
```

Two plain **Toggle** buttons, not tiles. The generated art *is* correctly assigned —
`HighDownforce_Aero.icon = wing_highdownforce`, `LowDownforce_Aero.icon =
wing_lowdownforce` — but **no code path ever reads `WingAeroProfile.icon`**. There is no
`Image` component displaying it and no `WingCardImpl` equivalent of `CarCardImpl`.

So the wing art is currently invisible by construction. Fixing this means building wing
tiles with an `Image`, and either a `WingCardImpl` or an extension of
`WingSetupScreenImpl` to populate them — the same shape as the car/track card work.

---

## 7. Open decisions for the user

These came up and were not settled. Worth asking before building.

1. **3D car for all six tiers, or add a livery swap?** One model exists. If the lobby
   shows the same car regardless of tile, that is visible to a client. A material swap
   keyed off `CarDefinition` is the honest fix but is extra work.
2. **Garage by panorama skybox, or real geometry?** The user's 4K/8K panorama memory is
   the cheap route and matches the reference closely. Real geometry costs an asset
   import but gives parallax when the camera moves. Panorama needs a new 2:1 image
   generated.
3. **Rotation interaction**: drag-to-spin, auto-rotate when idle, or both?
4. **Track screen arrangement** for 5 tiles — one row, or 3+2? (see §5)
5. **Wing lobby 3D asset** — there is no wing model. Front-wing asset, car cutaway, or
   skip the 3D wing lobby and make the wing screen a well-styled 2D screen?
6. **Are `car1..6` / `Track1..5` / `Wing1..2` safe to delete?** They are the 95 MB
   full-size originals, now gitignored and unused; the game imports downscaled copies.

---

## 8. Things that cost time this session — do not rediscover them

- **Do not overwrite the user's art files in place.** An automated resize of all PNGs
  under `Assets/Art/` was correctly blocked as irreversible. Work on copies.
- **The OpenRouter image provider is out of credit** ("requested up to 29479 tokens, but
  can only afford 6706"). Art now comes from Gemini via
  `Assets/Art/GEMINI-PROMPT.md`. The provider also fails with **HTTP 402 when several
  requests are in flight** — generate two at a time, and it ignores the requested aspect
  ratio (always 1024x1024).
- **The Unity MCP `execute_code` sandbox has a reduced Unity API surface.** These all
  failed to compile or run: `Graphics.Blit` with a destination `Rect`, `GL.Viewport`
  with 4 args, `Graphics.Blit` with 1 arg, `Texture2D.LoadImage` as an extension
  (call `UnityEngine.ImageConversion.LoadImage` explicitly), and `Scene.Find` (use
  `GetRootGameObjects()`). Contact sheets built with manual `GetPixel` came out black;
  per-tile `RenderTexture` blitting is the way that works.
- **Screen-space-overlay canvases cannot be captured** by a camera render without
  temporarily switching `canvas.renderMode` to `ScreenSpaceCamera` and restoring it.
  That is how every UI screenshot in `Assets/Screenshots/ui_*.png` was taken.
- **The Unity MCP image model is not configured anywhere findable** — not in
  `manifest.json`, not in EditorPrefs under any obvious key. It lives in
  MCP for Unity → Asset Generation.
- **"bunny stealth" / `stealth/space-bunny-alpha` is the model running the Claude Code
  conversation**, not an image model. The user asked to use it for images; it cannot
  generate them and is not on OpenRouter's image endpoint.

---

## 9. UI facts that constrain any layout work

These were discovered by inspection and are easy to trip over again.

- Canvas reference is **1920x1080**, `MatchWidthOrHeight`, factor 1.
- The card columns are `VerticalLayoutGroup` with **`childControlHeight = false`**. The
  layout group therefore *ignores* a card's `LayoutElement.preferredHeight` and keeps
  the card's own rect height. This silently broke the first card layout: cards sat at
  their original 180 units while the art panel was 250, so pictures spilled onto
  neighbours. The card now sets `sizeDelta` explicitly.
- The columns also had **`childForceExpandWidth = true`**, which stretched every card to
  the full column width regardless of its preferred width. Now `false`; card is 300 wide.
- A card is **300 x 270** with a 140-tall art panel on top and labels below. Three cards
  per column is the most the Rentable column ever holds (3 rentals), and 3 x 270 + 2 x
  10 spacing + 12 padding = 830, inside the 864 available. **Any card size change must
  preserve that.**
- The card layout is produced by `Assets/Editor/ClientDemoUiBuilder.cs`, which is
  **re-runnable** via these menu items:
  - `Tools > BuildClientDemoUI` — card size, art panel, label anchors, badge
  - `Tools > WireClientDemoCardArt` — binds the `Image` to the serialized field
  - `Tools > BuildClientDemoScreens` — backgrounds, 16:9 crop, scrim
  - `Tools > InstallScreensIntoScenes` — puts screen prefabs into their scenes
  - `Tools > CropBackgroundsToWidescreen`
- uGUI has **no "cover" fit mode** (`Simple` letterboxes, `Filled` only does directional
  and radial), which is why backgrounds are cropped to 16:9 at build time instead.
- A `GameObject` can carry only one `Graphic`, so an `Image` and a
  `TextMeshProUGUI` cannot share an object. This caused a build failure.
- Card art displays with `preserveAspect`, so square-ish sources are fine there.

---

## 10. Asset inventory

**Generated art, in use** (`Assets/Art/`, all sprites, ~5.9 MB):

| Path | Goes into |
|---|---|
| `Backgrounds/bg_loading.png` | 00_LoadingScene via BrandingScreen_Prefab |
| `Backgrounds/bg_carselect.png` | 10_CarSelectionScene |
| `Backgrounds/bg_trackselect.png` | 20_TrackSelectionScene |
| `Backgrounds/bg_wingsetup.png` | **MISSING — needs generating** |
| `Cars/car_gen{1,2,3}_{starter,rental,unlock}.png` (6) | `CarDefinition._icon` |
| `Tracks/track_{monaco,monza,silverstone,spa,suzuka}.png` (5) | `TrackDefinition._thumbnail` |
| `Wings/wing_{high,low}downforce.png` (2) | `WingAeroProfile.icon` — **assigned but never displayed** |

The cars and tracks were **generated out of order** and remapped by inspecting the
images. The tracks arrived exactly reversed. If art is ever regenerated, inspect before
wiring — wiring by filename put a 2020s car on the free starter and mislabelled four
circuits.

**Unused originals**, gitignored, ~95 MB, safe to delete once the set is signed off:
`Assets/Art/Cars/car1..6.png`, `Tracks/Track1..5.png`, `Wings/Wing1..2.png`.

**Not generated yet**: `bg_wingsetup.png` (see §6a) and any garage panorama.

**Key prefabs**: `Assets/Prefabs/{CarCard,TrackCard,BrandingScreen,CarSelectionScreen,TrackSelectionScreen,WingSetupScreen}_Prefab.prefab`,
`Assets/Prefabs/F1_Body.prefab` (the car model).

---

## 11. Verification habit that has worked

Never assert a UI change works — render it and look. The pattern:

1. `Tools > …` menu item to apply
2. Load `00_LoadingScene` and enter play mode (starting mid-flow gives no session, so
   cards do not populate — the flow has to run from the start)
3. Reflectively call `GameFlowManager.GoToTrackSelection()` etc. to advance
4. Render via a temporary camera with `canvas.renderMode` switched to `ScreenSpaceCamera`

The car screen populates all six cards from `00_LoadingScene` with no intervention.

EditMode suite: **83 tests, one pre-existing failure** —
`SceneContentTests.TrackScene_HostIsFixedToRace`, which asserts `Track_01` carries a
`FlowSceneHost` it has never had; `GameFlowManager` deliberately falls back around this
("Track_01 still finds its screens this way until Phase 5 splits it into flow shells").
Not a regression. Do not try to "fix" it without also doing Phase 5.
