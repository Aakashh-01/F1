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

## 2. What the user wants next — DECIDED

The user has answered the open questions. **These are settled, do not re-ask them.**

| Decision | Answer |
|---|---|
| One 3D car per tier, or livery swap? | **One car for all tiers now. Livery swap is a later task — explicitly deferred, not forgotten.** |
| Garage technique? | **Real geometry with parallax. Import a 3D garage, do NOT fake it with a cheap skybox panorama.** |
| Rotation interaction | **Drag to spin.** |
| Track screen arrangement | **The one available track is featured at the CENTRE. The other four sit beside it.** |
| The 95 MB originals | **Deleted.** Done — see §11. |
| Wing lobby | **Still open — see §5 for what the question actually is.** |

Five work items, in the user's words:

1. **3D lobby for car selection** — a 3D garage, car 3D asset at the centre, player drags
   to rotate it
2. **3D lobby for wing selection** — same treatment
3. **Car selection layout** — tiles move to a **horizontal strip along the bottom**, like
   the reference. Current canvas layout is "not good enough".
4. **Track selection** — available track centred, other four beside it
5. **Wing screen has no visuals** — tiles and background not visible

The user also said the gameplay itself is good; this is a **presentation** pass.

### Note on "real geometry" for the garage

The user rejected the panorama skybox in favour of imported geometry. That means an
asset has to be sourced — there is no garage mesh in the project. Whether that is
purchased, downloaded, or generated as 3D is not yet decided and is the one genuinely
open logistics item for the car lobby.

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

## 4. Item 2 — the 3D wing lobby, explained properly

The user asked for a re-explanation, so here it is in plain terms.

### What the wing feature currently is

The game offers the player exactly **two** wing setups:

- `HighDownforce_Aero` — more downforce, slower through corners, the "safe" setup
- `LowDownforce_Aero` — less downforce, faster on the straights, the "fast" setup

They are two `WingAeroProfile` ScriptableObjects, and they are **pure numbers** —
`downforceCoeff`, `frontBias`, drag values. There is no 3D object attached to either one.

### What the problem is

If you build a 3D wing lobby, something has to *appear* on screen when the player picks
one of the two. And the honest answer is: **the project has no wing model to show.**

I checked the car model, and there is good news and a catch.

**Good news:** `F1_Body.prefab` is not one fused mesh. It is a glTF import of 793
transforms, and the wings are separate, named objects you can address individually:

| Object name | What it is |
|---|---|
| `a_frontwing_fl_01_33_54` | front wing |
| `frontflap_fl_26_42` | front flap |
| `rearwing_top_62_90` | rear wing main plane |
| `rearwingmoving_top_2_74_119` | the **moving** rear wing element (under `DRS2_75_118`) |
| `a_rearwing_top_105_166` | rear wing, second instance |
| `wing_mirror_l_66_99` | left wing mirror |

**The catch:** there is only **one** set of wings. Not two variants. So "high downforce"
and "low downforce" are not two different models sitting in the scene waiting to be
shown — they are the *same* wing parts in two different states.

### So the actual question for the user

The wing lobby is only worth building 3D if the two choices produce a **visible
difference**. Realistic ways to get one, cheapest first:

1. **Animate the existing wing.** The car already has a moving rear wing element. Tweak
   the rear wing flap angle between the two settings — more angle reads as more
   downforce. Needs no new asset, just a rotation on an existing object. Closest to
   "real", least work.
2. **Toggle parts on and off.** Show/hide `rearwingmoving_top_2` or the front flap so the
   two configurations are visibly different silhouettes. Also no new asset.
3. **Import two wing models.** Most convincing, needs an asset, and then they have to be
   attached to the car at the right mount points — the car has no wing swap system.
4. **Do not build a 3D wing lobby.** Keep wing selection a 2D screen, but fix it so it
   actually shows the two wing images (see §6b — right now it shows nothing at all).

My recommendation is **1 or 2**, because the wing parts are already addressable and
either is a small amount of work against option 3's asset hunt. But this is a design
call and the user should pick — the reason it is still open is that option 4 is a
perfectly reasonable answer and I do not want to assume a 3D lobby is mandatory.

### The wing images, for the tile strip either way

`Assets/Art/Wings/wing_highdownforce.png` and `wing_lowdownforce.png` exist, are
assigned to the two profiles' `icon` fields, and are visibly distinct (different
endplate structures). Neither reads as a "slim low-drag blade" — both are multi-element.
If wing selection stays 2D, these are the tile art and no regeneration is needed.

---

## 5. Item 4 — track selection layout

Current state: two `VerticalLayoutGroup` columns, `OwnedColumn` anchored 0.1–0.4 and
`LockedColumn` 0.6–0.9, both 0.05–0.85, `UpperCenter`, spacing 10.

With 5 tracks that produces a ragged 2×3 grid with one hole — which is what the user is
reacting to. Captured screenshot showed Monaco top-left, Monza top-right, Spa mid-left,
Silverstone mid-right, Suzuka bottom-right.

**DECIDED: the one available track (Monaco) is featured at the centre, the other four sit
beside it.** The "owned vs locked" column split is what creates the hole and has to go —
the playable/locked distinction is already carried per-card by the `AVAILABLE` vs
`COMING SOON` status line, so it does not need its own column.

The natural read of the decision is a centred hero tile for Monaco, larger, with the four
unbuilt circuits as smaller supporting tiles arranged around or beside it. The exact
arrangement (a row either side, a 2+2 flanking, a strip beneath) is a layout decision for
the implementation, not a question for the user.

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

## 7. Open items

Most questions are now settled (§2). Two remain.

1. **Wing lobby: build it 3D or keep it 2D?** See §4 for the full explanation. The car
   has addressable wing objects, so a visible difference between the two setups is
   achievable without new assets, but "don't build a 3D wing lobby, just fix the 2D
   screen" remains a legitimate answer. **Ask the user.**
2. **Where does the garage geometry come from?** The user chose real geometry over a
   skybox panorama, but no garage mesh exists in the project. Purchase, download, or
   generate? That is logistics, not a design question, and it may need the user.

Also deferred, by explicit agreement, not oversight:

- **Livery swap** so each tier shows a different-coloured car in the lobby. Deferred
  until later; the single shared model is the accepted state for now.
- `SceneContentTests.TrackScene_HostIsFixedToRace` still fails. Belongs to Phase 5, do
  not "fix" it early.
- `50_RaceScene` still has no screen prefab — there is no `RaceScreen_Prefab` in the
  project at all, only the `RaceScreenImpl` script. Building one is a bigger job than
  this pass and was not started.

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

**Unused originals — DELETED.** `car1..6.png`, `Track1..5.png`, `Wing1..2.png` were the
full-size generations (~95 MB, 5-9 MB each). The game imported downscaled copies, and
with the user's approval the originals were removed. `Assets/Art` is now 7.7 MB. The
matching `.gitignore` rules were removed too, so they cannot silently swallow a future
drop of a file with one of those names.

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
