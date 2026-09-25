# Client demo art — generation brief

Status: **3 of 17 images generated.** The rest are blocked on OpenRouter credit
exhaustion, not on anything in the project. See "Finishing the set" below.

Generated art lands in `Assets/Art/{Backgrounds,Cars,Tracks,Wings}/`, is imported as a
Sprite, and is wired into the screens and cards by
`Assets/Editor/ClientDemoUiBuilder.cs`. Adding an image is: generate it into the
folder, then re-run **Tools > BuildClientDemoScreens** and **Tools > BuildClientDemoUI**.

## Two things the provider does that matter here

- It **ignores the requested aspect ratio** and returns 1024x1024 every time. Request
  `width`/`height` anyway for documentation, but do not rely on them.
- It **fails with HTTP 402 when several requests are in flight at once.** Generate two at
  a time with a pause between, or most of the batch fails.

## Style anchor

Every prompt carries the same lighting language so the set reads as one system rather
than a folder of unrelated pictures:

> cinematic low-key lighting, deep shadows, subtle rim light, shallow depth of field,
> photorealistic, no text, no sponsor logos, no watermarks

## Already generated

| File | Used by |
|---|---|
| `Backgrounds/bg_loading.png` | 00_LoadingScene via BrandingScreen_Prefab |
| `Backgrounds/bg_carselect.png` | 10_CarSelectionScene via CarSelectionScreen_Prefab |
| `Backgrounds/bg_trackselect.png` | 20_TrackSelectionScene via TrackSelectionScreen_Prefab |

## Still to generate

### Background (1)

**`Backgrounds/bg_wingsetup.png`** → `WingSetupScreen_Prefab`

> Dark Formula 1 pit garage interior at night, carbon fibre workbench, tool chests
> softly out of focus in the background, single dramatic overhead work light, deep
> shadows, cold blue-grey palette with warm amber accent, cinematic low-key lighting,
> shallow depth of field, photorealistic, no car, no text, no logos, no people

### Cars (6) → `CarDefinition._icon`

All six definitions point at the same 3D car prefab, so there is one model and six
artworks to invent. Colour-coded by generation; the two variants of each generation use
different camera angles so a rental and a purchase of the same car do not look like a
duplicated tile.

| File | Definition | Livery / angle |
|---|---|---|
| `Cars/car_gen1_starter.png` | `CarDefinition_car_gen1_starter` | 1990s, white + red, three-quarter front |
| `Cars/car_gen1_rental.png` | `CarDefinition_car_gen1_rental` | 1990s, white + red/blue, side profile |
| `Cars/car_gen2_rental.png` | `CarDefinition_car_gen2_rental` | 2010s, deep blue + gold, three-quarter front |
| `Cars/car_gen2_unlock.png` | `CarDefinition_car_gen2_unlock` | 2010s, deep blue + gold, rear three-quarter |
| `Cars/car_gen3_rental.png` | `CarDefinition_car_gen3_rental` | 2020s, matte black + neon cyan, three-quarter front |
| `Cars/car_gen3_unlock.png` | `CarDefinition_car_gen3_unlock` | 2020s, matte black + neon cyan, side profile |

Prompt template:

> Professional automotive studio photograph of a <era> Formula 1 open-wheel race car,
> <angle> view, <livery>, <era detail>, studio lighting with soft rim light, dark
> seamless charcoal background, photorealistic, sharp detail, no text, no sponsor
> logos, no watermarks

### Tracks (5) → `TrackDefinition._thumbnail`

| File | Definition | Scene | Renders playable |
|---|---|---|---|
| `Tracks/track_monaco.png` | `TrackDefinition_track_monaco` | `Track_01` | **yes** |
| `Tracks/track_monza.png` | `TrackDefinition_track_monza` | `MonzaGP` (missing) | no — dimmed, COMING SOON |
| `Tracks/track_silverstone.png` | `TrackDefinition_track_silverstone` | `SilverstoneGP` (missing) | no — dimmed, COMING SOON |
| `Tracks/track_spa.png` | `TrackDefinition_track_spa` | `Track_01` | no — same circuit as Monaco |
| `Tracks/track_suzuka.png` | `TrackDefinition_track_suzuka` | `SuzukaGP` (missing) | no — dimmed, COMING SOON |

Availability is decided at runtime by `SceneAvailability` from the build settings, not
by anything in the art. Only `Track_01` is in the build list today.

Prompt template:

> Dramatic aerial <trackside|aerial> photograph of <circuit description>, <features>,
> cinematic low-key lighting, <time of day>, photorealistic, no text, no logos,
> no watermarks

### Wings (2) → `WingAeroProfile.icon`

Only two profiles exist: `HighDownforce_Aero` and `LowDownforce_Aero`.

| File | Profile | Description |
|---|---|---|
| `Wings/wing_highdownforce.png` | `HighDownforce_Aero` | large multi-element front wing, gold accents |
| `Wings/wing_lowdownforce.png` | `LowDownforce_Aero` | slim low-drag blade, minimal elements |

Prompt template:

> Dramatic close-up studio photograph of a <high downforce multi-element | slim
> low-drag> Formula 1 front wing, matte carbon fibre with <gold> accents, visible
> aerodynamic flaps and endplates, studio lighting with soft rim light, dark seamless
> charcoal background, photorealistic, sharp detail, no text, no sponsor logos,
> no watermarks

## Finishing the set

1. Top up the OpenRouter key the editor is configured against.
2. Generate the 14 images above, **two at a time**, into the folders named.
3. Set each new texture's importer to **Sprite**, Read/Write **off**, mipmaps **off**.
4. Assign the sprites: cars and tracks into their `ScriptableObject` assets' `Icon` /
   `Thumbnail` fields, wings into `WingAeroProfile.icon`.
5. Re-run **Tools > BuildClientDemoScreens** and **Tools > BuildClientDemoUI**.

Card art is displayed with `preserveAspect`, so square source images are fine there. Only
backgrounds need the 16:9 crop, and `BuildClientDemoScreens` applies it automatically.

## Not doing

Buttons are left as default grey uGUI. A real UI asset pack is to be imported and wired
later; the card and screen sprites are isolated enough that swapping them is a
re-run of the builder rather than a rewrite.
