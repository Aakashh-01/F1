# Image generation brief — paste into Gemini

**What this is:** a complete, self-contained brief for generating the 14 images this
project still needs. Paste the whole thing into a Gemini chat that can generate images
(Gemini 2.5 Pro or Flash with image output enabled), then save each result into the
folders named below.

**Total: 14 images.** Three already exist and are not listed here.

---

## How to work through this

1. Paste everything below into one Gemini conversation.
2. Ask for the images **a few at a time** (3–4 per message), not all 14 at once. It
   gives better results per image and makes it far easier to keep the filenames straight.
3. Download each image and save it into the folder shown, with the exact filename given.
4. After saving, **open each one and check it** before moving on. The two things to look
   for are listed under Quality checks.

In Unity, go to **Assets → Editor → BuildClientDemoUI** and then
**Tools → BuildClientDemoScreens** afterwards. No code changes are needed.

---

## The project

A Formula 1 racing game. The player picks a car, picks a circuit, sets up wings, and
races AI opponents. These images are the presentation layer of the front-end menu screens,
so they need to look like one coherent, premium set rather than fourteen unrelated
pictures.

---

## Style guide — every image must follow this

This is what makes the set look designed rather than assembled. Keep it identical across
all 14.

> **Cinematic motorsport photography. Low-key lighting: deep shadows, controlled
> highlights, a subtle rim light separating the subject from the background. Shallow depth
> of field. Muted, desaturated palette weighted toward near-black and charcoal, with a
> single restrained accent colour. Photorealistic. Editorial and premium, not glossy
> advertising.**

### Hard exclusions — important

Image models love adding text, especially to racing cars and liveries. Every prompt below
already says this, but check the output:

- **No text, no letters, no numbers, no sponsor logos, no team branding, no watermarks**
- **No visible brand names** on cars, wings, or trackside furniture
- **No people's faces in focus** — distant crowds are fine, blurred and abstract
- **No car number roundels**

---

# 1. Backgrounds (1 image)

Save to: `Assets/Art/Backgrounds/`
Aspect ratio: **16:9**

### `bg_wingsetup.png`

> Dark Formula 1 pit garage interior at night. A carbon fibre workbench in the middle
> ground, tool chests and equipment softly out of focus in the background. A single
> dramatic overhead work light throws a hard pool of light and leaves everything else in
> deep shadow. Cold blue-grey palette with one warm amber accent. Cinematic low-key
> lighting, shallow depth of field. Empty — no car, no people. Photorealistic. No text, no
> letters, no numbers, no logos, no watermarks.

---

# 2. Cars (6 images)

Save to: `Assets/Art/Cars/`
Aspect ratio: **4:3**

All six are the same underlying car in three generations. Within each generation the two
images use **different camera angles** on purpose — the game lists a rental and a purchase
option for the same car, and identical thumbnails would look like a bug.

### `car_gen1_starter.png` — early 1990s, entry level, three-quarter front

> Professional automotive studio photograph of an early 1990s Formula 1 open-wheel race
> car. Three-quarter front view, slightly low camera angle. White bodywork with bold red
> accents, exposed slick tyres, simple front wing, no halo. Studio lighting with soft rim
> light against a dark seamless charcoal backdrop. Cinematic low-key lighting. Photorealistic,
> sharp detail. No text, no letters, no numbers, no sponsor logos, no watermarks.

### `car_gen1_rental.png` — early 1990s, side profile

> Professional automotive studio photograph of an early 1990s Formula 1 open-wheel race
> car. Full side profile view. White bodywork with bold red and blue accents, exposed slick
> tyres, simple front wing, no halo. Studio lighting with soft rim light against a dark
> seamless charcoal backdrop. Cinematic low-key lighting. Photorealistic, sharp detail. No
> text, no letters, no numbers, no sponsor logos, no watermarks.

### `car_gen2_rental.png` — mid 2010s, three-quarter front

> Professional automotive studio photograph of a mid-2010s Formula 1 open-wheel race car.
> Three-quarter front view, slightly low camera angle. Deep metallic blue bodywork with
> gold accents, exposed slick tyres, complex multi-element front wing, halo cockpit
> protection. Studio lighting with soft rim light against a dark seamless charcoal backdrop.
> Cinematic low-key lighting. Photorealistic, sharp detail. No text, no letters, no numbers,
> no sponsor logos, no watermarks.

### `car_gen2_unlock.png` — mid 2010s, rear three-quarter

> Professional automotive studio photograph of a mid-2010s Formula 1 open-wheel race car.
> Rear three-quarter view, showing the rear wing and diffuser. Deep metallic blue bodywork
> with gold accents, exposed slick tyres, halo cockpit protection. Studio lighting with soft
> rim light against a dark seamless charcoal backdrop. Cinematic low-key lighting.
> Photorealistic, sharp detail. No text, no letters, no numbers, no sponsor logos, no
> watermarks.

### `car_gen3_rental.png` — current generation, three-quarter front

> Professional automotive studio photograph of a current-generation Formula 1 open-wheel
> race car. Three-quarter front view, slightly low camera angle. Matte black bodywork with
> neon cyan accents, aggressive aerodynamic sculpting, exposed slick tyres, halo cockpit.
> Studio lighting with a cyan rim light against a dark seamless charcoal backdrop. Cinematic
> low-key lighting. Photorealistic, sharp detail. No text, no letters, no numbers, no
> sponsor logos, no watermarks.

### `car_gen3_unlock.png` — current generation, side profile

> Professional automotive studio photograph of a current-generation Formula 1 open-wheel
> race car. Full side profile view. Matte black bodywork with neon cyan accents, aggressive
> aerodynamic sculpting, exposed slick tyres, halo cockpit. Studio lighting with a cyan rim
> light against a dark seamless charcoal backdrop. Cinematic low-key lighting.
> Photorealistic, sharp detail. No text, no letters, no numbers, no sponsor logos, no
> watermarks.

---

# 3. Tracks (5 images)

Save to: `Assets/Art/Tracks/`
Aspect ratio: **16:9**

These are menu thumbnails, so the **circuit itself is the subject** — a recognisable
corner or characteristic shape rather than a wide establishing shot. The game's five
circuits are Monaco, Monza, Silverstone, Spa and Suzuka; only one is currently built, but
all five appear on the selection screen, so each needs a distinct silhouette.

### `track_monaco.png`

> Dramatic aerial trackside photograph of a tight Formula 1 street circuit. A slow hairpin
> corner bordered by Armco and grandstands, the harbour and city buildings soft in the
> background haze. Warm late-afternoon light raking across the asphalt, deep shadows in the
> foreground. Cinematic low-key lighting. Photorealistic. No text, no letters, no numbers,
> no sponsor logos, no watermarks.

### `track_monza.png`

> Dramatic aerial photograph of a long high-speed Formula 1 circuit sweeping through dense
> green woodland. Long straights and a fast chicane, trees closing in on both sides,
> dappled sunlight across the asphalt. Cinematic low-key lighting, deep shadows.
> Photorealistic. No text, no letters, no numbers, no sponsor logos, no watermarks.

### `track_silverstone.png`

> Dramatic aerial photograph of a Formula 1 circuit rolling over wide open green
> countryside. Sweeping elevation changes, a long straight falling away into the distance,
> open sky with heavy cloud. Muted natural palette, cinematic low-key lighting.
> Photorealistic. No text, no letters, no numbers, no sponsor logos, no watermarks.

### `track_spa.png`

> Dramatic aerial photograph of a Formula 1 circuit climbing steeply through dense dark
> forest. Pronounced elevation change, a blind crest, red gravel run-off traps, mist pooling
> in the valley below. Deep greens and heavy shadows, cinematic low-key lighting.
> Photorealistic. No text, no letters, no numbers, no sponsor logos, no watermarks.

### `track_suzuka.png`

> Dramatic aerial photograph of a twisty technical Formula 1 circuit. A crossover bridge
> passing directly over the track, tight esses and a hairpin, a modern grandstand. Late
> afternoon light throwing long shadows across the asphalt. Cinematic low-key lighting.
> Photorealistic. No text, no letters, no numbers, no sponsor logos, no watermarks.

---

# 4. Wings (2 images)

Save to: `Assets/Art/Wings/`
Aspect ratio: **4:3**

The wing setup screen offers a high-downforce and a low-downforce setup. **These two must
look like the same part family in two configurations** — same camera, same lighting, same
background — so the player reads them as a choice rather than two unrelated objects.

### `wing_highdownforce.png`

> Dramatic close-up studio photograph of a large multi-element Formula 1 front wing in a
> high-downforce configuration. Wide chord, several overlapping aerodynamic flaps, tall
> endplates. Matte carbon fibre with restrained gold accents. Studio lighting with soft rim
> light against a dark seamless charcoal backdrop. Cinematic low-key lighting. Photorealistic,
> sharp detail. No text, no letters, no numbers, no sponsor logos, no watermarks.

### `wing_lowdownforce.png`

> Dramatic close-up studio photograph of a slim low-drag Formula 1 front wing in a
> low-downforce configuration. Narrow chord, a single thin blade, minimal elements, low
> profile endplates. Matte carbon fibre with restrained gold accents. **Identical camera
> angle, identical lighting and identical backdrop to the high-downforce wing**, so the two
> read as the same part in two configurations. Cinematic low-key lighting. Photorealistic,
> sharp detail. No text, no letters, no numbers, no sponsor logos, no watermarks.

---

# Quality checks — do these before saving

1. **Stray text.** Zoom into the car and wing images. Anything that looks like lettering,
   a number, a badge or a sponsor block must be regenerated — this is the single most
   common failure and it is very visible on a menu tile.
2. **Six cars must read as three generations, two angles each.** They should clearly be
   the same car in each pair, and clearly different between generations.
3. **The two wings must be visibly the same object in two configurations.** If the camera
   or lighting differs, regenerate the second one.
4. **The five tracks must be visibly different circuits.** If two look alike, regenerate.
5. **Consistency.** Spot-check any image against the three that already exist — they use
   the same style language. If an image looks brighter, flatter or more saturated than the
   rest, regenerate it.

---

# Checklist

| # | File | Folder | Ratio |
|---|---|---|---|
| 1 | `bg_wingsetup.png` | `Assets/Art/Backgrounds/` | 16:9 |
| 2 | `car_gen1_starter.png` | `Assets/Art/Cars/` | 4:3 |
| 3 | `car_gen1_rental.png` | `Assets/Art/Cars/` | 4:3 |
| 4 | `car_gen2_rental.png` | `Assets/Art/Cars/` | 4:3 |
| 5 | `car_gen2_unlock.png` | `Assets/Art/Cars/` | 4:3 |
| 6 | `car_gen3_rental.png` | `Assets/Art/Cars/` | 4:3 |
| 7 | `car_gen3_unlock.png` | `Assets/Art/Cars/` | 4:3 |
| 8 | `track_monaco.png` | `Assets/Art/Tracks/` | 16:9 |
| 9 | `track_monza.png` | `Assets/Art/Tracks/` | 16:9 |
| 10 | `track_silverstone.png` | `Assets/Art/Tracks/` | 16:9 |
| 11 | `track_spa.png` | `Assets/Art/Tracks/` | 16:9 |
| 12 | `track_suzuka.png` | `Assets/Art/Tracks/` | 16:9 |
| 13 | `wing_highdownforce.png` | `Assets/Art/Wings/` | 4:3 |
| 14 | `wing_lowdownforce.png` | `Assets/Art/Wings/` | 4:3 |

Already done, do not regenerate:
`Assets/Art/Backgrounds/bg_loading.png`, `bg_carselect.png`, `bg_trackselect.png`
