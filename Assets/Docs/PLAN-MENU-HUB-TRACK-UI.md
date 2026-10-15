# Menu Hub — Track Selection UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move track selection into `05_GarageScene` as a second hosted screen so loading → car panel → NEXT → track panel happens in one scene with one shared background, then CONFIRM continues into the unchanged wing-setup scene.

**Architecture:** The hub scene hosts two `FlowSceneHost` roots — `GameScreen.Lobby` (car panel, existing) and `GameScreen.TrackSelection` (new track panel). `FlowSceneNames.TrackSelection` repoints to `"05_GarageScene"`, so `GameFlowManager.GoToTrackSelection()` rides the existing transition machinery: `SceneFlowService.LoadFlowScene`'s same-scene guard (SceneFlowService.cs:243) makes it an instant in-place screen swap, and wing-setup BACK (which calls `GoToTrackSelection`) lands back on the hub automatically. A `GarageHubView` component toggles the two host roots and the showcase car from `GameFlowManager.OnScreenChanged`. Zero `GameFlowManager` changes.

**Tech Stack:** Unity 6000.0.63f1, URP, uGUI + TextMeshPro, Input System (`InputSystemUIInputModule`), Unity Test Framework. Editor work runs through Unity MCP (`refresh_unity`, `execute_menu_item`, `run_tests`/`get_test_job`, `read_console`, `manage_camera`).

**Spec:** Approved in-chat design (2026-10-13) + reference images `Assets/Scenes/ReferenceImageLobby.png`, `ReferenceImageCarselection.png`, `ReferenceImageTrack.png`. User decisions: rebuild camera/car framing per ReferenceImageLobby and delete the current garage room now (new environment asset arrives later) · delete retired scene files from disk · hide the car while the track panel is up · add an authored difficulty field to `TrackDefinition`.

## Global Constraints

- Working directory: `G:\F1-Git_Repo\F1`. Branch: `feature/client-demo-ui`. The tree carries the user's **uncommitted spline/AI WIP** — never `git add -A`/`git add .`; stage only the exact files each task lists. Never touch: `Assets/Scripts/Brain/AIDriver*.cs`, `AILongitudinalPlanner.cs`, `AIRacingLine.cs`, AI tests, `Assets/Docs/PLAN-AI-RACING-PHASE-2-3.md`, `Assets/Docs/SESSION-HANDOFF-SPLINE-RACING-LINE.md`, `Python/`, `.gitignore`, `Packages/packages-lock.json`.
- **Never insert or renumber `GameScreen` enum members** — `FlowSceneHost._hostedScreen` is a serialized int.
- Never hand-edit `.unity`/`.prefab` YAML — all scene/prefab authoring goes through the `Tools/Garage/` builder menus via Unity MCP.
- TMP default SDF atlas guarantees only ASCII — no `◀/▶`-style glyphs; use `"<"`, `">"`.
- Buttons self-wire in `Awake` (the inert-buttons trap) — no UnityEvent two-pass wiring.
- `30_WingSetupScene`, `40_PreRaceScene`, `50_RaceScene`, race stack: **untouched**.
- Out of scope: wing-screen changes, purchasing/rental UI, flag/map art, environment asset integration, the user's AI WIP, the 2 pre-existing PlayMode failures (`FutureFlowScenes_AreNamedButNotYetBuilt`, `HostScreenMode_QualifyingOrRace_FollowsTheSessionType`) — they are baseline.
- Unity MCP steps assume the editor is open on this project; after any `.cs` change: stop play mode if running, then `refresh_unity` before running menus/tests.

---

### Task 0: Baseline

**Files:** none modified.

- [ ] **Step 1: Record the tree.** Run `git status` and save the output to the task notes. Confirm `05_LobbyScene.unity` and `20_TrackSelectionScene.unity` are currently tracked and unmodified.
- [ ] **Step 2: Record test baseline.** Via Unity MCP `run_tests` (EditMode, then PlayMode; poll `get_test_job`). Expected: EditMode all green; PlayMode green except the 2 documented failures listed above. Note the counts.

---

### Task 1: Track difficulty display field + authoring

**Files:**
- Modify: `Assets/Scripts/GameData/TrackDefinition.cs`
- Modify: `Assets/Editor/GarageBuilder.cs` (adds one authoring menu + Build All chain)
- Test: `Assets/Tests/EditMode/TrackMenuDisplayTests.cs` (new)

**Interfaces:**
- Produces: `TrackDefinition.DifficultyLabel` (string property). Task 4's card rendering reads it. Builder menu `Tools/Garage/Author Track Menu Display`.

- [ ] **Step 1: Write the failing test.** Create `Assets/Tests/EditMode/TrackMenuDisplayTests.cs`:

```csharp
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using F1.GameData;

/// <summary>
/// Guards the one-shot authoring of the track panel's display data. Like the car
/// display stats, the values are showroom placeholders, but an empty difficulty cell
/// on every card would read as broken rather than placeholder.
/// </summary>
public class TrackMenuDisplayTests
{
    private static TrackDefinition[] LoadTracks() =>
        Resources.LoadAll<TrackDefinition>("GameData/Tracks")
            .OrderBy(t => t.name)
            .ToArray();

    [Test]
    public void EveryTrack_HasADifficultyLabelAndCoreStats()
    {
        var tracks = LoadTracks();
        Assert.GreaterOrEqual(tracks.Length, 5, "Expected the five circuit assets to be present.");

        foreach (var track in tracks)
        {
            Assert.IsFalse(string.IsNullOrWhiteSpace(track.DifficultyLabel),
                $"'{track.name}' has no difficulty label; run Tools > Garage > Author Track Menu Display.");
            Assert.Greater(track.TrackLengthMeters, 0f, $"'{track.name}' has no length.");
            Assert.Greater(track.StandardRaceLaps, 0, $"'{track.name}' has no lap count.");
        }
    }

    [Test]
    public void DifficultyLabels_AreUppercaseShortLabels()
    {
        foreach (var track in LoadTracks())
            StringAssert.IsMatch(@"^[A-Z]+$", track.DifficultyLabel,
                $"'{track.name}' difficulty should be a short uppercase label like MEDIUM.");
    }
}
```

- [ ] **Step 2: Run it, expect fail.** `run_tests` EditMode filtered to `TrackMenuDisplayTests`. Expected: compile error — `TrackDefinition` has no `DifficultyLabel`.
- [ ] **Step 3: Add the field.** In `Assets/Scripts/GameData/TrackDefinition.cs`, after the `_devBestLapTime` block (before `#if UNITY_EDITOR`), insert:

```csharp
        [Header("Menu Display")]
        [Tooltip("Shown on the track panel's card. Authored placeholder until real difficulty data exists.")]
        [SerializeField] private string _difficultyLabel;
        public string DifficultyLabel => _difficultyLabel;
```

- [ ] **Step 4: Add the authoring pass.** In `Assets/Editor/GarageBuilder.cs`, after `LoadAllCars()` (line ~133), add:

```csharp
    // Asset order is sorted by name: monaco, monza, silverstone, spa, suzuka.
    private static readonly string[] TrackDifficulty =
        { "HARD", "MEDIUM", "MEDIUM", "HARD", "HARD" };

    /// <summary>
    /// One-shot authoring of the track panel's display data, same contract as
    /// AuthorCarDisplayStats: plausible placeholders, applied by sorted asset order.
    /// </summary>
    [MenuItem("Tools/Garage/Author Track Menu Display", priority = 14)]
    public static void AuthorTrackMenuDisplay()
    {
        var tracks = AssetDatabase.FindAssets("t:TrackDefinition")
            .Select(g => AssetDatabase.LoadAssetAtPath<TrackDefinition>(AssetDatabase.GUIDToAssetPath(g)))
            .Where(t => t != null)
            .OrderBy(t => t.name)
            .ToList();

        for (int i = 0; i < tracks.Count && i < TrackDifficulty.Length; i++)
        {
            var so = new SerializedObject(tracks[i]);
            so.FindProperty("_difficultyLabel").stringValue = TrackDifficulty[i];
            so.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(tracks[i]);
        }

        AssetDatabase.SaveAssets();
        Debug.Log($"[Garage] Authored difficulty labels on {System.Math.Min(tracks.Count, TrackDifficulty.Length)} track asset(s).");
    }
```

And extend `BuildAll()` (line ~40) to run it first:

```csharp
        AuthorCarDisplayStats();
        AuthorTrackMenuDisplay();
        BuildGarageScreenPrefab();
```

- [ ] **Step 5: Compile + run the authoring.** `refresh_unity`, then `execute_menu_item` `Tools/Garage/Author Track Menu Display`, then `read_console` to confirm `[Garage] Authored difficulty labels on 5 track asset(s).`
- [ ] **Step 6: Run the test, expect pass.** EditMode `TrackMenuDisplayTests` — both tests green.
- [ ] **Step 7: Commit.**

```bash
git add Assets/Scripts/GameData/TrackDefinition.cs Assets/Editor/GarageBuilder.cs Assets/Tests/EditMode/TrackMenuDisplayTests.cs Assets/Tests/EditMode/TrackMenuDisplayTests.cs.meta
git commit -m "Add an authored difficulty label to TrackDefinition for the track panel"
```

---

### Task 2: Flow seam — `ITrackPanelScreen` + `TrackPanelScreen`

**Files:**
- Modify: `Assets/Scripts/GameFlow/ScreenInterfaces.cs` (append after `ITrackSelectionScreen`, line ~166)
- Modify: `Assets/Scripts/GameFlow/ScreenControllers.cs` (append after `TrackSelectionScreen`, line ~147)

**Interfaces:**
- Produces: `ITrackPanelScreen : ITrackSelectionScreen` with events `OnPrevTrackPressed`, `OnNextTrackPressed`, `OnConfirmPressed`, view method `ShowFocusedTrack(TrackDefinition track, bool isSelected, bool isAvailable)`. Abstract `TrackPanelScreen : TrackSelectionScreen, ITrackPanelScreen` with `TriggerBackPressed/TriggerPrevTrackPressed/TriggerNextTrackPressed/TriggerConfirmPressed` protectors — the base declares `OnBackPressed` for its scene controller but has no protector, so the panel's view-side BACK button needs this one added. Tasks 4/5 consume these exact names.

- [ ] **Step 1: Append the interface.** In `ScreenInterfaces.cs` after the `ITrackSelectionScreen` interface:

```csharp
    /// <summary>
    /// The track panel — the second hosted screen of the menu hub. Same carousel grammar
    /// as the garage's car panel: prev/next arrows cycle a focus track, the centre slot is
    /// the hero, and focusing a raceable track makes it the session track immediately.
    /// Focusing an unraceable circuit — locked, or no scene of its own — is display-only:
    /// its stats render, Confirm goes dark, the session track is unchanged.
    ///
    /// The screen owns widgets and events only; cycle order, availability and navigation
    /// live in the scene controller.
    /// </summary>
    public interface ITrackPanelScreen : ITrackSelectionScreen
    {
        event Action OnPrevTrackPressed;
        event Action OnNextTrackPressed;

        /// <summary>Confirm was pressed with a raceable session track expected.</summary>
        event Action OnConfirmPressed;

        /// <summary>
        /// Renders <paramref name="track"/> as the centre slot with its neighbours flanking.
        /// <paramref name="isSelected"/> is true when the flow accepted the track as the
        /// session track; <paramref name="isAvailable"/> is false for circuits with no
        /// scene, which render but cannot be raced.
        /// </summary>
        void ShowFocusedTrack(TrackDefinition track, bool isSelected, bool isAvailable);
    }
```

- [ ] **Step 2: Append the abstract class.** In `ScreenControllers.cs` after the `TrackSelectionScreen` class:

```csharp
    /// <summary>
    /// The track panel's controller-side base. Inherits the selection events from
    /// <see cref="TrackSelectionScreen"/> unchanged — <c>OnTrackSelected</c> stays the
    /// "a track became the session track" channel, and <c>OnBackPressed</c> stays
    /// "back to the car panel" — so the flow's screen assignment needs nothing new.
    ///
    /// TriggerBackPressed is declared here rather than on the base because
    /// <see cref="TrackSelectionScreen"/> declares <c>OnBackPressed</c> for its scene
    /// controller but exposes no protector; the panel's BACK button lives in the view.
    /// </summary>
    public abstract class TrackPanelScreen : TrackSelectionScreen, ITrackPanelScreen
    {
        public event Action OnPrevTrackPressed;
        public event Action OnNextTrackPressed;
        public event Action OnConfirmPressed;

        protected void TriggerBackPressed() => OnBackPressed?.Invoke();
        protected void TriggerPrevTrackPressed() => OnPrevTrackPressed?.Invoke();
        protected void TriggerNextTrackPressed() => OnNextTrackPressed?.Invoke();
        protected void TriggerConfirmPressed() => OnConfirmPressed?.Invoke();

        public abstract void ShowFocusedTrack(TrackDefinition track, bool isSelected, bool isAvailable);
    }
```

- [ ] **Step 3: Compile.** `refresh_unity`; `read_console` shows no compile errors.
- [ ] **Step 4: Commit.**

```bash
git add Assets/Scripts/GameFlow/ScreenInterfaces.cs Assets/Scripts/GameFlow/ScreenControllers.cs
git commit -m "Add the track panel screen seam alongside the track selection screen"
```

---

### Task 3: Shared chrome widgets + `GarageScreenImpl` rework

**Files:**
- Create: `Assets/Scripts/UI/ProfileWidget.cs`
- Create: `Assets/Scripts/UI/CurrencyWidget.cs`
- Modify: `Assets/Scripts/UI/GarageScreenImpl.cs` (replace 6 chrome fields with 2 widget refs; rewrite `SetCurrency`/`RefreshFromProfile`)
- Modify: `Assets/Editor/GarageBuilder.cs` (`BuildGarageScreenPrefab` — attach + wire the widgets; the prefab itself is rebuilt in Task 6)

**Interfaces:**
- Produces: `F1.UI.ProfileWidget.Refresh()`, `F1.UI.CurrencyWidget.SetCareerPoints(int)` + `Refresh()`. Task 4's `TrackPanelScreenImpl` and Task 6's builder use these.

- [ ] **Step 1: Create `ProfileWidget`.**

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.Progression;

namespace F1.UI
{
    /// <summary>
    /// The player identity readout — name, team, level, XP bar. Shared hub chrome: both
    /// panels render it identically, so it lives here rather than in either impl, and
    /// swapping a panel prefab never orphans the profile display.
    /// </summary>
    [DisallowMultipleComponent]
    public class ProfileWidget : MonoBehaviour
    {
        [SerializeField] private TMP_Text _playerNameText;
        [SerializeField] private TMP_Text _teamText;
        [SerializeField] private TMP_Text _levelText;
        [SerializeField] private Image _xpFill;

        public void Refresh()
        {
            var profile = PlayerProfileManager.Current;
            if (profile == null) return;

            if (_playerNameText != null) _playerNameText.text = profile.playerName;
            if (_teamText != null) _teamText.text = profile.teamName;
            if (_levelText != null) _levelText.text = $"LEVEL {profile.playerLevel}";
            if (_xpFill != null) _xpFill.fillAmount = profile.XpProgressToNextLevel();
        }
    }
}
```

- [ ] **Step 2: Create `CurrencyWidget`.**

```csharp
using UnityEngine;
using TMPro;
using F1.Progression;

namespace F1.UI
{
    /// <summary>
    /// The dual-currency readout. Career points lead because unlocks are denominated in
    /// careerPoints; premium sits under it.
    /// </summary>
    [DisallowMultipleComponent]
    public class CurrencyWidget : MonoBehaviour
    {
        [SerializeField] private TMP_Text _careerPointsText;
        [SerializeField] private TMP_Text _premiumText;

        public void SetCareerPoints(int amount)
        {
            if (_careerPointsText != null)
                _careerPointsText.text = amount.ToString("N0");
        }

        public void Refresh()
        {
            var profile = PlayerProfileManager.Current;
            if (profile == null) return;

            SetCareerPoints(profile.careerPoints);
            if (_premiumText != null) _premiumText.text = profile.premiumCurrency.ToString("N0");
        }
    }
}
```

- [ ] **Step 3: Rework `GarageScreenImpl`.** In `Assets/Scripts/UI/GarageScreenImpl.cs`:
  Replace the two field blocks:

```csharp
        [Header("Currency")]
        [SerializeField] private TMP_Text _careerPointsText;
        [SerializeField] private TMP_Text _premiumText;

        [Header("Player Profile")]
        [SerializeField] private TMP_Text _playerNameText;
        [SerializeField] private TMP_Text _teamText;
        [SerializeField] private TMP_Text _levelText;
        [SerializeField] private Image _xpFill;
```

  with:

```csharp
        [Header("Shared chrome")]
        [SerializeField] private CurrencyWidget _currency;
        [SerializeField] private ProfileWidget _profile;
```

  Replace `SetCurrency` and `RefreshFromProfile` with:

```csharp
        /// <summary>
        /// The career-points readout. Unlocks cost careerPoints, so this — not the premium
        /// figure — is what SetCurrency means here.
        /// </summary>
        public override void SetCurrency(int amount)
        {
            if (_currency != null) _currency.SetCareerPoints(amount);
        }

        public override void RefreshFromProfile()
        {
            if (_currency != null) _currency.Refresh();
            if (_profile != null) _profile.Refresh();
        }
```

  Also remove the now-unused `using F1.Progression;` **only if** nothing else in the file uses it — `PlayerProfileManager` and `HasActiveRental` in `BuildStatus` still do, so **keep the using**.

- [ ] **Step 4: Rework the builder's garage prefab wiring.** In `GarageBuilder.BuildGarageScreenPrefab`:
  After the profile panel block (ends with `var xpFill = NewBarFill(...);`), add:

```csharp
        var profileWidget = profile.gameObject.AddComponent<F1.UI.ProfileWidget>();
        var profileSo = new SerializedObject(profileWidget);
        Set(profileSo, "_playerNameText", playerName.GetComponent<TMP_Text>());
        Set(profileSo, "_teamText", team.GetComponent<TMP_Text>());
        Set(profileSo, "_levelText", level.GetComponent<TMP_Text>());
        Set(profileSo, "_xpFill", xpFill);
        profileSo.ApplyModifiedPropertiesWithoutUndo();
```

  After the currency panel block (ends with the `premiumValue` PlaceAt), add:

```csharp
        var currencyWidget = currency.gameObject.AddComponent<F1.UI.CurrencyWidget>();
        var currencySo = new SerializedObject(currencyWidget);
        Set(currencySo, "_careerPointsText", careerValue.GetComponent<TMP_Text>());
        Set(currencySo, "_premiumText", premiumValue.GetComponent<TMP_Text>());
        currencySo.ApplyModifiedPropertiesWithoutUndo();
```

  In the "Serialized wiring" section, replace these six lines:

```csharp
        Set(so, "_careerPointsText", careerValue.GetComponent<TMP_Text>());
        Set(so, "_premiumText", premiumValue.GetComponent<TMP_Text>());
        Set(so, "_playerNameText", playerName.GetComponent<TMP_Text>());
        Set(so, "_teamText", team.GetComponent<TMP_Text>());
        Set(so, "_levelText", level.GetComponent<TMP_Text>());
        Set(so, "_xpFill", xpFill);
```

  with:

```csharp
        Set(so, "_currency", currencyWidget);
        Set(so, "_profile", profileWidget);
```

- [ ] **Step 5: Compile + regression.** `refresh_unity`; no compile errors. Run EditMode suite (all green — nothing asserts the removed fields) and PlayMode `GarageSelectionTests` (4/4 — the old prefab's orphaned serialized fields drop silently; `_currency`/`_profile` are null until Task 6 rebuilds the prefab, and nothing in these tests reads chrome text).
- [ ] **Step 6: Commit.**

```bash
git add Assets/Scripts/UI/ProfileWidget.cs Assets/Scripts/UI/ProfileWidget.cs.meta Assets/Scripts/UI/CurrencyWidget.cs Assets/Scripts/UI/CurrencyWidget.cs.meta Assets/Scripts/UI/GarageScreenImpl.cs Assets/Editor/GarageBuilder.cs
git commit -m "Extract profile and currency chrome into shared widgets"
```

---

### Task 4: `TrackPanelScreenImpl` — the carousel view

**Files:**
- Create: `Assets/Scripts/UI/TrackPanelScreenImpl.cs`

**Interfaces:**
- Consumes: `TrackPanelScreen` (Task 2), `ProfileWidget`/`CurrencyWidget` (Task 3).
- Produces: `F1.UI.TrackPanelScreenImpl` — Task 5's controller binds its events; Task 6's builder instantiates the prefab carrying it.

- [ ] **Step 1: Create the file.**

```csharp
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;

namespace F1.UI
{
    /// <summary>
    /// The track panel view: a three-slot carousel (left dimmed / centre hero / right
    /// dimmed) with prev/next arrows, BACK and CONFIRM, over the shared hub chrome.
    ///
    /// Pure rendering. The focus index, the cycle order and every navigation decision
    /// live in the scene controller; this class draws whatever
    /// <see cref="ShowFocusedTrack"/> pushes and raises events for whatever is pressed.
    /// Every widget is null-guarded because scenes are also loaded raw by tests.
    /// </summary>
    [DisallowMultipleComponent]
    public class TrackPanelScreenImpl : TrackPanelScreen
    {
        /// <summary>One visible carousel slot.</summary>
        [Serializable]
        public struct TrackSlot
        {
            public GameObject Root;
            public Image Art;
            public TMP_Text ShortCode;
            public TMP_Text Name;
            public TMP_Text Length;
            public TMP_Text Laps;
            public TMP_Text Difficulty;
            public GameObject SelectedBadge;
            public CanvasGroup Dim;
        }

        [Header("Header")]
        [SerializeField] private TMP_Text _titleText;

        [Header("Slots — order: left, centre, right")]
        [SerializeField] private TrackSlot[] _slots;

        [Header("Actions")]
        [SerializeField] private Button _backButton;
        [SerializeField] private Button _prevButton;
        [SerializeField] private Button _nextButton;
        [SerializeField] private Button _confirmButton;

        [Header("Shared chrome")]
        [SerializeField] private ProfileWidget _profile;
        [SerializeField] private CurrencyWidget _currency;

        // A dimmed neighbour must still read as inactive rather than broken. The centre
        // slot is always full alpha; the sides blend toward the backdrop.
        private const float SideAlpha = 0.6f;

        // Mirrors SlimUiSkin.ArtWellFill — that class lives in the editor assembly and
        // cannot be referenced from runtime code.
        private static readonly Color EmptyArtFill = new Color(0.045f, 0.050f, 0.072f, 1f);

        private readonly List<TrackDefinition> _tracks = new List<TrackDefinition>();
        private string _selectedTrackId;

        private void Awake()
        {
            // Self-wired rather than UnityEvent-persisted — see GarageScreenImpl.Awake.
            if (_backButton != null) _backButton.onClick.AddListener(HandleBackClicked);
            if (_prevButton != null) _prevButton.onClick.AddListener(HandlePrevClicked);
            if (_nextButton != null) _nextButton.onClick.AddListener(HandleNextClicked);
            if (_confirmButton != null) _confirmButton.onClick.AddListener(HandleConfirmClicked);
        }

        private void OnDestroy()
        {
            if (_backButton != null) _backButton.onClick.RemoveListener(HandleBackClicked);
            if (_prevButton != null) _prevButton.onClick.RemoveListener(HandlePrevClicked);
            if (_nextButton != null) _nextButton.onClick.RemoveListener(HandleNextClicked);
            if (_confirmButton != null) _confirmButton.onClick.RemoveListener(HandleConfirmClicked);
        }

        private void HandleBackClicked() => TriggerBackPressed();
        private void HandlePrevClicked() => TriggerPrevTrackPressed();
        private void HandleNextClicked() => TriggerNextTrackPressed();
        private void HandleConfirmClicked() => TriggerConfirmPressed();

        public override void Initialize(GameFlowManager flowManager)
        {
            RefreshFromProfile();
        }

        public override void RefreshFromProfile()
        {
            if (_profile != null) _profile.Refresh();
            if (_currency != null) _currency.Refresh();
        }

        public override void SetCurrency(int amount)
        {
            if (_currency != null) _currency.SetCareerPoints(amount);
        }

        public override void PopulateTracks(IReadOnlyList<TrackDefinition> owned,
            IReadOnlyList<TrackDefinition> locked)
        {
            _tracks.Clear();
            _tracks.AddRange(owned);
            _tracks.AddRange(locked);
        }

        public override void SetSelectedTrack(TrackDefinition track)
        {
            _selectedTrackId = track != null ? track.TrackId : null;
        }

        public override void ShowFocusedTrack(TrackDefinition track, bool isSelected, bool isAvailable)
        {
            int index = track != null ? _tracks.FindIndex(t => t.TrackId == track.TrackId) : -1;
            if (index < 0 || _slots == null || _slots.Length < 3)
            {
                ShowEmpty();
                return;
            }

            ShowSlot(_slots[0], _tracks[Wrap(index - 1)], SideAlpha);
            ShowSlot(_slots[1], _tracks[index], 1f);
            ShowSlot(_slots[2], _tracks[Wrap(index + 1)], SideAlpha);

            // Confirm keys off the flow's verdict, not off availability: a locked focus
            // over a still-selected session track must not offer to race the locked one.
            if (_confirmButton != null) _confirmButton.interactable = isSelected;
        }

        private int Wrap(int i) => ((i % _tracks.Count) + _tracks.Count) % _tracks.Count;

        private void ShowEmpty()
        {
            if (_slots != null)
                foreach (var slot in _slots)
                    if (slot.Root != null) slot.Root.SetActive(false);
            if (_confirmButton != null) _confirmButton.interactable = false;
        }

        private void ShowSlot(TrackSlot slot, TrackDefinition track, float alpha)
        {
            if (slot.Root == null) return;
            slot.Root.SetActive(true);

            bool hasArt = track != null && (track.Thumbnail != null || track.MinimapTexture != null);
            if (slot.Art != null)
            {
                slot.Art.sprite = hasArt
                    ? (track.Thumbnail != null ? track.Thumbnail : track.MinimapTexture)
                    : null;
                // Without art the well renders as a dark panel rather than Unity's white
                // "missing sprite" quad.
                slot.Art.color = hasArt ? Color.white : EmptyArtFill;
            }
            if (slot.ShortCode != null) slot.ShortCode.text = track?.ShortCode ?? "";
            if (slot.Name != null) slot.Name.text = track?.DisplayName ?? "—";
            if (slot.Length != null)
                slot.Length.text = track != null ? $"{track.TrackLengthMeters / 1000f:0.000} KM" : "—";
            if (slot.Laps != null)
                slot.Laps.text = track != null ? $"{track.StandardRaceLaps}" : "—";
            if (slot.Difficulty != null)
                slot.Difficulty.text = string.IsNullOrEmpty(track?.DifficultyLabel) ? "—" : track.DifficultyLabel;
            if (slot.SelectedBadge != null)
                slot.SelectedBadge.SetActive(track != null && track.TrackId == _selectedTrackId);
            if (slot.Dim != null) slot.Dim.alpha = alpha;
        }
    }
}
```

- [ ] **Step 2: Compile.** `refresh_unity`; no errors. (Nothing instantiates it yet.)
- [ ] **Step 3: Commit.**

```bash
git add Assets/Scripts/UI/TrackPanelScreenImpl.cs Assets/Scripts/UI/TrackPanelScreenImpl.cs.meta
git commit -m "Add the track panel carousel view"
```

---

### Task 5: Runtime — `TrackPanelSceneController` + `GarageHubView`

**Files:**
- Create: `Assets/Scripts/GameFlow/TrackPanelSceneController.cs`
- Create: `Assets/Scripts/GameFlow/GarageHubView.cs`

**Interfaces:**
- Consumes: `TrackPanelScreen` (Task 2), `GameFlowManager.CurrentScreen/OnScreenChanged/SelectTrack/GoToLobby/GoToWingSetup/SelectedTrack`, `SceneAvailability.IsAvailable(TrackDefinition)`, `ProgressionRegistry.GetOwnedTracks/GetLockedTracks`, `ShowcaseCarSwapper`.
- Produces: scene components Task 7's builder places on the hub's two host roots.

- [ ] **Step 1: Create `TrackPanelSceneController`.**

```csharp
using System.Collections.Generic;
using UnityEngine;
using F1.GameData;
using F1.Progression;

namespace F1.GameFlow
{
    /// <summary>
    /// Owns the track panel's behaviour inside the menu hub: wires the panel's events to
    /// the flow and runs the focus cycle.
    ///
    /// The panel is the second hosted screen of 05_GarageScene. Its host root starts
    /// inactive and the hub view raises it only when the flow's current screen is
    /// TrackSelection — which is also what defers this Start until the panel is actually
    /// on stage. That deferral is load-bearing: binding and focusing must not run while
    /// the car panel owns the scene, or entering the garage would silently pick a track.
    ///
    /// Same rule as the garage's car panel: the displayed track IS the selection.
    /// Focusing a raceable circuit makes it the session track immediately, with the
    /// flow's verdict pushed back through the screen; focusing an unbuilt one is
    /// display-only and Confirm goes dark.
    /// </summary>
    [DisallowMultipleComponent]
    public class TrackPanelSceneController : MonoBehaviour
    {
        private GameFlowManager _flow;
        private TrackPanelScreen _screen;

        private readonly List<TrackDefinition> _cycle = new List<TrackDefinition>();
        private int _focusIndex;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError("[TrackPanelSceneController] No GameFlowManager found.", this);
                enabled = false;
                return;
            }

            _screen = GetComponentInChildren<TrackPanelScreen>(true);
            if (_screen == null)
            {
                Debug.LogError(
                    "[TrackPanelSceneController] No TrackPanelScreen under this host. The " +
                    "FlowSceneHost is probably missing its screen prefab.", this);
                enabled = false;
                return;
            }

            _screen.OnPrevTrackPressed += OnPrevTrackPressed;
            _screen.OnNextTrackPressed += OnNextTrackPressed;
            _screen.OnBackPressed += OnBackPressed;
            _screen.OnConfirmPressed += OnConfirmPressed;

            RebuildCycle();
            if (_cycle.Count == 0)
            {
                Debug.LogError("[TrackPanelSceneController] No tracks are registered.", this);
                return;
            }

            _screen.PopulateTracks(_cycle, System.Array.Empty<TrackDefinition>());
            FocusInitialTrack();
        }

        private void OnDestroy()
        {
            if (_screen == null) return;

            _screen.OnPrevTrackPressed -= OnPrevTrackPressed;
            _screen.OnNextTrackPressed -= OnNextTrackPressed;
            _screen.OnBackPressed -= OnBackPressed;
            _screen.OnConfirmPressed -= OnConfirmPressed;
        }

        private void OnPrevTrackPressed() => MoveFocus(-1);

        private void OnNextTrackPressed() => MoveFocus(+1);

        private void MoveFocus(int direction)
        {
            if (_cycle.Count == 0) return;
            _focusIndex = ((_focusIndex + direction) % _cycle.Count + _cycle.Count) % _cycle.Count;
            Focus(_cycle[_focusIndex]);
        }

        /// <summary>
        /// Entry focus starts on the session track when there is one, so returning from
        /// wing setup shows what was raced; otherwise the first track in cycle order.
        /// </summary>
        private void FocusInitialTrack()
        {
            _focusIndex = 0;
            var chosen = _flow.SelectedTrack;
            if (chosen != null)
            {
                for (int i = 0; i < _cycle.Count; i++)
                {
                    if (_cycle[i].TrackId == chosen.TrackId)
                    {
                        _focusIndex = i;
                        break;
                    }
                }
            }

            Focus(_cycle[_focusIndex]);
        }

        /// <summary>
        /// Owned tracks first, then locked, each in registry order — the same shape as
        /// the car cycle, so cycling forward moves from what the player can race towards
        /// what they cannot yet.
        /// </summary>
        private void RebuildCycle()
        {
            _cycle.Clear();
            foreach (var track in ProgressionRegistry.GetOwnedTracks())
                _cycle.Add(track);
            foreach (var track in ProgressionRegistry.GetLockedTracks())
                _cycle.Add(track);
        }

        private void Focus(TrackDefinition track)
        {
            if (track == null) return;

            // Raceable, not merely owned: Spa is owned but shares its corner with Monaco
            // and has no scene of its own, so it must stay display-only. Locked circuits
            // are turned away by SelectTrack's own ownership guard (GameFlowManager.cs:280),
            // so they land display-only through the same verdict push-back.
            bool isAvailable = SceneAvailability.IsAvailable(track);
            if (isAvailable)
            {
                _flow.SelectTrack(track);
            }
            else
            {
                Debug.Log(
                    $"[TrackPanelSceneController] '{track.TrackId}' is not raceable yet; the " +
                    "session track is unchanged.");
            }

            _screen.SetSelectedTrack(_flow.SelectedTrack);
            _screen.ShowFocusedTrack(track, _flow.SelectedTrack == track, isAvailable);
        }

        private void OnConfirmPressed()
        {
            if (_flow.SelectedTrack == null)
            {
                Debug.Log("[TrackPanelSceneController] Confirm pressed with no session track.");
                return;
            }

            if (!SceneAvailability.IsAvailable(_flow.SelectedTrack))
            {
                Debug.Log(
                    $"[TrackPanelSceneController] '{_flow.SelectedTrack.TrackId}' has no scene; " +
                    "staying on the panel.");
                return;
            }

            // Invariant 3: track selection always precedes wing selection.
            _flow.GoToWingSetup();
        }

        private void OnBackPressed() => _flow.GoToLobby();
    }
}
```

- [ ] **Step 2: Create `GarageHubView`.**

```csharp
using UnityEngine;

namespace F1.GameFlow
{
    /// <summary>
    /// The menu hub's stage manager. Two hosted screens share one background scene — the
    /// car panel (GameScreen.Lobby) and the track panel (GameScreen.TrackSelection) — and
    /// this decides which is on stage from the flow's current screen, the same signal the
    /// flow itself just committed to. It also takes the showcase car off stage during
    /// track selection, which stops the turntable for free.
    ///
    /// Toggling the host ROOTS (not the screens) is deliberate: each screen's scene
    /// controller sits on its host root, so deactivating the root pauses its behaviour
    /// with it, and GameFlowManager.FindSceneHost matches registered hosts regardless of
    /// active state, so transitions in and out of the hub keep resolving both screens.
    ///
    /// With no flow (raw scene load by tests or art review) this does nothing and the
    /// scene's authored default — car panel on stage — holds.
    /// </summary>
    [DisallowMultipleComponent]
    public class GarageHubView : MonoBehaviour
    {
        private GameFlowManager _flow;
        private FlowSceneHost _carHost;
        private FlowSceneHost _trackHost;
        private GameObject _carSocket;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null) return;

            foreach (var host in FindObjectsByType<FlowSceneHost>(
                         FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (host.HostedScreen == GameFlowManager.GameScreen.Lobby) _carHost = host;
                else if (host.HostedScreen == GameFlowManager.GameScreen.TrackSelection) _trackHost = host;
            }

            var swapper = FindAnyObjectByType<ShowcaseCarSwapper>();
            if (swapper != null) _carSocket = swapper.gameObject;

            _flow.OnScreenChanged += Apply;
            Apply(_flow.CurrentScreen);
        }

        private void OnDestroy()
        {
            if (_flow != null) _flow.OnScreenChanged -= Apply;
        }

        private void Apply(GameFlowManager.GameScreen screen)
        {
            SetActive(_carHost, screen == GameFlowManager.GameScreen.Lobby);
            SetActive(_trackHost, screen == GameFlowManager.GameScreen.TrackSelection);
            if (_carSocket != null)
                _carSocket.SetActive(screen == GameFlowManager.GameScreen.Lobby);
        }

        private static void SetActive(FlowSceneHost host, bool active)
        {
            if (host != null) host.gameObject.SetActive(active);
        }
    }
}
```

- [ ] **Step 3: Compile + regression.** `refresh_unity`; no errors. Run EditMode suite and PlayMode `GarageSelectionTests` — nothing instantiates or runs the new components yet, so both stay at baseline.
- [ ] **Step 4: Commit.**

```bash
git add Assets/Scripts/GameFlow/TrackPanelSceneController.cs Assets/Scripts/GameFlow/TrackPanelSceneController.cs.meta Assets/Scripts/GameFlow/GarageHubView.cs Assets/Scripts/GameFlow/GarageHubView.cs.meta
git commit -m "Add the track panel controller and the hub stage manager"
```

---

### Task 6: Builder — track panel prefab + garage prefab widget rework

**Files:**
- Modify: `Assets/Editor/GarageBuilder.cs` (add `TrackPanelPrefabPath` const, `BuildTrackPanelPrefab`, a `BuildTrackSlot` helper, and extend `BuildAll`)

**Interfaces:**
- Consumes: `F1.UI.TrackPanelScreenImpl` (Task 4) and its serialized field names; GarageBuilder's existing `NewPanel/NewText/NewButton/NewBar/NewBarFill/PlaceAt/Set/EnsureFolder` helpers.
- Produces: `Assets/Prefabs/TrackPanelScreen_Prefab.prefab`; rebuilt `GarageScreen_Prefab.prefab` with widget refs.

- [ ] **Step 1: Add the const** next to the other paths (line ~34):

```csharp
    private const string TrackPanelPrefabPath = "Assets/Prefabs/TrackPanelScreen_Prefab.prefab";
```

- [ ] **Step 2: Add `BuildTrackPanelPrefab`** after `BuildGarageScreenPrefab` (line ~279). The header comment above `BuildGarageScreenPrefab` about self-wired buttons applies here too.

```csharp
    /// <summary>
    /// Builds the track panel: SELECT TRACK title, three carousel slots (sides dimmed),
    /// edge arrows, BACK and CONFIRM, over the shared profile/currency chrome. Same
    /// self-wiring rule as the garage screen — serialized references only.
    ///
    /// Re-runnable: edits the prefab in place.
    /// </summary>
    [MenuItem("Tools/Garage/Build Track Panel Prefab", priority = 15)]
    public static void BuildTrackPanelPrefab()
    {
        SlimUiSkin.EnsureLoaded();
        EnsureFolder("Assets/Prefabs");

        var root = new GameObject("TrackPanelScreen_Prefab",
            typeof(RectTransform), typeof(Canvas),
            typeof(UnityEngine.UI.CanvasScaler),
            typeof(UnityEngine.UI.GraphicRaycaster),
            typeof(F1.UI.TrackPanelScreenImpl));

        var rt = (RectTransform)root.transform;
        root.GetComponent<Canvas>().renderMode = RenderMode.ScreenSpaceOverlay;
        var scaler = root.GetComponent<UnityEngine.UI.CanvasScaler>();
        scaler.uiScaleMode = UnityEngine.UI.CanvasScaler.ScaleMode.ScaleWithScreenSize;
        scaler.referenceResolution = new Vector2(1920f, 1080f);
        scaler.screenMatchMode = UnityEngine.UI.CanvasScaler.ScreenMatchMode.MatchWidthOrHeight;
        scaler.matchWidthOrHeight = 1f;

        // No root Graphic: the garage behind is the point, exactly as on the car panel.

        var title = NewText("Title", rt, "SELECT TRACK", 34f, TextAlignmentOptions.Center, SlimUiSkin.TileInk);
        PlaceAt(title, new Vector2(0.5f, 1f), new Vector2(600f, 60f), new Vector2(0f, 36f));

        // Carousel slots. PlaceAt zeroes the offset on a middle anchor, so the sides get
        // their anchoredPosition set directly after.
        var centre = NewPanel("SlotCentre", rt, new Vector2(0.5f, 0.5f), new Vector2(430f, 560f), Vector2.zero);
        centre.anchoredPosition = new Vector2(0f, 20f);
        var left = NewPanel("SlotLeft", rt, new Vector2(0.5f, 0.5f), new Vector2(320f, 430f), Vector2.zero);
        left.anchoredPosition = new Vector2(-560f, 60f);
        var right = NewPanel("SlotRight", rt, new Vector2(0.5f, 0.5f), new Vector2(320f, 430f), Vector2.zero);
        right.anchoredPosition = new Vector2(560f, 60f);

        var slots = new F1.UI.TrackPanelScreenImpl.TrackSlot[3];
        slots[1] = BuildTrackSlot(centre);
        slots[0] = BuildTrackSlot(left);
        slots[2] = BuildTrackSlot(right);

        // ASCII "<"/">": the default TMP SDF atlas only guarantees ASCII.
        var prev = NewButton("PrevButton", rt, "<", 40f, primary: false);
        PlaceAt(prev, new Vector2(0f, 0.5f), new Vector2(90f, 130f), new Vector2(60f, 0f));
        var next = NewButton("NextButton", rt, ">", 40f, primary: false);
        PlaceAt(next, new Vector2(1f, 0.5f), new Vector2(90f, 130f), new Vector2(60f, 0f));

        var back = NewButton("BackButton", rt, "BACK", 26f, primary: false);
        PlaceAt(back, new Vector2(0f, 0f), new Vector2(240f, 80f), new Vector2(24f, 28f));
        var confirm = NewButton("ConfirmButton", rt, "CONFIRM", 26f, primary: true);
        PlaceAt(confirm, new Vector2(1f, 0f), new Vector2(320f, 80f), new Vector2(24f, 28f));
        confirm.GetComponent<UnityEngine.UI.Button>().interactable = false;

        // Shared chrome — identical placement to the car panel so the corner identity
        // holds across the swap.
        var profile = NewPanel("ProfilePanel", rt, new Vector2(0f, 1f), new Vector2(400f, 140f), new Vector2(24f, 24f));
        var playerName = NewText("PlayerName", profile, "Player", 24f, TextAlignmentOptions.Left, SlimUiSkin.TileInk);
        PlaceAt(playerName, new Vector2(0f, 1f), new Vector2(250f, 34f), new Vector2(20f, 16f));
        var team = NewText("Team", profile, "Team", 17f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted);
        PlaceAt(team, new Vector2(0f, 1f), new Vector2(250f, 26f), new Vector2(20f, 56f));
        var level = NewText("Level", profile, "LEVEL 1", 17f, TextAlignmentOptions.Right, SlimUiSkin.TileInk);
        PlaceAt(level, new Vector2(1f, 1f), new Vector2(130f, 26f), new Vector2(20f, 18f));
        var xpBg = NewBar("XpBar", profile, new Vector2(0f, 1f), new Vector2(360f, 12f), new Vector2(20f, 100f));
        var xpFill = NewBarFill(xpBg, SlimUiSkin.Accent);
        var profileWidget = profile.gameObject.AddComponent<F1.UI.ProfileWidget>();
        var profileSo = new SerializedObject(profileWidget);
        Set(profileSo, "_playerNameText", playerName.GetComponent<TMP_Text>());
        Set(profileSo, "_teamText", team.GetComponent<TMP_Text>());
        Set(profileSo, "_levelText", level.GetComponent<TMP_Text>());
        Set(profileSo, "_xpFill", xpFill);
        profileSo.ApplyModifiedPropertiesWithoutUndo();

        var currency = NewPanel("CurrencyPanel", rt, new Vector2(1f, 1f), new Vector2(330f, 140f), new Vector2(24f, 24f));
        NewText("CareerLabel", currency, "CAREER PTS", 14f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted)
            .AndPlace(new Vector2(0f, 1f), new Vector2(290f, 22f), new Vector2(20f, 16f));
        var careerValue = NewText("CareerValue", currency, "0", 30f, TextAlignmentOptions.Left, SlimUiSkin.TileInk);
        PlaceAt(careerValue, new Vector2(0f, 1f), new Vector2(290f, 42f), new Vector2(20f, 38f));
        NewText("PremiumLabel", currency, "PREMIUM", 14f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted)
            .AndPlace(new Vector2(0f, 1f), new Vector2(290f, 22f), new Vector2(20f, 86f));
        var premiumValue = NewText("PremiumValue", currency, "0", 22f, TextAlignmentOptions.Left, SlimUiSkin.TileInk);
        PlaceAt(premiumValue, new Vector2(0f, 1f), new Vector2(290f, 32f), new Vector2(20f, 104f));
        var currencyWidget = currency.gameObject.AddComponent<F1.UI.CurrencyWidget>();
        var currencySo = new SerializedObject(currencyWidget);
        Set(currencySo, "_careerPointsText", careerValue.GetComponent<TMP_Text>());
        Set(currencySo, "_premiumText", premiumValue.GetComponent<TMP_Text>());
        currencySo.ApplyModifiedPropertiesWithoutUndo();

        var impl = root.GetComponent<F1.UI.TrackPanelScreenImpl>();
        var so = new SerializedObject(impl);
        Set(so, "_titleText", title.GetComponent<TMP_Text>());
        var slotsProp = so.FindProperty("_slots");
        slotsProp.arraySize = 3;
        for (int i = 0; i < 3; i++)
        {
            var slot = slotsProp.GetArrayElementAtIndex(i);
            var s = slots[i];
            slot.FindPropertyRelative("Root").objectReferenceValue = s.Root;
            slot.FindPropertyRelative("Art").objectReferenceValue = s.Art;
            slot.FindPropertyRelative("ShortCode").objectReferenceValue = s.ShortCode;
            slot.FindPropertyRelative("Name").objectReferenceValue = s.Name;
            slot.FindPropertyRelative("Length").objectReferenceValue = s.Length;
            slot.FindPropertyRelative("Laps").objectReferenceValue = s.Laps;
            slot.FindPropertyRelative("Difficulty").objectReferenceValue = s.Difficulty;
            slot.FindPropertyRelative("SelectedBadge").objectReferenceValue = s.SelectedBadge;
            slot.FindPropertyRelative("Dim").objectReferenceValue = s.Dim;
        }
        Set(so, "_backButton", back.GetComponent<UnityEngine.UI.Button>());
        Set(so, "_prevButton", prev.GetComponent<UnityEngine.UI.Button>());
        Set(so, "_nextButton", next.GetComponent<UnityEngine.UI.Button>());
        Set(so, "_confirmButton", confirm.GetComponent<UnityEngine.UI.Button>());
        Set(so, "_profile", profileWidget);
        Set(so, "_currency", currencyWidget);
        so.ApplyModifiedPropertiesWithoutUndo();

        PrefabUtility.SaveAsPrefabAsset(root, TrackPanelPrefabPath);
        Object.DestroyImmediate(root);
        Debug.Log($"[Garage] Track panel prefab written to {TrackPanelPrefabPath}");
    }

    /// <summary>
    /// Fills one carousel slot: art well up top, short-code chip, name, then the
    /// LENGTH / LAPS / DIFFICULTY stat band. The CanvasGroup is the side-slot dimmer.
    /// </summary>
    private static F1.UI.TrackPanelScreenImpl.TrackSlot BuildTrackSlot(RectTransform parent)
    {
        var slot = new F1.UI.TrackPanelScreenImpl.TrackSlot { Root = parent.gameObject };

        var dim = parent.gameObject.AddComponent<CanvasGroup>();
        dim.blocksRaycasts = false;
        dim.interactable = false;
        slot.Dim = dim;

        var badge = NewText("SelectedBadge", parent, "SELECTED", 15f, TextAlignmentOptions.Right, SlimUiSkin.Accent);
        PlaceAt(badge, new Vector2(1f, 1f), new Vector2(140f, 26f), new Vector2(16f, 12f));
        slot.SelectedBadge = badge.gameObject;

        var art = NewBar("ArtWell", parent, new Vector2(0.5f, 1f), new Vector2(parent.sizeDelta.x - 32f, parent.sizeDelta.y * 0.42f), new Vector2(0f, 16f));
        slot.Art = art.GetComponent<UnityEngine.UI.Image>();

        var code = NewText("ShortCode", parent, "", 14f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted);
        PlaceAt(code, new Vector2(0f, 1f), new Vector2(120f, 22f), new Vector2(16f, 16f));
        slot.ShortCode = code.GetComponent<TMP_Text>();

        var name = NewText("Name", parent, "Track", 26f, TextAlignmentOptions.Center, SlimUiSkin.TileInk);
        PlaceAt(name, new Vector2(0.5f, 1f), new Vector2(parent.sizeDelta.x - 32f, 40f), new Vector2(0f, parent.sizeDelta.y * 0.42f + 28f));
        slot.Name = name.GetComponent<TMP_Text>();

        string[] labels = { "LENGTH", "LAPS", "DIFFICULTY" };
        var texts = new TMP_Text[3];
        for (int i = 0; i < 3; i++)
        {
            float y = parent.sizeDelta.y * 0.42f + 92f + i * 46f;
            NewText($"Row{i}Label", parent, labels[i], 13f, TextAlignmentOptions.Left, SlimUiSkin.TileInkMuted)
                .AndPlace(new Vector2(0f, 1f), new Vector2(160f, 20f), new Vector2(24f, y));
            texts[i] = NewText($"Row{i}Value", parent, "—", 18f, TextAlignmentOptions.Right, SlimUiSkin.TileInk)
                .AndPlace(new Vector2(1f, 1f), new Vector2(160f, 24f), new Vector2(24f, y))
                .GetComponent<TMP_Text>();
        }
        slot.Length = texts[0];
        slot.Laps = texts[1];
        slot.Difficulty = texts[2];

        return slot;
    }
```

  Note: `NewText`/`NewBar`/`NewButton` return `RectTransform`; `slot.Art`/`slot.SelectedBadge` etc. need the exact component types shown. `badge.gameObject` for the badge, `art.GetComponent<UnityEngine.UI.Image>()` for art.

- [ ] **Step 3: Extend `BuildAll`** (line ~40) to include the new prefab between the garage prefab and the scene:

```csharp
        AuthorCarDisplayStats();
        AuthorTrackMenuDisplay();
        BuildGarageScreenPrefab();
        BuildTrackPanelPrefab();
        BuildGarageScene();
        UpdateBuildSettingsAndFlowRoot();
```

- [ ] **Step 4: Compile + build the two prefabs.** `refresh_unity`; `execute_menu_item` `Tools/Garage/Build Garage Screen Prefab`; `execute_menu_item` `Tools/Garage/Build Track Panel Prefab`; `read_console` — expect both "written to" logs, no errors.
- [ ] **Step 5: Commit.**

```bash
git add Assets/Editor/GarageBuilder.cs Assets/Prefabs/TrackPanelScreen_Prefab.prefab Assets/Prefabs/TrackPanelScreen_Prefab.prefab.meta Assets/Prefabs/GarageScreen_Prefab.prefab
git commit -m "Build the track panel prefab and rewire the garage prefab onto the shared widgets"
```

---

### Task 7: FlowSceneNames + builder — hub scene + routing rewrite

**Files:**
- Modify: `Assets/Scripts/GameFlow/FlowSceneNames.cs`
- Modify: `Assets/Editor/GarageBuilder.cs` (`BuildGarageScene` full rewrite, `UpdateBuildSettingsAndFlowRoot` full rewrite, header comment)
- Modify: `Assets/Editor/LobbyBuilder.cs` (header warning only)

**Interfaces:**
- Produces: `FlowSceneNames.TrackSelection == "05_GarageScene"` and `FlowSceneNames.RetiredTrackSelection == "20_TrackSelectionScene"` (the builder's routing below compiles against `RetiredTrackSelection`, so the constant lands in this same commit — Task 8 is what runs the builder); rebuilt `05_GarageScene.unity` with two hosts; routing that removes the retired build entries and rewrites `_lobbyScene`/`_trackSelectionScene`.
- Consumes: `F1.UI` prefabs from Task 6; `GarageHubView`/`TrackPanelSceneController` (Task 5); `FlowSceneHost.Configure(GameScreen, GameObject, HostScreenMode, Transform)`; GarageBuilder's existing consts `GaragePrefabPath`, `GarageScenePath`, `ShowcasePrefabPath` and helpers `EnsureRoot(Scene, string)`/`EnsureEventSystem(Scene)`.

- [ ] **Step 1: Update `FlowSceneNames`.** In `Assets/Scripts/GameFlow/FlowSceneNames.cs`:

  Replace the `RetiredGarageLobby` doc comment with:

```csharp
        /// <summary>
        /// The garage scene's predecessor. Deleted from disk by the menu-hub rework (the
        /// hub scene's framing is authored in GarageBuilder now), kept as a constant for
        /// the build-list exclusion tests, on the pattern of <see cref="CarSelection"/>.
        /// </summary>
        public const string RetiredGarageLobby = "05_LobbyScene";
```

  Replace the `TrackSelection` line with:

```csharp
        /// <summary>
        /// Track selection. Since the menu-hub rework this NAMES THE HUB SCENE: the track
        /// panel is 05_GarageScene's second hosted screen, so GoToTrackSelection resolves
        /// through SceneFlowService's same-scene guard into an in-place panel swap, and
        /// wing setup's BACK lands on the hub with the track panel showing.
        /// </summary>
        public const string TrackSelection = "05_GarageScene";

        /// <summary>
        /// Track selection's predecessor: its own scene. Deleted from disk by the
        /// menu-hub rework; kept as a constant for the build-list exclusion tests.
        /// </summary>
        public const string RetiredTrackSelection = "20_TrackSelectionScene";
```

  (Until Task 8 runs the routing step, 00_LoadingScene's serialized `_trackSelectionScene` still names the retired scene — a transient state; nothing runs the game in between.)

- [ ] **Step 2: Rewrite the builder's file header** (lines 1–17). Replace the paragraph about capturing the rig with:

```
// The garage scene is the MENU HUB: one background, two hosted screens (car panel on
// GameScreen.Lobby, track panel on GameScreen.TrackSelection). The 3D framing is
// authored in code below — Assets/Scenes/05_LobbyScene.unity (the old rig source) and
// 20_TrackSelectionScene.unity are deleted, so there is nothing left to capture from.
// The room art is not built yet; the car stands alone against a solid backdrop until
// the new lobby environment asset lands (see Assets/Scenes/ReferenceImageLobby.png).
```

  Keep the existing WARNING paragraph about `Tools > BuildFlowScenes`, and add:

```
// WARNING: Tools > Lobby > Build Wing Setup Lobby still captures its rig from the
// deleted 05_LobbyScene and will error if run. 30_WingSetupScene is already built; do
// not rebuild it here.
```

- [ ] **Step 3: Rewrite `BuildGarageScene`** (lines ~294–334) wholesale:

```csharp
    // ==================================================================================
    // Menu hub scene
    // ==================================================================================

    // Framing constants, matched to Assets/Scenes/ReferenceImageLobby.png: head-on, low,
    // symmetric, car centred. The verification pass in the plan tunes these against a
    // real screenshot; change them here, never in the scene file.
    private static readonly Vector3 SocketPosition = new Vector3(0f, 0.05f, 0f);
    private const float SocketYawDegrees = 180f;   // nose toward the camera at -Z
    private const float SocketScale = 0.8f;
    private static readonly Vector3 CameraPosition = new Vector3(0f, 0.85f, -6.2f);
    private static readonly Vector3 CameraLookAt = new Vector3(0f, 0.72f, 0.35f);
    private const float CameraFov = 34f;
    private static readonly Color BackdropColor = new Color(0.055f, 0.065f, 0.08f, 1f);

    /// <summary>
    /// Builds 05_GarageScene as the menu hub: car socket + turntable + swapper, a camera
    /// and two lights framing the car head-on, a FlowSceneHost for the car panel
    /// (GameScreen.Lobby) and a second, start-inactive host for the track panel
    /// (GameScreen.TrackSelection), a GarageHubView to swap them, and one EventSystem.
    ///
    /// Always builds a fresh empty scene and saves over the path, so the old Simple
    /// Garage room cannot survive a rebuild.
    /// </summary>
    [MenuItem("Tools/Garage/Build Garage Scene", priority = 12)]
    public static void BuildGarageScene()
    {
        var garagePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(GaragePrefabPath);
        var trackPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(TrackPanelPrefabPath);
        if (garagePrefab == null)
            Debug.LogError($"[Garage] {GaragePrefabPath} missing; build the screen prefab first.");
        if (trackPrefab == null)
            Debug.LogError($"[Garage] {TrackPanelPrefabPath} missing; build the track panel prefab first.");

        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        // --- Car socket -------------------------------------------------------------
        var showcasePrefab = AssetDatabase.LoadAssetAtPath<GameObject>(ShowcasePrefabPath);
        if (showcasePrefab == null)
            Debug.LogError($"[Garage] {ShowcasePrefabPath} missing; build it via Tools > Lobby.");

        // NewScene(Single) makes the new scene active, so everything created below lands
        // in it directly — no MoveGameObjectToScene round trips.
        var socket = new GameObject("CarSocket");
        socket.transform.localPosition = SocketPosition;
        socket.transform.localRotation = Quaternion.Euler(0f, SocketYawDegrees, 0f);
        socket.transform.localScale = new Vector3(SocketScale, SocketScale, SocketScale);
        socket.AddComponent<GarageTurntable>();
        var swapper = socket.AddComponent<ShowcaseCarSwapper>();
        WireSwapperEntries(swapper, showcasePrefab);

        // --- Camera, head-on and low -------------------------------------------------
        var camGo = new GameObject("LobbyCamera", typeof(Camera));
        camGo.tag = "MainCamera";
        var cam = camGo.GetComponent<Camera>();
        cam.fieldOfView = CameraFov;
        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 60f;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = BackdropColor;
        if (camGo.GetComponent<UnityEngine.Rendering.Universal.UniversalAdditionalCameraData>() == null)
            camGo.AddComponent<UnityEngine.Rendering.Universal.UniversalAdditionalCameraData>();
        camGo.transform.position = CameraPosition;
        camGo.transform.rotation = Quaternion.LookRotation(
            (CameraLookAt - CameraPosition).normalized, Vector3.up);

        // --- Key + fill ---------------------------------------------------------------
        // The reference's ceiling strips are art; until then a front-top key and a cool
        // rear fill give the car readable form against the backdrop.
        var keyGo = new GameObject("LobbyKeyLight", typeof(Light));
        var key = keyGo.GetComponent<Light>();
        key.type = LightType.Spot;
        key.color = new Color(1f, 0.98f, 0.94f);
        key.intensity = 1.4f;
        key.range = 35f;
        key.spotAngle = 62f;
        key.innerSpotAngle = 40f;
        key.shadows = LightShadows.Soft;
        keyGo.transform.position = new Vector3(0f, 5.5f, -4.5f);
        keyGo.transform.rotation = Quaternion.LookRotation(
            (SocketPosition + Vector3.up * 0.7f - new Vector3(0f, 5.5f, -4.5f)).normalized, Vector3.up);

        var fillGo = new GameObject("LobbyFillLight", typeof(Light));
        var fill = fillGo.GetComponent<Light>();
        fill.type = LightType.Directional;
        fill.color = new Color(0.75f, 0.82f, 0.95f);
        fill.intensity = 0.35f;
        fillGo.transform.rotation = Quaternion.Euler(38f, 145f, 0f);

        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
        RenderSettings.ambientLight = new Color(0.32f, 0.33f, 0.36f);

        // --- Hosts --------------------------------------------------------------------
        var carHostGo = EnsureRoot(scene, "FlowSceneHost");
        var carHost = EnsureComponent<FlowSceneHost>(carHostGo);
        carHost.Configure(GameFlowManager.GameScreen.Lobby, garagePrefab,
            HostScreenMode.Fixed, carHostGo.transform);
        EnsureComponent<GarageSceneController>(carHostGo);

        var trackHostGo = EnsureRoot(scene, "TrackPanelHost");
        var trackHost = EnsureComponent<FlowSceneHost>(trackHostGo);
        trackHost.Configure(GameFlowManager.GameScreen.TrackSelection, trackPrefab,
            HostScreenMode.Fixed, trackHostGo.transform);
        EnsureComponent<TrackPanelSceneController>(trackHostGo);
        // The car panel owns the stage on entry; GarageHubView swaps from here.
        trackHostGo.SetActive(false);

        var hubGo = EnsureRoot(scene, "MenuHub");
        EnsureComponent<GarageHubView>(hubGo);

        EnsureEventSystem(scene);

        EditorSceneManager.SaveScene(scene, GarageScenePath);
        Debug.Log($"[Garage] Menu hub scene saved to {GarageScenePath}");
    }

    private static void WireSwapperEntries(ShowcaseCarSwapper swapper, GameObject showcasePrefab)
    {
        var cars = LoadAllCars();
        var so = new SerializedObject(swapper);
        var entries = so.FindProperty("_entries");
        entries.arraySize = cars.Count;
        for (int i = 0; i < cars.Count; i++)
        {
            var entry = entries.GetArrayElementAtIndex(i);
            entry.FindPropertyRelative("Car").objectReferenceValue = cars[i];
            entry.FindPropertyRelative("ShowcasePrefab").objectReferenceValue = showcasePrefab;
        }
        so.ApplyModifiedPropertiesWithoutUndo();
    }
```

  Also **delete** the now-unused `SwapShowcaseForSocket` method (lines ~336–393) and the `RetiredLobbyScenePath` const (line 37) — Task 8 removes the file it pointed at, and the new build never references it.

- [ ] **Step 4: Rewrite `UpdateBuildSettingsAndFlowRoot`** (lines ~399–447) wholesale:

```csharp
    /// <summary>
    /// Routing: drop the retired scenes from the build list, keep the route order
    /// Loading -> Garage(hub) -> WingSetup -> PreRace -> Race -> Track_01 -> Results, and
    /// rewrite 00_LoadingScene's serialized scene fields, which would otherwise keep
    /// pointing at retired scenes and quietly load them.
    /// </summary>
    [MenuItem("Tools/Garage/Update Build Settings && Flow Root", priority = 13)]
    public static void UpdateBuildSettingsAndFlowRoot()
    {
        var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
        int removed = scenes.RemoveAll(s =>
            Path.GetFileNameWithoutExtension(s.path) == FlowSceneNames.RetiredGarageLobby ||
            Path.GetFileNameWithoutExtension(s.path) == FlowSceneNames.RetiredTrackSelection);
        if (removed > 0)
            Debug.Log($"[Garage] Removed {removed} retired scene entr(ies) from the build list.");

        bool hasHub = scenes.Exists(s =>
            Path.GetFileNameWithoutExtension(s.path) == FlowSceneNames.Lobby);
        if (!hasHub)
        {
            scenes.Insert(1, new EditorBuildSettingsScene(GarageScenePath, true));
            Debug.Log("[Garage] Inserted the hub scene after loading.");
        }
        EditorBuildSettings.scenes = scenes.ToArray();

        var loadingPath = "Assets/Scenes/" + FlowSceneNames.Loading + ".unity";
        if (!File.Exists(ToAbsolute(loadingPath)))
        {
            Debug.LogError($"[Garage] {loadingPath} does not exist; flow root not updated.");
            return;
        }

        var scene = EditorSceneManager.OpenScene(loadingPath, OpenSceneMode.Single);
        var flow = Object.FindAnyObjectByType<GameFlowManager>(FindObjectsInactive.Include);
        if (flow == null)
        {
            Debug.LogError("[Garage] 00_LoadingScene has no GameFlowManager; flow root not updated.");
            return;
        }

        var so = new SerializedObject(flow);
        var lobbyProp = so.FindProperty("_lobbyScene");
        var trackProp = so.FindProperty("_trackSelectionScene");
        if (lobbyProp == null || trackProp == null)
        {
            Debug.LogError("[Garage] GameFlowManager scene fields not found; flow root not updated.");
            return;
        }
        lobbyProp.stringValue = FlowSceneNames.Lobby;
        trackProp.stringValue = FlowSceneNames.TrackSelection;
        so.ApplyModifiedPropertiesWithoutUndo();
        EditorSceneManager.SaveScene(scene);
        Debug.Log($"[Garage] Flow root scenes -> lobby '{FlowSceneNames.Lobby}', " +
                  $"track selection '{FlowSceneNames.TrackSelection}'.");
    }
```

- [ ] **Step 5: LobbyBuilder header warning.** In `Assets/Editor/LobbyBuilder.cs`, immediately after the existing file-header comment block (before `public static class LobbyBuilder`), add:

```csharp
// 2026-10-13: Assets/Scenes/05_LobbyScene.unity is deleted (menu-hub rework). The rig
// capture below and Tools > Lobby > Build Wing Setup Lobby read from it and will error
// if run. 30_WingSetupScene is already built; do not rebuild it here.
```

- [ ] **Step 6: Compile.** `refresh_unity`; no errors. (The builder is not run yet — Task 8 runs it after the deletions.)
- [ ] **Step 7: Commit.**

```bash
git add Assets/Scripts/GameFlow/FlowSceneNames.cs Assets/Editor/GarageBuilder.cs Assets/Editor/LobbyBuilder.cs
git commit -m "Repoint track selection at the hub scene and rebuild it with authored framing"
```

---

### Task 8: Cutover — deletions, Build All, EditMode/PlayMode route-test updates

**Files:**
- Delete: `Assets/Scenes/05_LobbyScene.unity` + `.meta`, `Assets/Scenes/20_TrackSelectionScene.unity` + `.meta` (via `git rm` — recoverable from history)
- Modify: `Assets/Tests/EditMode/SceneContentTests.cs`
- Modify: `Assets/Tests/PlayMode/Phase3FlowSceneTests.cs`

**Interfaces:**
- Consumes: `FlowSceneNames.TrackSelection`/`RetiredGarageLobby`/`RetiredTrackSelection` (Task 7); the `Tools/Garage/Build All` chain (Tasks 6–7).
- Produces: retired scenes gone from disk and out of the build list; rebuilt hub scene + rewritten routing; scene-wiring tests asserting the hub invariants.

- [ ] **Step 1: Delete the retired scenes.** First confirm via `git status` that neither file carries uncommitted changes, then:

```bash
git rm Assets/Scenes/05_LobbyScene.unity Assets/Scenes/05_LobbyScene.unity.meta Assets/Scenes/20_TrackSelectionScene.unity Assets/Scenes/20_TrackSelectionScene.unity.meta
```

- [ ] **Step 2: Refresh Unity, then run Build All.** `refresh_unity` (the editor must let go of the deleted scenes — if either was open, close it first). Then `execute_menu_item` `Tools/Garage/Build All`. Confirm via `read_console`:
  - `[Garage] Authored difficulty labels on 5 track asset(s).`
  - `[Garage] Screen prefab written to ...` and `Track panel prefab written to ...`
  - `[Garage] Menu hub scene saved to Assets/Scenes/05_GarageScene.unity`
  - `[Garage] Removed 1 retired scene entr(ies) from the build list.`
  - `Flow root scenes -> lobby '05_GarageScene', track selection '05_GarageScene'.`
  - No errors.

- [ ] **Step 3: Rewrite the affected SceneContentTests.** In `Assets/Tests/EditMode/SceneContentTests.cs`:
  Replace `TrackSelectionScene_IsWiredCorrectly` (lines 112–129) and `GarageScene_IsWiredCorrectly` (lines 150–173) with:

```csharp
    [Test]
    public void HubScene_CarPanelHost_IsWiredCorrectly()
    {
        var scene = EditorSceneManager.OpenScene(
            $"Assets/Scenes/{FlowSceneNames.Lobby}.unity", OpenSceneMode.Single);

        var host = FindHost(scene, GameFlowManager.GameScreen.Lobby);
        Assert.IsNotNull(host, "The hub scene must carry a host declaring GameScreen.Lobby.");

        var prefab = HostScreenPrefab(host);
        Assert.IsNotNull(prefab, "The car panel host must have a screen prefab assigned.");
        Assert.IsNotNull(prefab.GetComponent<GarageScreen>(),
            "The car panel prefab must actually be a garage screen.");

        Assert.IsNotNull(Object.FindAnyObjectByType<GarageSceneController>(),
            "The hub scene must carry the garage scene controller.");
        Assert.IsNotNull(Object.FindAnyObjectByType<GarageTurntable>(),
            "The hub scene must carry a turntable for the displayed car.");
        Assert.IsNotNull(Object.FindAnyObjectByType<ShowcaseCarSwapper>(),
            "The hub scene must carry a showcase swapper.");
        Assert.IsNotNull(Object.FindAnyObjectByType<GarageHubView>(),
            "The hub scene must carry the stage manager that swaps the panels.");
    }

    [Test]
    public void HubScene_TrackPanelHost_IsWiredCorrectly()
    {
        var scene = EditorSceneManager.OpenScene(
            $"Assets/Scenes/{FlowSceneNames.TrackSelection}.unity", OpenSceneMode.Single);

        var host = FindHost(scene, GameFlowManager.GameScreen.TrackSelection);
        Assert.IsNotNull(host,
            "The hub scene must carry a host declaring GameScreen.TrackSelection; the track " +
            "panel is its second hosted screen.");
        Assert.IsFalse(host.gameObject.activeSelf,
            "The track panel host starts inactive; the car panel owns the stage on entry.");

        var prefab = HostScreenPrefab(host);
        Assert.IsNotNull(prefab, "The track panel host must have a screen prefab assigned.");
        Assert.IsNotNull(prefab.GetComponent<TrackPanelScreen>(),
            "The track panel prefab must actually be a track panel screen.");

        Assert.IsNotNull(Object.FindAnyObjectByType<TrackPanelSceneController>(FindObjectsInactive.Include),
            "The hub scene must carry the track panel scene controller.");
    }
```

  Replace `RetiredGarageLobby_IsOnDiskButOutOfTheBuildList` (lines 175–185) with:

```csharp
    [Test]
    public void RetiredMenuScenes_AreDeletedFromDiskAndOutOfTheBuildList()
    {
        // Both retired scenes are gone from disk — the user chose deletion over retention
        // for this rework — and must be gone from the build list, so no serialized
        // reference can load either of them.
        foreach (var name in new[] { FlowSceneNames.RetiredGarageLobby, FlowSceneNames.RetiredTrackSelection })
        {
            Assert.IsFalse(File.Exists(ToAbsolute($"Assets/Scenes/{name}.unity")),
                $"The retired scene '{name}' should have been deleted from disk.");
            Assert.IsFalse(
                System.Array.Exists(EditorBuildSettings.scenes,
                    s => Path.GetFileNameWithoutExtension(s.path) == name),
                $"The retired scene '{name}' must not be in the build list.");
        }
    }
```

  Replace `EveryFlowScene_HostsExactlyOneScreen` (lines 187–213) with:

```csharp
    [Test]
    public void EveryFlowScene_HostsEachScreenAtMostOnce()
    {
        // One HOST PER SCREEN. A scene may host several screens — the menu hub hosts the
        // car and track panels — but two hosts claiming the same screen is still the
        // unreachable-host defect the old one-per-scene rule guarded against.
        foreach (var name in new[]
                 {
                     FlowSceneNames.Loading,
                     FlowSceneNames.Lobby,
                     FlowSceneNames.WingSetup
                 })
        {
            var scene = EditorSceneManager.OpenScene(
                $"Assets/Scenes/{name}.unity", OpenSceneMode.Single);

            var seen = new System.Collections.Generic.HashSet<GameFlowManager.GameScreen>();
            foreach (var root in scene.GetRootGameObjects())
            {
                foreach (var host in root.GetComponentsInChildren<FlowSceneHost>(true))
                {
                    Assert.IsTrue(seen.Add(host.HostedScreen),
                        $"{name} hosts {host.HostedScreen} twice; one of them is unreachable.");
                }
            }

            Assert.Greater(seen.Count, 0, $"{name} must carry at least one screen host.");
        }

        // The hub carries exactly the pair that share its background.
        var hub = EditorSceneManager.OpenScene(
            $"Assets/Scenes/{FlowSceneNames.Lobby}.unity", OpenSceneMode.Single);
        var hubScreens = new System.Collections.Generic.HashSet<GameFlowManager.GameScreen>();
        foreach (var root in hub.GetRootGameObjects())
            foreach (var host in root.GetComponentsInChildren<FlowSceneHost>(true))
                hubScreens.Add(host.HostedScreen);

        Assert.IsTrue(hubScreens.SetEquals(new[]
        {
            GameFlowManager.GameScreen.Lobby, GameFlowManager.GameScreen.TrackSelection
        }), "The hub scene must host exactly the car and track panels.");
    }
```

  And add this helper next to `HostScreenPrefab`:

```csharp
    private static FlowSceneHost FindHost(UnityEditor.SceneManagement.Scene scene,
        GameFlowManager.GameScreen screen)
    {
        foreach (var root in scene.GetRootGameObjects())
            foreach (var host in root.GetComponentsInChildren<FlowSceneHost>(true))
                if (host.HostedScreen == screen)
                    return host;
        return null;
    }
```

  (`SceneContentTests.cs` already imports `System.IO` and `UnityEditor.SceneManagement`.)

  In `EveryFlowScene_HasExactlyOneEventSystem`, replace the scene array with
  `{ FlowSceneNames.Loading, FlowSceneNames.Lobby, FlowSceneNames.WingSetup }` — `FlowSceneNames.TrackSelection` now names the hub, so keeping it would open the same scene twice.

- [ ] **Step 4: Update Phase3FlowSceneTests.** In `Assets/Tests/PlayMode/Phase3FlowSceneTests.cs`:
  (a) After `BuildSettings_ExcludesTheRetiredGarageLobby` (line ~155), add:

```csharp
    [Test]
    public void BuildSettings_ExcludesTheRetiredTrackSelection()
    {
        foreach (var entry in UnityEditor.EditorBuildSettings.scenes)
        {
            Assert.AreNotEqual(FlowSceneNames.RetiredTrackSelection,
                Path.GetFileNameWithoutExtension(entry.path),
                "20_TrackSelectionScene was folded into 05_GarageScene and must not ship.");
        }
    }
```

  (b) In `EnteringLobbyScene_HostsItsScreen` (line ~262), replace the single-host lookup:

```csharp
        var host = UnityEngine.Object.FindAnyObjectByType<FlowSceneHost>();
        Assert.IsNotNull(host, $"{FlowSceneNames.Lobby} must carry a FlowSceneHost.");
        Assert.AreEqual(GameFlowManager.GameScreen.Lobby, host.HostedScreen);
```

  with a scan, because the hub now carries two hosts and an arbitrary one may be the track panel's:

```csharp
        FlowSceneHost host = null;
        foreach (var candidate in UnityEngine.Object.FindObjectsByType<FlowSceneHost>(
                     UnityEngine.FindObjectsInactive.Include, UnityEngine.FindObjectsSortMode.None))
        {
            if (candidate.HostedScreen == GameFlowManager.GameScreen.Lobby) host = candidate;
        }
        Assert.IsNotNull(host, $"{FlowSceneNames.Lobby} must carry a host declaring Lobby.");
        Assert.IsTrue(host.HasScreenInstance,
            "The host must have instantiated the hub screen from its prefab.");
```

- [ ] **Step 5: Run the suites.** EditMode full run — expect all green. PlayMode full run — green except the 2 documented pre-existing failures.
- [ ] **Step 6: Commit.**

```bash
git add Assets/Tests/EditMode/SceneContentTests.cs Assets/Tests/PlayMode/Phase3FlowSceneTests.cs
git commit -m "Cut track selection over to the hub scene and retire its standalone scene"
```

  (The `git rm` from Step 1 is already staged; committing after staging the two modified files commits everything staged. Verify with `git status` first that ONLY the four intended paths are staged.)

---

### Task 9: PlayMode — hub panel-swap behavior tests

**Files:**
- Create: `Assets/Tests/PlayMode/TrackPanelFlowTests.cs`

**Interfaces:**
- Consumes: everything built so far; the `ProfileIsolation`/`SceneOpWait` test helpers used by `GarageSelectionTests`.

- [ ] **Step 1: Create the file.**

```csharp
using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using UnityEngine.UI;
using F1.GameFlow;
using F1.Progression;

/// <summary>
/// The hub's panel contract, driven through the real scene and the real flow: NEXT on
/// the car panel swaps to the track panel IN PLACE — the background scene never reloads,
/// the session state moves to TrackSelection, and the showcased car leaves the stage.
/// BACK returns the same way; CONFIRM only fires with a raceable session track; wing
/// setup's back-route lands on the hub with the track panel showing.
///
/// Clicks go through the real buttons' onClick so the self-wiring is under test.
/// </summary>
public class TrackPanelFlowTests
{
    private GameObject _flowObject;
    private GameFlowManager _flow;

    [UnitySetUp]
    public IEnumerator SetUp()
    {
        ProfileIsolation.Begin();
        yield return null;
    }

    [UnityTearDown]
    public IEnumerator TearDown()
    {
        var sceneFlow = GameFlowManager.Instance?.SceneFlow;
        if (sceneFlow != null)
        {
            yield return SceneOpWait.UnloadAllContentBounded(sceneFlow);
            yield return SceneOpWait.UnloadSceneBounded(sceneFlow.CurrentFlowScene);
        }

        if (_flowObject != null) Object.DestroyImmediate(_flowObject);

        ProfileIsolation.End();
    }

    private IEnumerator EnterGarage()
    {
        _flowObject = new GameObject("TrackPanelTest_Flow");
        _flowObject.AddComponent<GameFlowManager>();
        _flowObject.AddComponent<FlowRoot>();
        _flow = GameFlowManager.Instance;

        _flow.SceneFlow?.AdoptFlowScene(null);
        yield return WaitUntil(() => _flow.SceneFlow == null || !_flow.SceneFlow.IsBusy, 20f);
        yield return null;

        _flow.GoToLobby();
        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.Lobby, 20f);
        yield return WaitUntil(() => _flow.LobbyScreenInstance != null, 20f);
        yield return WaitUntil(() => _flow.SelectedCar != null, 10f);
        yield return null;
    }

    private IEnumerator PressContinueAndSettleOnTheTrackPanel()
    {
        FindCarPanelButton("ContinueButton").onClick.Invoke();
        yield return WaitUntil(
            () => _flow.CurrentScreen == GameFlowManager.GameScreen.TrackSelection, 10f);
        yield return null;
    }

    private static FlowSceneHost Host(GameFlowManager.GameScreen screen)
    {
        foreach (var host in Object.FindObjectsByType<FlowSceneHost>(
                     FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (host.HostedScreen == screen)
                return host;
        return null;
    }

    private static Button FindCarPanelButton(string name)
    {
        var screen = Host(GameFlowManager.GameScreen.Lobby).ScreenInstance as Component;
        Assert.IsNotNull(screen, "No car panel is hosted.");
        var button = screen.transform.Find(name)?.GetComponent<Button>();
        Assert.IsNotNull(button, $"The car panel has no '{name}' button.");
        return button;
    }

    private static Button FindTrackPanelButton(string name)
    {
        var screen = Host(GameFlowManager.GameScreen.TrackSelection).ScreenInstance as Component;
        Assert.IsNotNull(screen, "No track panel is hosted.");
        var button = screen.transform.Find(name)?.GetComponent<Button>();
        Assert.IsNotNull(button, $"The track panel has no '{name}' button.");
        return button;
    }

    private static IEnumerator WaitUntil(System.Func<bool> condition, float timeoutSeconds)
    {
        float deadline = Time.realtimeSinceStartup + timeoutSeconds;
        while (!condition() && Time.realtimeSinceStartup < deadline)
            yield return null;
    }

    [UnityTest]
    public IEnumerator Continue_SwapsToTheTrackPanelInTheSameScene()
    {
        yield return EnterGarage();

        yield return PressContinueAndSettleOnTheTrackPanel();

        Assert.AreEqual(FlowSceneNames.Lobby, SceneManager.GetActiveScene().name,
            "Track selection must not reload the scene; the hub background persists.");
        Assert.AreEqual(GameFlowState.TrackSelection, _flow.CurrentFlowState);
        Assert.IsTrue(Host(GameFlowManager.GameScreen.TrackSelection).gameObject.activeSelf,
            "The track panel host should be on stage.");
        Assert.IsFalse(Host(GameFlowManager.GameScreen.Lobby).gameObject.activeSelf,
            "The car panel host should be off stage.");
        Assert.IsNotNull(_flow.SelectedTrack,
            "Focusing the first raceable circuit should have selected it.");
    }

    [UnityTest]
    public IEnumerator Continue_TakesTheShowcaseCarOffStage()
    {
        yield return EnterGarage();
        var swapper = Object.FindAnyObjectByType<ShowcaseCarSwapper>();
        Assert.IsNotNull(swapper);
        Assert.IsTrue(swapper.activeInHierarchy, "The car should be on stage in the garage.");

        yield return PressContinueAndSettleOnTheTrackPanel();

        Assert.IsFalse(swapper.activeInHierarchy,
            "The reference shows the turntable empty during track selection.");
    }

    [UnityTest]
    public IEnumerator Back_ReturnsToTheCarPanelInTheSameScene()
    {
        yield return EnterGarage();
        yield return PressContinueAndSettleOnTheTrackPanel();

        FindTrackPanelButton("BackButton").onClick.Invoke();
        yield return WaitUntil(
            () => _flow.CurrentScreen == GameFlowManager.GameScreen.Lobby, 10f);
        yield return null;

        Assert.AreEqual(FlowSceneNames.Lobby, SceneManager.GetActiveScene().name);
        Assert.IsTrue(Host(GameFlowManager.GameScreen.Lobby).gameObject.activeSelf);
        Assert.IsFalse(Host(GameFlowManager.GameScreen.TrackSelection).gameObject.activeSelf);
        Assert.IsNotNull(_flow.SelectedCar, "The session car must survive the round trip.");
    }

    [UnityTest]
    public IEnumerator FocusingAnUnraceableTrack_ParksConfirmAndKeepsTheSessionTrack()
    {
        yield return EnterGarage();
        yield return PressContinueAndSettleOnTheTrackPanel();

        var sessionTrack = _flow.SelectedTrack;
        Assert.IsNotNull(sessionTrack);

        var confirm = FindTrackPanelButton("ConfirmButton");
        int cycleLength = ProgressionRegistry.GetOwnedTracks().Count
            + ProgressionRegistry.GetLockedTracks().Count;

        // An unraceable focus is identifiable from the outside: Confirm dark AND the
        // session track unchanged. The cycle is guaranteed to contain at least one (Spa
        // is owned but shares Track_01 and has no scene of its own; locked circuits are
        // refused by SelectTrack's ownership guard), but not at a fixed index, so cycle
        // until one turns up.
        bool found = false;
        for (int i = 0; i < cycleLength && !found; i++)
        {
            FindTrackPanelButton("NextButton").onClick.Invoke();
            yield return null;
            found = !confirm.interactable && _flow.SelectedTrack == sessionTrack;
        }

        Assert.IsTrue(found,
            "The cycle should contain an unraceable circuit (owned Spa has no scene).");
        Assert.IsNotNull(_flow.SelectedTrack, "The session track must never drop to null.");
        Assert.IsFalse(confirm.interactable,
            "An unraceable focus must not let Confirm promise a race the flow refused.");
    }

    [UnityTest]
    public IEnumerator Confirm_GoesToWingSetup()
    {
        yield return EnterGarage();
        yield return PressContinueAndSettleOnTheTrackPanel();

        FindTrackPanelButton("ConfirmButton").onClick.Invoke();
        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.WingSetup, 20f);
        yield return WaitUntil(
            () => _flow.CurrentScreen == GameFlowManager.GameScreen.WingSetup, 10f);

        Assert.AreEqual(FlowSceneNames.WingSetup, SceneManager.GetActiveScene().name);
        Assert.IsNotNull(_flow.SelectedTrack);
    }

    [UnityTest]
    public IEnumerator ReturningFromWingSetup_LandsOnTheHubTrackPanel()
    {
        yield return EnterGarage();
        yield return PressContinueAndSettleOnTheTrackPanel();
        var selected = _flow.SelectedTrack;
        FindTrackPanelButton("ConfirmButton").onClick.Invoke();
        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.WingSetup, 20f);

        // This is exactly what WingSetupSceneController.OnBackPressed does.
        _flow.GoToTrackSelection();
        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.Lobby, 20f);
        yield return WaitUntil(
            () => _flow.CurrentScreen == GameFlowManager.GameScreen.TrackSelection, 10f);
        yield return null;

        Assert.AreEqual(FlowSceneNames.Lobby, SceneManager.GetActiveScene().name,
            "Wing back must land on the hub scene.");
        Assert.AreEqual(GameFlowManager.GameScreen.TrackSelection, _flow.CurrentScreen,
            "Wing back must show the track panel, not the car panel.");
        Assert.AreEqual(selected.TrackId, _flow.SelectedTrack.TrackId,
            "The session track must survive the round trip.");
        Assert.IsTrue(Host(GameFlowManager.GameScreen.TrackSelection).gameObject.activeSelf);
    }

    [UnityTest]
    public IEnumerator CycleNext_WrapsThroughEveryTrack()
    {
        yield return EnterGarage();
        yield return PressContinueAndSettleOnTheTrackPanel();

        int cycleLength = ProgressionRegistry.GetOwnedTracks().Count
            + ProgressionRegistry.GetLockedTracks().Count;
        Assert.Greater(cycleLength, 1, "The cycle needs at least two circuits to wrap.");

        // The centre slot's name is the visible focus. The builder authors that child as
        // SlotCentre/Name — the same Find-by-name grammar the button helpers use.
        var screen = Host(GameFlowManager.GameScreen.TrackSelection).ScreenInstance as Component;
        Assert.IsNotNull(screen, "No track panel is hosted.");
        var centreName = screen.transform.Find("SlotCentre/Name")?.GetComponent<TMP_Text>();
        Assert.IsNotNull(centreName, "The track panel must expose the centre slot's name.");

        string start = centreName.text;
        var seen = new HashSet<string> { start };
        for (int i = 0; i < cycleLength; i++)
        {
            FindTrackPanelButton("NextButton").onClick.Invoke();
            yield return null;
            seen.Add(centreName.text);
        }

        Assert.AreEqual(cycleLength, seen.Count,
            "A full cycle must visit every circuit exactly once.");
        Assert.AreEqual(start, centreName.text,
            "After a full cycle the focus must be back where it started.");
        Assert.IsNotNull(_flow.SelectedTrack, "The session track must survive a full cycle.");
    }
}
```

- [ ] **Step 2: Run.** `refresh_unity` not needed (no code change) — run PlayMode filtered to `TrackPanelFlowTests`. All 7 green.
- [ ] **Step 3: Commit.**

```bash
git add Assets/Tests/PlayMode/TrackPanelFlowTests.cs Assets/Tests/PlayMode/TrackPanelFlowTests.cs.meta
git commit -m "Cover the hub panel swaps, confirm gating and wing round trip in play mode"
```

---

### Task 10: Full verification — suites, visuals, camera tuning, tree audit

**Files:**
- Modify: `Assets/Editor/GarageBuilder.cs` (only the framing constants, if tuning is needed)

- [ ] **Step 1: Full suites.** `run_tests` EditMode (all green) and PlayMode (green except the 2 documented failures). Compare counts against the Task 0 baseline.
- [ ] **Step 2: Visual pass — car panel.** Via Unity MCP: open `Assets/Scenes/00_LoadingScene.unity`, enter play mode, wait for the garage, `manage_camera` screenshot. Check against `ReferenceImageCarselection.png`: car head-on and centred, profile top-left, currency top-right, stats left, arrows flanking, Continue lit on the starter. Exit play.
- [ ] **Step 3: Tune the framing if needed.** If the car is off-centre/too small/clipped: edit `SocketPosition/SocketYawDegrees/CameraPosition/CameraLookAt/CameraFov` in `GarageBuilder.cs`, `refresh_unity`, re-run `Tools/Garage/Build Garage Scene` **and** `Tools/Garage/Update Build Settings && Flow Root`, re-play, re-screenshot. Iterate until it matches `ReferenceImageLobby.png`'s composition. Commit any constant changes.
- [ ] **Step 4: Visual pass — track panel.** Play again, click Continue (via MCP `execute_code` invoking the button or a scripted click), screenshot: SELECT TRACK title, 3 cards with centre highlighted, chrome identical to the car panel, **car absent**, platform empty. Cycle Next a few times — centre hero changes, sides stay dimmed, wrap works.
- [ ] **Step 5: Visual pass — locked focus.** Cycle to Spa: stats render, CONFIRM dark. Click BACK — car panel returns with the car on stage.
- [ ] **Step 6: Full flow.** Continue → track panel → Confirm → wing scene (unchanged UI) → pick a wing → continue → pre-race + qualifying load. From wing, go BACK → hub with track panel showing.
- [ ] **Step 7: Tree audit.** `git status`: the AI/spline WIP files are untouched; `05_LobbyScene.unity`/`20_TrackSelectionScene.unity` show as deleted; no unexpected files. `read_console` shows no errors from the play session.
- [ ] **Step 8: Final commit (if tuning changed anything).**

```bash
git add Assets/Editor/GarageBuilder.cs
git commit -m "Tune the hub camera framing to the lobby reference"
```

---

## Risks & mitigations

| Risk | Mitigation |
|---|---|
| Serialized `_trackSelectionScene` in 00_LoadingScene keeps pointing at the retired scene | `UpdateBuildSettingsAndFlowRoot` rewrites it explicitly; `BuildSettings_ExcludesTheRetiredTrackSelection` + `SceneAvailability`-based controller guards fail loudly otherwise |
| Two hosts per scene breaks an assumption somewhere | `EveryFlowScene_HostsEachScreenAtMostOnce` re-states the real invariant (unique screen per scene); `FindSceneHost` matches by `HostedScreen` over registered hosts regardless of active state |
| Track panel controller binds while the car panel owns the stage (auto-selects a track at boot) | Host root starts inactive; Start defers until `GarageHubView` raises it, which happens only when the flow's current screen is TrackSelection |
| Deleted scenes still referenced by a stale serialized field elsewhere | Build-list exclusion tests + full flow play-through in Task 10 |
| `Tools > Lobby > Build Wing Setup Lobby` errors after deletion | Header warning in LobbyBuilder; wing scene is already built and out of scope |
| Same-scene transition raises `OnFlowSceneLoadStarted` and flashes a loading card | `FlowOverlay.OnFlowLoadStarted` filters to PreRace/Race only (verified — FlowOverlay.cs:125) |

## Rollback

Revert the commits (`git revert` or reset the branch); `git checkout HEAD~N -- Assets/Scenes/05_LobbyScene.unity Assets/Scenes/20_TrackSelectionScene.unity` restores both deleted scenes from history; re-run `Tools > Garage/Build All` to regenerate prefab/scene/routing. `05_GarageScene` is fully builder-generated, so rollback is total.
