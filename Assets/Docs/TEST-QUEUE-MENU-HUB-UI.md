# Test Queue — Menu Hub / Garage + Track UI Rework

Deferred test execution for the menu-hub UI rework (PLAN-MENU-HUB-TRACK-UI.md). All
development tasks (7-9) were completed with test **files** written but **not run**, per
explicit instruction. This file is the queue: execute it top to bottom when the user
says so, and record the outcome of each entry in the Result column.

How to run: Unity MCP `run_tests` with the given `test_mode`; for a single fixture use
its full name in `group_names`/filter. Poll `get_test_job` until the job finishes.
If the editor was unfocused, activate the Unity window first (known throttling issue).

Documented pre-existing failures that are NOT caused by this rework and are OUT OF
SCOPE (per user directive, do not chase them):
- `Phase3FlowSceneTests.FutureFlowScenes_AreNamedButNotYetBuilt` (stale, documented)
- `HostScreenMode_QualifyingOrRace_FollowsTheSessionType` (stale, documented)
- AI/spline PlayMode fixtures (`AIRealTrack*`, `AILongitudinalPlanner*`) — user WIP
- Qualifying/race shell issues (S4 hang) — pre-existing, unrelated to menu UI

---

## Queue 1 — EditMode: new fixtures from this rework

| # | Fixture | Expect | Result |
|---|---------|--------|--------|
| 1.1 | `GarageDisplayStatsTests` (8 tests: Normalize01_* x5, EveryCarAsset_HasGarageDisplayStatsAuthored, EveryCarAsset_HasAPairwiseDistinctTint, EveryCarAsset_StatsSitInsideThePanelRanges) | all green | |
| 1.2 | `PlayerProfileProgressTests` (8 tests: XpPerLevel_IsStillOneThousand, XpProgress_* x6, OldSave_WithoutTeamName_DefaultsTheTeam) | all green | |
| 1.3 | `TrackMenuDisplayTests` (EveryTrack_HasADifficultyLabelAndCoreStats, DifficultyLabels_AreUppercaseShortLabels) | all green | |

## Queue 2 — EditMode: rewritten scene-wiring suite

| # | Test | Expect | Result |
|---|------|--------|--------|
| 2.1 | `SceneContentTests` — full fixture, incl. the new `HubScene_CarPanelHost_IsWiredCorrectly`, `HubScene_TrackPanelHost_IsWiredCorrectly`, `RetiredMenuScenes_AreDeletedFromDiskAndOutOfTheBuildList`, `EveryFlowScene_HostsEachScreenAtMostOnce` | all green | |

## Queue 3 — EditMode: full suite gate

| # | Run | Expect | Result |
|---|-----|--------|--------|
| 3.1 | Full EditMode run | all green; compare count vs Task 0 baseline (+new tests above) | |

## Queue 4 — PlayMode: hub + garage behavior

| # | Fixture | Expect | Result |
|---|---------|--------|--------|
| 4.1 | `GarageSelectionTests` (4 tests: EnteringTheGarage_FocusesAndSelectsTheStarterCar, CyclingNext_SixTimesWrapsThroughEveryCar, FocusingALockedCar_ParksContinueAndKeepsTheSessionCar, CyclingBackToAnAvailableCar_RestoresContinueAndTheSessionCar) | all green | |
| 4.2 | `TrackPanelFlowTests` (7 tests: Continue_SwapsToTheTrackPanelInTheSameScene, Continue_TakesTheShowcaseCarOffStage, Back_ReturnsToTheCarPanelInTheSameScene, FocusingAnUnraceableTrack_ParksConfirmAndKeepsTheSessionTrack, Confirm_GoesToWingSetup, ReturningFromWingSetup_LandsOnTheHubTrackPanel, CycleNext_WrapsThroughEveryTrack) | all green | |
| 4.3 | `Phase3FlowSceneTests` — full fixture, incl. the updated `EnteringLobbyScene_HostsItsScreen` (two-host scan) and new `BuildSettings_ExcludesTheRetiredTrackSelection`; expect ONLY the documented `FutureFlowScenes_AreNamedButNotYetBuilt` failure | green except 1 documented | |

## Queue 5 — PlayMode: full suite gate

| # | Run | Expect | Result |
|---|-----|--------|--------|
| 5.1 | Full PlayMode run | green EXCEPT the documented pre-existing failures listed at the top (AI WIP, S4 shell, the 2 stale). Any NEW failure in a menu/flow/UI fixture is a rework regression — investigate before proceeding | |

---

## Acceptance

- Queues 1-4: zero failures.
- Queue 5: no NEW failures versus the pre-rework baseline recorded in Hub Task 0;
  only the documented out-of-scope failures listed above.
- After the queue: record outcomes here, then report to the user.
