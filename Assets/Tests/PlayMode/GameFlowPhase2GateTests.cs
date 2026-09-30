using NUnit.Framework;
using UnityEngine;
using F1.GameData;
using F1.GameFlow;
using WingType = F1.GameData.WingType;

/// <summary>
/// Guards the Phase 2 exit gates that are not covered by
/// <see cref="SceneFlowServiceTests"/>: the data registry is populated, and the flow
/// manager really does have a single source of truth for the current selection.
/// </summary>
public class GameFlowPhase2GateTests
{
    private GameObject _flowObject;

    [SetUp]
    public void SetUp()
    {
        // These tests must not depend on whatever the developer's local save file
        // happens to contain. Ensure the free starter car and tracks are available on
        // the live profile so car/track selection is exercisable.
        ProfileIsolation.Begin();
        GameDataRegistry.Initialize();
        F1.Progression.PlayerProfileManager.GrantStarterContent(
            F1.Progression.PlayerProfileManager.Current);

        _flowObject = new GameObject("Phase2GateTest_Flow");
        _flowObject.AddComponent<GameFlowManager>();
    }

    [TearDown]
    public void TearDown()
    {
        if (_flowObject != null)
            Object.DestroyImmediate(_flowObject);

        ProfileIsolation.End();
    }

    private GameFlowManager Flow => GameFlowManager.Instance;

    // --- Data registry (finding B2) ---

    [Test]
    public void Registry_HasAtLeastOneCar()
    {
        GameDataRegistry.Initialize();
        Assert.Greater(GameDataRegistry.AllCars.Count, 0,
            "No CarDefinition assets exist. Phase 4's exit gate is unreachable until this is fixed (finding B2).");
    }

    [Test]
    public void Registry_HasAtLeastOneTrack()
    {
        GameDataRegistry.Initialize();
        Assert.Greater(GameDataRegistry.AllTracks.Count, 0,
            "No TrackDefinition assets exist (finding B2).");
    }

    [Test]
    public void Registry_HasAFreeStarterCar()
    {
        GameDataRegistry.Initialize();
        Assert.Greater(GameDataRegistry.FreeCars.Count, 0,
            "The GDD requires a free starter car; a fresh profile can select nothing.");
    }

    [Test]
    public void Registry_HasTwoFreeTracks()
    {
        GameDataRegistry.Initialize();
        Assert.GreaterOrEqual(GameDataRegistry.FreeTracks.Count, 2,
            "The GDD requires 2 free tracks.");
    }

    [Test]
    public void Registry_EveryFreeTrackPointsAtALoadableScene()
    {
        // A free track the player can select must be routable to. Locked tracks keep
        // their intended scene names, which Phase 5 creates.
        GameDataRegistry.Initialize();
        foreach (var track in GameDataRegistry.FreeTracks)
        {
            Assert.IsNotEmpty(track.SceneName, $"Free track '{track.TrackId}' has no scene name.");
            Assert.IsTrue(SceneFlowService.IsSceneAvailable(track.SceneName),
                $"Free track '{track.TrackId}' points at '{track.SceneName}', which cannot be loaded.");
        }
    }

    [Test]
    public void Registry_CarIdsAndTrackIdsAreUnique()
    {
        GameDataRegistry.Initialize();

        var carIds = new System.Collections.Generic.HashSet<string>();
        foreach (var car in GameDataRegistry.AllCars)
        {
            Assert.IsNotEmpty(car.CarId, $"Car '{car.name}' has an empty CarId.");
            Assert.IsTrue(carIds.Add(car.CarId), $"Duplicate CarId '{car.CarId}'.");
        }

        var trackIds = new System.Collections.Generic.HashSet<string>();
        foreach (var track in GameDataRegistry.AllTracks)
        {
            Assert.IsNotEmpty(track.TrackId, $"Track '{track.name}' has an empty TrackId.");
            Assert.IsTrue(trackIds.Add(track.TrackId), $"Duplicate TrackId '{track.TrackId}'.");
        }
    }

    [Test]
    public void Registry_EveryCarHasAPhysicsProfileAndBothWings()
    {
        // CreatePlayerPhysicsProfile depends on these being wired; a null here would
        // surface much later as a car that spawns with no handling model.
        GameDataRegistry.Initialize();
        foreach (var car in GameDataRegistry.AllCars)
        {
            Assert.IsNotNull(car.BasePhysicsProfile,
                $"Car '{car.CarId}' has no base physics profile.");
            Assert.IsNotNull(car.HighDownforceAero,
                $"Car '{car.CarId}' has no high-downforce aero profile.");
            Assert.IsNotNull(car.LowDownforceAero,
                $"Car '{car.CarId}' has no low-downforce aero profile.");
        }
    }

    [Test]
    public void Registry_FreeCarsExcludeRentalVariants()
    {
        // Rental variants also have UnlockCostPoints == 0 (they cost money to rent, not
        // points to own). If they leaked into FreeCars, GrantStarterContent would hand a
        // new profile whichever sorted first — a rental — and ownership would be wrong.
        GameDataRegistry.Initialize();
        foreach (var car in GameDataRegistry.FreeCars)
            Assert.IsFalse(car.IsRental, $"Rental car '{car.CarId}' must not be in FreeCars.");

        foreach (var car in GameDataRegistry.AllCars)
        {
            if (!car.IsRental)
                continue;

            bool listed = false;
            foreach (var candidate in GameDataRegistry.RentalCars)
            {
                if (candidate.CarId == car.CarId) { listed = true; break; }
            }

            Assert.IsTrue(listed, $"Rental car '{car.CarId}' must appear in RentalCars.");
        }
    }

    // --- Car list partitioning (every car must appear exactly once) ---

    [Test]
    public void CarLists_PartitionTheRegistryWithoutDuplicates()
    {
        // GetAvailableCars used to treat every non-owned car as rentable, so locked cars
        // were listed as both available and locked and the selection grid rendered each
        // car twice.
        GameDataRegistry.Initialize();
        F1.Progression.PlayerProfileManager.GrantStarterContent(
            F1.Progression.PlayerProfileManager.Current);

        var profile = F1.Progression.PlayerProfileManager.Current;
        var owned = F1.Progression.ProgressionRegistry.GetOwnedCars();
        var available = F1.Progression.ProgressionRegistry.GetAvailableCars();
        var locked = F1.Progression.ProgressionRegistry.GetLockedCars();

        int total = owned.Count + available.Count + locked.Count;
        Assert.AreEqual(GameDataRegistry.AllCars.Count, total,
            "Owned + available + locked must account for each car exactly once.");

        var seen = new System.Collections.Generic.HashSet<string>();
        foreach (var list in new[] { owned, available, locked })
        {
            foreach (var car in list)
            {
                Assert.IsTrue(seen.Add(car.CarId),
                    $"Car '{car.CarId}' appears in more than one car list, so the grid duplicates it.");
            }
        }
    }

    [Test]
    public void AvailableCars_ExcludePointsGatedLockedCars()
    {
        GameDataRegistry.Initialize();
        F1.Progression.PlayerProfileManager.GrantStarterContent(
            F1.Progression.PlayerProfileManager.Current);

        var profile = F1.Progression.PlayerProfileManager.Current;
        var available = F1.Progression.ProgressionRegistry.GetAvailableCars();

        foreach (var car in available)
        {
            bool usable = profile.OwnsCar(car.CarId)
                          || profile.HasActiveRental(car.CarId)
                          || car.IsRental;
            Assert.IsTrue(usable,
                $"Car '{car.CarId}' is reported available but the player can neither own, rent, nor start a rental on it.");
        }
    }

    [Test]
    public void AvailableCars_IncludeRentalVariants()
    {
        GameDataRegistry.Initialize();
        var available = F1.Progression.ProgressionRegistry.GetAvailableCars();

        foreach (var car in GameDataRegistry.RentalCars)
        {
            bool listed = false;
            foreach (var candidate in available)
            {
                if (candidate.CarId == car.CarId) { listed = true; break; }
            }

            Assert.IsTrue(listed, $"Rental car '{car.CarId}' must be offered to the player.");
        }
    }

    // --- Starter content (the CreateNewProfile stub that used to grant nothing) ---

    [Test]
    public void StarterContent_GrantsOneFreeCarAndEveryFreeTrack()
    {
        GameDataRegistry.Initialize();
        var profile = new F1.Progression.PlayerProfile();

        F1.Progression.PlayerProfileManager.GrantStarterContent(profile);

        Assert.AreEqual(1, profile.ownedCarIds.Count,
            "A new profile should own exactly one starter car, not the whole free tier.");
        var starterId = profile.ownedCarIds[0];
        var starter = GameDataRegistry.GetCar(starterId);
        Assert.IsNotNull(starter, $"Starter car '{starterId}' is not in the registry.");
        Assert.IsFalse(starter.IsRental, "The granted starter car must not be a rental variant.");

        foreach (var track in GameDataRegistry.FreeTracks)
            Assert.IsTrue(profile.OwnsTrack(track.TrackId),
                $"Free track '{track.TrackId}' was not granted to a new profile.");
    }

    [Test]
    public void StarterContent_MakesTheStarterCarSelectableThroughTheFlow()
    {
        // The end-to-end shape of the bug this fixes: a fresh profile could not select
        // anything, so the flow dead-ended at car selection.
        GameDataRegistry.Initialize();
        var profile = new F1.Progression.PlayerProfile();
        F1.Progression.PlayerProfileManager.GrantStarterContent(profile);

        var starterId = profile.ownedCarIds[0];
        var car = GameDataRegistry.GetCar(starterId);
        var track = GameDataRegistry.FreeTracks[0];

        var flow = Flow;
        flow.SelectCar(car);
        flow.SelectTrack(track);

        Assert.AreSame(car, flow.SelectedCar, "The starter car must be selectable through the flow.");
        Assert.AreSame(track, flow.SelectedTrack, "A free track must be selectable through the flow.");
        Assert.IsTrue(flow.Session.IsSelectionComplete);
    }

    // --- Single source of truth (finding L1) ---

    [Test]
    public void Selection_IsReadThroughTheSessionContext()
    {
        var flow = Flow;
        Assert.IsNotNull(flow, "GameFlowManager.Instance should exist after Awake.");

        var car = GameDataRegistry.GetCar("car_gen1_starter");
        var track = GameDataRegistry.GetTrack("track_monaco");
        Assert.IsNotNull(car, "The free starter car should exist in the registry.");
        Assert.IsNotNull(track, "The free Monaco track should exist in the registry.");

        flow.SelectCar(car);
        flow.SelectTrack(track);
        flow.SelectWing(WingType.LowDownforce);

        // The manager's public properties and the session must be the same object.
        Assert.AreSame(flow.Session.SelectedCar, flow.SelectedCar);
        Assert.AreSame(flow.Session.SelectedTrack, flow.SelectedTrack);
        Assert.AreEqual(flow.Session.SelectedWing, flow.SelectedWing);
        Assert.AreEqual(WingType.LowDownforce, flow.SelectedWing);

        // And the values actually round-trip through the flow.
        Assert.AreEqual("car_gen1_starter", flow.SelectedCar.CarId);
        Assert.AreEqual("track_monaco", flow.SelectedTrack.TrackId);
        Assert.AreEqual("Track_01", flow.GetSelectedTrackSceneName());
    }

    [Test]
    public void Selection_SurvivesReadingBackEveryAccessor()
    {
        var flow = Flow;
        var car = GameDataRegistry.GetCar("car_gen1_starter");
        var track = GameDataRegistry.GetTrack("track_spa");

        flow.SelectCar(car);
        flow.SelectTrack(track);

        // Reading accessors in any order must not perturb the selection.
        for (int i = 0; i < 5; i++)
        {
            Assert.AreSame(car, flow.SelectedCar);
            Assert.AreSame(track, flow.SelectedTrack);
            Assert.AreSame(car, flow.Session.SelectedCar);
            Assert.AreSame(track, flow.Session.SelectedTrack);
        }
    }

    [Test]
    public void CreatePlayerPhysicsProfile_UsesTheSelectedWing()
    {
        var flow = Flow;
        var car = GameDataRegistry.GetCar("car_gen1_starter");
        Assert.IsNotNull(car);

        flow.SelectCar(car);
        flow.SelectWing(WingType.HighDownforce);
        var high = flow.CreatePlayerPhysicsProfile();

        flow.SelectWing(WingType.LowDownforce);
        var low = flow.CreatePlayerPhysicsProfile();

        Assert.IsNotNull(high, "A physics profile must be produced for a selected car.");
        Assert.IsNotNull(low, "A physics profile must be produced for a selected car.");
    }

    [Test]
    public void CreatePlayerPhysicsProfile_NullWithoutASelectedCar()
    {
        var flow = Flow;
        Assert.IsNull(flow.CreatePlayerPhysicsProfile(),
            "No car selected must yield null, not an exception.");
    }

    // --- ModeSelection removal (finding L3) ---

    [Test]
    public void GameScreen_HasNoModeSelectionMember()
    {
        // Invariant 15: ModeSelectionScreen is not part of the active route.
        foreach (var name in System.Enum.GetNames(typeof(GameFlowManager.GameScreen)))
            Assert.AreNotEqual("ModeSelection", name,
                "ModeSelection must not exist in the flow's screen enum (finding L3).");
    }

    [Test]
    public void GameFlowState_HasNoModeSelectionMember()
    {
        foreach (var name in System.Enum.GetNames(typeof(GameFlowState)))
            Assert.AreNotEqual("ModeSelection", name,
                "ModeSelection must not exist in the flow state enum.");
    }

    [Test]
    public void FlowManager_HasNoGoToModeSelectionMethod()
    {
        var method = typeof(GameFlowManager).GetMethod("GoToModeSelection");
        Assert.IsNull(method,
            "GoToModeSelection() must be removed from the flow (finding L3).");
    }
}
