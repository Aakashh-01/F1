using System.Linq;
using NUnit.Framework;
using UnityEngine;
using UnityEditor.SceneManagement;

/// <summary>
/// Every AI car's lap progress depends on two things that were only accidentally true.
///
/// <para>
/// <see cref="LapTracker"/> asks two questions every tick: which track am I measuring
/// against, and which layer is that track's surface on. Both have to be right before any
/// distance can be banked, and both fail silently — a tracker pointed at nothing, or one
/// raycasting a layer the tarmac is not on, simply never passes its grounded check, so
/// <see cref="LapTracker.TotalDistance"/> stays at zero forever. No exception, no log line.
/// The field then ranks level, lap times stay at 0, and the race HUD position is fiction.
/// </para>
///
/// <para>
/// The player's answers are set explicitly at spawn: <c>PlayerCarSpawner</c> binds the track
/// and points <c>TrackLayer</c> at the Ground layer. The AI's were not. They relied on the
/// tracker finding the track itself in <c>Awake</c>, and on the AI prefab happening to
/// serialize <c>_trackLayer: 3</c> — which is only right while layer 3 is still Ground.
/// Renumber the layers, or spawn the grid before the track content exists, and every AI
/// silently stops banking lap distance. See the failure written out at
/// <c>LapTracker.TrackLayer</c>.
/// </para>
///
/// <para>
/// So these assert that <see cref="RaceGridManager"/> — the one place every AI car is
/// configured — hands each car's tracker the same track the player is bound to, and that
/// track surface's <i>actual</i> layer. The layer is asserted against the live collider
/// rather than a literal, which is what makes it a real guard: the fixture puts its surface
/// on a layer nothing in the project uses, so a hard-coded 3 fails here.
/// </para>
/// </summary>
public class AITrackBindingTests
{
    /// <summary>
    /// A layer nothing in the project uses, and emphatically not 3.
    ///
    /// TagManager defines 0 Default, 1 TransparentFX, 2 Ignore Raycast, 3 Ground, 4 Water,
    /// 5 UI, 6 Traffic, and leaves 7..31 unnamed. The track fixture puts its surface here so
    /// that "the assigned layer equals the surface collider's layer" is an assertion about
    /// the real value, rather than an accident that happens to agree with a hard-coded one.
    /// The test car's own colliders stay on Default, so the road is the only thing the
    /// grounded raycast can possibly hit — the same situation as a real AI car, whose
    /// colliders are on Traffic while the tarmac is on Ground.
    /// </summary>
    private const int UnusualSurfaceLayer = 8;

    /// <summary>Radius of the circular test circuit, in metres.</summary>
    private const float CircuitRadius = 40f;

    private const int WaypointCount = 12;

    private GameObject _root;
    private GameObject _surface;
    private GameObject _placementGo;
    private GameObject _aiCarTemplate;
    private AIRacingLine _line;
    private TrackPlacement _placement;
    private RaceGridManager _grid;

    [SetUp]
    public void SetUp()
    {
        // An empty scene, so the only TrackPlacement that exists is the one built below.
        // Sibling tests open Track_01, which contains a real one — and a "no track present"
        // assertion that quietly found that one would be testing nothing at all.
        EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        _root = new GameObject("AITrackBindingFixture");
        BuildSurface();
        BuildRacingLine();
        BuildTrackPlacement();
        BuildAITemplate();
        BuildGrid();
    }

    [TearDown]
    public void TearDown()
    {
        if (_grid != null) _grid.DestroySpawnedGrid();
        if (_root != null) Object.DestroyImmediate(_root);
    }

    // --- The defect ---

    [Test]
    public void GridSpawn_BindsEveryAiTrackerToTheTrack()
    {
        // The seam. PlayerCarSpawner's BindTrack is what makes the player's lap times real;
        // this is the same call, for the rest of the field.
        _grid.SpawnGrid();

        var trackers = TrackersOfSpawnedCars();
        Assert.IsNotEmpty(trackers, "The grid spawned no cars, so this proves nothing.");

        foreach (var tracker in trackers)
        {
            Assert.AreSame(_placement, tracker.Track,
                "An AI car's lap tracker is not bound to the track this grid belongs to. " +
                "Unbound it measures nothing: lap distance stays 0, no lap ever completes, " +
                "and the standings rank the whole field level — with nothing in the console " +
                "to say why.");
        }
    }

    [Test]
    public void GridSpawn_GivesEveryAiTrackerTheTrackSurfacesRealLayer()
    {
        // Asserted against the collider, not against 3. This is the assertion that fails if
        // the number is hard-coded again, because the fixture's surface is not on 3.
        _grid.SpawnGrid();

        var trackers = TrackersOfSpawnedCars();
        Assert.IsNotEmpty(trackers, "The grid spawned no cars, so this proves nothing.");

        foreach (var tracker in trackers)
        {
            Assert.AreEqual(_surface.layer, tracker.TrackLayer,
                $"The AI tracker's TrackLayer is {tracker.TrackLayer} but the track surface " +
                $"is on layer {_surface.layer}. IsOnTrackSurface raycasts 1 << TrackLayer, so " +
                "a wrong layer misses the road entirely and the car banks no lap distance for " +
                "the rest of the race — silently. Read the layer off the surface; do not " +
                "hard-code it.");
        }
    }

    [Test]
    public void GridSpawn_AnAiCarBanksLapDistanceOnceItMovesAlongTheLine()
    {
        // The end-to-end version, and the one that matches what the defect actually did to
        // the game: not a wrong number on a HUD, but a number that never moves at all.
        // Every intermediate assertion can pass while this one does not.
        _grid.SpawnGrid();

        var car = _grid.SpawnedCars.First();
        var tracker = car.GetComponent<LapTracker>();

        DriveAlongTheLine(car, tracker, 0.1f);

        Assert.Greater(tracker.TotalDistance, 20f,
            "An AI car driven forward around its own circuit banked no distance (" +
            tracker.TotalDistance.ToString("0.0") + " m). This is the whole failure: the " +
            "grounded check never passes, so nothing about the car's progress is ever real.");
    }

    // --- The guard: doing nothing must be safe ---

    [Test]
    public void GridSpawn_WithNoTrackPresent_DoesNotThrowAndLeavesTrackersUnbound()
    {
        // An early spawn, an editor-time grid, a test — there may be no track content yet.
        // Binding null here would be worse than doing nothing: BindTrack also resets the
        // tracker, so it would discard state and leave the tracker believing it is set up.
        HideTrackContent();

        Assert.DoesNotThrow(() => _grid.SpawnGrid(),
            "Spawning a grid with no track content must not throw. A grid can legitimately be " +
            "asked to place cars before the track has finished loading.");

        Assert.IsNull(Object.FindAnyObjectByType<TrackPlacement>(),
            "The fixture promised a scene with no findable track; something else is in here.");

        var trackers = TrackersOfSpawnedCars();
        Assert.IsNotEmpty(trackers, "The grid spawned no cars, so this proves nothing.");

        foreach (var tracker in trackers)
        {
            Assert.IsNull(tracker.Track,
                "A tracker with no track must be left unbound, not bound to null.");

            // The difference between "unbound" and "bound to nothing" is behaviour, not just
            // the field. An unbound tracker reports that it has no track; a tracker pointed
            // at nothing would report a confident zero and look configured.
            Assert.IsTrue(tracker.LapProgressChanged,
                "A tracker with no track must report its progress as changed, which is how it " +
                "says 'I have no track yet' rather than 'I am at zero on one'.");
        }
    }

    [Test]
    public void GridSpawn_WithNoTrackPresent_LeavesATrackerThatAlreadyResolvedOneAlone()
    {
        // The state-corruption half of the guard above, and the reason the guard is not
        // "just skip the car". The template's tracker is bound before the grid runs, and
        // Instantiate copies that serialized reference onto every car it spawns. Binding
        // null at spawn would erase it — turning a tracker that knows its track into one
        // that has never heard of it.
        var templateTracker = _aiCarTemplate.GetComponent<LapTracker>();
        templateTracker.BindTrack(_placement);
        Assert.AreSame(_placement, templateTracker.Track, "The fixture failed to pre-bind.");

        HideTrackContent();
        _grid.SpawnGrid();

        foreach (var tracker in TrackersOfSpawnedCars())
        {
            Assert.AreSame(_placement, tracker.Track,
                "Spawning with no track placement erased a binding the tracker had already " +
                "resolved. Doing nothing is the safe answer; binding null is not.");
        }
    }

    // --- Fixture ---

    /// <summary>
    /// Takes the track out of the scene without destroying it.
    ///
    /// Deactivating rather than destroying matters for the second guard: a destroyed object
    /// still compares equal to the reference a tracker is holding, which would make the
    /// assertion pass or fail for a reason that has nothing to do with the code. A
    /// deactivated one is a real, live TrackPlacement that a scene search will not return —
    /// which is exactly the "grid spawned before the track content was ready" case.
    /// </summary>
    private void HideTrackContent()
    {
        _placementGo.SetActive(false);
    }

    /// <summary>The road. One collider, on a layer nothing else in the project uses.</summary>
    private void BuildSurface()
    {
        _surface = GameObject.CreatePrimitive(PrimitiveType.Cube);
        _surface.name = "Road";
        _surface.transform.SetParent(_root.transform, false);
        _surface.transform.position = new Vector3(0f, -0.5f, 0f);
        _surface.transform.localScale = new Vector3(400f, 1f, 400f);
        _surface.layer = UnusualSurfaceLayer;
    }

    private void BuildRacingLine()
    {
        var lineGo = new GameObject("RacingLine");
        lineGo.transform.SetParent(_root.transform, false);
        _line = lineGo.AddComponent<AIRacingLine>();
        _line.loop = true;

        for (int i = 0; i < WaypointCount; i++)
        {
            float angle = i / (float)WaypointCount * Mathf.PI * 2f;
            var waypoint = new GameObject($"WP_{i:00}");
            waypoint.transform.SetParent(lineGo.transform, false);
            waypoint.transform.position = new Vector3(
                Mathf.Cos(angle) * CircuitRadius, 0.5f, Mathf.Sin(angle) * CircuitRadius);
            waypoint.AddComponent<AIRacingWaypoint>();
        }

        // Awake and OnValidate do not run for a plain MonoBehaviour built from a test, so
        // the line's waypoint list is populated explicitly rather than by the editor.
        _line.RefreshWaypoints();
        Assert.AreEqual(WaypointCount, _line.Count, "The fixture's racing line is empty.");
    }

    private void BuildTrackPlacement()
    {
        var startPose = new GameObject("StartPose");
        startPose.transform.SetParent(_root.transform, false);
        startPose.transform.position = new Vector3(CircuitRadius, 0.5f, 0f);
        startPose.transform.rotation = Quaternion.LookRotation(Vector3.forward, Vector3.up);

        var gridAnchor = new GameObject("GridAnchor");
        gridAnchor.transform.SetParent(_root.transform, false);
        gridAnchor.transform.position = startPose.transform.position;
        gridAnchor.transform.rotation = startPose.transform.rotation;

        // A sibling of the line rather than its parent: hiding the track content must not
        // take the racing line or the road with it, only stop a scene search finding it.
        _placementGo = new GameObject("TrackPlacement");
        _placementGo.transform.SetParent(_root.transform, false);
        _placement = _placementGo.AddComponent<TrackPlacement>();

        _placement.Configure(_line, _surface.GetComponent<Collider>(), startPose.transform,
            gridAnchor.transform, 0, 2f * Mathf.PI * CircuitRadius);

        Assert.IsTrue(_placement.IsValid,
            "The fixture's track placement is invalid, so a tracker bound to it would " +
            "correctly measure nothing and every assertion below would be meaningless.");
    }

    /// <summary>
    /// The car the grid clones.
    ///
    /// Deliberately not the real AI prefab: instantiating that pulls in the physics
    /// coordinator, the driver, the perception sensor and four raycast wheels, none of which
    /// are what this file is about. <see cref="RaceGridManager.ConfigureCar"/> treats every
    /// one of them as optional, so the seam under test — the tracker — is reachable without
    /// them, and a failure here is unambiguously about the track binding.
    /// </summary>
    private void BuildAITemplate()
    {
        _aiCarTemplate = new GameObject("TestAICar");
        _aiCarTemplate.transform.SetParent(_root.transform, false);

        // Parked well clear of the circuit so it is never under a spawned car.
        _aiCarTemplate.transform.position = new Vector3(0f, 200f, 0f);

        var body = _aiCarTemplate.AddComponent<Rigidbody>();
        body.useGravity = false;

        var collider = _aiCarTemplate.AddComponent<BoxCollider>();
        collider.size = new Vector3(1.8f, 1f, 4f);
        collider.center = new Vector3(0f, -0.3f, 0f);

        _aiCarTemplate.AddComponent<LapTracker>();
    }

    private void BuildGrid()
    {
        var gridGo = new GameObject("RaceGridManager");
        gridGo.transform.SetParent(_root.transform, false);
        _grid = gridGo.AddComponent<RaceGridManager>();

        _grid.spawnOnStart = false;
        _grid.includePlayerInGrid = false;

        // Boards build a world-space canvas and TMP labels, which is a presentation concern
        // this file is not about; DriverIdentifier's Awake does not run from a test anyway.
        _grid.showDriverBoards = false;

        _grid.racingLine = _line;
        _grid.gridAnchor = _placement.GridAnchor;
        _grid.defaultAICarPrefab = _aiCarTemplate;
        _grid.aiEntries = new[]
        {
            new RaceGridManager.RaceGridEntry { driverName = "Test AI", gridPosition = 0 }
        };
    }

    private LapTracker[] TrackersOfSpawnedCars()
    {
        return _grid.SpawnedCars
            .Where(car => car != null)
            .Select(car => car.GetComponent<LapTracker>())
            .Where(tracker => tracker != null)
            .ToArray();
    }

    /// <summary>
    /// Walks a car forward around the circuit, one waypoint per tick, as a real lap is driven.
    ///
    /// The car is teleported rather than simulated, but nothing here depends on physics: the
    /// grounded check is a raycast from the car's own position, and the road is beneath it
    /// the whole way round. Steps are ~21 m, comfortably inside the tracker's 40 m teleport
    /// guard, so these read as genuine movement and not as the respawns it discards.
    /// </summary>
    private void DriveAlongTheLine(GameObject car, LapTracker tracker, float deltaTime)
    {
        int count = _line.Count;

        // The first tick only establishes the starting point.
        tracker.Tick(deltaTime);

        for (int i = 1; i <= count; i++)
        {
            car.transform.position = _line.GetPosition(i % count);
            Physics.SyncTransforms();
            tracker.Tick(deltaTime);
        }
    }
}
