using System.Linq;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using F1.GameFlow;

/// <summary>
/// Slice step S5b: the race shell, and the grid position a qualifying result earns.
///
/// S4 left the race pointing at the track content scene, so pressing "Go to Race" asked the
/// routing service to load a scene that was already loaded as content. S5b gives the race its
/// own shell, and this file covers the two things about it that can be wrong without anyone
/// noticing: the scene's wiring, and the rule that turns a qualifying lap into a grid place.
///
/// The rule is asserted against <see cref="GridPositionResolver"/> rather than through the
/// flow manager, because it is a pure function and standing up the singleton to ask about
/// arithmetic would test the plumbing instead of the policy.
/// </summary>
public class SliceS5RaceShellTests
{
    private const string RaceScenePath = "Assets/Scenes/50_RaceScene.unity";
    private const string TrackScenePath = "Assets/Scenes/Track_01.unity";

    // --- The grid position rule ---

    /// <summary>A field of 8 whose benchmarks run from quick to slow, hardest first.</summary>
    private static float[] Ladder() => new[] { 78f, 78f, 84f, 84f, 84f, 91f, 91f, 91f };

    private static int Resolve(float playerLap, float[] field) =>
        GridPositionResolver.Resolve(playerLap, field.Length, i => field[i]);

    [Test]
    public void NoField_LeavesThePlayerOnPole()
    {
        Assert.AreEqual(1, GridPositionResolver.Resolve(80f, 0, null),
            "With no cars there is no grid order to place the player in.");
    }

    [Test]
    public void NoQualifyingLap_LeavesThePlayerOnPole()
    {
        // The rule is fed from the session, so a zero or negative lap is reachable. It must
        // not be read as "infinitely fast" and hand the player pole by accident — or as
        // "infinitely slow" and bury them at the back.
        Assert.AreEqual(1, Resolve(0f, Ladder()));
        Assert.AreEqual(1, Resolve(-5f, Ladder()));
    }

    [Test]
    public void AQuickQualifyingLapStartsThePlayerAtTheFront()
    {
        // Quicker than every car in the field: pole, with the two 78s behind.
        Assert.AreEqual(1, Resolve(70f, Ladder()));
    }

    [Test]
    public void ASlowQualifyingLapStartsThePlayerAtTheBack()
    {
        // Slower than every car: last, and no further — the player is not one of the AI.
        Assert.AreEqual(9, Resolve(120f, Ladder()));
    }

    [Test]
    public void AMidFieldLapLandsBetweenTheCarsAroundIt()
    {
        // 82s is quicker than the 84s and the 91s, slower than the two 78s, so the player is
        // third: two cars ahead, six behind.
        Assert.AreEqual(3, Resolve(82f, Ladder()));
    }

    [Test]
    public void ATieWithACarStartsThePlayerBehindIt()
    {
        // 84s matches three cars exactly, and the two 78s are quicker still, so five cars
        // rank ahead and the player starts 6th. Matching a benchmark is not beating it, and
        // the conservative reading of a tie is the one that does not hand out a place for it.
        Assert.AreEqual(6, Resolve(84f, Ladder()));
    }

    [Test]
    public void ThePlayerIsNeverPlacedPastTheLastCar()
    {
        // A field of 1 puts the player 2nd at worst, however slow the lap.
        Assert.AreEqual(2, Resolve(999f, new[] { 80f }));
        Assert.AreEqual(2, GridPositionResolver.Resolve(999f, 1, i => 80f));
    }

    [Test]
    public void TheBenchmarkForAnIndexMatchesTheCarSpawnedAtThatIndex()
    {
        // The rule counts cars rather than reading them in order, so the order of the ladder
        // cannot change the answer — an earlier version of this test asserted that it did,
        // which was a property the function never had. What does matter is that the two views
        // of the field agree on which car is which.
        //
        // The resolver is fed <c>GetBenchmarkLapSeconds(i)</c> and the grid spawns
        // <c>BuildGridEntries()[i]</c>. If the roster won by index in one and not the other,
        // the player would be ranked against a different car than the one standing there —
        // a grid where P3's board and the benchmark used to place the player disagree, and
        // nothing in either view would look wrong.
        var field = ScriptableObject.CreateInstance<AIFieldDefinition>();
        try
        {
            field.fieldSize = 6;
            field.hardReferenceLapSeconds = 78f;
            field.mediumReferenceLapSeconds = 84f;
            field.easyReferenceLapSeconds = 91f;
            field.roster = new[]
            {
                new AIFieldDefinition.FieldEntry
                {
                    driverName = "M. Allegretti",
                    difficulty = AIDifficultyPreset.Easy,
                    benchmarkLapOverride = 75f
                }
            };

            var entries = field.BuildGridEntries();

            for (int i = 0; i < entries.Length; i++)
            {
                bool rosterOverrides = field.roster != null && i < field.roster.Length &&
                                       field.roster[i].benchmarkLapOverride > 0f;

                float expected = rosterOverrides
                    ? field.roster[i].benchmarkLapOverride
                    : field.GetReferenceLapSeconds(entries[i].fallbackDifficulty);

                Assert.AreEqual(expected, field.GetBenchmarkLapSeconds(i), 0.001f,
                    "Benchmark and spawned car disagree at index " + i + " (" +
                    entries[i].driverName + ").");
            }
        }
        finally
        {
            Object.DestroyImmediate(field);
        }
    }

    [Test]
    public void TheFieldDefinitionSuppliesABenchmarkForEveryCarItSpawns()
    {
        // The resolver indexes the field the same way the grid spawns it, so a field that
        // answers a different number of questions than it produces cars would rank the player
        // against cars that are not on the grid.
        var field = ScriptableObject.CreateInstance<AIFieldDefinition>();
        try
        {
            field.fieldSize = 8;
            for (int i = 0; i < field.Count; i++)
                Assert.Greater(field.GetBenchmarkLapSeconds(i), 0f,
                    "Car " + i + " has no benchmark, so nothing can be ranked against it.");
        }
        finally
        {
            Object.DestroyImmediate(field);
        }
    }

    [Test]
    public void AHarderCarIsBenchmarkedFasterThanAnEasierOne()
    {
        // The benchmark is derived from difficulty, so if the reference times were set
        // backwards the field would grid itself in the wrong order — hardest cars at the back.
        var field = ScriptableObject.CreateInstance<AIFieldDefinition>();
        try
        {
            field.hardReferenceLapSeconds = 78f;
            field.mediumReferenceLapSeconds = 84f;
            field.easyReferenceLapSeconds = 91f;

            Assert.Less(field.GetReferenceLapSeconds(AIDifficultyPreset.Hard),
                field.GetReferenceLapSeconds(AIDifficultyPreset.Medium));
            Assert.Less(field.GetReferenceLapSeconds(AIDifficultyPreset.Medium),
                field.GetReferenceLapSeconds(AIDifficultyPreset.Easy));
        }
        finally
        {
            Object.DestroyImmediate(field);
        }
    }

    [Test]
    public void AnAuthoredRosterEntryCanOverrideItsBenchmark()
    {
        // A named driver is a specific car, not a difficulty band, so the roster gets to say
        // what it laps.
        var field = ScriptableObject.CreateInstance<AIFieldDefinition>();
        try
        {
            field.fieldSize = 2;
            field.roster = new[]
            {
                new AIFieldDefinition.FieldEntry
                {
                    driverName = "M. Allegretti",
                    difficulty = AIDifficultyPreset.Easy,
                    benchmarkLapOverride = 75f
                }
            };

            Assert.AreEqual(75f, field.GetBenchmarkLapSeconds(0),
                "The roster's override should beat the Easy reference time.");
        }
        finally
        {
            Object.DestroyImmediate(field);
        }
    }

    // --- The shell's wiring ---

    private static T Find<T>(Scene scene) where T : Component
    {
        foreach (var root in scene.GetRootGameObjects())
        {
            var found = root.GetComponentInChildren<T>(true);
            if (found != null) return found;
        }
        return null;
    }

    [Test]
    public void RaceSceneExistsAndHostsTheRaceScreen()
    {
        var scene = EditorSceneManager.OpenScene(RaceScenePath, OpenSceneMode.Single);

        var host = Find<FlowSceneHost>(scene);
        Assert.IsNotNull(host, "The race shell must carry a FlowSceneHost, like every shell.");

        var so = new UnityEditor.SerializedObject(host);
        Assert.AreEqual((int)GameFlowManager.GameScreen.Race,
            so.FindProperty("_hostedScreen").enumValueIndex,
            "The race shell hosts Race. If it does not, pressing Go to Race resolves no screen.");
    }

    [Test]
    public void RaceSceneActuallyHasAScreenPrefabToInstantiate()
    {
        // The test above passed for the whole time the race had no HUD at all, because it
        // only checked which screen the host *declares*. Declaring Race and having nothing to
        // instantiate is a legal state — FlowSceneHost says so, and every other assertion in
        // the project was written against that. The result was a race that ran with no
        // position, no lap counter and no clock on screen, and no test failed.
        //
        // So the declaration is asserted separately from the thing that makes it real. The
        // prefab is what turns "hosts Race" into a HUD.
        var scene = EditorSceneManager.OpenScene(RaceScenePath, OpenSceneMode.Single);

        var host = Find<FlowSceneHost>(scene);
        Assert.IsNotNull(host, "The race shell must carry a FlowSceneHost, like every shell.");

        var so = new UnityEditor.SerializedObject(host);
        var prefab = so.FindProperty("_screenPrefab").objectReferenceValue as GameObject;

        Assert.IsNotNull(prefab,
            "The race host declares the Race screen but has no prefab assigned, so " +
            "FlowSceneHost.EnsureScreenInstance returns early and the race runs with no " +
            "HUD. Build it with Tools > Race > Build And Install Race Screen.");

        var screen = prefab.GetComponent<ScreenController>();
        Assert.IsNotNull(screen,
            "The race screen prefab carries no ScreenController, so hosting it would fail " +
            "at runtime rather than at build time.");

        // The concrete implementation, not the abstract placeholder — the same distinction
        // SliceS4QualifyingShellTests makes for the qualifying shell. Checked by name
        // because Assets/Scripts/UI has no assembly definition and therefore compiles into
        // the predefined Assembly-CSharp, which an asmdef-based test assembly cannot
        // reference; the abstract contract in F1.GameFlow is reachable, which is enough.
        Assert.AreEqual("RaceScreenImpl", screen.GetType().Name,
            "The hosted race screen should be the concrete UI implementation.");
    }

    [Test]
    public void RaceScreenPrefabWiresEveryReadoutItAdvertises()
    {
        // A prefab that exists but leaves its text fields null renders an empty HUD, which
        // is the same failure as no HUD wearing a different hat. Every one of these is what
        // the interface contract offers a caller, so every one of them is asserted assigned.
        var prefab = UnityEditor.AssetDatabase.LoadAssetAtPath<GameObject>(
            "Assets/Prefabs/RaceScreen_Prefab.prefab");
        Assert.IsNotNull(prefab,
            "RaceScreen_Prefab is missing. Tools > Race > Build And Install Race Screen.");

        var screen = prefab.GetComponent<ScreenController>();
        Assert.IsNotNull(screen);

        var so = new UnityEditor.SerializedObject(screen);
        foreach (string field in new[]
                 {
                     "_trackNameText", "_totalLapsText", "_positionText",
                     "_currentLapText", "_lapTimeText", "_bestLapText",
                     "_sector1Text", "_sector2Text", "_sector3Text",
                     "_finishPanel", "_finishPositionText", "_pointsEarnedText",
                     "_pauseButton",
                 })
        {
            var prop = so.FindProperty(field);
            Assert.IsNotNull(prop, $"RaceScreenImpl has no serialized field '{field}'.");
            Assert.IsNotNull(prop.objectReferenceValue,
                $"RaceScreenImpl.{field} is unassigned, so that part of the HUD never draws.");
        }
    }

    [Test]
    public void RaceSceneCarriesTheControllerAndASpawner()
    {
        var scene = EditorSceneManager.OpenScene(RaceScenePath, OpenSceneMode.Single);

        Assert.IsNotNull(Find<RaceSceneController>(scene),
            "The shell without its controller is an empty scene.");
        Assert.IsNotNull(Find<PlayerCarSpawner>(scene),
            "The shell must be able to spawn the player's car onto the grid.");
    }

    [Test]
    public void RaceSceneSpawnerIsWiredToThePlayerCar()
    {
        // The same prefab the pre-race shell uses, so there is one definition of the player's
        // car rather than two that can drift.
        var scene = EditorSceneManager.OpenScene(RaceScenePath, OpenSceneMode.Single);
        var spawner = Find<PlayerCarSpawner>(scene);
        Assert.IsNotNull(spawner);

        var so = new UnityEditor.SerializedObject(spawner);
        var prefab = so.FindProperty("_playerCarPrefab").objectReferenceValue as GameObject;
        Assert.IsNotNull(prefab, "The race spawner has no car prefab, so no car can race.");
        Assert.AreEqual("F1_Body", prefab.name);
    }

    [Test]
    public void TheRaceIsNoLongerRoutedToTheTrackContentScene()
    {
        // This is the specific defect S5b exists to fix. A flow manager created in code —
        // which is how every test and the boot route make one — takes its defaults from the
        // field initialisers, so a default still pointing at the track scene would route the
        // race into a scene that is already loaded as content.
        var flow = new GameObject("FlowDefaultProbe").AddComponent<GameFlowManager>();
        try
        {
            var so = new UnityEditor.SerializedObject(flow);
            var raceScene = so.FindProperty("_raceScene").stringValue;
            var qualifyingScene = so.FindProperty("_qualifyingScene").stringValue;

            Assert.AreEqual(FlowSceneNames.Race, raceScene,
                "The race must route to its own shell, not to the shared track content.");
            Assert.AreEqual(FlowSceneNames.PreRace, qualifyingScene,
                "Qualifying must keep routing to its own shell.");
        }
        finally
        {
            Object.DestroyImmediate(flow.gameObject);
        }
    }

    [Test]
    public void TrackContentCarriesNoScreenHostAndNoRaceSession()
    {
        // The content scene is loaded additively under whichever session is running. Anything
        // on it that claims a screen or owns the race session competes with the shell, and
        // which one wins depends on load order.
        var scene = EditorSceneManager.OpenScene(TrackScenePath, OpenSceneMode.Single);

        Assert.IsNull(Find<FlowSceneHost>(scene),
            "A host on the content scene competes with the shell for screen resolution.");
        Assert.IsNull(Find<TrackModeManager>(scene),
            "The race session belongs to the race shell. TrackModeManager spawned AI at the " +
            "world origin and had no grid.");
    }

    [Test]
    public void TrackContentStillCarriesTheGridTheRaceDrives()
    {
        // The counterweight to the test above: stripping the track scene back to content must
        // not take the grid with it. The grid anchor is geometry, and the race shell drives
        // the manager that owns it rather than duplicating it (invariant 9/18).
        var scene = EditorSceneManager.OpenScene(TrackScenePath, OpenSceneMode.Single);

        var grid = Find<RaceGridManager>(scene);
        Assert.IsNotNull(grid, "The grid manager is the track's, and the race shell drives it.");

        var so = new UnityEditor.SerializedObject(grid);
        Assert.IsNotNull(so.FindProperty("gridAnchor").objectReferenceValue,
            "Without the grid anchor the grid has no origin.");
        Assert.IsNotNull(so.FindProperty("defaultAICarPrefab").objectReferenceValue,
            "Without an AI prefab every field entry fails to spawn.");
        Assert.IsNotNull(so.FindProperty("aiField").objectReferenceValue,
            "Without a field the grid is empty.");
        Assert.IsFalse(so.FindProperty("spawnOnStart").boolValue,
            "The content scene must not spawn a grid by itself; the race shell asks.");
    }
}
