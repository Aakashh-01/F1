using System.Linq;
using NUnit.Framework;
using UnityEngine;
using F1.GameFlow;

/// <summary>
/// Slice step S5a: the grid before anything is built on top of it.
///
/// S4 handed the player a car on the canonical start pose. S5 has to put that car on a
/// *grid* with a field around it, and the grid machinery it will use is the pre-existing
/// <c>RaceGridManager</c> plus the <c>AIFieldDefinition</c> added to it. Neither had ever
/// been driven, and both fail quietly:
/// a field definition that produces duplicate car numbers, or a grid manager with no AI
/// prefab assigned, both render a race that looks like it is working.
///
/// So this file asserts the two halves separately — the field's generation rules, which are
/// pure logic, and the grid's geometry and wiring, which are a scene fact. The spawn path
/// itself is deliberately not exercised here: instantiating cars runs their whole physics
/// Awake, which is a play-mode concern. S5's PlayMode tests cover the spawn, and the exit
/// gate is "AI present" rather than "AI configured".
/// </summary>
public class SliceS5AIGridTests
{
    private const string TrackScenePath = "Assets/Scenes/Track_01.unity";

    private static AIFieldDefinition MakeField(int fieldSize)
    {
        var field = ScriptableObject.CreateInstance<AIFieldDefinition>();
        field.fieldSize = fieldSize;
        return field;
    }

    // --- The field definition: how many cars, and who they are ---

    [Test]
    public void Field_ProducesExactlyTheFieldSizeInCars()
    {
        var field = MakeField(8);

        var entries = field.BuildGridEntries();

        Assert.AreEqual(8, entries.Length,
            "fieldSize is the whole point of the field: the project owns one car prefab, so " +
            "the field is a number rather than a list of prefab references.");
    }

    [Test]
    public void Field_OfZeroIsAnEmptyFieldRatherThanAnError()
    {
        // A solo run is a legitimate configuration, not a mistake. It must produce no cars
        // and not throw, because the racing shell is allowed to run a player-only race.
        var field = MakeField(0);

        var entries = field.BuildGridEntries();

        Assert.IsNotNull(entries);
        Assert.AreEqual(0, entries.Length);
    }

    [Test]
    public void GeneratedCars_GetUniqueNonZeroNumbers()
    {
        // The number is the car's identity: it goes on the board and in the standings. Two
        // cars sharing one makes both displays lie, and nothing else in the field would
        // reveal it.
        var field = MakeField(12);

        var entries = field.BuildGridEntries();
        var numbers = entries.Select(e => e.driverNumber).ToList();

        Assert.IsTrue(numbers.All(n => n > 0),
            "A car with no number shows a blank board: " +
            string.Join(", ", numbers.Where(n => n <= 0)));
        Assert.AreEqual(numbers.Count, numbers.Distinct().Count(),
            "Two cars in one field share a number: " + string.Join(", ", numbers));
    }

    [Test]
    public void NumberPoolSmallerThanTheField_StillProducesUniqueNumbers()
    {
        // The pools are sized for a realistic grid, not enforced to be. Running out of pool
        // must degrade to "the first unused number" rather than to duplicates — otherwise
        // growing the field past the pool silently corrupts the boards.
        var field = MakeField(8);
        field.numberPool = new[] { 4, 7, 11 };

        var entries = field.BuildGridEntries();
        var numbers = entries.Select(e => e.driverNumber).ToList();

        Assert.AreEqual(8, numbers.Distinct().Count(),
            "Numbers repeated once the pool ran out: " + string.Join(", ", numbers));
    }

    [Test]
    public void AuthoredRoster_WinsByIndex_AndTheRestAreGenerated()
    {
        // The roster is the optional half: named drivers keep their name and number, and
        // the cars behind them are filled in automatically. Getting the index wrong would
        // hand a named driver's number to a generated car.
        var field = MakeField(4);
        field.roster = new[]
        {
            new AIFieldDefinition.FieldEntry { driverName = "M. Allegretti", number = 1 },
            new AIFieldDefinition.FieldEntry { driverName = "K. Sørensen", number = 2 }
        };

        var entries = field.BuildGridEntries();

        Assert.AreEqual(4, entries.Length);
        Assert.AreEqual("M. Allegretti", entries[0].driverName);
        Assert.AreEqual(1, entries[0].driverNumber);
        Assert.AreEqual("K. Sørensen", entries[1].driverName);
        Assert.AreEqual(2, entries[1].driverNumber);

        // Past the roster the pools take over — and must not reissue 1 or 2.
        Assert.AreNotEqual(1, entries[2].driverNumber);
        Assert.AreNotEqual(2, entries[2].driverNumber);
        Assert.IsNotEmpty(entries[3].driverName);
    }

    [Test]
    public void GeneratedField_MixesDifficulty()
    {
        // A field where every car pushes identically is a procession, and it makes the race
        // unwatchable. The ladder deliberately mixes them, so assert it still does.
        var field = MakeField(8);

        var presets = field.BuildGridEntries()
            .Select(e => e.fallbackDifficulty)
            .Distinct()
            .ToList();

        Assert.Greater(presets.Count, 1,
            "Every AI car has the same difficulty, so the field is a procession.");
    }

    [Test]
    public void GeneratedCars_PreferOppositeSidesOfTheTrack()
    {
        // Spread across the road, not nose to tail on one line. The overtake side and the
        // lane offset both alternate by index, so a field of any size fills both sides.
        var field = MakeField(8);

        var lanes = field.BuildGridEntries()
            .Select(e => e.preferredLaneOffset01)
            .ToList();

        Assert.IsTrue(lanes.Any(l => l > 0f) && lanes.Any(l => l < 0f),
            "The whole field is queued on one side of the track: " + string.Join(", ", lanes));
    }

    [Test]
    public void Entries_LeaveTheCarPrefabToTheGridManager()
    {
        // The seam. The field knows names, numbers and difficulty; the grid manager knows
        // prefabs and geometry. If an entry ever starts naming a prefab, the project is back
        // to needing a hand-built list per field size.
        var field = MakeField(4);

        foreach (var entry in field.BuildGridEntries())
        {
            Assert.IsNull(entry.carPrefabOverride,
                "A field entry must not name a car prefab — every car is the one prefab.");
            Assert.IsNull(entry.existingCar,
                "A field entry must not reference a pre-placed car; the grid spawns the field.");
        }
    }

    // --- The board: what a car says about itself ---

    /// <summary>
    /// The board's label rule, tested without building a board.
    ///
    /// <c>DriverIdentifier.Awake</c> only runs in play mode, so the canvas and its labels
    /// cannot exist in an EditMode test. What can be tested — and what would actually break —
    /// is which of the two identities becomes the headline, because that is a decision made in
    /// one expression and it is the difference between a grid that says "P5" and one that
    /// says "7".
    /// </summary>
    private static DriverIdentifier NewIdentifier()
    {
        var go = new GameObject("BoardTestCar");
        return go.AddComponent<DriverIdentifier>();
    }

    [Test]
    public void Board_ShowsTheGridSlotAsItsHeadline()
    {
        var identifier = NewIdentifier();
        try
        {
            identifier.SetIdentity("K. Oduya", 7, 5);

            // Slot 5 is the sixth place on the grid, and a grid labels places from one.
            Assert.AreEqual("P6", identifier.BoardHeadline,
                "The grid slot is the more useful headline on a grid: it says where the car " +
                "is standing, not merely which car it is.");
            Assert.AreEqual(5, identifier.GridSlot);
            Assert.AreEqual(7, identifier.Number,
                "The race number must survive, because the standings list identifies cars by it.");
        }
        finally
        {
            Object.DestroyImmediate(identifier.gameObject);
        }
    }

    [Test]
    public void Board_PoleSlotReadsAsP1()
    {
        // Found by looking at the grid, not by reading the code: the first field spawned
        // labelled its pole car "2" while the row behind it read "P1" and "P2". Zero was being
        // used as the "never on a grid" sentinel, and zero is a real slot — pole.
        var identifier = NewIdentifier();
        try
        {
            identifier.SetIdentity("V. Verga", 2, 0);

            Assert.AreEqual("P1", identifier.BoardHeadline,
                "Slot zero is pole, not 'unset'. Getting this wrong mislabels exactly the car " +
                "at the front of the grid.");
            Assert.AreEqual(0, identifier.GridSlot);
        }
        finally
        {
            Object.DestroyImmediate(identifier.gameObject);
        }
    }

    [Test]
    public void Board_FallsBackToTheNumberWhenTheCarWasNeverOnAGrid()
    {
        // A car spawned outside a race has a number and a name but no slot. Showing an empty
        // headline in that case would leave a nameless-looking board.
        var identifier = NewIdentifier();
        try
        {
            identifier.SetIdentity("K. Oduya", 7);

            Assert.AreEqual("7", identifier.BoardHeadline);
            Assert.AreEqual(DriverIdentifier.NoGridSlot, identifier.GridSlot);
        }
        finally
        {
            Object.DestroyImmediate(identifier.gameObject);
        }
    }

    [Test]
    public void Board_WithNeitherIdentityHasNoHeadline()
    {
        var identifier = NewIdentifier();
        try
        {
            identifier.SetIdentity("K. Oduya", 0, DriverIdentifier.NoGridSlot);

            Assert.AreEqual(string.Empty, identifier.BoardHeadline,
                "With no slot and no number the board shows the name alone, not 'P0'.");
        }
        finally
        {
            Object.DestroyImmediate(identifier.gameObject);
        }
    }

    [Test]
    public void Board_IgnoresABlankNameAndANonsensicalSlot()
    {
        // Two small guards, because both are reachable from a hand-authored grid entry: an
        // entry with an empty name would otherwise wipe the driver's name off the board, and
        // a negative slot would render as "P-1".
        var identifier = NewIdentifier();
        try
        {
            identifier.SetIdentity("K. Oduya", 7, 5);
            identifier.SetIdentity("   ", 7, -3);

            Assert.AreEqual("K. Oduya", identifier.DriverName,
                "A blank name must not erase the driver's name.");
            Assert.AreEqual(DriverIdentifier.NoGridSlot, identifier.GridSlot,
                "A negative slot is not a slot.");
            Assert.AreEqual("7", identifier.BoardHeadline);
        }
        finally
        {
            Object.DestroyImmediate(identifier.gameObject);
        }
    }

    // --- The grid: geometry, and the wiring that makes it reachable ---

    private static RaceGridManager OpenTrackGrid()
    {
        UnityEditor.SceneManagement.EditorSceneManager.OpenScene(
            TrackScenePath, UnityEditor.SceneManagement.OpenSceneMode.Single);

        var manager = Object.FindAnyObjectByType<RaceGridManager>();
        Assert.IsNotNull(manager, "Track_01 must contain the RaceGridManager that owns the grid.");
        return manager;
    }

    [Test]
    public void TrackGrid_IsWiredToAFieldAndToTheOneCarPrefab()
    {
        // This is the whole of S5a in one assertion. An unwired grid manager is inert and
        // looks exactly like a wired one from the outside: the race would simply have no AI
        // in it, with nothing in the console to say why.
        var manager = OpenTrackGrid();

        Assert.IsNotNull(manager.aiField,
            "The track's grid manager has no AI field, so it spawns an empty grid.");
        Assert.IsNotNull(manager.defaultAICarPrefab,
            "The track's grid manager has no AI car prefab, so every entry fails to spawn.");
        Assert.IsTrue(manager.showDriverBoards,
            "Every car on the grid is the same prefab; without boards they are indistinguishable.");
    }

    [Test]
    public void TrackGrid_DoesNotSpawnOnItsOwn()
    {
        // The track scene is shared *content*, loaded by the qualifying shell as well as the
        // race. If the grid spawned itself, AI would appear during qualifying, which is
        // player-only (invariant 6). The race shell drives SpawnGrid instead.
        var manager = OpenTrackGrid();

        Assert.IsFalse(manager.spawnOnStart,
            "The shared track content must not spawn a grid by itself; the race shell asks.");
    }

    [Test]
    public void GridSlots_AreAllDistinct()
    {
        // Cars sharing a slot spawn inside each other and explode apart on the first frame.
        var manager = OpenTrackGrid();
        int count = manager.aiField.Count;

        var positions = Enumerable.Range(0, count)
            .Select(manager.GetGridPosition)
            .ToList();

        for (int i = 0; i < positions.Count; i++)
        {
            for (int j = i + 1; j < positions.Count; j++)
            {
                Assert.Greater(Vector3.Distance(positions[i], positions[j]), 1f,
                    $"Grid slots {i} and {j} resolve to the same place.");
            }
        }
    }

    [Test]
    public void GridSlots_StackBackwardsFromThePole_AndPoleIsOnTheAnchor()
    {
        // The grid derives from the track's own anchor (invariant 9/10), so pole is at the
        // anchor and every later row is behind it. A grid that stacks forwards would put the
        // back of the field on the start line.
        var manager = OpenTrackGrid();

        Assert.IsNotNull(manager.gridAnchor,
            "The track content owns the grid anchor; without one the grid has no origin.");

        var anchor = manager.gridAnchor;
        var forward = Vector3.ProjectOnPlane(anchor.forward, Vector3.up).normalized;
        var right = Vector3.Cross(Vector3.up, forward).normalized;

        var pole = manager.GetGridPosition(0);
        var poleOffset = pole - anchor.position;

        // The pole sits on the anchor's line, offset sideways into the first column.
        Assert.Less(Mathf.Abs(Vector3.Dot(poleOffset, forward)), 1f,
            "Pole should sit on the anchor's line, not up the road from it.");
        Assert.Greater(Mathf.Abs(Vector3.Dot(poleOffset, right)), 0.5f,
            "Pole should sit in a grid column beside the anchor, not on top of it.");

        // Two per row, so slot 2 is one full row behind slot 0.
        var third = manager.GetGridPosition(2);
        float back = Vector3.Dot(third - pole, forward);
        Assert.Less(back, -1f, "Slot 2 should be a row behind the pole, not beside it.");
        Assert.AreEqual(manager.gridRowSpacing, -back, 0.5f,
            "Rows should be exactly gridRowSpacing apart.");
    }

    [Test]
    public void PoleAndSecondSlot_AreOnOppositeSides_AndTheSameRoadLevel()
    {
        // Two-wide, alternating sides, same row. This is what makes the grid look like a
        // grid rather than a queue.
        var manager = OpenTrackGrid();
        var anchor = manager.gridAnchor;
        var forward = Vector3.ProjectOnPlane(anchor.forward, Vector3.up).normalized;
        var right = Vector3.Cross(Vector3.up, forward).normalized;

        var pole = manager.GetGridPosition(0) - anchor.position;
        var second = manager.GetGridPosition(1) - anchor.position;

        Assert.Greater(Vector3.Dot(pole, right), 0f, "Pole should be on the right (poleOnRight).");
        Assert.Less(Vector3.Dot(second, right), 0f, "The second slot should be on the left.");
        Assert.AreEqual(pole.y, second.y, 0.1f,
            "Two cars in the same row should sit at the same height.");
    }
}
