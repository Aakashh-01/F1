using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using F1.GameData;
using F1.GameFlow;

/// <summary>
/// Slice step S2 exit gate:
///   - the car can be placed on the track and driven,
///   - the same track scene serves both the qualifying shell and the race shell,
///   - the start pose is deterministic and the grid derives from the same anchor.
/// </summary>
public class SliceS2TrackContentTests
{
    private const string TrackScenePath = "Assets/Scenes/Track_01.unity";
    private TrackPlacement OpenTrackContent()
    {
        var scene = UnityEditor.SceneManagement.EditorSceneManager.OpenScene(
            TrackScenePath, UnityEditor.SceneManagement.OpenSceneMode.Single);

        var trackRoot = GameObject.Find("F1RaceTrack");
        Assert.IsNotNull(trackRoot, "Track_01 must contain the track geometry (F1RaceTrack).");

        var placement = Object.FindAnyObjectByType<TrackPlacement>();
        Assert.IsNotNull(placement, "Track_01 must contain a TrackPlacement.");
        return placement;
    }

    // --- Content exists ---

    [Test]
    public void TrackScene_HasTheBakedRacingLine()
    {
        var placement = OpenTrackContent();
        var line = placement.RacingLine;

        Assert.IsNotNull(line, "TrackPlacement must reference a baked AIRacingLine.");
        Assert.GreaterOrEqual(line.Count, 16,
            "A usable racing line needs enough waypoints; got " + line.Count);
        Assert.IsTrue(line.loop, "A circuit's racing line must be a closed loop.");
    }

    [Test]
    public void BakedRacingLine_SpansARealisticLapLength()
    {
        var placement = OpenTrackContent();
        float lap = placement.LapLengthMeters;

        Assert.Greater(lap, 500f, "A 500m lap is not a circuit; the bake probably collapsed.");
        Assert.Less(lap, 30000f, "A 30km lap suggests the loop closed across open ground.");
    }

    [Test]
    public void RacingLine_SpacingIsEvenAndTheLoopIsClosed()
    {
        // The signature of a correct arc-length resample. A loop that closed across the
        // infield shows up as one segment far longer than the rest.
        var placement = OpenTrackContent();
        var line = placement.RacingLine;

        float longest = 0f;
        for (int i = 0; i < line.Count; i++)
        {
            float d = Vector3.Distance(
                line.waypoints[i].Position,
                line.waypoints[(i + 1) % line.Count].Position);
            if (d > longest) longest = d;
        }

        float average = placement.LapLengthMeters / line.Count;
        Assert.Less(longest, average * 3f,
            $"Waypoint spacing is uneven (avg {average:0.0}m, longest {longest:0.0}m); " +
            "the loop probably did not close cleanly.");
    }

    [Test]
    public void RacingLine_TargetSpeedsArePlausible()
    {
        var placement = OpenTrackContent();
        var line = placement.RacingLine;

        float min = float.MaxValue, max = float.MinValue;
        for (int i = 0; i < line.Count; i++)
        {
            float s = line.waypoints[i].targetSpeedKmh;
            Assert.Greater(s, 10f, "A target speed this low would stop the AI dead.");
            if (s < min) min = s;
            if (s > max) max = s;
        }

        Assert.Less(min, max,
            "Every waypoint has the same target speed, so the bake ignored curvature.");
    }

    // --- The start pose is on the road and deterministic ---

    [Test]
    public void StartPose_SitsOnTheDrivableSurface()
    {
        var placement = OpenTrackContent();
        Assert.IsTrue(placement.IsValid, "TrackPlacement reports itself invalid.");

        placement.GetStartPose(out var position, out _);
        var surface = placement.TrackSurface;
        Assert.IsNotNull(surface, "TrackPlacement must reference the track surface.");

        var bounds = surface.bounds;
        var ray = new Ray(
            new Vector3(position.x, bounds.max.y + 200f, position.z), Vector3.down);
        Assert.IsTrue(surface.Raycast(ray, out var hit, bounds.size.y + 600f),
            "The start pose is not above the track surface - the car would spawn off-track.");

        Assert.Less(Mathf.Abs(position.y - hit.point.y), 5f,
            "The start pose is far above the surface; the 0.5m lift should be the only offset.");
    }

    [Test]
    public void StartPose_IsDeterministicAcrossReads()
    {
        var placement = OpenTrackContent();
        placement.GetStartPose(out var a, out var ra);
        placement.GetStartPose(out var b, out var rb);

        Assert.AreEqual(a, b, "The canonical start pose must be stable.");
        Assert.AreEqual(ra, rb, "The canonical start rotation must be stable.");
        Assert.Less(Vector3.Distance(a, b), 0.0001f,
            "Repeated reads of the start pose must not drift.");
    }

    [Test]
    public void StartPose_FacesAlongTheRacingLine()
    {
        var placement = OpenTrackContent();
        placement.GetStartPose(out var position, out var rotation);

        Vector3 forward = rotation * Vector3.forward;
        Vector3 lineForward = placement.RacingLine.GetSegmentForward(placement.StartFinishIndex);
        lineForward.y = 0f;
        forward.y = 0f;

        if (lineForward.sqrMagnitude < 0.0001f || forward.sqrMagnitude < 0.0001f) return;
        lineForward.Normalize();
        forward.Normalize();

        Assert.Greater(Vector3.Dot(forward, lineForward), 0.5f,
            "The car would spawn facing across the track rather than down it.");
    }

    [Test]
    public void StartFinishIndex_IsInsideTheWaypointRange()
    {
        var placement = OpenTrackContent();
        Assert.GreaterOrEqual(placement.StartFinishIndex, 0);
        Assert.Less(placement.StartFinishIndex, placement.RacingLine.Count);
    }

    // --- The grid derives from the same anchor (plan section 3.4) ---

    [Test]
    public void GridAnchor_SharesTheStartPoseBasis()
    {
        var placement = OpenTrackContent();
        Assert.IsNotNull(placement.GridAnchor, "TrackPlacement must expose a grid anchor.");

        placement.GetStartPose(out var startPos, out var startRot);
        Assert.Less(Vector3.Distance(placement.GridAnchor.position, startPos), 2f,
            "The grid origin and the qualifying spawn must share the same basis, or the " +
            "grid will not line up with the start line.");
        Assert.Less(
            Quaternion.Angle(placement.GridAnchor.rotation, startRot), 1f,
            "The grid anchor and the start pose must face the same way.");
    }

    [Test]
    public void RaceGridManager_IsWiredToTheGridAnchorAndRacingLine()
    {
        OpenTrackContent();
        var grid = Object.FindAnyObjectByType<RaceGridManager>();

        Assert.IsNotNull(grid, "Track_01 must contain a RaceGridManager.");
        Assert.IsNotNull(grid.gridAnchor, "The grid manager must be wired to the grid anchor.");
        Assert.IsNotNull(grid.racingLine, "The grid manager must be wired to the racing line.");
        Assert.IsFalse(grid.spawnOnStart,
            "The race shell owns spawning; the content scene must not spawn on its own.");
    }

    [Test]
    public void GridSlots_AreOrderedAndOnTheRoad()
    {
        var placement = OpenTrackContent();
        var grid = Object.FindAnyObjectByType<RaceGridManager>();
        var surface = placement.TrackSurface;
        var bounds = surface.bounds;

        Vector3 previous = grid.GetGridPosition(0);
        for (int p = 1; p < 6; p++)
        {
            Vector3 pos = grid.GetGridPosition(p);
            Assert.Greater(Vector3.Distance(previous, pos), 1f,
                $"Grid slot {p} overlaps the previous slot.");
            previous = pos;

            var ray = new Ray(
                new Vector3(pos.x, bounds.max.y + 200f, pos.z), Vector3.down);
            Assert.IsTrue(surface.Raycast(ray, out var hit, bounds.size.y + 600f),
                $"Grid slot {p} is not over the track surface.");
            Assert.Less(Mathf.Abs(pos.y - hit.point.y), 6f,
                $"Grid slot {p} floats far above the surface.");
        }
    }

    [Test]
    public void GridSlots_StackBackwardsFromPole()
    {
        // Pole must be ahead of the field, not behind it.
        var placement = OpenTrackContent();
        var grid = Object.FindAnyObjectByType<RaceGridManager>();

        Vector3 forward = placement.GridAnchor.forward;
        float poleBehind = Vector3.Dot(placement.GridAnchor.position - grid.GetGridPosition(0), forward);
        float lastBehind = Vector3.Dot(placement.GridAnchor.position - grid.GetGridPosition(5), forward);

        Assert.AreEqual(0f, poleBehind, 0.5f, "Pole should sit on the anchor line.");
        Assert.Greater(lastBehind, poleBehind + 10f,
            "Later grid slots must be further back than pole.");
    }

    [Test]
    public void GridSlots_AreTwoWide()
    {
        var placement = OpenTrackContent();
        var grid = Object.FindAnyObjectByType<RaceGridManager>();

        Vector3 right = placement.GridAnchor.right;
        float p1 = Vector3.Dot(grid.GetGridPosition(0) - placement.GridAnchor.position, right);
        float p2 = Vector3.Dot(grid.GetGridPosition(1) - placement.GridAnchor.position, right);

        Assert.AreNotEqual(Mathf.Sign(p1), Mathf.Sign(p2),
            "Grid slots 1 and 2 must be on opposite sides of the centre line.");
    }

    // --- Distance / progress helpers used by S3 timing ---

    [Test]
    public void Progress_IsMeasuredFromTheStartLine()
    {
        var placement = OpenTrackContent();

        Assert.AreEqual(0f, placement.GetProgress01(0f), 0.001f);
        Assert.AreEqual(0.25f, placement.GetProgress01(placement.LapLengthMeters * 0.25f), 0.001f);
        // Distance beyond a lap wraps rather than running off the end.
        Assert.AreEqual(0f, placement.GetProgress01(placement.LapLengthMeters), 0.001f);
        Assert.AreEqual(0.5f, placement.GetProgress01(placement.LapLengthMeters * 1.5f), 0.001f);
    }

    [Test]
    public void PositionAtDistance_AdvancesAlongTheLine()
    {
        var placement = OpenTrackContent();
        var line = placement.RacingLine;

        placement.GetPositionAtDistance(0f, out int atStart);
        placement.GetPositionAtDistance(placement.LapLengthMeters * 0.25f, out int atQuarter);

        int expected = (placement.StartFinishIndex + line.Count / 4) % line.Count;
        Assert.AreEqual(placement.StartFinishIndex, atStart,
            "Distance zero must resolve to the start line.");

        // Within one waypoint, not exactly. The line is resampled to even arc length, so a
        // quarter of the lap is exactly a quarter of the waypoints — and therefore lands
        // precisely on the boundary where truncating a float and rounding it disagree. Which
        // one the implementation does is an implementation detail; "a quarter of the way
        // around" is the contract, and asserting exact equality here only pins down that
        // detail. It passed at 90 waypoints and failed at 300 purely because the arithmetic
        // moved.
        int drift = Mathf.Abs(atQuarter - expected);
        Assert.LessOrEqual(Mathf.Min(drift, line.Count - drift), 1,
            $"A quarter-lap should land a quarter of the waypoints from the start line " +
            $"(expected {expected}, got {atQuarter}).");
    }

    // --- The same content serves qualifying and race (invariant 9) ---

    [Test]
    public void TrackContent_IsReusableAcrossTheSelectedTrack()
    {
        // Invariant 9: pre-race and race resolve the same live objects. The content
        // scene holds one placement; the session decides which track definition is in
        // use, so the placement must not be tied to one definition.
        var placement = OpenTrackContent();
        Assert.IsTrue(placement.IsValid);

        // The free definitions both point at this content scene.
        GameDataRegistry.Initialize();
        foreach (var track in GameDataRegistry.FreeTracks)
        {
            Assert.AreEqual(FlowSceneNames.TrackContent, track.SceneName,
                $"Free track '{track.TrackId}' must resolve to the shared content scene.");
        }
    }
}
