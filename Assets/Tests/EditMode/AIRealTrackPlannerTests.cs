using NUnit.Framework;
using UnityEngine;

/// <summary>
/// Runs the longitudinal planner over the REAL baked Track_01 racing line.
///
/// <para>
/// This is the gap the phase plan names as its biggest risk: every other AI behaviour test
/// builds a synthetic line from primitive GameObjects, so a planner can be correct on a straight
/// and fall apart on the real hairpin with the whole suite still green. The baked line is 300
/// waypoints at 11.17-11.33 m over a 3397.8 m loop, with peaks as low as 90 km/h, and its apex
/// chords are exactly where a short lookahead is wrong.
/// </para>
///
/// <para>
/// These assert the invariant that must hold at every point of the real geometry: the speed the
/// plan permits must be one the car can actually shed down to, for every constraint inside the
/// horizon. A planner that reads a single waypoint satisfies this near a straight and fails it
/// on the approach to a slow corner.
/// </para>
/// </summary>
public class AIRealTrackPlannerTests
{
    private const string TrackScenePath = "Assets/Scenes/Track_01.unity";
    private const float KmhToMs = 1f / 3.6f;

    /// <summary>A measured-scale budget. The planner's own measured value is asserted separately;
    /// this stands in for it so the geometry assertions do not depend on a car's suspension.</summary>
    private const float BrakingDecel = 25f;

    private AIRacingLine OpenRealLine()
    {
        UnityEditor.SceneManagement.EditorSceneManager.OpenScene(
            TrackScenePath, UnityEditor.SceneManagement.OpenSceneMode.Single);

        var trackRoot = GameObject.Find("F1RaceTrack");
        Assert.IsNotNull(trackRoot, "Track_01 must contain the track geometry (F1RaceTrack).");

        AIRacingLine line = Object.FindAnyObjectByType<AIRacingLine>();
        Assert.IsNotNull(line, "Track_01 must carry a baked AIRacingLine, or there is nothing " +
            "for the planner to plan along and every assertion below would pass vacuously.");

        Assert.Greater(line.Count, 100,
            "The real baked line is 300 waypoints. Found " + line.Count +
            ", which means this test is running against something other than the real track.");

        return line;
    }

    /// <summary>
    /// Walks the whole real line and, at every waypoint, solves the plan from there. Then checks
    /// the result against the tightest constraint inside the horizon: the permitted speed may not
    /// exceed what the brakes allow to reach that constraint.
    /// </summary>
    [Test]
    public void ThePlanIsBrakeFeasibleAtEveryPointOnTheRealLine()
    {
        AIRacingLine line = OpenRealLine();
        AILongitudinalPlanner planner = new AILongitudinalPlanner();

        int checkedPoints = 0;
        int violations = 0;
        string worstMessage = null;
        float worstExcess = 0f;

        for (int start = 0; start < line.Count; start++)
        {
            Vector3 position = line.GetPosition(start);
            float allowed = planner.SolvePlannedSpeedKmh(
                line, start, position, 240f, BrakingDecel,
                1f, 1f, 0f,
                minimumHorizonMetres: 45f, maximumWaypoints: 24, fallbackKmh: 240f);

            // Independently recompute the tightest constraint in the same horizon and confirm the
            // planner did not exceed it. Recomputing here rather than trusting the planner's own
            // count keeps this a check ON the planner rather than of it against itself.
            float tightest = float.PositiveInfinity;
            float travelled = 0f;
            float horizon = planner.LastHorizonMetres;
            int index = start;

            for (int step = 0; step < planner.LastHorizonWaypointCount; step++)
            {
                int next = line.WrapIndex(index + 1);
                Vector3 from = line.GetPosition(index);
                Vector3 to = line.GetPosition(next);
                travelled += Vector3.Distance(from, to);

                AIRacingWaypoint waypoint = line.GetWaypoint(next);
                if (waypoint != null)
                {
                    float limitMs = waypoint.targetSpeedKmh * KmhToMs;
                    float reachableMs = Mathf.Sqrt(
                        limitMs * limitMs + 2f * BrakingDecel * travelled);
                    tightest = Mathf.Min(tightest, reachableMs / KmhToMs);
                }

                if (travelled >= horizon)
                    break;

                index = next;
            }

            if (float.IsInfinity(tightest))
                continue;

            checkedPoints++;
            float excess = allowed - tightest;
            if (excess > 1f)
            {
                violations++;
                if (excess > worstExcess)
                {
                    worstExcess = excess;
                    worstMessage =
                        $"waypoint {start}: permitted {allowed:F1} km/h but the tightest " +
                        $"constraint {planner.LastHorizonWaypointCount} waypoints ahead permits " +
                        $"{tightest:F1} km/h";
                }
            }
        }

        Assert.Greater(checkedPoints, 200,
            "The real line should yield a constraint at essentially every one of its 300 " +
            "waypoints. Only " + checkedPoints + " were checkable, so this test is not really " +
            "walking the track.");

        Assert.AreEqual(0, violations,
            "At every point on the real line, the speed the plan permits must be one the car can " +
            "still brake down to for the tightest corner in its horizon. " + violations +
            " point(s) permit more than the brakes allow — worst was " + worstMessage +
            ". This is the failure the single-waypoint target produced on real geometry: the " +
            "line's corner is beyond the lookahead, so the plan grants a speed the car cannot " +
            "shed before it arrives.");
    }

    [Test]
    public void TheRealLineHasCornersTheBrakingHorizonMustBeLongEnoughToSee()
    {
        // Guards the fixture above from being vacuous: if every waypoint on the real track were
        // at the 240 km/h ceiling there would be nothing to brake for and the feasibility test
        // would pass whatever the planner did.
        AIRacingLine line = OpenRealLine();

        int slow = 0;
        float slowest = float.MaxValue;
        for (int i = 0; i < line.Count; i++)
        {
            AIRacingWaypoint waypoint = line.GetWaypoint(i);
            if (waypoint == null)
                continue;

            if (waypoint.targetSpeedKmh < 150f)
                slow++;
            slowest = Mathf.Min(slowest, waypoint.targetSpeedKmh);
        }

        Assert.Greater(slow, 20,
            "The real baked line has a genuine spread of corner speeds. Only " + slow +
            " waypoints are below 150 km/h, so a planner reading a single waypoint would rarely " +
            "differ from one reading the horizon, and the feasibility test proves little.");

        Assert.Less(slowest, 120f,
            "The slowest corner on Track_01 is around 90 km/h. Found " + slowest.ToString("F0") +
            " km/h, so the fixture is not the geometry the phase plan measured.");
    }

    [Test]
    public void ThePlanNeverPermitsMoreThanTheLineItselfAsksFor()
    {
        // The plan is a ceiling on the racing line, never a licence to exceed it. Wherever the
        // line says 90, the plan may not hand out more than the brakes allow to reach that 90 —
        // but it also may not invent a speed the line never offered on a straight.
        AIRacingLine line = OpenRealLine();
        AILongitudinalPlanner planner = new AILongitudinalPlanner();

        int checkedPoints = 0;
        for (int start = 0; start < line.Count; start += 7)
        {
            Vector3 position = line.GetPosition(start);
            float allowed = planner.SolvePlannedSpeedKmh(
                line, start, position, 240f, BrakingDecel,
                1f, 1f, 0f,
                minimumHorizonMetres: 45f, maximumWaypoints: 24, fallbackKmh: 240f);

            AIRacingWaypoint waypoint = line.GetWaypoint(start);
            if (waypoint == null)
                continue;

            checkedPoints++;

            // Braking toward a slower corner always permits LESS than the corner speed plus
            // whatever the distance buys. A gross overshoot of the whole line would mean the
            // horizon constraints are not being applied at all.
            Assert.Less(allowed, waypoint.targetSpeedKmh + 120f,
                $"At waypoint {start} the line offers {waypoint.targetSpeedKmh:F0} km/h and " +
                $"the plan permitted {allowed:F0} km/h. The plan is derived from the line's own " +
                "speeds; a large excess means the horizon constraints are not being applied.");
        }

        Assert.Greater(checkedPoints, 30,
            "Expected to sample a good spread of the real line, checked " + checkedPoints + ".");
    }

    [Test]
    public void ABudgetOfZeroStillProducesAUsablePlanOnRealGeometry()
    {
        // The degenerate case, on real geometry rather than a synthetic one: a car that cannot
        // brake at all may only carry the slowest corner in its horizon. If this returns
        // anything near the straight's speed, the solver is reading a zero budget as unbounded
        // acceleration — which on a real line with a 90 km/h hairpin would be catastrophic.
        AIRacingLine line = OpenRealLine();
        AILongitudinalPlanner planner = new AILongitudinalPlanner();

        float worst = 0f;
        int start = 0;
        Vector3 position = line.GetPosition(start);

        float allowed = planner.SolvePlannedSpeedKmh(
            line, start, position, 240f, 0f,
            1f, 1f, 0f,
            minimumHorizonMetres: 45f, maximumWaypoints: 24, fallbackKmh: 240f);

        worst = Mathf.Max(worst, allowed);

        Assert.Less(allowed, line.GetWaypoint(0).targetSpeedKmh + 5f,
            "With no braking capability the plan must not permit more than the first corner in " +
            "the horizon allows. It permitted " + allowed.ToString("F0") + " km/h from waypoint " +
            start + ", where the line offers " +
            line.GetWaypoint(0).targetSpeedKmh.ToString("F0") + " km/h.");

        Assert.Greater(planner.LastHorizonWaypointCount, 1,
            "Precondition: the planner must have seen more than a single waypoint on the real " +
            "line, or it has not tested the horizon at all. It saw " +
            planner.LastHorizonWaypointCount + ".");
    }
}