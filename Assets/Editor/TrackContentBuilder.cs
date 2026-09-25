// Assets/Editor/TrackContentBuilder.cs
//
// Builds slice step S2's track content: places the track prefab in the track scene,
// bakes a racing line from its road mesh, derives the canonical start pose and grid
// anchor, and wires RaceGridManager to them.
//
// Idempotent: re-running replaces the generated hierarchy rather than adding a second
// copy. Run via Unity menu: Tools > BuildTrackContent.
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public class TrackContentBuilder
{
    private const string TrackScenePath = "Assets/Scenes/Track_01.unity";
    private const string TrackPrefabPath =
        "Assets/Assets/Environmental Race Track Pack/Prefabs/F1 RaceTrack.prefab";
    private const string RoadChildName = "MainTrack_Object";
    private const string TrackRootName = "F1RaceTrack";
    private const string PlacementName = "TrackPlacement";
    private const string RacingLineName = "RacingLine";
    private const string GridManagerName = "RaceGridManager";
    private const string StartPoseName = "StartPose";
    private const string GridAnchorName = "GridAnchor";

    // The grid's field and the one car prefab it spawns. Declared here so a re-bake keeps the
    // grid wired rather than silently emptying it.
    private const string AiCarPrefabPath = "Assets/Prefabs/AI_F1_Body.prefab";
    private const string AiFieldAssetPath = "Assets/Resources/GameData/AI/AI_Field_Default.asset";
    private const string LightName = "TrackSun";

    [MenuItem("Tools/BuildTrackContent")]
    public static void Build()
    {
        var scene = EditorSceneManager.OpenScene(TrackScenePath, OpenSceneMode.Single);

        // --- 1. Track geometry ---
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(TrackPrefabPath);
        if (prefab == null)
        {
            Debug.LogError($"[TrackContent] Track prefab not found at {TrackPrefabPath}.");
            return;
        }

        var trackRoot = GameObject.Find(TrackRootName);
        if (trackRoot == null)
        {
            trackRoot = PrefabUtility.InstantiatePrefab(prefab) as GameObject;
            trackRoot.name = TrackRootName;
        }

        var road = trackRoot.transform.Find(RoadChildName);
        if (road == null)
        {
            Debug.LogError($"[TrackContent] '{RoadChildName}' not found under {TrackRootName}.");
            return;
        }

        // --- 2. Idempotent cleanup of previously generated content ---
        var staleRacingLine = trackRoot.transform.Find(RacingLineName);
        if (staleRacingLine != null) Object.DestroyImmediate(staleRacingLine.gameObject);

        var stalePlacement = GameObject.Find(PlacementName);
        if (stalePlacement != null) Object.DestroyImmediate(stalePlacement);

        var staleGrid = GameObject.Find(GridManagerName);
        if (staleGrid != null) Object.DestroyImmediate(staleGrid);

        // --- 3. Bake the racing line ---
        var baker = TrackRacingLineBaker.FromRoad(road.gameObject);
        if (baker == null) return;

        var line = baker.Bake();
        if (line.Count < 8)
        {
            Debug.LogError("[TrackContent] Racing-line bake failed; track content not written.");
            return;
        }

        var racingLine = baker.CreateRacingLine(line, trackRoot.transform, RacingLineName);

        // --- 4. Canonical start pose and grid anchor ---
        // The start/finish is the point on the baked line nearest the pit road, which is
        // where a real circuit puts it. Falls back to index 0 if the pit road is absent.
        int startIndex = FindStartFinishIndex(trackRoot, line);

        var placementGo = new GameObject(PlacementName);
        placementGo.transform.SetParent(trackRoot.transform, false);

        var startPose = new GameObject(StartPoseName).transform;
        startPose.SetParent(placementGo.transform, false);
        startPose.position = RoadLevel(road.GetComponent<Collider>(), line[startIndex]);
        startPose.rotation = LookAlong(line, startIndex);

        var gridAnchor = new GameObject(GridAnchorName).transform;
        gridAnchor.SetParent(placementGo.transform, false);
        // Same basis as the start pose: grid row 0 sits on the line and the rest stack
        // backwards, so pole is genuinely ahead.
        gridAnchor.position = RoadLevel(road.GetComponent<Collider>(), line[startIndex]);
        gridAnchor.rotation = LookAlong(line, startIndex);

        var placement = placementGo.AddComponent<TrackPlacement>();
        placement.Configure(
            racingLine,
            road.GetComponent<Collider>(),
            startPose,
            gridAnchor,
            startIndex,
            baker.BakedLapLength);

        GameObject gridGo;
        // --- 5. Wire the existing grid system to the new references ---
        //
        // Reuses an existing grid manager rather than creating a second one. Creating one every
        // run is the obvious way to write this and the wrong one: a re-bake left the previous
        // manager in the scene, so the track had two, and FindAnyObjectByType picked between
        // them arbitrarily — meaning the race could be driven by a grid manager that was not
        // the one carrying the field. Two grid managers is the same class of bug as two hosts
        // claiming one screen, which is defect P16.
        var gridTransform = placementGo.transform.Find(GridManagerName);
        if (gridTransform == null)
        {
            gridGo = new GameObject(GridManagerName);
            gridGo.transform.SetParent(placementGo.transform, false);
        }
        else
        {
            gridGo = gridTransform.gameObject;
        }

        var grid = gridGo.GetComponent<RaceGridManager>();
        if (grid == null)
            grid = gridGo.AddComponent<RaceGridManager>();

        grid.racingLine = racingLine;
        grid.gridAnchor = gridAnchor;
        grid.spawnOnStart = false; // the race shell owns spawning, not the content scene
        grid.gridRowSpacing = 18f;
        grid.gridColumnSpacing = 12f;

        // The field and the car it spawns. Re-applied on every bake so a re-bake cannot leave
        // the grid inert — an unwired grid manager produces an empty grid and nothing in the
        // console to say why.
        var aiPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(AiCarPrefabPath);
        var aiField = AssetDatabase.LoadAssetAtPath<AIFieldDefinition>(AiFieldAssetPath);
        if (aiPrefab != null) grid.defaultAICarPrefab = aiPrefab;
        if (aiField != null) grid.aiField = aiField;
        grid.showDriverBoards = true;

        // --- 6. Lighting ---
        // The track scene renders nothing without a light. It lives here rather than in the
        // pre-race or race shell because the track content is what both shells load and what
        // both cameras actually look at: one light serves every session, and a shell that
        // forgot to bring its own would not render a black world.
        var lightGo = GameObject.Find(LightName);
        if (lightGo == null)
            lightGo = new GameObject(LightName);

        var light = lightGo.GetComponent<Light>();
        if (light == null)
            light = lightGo.AddComponent<Light>();

        light.type = LightType.Directional;
        light.color = new Color(1f, 0.97f, 0.92f);
        light.intensity = 1.15f;
        light.shadows = LightShadows.Soft;
        // A high three-quarter key light: enough shape on the car and the road to read
        // speed and distance, without the long raking shadows that hide the racing surface.
        lightGo.transform.rotation = Quaternion.Euler(48f, -35f, 0f);

        // --- 7. Report and save ---
        var report = new System.Text.StringBuilder();
        report.AppendLine("[TrackContent] Track content built.");
        report.AppendLine("  waypoints        = " + racingLine.Count);
        report.AppendLine("  baked lap length = " + baker.BakedLapLength.ToString("0.0") + " m");
        report.AppendLine("  start/finish idx = " + startIndex);
        report.AppendLine("  start position   = " + line[startIndex]);
        report.AppendLine("  IsValid          = " + placement.IsValid);

        float min = float.MaxValue, max = float.MinValue;
        for (int i = 0; i < racingLine.Count; i++)
        {
            var s = racingLine.waypoints[i].targetSpeedKmh;
            if (s < min) min = s;
            if (s > max) max = s;
        }
        report.AppendLine("  target speeds    = " + min.ToString("0") + " .. " + max.ToString("0") + " km/h");

        // Longest gap between adjacent waypoints: a sanity check that the loop closed
        // rather than jumping across the infield.
        float longest = 0f;
        for (int i = 0; i < line.Count; i++)
        {
            float d = Vector3.Distance(line[i], line[(i + 1) % line.Count]);
            if (d > longest) longest = d;
        }
        float average = baker.BakedLapLength / Mathf.Max(1, line.Count);
        report.AppendLine("  spacing          = avg " + average.ToString("0.0")
            + " m, longest gap " + longest.ToString("0.0") + " m");
        if (longest > average * 6f)
            report.AppendLine("  !! longest gap is over 6x the average - the loop may have closed across the infield.");

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        AssetDatabase.SaveAssets();

        Debug.Log(report.ToString());
    }

    private static int FindStartFinishIndex(GameObject trackRoot, List<Vector3> line)
    {
        var pit = trackRoot.transform.Find("Pit_Road");
        if (pit == null) return 0;

        var pitCollider = pit.GetComponent<Collider>();
        Vector3 target = pitCollider != null ? pitCollider.bounds.center : pit.position;

        int best = 0;
        float bestDistance = float.MaxValue;
        for (int i = 0; i < line.Count; i++)
        {
            // Compare in the XZ plane: pit road and racing line differ in height.
            float dx = line[i].x - target.x;
            float dz = line[i].z - target.z;
            float d = dx * dx + dz * dz;
            if (d < bestDistance)
            {
                bestDistance = d;
                best = i;
            }
        }

        return best;
    }

    /// <summary>
    /// Drops a baked line point onto the road surface itself.
    ///
    /// The baker lifts every waypoint half a metre so a waypoint sits *above* the tarmac
    /// rather than inside it, which is right for a waypoint and wrong for an anchor: a start
    /// pose and a grid anchor taken straight from the line inherit that lift, and then
    /// everything placed relative to them is placed half a metre high. The car's spawner adds
    /// its own clearance on top, and the two together lift the car clean off the line.
    ///
    /// This was found once already and fixed by hand, and the hand-fix was then silently
    /// undone by the next bake. Doing it here means a re-bake cannot reintroduce it.
    /// </summary>
    private static Vector3 RoadLevel(Collider road, Vector3 linePoint)
    {
        if (road == null)
            return linePoint;

        var bounds = road.bounds;
        var ray = new Ray(new Vector3(linePoint.x, bounds.max.y + 100f, linePoint.z), Vector3.down);
        return road.Raycast(ray, out var hit, bounds.size.y + 400f) ? hit.point : linePoint;
    }

    private static Quaternion LookAlong(List<Vector3> line, int index)
    {
        int n = line.Count;
        Vector3 forward = line[(index + 1) % n] - line[(index - 1 + n) % n];
        if (forward.sqrMagnitude < 0.0001f) return Quaternion.identity;
        forward.Normalize();
        return Quaternion.LookRotation(forward, Vector3.up);
    }
}
