// Assets/Editor/TrackRacingLineBaker.cs
//
// Derives a drivable racing line, a canonical start pose and a grid anchor from a track
// mesh, so slice step S2 does not depend on hand-placing dozens of waypoints.
//
// Method: the track ribbon is roughly star-shaped about the centroid of its own mesh,
// so for each compass bucket the *median* radius of the tarmac vertices in that bucket is
// a robust estimate of the centreline (the median discards the two ribbon edges). The
// result is inherently a closed loop. It is then smoothed, resampled to evenly spaced
// waypoints, snapped down onto the road surface for correct elevation, and given target
// speeds from local curvature.
//
// "Tarmac vertices" is the load-bearing part. The track mesh is not one surface: it is five
// submeshes with five materials — CrackedRoad, GuardRail, Grass, Sand, Fencing — and the
// Grass submesh alone spans 2700 x 2600 m of terrain around the circuit. Reading every
// vertex as if it were road let that terrain dominate the polar median and threw the
// centreline 335 m past the end of the tarmac, into a field, as a thirty-waypoint spur
// that doubled back on itself. The bake therefore reads exactly one submesh.
//
// This is an approximation, not a racing-line optimiser: it produces the geometric
// centreline, not an optimal line that clips apexes. A hand-tuned line is later polish.
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public class TrackRacingLineBaker
{
    [SerializeField] private MeshFilter _roadFilter;

    // Finer than it looks like it needs to be. 180 buckets over 360 degrees is one bucket per
    // two degrees, which sounds dense until you remember the track is a 3.2 km loop: each
    // bucket is a chord tens of metres long, and a corner that does not happen to be aligned
    // with a bucket boundary is represented by whatever the bucket's median radius was.
    [SerializeField] private int _angleBuckets = 360;

    // 90 waypoints over a 3.2 km lap is 36 m between them, so the AI aim at a straight chord
    // that ignores whatever the corner does between the two ends. That is the line leaving the
    // road at corner entries and exits, and the AI faithfully driving off the track because
    // they are following it. At 300 the spacing is about 11 m, which tracks a corner closely
    // enough to stay on the surface.
    [SerializeField] private int _waypointCount = 300;

    // Smoothing is what rounds a corner off, and then the resample turns that rounded corner
    // into a chord across the corner rather than around it. Eight passes was enough to pull
    // the line well inside the apex at every tight turn. Two removes the sampling noise the
    // median introduces without reshaping the track.
    [SerializeField] private int _smoothPasses = 2;

    [SerializeField] private float _straightSpeedKmh = 240f;
    [SerializeField] private float _minSpeedKmh = 90f;
    [SerializeField] private int _curvatureLookahead = 4;

    // Cornering grip and braking, in m/s^2. Cornering speed follows v = sqrt(a * R), so these
    // two numbers are what turn the geometry of the circuit into a speed profile. A race car
    // with downforce pulls well over 1 g in a fast corner; 12 m/s^2 is roughly 1.2 g, which
    // leaves the AI some margin instead of asking it to drive on the limit it does not have.
    [SerializeField] private float _maxLateralAccel = 12f;
    [SerializeField] private float _brakeAccel = 18f;

    private MeshCollider _roadCollider;
    // Which submesh is the drivable surface. Matched against the renderer's material names
    // rather than hardcoded, because submesh order is an export detail that changes when the
    // source asset is reimported.
    [SerializeField] private string _tarmacMaterialHint = "Road";

    private int _tarmacSubmesh = -1;
    private Vector3[] _tarmacVerts = new Vector3[0];
    private int[] _tarmacTris = new int[0];
    private int[] _triangleVertexMap = new int[0];

    private List<Vector3> _line = new List<Vector3>();

    public IReadOnlyList<Vector3> BakedLine => _line;
    public float BakedLapLength { get; private set; }

    public static TrackRacingLineBaker FromRoad(GameObject road)
    {
        var baker = new TrackRacingLineBaker();
        baker._roadFilter = road.GetComponent<MeshFilter>();
        baker._roadCollider = road.GetComponent<MeshCollider>();
        if (baker._roadFilter == null || baker._roadCollider == null)
        {
            Debug.LogError("[TrackBaker] The road needs both a MeshFilter and a MeshCollider.");
            return null;
        }

        baker.ResolveTarmac();
        return baker;
    }

    /// <summary>
    /// Picks the submesh that is the racing surface and caches its world-space vertices.
    ///
    /// Falls back to submesh 0 when nothing matches, because a wrong-but-close guess beats
    /// refusing to bake — but it warns, since that fallback is precisely what silently lets
    /// the surrounding terrain back into the centreline.
    /// </summary>
    private void ResolveTarmac()
    {
        _tarmacSubmesh = -1;

        var mesh = _roadFilter.sharedMesh;
        var renderer = _roadFilter.GetComponent<MeshRenderer>();
        var materials = renderer != null ? renderer.sharedMaterials : null;

        if (materials != null && !string.IsNullOrEmpty(_tarmacMaterialHint))
        {
            for (int i = 0; i < materials.Length && i < mesh.subMeshCount; i++)
            {
                string name = materials[i] != null ? materials[i].name : string.Empty;
                if (name.IndexOf(_tarmacMaterialHint, System.StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    _tarmacSubmesh = i;
                    break;
                }
            }
        }

        if (_tarmacSubmesh < 0)
        {
            _tarmacSubmesh = 0;
            string available = materials != null && materials.Length > 0 && materials[0] != null
                ? materials[0].name
                : "<none>";
            Debug.LogWarning($"[TrackBaker] No material matched '{_tarmacMaterialHint}'; " +
                             $"falling back to submesh 0 ('{available}'). If the tarmac is not " +
                             "submesh 0 the centreline will drift onto the surrounding terrain.");
        }

        _tarmacTris = mesh.GetTriangles(_tarmacSubmesh);

        // A submesh indexes the mesh's shared vertex array, so gather the indices it actually
        // uses rather than assuming they are contiguous, and keep a map back from each
        // triangle corner to its slot in the compacted array.
        var used = new HashSet<int>();
        for (int i = 0; i < _tarmacTris.Length; i++) used.Add(_tarmacTris[i]);

        var transform = _roadFilter.transform;
        var local = mesh.vertices;
        var world = new Vector3[used.Count];
        var compacted = new Dictionary<int, int>(used.Count);
        int w = 0;
        foreach (int index in used)
        {
            compacted[index] = w;
            world[w++] = transform.TransformPoint(local[index]);
        }
        _tarmacVerts = world;

        _triangleVertexMap = new int[_tarmacTris.Length];
        for (int i = 0; i < _tarmacTris.Length; i++)
            _triangleVertexMap[i] = compacted[_tarmacTris[i]];

        Debug.Log($"[TrackBaker] Tarmac submesh {_tarmacSubmesh}: " +
                  $"{_tarmacTris.Length / 3} triangles, {_tarmacVerts.Length} vertices " +
                  $"(of {local.Length} in the whole mesh).");
    }

    /// <summary>
    /// Returns the snapped, closed centreline in world space, or an empty list on
    /// failure. Never throws; the caller checks the count.
    /// </summary>
    public List<Vector3> Bake()
    {
        _line.Clear();
        BakedLapLength = 0f;

        // Preferred path: the tarmac is authored as a chain of cross-section cells, so its
        // centreline is read directly rather than inferred. The polar fallback cannot
        // represent a hairpin at all, so this is not a nicety.
        var cells = BakeFromCrossSections();
        if (cells.Count >= 8)
        {
            SnapToTarmac(cells);
            _line.AddRange(cells);
            BakedLapLength = LapLength(_line);
            return _line;
        }

        Debug.LogWarning("[TrackBaker] Tarmac is not a chain of cross-section cells; " +
                         "falling back to the polar-centroid estimate, which cannot " +
                         "represent hairpins.");
        return BakePolarCentroid();
    }

    /// <summary>
    /// Reads the centreline straight off the tarmac's cross-section cells.
    ///
    /// This track's tarmac is not a triangulated surface: it is 341 disconnected cells, each
    /// two triangles over four vertices, laid end to end every ten metres, sharing no edges
    /// with its neighbours. Each one spans the full width of the road, so the mean of its
    /// vertices is a point on the centreline. Ordering the cells by proximity and walking the
    /// resulting chain gives the racing line exactly, with no smoothing guesswork and no
    /// assumption that the circuit is star-shaped.
    ///
    /// Returns an empty list when the mesh is not built this way, so the caller can fall back.
    /// </summary>
    private List<Vector3> BakeFromCrossSections()
    {
        var centres = ExtractCellCentres();
        if (centres.Count < 8) return new List<Vector3>();

        // Link cells that sit within a stride of each other. The stride comes from the mesh
        // itself (the median gap to a cell's nearest neighbour) rather than a hardcoded
        // number, so it scales with the track.
        float stride = MedianNeighbourGap(centres);
        if (stride < 0.5f) return new List<Vector3>();
        float linkDistance = stride * 1.6f;

        int n = centres.Count;
        var adjacency = new List<int>[n];
        for (int i = 0; i < n; i++) adjacency[i] = new List<int>();

        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                if ((centres[i] - centres[j]).sqrMagnitude <= linkDistance * linkDistance)
                {
                    adjacency[i].Add(j);
                    adjacency[j].Add(i);
                }
            }
        }

        var order = WalkCellChain(centres, adjacency);
        if (order == null)
        {
            Debug.LogWarning("[TrackBaker] The cross-section cells do not form a single closed " +
                             $"loop (linked {CountLinked(adjacency)} of {n}); using the polar estimate.");
            return new List<Vector3>();
        }

        Debug.Log($"[TrackBaker] Centreline from {n} cross-section cells, stride {stride:0.0} m.");

        var raw = new List<Vector3>(order.Count);
        foreach (int index in order) raw.Add(centres[index]);

        // The cells are already the true cross-sections, so this is a light pass that removes
        // export wobble without reshaping anything.
        SmoothClosed(raw, 1);
        return Resample(raw, _waypointCount);
    }

    /// <summary>
    /// Groups the tarmac triangles into connected components (one per cross-section cell) and
    /// returns the world-space centre of each.
    /// </summary>
    private List<Vector3> ExtractCellCentres()
    {
        var tri = _tarmacTris;
        int triangleCount = tri.Length / 3;
        var result = new List<Vector3>();
        if (triangleCount == 0) return result;

        // Two triangles belong to the same cell when they share an edge. An edge here is a
        // pair of mesh vertex indices; pack it into a single sortable long.
        var edgeOwner = new Dictionary<long, int>(triangleCount * 3);
        var parent = new int[triangleCount];
        for (int i = 0; i < triangleCount; i++) parent[i] = i;

        for (int t = 0; t < triangleCount; t++)
        {
            int a = _triangleVertexMap[t * 3];
            int b = _triangleVertexMap[t * 3 + 1];
            int c = _triangleVertexMap[t * 3 + 2];
            Union(parent, t, a, b, edgeOwner);
            Union(parent, t, b, c, edgeOwner);
            Union(parent, t, c, a, edgeOwner);
        }

        var byRoot = new Dictionary<int, List<int>>();
        for (int t = 0; t < triangleCount; t++)
        {
            int root = Find(parent, t);
            if (!byRoot.TryGetValue(root, out var list))
            {
                list = new List<int>();
                byRoot[root] = list;
            }
            list.Add(t);
        }

        foreach (var cell in byRoot.Values)
        {
            var corners = new HashSet<int>();
            foreach (int t in cell)
            {
                corners.Add(_triangleVertexMap[t * 3]);
                corners.Add(_triangleVertexMap[t * 3 + 1]);
                corners.Add(_triangleVertexMap[t * 3 + 2]);
            }

            Vector3 sum = Vector3.zero;
            foreach (int corner in corners) sum += _tarmacVerts[corner];
            result.Add(sum / corners.Count);
        }

        return result;
    }

    private static long EdgeKey(int a, int b)
    {
        int lo = a < b ? a : b;
        int hi = a < b ? b : a;
        return (long)lo * 100000L + (hi - lo);
    }

    private static void Union(int[] parent, int t, int a, int b,
                              Dictionary<long, int> edgeOwner)
    {
        long key = EdgeKey(a, b);
        if (edgeOwner.TryGetValue(key, out int other))
        {
            int ra = Find(parent, t);
            int rb = Find(parent, other);
            if (ra != rb) parent[ra] = rb;
        }
        else
        {
            edgeOwner[key] = t;
        }
    }

    private static int Find(int[] parent, int i)
    {
        while (parent[i] != i)
        {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    }

    private static int CountLinked(List<int>[] adjacency)
    {
        int count = 0;
        for (int i = 0; i < adjacency.Length; i++) count += adjacency[i].Count;
        return count / 2;
    }

    private static float MedianNeighbourGap(List<Vector3> centres)
    {
        var gaps = new List<float>(centres.Count);
        for (int i = 0; i < centres.Count; i++)
        {
            float best = float.MaxValue;
            for (int j = 0; j < centres.Count; j++)
            {
                if (i == j) continue;
                float d = (centres[i] - centres[j]).sqrMagnitude;
                if (d < best) best = d;
            }
            gaps.Add(Mathf.Sqrt(best));
        }
        gaps.Sort();
        return gaps[gaps.Count / 2];
    }

    /// <summary>
    /// Walks the cell graph into one closed loop, always taking the straightest unvisited
    /// neighbour. Returns null if the walk dead-ends before closing on itself, which is the
    /// signal that this mesh is not a simple chain of cells.
    /// </summary>
    private static List<int> WalkCellChain(List<Vector3> centres, List<int>[] adjacency)
    {
        int n = centres.Count;
        var best = new List<int>();

        // Several starts, because a chain has two ends that are indistinguishable and one of
        // them is a much better place to begin than the other.
        int stride = Mathf.Max(1, n / 24);
        for (int start = 0; start < n; start += stride)
        {
            var visited = new bool[n];
            var order = new List<int>(n) { start };
            visited[start] = true;

            int current = start;
            Vector3 heading = Vector3.zero;
            bool hasHeading = false;

            for (int guard = 0; guard < n + 8; guard++)
            {
                int next = -1;
                float bestScore = float.MinValue;
                foreach (int candidate in adjacency[current])
                {
                    if (visited[candidate]) continue;

                    Vector3 step = centres[candidate] - centres[current];
                    float length = step.magnitude;
                    if (length < 1e-4f) continue;
                    step /= length;

                    // First move takes the nearest cell; after that, prefer the straightest
                    // continuation so the walk follows the road instead of hopping between the
                    // two arms of a hairpin that happen to pass close by.
                    float score = hasHeading ? Vector3.Dot(step, heading) * 10f - length * 0.1f
                                             : -length;
                    if (score > bestScore)
                    {
                        bestScore = score;
                        next = candidate;
                    }
                }

                if (next < 0) break;
                heading = (centres[next] - centres[current]).normalized;
                hasHeading = true;
                current = next;
                visited[current] = true;
                order.Add(current);
            }

            // The walk finishes on the cell next to the start — the start itself is already
            // visited, so it is never re-entered. "Did we consume the whole chain" is the
            // closure test, not "did we land back on the start".
            if (order.Count > best.Count) best = order;
        }

        // A closed loop that consumed every cell is the success condition. A walk that
        // dead-ends has left part of the chain unvisited and cannot describe a lap.
        return best.Count == n && n >= 8 ? best : null;
    }

    /// <summary>
    /// The original polar-centroid estimate, kept as a fallback for tarmac meshes that are
    /// not laid out as cross-section cells. See the class comment for why it is second choice.
    /// </summary>
    private List<Vector3> BakePolarCentroid()
    {
        _line.Clear();
        BakedLapLength = 0f;

        var verts = _roadFilter.sharedMesh.vertices;
        if (verts == null || verts.Length == 0)
        {
            Debug.LogError("[TrackBaker] Road mesh has no vertices.");
            return _line;
        }

        if (_tarmacVerts.Length == 0 || _tarmacTris.Length == 0)
        {
            Debug.LogError("[TrackBaker] Tarmac submesh is empty; nothing to bake from.");
            return _line;
        }

        // --- Pass 1: bucket tarmac vertices by compass angle, keeping median radius ---
        //
        // Only the tarmac. The centroid in particular has to come from the tarmac too: the
        // Grass submesh is 2700 x 2600 m of terrain around the circuit, so a centroid taken
        // over every vertex is not the centre of the track at all, and every radius measured
        // from it is wrong by however far the terrain dragged it.
        var radii = new List<float>[_angleBuckets];
        for (int i = 0; i < _angleBuckets; i++) radii[i] = new List<float>();

        Vector2 centroid = Vector2.zero;
        foreach (var w in _tarmacVerts) centroid += new Vector2(w.x, w.z);
        centroid /= _tarmacVerts.Length;

        foreach (var w in _tarmacVerts)
        {
            Vector2 d = new Vector2(w.x - centroid.x, w.z - centroid.y);
            float r = d.magnitude;
            if (r < 1f) continue; // ignore vertices sitting on the centroid

            float angle = Mathf.Atan2(d.y, d.x); // (-pi, pi]
            if (angle < 0f) angle += Mathf.PI * 2f;
            int bucket = Mathf.Clamp(
                Mathf.FloorToInt(angle / (Mathf.PI * 2f) * _angleBuckets), 0, _angleBuckets - 1);
            radii[bucket].Add(r);
        }

        // --- Pass 2: median per bucket, filling empty buckets by interpolation ---
        var polar = new float[_angleBuckets];
        for (int i = 0; i < _angleBuckets; i++)
        {
            var list = radii[i];
            if (list.Count == 0) { polar[i] = -1f; continue; }
            list.Sort();
            polar[i] = list[list.Count / 2];
        }

        FillGaps(polar);
        if (!HasUsableData(polar)) return _line;

        // --- Pass 3: back to XZ around the centroid, then smooth the closed loop ---
        var loop = new List<Vector3>(_angleBuckets);
        for (int i = 0; i < _angleBuckets; i++)
        {
            float angle = (i + 0.5f) / _angleBuckets * Mathf.PI * 2f;
            loop.Add(new Vector3(
                centroid.x + Mathf.Cos(angle) * polar[i],
                0f,
                centroid.y + Mathf.Sin(angle) * polar[i]));
        }

        for (int pass = 0; pass < _smoothPasses; pass++) SmoothClosed(loop);

        // --- Pass 4: resample evenly by arc length ---
        var resampled = Resample(loop, _waypointCount);
        if (resampled.Count < 8)
        {
            Debug.LogError("[TrackBaker] Resampling produced too few points; the extracted " +
                           "loop is probably degenerate.");
            return _line;
        }

        // --- Pass 4b: remove spikes ---
        //
        // A guard, not the primary defence. The spur that used to run 335 m off the end of the
        // tarmac was caused by the surrounding terrain being averaged in as if it were road;
        // reading the tarmac submesh alone is what removes it. What remains here is the case
        // the median method genuinely cannot handle: a circuit that is not star-shaped about
        // its own centroid, so one radius per compass angle is simply the wrong model and a
        // spur survives resampling.
        //
        // A spike is recognisable without knowing the road: it is far from the midpoint of its
        // neighbours while those neighbours are close to each other. Pulling it back onto them
        // costs nothing on a well-formed line, because on a well-formed line no point is far
        // from its neighbours' midpoint.
        Despike(resampled);

        // --- Pass 5: snap each point down onto the tarmac for elevation ---
        SnapToTarmac(resampled);

        _line.AddRange(resampled);
        BakedLapLength = LapLength(resampled);
        return _line;
    }

    /// <summary>
    /// Puts every point on the tarmac surface and lifts it clear of the road.
    ///
    /// Works against the tarmac triangles directly, not the road's MeshCollider. That collider
    /// wraps the whole five-submesh mesh, whose triangles are coarse enough to span hundreds of
    /// metres — the nearest triangle to the old grass spur had a 1449 m perimeter — so a
    /// downward ray happily "found road" under a waypoint standing in a field and reported a
    /// perfectly plausible height for it. Snapping to the nearest tarmac triangle cannot do
    /// that, and it doubles as the check that the line is actually on the road.
    /// </summary>
    private void SnapToTarmac(List<Vector3> points)
    {
        int offRoad = 0;
        for (int i = 0; i < points.Count; i++)
        {
            Vector3 p = points[i];
            if (TrySnapToTarmac(p, out Vector3 snapped))
            {
                // Lift slightly so the waypoint sits above the surface, not inside it.
                points[i] = snapped + Vector3.up * 0.5f;
            }
            else
            {
                offRoad++;
            }
        }

        if (offRoad > 0)
        {
            Debug.LogWarning($"[TrackBaker] {offRoad} of {points.Count} waypoints have no " +
                             "tarmac under them and were left at the polar estimate. On this " +
                             "track that usually means the circuit is not star-shaped about " +
                             "the tarmac's centroid; check the spur in the scene view.");
        }
    }

    /// <summary>
    /// Finds the point on the nearest tarmac triangle to <paramref name="p"/>.
    ///
    /// Brute force over the tarmac triangles: a few hundred of them against a few hundred
    /// waypoints, which is nothing, and it needs no collider, no bounds guess and no ray.
    /// Returns false only when the tarmac has no triangles at all.
    /// </summary>
    private bool TrySnapToTarmac(Vector3 p, out Vector3 snapped)
    {
        snapped = p;
        float bestSqr = float.MaxValue;
        bool found = false;

        for (int t = 0; t < _tarmacTris.Length; t += 3)
        {
            Vector3 a = _tarmacVerts[IndexOfVertex(t)];
            Vector3 b = _tarmacVerts[IndexOfVertex(t + 1)];
            Vector3 c = _tarmacVerts[IndexOfVertex(t + 2)];

            Vector3 q = ClosestPointOnTriangle(a, b, c, p);
            float sqr = (q - p).sqrMagnitude;
            if (sqr < bestSqr)
            {
                bestSqr = sqr;
                snapped = q;
                found = true;
            }
        }

        return found;
    }

    /// <summary>
    /// Maps a tarmac-submesh triangle index onto the compacted vertex array built by
    /// ResolveTarmac.
    /// </summary>
    private int IndexOfVertex(int triangleCorner)
    {
        return _triangleVertexMap[triangleCorner];
    }

    private static Vector3 ClosestPointOnTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3 p)
    {
        // Ericson, Real-Time Collision Detection: the standard Voronoi-region walk.
        Vector3 ab = b - a, ac = c - a, ap = p - a;
        float d1 = Vector3.Dot(ab, ap), d2 = Vector3.Dot(ac, ap);
        if (d1 <= 0f && d2 <= 0f) return a;

        Vector3 bp = p - b;
        float d3 = Vector3.Dot(ab, bp), d4 = Vector3.Dot(ac, bp);
        if (d3 >= 0f && d4 <= d3) return b;

        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0f && d1 >= 0f && d3 <= 0f) return a + ab * (d1 / (d1 - d3));

        Vector3 cp = p - c;
        float d5 = Vector3.Dot(ab, cp), d6 = Vector3.Dot(ac, cp);
        if (d6 >= 0f && d5 <= d6) return c;

        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0f && d2 >= 0f && d6 <= 0f) return a + ac * (d2 / (d2 - d6));

        float va = d3 * d6 - d5 * d4;
        if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

        float denom = 1f / (va + vb + vc);
        return a + ab * (vb * denom) + ac * (vc * denom);
    }

    private static Vector3 ForwardAt(List<Vector3> line, int index)
    {
        int n = line.Count;
        Vector3 a = line[(index - 1 + n) % n];
        Vector3 b = line[(index + 1) % n];
        Vector3 f = b - a;
        return f.sqrMagnitude < 0.0001f ? Vector3.forward : f.normalized;
    }

    private static bool HasUsableData(float[] polar)
    {
        int filled = 0;
        float sum = 0f;
        for (int i = 0; i < polar.Length; i++)
        {
            if (polar[i] > 0f) { filled++; sum += polar[i]; }
        }

        if (filled < polar.Length * 0.5f)
        {
            Debug.LogError($"[TrackBaker] Only {filled}/{polar.Length} compass buckets " +
                           "contained road. The track is probably not star-shaped about its " +
                           "centroid (a figure-eight, for example) and this method does not apply.");
            return false;
        }

        return sum > 0f;
    }

    private static void FillGaps(float[] values)
    {
        int n = values.Length;
        bool[] missing = new bool[n];
        int missingCount = 0;
        for (int i = 0; i < n; i++)
        {
            missing[i] = values[i] <= 0f;
            if (missing[i]) missingCount++;
        }

        if (missingCount == 0 || missingCount == n) return;

        for (int i = 0; i < n; i++)
        {
            if (!missing[i]) continue;

            int back = 0;
            while (back < n && missing[(i - back + n * 2) % n]) back++;
            int fwd = 0;
            while (fwd < n && missing[(i + fwd) % n]) fwd++;

            int prevIndex = (i - back + n * 2) % n;
            int nextIndex = (i + fwd) % n;
            float prevValue = values[prevIndex];
            float nextValue = values[nextIndex];

            float t = (float)back / (back + fwd);
            values[i] = Mathf.Lerp(prevValue, nextValue, t);
        }
    }

    /// <summary>
    /// Pulls waypoints that sit far from their neighbours' midpoint back onto the line.
    ///
    /// Runs to a fixed number of passes rather than to a fixed threshold, because a single
    /// pass can leave a spike next to its replacement: moving the worst point first changes
    /// which point is worst next. Three passes settles it, and a line with no spikes is
    /// unchanged by all three.
    /// </summary>
    private static void Despike(List<Vector3> line)
    {
        int n = line.Count;
        if (n < 8) return;

        for (int pass = 0; pass < 3; pass++)
        {
            // The threshold is relative to the line's own spacing, so it scales with the track
            // rather than being a number that happens to suit one circuit.
            float spacing = LapLength(line) / n;
            float threshold = Mathf.Max(2f, spacing * 0.75f);

            bool changed = false;
            for (int i = 0; i < n; i++)
            {
                Vector3 prev = line[(i - 1 + n) % n];
                Vector3 next = line[(i + 1) % n];
                Vector3 midpoint = (prev + next) * 0.5f;

                if (Vector3.Distance(line[i], midpoint) > threshold)
                {
                    line[i] = midpoint;
                    changed = true;
                }
            }

            if (!changed) return;
        }
    }

    private void SmoothClosed(List<Vector3> line)
    {
        for (int pass = 0; pass < _smoothPasses; pass++) SmoothClosed(line, 1);
    }

    private static void SmoothClosed(List<Vector3> line, int passes)
    {
        int n = line.Count;
        if (n < 3) return;

        for (int pass = 0; pass < passes; pass++)
        {
            var copy = new List<Vector3>(line);
            for (int i = 0; i < n; i++)
            {
                Vector3 prev = copy[(i - 1 + n) % n];
                Vector3 next = copy[(i + 1) % n];
                line[i] = (prev + copy[i] * 2f + next) * 0.25f;
            }
        }
    }

    private static List<Vector3> Resample(List<Vector3> line, int count)
    {
        var result = new List<Vector3>(count);
        if (line.Count < 3) return result;

        var cumulative = new List<float>(line.Count + 1);
        cumulative.Add(0f);
        for (int i = 0; i < line.Count; i++)
        {
            Vector3 a = line[i];
            Vector3 b = line[(i + 1) % line.Count];
            cumulative.Add(cumulative[i] + Vector3.Distance(a, b));
        }

        float total = cumulative[line.Count];
        if (total < 1f) return result;

        int segment = 0;
        for (int i = 0; i < count; i++)
        {
            float target = total * i / count;
            while (segment < line.Count - 1 && cumulative[segment + 1] < target) segment++;

            float segStart = cumulative[segment];
            float segLength = cumulative[segment + 1] - segStart;
            float t = segLength > 0.0001f ? (target - segStart) / segLength : 0f;
            result.Add(Vector3.Lerp(line[segment], line[(segment + 1) % line.Count], t));
        }

        return result;
    }

    private static float LapLength(List<Vector3> line)
    {
        float total = 0f;
        for (int i = 0; i < line.Count; i++)
            total += Vector3.Distance(line[i], line[(i + 1) % line.Count]);
        return total;
    }

    /// <summary>
    /// Target speed for every waypoint, in km/h.
    ///
    /// Derived from the corner the line actually describes rather than from how much it
    /// happens to turn between two samples. The speed a car can hold through a corner of
    /// radius R is set by grip, not by steering angle: v = sqrt(a_lateral * R). That single
    /// relation is what makes a hairpin slow and a straight fast without any tuning per
    /// corner, and it stays correct when the racing line is resampled more finely.
    ///
    /// The previous version mapped the turn angle across a fixed lookahead onto a speed range,
    /// which is not a physical quantity. On a correctly extracted line it read the hairpin as
    /// barely a corner and asked for 211 km/h through it — about 5.8 g — so the AI would arrive
    /// at full speed and drive straight off. It only produced sensible numbers before because
    /// the line's own spikes were creating curvature that the road does not have.
    ///
    /// A backward pass then propagates each corner's limit forwards along the approach, so the
    /// car is already slowing on entry rather than braking mid-corner. Two laps of the loop are
    /// enough to settle it everywhere.
    /// </summary>
    public float[] ComputeTargetSpeeds(List<Vector3> line)
    {
        int n = line.Count;
        var speeds = new float[n];
        if (n < 8)
        {
            for (int i = 0; i < n; i++) speeds[i] = _straightSpeedKmh;
            return speeds;
        }

        for (int i = 0; i < n; i++)
        {
            float radius = RadiusAt(line, i);
            // Infinite radius on a straight, so the straight speed is the natural ceiling.
            float v = Mathf.Sqrt(_maxLateralAccel * radius);
            speeds[i] = Mathf.Clamp(v * 3.6f, _minSpeedKmh, _straightSpeedKmh);
        }

        // Backward braking pass: you cannot enter a corner faster than you can shed the excess
        // speed over the distance available before it.
        for (int lap = 0; lap < 2; lap++)
        {
            for (int step = 0; step < n; step++)
            {
                int i = n - 1 - step;
                int next = (i + 1) % n;
                float gap = Vector3.Distance(line[i], line[next]);
                if (gap < 0.01f) continue;

                float v = speeds[i] / 3.6f;
                float vNext = speeds[next] / 3.6f;
                float reachable = Mathf.Sqrt(vNext * vNext + 2f * _brakeAccel * gap);
                speeds[i] = Mathf.Min(speeds[i], reachable * 3.6f);
            }
        }

        return speeds;
    }

    /// <summary>
    /// Radius of the circle through the points either side of <paramref name="index"/>.
    /// Returns a very large number on a straight, where the three points are collinear.
    /// </summary>
    private float RadiusAt(List<Vector3> line, int index)
    {
        int n = line.Count;
        int k = Mathf.Clamp(_curvatureLookahead, 1, n - 1);

        Vector3 p0 = line[(index - k + n * 2) % n];
        Vector3 p1 = line[index];
        Vector3 p2 = line[(index + k) % n];

        float a = Vector3.Distance(p1, p0);
        float b = Vector3.Distance(p2, p1);
        float c = Vector3.Distance(p2, p0);
        if (a < 0.01f || b < 0.01f || c < 0.01f) return _straightSpeedKmh * _straightSpeedKmh;

        // Twice the triangle's area, from the cross product. Zero means collinear.
        Vector3 ab = p1 - p0, ac = p2 - p0;
        float twiceArea = ab.x * ac.z - ab.z * ac.x;
        if (Mathf.Abs(twiceArea) < 0.0001f) return _straightSpeedKmh * _straightSpeedKmh;

        // Circumradius R = abc / (4 * area) = abc / (2 * twiceArea).
        return Mathf.Abs(a * b * c / (2f * twiceArea));
    }

    /// <summary>Creates the waypoint hierarchy and returns the configured racing line.</summary>
    public AIRacingLine CreateRacingLine(List<Vector3> line, Transform parent, string name)
    {
        var root = new GameObject(name);
        root.transform.SetParent(parent, false);
        var racingLine = root.AddComponent<AIRacingLine>();
        racingLine.loop = true;

        var waypoints = new AIRacingWaypoint[line.Count];
        var speeds = ComputeTargetSpeeds(line);
        for (int i = 0; i < line.Count; i++)
        {
            var go = new GameObject($"WP_{i:000}");
            go.transform.SetParent(root.transform, false);
            go.transform.position = line[i];

            Vector3 forward = ForwardAt(line, i);
            if (forward.sqrMagnitude > 0.0001f)
                go.transform.rotation = Quaternion.LookRotation(forward.normalized, Vector3.up);

            var wp = go.AddComponent<AIRacingWaypoint>();
            wp.targetSpeedKmh = speeds[i];
            wp.laneWidth = 16f;
            wp.preferredLaneOffset01 = 0f;
            waypoints[i] = wp;
        }

        racingLine.waypoints = waypoints;
        return racingLine;
    }
}
