using System.Collections.Generic;
using UnityEngine;
using F1.TrackData;

// The class TrackData lives INSIDE the namespace F1.TrackData, so from any sibling namespace the
// bare name `TrackData` binds to that namespace and the class becomes unreachable by it
// (`TrackData.SpeedProfileResolutionMetres` then fails to resolve with CS0234). The alias makes
// the collision explicit once instead of at every use site.
using TrackDataAsset = F1.TrackData.TrackData;

namespace F1.EditorTools
{
    public sealed class BakeResult
    {
        public bool Success;
        public string FailureReason;
        public Vector3[] ControlPoints;
        public float[] SpeedProfileKmh;
        public float[] WidthMetres;
        public float LengthMetres;
        public float[] SectorStarts;
        public float ReferenceLapSeconds;
    }

    /// <summary>
    /// The parts of a racing-line bake that need no scene and no mesh, extracted so they can be
    /// tested. Before this existed the whole bake lived in TrackRacingLineBaker, which sits in an
    /// assembly no test can reference, so no bake had ever been verified on any track.
    /// </summary>
    public static class RacingLineBaker
    {
        private const float KmhToMs = 1f / 3.6f;   // km/h -> m/s MULTIPLIES by this. Dividing inflates by 3.6x.
        private const int TargetControlPoints = 60;
        private const int SpeedProfileSamplesPerLapMin = 32;

        /// <summary>
        /// A centreline whose tightest corner is tighter than this is not a circuit corner, it is a
        /// numerical pinch. 8 m is far below any real hairpin (F1's tightest is roughly 40 m) and
        /// far above the artefact this catches (0.3 m), so the check is not sensitive to where in
        /// that range the threshold sits.
        ///
        /// <para>
        /// A RATIO-based threshold was tried first and rejected on the grounds that it would not
        /// separate a real circuit from a pinching one by enough margin. That reasoning is sound,
        /// but the specific figures quoted in earlier drafts of this comment were measured on
        /// fixtures that were not themselves valid circuits, so they have been dropped rather than
        /// restated. The radius check separates a real hairpin (12-40 m) from the degenerate case
        /// (4.51 m at this window) by a wide margin, with 8 m comfortably between them.
        /// </para>
        /// </summary>
        private const float MinimumCornerRadiusMetres = 8f;

        /// <summary>
        /// Tightest corner on the centreline, as a circumradius over a sliding three-point window.
        ///
        /// <para>
        /// Returns <see cref="float.NaN"/> when no corner is measurable AT ALL, which is distinct
        /// from "the corner is very gentle". A collinear centreline produces no measurable triangle
        /// anywhere, and reporting that as <see cref="float.PositiveInfinity"/> — "an infinitely
        /// gentle corner" — makes it pass a minimum-radius check, because +Infinity is not less
        /// than any threshold. Measured: 16 collinear points baked successfully to a 312 m
        /// all-240 km/h profile before this distinguished the two cases.
        /// </para>
        /// </summary>
        private static float MinimumCornerRadius(IList<Vector3> line, int window)
        {
            int n = line.Count;
            if (n < 8) return float.NaN;

            float tightest = float.PositiveInfinity;
            int half = Mathf.Max(1, window / 2);

            for (int i = 0; i < n; i++)
            {
                Vector3 a = line[(i - half + n * 2) % n];
                Vector3 b = line[i];
                Vector3 c = line[(i + half) % n];

                float ab = Vector3.Distance(a, b);
                float bc = Vector3.Distance(b, c);
                float ca = Vector3.Distance(c, a);
                if (ab < 0.001f || bc < 0.001f || ca < 0.001f) continue;

                // Twice the triangle's area from the cross product; zero means collinear.
                float twiceArea = Vector3.Cross(b - a, c - a).magnitude;
                if (twiceArea < 1e-4f) continue;

                float radius = ab * bc * ca / (2f * twiceArea);
                if (radius < tightest) tightest = radius;
            }

            return float.IsPositiveInfinity(tightest) ? float.NaN : tightest;
        }

        public static BakeResult Bake(
            List<Vector3> centreline,
            float maxLateralAccel,
            float brakeAccel,
            float straightSpeedKmh,
            float minSpeedKmh,
            float trackWidthMetres,
            out string failureReason)
        {
            var result = new BakeResult
            {
                ControlPoints = new Vector3[0],
                SpeedProfileKmh = new float[0],
                WidthMetres = new float[0],
                SectorStarts = new float[3] { 0f, 0f, 0f }
            };

            if (centreline == null || centreline.Count < 8)
            {
                failureReason = $"A racing line needs at least 8 points to describe a circuit; got " +
                                $"{centreline?.Count ?? 0}. The tarmac was probably not read as " +
                                "cross-section cells, or the submesh lookup failed.";
                result.FailureReason = failureReason;
                return result;
            }

            // Pinch check, on the RAW centreline rather than the decimated one.
            //
            // A centreline that folds through itself is not a circuit. Nothing else in this method
            // catches it: every individual point is well formed, the loop closes, and the lap has
            // a plausible length -- so a pinching centreline sails through and yields a speed
            // profile full of artefacts that look like corners. Measured at this window (5) on the
            // polar fixture this task originally used: tightest corner 4.51 m.
            //
            // Checked BEFORE decimation, because averaging to 60 control points would smooth the
            // pinch away and hide it.
            float tightest = MinimumCornerRadius(centreline, 5);

            // No measurable corner anywhere means the centreline never turns: a straight line,
            // folded back on itself. That is not a gentle circuit, it is a degenerate input, and it
            // must be refused under its own reason rather than falling through to the pinch check
            // (+Infinity is not below the threshold) or to the length check (a folded line is long).
            if (float.IsNaN(tightest))
            {
                failureReason = "This centreline has no measurable corner anywhere along it -- every " +
                                "point is collinear with its neighbours. A circuit must turn; a straight " +
                                "line folded back on itself is not one. The tarmac extraction almost " +
                                "certainly read a straight strip of geometry instead of a circuit.";
                result.FailureReason = failureReason;
                return result;
            }

            if (tightest < MinimumCornerRadiusMetres)
            {
                failureReason = $"This centreline's tightest corner has a radius of {tightest:F2} m, " +
                                $"below the {MinimumCornerRadiusMetres:F0} m any real circuit has. " +
                                "That is a numerical pinch rather than a corner -- check the " +
                                "tarmac extraction, which usually means the surrounding terrain " +
                                "is being read as road.";
                result.FailureReason = failureReason;
                return result;
            }

            result.ControlPoints = Decimate(centreline, TargetControlPoints);
            var spline = new ArcLengthSpline(result.ControlPoints, true);

            if (!spline.IsValid)
            {
                failureReason = "The decimated control points do not form a usable closed spline. " +
                                "This usually means the extracted centreline is collinear or has " +
                                "duplicated points.";
                result.FailureReason = failureReason;
                return result;
            }

            if (spline.Length < 100f)
            {
                failureReason = $"The centreline measures only {spline.Length:F1} m. A circuit is " +
                                "kilometres long; the tarmac extraction almost certainly failed.";
                result.FailureReason = failureReason;
                return result;
            }

            result.LengthMetres = spline.Length;
            result.SpeedProfileKmh = DeriveSpeedProfile(spline, maxLateralAccel, brakeAccel,
                straightSpeedKmh, minSpeedKmh);
            result.WidthMetres = BuildWidth(spline.Length, trackWidthMetres);
            result.SectorStarts = SplitSectors(spline.Length);
            result.ReferenceLapSeconds = EstimateLapTime(result.SpeedProfileKmh, spline.Length);

            failureReason = null;
            result.Success = true;
            return result;
        }

        /// <summary>
        /// Reduces a dense centreline to a fixed number of control points, always keeping one at
        /// the start so the spline's s = 0 coincides with the line's first point.
        /// </summary>
        private static Vector3[] Decimate(List<Vector3> line, int target)
        {
            if (line.Count <= target)
                return line.ToArray();

            var result = new Vector3[target];
            for (int i = 0; i < target; i++)
                result[i] = line[(int)((long)i * line.Count / target)];

            // Drop consecutive duplicates: a Catmull-Rom knot spacing of zero is undefined.
            var cleaned = new List<Vector3>(result.Length);
            for (int i = 0; i < result.Length; i++)
            {
                if (cleaned.Count == 0 ||
                    Vector3.Distance(cleaned[cleaned.Count - 1], result[i]) > 1e-3f)
                    cleaned.Add(result[i]);
            }

            if (cleaned.Count >= 3) return cleaned.ToArray();
            return result;
        }

        /// <summary>
        /// The physical speed limit at every arc length, then a backward braking pass.
        ///
        /// <para>
        /// Corner speed is v = sqrt(a_lat / kappa), which needs no per-corner tuning: a hairpin is
        /// slow and a straight is fast purely because of its radius. The backward pass then
        /// propagates each corner's limit forward along the approach, so a car is already slowing
        /// on entry. It runs twice so the loop settles everywhere.
        /// </para>
        /// </summary>
        private static float[] DeriveSpeedProfile(ArcLengthSpline spline, float maxLateralAccel,
            float brakeAccel, float straightSpeedKmh, float minSpeedKmh)
        {
            int count = Mathf.Max(SpeedProfileSamplesPerLapMin,
                Mathf.CeilToInt(spline.Length / TrackDataAsset.SpeedProfileResolutionMetres));
            var speeds = new float[count];

            for (int i = 0; i < count; i++)
            {
                float s = spline.Length * i / count;
                float kappa = spline.GetCurvature(s);

                // Infinite radius on a straight gives infinite v, so the straight speed is the
                // natural ceiling and needs no special case.
                float v = kappa <= 1e-6f ? straightSpeedKmh : Mathf.Sqrt(maxLateralAccel / kappa) * 3.6f;
                speeds[i] = Mathf.Clamp(v, minSpeedKmh, straightSpeedKmh);
            }

            float sampleGap = spline.Length / count;
            for (int lap = 0; lap < 2; lap++)
            {
                for (int step = 0; step < count; step++)
                {
                    int i = count - 1 - step;
                    int j = (i + 1) % count;

                    float vNext = speeds[j] * KmhToMs;
                    float reachable = Mathf.Sqrt(vNext * vNext + 2f * brakeAccel * sampleGap);

                    speeds[i] = Mathf.Min(speeds[i], reachable * 3.6f);
                }
            }

            return speeds;
        }

        private static float[] BuildWidth(float length, float width)
        {
            int count = Mathf.Max(2, Mathf.CeilToInt(length / TrackDataAsset.WidthResolutionMetres));
            var widths = new float[count];
            for (int i = 0; i < count; i++) widths[i] = width;
            return widths;
        }

        private static float[] SplitSectors(float length) => new[]
        {
            0f,
            length / 3f,
            length * 2f / 3f
        };

        /// <summary>
        /// Lap time from the speed profile: the sum of time spent in each sample, with the
        /// distance the sample represents. An estimate, not a simulation — it exists so tiers can
        /// be expressed as a fraction of a reference, and a human can override it later.
        /// </summary>
        private static float EstimateLapTime(float[] speedsKmh, float length)
        {
            if (speedsKmh.Length == 0 || length <= 0f) return 0f;

            float total = 0f;
            float sampleGap = length / speedsKmh.Length;
            for (int i = 0; i < speedsKmh.Length; i++)
            {
                float v = Mathf.Max(1f, speedsKmh[i] * KmhToMs);
                total += sampleGap / v;
            }

            return total;
        }
    }
}