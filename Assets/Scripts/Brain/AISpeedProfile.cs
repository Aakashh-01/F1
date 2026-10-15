using UnityEngine;

/// <summary>
/// Solves the speed a car may carry right now so it can still reach every corner ahead of it.
///
/// <para>
/// The racing line already stores a braking-aware speed profile — <c>TrackRacingLineBaker</c>
/// runs a backward pass over the corners at bake time — so the information needed to brake in
/// time is present on the line. The AI simply was not reading it: it took the speed of the one
/// waypoint immediately ahead, about 11 m away. Braking from 240 km/h to 90 km/h needs roughly
/// 42 m, so by the time the AI saw the slow waypoint it was already inside the braking zone and
/// had no option but to saturate the brake and oscillate.
///
/// </para>
///
/// <para>
/// This is deliberately a pure function over plain numbers. It has no dependency on the
/// controller, the sensor, or the scene, so the arithmetic that decides whether a car brakes in
/// time can be tested directly instead of inferred from a race.
/// </para>
/// </summary>
public static class AISpeedProfile
{
    private const float KmhToMs = 1f / 3.6f;

    /// <summary>
    /// Highest speed, in km/h, the car may carry at the origin such that it can still be at each
    /// supplied target speed at that point's distance without exceeding what the brakes can do.
    ///
    /// <param name="targetSpeedsKmh">Target speeds ahead, nearest first. Units km/h.</param>
    /// <param name="distancesMetres">Cumulative distance along the line from the origin to each
    /// of those points. Units m, non-decreasing, starting at or above zero. This is a distance
    /// <i>from where the car is now</i>, not a segment length.</param>
    /// <param name="brakeDecelMs2">Deceleration the car can actually achieve, m/s^2. This is a
    /// real capability, not a wish: see <see cref="MeasureBrakeDecel"/>.</param>
    /// <returns>The allowed speed in km/h. Returns <see cref="float.PositiveInfinity"/> when no
    /// constraint is supplied.</returns>
    public static float SolveAllowedSpeedKmh(
        float[] targetSpeedsKmh,
        float[] distancesMetres,
        float brakeDecelMs2)
    {
        if (targetSpeedsKmh == null || distancesMetres == null) return float.PositiveInfinity;
        return SolveAllowedSpeedKmh(
            targetSpeedsKmh,
            distancesMetres,
            Mathf.Min(targetSpeedsKmh.Length, distancesMetres.Length),
            brakeDecelMs2);
    }

    /// <summary>
    /// As above, but over an explicit prefix of the arrays.
    ///
    /// <para>
    /// This exists because the per-tick caller reuses buffers sized to the largest horizon it
    /// might see, and fills only as many entries as the horizon actually reached. Reading the
    /// whole buffer would treat the unused tail as real constraints — and since those are
    /// zero-filled, a zero target speed at zero distance pins the result to a standstill.
    /// </para>
    /// </summary>
    /// <param name="count">How many leading entries of each array are live. Clamped to the
    /// arrays' lengths; a count of zero returns <see cref="float.PositiveInfinity"/>.</param>
    public static float SolveAllowedSpeedKmh(
        float[] targetSpeedsKmh,
        float[] distancesMetres,
        int count,
        float brakeDecelMs2)
    {
        if (targetSpeedsKmh == null || distancesMetres == null) return float.PositiveInfinity;

        count = Mathf.Clamp(count, 0, Mathf.Min(targetSpeedsKmh.Length, distancesMetres.Length));
        if (count == 0) return float.PositiveInfinity;

        float allowedMs = float.PositiveInfinity;
        float decel = Mathf.Max(0f, brakeDecelMs2);

        for (int i = 0; i < count; i++)
        {
            float distance = Mathf.Max(0f, distancesMetres[i]);
            float limitMs = targetSpeedsKmh[i] * KmhToMs;

            // Speed that can still be shed by the time we reach this point:
            //   v_now^2 <= v_limit^2 + 2 * a * d
            // With no braking capability this degenerates to the limit itself, which is correct:
            // a car that cannot slow can only be travelling as fast as the corner ahead allows.
            float reachableMs = Mathf.Sqrt(limitMs * limitMs + 2f * decel * distance);
            allowedMs = Mathf.Min(allowedMs, reachableMs);
        }

        return allowedMs / KmhToMs;
    }

    /// <summary>
    /// Backward pass over the whole supplied profile, producing the fastest speed at every point
    /// that is still reachable under the brakes. Used for the speed the car should be carrying
    /// at each waypoint, and for drawing the plan.
    ///
    /// <param name="targetSpeedsKmh">Corner speeds along the line, units km/h.</param>
    /// <param name="segmentDistancesMetres">Distance from each point to the NEXT one, units m.
    /// Same length as <paramref name="targetSpeedsKmh"/>; the last entry closes the loop back to
    /// the first.</param>
    /// <param name="brakeDecelMs2">Deceleration the car can achieve, m/s^2.</param>
    /// <param name="startIndex">Index the profile is being read from, so the backward pass wraps
    /// the loop from there rather than from zero.</param>
    public static void SolveForwardProfileKmh(
        float[] targetSpeedsKmh,
        float[] segmentDistancesMetres,
        float brakeDecelMs2,
        int startIndex,
        float[] outputKmh)
    {
        if (targetSpeedsKmh == null || segmentDistancesMetres == null || outputKmh == null) return;

        int count = targetSpeedsKmh.Length;
        if (count == 0) return;
        if (segmentDistancesMetres.Length < count || outputKmh.Length < count) return;

        float decel = Mathf.Max(0f, brakeDecelMs2);
        int origin = ((startIndex % count) + count) % count;

        // Walk backwards around the loop so the closing segment is respected.
        float carriedMs = float.PositiveInfinity;
        for (int step = 0; step < count; step++)
        {
            int i = origin - step;
            i = ((i % count) + count) % count;

            float cornerMs = targetSpeedsKmh[i] * KmhToMs;
            float distance = Mathf.Max(0f, segmentDistancesMetres[i]);

            float allowedMs = Mathf.Sqrt(carriedMs * carriedMs + 2f * decel * distance);
            float resultMs = Mathf.Min(cornerMs, allowedMs);

            outputKmh[i] = resultMs / KmhToMs;
            carriedMs = resultMs;
        }
    }

    /// <summary>
    /// Turns one step of observed deceleration into a braking-capability estimate.
    ///
    /// <para>
    /// The AI has no direct measurement of what the car can do — the tyre model has no
    /// longitudinal half (<c>RaycastWheel</c> hard-zeroes longitudinal slip), so there is no
    /// wheelspin signal to read. What it can do is notice how fast it actually shed speed and
    /// learn from that.
    /// </para>
    ///
    /// <para>
    /// Only a step that began fast enough and genuinely lost speed teaches us anything. Near
    /// standstill the reading is dominated by drivetrain spool-up, and a step that gained speed
    /// says nothing about braking. Both are rejected rather than averaged in.
    /// </para>
    /// </summary>
    /// <param name="currentEstimateMs2">The running estimate to replace.</param>
    /// <param name="speedKmhBeforeStep">Speed at the start of the step — the gate uses this,
    /// because by the end of a long braking run the car may already be too slow to measure.</param>
    /// <param name="speedKmhAfterStep">Speed at the end of the step.</param>
    /// <param name="dtSeconds">Length of the step.</param>
    /// <param name="minimumSpeedKmhForMeasurement">Below this, the reading is not trusted.</param>
    public static float MeasureBrakeDecel(
        float currentEstimateMs2,
        float speedKmhBeforeStep,
        float speedKmhAfterStep,
        float dtSeconds,
        float minimumSpeedKmhForMeasurement)
    {
        if (dtSeconds <= 1e-4f) return currentEstimateMs2;
        if (speedKmhBeforeStep < minimumSpeedKmhForMeasurement) return currentEstimateMs2;
        if (speedKmhAfterStep >= speedKmhBeforeStep) return currentEstimateMs2;

        float decel = (speedKmhBeforeStep - speedKmhAfterStep) / 3.6f / dtSeconds;
        if (decel <= 0f) return currentEstimateMs2;

        // A real F1 car on this physics peaks around 50 m/s^2. Anything far above that is a
        // mis-timed sample, not a discovery.
        const float maxPlausible = 80f;
        return Mathf.Clamp(decel, 0.5f, maxPlausible);
    }
}