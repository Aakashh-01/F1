using NUnit.Framework;
using UnityEngine;
using F1.TrackData;

public class TrackRuntimeTests
{
    /// <summary>
    /// A synthetic closed circuit, built through the TrackData authoring seam exactly as an editor
    /// bake would. Deliberately not Track_01's shape, so nothing here can pass by accident on one
    /// particular mesh.
    /// </summary>
    private static TrackData BuildTrack(float straightKmh = 240f)
    {
        var data = ScriptableObject.CreateInstance<TrackData>();
        data.SetControlPoints(new[]
        {
            new Vector3(0f, 0f, 0f),
            new Vector3(400f, 0f, 0f),
            new Vector3(800f, 0f, 0f),
            new Vector3(800f, 0f, 400f),
            new Vector3(400f, 0f, 700f),
            new Vector3(0f, 0f, 500f),
        }, closed: true);

        data.WriteUniformProfile(straightKmh, widthMetres: 16f);
        return data;
    }

    [Test]
    public void ARuntimeFromAValidAssetIsUsable()
    {
        var track = new TrackRuntime(BuildTrack());

        Assert.IsTrue(track.IsValid, "A track with six control points is a track.");
        Assert.Greater(track.Length, 2000f, $"A circuit of this size should be kilometres long, got {track.Length:F0} m.");
        Assert.That(track.GetPosition(0f).x, Is.EqualTo(0f).Within(0.05f));
    }

    [Test]
    public void ProjectionRecoversTheArcLengthItWasGiven()
    {
        TrackRuntime track = new TrackRuntime(BuildTrack());

        for (int i = 1; i < 60; i++)
        {
            float s = track.Length * i / 60f;
            Vector3 world = track.GetPosition(s);

            bool ok = track.ProjectToArcLength(world, out float recovered, out float lateral);
            Assert.IsTrue(ok, $"Projection failed at s={s:F1} m on a valid track.");
            Assert.That(Mathf.Abs(recovered - s), Is.LessThan(0.5f),
                $"Placed at s={s:F3} m, projected to s={recovered:F3} m. Arc length must round-trip " +
                "to well under half a metre, or every downstream distance is wrong.");
            Assert.That(Mathf.Abs(lateral), Is.LessThan(0.1f),
                $"A point exactly on the centreline must project to ~0 lateral offset, got {lateral:F3} m.");
        }
    }

    [Test]
    public void ProjectionReportsTheSignedLateralOffset()
    {
        TrackRuntime track = new TrackRuntime(BuildTrack());
        float s = track.Length * 0.2f;
        Vector3 onLine = track.GetPosition(s);
        Vector3 right = track.GetNormal(s);

        bool okR = track.ProjectToArcLength(onLine + right * 4f, out float sR, out float offsetRight);
        bool okL = track.ProjectToArcLength(onLine - right * 4f, out float sL, out float offsetLeft);

        Assert.IsTrue(okR && okL, "Projection must succeed either side of the line.");
        Assert.That(offsetRight, Is.GreaterThan(3.5f).And.LessThan(4.5f),
            $"4 m to the right of the normal must read as ~+4 m, got {offsetRight:F3} m.");
        Assert.That(offsetLeft, Is.LessThan(-3.5f).And.GreaterThan(-4.5f),
            $"4 m to the left of the normal must read as ~-4 m, got {offsetLeft:F3} m.");
        Assert.That(Mathf.Abs(sR - sL), Is.LessThan(0.5f),
            "Both offsets describe the same point on the track, so they must share an arc length.");
    }

    [Test]
    public void ProjectionWrapsCleanlyAcrossTheStartFinishSeam()
    {
        TrackRuntime track = new TrackRuntime(BuildTrack());

        Vector3 justBefore = track.GetPosition(track.Length - 2f);
        Vector3 justAfter = track.GetPosition(0f);

        track.ProjectToArcLength(justBefore, out float sA, out _);
        track.ProjectToArcLength(justAfter, out float sB, out _);

        // The two points are two metres apart on a closed loop. Their arc lengths must therefore
        // differ by two metres modulo the lap -- NOT by the whole lap, which is what a projection
        // that fails to wrap produces.
        float gap = sA - sB;
        gap = Mathf.Min(gap, track.Length - gap);

        Assert.That(sA, Is.GreaterThan(track.Length * 0.9f),
            "A sample just before the seam should have a high arc length.");
        Assert.That(gap, Is.LessThan(3f),
            $"Samples 2 m apart on the track projected {gap:F2} m apart in arc length. " +
            "The seam is where s wraps, and a projection that does not wrap here teleports the car.");
    }

    [Test]
    public void ProjectionStaysOnTheCorrectBranchWhereTheTrackPassesNearItself()
    {
        // A coarse scan picks the globally nearest coarse sample. That is wrong on a track that
        // runs close to itself -- a chicane, or a street circuit's two parallel sections -- where
        // the nearest point in space is not the point you are actually on the lap.
        //
        // The refinement below must recover the true branch rather than converging on the nearby
        // one, because getting this wrong puts the car on the wrong part of the track with the
        // right-looking numbers.
        var data = ScriptableObject.CreateInstance<TrackData>();
        data.SetControlPoints(new[]
        {
            new Vector3(0f, 0f, 0f),
            new Vector3(300f, 0f, 0f),
            new Vector3(600f, 0f, 0f),
            new Vector3(600f, 0f, 12f),   // comes back past itself, 12 m away
            new Vector3(300f, 0f, 12f),
            new Vector3(0f, 0f, 12f),
        }, closed: true);
        data.WriteUniformProfile(240f, 16f);

        var track = new TrackRuntime(data);
        Assert.IsTrue(track.IsValid);

        // Sit on the FIRST of the two parallel sections and demand the early arc length.
        float earlyS = track.Length * 0.1f;
        Vector3 world = track.GetPosition(earlyS);

        Assert.IsTrue(track.ProjectToArcLength(world, out float recovered, out float lateral),
            "Projection must succeed on a self-approaching track.");

        float gap = Mathf.Abs(recovered - earlyS);
        if (gap > track.Length * 0.5f) gap = track.Length - gap;

        Assert.Less(gap, 5f,
            $"Projected s={recovered:F1} but the point sits at s={earlyS:F1} (lap {track.Length:F0} m, " +
            "two sections 12 m apart). Projection jumped to the neighbouring section of the track.");
    }

    [Test]
    public void SpeedAndWidthAreReadAsStoredProfiles()
    {
        TrackRuntime track = new TrackRuntime(BuildTrack(straightKmh: 187f));

        for (int i = 0; i < 20; i++)
        {
            float s = track.Length * i / 20f;
            Assert.That(track.GetSpeedLimitKmh(s), Is.EqualTo(187f).Within(0.01f),
                $"Speed limit at s={s:F1} m should be the stored 187 km/h.");
            Assert.That(track.GetWidth(s), Is.EqualTo(16f).Within(0.01f));
        }
    }

    [Test]
    public void SectorLookupCoversEveryPointOfTheLap()
    {
        TrackRuntime track = new TrackRuntime(BuildTrack());

        int[] counts = new int[3];
        const int samples = 300;
        for (int i = 0; i < samples; i++)
            counts[(int)track.GetSector(track.Length * i / samples)]++;

        Assert.That(counts[0], Is.GreaterThan(0), "Sector 1 must be reachable.");
        Assert.That(counts[1], Is.GreaterThan(0), "Sector 2 must be reachable.");
        Assert.That(counts[2], Is.GreaterThan(0),
            "Sector 3 must be reachable; a track whose final third is unreachable breaks sector timing.");
        Assert.That(counts[0] + counts[1] + counts[2], Is.EqualTo(samples),
            "Every point on the lap must fall in exactly one sector.");
    }

    [Test]
    public void ARuntimeFromAnEmptyAssetRefusesToPretendItIsUsable()
    {
        // Review Focus: a car with no usable line must stop, not drive blind. This is the gate
        // that lets AIDriverController disable itself instead of guessing a direction.
        var empty = ScriptableObject.CreateInstance<TrackData>();
        var track = new TrackRuntime(empty);

        Assert.IsFalse(track.IsValid, "A TrackData with no control points is not a track.");
        Assert.That(track.Length, Is.EqualTo(0f).Within(0.001f));

        Assert.IsFalse(track.ProjectToArcLength(Vector3.one, out float s, out float lateral),
            "Projection on an invalid track must report failure rather than return a value.");
        Assert.That(s, Is.EqualTo(0f));
        Assert.That(lateral, Is.EqualTo(0f));
    }

    [Test]
    public void ARuntimeFromTooFewControlPointsIsInvalidRatherThanThrowing()
    {
        var data = ScriptableObject.CreateInstance<TrackData>();
        data.SetControlPoints(new[] { Vector3.zero, Vector3.one }, closed: true);
        data.WriteUniformProfile(240f, 16f);

        var track = new TrackRuntime(data);
        Assert.IsFalse(track.IsValid, "Two control points cannot describe a closed circuit.");
    }

    [Test]
    public void ARuntimeFromNullIsInvalidRatherThanThrowing()
    {
        var track = new TrackRuntime(null);
        Assert.IsFalse(track.IsValid, "A null TrackData is not a track, and must not throw in Awake.");
    }
}