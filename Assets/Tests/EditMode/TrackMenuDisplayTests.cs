using System.Linq;
using NUnit.Framework;
using UnityEngine;
using F1.GameData;

/// <summary>
/// Guards the one-shot authoring of the track panel's display data. Like the car
/// display stats, the values are showroom placeholders, but an empty difficulty cell
/// on every card would read as broken rather than placeholder.
/// </summary>
public class TrackMenuDisplayTests
{
    private static TrackDefinition[] LoadTracks() =>
        Resources.LoadAll<TrackDefinition>("GameData/Tracks")
            .OrderBy(t => t.name)
            .ToArray();

    [Test]
    public void EveryTrack_HasADifficultyLabelAndCoreStats()
    {
        var tracks = LoadTracks();
        Assert.GreaterOrEqual(tracks.Length, 5, "Expected the five circuit assets to be present.");

        foreach (var track in tracks)
        {
            Assert.IsFalse(string.IsNullOrWhiteSpace(track.DifficultyLabel),
                $"'{track.name}' has no difficulty label; run Tools > Garage > Author Track Menu Display.");
            Assert.Greater(track.TrackLengthMeters, 0f, $"'{track.name}' has no length.");
            Assert.Greater(track.StandardRaceLaps, 0, $"'{track.name}' has no lap count.");
        }
    }

    [Test]
    public void DifficultyLabels_AreUppercaseShortLabels()
    {
        foreach (var track in LoadTracks())
            StringAssert.IsMatch(@"^[A-Z]+$", track.DifficultyLabel,
                $"'{track.name}' difficulty should be a short uppercase label like MEDIUM.");
    }
}
