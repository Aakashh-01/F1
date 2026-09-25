using F1.GameData;
using F1.GameFlow;

/// <summary>
/// A <see cref="QualifyingScreen"/> that records what the HUD is told instead of
/// rendering it.
///
/// The real <c>QualifyingScreenImpl</c> needs a prefab with wired TMP labels, and there
/// is no qualifying HUD prefab in the project yet — it arrives with the pre-race shell in
/// S4. Asserting on the *screen contract* rather than on text in a TMP label is also the
/// stronger test: it verifies the binder talks to the screen through the interface the
/// flow resolves, which is the part that can silently regress.
///
/// Kept in its own file because it is a MonoBehaviour, and Unity requires a component
/// class to live in a file named after it. In the global namespace to match the rest of
/// the test assembly.
/// </summary>
public class RecordingQualifyingScreen : QualifyingScreen
{
    public int LapTimePushCount { get; private set; }
    public float LastCurrentLapTime { get; private set; } = -1f;
    public float LastBestLapTime { get; private set; } = -1f;

    /// <summary>Which of the two qualifying states the screen was last put into.</summary>
    public bool IsShowingResults { get; private set; }
    public int DrivingStatePushCount { get; private set; }
    public int ResultsStatePushCount { get; private set; }
    public float LastResultsLapTime { get; private set; } = -1f;

    public override void UpdateLapTime(float currentLapTime, float bestLapTime)
    {
        LapTimePushCount++;
        LastCurrentLapTime = currentLapTime;
        LastBestLapTime = bestLapTime;
    }

    public override void ShowDrivingState()
    {
        IsShowingResults = false;
        DrivingStatePushCount++;
    }

    public override void ShowResultsState(float lapTime)
    {
        IsShowingResults = true;
        ResultsStatePushCount++;
        LastResultsLapTime = lapTime;
    }

    public override void SetGhostData(string ghostJson) { }
    public override void UpdateSectorTimes(float[] sectorTimes, SectorColor[] sectorColors) { }
    public override void SetTrackInfo(TrackDefinition track) { }
}
