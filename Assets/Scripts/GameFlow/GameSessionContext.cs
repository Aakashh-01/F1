using System;
using F1.GameData;
using WingType = F1.GameData.WingType;

namespace F1.GameFlow
{
    /// <summary>
    /// Runtime contract for one lobby-to-race session.
    /// It deliberately does not load scenes or own UI.
    /// </summary>
    public enum SessionType
    {
        None,
        Qualifying,
        Race
    }

    public sealed class GameSessionContext
    {
        public GameFlowState FlowState { get; internal set; } = GameFlowState.Loading;
        public SessionType SessionType { get; internal set; } = SessionType.None;

        public CarDefinition SelectedCar { get; private set; }
        public TrackDefinition SelectedTrack { get; private set; }
        public WingType SelectedWing { get; private set; } = WingType.HighDownforce;
        public QualifyingRecord Qualifying { get; } = new QualifyingRecord();
        public int RaceGridPosition { get; internal set; } = 1;

        public bool IsSelectionComplete =>
            SelectedCar != null && SelectedTrack != null;

        public bool CanStartQualifying => IsSelectionComplete;

        public bool CanStartRace => Qualifying.CanStartRace;

        public bool HasGhostForNextAttempt => Qualifying.HasValidGhost;

        public bool ShouldSpawnGhostForCurrentAttempt =>
            Qualifying.AttemptCount > 0 && Qualifying.HasValidGhost;

        public void SetSelection(CarDefinition car, TrackDefinition track, WingType wing)
        {
            bool changed = SelectedCar != car || SelectedTrack != track || SelectedWing != wing;
            SelectedCar = car;
            SelectedTrack = track;
            SelectedWing = wing;

            if (changed)
                ResetSessionData();
        }

        public void BeginQualifyingSession()
        {
            Qualifying.BeginSession(SelectedTrack?.TrackId, SelectedCar?.CarId, SelectedWing);
            SessionType = F1.GameFlow.SessionType.Qualifying;
        }

        public void BeginQualifyingRetry()
        {
            SessionType = F1.GameFlow.SessionType.Qualifying;
        }

        public void RestoreSavedBestGhost(float bestLapTime, string ghostData)
        {
            if (bestLapTime <= 0f || string.IsNullOrWhiteSpace(ghostData))
                return;

            Qualifying.RestoreSavedBestGhost(bestLapTime, ghostData);
        }

        public bool RecordQualifyingLap(float lapTime, string ghostData, bool countForRanking)
        {
            return Qualifying.RecordLap(lapTime, ghostData, countForRanking);
        }

        /// <summary>
        /// Records the grid slot the player's qualifying result earned.
        ///
        /// One-based, because a grid is counted from one and because the race HUD's
        /// <c>SetRaceInfo</c> contract is already one-based. The grid manager counts from
        /// zero; that conversion belongs where the shell hands the slot over, not here,
        /// where the value is stored and reported.
        ///
        /// Clamped at pole: there is no such thing as a grid position of zero, and a bad
        /// benchmark must not be able to produce one.
        /// </summary>
        internal void SetRaceGridPosition(int position)
        {
            RaceGridPosition = position < 1 ? 1 : position;
        }

        public void ResetSessionData()
        {
            Qualifying.Reset(SelectedTrack?.TrackId, SelectedCar?.CarId, SelectedWing);
            RaceGridPosition = 1;
        }
    }

    /// <summary>
    /// Best-lap state for the current qualifying session. Attempt count is local
    /// to the session; ranking data is persisted only by the flow manager.
    /// </summary>
    [Serializable]
    public sealed class QualifyingRecord
    {
        public string TrackId { get; private set; }
        public string CarId { get; private set; }
        public WingType Wing { get; private set; }
        public int AttemptCount { get; private set; }
        public float BestLapTime { get; private set; }
        public string BestGhostData { get; private set; }
        public bool IsCompleted { get; private set; }
        public bool CountedForRanking { get; private set; }

        public bool CanStartRace => IsCompleted && BestLapTime > 0f;
        public bool HasValidGhost => CountedForRanking && BestLapTime > 0f && !string.IsNullOrWhiteSpace(BestGhostData);

        public void BeginSession(string trackId, string carId, WingType wing)
        {
            Reset(trackId, carId, wing);
        }

        public void RestoreSavedBestGhost(float bestLapTime, string ghostData)
        {
            if (bestLapTime <= 0f || string.IsNullOrWhiteSpace(ghostData))
                return;

            BestLapTime = bestLapTime;
            BestGhostData = ghostData;
            CountedForRanking = true;
        }

        public bool RecordLap(float lapTime, string ghostData, bool countForRanking)
        {
            if (lapTime <= 0f)
                return false;

            AttemptCount++;
            IsCompleted = true;
            CountedForRanking |= countForRanking;

            bool isBest = BestLapTime <= 0f || lapTime < BestLapTime;
            if (!isBest)
                return false;

            BestLapTime = lapTime;
            BestGhostData = countForRanking ? ghostData : null;
            return true;
        }

        public void Reset(string trackId, string carId, WingType wing)
        {
            TrackId = trackId;
            CarId = carId;
            Wing = wing;
            AttemptCount = 0;
            BestLapTime = 0f;
            BestGhostData = null;
            IsCompleted = false;
            CountedForRanking = false;
        }
    }

    public enum GameFlowState
    {
        Loading,
        CarSelection,
        TrackSelection,
        WingSetup,
        PreRace,
        Race,
        Results
    }
}