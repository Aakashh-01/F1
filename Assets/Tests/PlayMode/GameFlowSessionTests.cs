using NUnit.Framework;
using UnityEngine;
using F1.GameData;
using F1.GameFlow;

public class GameFlowSessionTests
{
    [Test]
    public void SavedGhostBaseline_DoesNotSpawnOnFirstAttempt()
    {
        var context = CreateContext();
        context.BeginQualifyingSession();
        context.RestoreSavedBestGhost(88f, "saved-ghost");

        Assert.IsTrue(context.HasGhostForNextAttempt);
        Assert.IsFalse(context.ShouldSpawnGhostForCurrentAttempt);
        Assert.IsFalse(context.CanStartRace);

        context.RecordQualifyingLap(90f, "first-lap", true);
        Assert.IsTrue(context.ShouldSpawnGhostForCurrentAttempt);

        Object.DestroyImmediate(context.SelectedCar);
        Object.DestroyImmediate(context.SelectedTrack);
    }


    [Test]
    public void FirstQualifyingAttempt_IsPlayerOnlyAndCannotStartRace()
    {
        var context = CreateContext();
        context.BeginQualifyingSession();

        Assert.AreEqual(0, context.Qualifying.AttemptCount);
        Assert.IsFalse(context.HasGhostForNextAttempt);
        Assert.IsFalse(context.CanStartRace);

        Object.DestroyImmediate(context.SelectedCar);
        Object.DestroyImmediate(context.SelectedTrack);
    }

    [Test]
    public void QualifyingLap_EnablesRaceAndRetryHasGhost()
    {
        var context = CreateContext();
        context.BeginQualifyingSession();

        bool isBest = context.RecordQualifyingLap(90f, "owned-ghost", true);

        Assert.IsTrue(isBest);
        Assert.AreEqual(1, context.Qualifying.AttemptCount);
        Assert.IsTrue(context.CanStartRace);
        Assert.IsTrue(context.HasGhostForNextAttempt);

        context.BeginQualifyingRetry();
        Assert.AreEqual(1, context.Qualifying.AttemptCount);
        Assert.IsTrue(context.HasGhostForNextAttempt);

        Object.DestroyImmediate(context.SelectedCar);
        Object.DestroyImmediate(context.SelectedTrack);
    }

    [Test]
    public void NewBestWithoutGhostData_ClearsPreviousGhost()
    {
        var context = CreateContext();
        context.BeginQualifyingSession();
        context.RestoreSavedBestGhost(88f, "saved-ghost");
        context.RecordQualifyingLap(87f, null, true);

        Assert.AreEqual(87f, context.Qualifying.BestLapTime, 0.001f);
        Assert.IsFalse(context.HasGhostForNextAttempt);

        Object.DestroyImmediate(context.SelectedCar);
        Object.DestroyImmediate(context.SelectedTrack);
    }

    [Test]
    public void SlowerQualifyingLap_DoesNotReplaceBestGhost()
    {
        var context = CreateContext();
        context.BeginQualifyingSession();
        context.RecordQualifyingLap(90f, "best-ghost", true);

        bool improved = context.RecordQualifyingLap(92f, "slower-ghost", true);

        Assert.IsFalse(improved);
        Assert.AreEqual(90f, context.Qualifying.BestLapTime, 0.001f);
        Assert.AreEqual("best-ghost", context.Qualifying.BestGhostData);

        Object.DestroyImmediate(context.SelectedCar);
        Object.DestroyImmediate(context.SelectedTrack);
    }

    [Test]
    public void RentedQualifyingLap_AllowsRaceButDoesNotCreateGhost()
    {
        var context = CreateContext();
        context.BeginQualifyingSession();
        context.RecordQualifyingLap(88f, "rental-ghost", false);

        Assert.IsTrue(context.CanStartRace);
        Assert.IsFalse(context.HasGhostForNextAttempt);
        Assert.IsNull(context.Qualifying.BestGhostData);

        Object.DestroyImmediate(context.SelectedCar);
        Object.DestroyImmediate(context.SelectedTrack);
    }

    [Test]
    public void ChangingSelection_ResetsQualifyingRecord()
    {
        var context = CreateContext();
        context.BeginQualifyingSession();
        context.RecordQualifyingLap(90f, "ghost", true);

        var nextCar = ScriptableObject.CreateInstance<CarDefinition>();
        context.SetSelection(nextCar, context.SelectedTrack, context.SelectedWing);

        Assert.AreEqual(0, context.Qualifying.AttemptCount);
        Assert.AreEqual(0f, context.Qualifying.BestLapTime);
        Assert.IsFalse(context.CanStartRace);

        Object.DestroyImmediate(nextCar);
        Object.DestroyImmediate(context.SelectedCar);
        Object.DestroyImmediate(context.SelectedTrack);
    }

    private static GameSessionContext CreateContext()
    {
        var context = new GameSessionContext();
        var car = ScriptableObject.CreateInstance<CarDefinition>();
        var track = ScriptableObject.CreateInstance<TrackDefinition>();
        context.SetSelection(car, track, WingType.HighDownforce);
        return context;
    }
}