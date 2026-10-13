using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using UnityEngine.UI;
using F1.GameFlow;
using F1.Progression;

/// <summary>
/// The hub's panel contract, driven through the real scene and the real flow: NEXT on
/// the car panel swaps to the track panel IN PLACE — the background scene never reloads,
/// the session state moves to TrackSelection, and the showcased car leaves the stage.
/// BACK returns the same way; CONFIRM only fires with a raceable session track; wing
/// setup's back-route lands on the hub with the track panel showing.
///
/// Clicks go through the real buttons' onClick so the self-wiring is under test.
/// </summary>
public class TrackPanelFlowTests
{
    private GameObject _flowObject;
    private GameFlowManager _flow;

    [UnitySetUp]
    public IEnumerator SetUp()
    {
        ProfileIsolation.Begin();
        yield return null;
    }

    [UnityTearDown]
    public IEnumerator TearDown()
    {
        var sceneFlow = GameFlowManager.Instance?.SceneFlow;
        if (sceneFlow != null)
        {
            yield return SceneOpWait.UnloadAllContentBounded(sceneFlow);
            yield return SceneOpWait.UnloadSceneBounded(sceneFlow.CurrentFlowScene);
        }

        if (_flowObject != null) Object.DestroyImmediate(_flowObject);

        ProfileIsolation.End();
    }

    private IEnumerator EnterGarage()
    {
        _flowObject = new GameObject("TrackPanelTest_Flow");
        _flowObject.AddComponent<GameFlowManager>();
        _flowObject.AddComponent<FlowRoot>();
        _flow = GameFlowManager.Instance;

        _flow.SceneFlow?.AdoptFlowScene(null);
        yield return WaitUntil(() => _flow.SceneFlow == null || !_flow.SceneFlow.IsBusy, 20f);
        yield return null;

        _flow.GoToLobby();
        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.Lobby, 20f);
        yield return WaitUntil(() => _flow.LobbyScreenInstance != null, 20f);
        yield return WaitUntil(() => _flow.SelectedCar != null, 10f);
        yield return null;
    }

    private IEnumerator PressContinueAndSettleOnTheTrackPanel()
    {
        FindCarPanelButton("ContinueButton").onClick.Invoke();
        yield return WaitUntil(
            () => _flow.CurrentScreen == GameFlowManager.GameScreen.TrackSelection, 10f);
        yield return null;
    }

    private static FlowSceneHost Host(GameFlowManager.GameScreen screen)
    {
        foreach (var host in Object.FindObjectsByType<FlowSceneHost>(
                     FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (host.HostedScreen == screen)
                return host;
        return null;
    }

    private static Button FindCarPanelButton(string name)
    {
        var screen = Host(GameFlowManager.GameScreen.Lobby).ScreenInstance as Component;
        Assert.IsNotNull(screen, "No car panel is hosted.");
        var button = screen.transform.Find(name)?.GetComponent<Button>();
        Assert.IsNotNull(button, $"The car panel has no '{name}' button.");
        return button;
    }

    private static Button FindTrackPanelButton(string name)
    {
        var screen = Host(GameFlowManager.GameScreen.TrackSelection).ScreenInstance as Component;
        Assert.IsNotNull(screen, "No track panel is hosted.");
        var button = screen.transform.Find(name)?.GetComponent<Button>();
        Assert.IsNotNull(button, $"The track panel has no '{name}' button.");
        return button;
    }

    private static IEnumerator WaitUntil(System.Func<bool> condition, float timeoutSeconds)
    {
        float deadline = Time.realtimeSinceStartup + timeoutSeconds;
        while (!condition() && Time.realtimeSinceStartup < deadline)
            yield return null;
    }

    [UnityTest]
    public IEnumerator Continue_SwapsToTheTrackPanelInTheSameScene()
    {
        yield return EnterGarage();

        yield return PressContinueAndSettleOnTheTrackPanel();

        Assert.AreEqual(FlowSceneNames.Lobby, SceneManager.GetActiveScene().name,
            "Track selection must not reload the scene; the hub background persists.");
        Assert.AreEqual(GameFlowState.TrackSelection, _flow.CurrentFlowState);
        Assert.IsTrue(Host(GameFlowManager.GameScreen.TrackSelection).gameObject.activeSelf,
            "The track panel host should be on stage.");
        Assert.IsFalse(Host(GameFlowManager.GameScreen.Lobby).gameObject.activeSelf,
            "The car panel host should be off stage.");
        Assert.IsNotNull(_flow.SelectedTrack,
            "Focusing the first raceable circuit should have selected it.");
    }

    [UnityTest]
    public IEnumerator Continue_TakesTheShowcaseCarOffStage()
    {
        yield return EnterGarage();
        var swapper = Object.FindAnyObjectByType<ShowcaseCarSwapper>();
        Assert.IsNotNull(swapper);
        Assert.IsTrue(swapper.gameObject.activeInHierarchy, "The car should be on stage in the garage.");

        yield return PressContinueAndSettleOnTheTrackPanel();

        Assert.IsFalse(swapper.gameObject.activeInHierarchy,
            "The reference shows the turntable empty during track selection.");
    }

    [UnityTest]
    public IEnumerator Back_ReturnsToTheCarPanelInTheSameScene()
    {
        yield return EnterGarage();
        yield return PressContinueAndSettleOnTheTrackPanel();

        FindTrackPanelButton("BackButton").onClick.Invoke();
        yield return WaitUntil(
            () => _flow.CurrentScreen == GameFlowManager.GameScreen.Lobby, 10f);
        yield return null;

        Assert.AreEqual(FlowSceneNames.Lobby, SceneManager.GetActiveScene().name);
        Assert.IsTrue(Host(GameFlowManager.GameScreen.Lobby).gameObject.activeSelf);
        Assert.IsFalse(Host(GameFlowManager.GameScreen.TrackSelection).gameObject.activeSelf);
        Assert.IsNotNull(_flow.SelectedCar, "The session car must survive the round trip.");
    }

    [UnityTest]
    public IEnumerator FocusingAnUnraceableTrack_ParksConfirmAndKeepsTheSessionTrack()
    {
        yield return EnterGarage();
        yield return PressContinueAndSettleOnTheTrackPanel();

        var sessionTrack = _flow.SelectedTrack;
        Assert.IsNotNull(sessionTrack);

        var confirm = FindTrackPanelButton("ConfirmButton");
        int cycleLength = ProgressionRegistry.GetOwnedTracks().Count
            + ProgressionRegistry.GetLockedTracks().Count;

        // An unraceable focus is identifiable from the outside: Confirm dark AND the
        // session track unchanged. The cycle is guaranteed to contain at least one (Spa
        // is owned but shares Track_01 and has no scene of its own; locked circuits are
        // refused by SelectTrack's ownership guard), but not at a fixed index, so cycle
        // until one turns up.
        bool found = false;
        for (int i = 0; i < cycleLength && !found; i++)
        {
            FindTrackPanelButton("NextButton").onClick.Invoke();
            yield return null;
            found = !confirm.interactable && _flow.SelectedTrack == sessionTrack;
        }

        Assert.IsTrue(found,
            "The cycle should contain an unraceable circuit (owned Spa has no scene).");
        Assert.IsNotNull(_flow.SelectedTrack, "The session track must never drop to null.");
        Assert.IsFalse(confirm.interactable,
            "An unraceable focus must not let Confirm promise a race the flow refused.");
    }

    [UnityTest]
    public IEnumerator Confirm_GoesToWingSetup()
    {
        yield return EnterGarage();
        yield return PressContinueAndSettleOnTheTrackPanel();

        FindTrackPanelButton("ConfirmButton").onClick.Invoke();
        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.WingSetup, 20f);
        yield return WaitUntil(
            () => _flow.CurrentScreen == GameFlowManager.GameScreen.WingSetup, 10f);

        Assert.AreEqual(FlowSceneNames.WingSetup, SceneManager.GetActiveScene().name);
        Assert.IsNotNull(_flow.SelectedTrack);
    }

    [UnityTest]
    public IEnumerator ReturningFromWingSetup_LandsOnTheHubTrackPanel()
    {
        yield return EnterGarage();
        yield return PressContinueAndSettleOnTheTrackPanel();
        var selected = _flow.SelectedTrack;
        FindTrackPanelButton("ConfirmButton").onClick.Invoke();
        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.WingSetup, 20f);

        // This is exactly what WingSetupSceneController.OnBackPressed does.
        _flow.GoToTrackSelection();
        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.Lobby, 20f);
        yield return WaitUntil(
            () => _flow.CurrentScreen == GameFlowManager.GameScreen.TrackSelection, 10f);
        yield return null;

        Assert.AreEqual(FlowSceneNames.Lobby, SceneManager.GetActiveScene().name,
            "Wing back must land on the hub scene.");
        Assert.AreEqual(GameFlowManager.GameScreen.TrackSelection, _flow.CurrentScreen,
            "Wing back must show the track panel, not the car panel.");
        Assert.AreEqual(selected.TrackId, _flow.SelectedTrack.TrackId,
            "The session track must survive the round trip.");
        Assert.IsTrue(Host(GameFlowManager.GameScreen.TrackSelection).gameObject.activeSelf);
    }

    [UnityTest]
    public IEnumerator CycleNext_WrapsThroughEveryTrack()
    {
        yield return EnterGarage();
        yield return PressContinueAndSettleOnTheTrackPanel();

        int cycleLength = ProgressionRegistry.GetOwnedTracks().Count
            + ProgressionRegistry.GetLockedTracks().Count;
        Assert.Greater(cycleLength, 1, "The cycle needs at least two circuits to wrap.");

        // The centre slot's name is the visible focus. The builder authors that child as
        // SlotCentre/Name — the same Find-by-name grammar the button helpers use.
        var screen = Host(GameFlowManager.GameScreen.TrackSelection).ScreenInstance as Component;
        Assert.IsNotNull(screen, "No track panel is hosted.");
        var centreName = screen.transform.Find("SlotCentre/Name")?.GetComponent<TMP_Text>();
        Assert.IsNotNull(centreName, "The track panel must expose the centre slot's name.");

        string start = centreName.text;
        var seen = new HashSet<string> { start };
        for (int i = 0; i < cycleLength; i++)
        {
            FindTrackPanelButton("NextButton").onClick.Invoke();
            yield return null;
            seen.Add(centreName.text);
        }

        Assert.AreEqual(cycleLength, seen.Count,
            "A full cycle must visit every circuit exactly once.");
        Assert.AreEqual(start, centreName.text,
            "After a full cycle the focus must be back where it started.");
        Assert.IsNotNull(_flow.SelectedTrack, "The session track must survive a full cycle.");
    }
}
