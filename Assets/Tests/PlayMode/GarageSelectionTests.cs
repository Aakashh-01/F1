using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using UnityEngine.UI;
using F1.GameData;
using F1.GameFlow;
using F1.Progression;

/// <summary>
/// The garage's selection contract, driven through the real scene and the real flow:
/// the displayed car IS the selection. Cycling focus onto a usable car makes it the
/// session car immediately; cycling onto a locked car changes nothing in the session and
/// parks Continue.
///
/// Clicks go through the real buttons' onClick, so the impl's self-wiring, the screen
/// events and the controller's handling are all under test — raising the screen's events
/// directly would skip exactly the wiring a builder mistake leaves out.
///
/// The swapper's <c>ShownCar</c> is the observation point for what is on display, and
/// <c>GameFlowManager.SelectedCar</c> for what the session carries.
/// </summary>
public class GarageSelectionTests
{
    private GameObject _flowObject;
    private GameFlowManager _flow;

    [UnitySetUp]
    public IEnumerator ProfileSetUp()
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

    /// <summary>
    /// Boots the flow the way the game does and navigates to the garage, which is now
    /// what FlowSceneNames.Lobby names.
    /// </summary>
    private IEnumerator EnterGarage()
    {
        _flowObject = new GameObject("GarageTest_Flow");
        _flowObject.AddComponent<GameFlowManager>();
        _flowObject.AddComponent<FlowRoot>();
        _flow = GameFlowManager.Instance;

        // The flow is born in the test runner's scene, which Unity refuses to unload.
        _flow.SceneFlow?.AdoptFlowScene(null);
        yield return WaitUntil(() => _flow.SceneFlow == null || !_flow.SceneFlow.IsBusy, 20f);
        yield return null;

        _flow.GoToLobby();
        yield return WaitUntil(
            () => SceneManager.GetActiveScene().name == FlowSceneNames.Lobby, 20f);
        yield return WaitUntil(() => _flow.LobbyScreenInstance != null, 20f);
        Assert.AreEqual(FlowSceneNames.Lobby, SceneManager.GetActiveScene().name,
            "The flow did not land in the garage scene.");

        // The controller focuses on Start, the frame after the load completes.
        yield return WaitUntil(() => _flow.SelectedCar != null, 10f);
        yield return null;
    }

    private static Button FindButton(string name)
    {
        // The screen resolves through the interface; the concrete impl is always a
        // component on the hosted prefab's root, which is where the buttons live.
        var screen = GameFlowManager.Instance?.LobbyScreenInstance as Component;
        Assert.IsNotNull(screen, "No lobby screen instance is hosted.");
        var button = screen.transform.Find(name)?.GetComponent<Button>();
        Assert.IsNotNull(button, $"The garage screen prefab has no '{name}' button.");
        return button;
    }

    private static ShowcaseCarSwapper Swapper()
    {
        var swapper = Object.FindAnyObjectByType<ShowcaseCarSwapper>();
        Assert.IsNotNull(swapper, "The garage scene carries no ShowcaseCarSwapper.");
        return swapper;
    }

    private static IEnumerator WaitUntil(System.Func<bool> condition, float timeoutSeconds)
    {
        float deadline = Time.realtimeSinceStartup + timeoutSeconds;
        while (!condition() && Time.realtimeSinceStartup < deadline)
            yield return null;
    }

    [UnityTest]
    public IEnumerator EnteringTheGarage_FocusesAndSelectsTheStarterCar()
    {
        yield return EnterGarage();

        // A fresh profile owns exactly the starter car, so it heads the cycle and entry
        // focus both displays it and makes it the session car.
        var shown = Swapper().ShownCar;
        Assert.IsNotNull(shown, "Entry focus displayed no car.");
        Assert.AreEqual(_flow.SelectedCar.CarId, shown.CarId,
            "The displayed car and the session car diverged on entry.");
        Assert.AreEqual(GameDataRegistry.FreeCars[0].CarId, shown.CarId,
            "A fresh profile should land on the free starter car.");
    }

    [UnityTest]
    public IEnumerator CyclingNext_SixTimesWrapsThroughEveryCar()
    {
        yield return EnterGarage();

        var next = FindButton("NextButton");
        var start = Swapper().ShownCar;
        Assert.IsNotNull(start);

        var seen = new HashSet<string> { start.CarId };
        for (int i = 0; i < 6; i++)
        {
            next.onClick.Invoke();
            yield return null;
            var shown = Swapper().ShownCar;
            Assert.IsNotNull(shown, $"Step {i + 1}: nothing on display.");
            seen.Add(shown.CarId);
        }

        Assert.AreEqual(GameDataRegistry.AllCars.Count, seen.Count,
            "Six next-presses should have visited every registered car exactly once.");
        Assert.AreEqual(start.CarId, Swapper().ShownCar.CarId,
            "Six presses on a six-car cycle must land back on the starting car.");
    }

    [UnityTest]
    public IEnumerator FocusingALockedCar_ParksContinueAndKeepsTheSessionCar()
    {
        yield return EnterGarage();

        var sessionCar = _flow.SelectedCar;
        Assert.IsNotNull(sessionCar);

        // One step from the starter lands on a car the fresh profile cannot use.
        FindButton("NextButton").onClick.Invoke();
        yield return null;

        var focused = Swapper().ShownCar;
        Assert.IsNotNull(focused);
        Assert.AreNotEqual(sessionCar.CarId, focused.CarId,
            "The next car was expected to be a different one.");
        Assert.IsFalse(PlayerProfileManager.Current.IsCarAvailable(focused.CarId),
            "The focused car was expected to be locked for a fresh profile.");
        Assert.AreEqual(sessionCar.CarId, _flow.SelectedCar.CarId,
            "A locked car must not become the session car.");
        Assert.IsFalse(FindButton("ContinueButton").interactable,
            "Continue must be parked while a locked car is on display.");
    }

    [UnityTest]
    public IEnumerator CyclingBackToAnAvailableCar_RestoresContinueAndTheSessionCar()
    {
        yield return EnterGarage();

        var sessionCar = _flow.SelectedCar;
        FindButton("NextButton").onClick.Invoke();
        yield return null;
        FindButton("PrevButton").onClick.Invoke();
        yield return null;

        Assert.AreEqual(sessionCar.CarId, Swapper().ShownCar.CarId,
            "Prev from the locked car should land back on the starter.");
        Assert.AreEqual(sessionCar.CarId, _flow.SelectedCar.CarId,
            "The session car should be re-accepted once the available car is focused again.");
        Assert.IsTrue(FindButton("ContinueButton").interactable,
            "Continue must light up again on an available car.");
    }
}
