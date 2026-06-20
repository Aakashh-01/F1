using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.UI;

public class ClientDemoUITests
{
    private readonly List<GameObject> _createdObjects = new List<GameObject>();

    [TearDown]
    public void TearDown()
    {
        for (int i = _createdObjects.Count - 1; i >= 0; i--)
        {
            if (_createdObjects[i] != null)
                Object.DestroyImmediate(_createdObjects[i]);
        }

        _createdObjects.Clear();
        MobileTouchControls.ResetInputs();
    }

    [Test]
    public void ClientDemoMenu_BuildsEntryAndRaceButtons()
    {
        GameObject root = CreateObject("Client Demo Menu");
        ClientDemoMenuController menu = root.AddComponent<ClientDemoMenuController>();

        Button[] buttons = Object.FindObjectsByType<Button>(FindObjectsInactive.Include, FindObjectsSortMode.None);

        Assert.IsNotEmpty(buttons);
        Assert.That(HasButtonLabel(buttons, "START"), Is.True);
        Assert.That(HasButtonLabel(buttons, "QUIT"), Is.True);

        menu.ShowRacePanel();
        buttons = Object.FindObjectsByType<Button>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        Assert.That(HasButtonLabel(buttons, "START RACE"), Is.True);
        Assert.That(HasButtonLabel(buttons, "BACK"), Is.True);
    }

    [Test]
    public void ClientDemoRacePositionTracker_RanksPlayerAgainstAiProgress()
    {
        AIRacingLine line = CreateLine();
        VehiclePhysicsCoordinator player = CreateCar("F1_Body", new Vector3(10f, 0f, 0f), false);
        VehiclePhysicsCoordinator ai = CreateCar("AI_F1_Body", new Vector3(90f, 0f, 0f), true);

        GameObject trackerObject = CreateObject("Client Demo Position Tracker");
        ClientDemoRacePositionTracker tracker = trackerObject.AddComponent<ClientDemoRacePositionTracker>();
        tracker.SetReferences(player, line, null);
        tracker.RefreshRacers();
        tracker.RefreshNow();

        Assert.GreaterOrEqual(tracker.RacerCount, 2);
        Assert.AreEqual(2, tracker.GetPositionFor(player));
        Assert.AreEqual(1, tracker.GetPositionFor(ai));
    }

    private static bool HasButtonLabel(Button[] buttons, string label)
    {
        for (int i = 0; i < buttons.Length; i++)
        {
            Text text = buttons[i].GetComponentInChildren<Text>(true);
            if (text != null && text.text == label)
                return true;
        }

        return false;
    }

    private AIRacingLine CreateLine()
    {
        GameObject lineObject = CreateObject("Client Demo Test Racing Line");
        AIRacingLine line = lineObject.AddComponent<AIRacingLine>();
        line.loop = false;
        CreateWaypoint(lineObject.transform, "WP_00", new Vector3(0f, 0f, 0f));
        CreateWaypoint(lineObject.transform, "WP_01", new Vector3(50f, 0f, 0f));
        CreateWaypoint(lineObject.transform, "WP_02", new Vector3(100f, 0f, 0f));
        line.RefreshWaypoints();
        return line;
    }

    private void CreateWaypoint(Transform parent, string name, Vector3 position)
    {
        GameObject waypointObject = CreateObject(name);
        waypointObject.transform.SetParent(parent);
        waypointObject.transform.position = position;
        waypointObject.AddComponent<AIRacingWaypoint>();
    }

    private VehiclePhysicsCoordinator CreateCar(string name, Vector3 position, bool useExternalInput)
    {
        GameObject car = CreateObject(name);
        car.transform.position = position;
        Rigidbody rb = car.AddComponent<Rigidbody>();
        rb.useGravity = false;

        VehiclePhysicsCoordinator coordinator = car.AddComponent<VehiclePhysicsCoordinator>();
        coordinator.applyProfileOnAwake = false;
        coordinator.rb = rb;
        coordinator.UseExternalInput = useExternalInput;
        return coordinator;
    }

    private GameObject CreateObject(string name)
    {
        GameObject gameObject = new GameObject(name);
        _createdObjects.Add(gameObject);
        return gameObject;
    }
}
