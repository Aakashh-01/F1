using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using F1.GameFlow;

/// <summary>
/// The end of a race: scoring it, and the results screen that reports it.
///
/// A race previously could not finish at all. Everything downstream of the flag existed as an
/// interface, an abstract screen, a concrete screen script and a flow method — and none of it
/// was reachable, because nothing detected the finish, nothing called the scoring, and the
/// scene the flow routes to did not exist. Those are the three breaks this file guards, and
/// they are the kind that fail silently: a race that simply never ends still looks like a game
/// that is still loading.
///
/// The scoring policy itself (<see cref="RacePointsTable"/>) is a pure function and is tested
/// exhaustively here.
///
/// CompleteRace is deliberately NOT tested. It awards career points through
/// PlayerProfileManager, which in the editor is the player's real save file — the same trap
/// that has already left a stale best lap in player_profile.json after a previous test run.
/// Asserting on it would mean mutating the developer's career balance to test a line of
/// arithmetic that the table below already covers. Closing that properly needs an injectable
/// profile path, which is a separate piece of work.
///
/// The results impl (F1.UI.ResultsScreenImpl) lives in Assembly-CSharp, which no test
/// assembly references, so it is reached by class name — the same route WingScreenUiTests
/// takes for the wing impls. An assembly definition cannot reference the predefined assembly,
/// so adding a reference is not an option.
/// </summary>
public class RaceFinishTests
{
    private const string ResultsScenePath = "Assets/Scenes/60_ResultsScene.unity";
    private const string ResultsPrefabPath = "Assets/Prefabs/ResultsScreen_Prefab.prefab";
    private const string RaceScenePath = "Assets/Scenes/50_RaceScene.unity";

    /// <summary>Finds a component by class name, since the type is not referenceable here.</summary>
    private static Component FindByTypeName(GameObject root, string typeName)
    {
        return root.GetComponentsInChildren<MonoBehaviour>(true)
            .FirstOrDefault(mb => mb != null && mb.GetType().Name == typeName);
    }

    // --- The GDD's points table ---

    [Test]
    public void EveryScoredPositionPaysTheGddAmount()
    {
        // The "Driver Ranking Rewards" table, transcribed. Written out literally rather than
        // generated, so a change to the game's economy has to be a deliberate edit here and
        // cannot drift silently into being whatever the code currently does.
        int[] expected =
        {
            1250, // 1st
            1000, // 2nd
            800,  // 3rd
            700,  // 4th
            600,  // 5th
            500,  // 6th
            400,  // 7th
            300,  // 8th
            200,  // 9th
            100,  // 10th
        };

        for (int i = 0; i < expected.Length; i++)
        {
            int position = i + 1;
            Assert.AreEqual(expected[i], RacePointsTable.PointsForPosition(position),
                $"P{position} must pay {expected[i]} points.");
        }

        Assert.AreEqual(expected.Length, RacePointsTable.ScoredPositions,
            "The table covers ten positions, which is what the GDD specifies.");
    }

    [Test]
    public void PointsFallMonotonicallyAsThePositionWorsens()
    {
        // A table where a worse finish pays more is not a bug anyone would notice by reading
        // it, and it would quietly make finishing badly the better strategy.
        for (int position = 2; position <= RacePointsTable.ScoredPositions; position++)
        {
            int better = RacePointsTable.PointsForPosition(position - 1);
            int worse = RacePointsTable.PointsForPosition(position);
            Assert.Less(worse, better,
                $"P{position} pays no less than P{position - 1}, so finishing worse is better.");
        }
    }

    [Test]
    public void APositionOutsideTheScoredRangePaysNothingRatherThanFailing()
    {
        // Zero is the honest answer for a finish the GDD does not score, and the caller awards
        // it and carries on. Throwing here would abort a race that had legitimately ended.
        Assert.AreEqual(0, RacePointsTable.PointsForPosition(RacePointsTable.ScoredPositions + 1),
            "Finishing outside the scored positions earns nothing.");
        Assert.AreEqual(0, RacePointsTable.PointsForPosition(0),
            "Position zero is not a finish.");
        Assert.AreEqual(0, RacePointsTable.PointsForPosition(-1),
            "A negative position is not a finish.");
    }

    [Test]
    public void CompletingAnAlreadyCompletedRaceAwardsNothingASecondTime()
    {
        // The double-award guard, tested without touching the profile.
        //
        // A real CompleteRace call awards career points through PlayerProfileManager, which
        // in the editor is the developer's real save file — so this deliberately does NOT make
        // a first call. It flags a result as already recorded and then calls in, which is
        // exactly the double-arrival case, and asserts the early return.
        //
        // That is the half worth guarding: the arithmetic it protects is the table above,
        // which is pure, but a silently doubled career balance is the one failure here that
        // nothing else in the game would notice.
        var probe = new GameObject("RaceFinishIdempotenceProbe");
        try
        {
            var flow = probe.AddComponent<GameFlowManager>();

            var property = typeof(GameFlowManager)
                .GetProperty(nameof(GameFlowManager.HasRaceResult));
            Assert.IsNotNull(property, "GameFlowManager.HasRaceResult has gone.");

            property.SetValue(flow, true);

            int awarded = flow.CompleteRace(1);

            Assert.AreEqual(0, awarded,
                "A race that has already been completed must not pay a second time. P1 is " +
                "1250 points, so anything non-zero here means a finish arriving twice — an " +
                "event firing twice, a resumed coroutine — doubles the career balance.");
            Assert.IsTrue(flow.HasRaceResult,
                "A refused second call must leave the recorded result alone, not clear it.");
        }
        finally
        {
            Object.DestroyImmediate(probe);
        }
    }

    // --- The results scene ---

    [Test]
    public void TheResultsSceneExistsAndShips()
    {
        // The flow routes here at the end of every race. A scene that exists but is not in the
        // build list is worse than one that is absent: the transition loads, reports success,
        // and then finds nothing there.
        Assert.IsNotNull(AssetDatabase.LoadAssetAtPath<SceneAsset>(ResultsScenePath),
            $"{ResultsScenePath} does not exist, so the flow routes to a scene that cannot load.");

        bool inBuild = EditorBuildSettings.scenes.Any(s => s.path == ResultsScenePath && s.enabled);
        Assert.IsTrue(inBuild,
            $"{ResultsScenePath} is not in the build list. The flow routes there after every " +
            "race, and a scene missing from the build list fails to load at runtime only.");
    }

    [Test]
    public void TheResultsSceneHostsTheResultsScreenAndNothingElse()
    {
        // Opened ADDITIVELY and closed again. SceneContentTests opens scenes Single, which
        // closes whatever the developer has open — that is how a hand-placed camera was lost
        // in this project once, and an EditMode test must never be able to do it.
        var scene = EditorSceneManager.OpenScene(ResultsScenePath, OpenSceneMode.Additive);
        try
        {
            var hosts = scene.GetRootGameObjects()
                .SelectMany(root => root.GetComponentsInChildren<FlowSceneHost>(true))
                .ToList();
            Assert.AreEqual(1, hosts.Count,
                "Exactly one host, so there is one results screen and no ambiguity about which " +
                "one receives the result.");

            var host = hosts[0];
            Assert.AreEqual(GameFlowManager.GameScreen.Results, host.HostedScreen,
                "The host must claim the Results screen, or the flow will not find it and the " +
                "result has nowhere to be presented.");

            Assert.IsNotNull(new SerializedObject(host).FindProperty("_screenPrefab").objectReferenceValue,
                "The host has no screen prefab, so the results scene loads with no UI at all.");

            var controller = host.GetComponent<ResultsSceneController>();
            Assert.IsNotNull(controller,
                "The results scene needs a controller to wire the buttons to the flow. Without " +
                "it the screen renders and every button is inert.");
        }
        finally
        {
            EditorSceneManager.CloseScene(scene, true);
        }
    }

    [Test]
    public void TheResultsScreenHasNoCameraOfItsOwn()
    {
        // Asserted as an ABSENCE, which is unusual, and deliberately so.
        //
        // The track stays loaded underneath this scene and carries no camera of its own; the
        // player's camera came from the spawned car in the race scene. So at this moment there
        // is no camera at all, and the only thing drawn is the screen's own opaque overlay.
        // Adding a flat-clear camera to fix the "no camera available" warning is precisely
        // what blanked the qualifying screen once: a cullingMask of 0 clears the shared render
        // target and renders no replacement, so whichever camera renders last wins, and that
        // one wins with an empty frame.
        var scene = EditorSceneManager.OpenScene(ResultsScenePath, OpenSceneMode.Additive);
        try
        {
            var cameras = scene.GetRootGameObjects()
                .SelectMany(root => root.GetComponentsInChildren<Camera>(true))
                .ToList();

            Assert.AreEqual(0, cameras.Count,
                "The results screen is an opaque Screen Space - Overlay canvas, which renders " +
                "with no camera at all. A camera here competes with whatever is live during " +
                "the transition and can clear the frame.");
        }
        finally
        {
            EditorSceneManager.CloseScene(scene, true);
        }
    }

    [Test]
    public void TheResultsSceneCanReceiveAClickAtAll()
    {
        // An EventSystem is what turns a Canvas from something you can SEE into something you
        // can USE. A Canvas does not need one to draw, so a screen missing it renders
        // perfectly and then ignores every click — which is exactly how this failed once: the
        // results panel came up complete and correct and both its buttons did nothing.
        //
        // The module's ACTIONS are asserted, not just the object's existence. An
        // InputSystemUIInputModule with unassigned actions is present, enabled and still
        // routes nothing, so a test that only looked for the component would sit green over
        // the identical failure.
        var scene = EditorSceneManager.OpenScene(ResultsScenePath, OpenSceneMode.Additive);
        try
        {
            var systems = scene.GetRootGameObjects()
                .SelectMany(root => root.GetComponentsInChildren<EventSystem>(true))
                .ToList();

            Assert.AreEqual(1, systems.Count,
                "The results scene needs an EventSystem. Without one its buttons cannot be " +
                "clicked, while the screen still draws normally — the failure looks like dead " +
                "buttons rather than a missing object.");

            var module = systems[0].GetComponents<MonoBehaviour>()
                .FirstOrDefault(mb => mb != null && mb.GetType().Name == "InputSystemUIInputModule");
            Assert.IsNotNull(module,
                "The EventSystem has no input module, so it routes no pointer or click events.");

            var so = new SerializedObject(module);

            var point = so.FindProperty("m_PointAction");
            Assert.IsNotNull(point, "The input module has no point action field.");
            Assert.IsNotNull(point.objectReferenceValue,
                "The input module's point action is unassigned, so the pointer never reaches a " +
                "button even though the EventSystem exists.");

            var click = so.FindProperty("m_LeftClickAction");
            Assert.IsNotNull(click, "The input module has no left-click action field.");
            Assert.IsNotNull(click.objectReferenceValue,
                "The input module's left-click action is unassigned, so a click on a button " +
                "goes nowhere.");
        }
        finally
        {
            EditorSceneManager.CloseScene(scene, true);
        }
    }

    // --- The results prefab ---

    [Test]
    public void TheResultsScreenWiresBothOfItsButtons()
    {
        // A prefab button whose onClick has no persistent listener looks perfect and does
        // nothing. The listener can only be written once the impl is a prefab asset, so this
        // is a genuine failure mode of the build rather than a hypothetical.
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(ResultsPrefabPath);
        Assert.IsNotNull(prefab, $"{ResultsPrefabPath} does not exist.");
        Assert.IsNotNull(FindByTypeName(prefab, "ResultsScreenImpl"),
            "The results prefab must carry the impl the flow presents to.");

        AssertButtonWired(prefab.transform, "ContinueButton",
            "Continue returns to the lobby, which is what closes the race loop.");

        AssertButtonWired(prefab.transform, "RetryButton",
            "Retry runs the same race again.");
    }

    private static void AssertButtonWired(Transform root, string buttonName, string why)
    {
        var buttonTransform = root.Find(buttonName);
        Assert.IsNotNull(buttonTransform, $"{ResultsPrefabPath} has no {buttonName}.");

        var button = buttonTransform.GetComponent<Button>();
        Assert.IsNotNull(button, $"{buttonName} has no Button component.");

        Assert.AreEqual(1, button.onClick.GetPersistentEventCount(),
            $"{buttonName} has no persistent listener, so it renders as a button and does " +
            $"nothing when pressed. {why}");
    }

    [Test]
    public void TheResultsScreenAnnouncesUnlocksRatherThanClaimingThem()
    {
        // The flow hands over the cheapest thing the player can now AFFORD and leaves the
        // points alone; the unlock stays a deliberate tap in the lobby. A screen that says
        // "NEW UNLOCKS!" is telling the player they own something they do not, and the next
        // click they make corrects it.
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(ResultsPrefabPath);
        Assert.IsNotNull(prefab, $"{ResultsPrefabPath} does not exist.");

        var impl = FindByTypeName(prefab, "ResultsScreenImpl");
        Assert.IsNotNull(impl, $"{ResultsPrefabPath} has no ResultsScreenImpl.");
        var so = new SerializedObject(impl);

        Assert.IsNotNull(so.FindProperty("_unlockPanel"), "The results screen has no unlock panel.");
        Assert.IsNull(so.FindProperty("_mainMenuButton").objectReferenceValue,
            "There is no main menu to route to — the lobby is the hub — so the results screen " +
            "must not offer a Main Menu button.");
    }

    // --- The race shell ---

    [Test]
    public void TheRaceShellRunsADrivableNumberOfLaps()
    {
        // Monaco's TrackDefinition carries 78 laps, the real Grand Prix distance. That is
        // correct for the shipped game and impossible to drive to a finish in a sitting, so
        // the shell overrides it. The override is what makes the finish reachable — and
        // therefore testable — at all.
        var scene = EditorSceneManager.OpenScene(RaceScenePath, OpenSceneMode.Additive);
        try
        {
            var shell = scene.GetRootGameObjects()
                .SelectMany(root => root.GetComponentsInChildren<RaceSceneController>(true))
                .FirstOrDefault();

            Assert.IsNotNull(shell, "The race shell has no RaceSceneController.");

            var so = new SerializedObject(shell);
            int laps = so.FindProperty("_raceLapsOverride").intValue;

            Assert.Greater(laps, 0,
                "The race lap override is unset, so a race runs the track's full distance and " +
                "the finish line is never reached in a demo or a test.");
            Assert.LessOrEqual(laps, 10,
                "A demo race should be short enough to drive to its finish in a sitting.");
        }
        finally
        {
            EditorSceneManager.CloseScene(scene, true);
        }
    }
}