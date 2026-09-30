using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Guards the AI perception layer contract.
///
/// <para>
/// This is a regression guard for a defect that was measured, not guessed. The AI sensor
/// cast against every layer in the project, and its cast volume reaches from the road
/// surface up to 1.6 m, so the tarmac itself registered as an obstacle. A car whose closest
/// "obstacle" was the road was classified as scenery, which switched off the car-following
/// law entirely and left it on a pure distance ramp: the speed target collapsed toward zero
/// and the emergency brake applied unconditionally, because the closing-speed check is
/// deliberately skipped for immovable things.
///
/// The result, measured live on Track_01, was a car sitting at 0 km/h with
/// <c>ClosestObstacle = MainTrack_Object</c> and 0.85 brake — permanently, because a car on
/// its own racing line is classed as traffic rather than wedged, so the stuck-recovery
/// correctly declined to fire and nothing else ever released it. The side casts hit the
/// road too, so lane changes were suppressed across the whole field.
/// </para>
///
/// <para>
/// The fix is a dedicated Traffic layer holding only car colliders, with the AI perception
/// mask narrowed to exactly that. These tests exist because the original configuration was
/// not wrong in any way a test could see: the mask was a legal value, every AI test passed,
/// and the cars did drive. Only looking at one showed the field grinding to a halt.
/// </para>
/// </summary>
public class AIPerceptionLayerTests
{
    private const string AiPrefabPath = "Assets/Prefabs/AI_F1_Body.prefab";
    private const string TrackScenePath = "Assets/Scenes/Track_01.unity";

    private static int TrafficLayer => LayerMask.NameToLayer("Traffic");

    private static int GroundLayer => LayerMask.NameToLayer("Ground");

    [Test]
    public void TheProjectDefinesATrafficLayer()
    {
        Assert.AreNotEqual(-1, TrafficLayer,
            "The 'Traffic' layer is missing from the Tag Manager. AI perception is masked to " +
            "it, so without it the AI either cannot see cars or see the road again.");
    }

    [Test]
    public void TheAiCarColliderSitsOnTheTrafficLayer()
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(AiPrefabPath);
        Assert.IsNotNull(prefab, AiPrefabPath + " is missing.");

        var colliders = prefab.GetComponentsInChildren<Collider>(true);
        Assert.IsNotEmpty(colliders, "The AI car prefab has no collider, so the sensor can " +
                                     "never see it — including its own reflection in traffic.");

        foreach (var col in colliders)
        {
            Assert.AreEqual(TrafficLayer, col.gameObject.layer,
                $"'{col.gameObject.name}' carries a collider but is on layer " +
                $"'{LayerMask.LayerToName(col.gameObject.layer)}'. The AI perception mask is " +
                "the Traffic layer alone, so a collider anywhere else is invisible to every " +
                "other car — they will drive straight through it.");
        }
    }

    [Test]
    public void TheAiSensorLooksOnlyAtCars()
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(AiPrefabPath);
        Assert.IsNotNull(prefab, AiPrefabPath + " is missing.");

        var sensor = prefab.GetComponentInChildren<AIPerceptionSensor>(true);
        Assert.IsNotNull(sensor, "The AI car prefab has no AIPerceptionSensor.");

        int mask = sensor.obstacleLayers.value;

        Assert.AreNotEqual(~0, mask,
            "The AI sensor is scanning every layer, which puts the road surface back into " +
            "the cast and re-creates the deadlock this layer exists to prevent.");

        Assert.AreEqual(1 << TrafficLayer, mask,
            "The AI sensor should look at the Traffic layer and nothing else. Found: " +
            LayerMask.LayerToName(TrafficLayer) + " expected, mask = " + mask + ".");
    }

    [Test]
    public void TheTrackSurfaceIsNotOnATrafficLayer()
    {
        // The other half of the contract. Cars and track have to be separable, and a track
        // collider quietly moved onto the Traffic layer would put the deadlock straight back
        // with no visible change to any car.
        var scene = UnityEditor.SceneManagement.EditorSceneManager.OpenScene(
            TrackScenePath, UnityEditor.SceneManagement.OpenSceneMode.Single);

        try
        {
            var colliders = scene.GetRootGameObjects()
                .SelectMany(go => go.GetComponentsInChildren<Collider>(true))
                .ToArray();

            Assert.IsNotEmpty(colliders, "The track content scene has no colliders at all.");

            foreach (var col in colliders)
            {
                Assert.AreNotEqual(TrafficLayer, col.gameObject.layer,
                    $"Track collider '{col.name}' is on the Traffic layer. Traffic is what " +
                    "the AI reads as another car, so the road would be treated as one.");
            }
        }
        finally
        {
            UnityEditor.SceneManagement.EditorSceneManager.OpenScene(
                "Assets/Scenes/00_LoadingScene.unity",
                UnityEditor.SceneManagement.OpenSceneMode.Single);
        }
    }

    [Test]
    public void TheLapTrackerPointsAtTheLayerTheTrackIsActuallyOn()
    {
        // The grounded check casts against a single layer, and it was pointed at Default
        // while the track surface is authored on Ground. It appeared to work only because a
        // car's own body collider was also on Default and sat directly beneath the cast
        // origin — the tracker was finding the car, not the road. Move the body to Traffic
        // and every AI silently stopped accumulating lap distance: no lap times, no
        // lap-completion events, and no progress for anything that ranks cars.
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(AiPrefabPath);
        Assert.IsNotNull(prefab, AiPrefabPath + " is missing.");

        var tracker = prefab.GetComponentInChildren<LapTracker>(true);
        Assert.IsNotNull(tracker, "The AI car has no LapTracker, so it can never report " +
                                  "progress and the race HUD cannot rank the field.");

        var so = new SerializedObject(tracker);
        int configured = so.FindProperty("_trackLayer").intValue;

        Assert.AreNotEqual(0, configured,
            "The lap tracker's grounded check is aimed at the Default layer, which is not " +
            "where the track surface is. It only ever passed by hitting the car's own " +
            "collider; on the Traffic layer that stops happening and lap distance freezes " +
            "at zero forever.");
    }
}
