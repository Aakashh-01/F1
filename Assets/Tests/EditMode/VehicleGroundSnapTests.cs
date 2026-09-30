using NUnit.Framework;
using UnityEngine;
using F1.Gameplay;

/// <summary>
/// The spawn height invariant: a car placed by the ground snap is resting on the surface, and
/// stays there.
///
/// This exists because the bug it guards was invisible to every other test in the project.
/// The snap measured the surface correctly, computed the right correction, and wrote it to
/// the rigidbody — and was then silently undone a few lines later by the
/// <c>Physics.SyncTransforms()</c> at the end of <c>SpawnGrid</c>, which pushed the
/// transform's stale height back down. Every assertion about the grid, the field and the
/// car's position passed, because by the time they ran the car was back at the height nobody
/// had put it at. The only symptom was a visible bump on the first physics step, which no
/// unit test in this project was watching for.
///
/// The assertion is therefore about the *pair* agreeing, not about either one alone: after a
/// snap, the body and the transform must report the same height, because a snap that leaves
/// them disagreeing is a snap that some later sync can undo.
/// </summary>
public class VehicleGroundSnapTests
{
    private GameObject _road;
    private GameObject _car;

    [SetUp]
    public void SetUp()
    {
        // A road at y = 0 with a top surface at y = 0, matching the track's cross-section
        // cells. The car below is built to the real prefab's proportions: a collider whose
        // bottom hangs 0.92 m below the rigidbody origin, which is the number that made the
        // old fixed-height spawns wrong in the first place.
        _road = GameObject.CreatePrimitive(PrimitiveType.Cube);
        _road.name = "Road";
        _road.transform.position = new Vector3(0f, -0.5f, 0f);
        _road.transform.localScale = new Vector3(200f, 1f, 200f);

        _car = new GameObject("Car");
        _car.transform.position = new Vector3(0f, 1f, 0f);
        _car.AddComponent<Rigidbody>();
        var collider = _car.AddComponent<BoxCollider>();
        collider.size = new Vector3(1.8f, 1.349f, 4f);
        collider.center = new Vector3(0f, -0.1475f, 0f);

        // The snap's own probe is a physics query, and queries only see colliders the physics
        // engine has been told about.
        Physics.SyncTransforms();
    }

    [TearDown]
    public void TearDown()
    {
        Object.DestroyImmediate(_car);
        Object.DestroyImmediate(_road);
    }

    [Test]
    public void SnapLeavesTheLowestColliderPointOnTheSurface()
    {
        Assert.IsTrue(VehicleGroundSnap.Snap(_car),
            "The snap declined to place a car that is plainly above a road. A snap that " +
            "cannot find a surface leaves the car wherever the caller put it, which here is " +
            "1 m up with its collider 0.92 m below the body origin.");

        var body = _car.GetComponent<Rigidbody>();
        float lowest = VehicleGroundSnap.LowestColliderOffset(body);
        float colliderBottom = body.position.y + lowest;

        Assert.AreEqual(VehicleGroundSnap.DefaultClearance, colliderBottom, 0.01f,
            "The car's lowest collider point should sit one clearance above the road, not " +
            "through it. A negative value here is a car embedded in the surface, and PhysX " +
            "ejecting it on the next step is the bump on the grid.");
    }

    [Test]
    public void SnapWritesTheSameHeightToTheBodyAndTheTransform()
    {
        // The regression this file exists for. The snap used to write only the body, leaving
        // the transform at the un-snapped height; the Physics.SyncTransforms() at the end of
        // SpawnGrid then pushed that stale height back into the body and discarded the lift.
        Assert.IsTrue(VehicleGroundSnap.Snap(_car));

        var body = _car.GetComponent<Rigidbody>();
        Assert.AreEqual(_car.transform.position.y, body.position.y, 0.0001f,
            "The body and the transform must agree after a snap. When they disagree, the " +
            "next Physics.SyncTransforms() wins and the corrected height is lost — which is " +
            "how a correct snap was measured, discarded, and reported as a bump.");
    }

    [Test]
    public void SnapSurvivesASyncTransforms()
    {
        // The end-to-end version: do what SpawnGrid does, and check the car is still resting
        // on the road afterwards. This is the assertion that would have failed before the fix,
        // and it is the one that matches what a player actually sees.
        Assert.IsTrue(VehicleGroundSnap.Snap(_car));

        Physics.SyncTransforms();

        var body = _car.GetComponent<Rigidbody>();
        float colliderBottom = body.position.y + VehicleGroundSnap.LowestColliderOffset(body);

        Assert.Greater(colliderBottom, 0f,
            "A car embedded in the road after a sync is the bump. The snap's correction has " +
            "to survive the sync, not merely be computed correctly beforehand.");
        Assert.AreEqual(VehicleGroundSnap.DefaultClearance, colliderBottom, 0.01f);
    }

    [Test]
    public void SnapClearsTheVelocitySoTheCarIsAtRest()
    {
        var body = _car.GetComponent<Rigidbody>();
        body.linearVelocity = new Vector3(12f, 0f, -30f);
        body.angularVelocity = new Vector3(0f, 4f, 0f);
        Physics.SyncTransforms();

        Assert.IsTrue(VehicleGroundSnap.Snap(_car));

        Assert.AreEqual(Vector3.zero, body.linearVelocity,
            "A car teleported into a new pose keeps its momentum, so a retry that reuses it " +
            "would fling it off the track at whatever speed the last lap ended at.");
        Assert.AreEqual(Vector3.zero, body.angularVelocity);
    }

    [Test]
    public void SnapLeavesACarWithNothingBeneathItAlone()
    {
        // Teleporting a car to somewhere arbitrary because a probe found nothing is far worse
        // than leaving it where the caller put it.
        _road.SetActive(false);
        _car.transform.position = new Vector3(0f, 40f, 0f);
        Physics.SyncTransforms();

        Vector3 before = _car.transform.position;

        Assert.IsFalse(VehicleGroundSnap.Snap(_car),
            "With no surface beneath it there is nothing to snap to.");
        Assert.AreEqual(before, _car.transform.position,
            "A car with nothing under it must not be moved.");
    }

    [Test]
    public void SnapRefusesToMoveACarByAnImplausibleDistance()
    {
        // A lift beyond MaxLift means the probe found the wrong thing — a gantry, a bridge, a
        // car floating in the air 30 m over the circuit. Teleporting the car is the worse
        // failure of the two.
        _car.transform.position = new Vector3(0f, 12f, 0f);
        Physics.SyncTransforms();

        Vector3 before = _car.transform.position;

        Assert.IsFalse(VehicleGroundSnap.Snap(_car));
        Assert.AreEqual(before, _car.transform.position,
            "A car 12 m above the road is not a car that needs snapping to the road; it is a " +
            "failed probe, and the snap should decline rather than act on it.");
    }
}
