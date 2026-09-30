using UnityEngine;
using F1.GameData;

namespace F1.GameFlow
{
    /// <summary>
    /// Spawns the selected player car, configured for the current session, at the track's
    /// canonical start pose.
    ///
    /// This closes finding M5 — the flow previously had no way to put a car on a track at
    /// all, so every session had to be driven from a hand-placed scene object.
    ///
    /// Three things happen on spawn, and all three belong to the session rather than to the
    /// prefab:
    ///
    ///   1. The car is placed at <see cref="TrackPlacement"/>'s start pose, so qualifying and
    ///      racing share one coordinate system (invariants 9 and 10).
    ///   2. The selected car's wing profile is applied to the physics coordinator, which is
    ///      what makes the wing choice actually change how the car drives.
    ///   3. The S3 timing stack is attached — tracker, reporter, HUD presenter. The car is
    ///      spawned after the track content is loaded, so each of those can be bound to the
    ///      real track and the real session rather than searching for them.
    ///
    /// Separate from <see cref="PreRaceSceneController"/> on purpose: the controller decides
    /// *when* a car should exist, this decides *what* it is. S5's race shell spawns through
    /// the same component, so there is one definition of "the player's car, set up correctly".
    /// </summary>
    [DisallowMultipleComponent]
    public class PlayerCarSpawner : MonoBehaviour
    {
        [Header("Car")]
        [Tooltip("Player car prefab. Must carry a VehiclePhysicsCoordinator and a Rigidbody.")]
        [SerializeField] private GameObject _playerCarPrefab;

        [Tooltip("Parent for the spawned car. Defaults to this transform. Keeping the car " +
                 "under a session root means unloading the session takes the car with it.")]
        [SerializeField] private Transform _spawnParent;

        [Tooltip("Lift applied to the start pose before spawning, in metres. The baked start " +
                 "pose sits on the road surface; the car's rigidbody origin is at its centre, " +
                 "so without this the car is spawned embedded in the track and shot upward.")]
        [SerializeField] private float _spawnHeightOffset = 0.6f;

        /// <summary>The spawned car, or null before the first successful spawn.</summary>
        public GameObject PlayerCar { get; private set; }

        public bool HasSpawned => PlayerCar != null;

        /// <summary>
        /// Instantiates the player car, applies the session's wing profile, attaches the
        /// timing stack, and places it at the track's start pose.
        ///
        /// Returns false — after logging why — rather than leaving a half-built car behind.
        /// A session with no car is visibly broken; one with a car at the wrong height, or
        /// with no timing stack, fails quietly much later and much more expensively.
        /// </summary>
        public bool Spawn(GameFlowManager flow, TrackPlacement track)
        {
            if (PlayerCar != null)
            {
                Debug.LogWarning(
                    "[PlayerCarSpawner] A player car is already spawned. Call Despawn first.", this);
                return false;
            }

            if (_playerCarPrefab == null)
            {
                Debug.LogError(
                    "[PlayerCarSpawner] No player car prefab assigned. The pre-race scene " +
                    "builder sets this; run Tools > BuildFlowScenes.", this);
                return false;
            }

            if (track == null || !track.IsValid)
            {
                Debug.LogError(
                    "[PlayerCarSpawner] No valid TrackPlacement, so there is no canonical " +
                    "start pose to spawn at.", this);
                return false;
            }

            var parent = _spawnParent != null ? _spawnParent : transform;
            PlayerCar = Instantiate(_playerCarPrefab, parent);
            PlayerCar.name = $"{_playerCarPrefab.name} (Player)";

            var coordinator = PlayerCar.GetComponent<VehiclePhysicsCoordinator>();
            if (coordinator == null)
            {
                Debug.LogError(
                    $"[PlayerCarSpawner] '{_playerCarPrefab.name}' has no " +
                    "VehiclePhysicsCoordinator, so the car cannot be configured for the " +
                    "session. Refusing to spawn an unconfigurable car.", this);
                Despawn();
                return false;
            }

            ApplyWingProfile(flow, coordinator);
            AttachTiming(track);
            PutOnTrafficLayer();
            PlaceAtStartPose(track);

            Debug.Log($"[PlayerCarSpawner] Spawned player car at {PlayerCar.transform.position}.");
            return true;
        }

        /// <summary>
        /// Moves the player's colliders onto the Traffic layer.
        ///
        /// The AI sense traffic and nothing else: their perception mask is the Traffic layer
        /// alone, so that the road surface, kerbs and walls cannot register as obstacles and
        /// park a car in a permanent full brake against the tarmac. That only works if the
        /// player is on that layer too — otherwise every AI drives straight through the
        /// player, which is a far worse bug than the one this fixes.
        ///
        /// Done here rather than on the prefab on purpose. The player's car prefab is
        /// hand-authored (its camera rig is worth weeks of tuning) and a layer change is a
        /// spawn-time concern, not an authoring one: it must hold for any prefab the
        /// developer later points this spawner at.
        /// </summary>
        private void PutOnTrafficLayer()
        {
            int traffic = LayerMask.NameToLayer("Traffic");
            if (traffic < 0)
            {
                Debug.LogWarning(
                    "[PlayerCarSpawner] No 'Traffic' layer in the project, so the player's " +
                    "colliders were left where they are. AI perception is masked to the " +
                    "Traffic layer, so the AI will not be able to see the player's car.",
                    this);
                return;
            }

            foreach (var col in PlayerCar.GetComponentsInChildren<Collider>())
            {
                // Only the colliders move. Renderers and the camera rig stay on their own
                // layers so culling and the chase camera are untouched.
                col.gameObject.layer = traffic;
            }

            // ...which means the lap tracker's grounded check can no longer pass by finding
            // the car's own body. Point it at the layer the track surface is actually on.
            // Without this the player's lap times silently stop accumulating the moment the
            // car is on the Traffic layer. See LapTracker.TrackLayer.
            int ground = LayerMask.NameToLayer("Ground");
            if (ground >= 0)
            {
                foreach (var tracker in PlayerCar.GetComponentsInChildren<LapTracker>(true))
                    tracker.TrackLayer = ground;
            }
        }

        /// <summary>
        /// Applies the selected car's physics profile with the selected wing applied.
        ///
        /// The prefab carries a default profile so the car is drivable in isolation; this
        /// replaces it so the wing the player chose is the wing the car drives with.
        /// </summary>
        private void ApplyWingProfile(GameFlowManager flow, VehiclePhysicsCoordinator coordinator)
        {
            var profile = flow != null ? flow.CreatePlayerPhysicsProfile() : null;
            if (profile == null)
            {
                Debug.LogWarning(
                    "[PlayerCarSpawner] No selected car, so the car keeps the profile from " +
                    "its prefab. It will drive, but not as the car the player chose.", this);
                return;
            }

            coordinator.physicsProfile = profile;
            coordinator.ApplyProfile(profile);
        }

        /// <summary>
        /// Attaches the S3 timing stack and binds it to this track.
        ///
        /// All three, always: the tracker measures, the reporter turns a completed lap into a
        /// qualifying record, and the presenter shows the clock. Attaching the tracker alone
        /// would produce a car that laps silently and never opens the race-entry gate.
        /// </summary>
        private void AttachTiming(TrackPlacement track)
        {
            var tracker = GetComponent<LapTracker>();
            if (tracker == null) tracker = PlayerCar.AddComponent<LapTracker>();
            tracker.BindTrack(track);

            if (GetComponent<QualifyingLapReporter>() == null)
                PlayerCar.AddComponent<QualifyingLapReporter>();

            if (GetComponent<LapHudPresenter>() == null)
                PlayerCar.AddComponent<LapHudPresenter>();
        }

        /// <summary>
        /// Puts the car on the canonical start pose, facing down the track, at a standstill.
        ///
        /// The velocity reset is not optional. A car teleported into a new pose keeps its
        /// rigidbody momentum, so a retry that reuses the same car would fling it off the
        /// track with whatever speed the last lap ended at.
        /// </summary>
        private void PlaceAtStartPose(TrackPlacement track)
        {
            track.GetStartPose(out var position, out var rotation);
            var spawnPosition = position + Vector3.up * _spawnHeightOffset;

            var body = PlayerCar.GetComponent<Rigidbody>();
            if (body == null)
            {
                PlayerCar.transform.SetPositionAndRotation(spawnPosition, rotation);
                return;
            }

            // Both the body and the transform, and the order matters — the same rule
            // RaceGridManager.PositionCarOnGrid follows, and for the same reason.
            //
            // The car is instantiated parented to the spawner, which sits at the world
            // origin, so before this the car is at the origin. PhysX caches a rigidbody at
            // the pose it was created in and reverts a transform-only write on the next
            // step, so the body write is not optional. The transform write is not optional
            // either: Physics.autoSyncTransforms is False here, so a later
            // Physics.SyncTransforms() pushes the transform's pose down into the body, and a
            // transform left behind at the origin wins.
            //
            // Writing one without the other cannot hold, which is why the spawner logged
            // "Spawned player car at (0.00, 1.00, 0.00)" while the car was somewhere else
            // entirely — the log was reading the transform, and the transform was stale.
            // The car root carries the rigidbody, so these are the same pose.
            body.position = spawnPosition;
            body.rotation = rotation;
            PlayerCar.transform.SetPositionAndRotation(spawnPosition, rotation);

            // The lift above is a starting guess, not the answer. This car's collider hangs
            // 0.922 m below its rigidbody origin, so 0.6 m put the car 0.32 m INTO the road
            // and PhysX shoved it back out on the next step. Measure instead — see
            // VehicleGroundSnap. The lift is kept as a coarse first guess so the car starts
            // roughly in the right place before the probe runs.
            F1.Gameplay.VehicleGroundSnap.Snap(PlayerCar);

            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            body.WakeUp();
        }

        /// <summary>
        /// Returns the car to the start pose and clears its lap state, for a qualifying
        /// retry. Reuses the existing car: the session is the same car, track and wing, and
        /// respawning would throw away nothing worth keeping while re-running the whole
        /// physics setup.
        /// </summary>
        public bool ResetToStartPose(TrackPlacement track)
        {
            if (PlayerCar == null)
            {
                Debug.LogWarning("[PlayerCarSpawner] Cannot reset: no car is spawned.", this);
                return false;
            }

            if (track == null || !track.IsValid)
            {
                Debug.LogError("[PlayerCarSpawner] Cannot reset: no valid track.", this);
                return false;
            }

            PlaceAtStartPose(track);

            var tracker = PlayerCar.GetComponent<LapTracker>();
            if (tracker != null)
                tracker.ResetTracker();

            return true;
        }

        public void Despawn()
        {
            if (PlayerCar == null) return;

            Destroy(PlayerCar);
            PlayerCar = null;
        }

        private void OnDestroy() => Despawn();
    }
}

