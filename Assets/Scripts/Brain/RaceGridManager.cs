using System.Collections.Generic;
using UnityEngine;

public class RaceGridManager : MonoBehaviour
{
    [System.Serializable]
    public class RaceGridEntry
    {
        public string driverName = "AI Driver";
        [Tooltip("Race number for the car's board. Zero means the manager assigns one from " +
                 "the grid slot. AIFieldDefinition sets this from its number pool.")]
        public int driverNumber;
        [Tooltip("Optional explicit grid position. Use -1 to auto-fill after the player and earlier entries.")]
        public int gridPosition = -1;
        public GameObject existingCar;
        public GameObject carPrefabOverride;
        public Transform spawnPoint;
        public VehiclePhysicsProfile physicsProfileOverride;
        public AIDifficultyProfile difficultyProfile;
        public AIDifficultyPreset fallbackDifficulty = AIDifficultyPreset.Medium;
        public bool preferRightOvertake = true;
        [Range(-1f, 1f)] public float preferredLaneOffset01;
    }

    [Header("References")]
    public AIRacingLine racingLine;
    public GameObject playerCar;
    public GameObject defaultAICarPrefab;
    public Transform spawnedParent;

    [Header("Defaults")]
    public VehiclePhysicsProfile defaultAIPhysicsProfile;
    public AIDifficultyProfile defaultDifficultyProfile;
    [Range(1, 12)] public int perceptionFrameStride = 3;
    public bool autoAssignCenteredLanes = true;
    [Range(0.1f, 0.8f)] public float autoPreferredLaneOffset01 = 0.42f;

    [Header("Grid")]
    public bool spawnOnStart = true;
    public bool includePlayerInGrid = true;
    [Min(0)] public int playerGridPosition;
    public Transform gridAnchor;
    [Min(1f)] public float gridRowSpacing = 8f;
    [Min(1f)] public float gridColumnSpacing = 5f;
    [Range(-8f, 8f)] public float gridCenterOffset;
    [Range(-1f, 1f)] public float gridVerticalOffset = 0.05f;
    public bool poleOnRight = true;
    public Vector3 fallbackGridOrigin;
    public RaceGridEntry[] aiEntries = new RaceGridEntry[0];

    [Header("AI Field (preferred over the entries above)")]
    [Tooltip("Optional. When assigned, this defines the whole field: how many AI cars, who " +
             "drives them and how hard they push. It wins over the aiEntries list, because " +
             "the project owns one AI car prefab and a field is a number plus a roster, not " +
             "a hand-built list of prefab references. Leave empty to use aiEntries directly.")]
    public AIFieldDefinition aiField;

    [Tooltip("Put a number and name board above each AI car. Without it every car on the " +
             "grid is the same prefab and therefore looks identical.")]
    public bool showDriverBoards = true;

    private readonly List<GameObject> _spawnedCars = new List<GameObject>();

    public IReadOnlyList<GameObject> SpawnedCars => _spawnedCars;

    /// <summary>
    /// How many AI cars the configured field will place. Zero when no field is assigned.
    ///
    /// Exposed so a caller can talk about the size of the grid without reaching into
    /// <see cref="aiField"/> or the entry list, and without building the entry array to count
    /// it.
    /// </summary>
    public int aiFieldCount => aiField != null
        ? aiField.Count
        : (aiEntries != null ? aiEntries.Length : 0);

    private void Awake()
    {
        ResolveReferences();
    }

    private void Start()
    {
        if (spawnOnStart)
            SpawnGrid();
    }

    public void SpawnGrid()
    {
        ResolveReferences();
        DestroySpawnedGrid();

        // Resolved once, so the reservation pass and the spawn pass cannot disagree about
        // how many cars there are. A field definition is a different source of truth than the
        // serialized list, and which one is in force is decided in exactly one place.
        RaceGridEntry[] entries = ResolveAIEntries();

        // Every slot the player or an authored entry has claimed, reserved before anything
        // is placed. The set is built in full first so an auto-filled car can never be handed
        // a slot that a later entry had already asked for.
        HashSet<int> occupiedGridPositions = new HashSet<int>();
        if (includePlayerInGrid && playerCar != null)
            occupiedGridPositions.Add(playerGridPosition);

        for (int i = 0; i < entries.Length; i++)
        {
            RaceGridEntry entry = entries[i];
            if (entry != null && entry.gridPosition >= 0)
                occupiedGridPositions.Add(entry.gridPosition);
        }

        if (includePlayerInGrid && playerCar != null)
            PositionCarOnGrid(playerCar, playerGridPosition);

        for (int i = 0; i < entries.Length; i++)
        {
            RaceGridEntry entry = entries[i];
            if (entry == null)
                continue;

            int gridPosition = entry.gridPosition >= 0
                ? entry.gridPosition
                : GetLowestOpenGridPosition(occupiedGridPositions);
            occupiedGridPositions.Add(gridPosition);

            bool useExistingSceneCar = IsSceneInstance(entry.existingCar);
            GameObject car = useExistingSceneCar
                ? entry.existingCar
                : SpawnCar(entry, i, gridPosition);
            if (car == null)
                continue;

            PositionCarOnGrid(car, gridPosition);
            ConfigureCar(car, entry, gridPosition);
            ApplyDriverIdentity(car, entry, i, gridPosition);
            if (!useExistingSceneCar)
                _spawnedCars.Add(car);
        }

        Physics.SyncTransforms();
    }

    /// <summary>
    /// The field the grid spawns from. A field definition wins when one is assigned, because
    /// it is the one that scales with a single number; the serialized list stays as the
    /// fallback so nothing that already relies on it stops working.
    /// </summary>
    private RaceGridEntry[] ResolveAIEntries()
    {
        if (aiField != null)
            return aiField.BuildGridEntries();

        return aiEntries ?? new RaceGridEntry[0];
    }

    /// <summary>
    /// Names and numbers the car, and gives it a board if boards are on.
    ///
    /// The number is whatever the entry carries. A field definition picks it from its number
    /// pool; an entry authored by hand can set one; failing both, the car's place in the
    /// field is used, which is stable and unique — so the board and the standings always
    /// agree.
    ///
    /// The board's headline is the *grid slot* rather than the number, and it is passed
    /// separately because the slot is decided here, by the reservation pass, and is not
    /// something the entry knows. It matters that the slot is the resolved one and not the
    /// entry's requested <c>gridPosition</c>: an entry asking for -1 gets whatever slot was
    /// actually free, and that is the slot the car is standing in.
    /// </summary>
    private void ApplyDriverIdentity(GameObject car, RaceGridEntry entry, int index, int gridPosition)
    {
        if (car == null)
            return;

        int number = entry.driverNumber > 0
            ? entry.driverNumber
            : Mathf.Max(1, index + 1);

        var identifier = car.GetComponent<DriverIdentifier>();
        if (identifier == null && showDriverBoards)
            identifier = car.AddComponent<DriverIdentifier>();

        if (identifier != null)
            identifier.SetIdentity(entry.driverName, number, gridPosition);
    }

    public void DestroySpawnedGrid()
    {
        for (int i = _spawnedCars.Count - 1; i >= 0; i--)
        {
            GameObject car = _spawnedCars[i];
            if (car == null)
                continue;

            if (Application.isPlaying)
                Destroy(car);
            else
                DestroyImmediate(car);
        }

        _spawnedCars.Clear();
    }

    public void ConfigureCar(GameObject car, RaceGridEntry entry, int gridIndex)
    {
        if (car == null || entry == null)
            return;

        VehiclePhysicsCoordinator coordinator = car.GetComponent<VehiclePhysicsCoordinator>();
        AIDriverController driver = car.GetComponent<AIDriverController>();
        AIPerceptionSensor perception = car.GetComponent<AIPerceptionSensor>();

        if (coordinator != null)
        {
            VehiclePhysicsProfile physicsProfile = entry.physicsProfileOverride != null
                ? entry.physicsProfileOverride
                : defaultAIPhysicsProfile;

            // Only overwrite the profile when there is one to overwrite it with. The AI car
            // prefab ships with a working profile of its own (F1_AI_Physics) and applies it in
            // Awake, so a grid with no default profile assigned used to null that field on
            // every spawn. The car still drove — the values were already applied — but the
            // component was left claiming it had no profile, which is a misleading thing to
            // find when a car later behaves differently from the one you configured.
            if (physicsProfile != null)
            {
                coordinator.physicsProfile = physicsProfile;
                coordinator.applyProfileOnAwake = true;

                if (Application.isPlaying)
                    coordinator.ApplyProfile(physicsProfile);
            }

            // The AI drives itself, so every input path is handed over unconditionally. This
            // one is not profile-dependent, and a car left accepting keyboard input while an
            // AI controller is also steering it is a car that fights its own driver.
            coordinator.UseExternalInput = true;
        }

        if (driver != null)
        {
            driver.coordinator = coordinator;
            driver.perception = perception;
            driver.racingLine = racingLine;
            driver.difficultyProfile = entry.difficultyProfile != null
                ? entry.difficultyProfile
                : defaultDifficultyProfile;
            driver.difficultyPreset = entry.difficultyProfile != null
                ? entry.difficultyProfile.preset
                : entry.fallbackDifficulty;
            driver.preferRightOvertake = entry.preferRightOvertake;
            driver.preferredLaneOffset01 = GetAssignedPreferredLaneOffset(entry, gridIndex);
        }

        if (perception != null)
        {
            int stride = Mathf.Max(1, perceptionFrameStride);
            perception.fixedFrameStride = stride;
            perception.fixedFrameOffset = gridIndex % stride;
        }
    }

    private GameObject SpawnCar(RaceGridEntry entry, int gridIndex, int gridPosition)
    {
        GameObject prefab = ResolvePrefab(entry);
        if (prefab == null)
            return null;

        // The *resolved* slot, not the entry's index in the field. These differ whenever the
        // player is not on pole: the field is index 0, but that car belongs wherever the
        // reservation put it. Spawning by index and then repositioning relied on the
        // reposition to win, which it did not — see PositionCarOnGrid.
        Vector3 position = GetGridPose(gridPosition, out Quaternion rotation);

        if (entry.spawnPoint != null)
        {
            position = entry.spawnPoint.position;
            rotation = entry.spawnPoint.rotation;
        }

        GameObject car = Instantiate(prefab, position, rotation, spawnedParent);
        car.name = string.IsNullOrWhiteSpace(entry.driverName) ? $"AI Driver {gridIndex + 1}" : entry.driverName;
        return car;
    }

    public Vector3 GetGridPosition(int gridPosition)
    {
        return GetGridPose(gridPosition, out _);
    }

    private Vector3 GetGridPose(int gridPosition, out Quaternion rotation)
    {
        Vector3 forward = GetGridForward();
        Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;
        if (right.sqrMagnitude <= 0.001f)
            right = transform.right;

        Vector3 origin = GetGridOrigin(forward, right);
        Vector3 position = origin
            + GetGridOffset(gridPosition, forward, right)
            + Vector3.up * gridVerticalOffset;

        rotation = Quaternion.LookRotation(forward, Vector3.up);
        return position;
    }

    private Vector3 GetGridOffset(int gridPosition, Vector3 forward, Vector3 right)
    {
        int safeGridPosition = Mathf.Max(0, gridPosition);
        int row = safeGridPosition / 2;
        bool rightSlot = safeGridPosition % 2 == 0 ? poleOnRight : !poleOnRight;
        float side = rightSlot ? 1f : -1f;
        return -forward * (row * gridRowSpacing)
            + right * (gridCenterOffset + side * gridColumnSpacing * 0.5f);
    }

    private void PositionCarOnGrid(GameObject car, int gridPosition)
    {
        if (car == null)
            return;

        Vector3 position = GetGridPose(gridPosition, out Quaternion rotation);

        // Both the transform and the rigidbody, and the order matters.
        //
        // A transform-only write is reverted by PhysX on the next step, because the body
        // keeps the pose it was cached at. A body-only write is reverted by the
        // Physics.SyncTransforms() at the end of SpawnGrid, which pushes the transform's pose
        // back into physics — and if the transform was never moved, that stale pose wins.
        // Writing the transform first and the body second leaves the two in agreement, so
        // whichever the sync uses, the car ends up where it was put.
        //
        // Getting this wrong was invisible for a long time: an AI car is instantiated at its
        // own grid pose, so the reposition that follows used to be a no-op, and the field
        // looked correct. It only became visible once the player was moved onto the grid from
        // the start pose, and then once the reservation started placing cars somewhere other
        // than their field index.
        car.transform.SetPositionAndRotation(position, rotation);

        var body = car.GetComponent<Rigidbody>();
        if (body == null)
            return;

        body.position = position;
        body.rotation = rotation;

#if UNITY_6000_0_OR_NEWER
        body.linearVelocity = Vector3.zero;
#else
        body.velocity = Vector3.zero;
#endif
        body.angularVelocity = Vector3.zero;
        body.WakeUp();
    }

    /// <summary>
    /// The lowest slot nobody has claimed.
    ///
    /// Filling forwards from wherever the player happens to sit was wrong the moment the
    /// player did not start on pole: the cars all went *behind* the player and the slots in
    /// front of them stayed empty, so a player starting 7th produced a grid with a six-slot
    /// hole at the front and the whole field pushed back behind them. Taking the lowest free
    /// slot fills from pole backwards and skips only the player's own, which is what a grid
    /// with a player mid-field actually looks like.
    ///
    /// For a player on pole — the previous default — this is identical to filling forwards.
    /// </summary>
    private static int GetLowestOpenGridPosition(HashSet<int> occupiedGridPositions)
    {
        int candidate = 0;
        while (occupiedGridPositions.Contains(candidate))
            candidate++;

        return candidate;
    }

    private float GetAssignedPreferredLaneOffset(RaceGridEntry entry, int gridIndex)
    {
        if (!autoAssignCenteredLanes || Mathf.Abs(entry.preferredLaneOffset01) > 0.01f)
            return entry.preferredLaneOffset01;

        bool rightSlot = Mathf.Max(0, gridIndex) % 2 == 0 ? poleOnRight : !poleOnRight;
        float side = rightSlot ? 1f : -1f;
        return side * autoPreferredLaneOffset01;
    }

    private Vector3 GetGridOrigin(Vector3 forward, Vector3 right)
    {
        if (gridAnchor != null)
            return gridAnchor.position;

        if (includePlayerInGrid && playerCar != null)
            return playerCar.transform.position - GetGridOffset(playerGridPosition, forward, right);

        return fallbackGridOrigin;
    }

    private Vector3 GetGridForward()
    {
        if (gridAnchor != null)
            return Vector3.ProjectOnPlane(gridAnchor.forward, Vector3.up).normalized;

        if (racingLine != null && racingLine.Count >= 2)
        {
            Vector3 referencePosition = includePlayerInGrid && playerCar != null
                ? playerCar.transform.position
                : fallbackGridOrigin;
            int nearestIndex = racingLine.FindNearestIndex(referencePosition);
            return racingLine.GetSegmentForward(nearestIndex);
        }

        Vector3 fallbackForward = Vector3.ProjectOnPlane(transform.forward, Vector3.up);
        return fallbackForward.sqrMagnitude > 0.001f ? fallbackForward.normalized : Vector3.forward;
    }

    private GameObject ResolvePrefab(RaceGridEntry entry)
    {
        if (entry.carPrefabOverride != null)
            return entry.carPrefabOverride;

        if (entry.existingCar != null && !IsSceneInstance(entry.existingCar))
            return entry.existingCar;

        return defaultAICarPrefab;
    }

    private static bool IsSceneInstance(GameObject candidate)
    {
        return candidate != null && candidate.scene.IsValid();
    }

    private void ResolveReferences()
    {
        if (racingLine == null)
            racingLine = FindAnyObjectByType<AIRacingLine>();
    }
}
