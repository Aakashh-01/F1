using System.Collections.Generic;
using UnityEngine;

public class RaceGridManager : MonoBehaviour
{
    [System.Serializable]
    public class RaceGridEntry
    {
        public string driverName = "AI Driver";
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

    private readonly List<GameObject> _spawnedCars = new List<GameObject>();

    public IReadOnlyList<GameObject> SpawnedCars => _spawnedCars;

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

        int nextAutoGridPosition = 0;
        HashSet<int> occupiedGridPositions = new HashSet<int>();
        if (includePlayerInGrid && playerCar != null)
        {
            PositionCarOnGrid(playerCar, playerGridPosition);
            occupiedGridPositions.Add(playerGridPosition);
            nextAutoGridPosition = Mathf.Max(nextAutoGridPosition, playerGridPosition + 1);
        }

        for (int i = 0; i < aiEntries.Length; i++)
        {
            RaceGridEntry entry = aiEntries[i];
            if (entry != null && entry.gridPosition >= 0)
                occupiedGridPositions.Add(entry.gridPosition);
        }

        for (int i = 0; i < aiEntries.Length; i++)
        {
            RaceGridEntry entry = aiEntries[i];
            if (entry == null)
                continue;

            int gridPosition = entry.gridPosition >= 0
                ? entry.gridPosition
                : GetNextOpenGridPosition(ref nextAutoGridPosition, occupiedGridPositions);
            bool useExistingSceneCar = IsSceneInstance(entry.existingCar);
            GameObject car = useExistingSceneCar ? entry.existingCar : SpawnCar(entry, i);
            if (car == null)
                continue;

            PositionCarOnGrid(car, gridPosition);
            ConfigureCar(car, entry, gridPosition);
            if (!useExistingSceneCar)
                _spawnedCars.Add(car);
        }

        Physics.SyncTransforms();
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

            coordinator.physicsProfile = physicsProfile;
            coordinator.applyProfileOnAwake = true;
            coordinator.UseExternalInput = true;

            if (Application.isPlaying && physicsProfile != null)
                coordinator.ApplyProfile(physicsProfile);
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
            driver.preferredLaneOffset01 = entry.preferredLaneOffset01;
        }

        if (perception != null)
        {
            int stride = Mathf.Max(1, perceptionFrameStride);
            perception.fixedFrameStride = stride;
            perception.fixedFrameOffset = gridIndex % stride;
        }
    }

    private GameObject SpawnCar(RaceGridEntry entry, int gridIndex)
    {
        GameObject prefab = ResolvePrefab(entry);
        if (prefab == null)
            return null;

        Vector3 position = GetGridPose(gridIndex, out Quaternion rotation);

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
        car.transform.SetPositionAndRotation(position, rotation);

        Rigidbody rb = car.GetComponent<Rigidbody>();
        if (rb == null)
            return;

#if UNITY_6000_0_OR_NEWER
        rb.linearVelocity = Vector3.zero;
#else
        rb.velocity = Vector3.zero;
#endif
        rb.angularVelocity = Vector3.zero;
    }

    private int GetNextOpenGridPosition(ref int nextAutoGridPosition, HashSet<int> occupiedGridPositions)
    {
        while (occupiedGridPositions.Contains(nextAutoGridPosition))
            nextAutoGridPosition++;

        int gridPosition = nextAutoGridPosition++;
        occupiedGridPositions.Add(gridPosition);
        return gridPosition;
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
