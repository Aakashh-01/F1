using System.Collections.Generic;
using UnityEngine;

public class ClientDemoRacePositionTracker : MonoBehaviour
{
    private sealed class RacerState
    {
        public VehiclePhysicsCoordinator vehicle;
        public float distanceAlongTrack;
        public float normalizedProgress;
        public float previousNormalizedProgress;
        public int lap;
        public bool hasProgress;
    }

    [Header("References")]
    [SerializeField] private VehiclePhysicsCoordinator player;
    [SerializeField] private AIRacingLine racingLine;
    [SerializeField] private RaceGridManager gridManager;

    [Header("Refresh")]
    [SerializeField, Min(0.05f)] private float positionRefreshInterval = 0.12f;
    [SerializeField, Min(0.25f)] private float racerDiscoveryInterval = 1f;

    private readonly List<RacerState> _racers = new List<RacerState>(12);
    private readonly List<RacerState> _ranking = new List<RacerState>(12);
    private readonly List<float> _segmentStarts = new List<float>(128);
    private readonly List<float> _segmentLengths = new List<float>(128);
    private float _trackLength;
    private float _nextPositionRefreshTime;
    private float _nextDiscoveryTime;
    private int _playerPosition = 1;

    public int PlayerPosition => Mathf.Clamp(_playerPosition, 1, Mathf.Max(1, RacerCount));
    public int RacerCount => _racers.Count;
    public string PositionText => $"P{PlayerPosition} / {Mathf.Max(1, RacerCount)}";

    private void Awake()
    {
        ResolveReferences();
        RebuildTrackCache();
        RefreshRacers();
    }

    private void Update()
    {
        if (Time.unscaledTime >= _nextDiscoveryTime)
        {
            RefreshRacers();
            _nextDiscoveryTime = Time.unscaledTime + racerDiscoveryInterval;
        }

        if (Time.unscaledTime >= _nextPositionRefreshTime)
        {
            RefreshNow();
            _nextPositionRefreshTime = Time.unscaledTime + positionRefreshInterval;
        }
    }

    public void RefreshNow()
    {
        ResolveReferences();

        if (racingLine == null || racingLine.Count < 2)
            return;

        int expectedSegmentCount = racingLine.loop ? racingLine.Count : racingLine.Count - 1;
        if (_trackLength <= 0.001f || _segmentStarts.Count != expectedSegmentCount)
            RebuildTrackCache();

        for (int i = _racers.Count - 1; i >= 0; i--)
        {
            RacerState racer = _racers[i];
            if (racer.vehicle == null)
            {
                _racers.RemoveAt(i);
                continue;
            }

            UpdateRacerProgress(racer);
        }

        RankRacers();
    }

    public void SetReferences(VehiclePhysicsCoordinator playerCar, AIRacingLine line, RaceGridManager raceGrid)
    {
        player = playerCar;
        racingLine = line;
        gridManager = raceGrid;
        RebuildTrackCache();
        RefreshRacers();
        RefreshNow();
    }

    public void RefreshRacers()
    {
        ResolveReferences();

        VehiclePhysicsCoordinator[] vehicles = FindObjectsByType<VehiclePhysicsCoordinator>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < vehicles.Length; i++)
        {
            if (vehicles[i] == null)
                continue;

            AddRacerIfMissing(vehicles[i]);
        }

        if (gridManager != null)
        {
            IReadOnlyList<GameObject> spawnedCars = gridManager.SpawnedCars;
            for (int i = 0; i < spawnedCars.Count; i++)
            {
                if (spawnedCars[i] == null)
                    continue;

                VehiclePhysicsCoordinator coordinator = spawnedCars[i].GetComponent<VehiclePhysicsCoordinator>();
                if (coordinator != null)
                    AddRacerIfMissing(coordinator);
            }
        }
    }

    public int GetPositionFor(VehiclePhysicsCoordinator vehicle)
    {
        if (vehicle == null)
            return 1;

        RefreshNow();
        for (int i = 0; i < _ranking.Count; i++)
        {
            if (_ranking[i].vehicle == vehicle)
                return i + 1;
        }

        return 1;
    }

    private void ResolveReferences()
    {
        if (racingLine == null)
            racingLine = FindAnyObjectByType<AIRacingLine>();

        if (gridManager == null)
            gridManager = FindAnyObjectByType<RaceGridManager>();

        if (player == null)
            player = FindPlayerCoordinator();
    }

    private VehiclePhysicsCoordinator FindPlayerCoordinator()
    {
        VehiclePhysicsCoordinator[] vehicles = FindObjectsByType<VehiclePhysicsCoordinator>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
        for (int i = 0; i < vehicles.Length; i++)
        {
            if (vehicles[i] != null && !vehicles[i].UseExternalInput)
                return vehicles[i];
        }

        for (int i = 0; i < vehicles.Length; i++)
        {
            if (vehicles[i] == null)
                continue;

            string vehicleName = vehicles[i].name;
            if (vehicleName.Contains("F1_Body") && !vehicleName.Contains("AI"))
                return vehicles[i];
        }

        return vehicles.Length > 0 ? vehicles[0] : null;
    }

    private void AddRacerIfMissing(VehiclePhysicsCoordinator vehicle)
    {
        for (int i = 0; i < _racers.Count; i++)
        {
            if (_racers[i].vehicle == vehicle)
                return;
        }

        _racers.Add(new RacerState { vehicle = vehicle });
    }

    private void RebuildTrackCache()
    {
        _segmentStarts.Clear();
        _segmentLengths.Clear();
        _trackLength = 0f;

        if (racingLine == null || racingLine.Count < 2)
            return;

        int segmentCount = racingLine.loop ? racingLine.Count : racingLine.Count - 1;
        for (int i = 0; i < segmentCount; i++)
        {
            _segmentStarts.Add(_trackLength);
            float segmentLength = Vector3.Distance(racingLine.GetPosition(i), racingLine.GetPosition(i + 1));
            _segmentLengths.Add(segmentLength);
            _trackLength += segmentLength;
        }
    }

    private void UpdateRacerProgress(RacerState racer)
    {
        float distance = GetDistanceAlongTrack(racer.vehicle.transform.position);
        float normalizedProgress = _trackLength > 0.001f ? distance / _trackLength : 0f;

        if (racer.hasProgress)
        {
            if (racer.previousNormalizedProgress > 0.82f && normalizedProgress < 0.18f)
                racer.lap++;
            else if (racer.previousNormalizedProgress < 0.18f && normalizedProgress > 0.82f && racer.lap > 0)
                racer.lap--;
        }

        racer.distanceAlongTrack = distance;
        racer.normalizedProgress = normalizedProgress;
        racer.previousNormalizedProgress = normalizedProgress;
        racer.hasProgress = true;
    }

    private float GetDistanceAlongTrack(Vector3 position)
    {
        if (racingLine == null || _segmentLengths.Count == 0)
            return 0f;

        int bestSegment = 0;
        float bestSegmentT = 0f;
        float bestDistanceSq = float.MaxValue;

        for (int i = 0; i < _segmentLengths.Count; i++)
        {
            Vector3 from = racingLine.GetPosition(i);
            Vector3 to = racingLine.GetPosition(i + 1);
            Vector3 segment = to - from;
            float segmentLengthSq = segment.sqrMagnitude;
            float t = segmentLengthSq > 0.001f
                ? Mathf.Clamp01(Vector3.Dot(position - from, segment) / segmentLengthSq)
                : 0f;
            Vector3 closest = Vector3.Lerp(from, to, t);
            float distanceSq = (position - closest).sqrMagnitude;

            if (distanceSq < bestDistanceSq)
            {
                bestDistanceSq = distanceSq;
                bestSegment = i;
                bestSegmentT = t;
            }
        }

        return _segmentStarts[bestSegment] + _segmentLengths[bestSegment] * bestSegmentT;
    }

    private void RankRacers()
    {
        _ranking.Clear();
        for (int i = 0; i < _racers.Count; i++)
        {
            if (_racers[i].vehicle != null)
                _ranking.Add(_racers[i]);
        }

        _ranking.Sort(CompareRacersDescending);

        _playerPosition = 1;
        for (int i = 0; i < _ranking.Count; i++)
        {
            if (_ranking[i].vehicle == player)
            {
                _playerPosition = i + 1;
                return;
            }
        }
    }

    private static int CompareRacersDescending(RacerState a, RacerState b)
    {
        int lapCompare = b.lap.CompareTo(a.lap);
        if (lapCompare != 0)
            return lapCompare;

        return b.distanceAlongTrack.CompareTo(a.distanceAlongTrack);
    }
}
