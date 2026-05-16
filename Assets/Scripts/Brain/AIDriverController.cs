using UnityEngine;

public enum AISpeedClampReason
{
    None,
    TrafficCaution,
    EmergencyBrake,
    TrackRecovery
}

[DefaultExecutionOrder(-150)]
public class AIDriverController : MonoBehaviour
{
    [Header("References")]
    public VehiclePhysicsCoordinator coordinator;
    public AIRacingLine racingLine;
    public AIPerceptionSensor perception;
    public AIDifficultyProfile difficultyProfile;

    [Header("Difficulty")]
    public AIDifficultyPreset difficultyPreset = AIDifficultyPreset.Medium;

    [Header("Path Following")]
    [Range(2f, 60f)] public float baseLookaheadDistance = 12f;
    [Range(0f, 0.4f)] public float lookaheadPerKmh = 0.08f;
    [Range(1, 6)] public int curvatureLookaheadSteps = 2;
    [Range(15f, 90f)] public float steeringAngleForFullInput = 38f;
    [Range(0.2f, 8f)] public float laneOffsetMoveSpeed = 3.5f;
    [Range(1f, 25f)] public float waypointReachDistance = 8f;
    [Range(1, 10)] public int forwardProgressSearchSteps = 4;

    [Header("Speed Control")]
    [Range(1f, 80f)] public float throttleFullErrorKmh = 22f;
    [Range(1f, 100f)] public float brakeFullErrorKmh = 34f;
    [Range(10f, 160f)] public float blockedTargetSpeedKmh = 45f;
    [Range(0f, 1f)] public float steeringThrottleReduction = 0.35f;

    [Header("Competitive Pace")]
    [Range(0f, 0.5f)] public float baseCornerCautionStrength = 0.35f;
    [Range(0f, 0.5f)] public float competitiveCornerCautionStrength = 0.16f;
    [Range(0f, 0.25f)] public float overtakeSpeedBoost = 0.08f;
    [Range(0f, 1f)] public float overtakeTrafficSlowdownScale = 0.35f;
    [Range(0f, 1f)] public float overtakeSteeringThrottleScale = 0.55f;

    [Header("Overtaking")]
    public bool preferRightOvertake = true;
    [Range(0f, 1f)] public float overtakeTrigger = 0.35f;
    [Range(0.1f, 4f)] public float overtakeCommitSeconds = 1.25f;

    [Header("Race Craft")]
    [Range(-1f, 1f)] public float preferredLaneOffset01;
    [Range(0f, 1f)] public float freeTrackLaneUse = 0.62f;
    [Range(0f, 1f)] public float waypointLaneInfluence = 0.72f;
    [Range(0f, 4f)] public float laneEdgeSafetyMargin = 1.4f;
    [Range(0f, 1f)] public float laneVariationStrength = 0.18f;
    [Range(0.01f, 0.5f)] public float laneVariationFrequency = 0.06f;

    [Header("Track Recovery")]
    [Range(0f, 8f)] public float trackRecoveryMargin = 1.25f;
    [Range(20f, 180f)] public float trackRecoverySpeedKmh = 95f;

    [Header("Debug")]
    public bool drawDebug = true;
    [HideInInspector] public int CurrentWaypointIndex = -1;
    [HideInInspector] public int LookaheadWaypointIndex = -1;
    [HideInInspector] public float LastTargetSpeedKmh;
    [HideInInspector] public float LastCornerCurvature;
    [HideInInspector] public float LastSteeringInput;
    [HideInInspector] public float LastThrottleInput;
    [HideInInspector] public float LastBrakeInput;
    [HideInInspector] public float DesiredLaneOffset;
    [HideInInspector] public float CurrentLaneOffset;
    [HideInInspector] public float LastWaypointLaneIntent;
    [HideInInspector] public bool IsOvertaking;
    [HideInInspector] public bool IsRecoveringTrack;
    [HideInInspector] public float LastLateralError;
    [HideInInspector] public float LastLaneLimit;
    [HideInInspector] public float LastUnblockedTargetSpeedKmh;
    [HideInInspector] public float LastSpeedTargetKmh;
    [HideInInspector] public AISpeedClampReason LastSpeedClampReason;

    private AIDifficultyProfile _runtimeDifficulty;
    private bool _hasProgressIndex;
    private float _laneVariationSeed;
    private float _overtakeCommitTimer;
    private int _overtakeDirection;

    private AIDifficultyProfile Difficulty
    {
        get
        {
            if (difficultyProfile != null)
                return difficultyProfile;

            if (_runtimeDifficulty == null || _runtimeDifficulty.preset != difficultyPreset)
                _runtimeDifficulty = AIDifficultyProfile.CreateRuntimeProfile(difficultyPreset);

            return _runtimeDifficulty;
        }
    }

    private void Awake()
    {
        _laneVariationSeed = Mathf.Abs(GetInstanceID() % 997) * 0.017f;
        ResolveReferences();
        if (coordinator != null)
            coordinator.UseExternalInput = true;
    }

    private void FixedUpdate()
    {
        Simulate();
    }

    public void Simulate()
    {
        ResolveReferences();

        if (coordinator == null || racingLine == null || racingLine.Count < 2)
            return;

        coordinator.UseExternalInput = true;
        if (perception != null)
            perception.Tick(false);

        AIDifficultyProfile difficulty = Difficulty;
        float speedKmh = coordinator.SpeedKmh;
        float lookaheadDistance = baseLookaheadDistance + speedKmh * lookaheadPerKmh;
        UpdateProgressIndex();
        Vector3 lookaheadPoint = racingLine.GetPointAheadFromSegment(CurrentWaypointIndex, transform.position, lookaheadDistance, out int lookaheadSegmentIndex);
        LookaheadWaypointIndex = racingLine.WrapIndex(lookaheadSegmentIndex + 1);

        AIRacingWaypoint waypoint = racingLine.GetWaypoint(LookaheadWaypointIndex);
        if (waypoint == null)
            return;

        LastCornerCurvature = racingLine.CalculateCurvature01(CurrentWaypointIndex, curvatureLookaheadSteps);
        float cornerScale = Mathf.Lerp(1f, difficulty.cornerConfidence, LastCornerCurvature);
        float cautionStrength = Mathf.Lerp(baseCornerCautionStrength, competitiveCornerCautionStrength, GetCompetitiveness01(difficulty));
        LastTargetSpeedKmh = waypoint.targetSpeedKmh * difficulty.speedMultiplier * cornerScale;
        LastTargetSpeedKmh *= Mathf.Lerp(1f, 1f - waypoint.brakingCaution * cautionStrength, LastCornerCurvature);

        UpdateTrackRecoveryState(waypoint);
        UpdateLaneOffset(waypoint, difficulty);
        if (IsOvertaking)
            LastTargetSpeedKmh *= 1f + overtakeSpeedBoost * GetCompetitiveness01(difficulty);

        if (IsRecoveringTrack)
            LastTargetSpeedKmh = Mathf.Min(LastTargetSpeedKmh, trackRecoverySpeedKmh);

        LastUnblockedTargetSpeedKmh = LastTargetSpeedKmh;
        Vector3 targetPoint = lookaheadPoint + racingLine.GetSegmentRight(lookaheadSegmentIndex) * CurrentLaneOffset;
        CalculateInputs(targetPoint, LastTargetSpeedKmh, speedKmh, difficulty);
        coordinator.SetExternalInput(LastSteeringInput, LastThrottleInput, LastBrakeInput);
    }

    private void UpdateProgressIndex()
    {
        if (!_hasProgressIndex || CurrentWaypointIndex < 0 || CurrentWaypointIndex >= racingLine.Count)
        {
            CurrentWaypointIndex = racingLine.FindNearestIndex(transform.position);
            _hasProgressIndex = CurrentWaypointIndex >= 0;
        }

        if (!_hasProgressIndex)
            return;

        int bestIndex = CurrentWaypointIndex;
        float bestDistance = DistanceToSegmentSq(CurrentWaypointIndex);
        int maxSteps = Mathf.Min(forwardProgressSearchSteps, racingLine.Count);
        for (int step = 1; step <= maxSteps; step++)
        {
            int candidate = racingLine.WrapIndex(CurrentWaypointIndex + step);
            float distance = DistanceToSegmentSq(candidate);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex = candidate;
            }
        }

        CurrentWaypointIndex = bestIndex;

        int guard = 0;
        while (guard < maxSteps)
        {
            Vector3 nextPoint = racingLine.GetPosition(CurrentWaypointIndex + 1);
            float distanceToNext = Vector3.Distance(transform.position, nextPoint);
            if (distanceToNext > waypointReachDistance && !racingLine.HasPassedWaypoint(CurrentWaypointIndex, transform.position))
                break;

            CurrentWaypointIndex = racingLine.WrapIndex(CurrentWaypointIndex + 1);
            guard++;
        }
    }

    private float DistanceToSegmentSq(int segmentIndex)
    {
        Vector3 closest = racingLine.GetClosestPointOnSegment(segmentIndex, transform.position);
        return Vector3.ProjectOnPlane(transform.position - closest, Vector3.up).sqrMagnitude;
    }

    private void UpdateTrackRecoveryState(AIRacingWaypoint waypoint)
    {
        LastLateralError = Mathf.Sqrt(DistanceToSegmentSq(CurrentWaypointIndex));
        LastLaneLimit = waypoint != null ? Mathf.Max(0f, waypoint.laneWidth) : 0f;
        IsRecoveringTrack = LastLaneLimit > 0f && LastLateralError > LastLaneLimit + trackRecoveryMargin;
    }

    private void UpdateLaneOffset(AIRacingWaypoint waypoint, AIDifficultyProfile difficulty)
    {
        float laneLimit = Mathf.Max(0f, waypoint.laneWidth);
        float usableLaneLimit = Mathf.Max(0f, laneLimit - laneEdgeSafetyMargin);
        LastWaypointLaneIntent = waypoint != null ? waypoint.preferredLaneOffset01 : 0f;
        float clearTrackLane = Mathf.Clamp(
            preferredLaneOffset01 + LastWaypointLaneIntent * waypointLaneInfluence,
            -1f,
            1f);
        _overtakeCommitTimer = Mathf.Max(0f, _overtakeCommitTimer - Time.fixedDeltaTime);
        if (_overtakeCommitTimer <= 0f)
            _overtakeDirection = 0;

        if (laneVariationStrength > 0f)
        {
            float laneNoise = Mathf.PerlinNoise(_laneVariationSeed, Time.time * laneVariationFrequency) * 2f - 1f;
            clearTrackLane += laneNoise * laneVariationStrength;
        }

        DesiredLaneOffset = Mathf.Clamp(clearTrackLane, -1f, 1f) * usableLaneLimit * freeTrackLaneUse;
        IsOvertaking = false;

        if (IsRecoveringTrack)
        {
            DesiredLaneOffset = 0f;
            _overtakeDirection = 0;
            _overtakeCommitTimer = 0f;
        }

        if (!IsRecoveringTrack && perception != null && perception.FrontBlocked && difficulty.overtakeWillingness >= overtakeTrigger)
        {
            float overtakeOffset = Mathf.Min(waypoint.overtakeWidth, usableLaneLimit);
            bool canGoRight = !perception.RightBlocked;
            bool canGoLeft = !perception.LeftBlocked;
            int selectedDirection = SelectOvertakeDirection(canGoLeft, canGoRight);

            if (selectedDirection != 0)
            {
                _overtakeDirection = selectedDirection;
                _overtakeCommitTimer = Mathf.Max(_overtakeCommitTimer, overtakeCommitSeconds);
                DesiredLaneOffset = overtakeOffset * selectedDirection;
                IsOvertaking = true;
            }
        }

        CurrentLaneOffset = Mathf.MoveTowards(
            CurrentLaneOffset,
            DesiredLaneOffset,
            laneOffsetMoveSpeed * Time.fixedDeltaTime);
    }

    private void CalculateInputs(Vector3 targetPoint, float targetSpeedKmh, float speedKmh, AIDifficultyProfile difficulty)
    {
        Vector3 localTarget = transform.InverseTransformPoint(targetPoint);
        float targetAngle = Mathf.Atan2(localTarget.x, Mathf.Max(0.1f, localTarget.z)) * Mathf.Rad2Deg;
        LastSteeringInput = Mathf.Clamp(targetAngle / Mathf.Max(1f, steeringAngleForFullInput), -1f, 1f);

        float speedTarget = targetSpeedKmh;
        LastSpeedClampReason = AISpeedClampReason.None;
        if (perception != null && perception.FrontBlocked)
        {
            float distanceBlend = Mathf.InverseLerp(perception.forwardDistance, 3f, perception.FrontDistance);
            float cautionTarget = Mathf.Lerp(blockedTargetSpeedKmh, 0f, distanceBlend * difficulty.avoidanceCaution);
            if (IsOvertaking)
                cautionTarget = Mathf.Lerp(speedTarget, cautionTarget, overtakeTrafficSlowdownScale);

            speedTarget = Mathf.Min(speedTarget, cautionTarget);
            if (speedTarget < targetSpeedKmh - 0.01f)
                LastSpeedClampReason = AISpeedClampReason.TrafficCaution;
        }

        LastSpeedTargetKmh = speedTarget;
        float speedError = speedTarget - speedKmh;
        LastThrottleInput = Mathf.Clamp01(speedError / throttleFullErrorKmh);
        LastBrakeInput = Mathf.Clamp01(-speedError / (brakeFullErrorKmh * Mathf.Max(0.2f, difficulty.brakingMargin)));

        float steeringLoad = Mathf.Abs(LastSteeringInput);
        float steeringThrottleScale = IsOvertaking
            ? steeringThrottleReduction * overtakeSteeringThrottleScale
            : steeringThrottleReduction;
        LastThrottleInput *= 1f - steeringLoad * steeringThrottleScale;

        if (perception != null && perception.FrontBlocked && perception.FrontDistance < 6f)
        {
            LastThrottleInput = 0f;
            LastBrakeInput = Mathf.Max(LastBrakeInput, 0.85f);
            LastSpeedClampReason = AISpeedClampReason.EmergencyBrake;
        }

        if (IsRecoveringTrack && LastSpeedClampReason == AISpeedClampReason.None)
            LastSpeedClampReason = AISpeedClampReason.TrackRecovery;
    }

    private int SelectOvertakeDirection(bool canGoLeft, bool canGoRight)
    {
        if (_overtakeDirection > 0 && canGoRight)
            return 1;
        if (_overtakeDirection < 0 && canGoLeft)
            return -1;
        if (preferRightOvertake && canGoRight)
            return 1;
        if (canGoLeft)
            return -1;
        if (canGoRight)
            return 1;

        return 0;
    }

    private static float GetCompetitiveness01(AIDifficultyProfile difficulty)
    {
        if (difficulty == null)
            return 0f;

        float pace = Mathf.InverseLerp(0.85f, 1.2f, difficulty.speedMultiplier);
        float corner = Mathf.InverseLerp(0.75f, 1.08f, difficulty.cornerConfidence);
        float overtake = Mathf.InverseLerp(0.3f, 0.98f, difficulty.overtakeWillingness);
        return Mathf.Clamp01((pace + corner + overtake) / 3f);
    }

    private void ResolveReferences()
    {
        if (coordinator == null) coordinator = GetComponent<VehiclePhysicsCoordinator>();
        if (perception == null) perception = GetComponent<AIPerceptionSensor>();
        if (racingLine == null) racingLine = FindAnyObjectByType<AIRacingLine>();
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawDebug || racingLine == null || LookaheadWaypointIndex < 0)
            return;

        Vector3 waypoint = racingLine.GetPosition(LookaheadWaypointIndex);
        Vector3 target = waypoint + racingLine.GetSegmentRight(LookaheadWaypointIndex) * CurrentLaneOffset;
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(transform.position + Vector3.up, target + Vector3.up);
        Gizmos.DrawWireSphere(target + Vector3.up, 1.5f);
    }
}
