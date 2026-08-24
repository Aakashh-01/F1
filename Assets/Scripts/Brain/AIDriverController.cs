using UnityEngine;

public enum AISpeedClampReason
{
    None,
    TrafficCaution,
    EmergencyBrake,
    TrackRecovery,
    SideTraffic
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
    [Range(0f, 1f)] public float steeringThrottleReduction = 0.24f;

    [Header("Competitive Pace")]
    [Range(0f, 0.5f)] public float baseCornerCautionStrength = 0.28f;
    [Range(0f, 0.5f)] public float competitiveCornerCautionStrength = 0.1f;
    [Range(0f, 0.25f)] public float overtakeSpeedBoost = 0.08f;
    [Range(0f, 1f)] public float overtakeTrafficSlowdownScale = 0.35f;
    [Range(0f, 1f)] public float overtakeSteeringThrottleScale = 0.5f;

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

    [Header("Traffic Safety")]
    [Range(0f, 2f)] public float sideTrafficLaneHoldBuffer = 0.35f;
    [Range(0f, 1f)] public float sideTrafficTurnThrottleScale = 0.74f;
    [Range(0f, 0.5f)] public float sideTrafficTurnBrake = 0.12f;
    [Range(0f, 1f)] public float sideTrafficSteeringThreshold = 0.32f;
    [Range(0f, 1f)] public float packCornerCurvatureThreshold = 0.42f;
    [Range(0.2f, 1f)] public float packCornerSpeedScale = 0.82f;

    [Header("Track Recovery")]
    [Range(0f, 8f)] public float trackRecoveryMargin = 1.25f;
    [Range(20f, 180f)] public float trackRecoverySpeedKmh = 95f;

    [Header("Stuck Recovery")]
    [Tooltip("Speed below which the AI counts as not making progress.")]
    [Range(0f, 20f)] public float stuckSpeedThresholdKmh = 6f;
    [Tooltip("How long the AI must want to move without progressing before recovering.")]
    [Range(0.5f, 15f)] public float stuckDetectionSeconds = 4f;
    [Tooltip("How long the reverse manoeuvre lasts.")]
    [Range(0.3f, 5f)] public float recoveryReverseSeconds = 1.2f;
    [Tooltip("Minimum delay between recovery attempts.")]
    [Range(0f, 10f)] public float recoveryCooldownSeconds = 4f;
    [Tooltip("Throttle demand required to consider the AI 'trying to move'.")]
    [Range(0f, 1f)] public float stuckThrottleDemandThreshold = 0.25f;

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
    [HideInInspector] public bool InStuckRecovery;
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
    private bool _referencesResolved;
    private float _stuckTimer;
    private float _recoveryTimer;
    private float _recoveryCooldownTimer;

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
        if (!_referencesResolved)
        {
            ResolveReferences();
            // Stop the per-tick FindAnyObjectByType/GetComponent lookups once all
            // three references are present. Late injection (RaceGridManager /
            // tests) assigns fields directly, so the latch flips next tick.
            _referencesResolved = coordinator != null && perception != null && racingLine != null;
        }

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
        UpdateStuckState(Time.fixedDeltaTime);
        coordinator.SetExternalInput(LastSteeringInput, LastThrottleInput, LastBrakeInput);
    }

    private void UpdateStuckState(float dt)
    {
        _recoveryCooldownTimer = Mathf.Max(0f, _recoveryCooldownTimer - dt);

        if (InStuckRecovery)
        {
            ApplyRecoveryInputs();
            _recoveryTimer -= dt;
            if (_recoveryTimer <= 0f)
            {
                InStuckRecovery = false;
                _stuckTimer = 0f;
                _recoveryCooldownTimer = recoveryCooldownSeconds;
            }

            return;
        }

        bool tryingToMove = LastThrottleInput > stuckThrottleDemandThreshold;
        bool notProgressing = coordinator.SpeedKmh < stuckSpeedThresholdKmh;
        // Only recover when the stop is externally explainable (off-track or
        // displaced from the line). An AI held up while sitting ON its line is
        // traffic, not a wedged car — reversing there would cause ramming.
        bool plausiblyWedged = IsRecoveringTrack ||
            LastLateralError > Mathf.Max(0.5f, LastLaneLimit * 0.5f);

        if (_recoveryCooldownTimer <= 0f && tryingToMove && notProgressing && plausiblyWedged)
        {
            _stuckTimer += dt;
            if (_stuckTimer >= stuckDetectionSeconds)
            {
                InStuckRecovery = true;
                _recoveryTimer = recoveryReverseSeconds;
                CancelBlockedOvertake(_overtakeDirection);
            }
        }
        else
        {
            _stuckTimer = Mathf.Max(0f, _stuckTimer - dt * 2f);
        }
    }

    private void ApplyRecoveryInputs()
    {
        // Reverse manoeuvre: brake input becomes reverse drive at standstill
        // (DrivetrainBrakeSystem.ShouldUseReverse), steering arcs away from the
        // current heading so the tail swings back toward the racing line.
        float steerAway = Mathf.Abs(LastSteeringInput) > 0.05f ? -Mathf.Sign(LastSteeringInput) : 1f;
        LastSteeringInput = 0.6f * steerAway;
        LastThrottleInput = 0f;
        LastBrakeInput = 1f;
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

        ApplySideTrafficLaneGuard();
        CurrentLaneOffset = Mathf.MoveTowards(
            CurrentLaneOffset,
            DesiredLaneOffset,
            laneOffsetMoveSpeed * Time.fixedDeltaTime);
    }

    private void ApplySideTrafficLaneGuard()
    {
        if (perception == null || IsRecoveringTrack)
            return;

        float laneDelta = DesiredLaneOffset - CurrentLaneOffset;
        bool movingRight = laneDelta > sideTrafficLaneHoldBuffer;
        bool movingLeft = laneDelta < -sideTrafficLaneHoldBuffer;

        if (perception.RightBlocked && movingRight)
        {
            DesiredLaneOffset = Mathf.Min(DesiredLaneOffset, CurrentLaneOffset);
            CancelBlockedOvertake(1);
        }

        if (perception.LeftBlocked && movingLeft)
        {
            DesiredLaneOffset = Mathf.Max(DesiredLaneOffset, CurrentLaneOffset);
            CancelBlockedOvertake(-1);
        }
    }

    private void CancelBlockedOvertake(int blockedDirection)
    {
        if (_overtakeDirection != blockedDirection)
            return;

        _overtakeDirection = 0;
        _overtakeCommitTimer = 0f;
        IsOvertaking = false;
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

        if (HasPackCornerRisk())
        {
            speedTarget = Mathf.Min(speedTarget, targetSpeedKmh * packCornerSpeedScale);
            if (LastSpeedClampReason == AISpeedClampReason.None)
                LastSpeedClampReason = AISpeedClampReason.SideTraffic;
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

        if (HasSideTrafficTurnRisk(steeringLoad))
        {
            float sideCaution = Mathf.InverseLerp(sideTrafficSteeringThreshold, 1f, steeringLoad);
            LastThrottleInput *= Mathf.Lerp(1f, sideTrafficTurnThrottleScale, sideCaution);
            LastBrakeInput = Mathf.Max(LastBrakeInput, sideTrafficTurnBrake * sideCaution);
            if (LastSpeedClampReason == AISpeedClampReason.None)
                LastSpeedClampReason = AISpeedClampReason.SideTraffic;
        }

        if (perception != null && perception.FrontBlocked && perception.FrontDistance < 6f)
        {
            LastThrottleInput = 0f;
            LastBrakeInput = Mathf.Max(LastBrakeInput, 0.85f);
            LastSpeedClampReason = AISpeedClampReason.EmergencyBrake;
        }

        if (IsRecoveringTrack && LastSpeedClampReason == AISpeedClampReason.None)
            LastSpeedClampReason = AISpeedClampReason.TrackRecovery;
    }

    private bool HasSideTrafficTurnRisk(float steeringLoad)
    {
        if (perception == null || steeringLoad < sideTrafficSteeringThreshold)
            return false;

        float laneDelta = DesiredLaneOffset - CurrentLaneOffset;
        bool rightRisk = perception.RightBlocked
            && (LastSteeringInput > sideTrafficSteeringThreshold || laneDelta > sideTrafficLaneHoldBuffer);
        bool leftRisk = perception.LeftBlocked
            && (LastSteeringInput < -sideTrafficSteeringThreshold || laneDelta < -sideTrafficLaneHoldBuffer);

        return rightRisk || leftRisk;
    }

    private bool HasPackCornerRisk()
    {
        if (perception == null || LastCornerCurvature < packCornerCurvatureThreshold)
            return false;

        return perception.FrontBlocked || perception.LeftBlocked || perception.RightBlocked;
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
