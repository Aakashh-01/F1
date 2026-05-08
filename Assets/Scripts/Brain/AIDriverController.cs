using UnityEngine;

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

    [Header("Overtaking")]
    public bool preferRightOvertake = true;
    [Range(0f, 1f)] public float overtakeTrigger = 0.35f;

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

    private AIDifficultyProfile _runtimeDifficulty;
    private bool _hasProgressIndex;

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
        LastTargetSpeedKmh = waypoint.targetSpeedKmh * difficulty.speedMultiplier * cornerScale;
        LastTargetSpeedKmh *= Mathf.Lerp(1f, 1f - waypoint.brakingCaution * 0.35f, LastCornerCurvature);

        UpdateLaneOffset(waypoint, difficulty);
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

    private void UpdateLaneOffset(AIRacingWaypoint waypoint, AIDifficultyProfile difficulty)
    {
        DesiredLaneOffset = 0f;

        if (perception != null && perception.FrontBlocked && difficulty.overtakeWillingness >= overtakeTrigger)
        {
            float overtakeOffset = Mathf.Min(waypoint.overtakeWidth, waypoint.laneWidth);
            bool canGoRight = !perception.RightBlocked;
            bool canGoLeft = !perception.LeftBlocked;

            if (preferRightOvertake && canGoRight)
                DesiredLaneOffset = overtakeOffset;
            else if (canGoLeft)
                DesiredLaneOffset = -overtakeOffset;
            else if (canGoRight)
                DesiredLaneOffset = overtakeOffset;
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
        if (perception != null && perception.FrontBlocked)
        {
            float distanceBlend = Mathf.InverseLerp(perception.forwardDistance, 3f, perception.FrontDistance);
            float cautionTarget = Mathf.Lerp(blockedTargetSpeedKmh, 0f, distanceBlend * difficulty.avoidanceCaution);
            speedTarget = Mathf.Min(speedTarget, cautionTarget);
        }

        float speedError = speedTarget - speedKmh;
        LastThrottleInput = Mathf.Clamp01(speedError / throttleFullErrorKmh);
        LastBrakeInput = Mathf.Clamp01(-speedError / (brakeFullErrorKmh * Mathf.Max(0.2f, difficulty.brakingMargin)));

        float steeringLoad = Mathf.Abs(LastSteeringInput);
        LastThrottleInput *= 1f - steeringLoad * steeringThrottleReduction;

        if (perception != null && perception.FrontBlocked && perception.FrontDistance < 6f)
        {
            LastThrottleInput = 0f;
            LastBrakeInput = Mathf.Max(LastBrakeInput, 0.85f);
        }
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
