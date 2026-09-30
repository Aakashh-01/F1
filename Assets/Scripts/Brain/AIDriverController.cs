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

    [Header("Car Following")]
    [Tooltip("Gap a follower wants beyond the leader's car, in metres.")]
    [Range(0.5f, 12f)] public float followMinGap = 5f;
    [Tooltip("Seconds of headway a follower keeps on top of the minimum gap. This is what " +
             "scales the gap with speed: a car doing 300 km/h needs far more room than a car " +
             "in the slow corners, and a fixed gap guarantees contact at the fast end. Note the " +
             "sensor only reaches ~32 m, so at very high speed the desired gap is out of range " +
             "and the follower settles for holding the leader's speed instead of the full gap — " +
             "which is still a convoy rather than a pile-up.")]
    [Range(0.2f, 4f)] public float followTimeHeadway = 1.5f;
    [Tooltip("How much of a surplus gap a follower is allowed to eat, in km/h per metre. " +
             "Small values make the car sit back and take a pass slowly; large values make it " +
             "attack. Combined with the closing cap below this is what stops a car diving into " +
             "the back of a slower one.")]
    [Range(0.1f, 4f)] public float followApproachKmhPerMetre = 1f;
    [Tooltip("Hard ceiling on how much faster a follower may travel than the car ahead, km/h. " +
             "This is the guarantee that a follower never closes on a slower leader faster " +
             "than it can shed the difference.")]
    [Range(0f, 60f)] public float followMaxClosingKmh = 20f;
    [Tooltip("Speed, in km/h, below which this car and its leader both count as standing " +
             "still rather than queueing. A following law converts spare gap into a speed " +
             "advantage, which is right for a car in motion and useless on the grid: every " +
             "car is at a standstill, so every car is told it may only be a few km/h faster " +
             "than the car in front, and a field that cannot launch is a field that never " +
             "races. Below this pace, with the leader also below it, there is no closing to " +
             "manage and the car drives at its own target. The exemption ends the moment " +
             "either car is genuinely moving, so it is a launch, not a permanent blind spot.")]
    [Range(0f, 30f)] public float launchStandingSpeedKmh = 5f;

    [Header("Traffic Safety")]
    [Range(0f, 2f)] public float sideTrafficLaneHoldBuffer = 0.35f;
    [Range(0f, 1f)] public float sideTrafficTurnThrottleScale = 0.74f;
    [Range(0f, 0.5f)] public float sideTrafficTurnBrake = 0.12f;
    [Range(0f, 1f)] public float sideTrafficSteeringThreshold = 0.32f;
    [Range(0f, 1f)] public float packCornerCurvatureThreshold = 0.42f;
    [Range(0.2f, 1f)] public float packCornerSpeedScale = 0.82f;

    [Header("Emergency Braking")]
    [Tooltip("Floor on the emergency trigger distance, and its value at a standstill. Kept at " +
             "the old fixed 6 m on purpose: the trigger only ever grows with speed from here, " +
             "so no behaviour at rest is tightened, only the part that was genuinely too late.")]
    [Range(1f, 12f)] public float emergencyMinGap = 6f;
    [Tooltip("Seconds of headway reserved for the emergency stop. The old behaviour braked hard " +
             "at a fixed 6 m, which at racing speed is inside the car's own stopping distance — " +
             "by the time it triggered, contact was already unavoidable. The trigger distance " +
             "now grows with speed so the car starts stopping while it still can.")]
    [Range(0.1f, 4f)] public float emergencyTimeHeadway = 1.1f;
    [Tooltip("Extra metres per m/s of speed added to the emergency trigger, on top of the time " +
             "headway, to cover the drivetrain's brake spool-up.")]
    [Range(0f, 3f)] public float emergencySpeedMargin = 1.2f;
    [Tooltip("Closing speed, in km/h, below which a car ahead is a queue rather than a hazard. " +
             "The trigger is otherwise a pure distance test, and on the grid every car has a " +
             "stationary neighbour well inside it — so every car brakes at 0.85, no car moves, " +
             "no car's leader ever moves, and the whole field deadlocks on the line. Closing " +
             "speed is what actually decides whether a contact is imminent: two stationary cars " +
             "4 m apart are not about to collide. Immovable scenery is exempt from this, " +
             "because a wall does not need to move for you to hit it.")]
    [Range(0f, 30f)] public float emergencyClosingKmh = 4f;

    [Header("Reverse Guard")]
    [Tooltip("A wedged car only reverses if the space behind it is clear. Reversing into the " +
             "car following it is the worst possible way to lose a start, and on a dense grid " +
             "the car behind is always there.")]
    public bool requireRearClearanceToReverse = true;

    [Header("Side Overlap Escape")]
    [Tooltip("Seconds after the level loads during which a car still counts as being on the " +
             "grid, and the grid-launch exemption still applies to it. Measured on the clock " +
             "rather than on distance travelled: a distance test silently depends on the lap " +
             "tracker being bound, and a tracker that never bound reports zero forever, " +
             "which would hold the exemption open for the whole race.")]
    [Range(1f, 30f)] public float gridLaunchWindowSeconds = 8f;

    [Tooltip("How close a car alongside counts as contact rather than company. The car is " +
             "about 2 m wide, so anything under this is inside it.")]
    [Range(0.5f, 4f)] public float sideOverlapEscapeMetres = 1.8f;

    [Tooltip("How far to move sideways in one tick when escaping a contact, in metres.")]
    [Range(0.1f, 4f)] public float sideOverlapEscapeShift = 1.6f;

    [Tooltip("Speed floor a car in contact is allowed to keep. Enough to slide clear of a " +
             "car on top of it; the contact is mutual, so a follow-the-leader clamp would " +
             "otherwise hold both at a standstill with neither able to restart.")]
    [Range(0f, 80f)] public float overlapEscapeMinSpeedKmh = 40f;

    [Header("Track Recovery")]
    [Range(0f, 8f)] public float trackRecoveryMargin = 1.25f;

    [Tooltip("Speed an off-line car is held to while it rejoins. This used to be 95 km/h, " +
             "which is faster than most corners: a car that had drifted wide was therefore " +
             "steered back toward the racing line — where the traffic is — at near-racing " +
             "speed, with the side-traffic guard stood down. A recovering car is the least " +
             "controlled car on the track and it is the last one that should be flat out.")]
    [Range(20f, 180f)] public float trackRecoverySpeedKmh = 45f;

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
    [HideInInspector] public float LastFollowTargetKmh;
    [HideInInspector] public float LastDesiredGap;
    [HideInInspector] public float LastEmergencyDistance;
    [HideInInspector] public bool LastRearClearToReverse = true;
    [HideInInspector] public float LastLeaderSpeedKmh;
    [HideInInspector] public float LastClosingSpeedKmh;

    private AIDifficultyProfile _runtimeDifficulty;
    private bool _hasProgressIndex;
    private float _laneVariationSeed;
    private float _overtakeCommitTimer;
    private int _overtakeDirection;
    private bool _referencesResolved;
    private float _stuckTimer;
    private float _recoveryTimer;
    private float _recoveryCooldownTimer;

    /// <summary>
    /// Cached lap tracker, used only to tell a grid launch from a mid-race pile-up. Null is
    /// legitimate — a car that is not being timed simply keeps the old behaviour.
    /// </summary>
    private LapTracker _tracker;

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
        _tracker = GetComponent<LapTracker>();
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
        {
            perception.Tick(false);
            // The cast itself is throttled to a third of the physics rate to keep the box
            // casts affordable, but the leader's SPEED must not be throttled with it. The
            // following law runs every tick, so feeding it a leader reading that is up to
            // two fixed steps stale would hand it a gap error in exactly the direction that
            // closes the gap.
            perception.SampleLeaderState();
        }

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

        // A brake that is holding position must not be read as a request to reverse. The
        // drivetrain treats "brake held, no throttle, stationary" as reverse, so without this
        // an AI that eases off behind traffic drives backwards into the car behind it — and
        // because a whole pack brakes together, one car stopping reverses everything behind
        // it into itself. The one exception is the stuck-recovery manoeuvre, which means to
        // reverse on purpose and has already checked the space behind it.
        //
        // Set here, after the inputs are final and on the driver that owns them, rather than
        // in the drivetrain, so the player's ordinary reversing is untouched.
        coordinator.SuppressReverse = !IsDeliberatelyReversing;

        coordinator.SetExternalInput(LastSteeringInput, LastThrottleInput, LastBrakeInput);
    }

    /// <summary>
    /// True when the brake currently being commanded is a deliberate reverse request rather
    /// than a hold. Only the stuck-recovery manoeuvre qualifies.
    /// </summary>
    private bool IsDeliberatelyReversing => InStuckRecovery && LastRearClearToReverse;

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

        // A car held at the start gate wants to move, is not moving, and is on its line.
        // Every one of those is true of every car on the grid, so the detector below would
        // fire for the whole field during the countdown and the entire grid would reverse
        // the instant the lights went out. Being parked is not being stuck.
        if (coordinator.InputLocked)
        {
            _stuckTimer = 0f;
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
        // The reverse manoeuvre backs the car up, so it must know what is behind it. The
        // guard above stops a car that is merely held up in traffic from recovering at all,
        // but a genuinely wedged car still recovers, and on a packed grid the car it would
        // back into is a car following it, not empty air. With a blocked rear the recovery
        // is abandoned rather than driven into the follower; the next attempt waits out the
        // cooldown, by which point the situation usually has changed.
        LastRearClearToReverse = !requireRearClearanceToReverse
            || perception == null
            || !perception.RearBlocked;

        if (!LastRearClearToReverse)
        {
            InStuckRecovery = false;
            _recoveryTimer = 0f;
            _stuckTimer = 0f;
            _recoveryCooldownTimer = recoveryCooldownSeconds;
            // Hold position rather than reverse. The brake still has to mean "stop", which
            // is what SuppressReverse is for.
            LastThrottleInput = 0f;
            LastBrakeInput = 1f;
            return;
        }

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

                // Relative to where this car already is, not an absolute lane.
                //
                // The overtake offset used to BE the target offset, so a car already sitting
                // at +3.68 m on its racing line and passing to the right was asked to go to
                // +5 m — a 1.3 m move, against a car 2.016 m wide. It was asked to pass
                // through the car it was passing. Asking for a lane relative to the CURRENT
                // offset, clamped to the usable track, means the move is the width of a lane
                // change in the chosen direction and never smaller than the car being passed.
                float target = CurrentLaneOffset + overtakeOffset * selectedDirection;
                DesiredLaneOffset = Mathf.Clamp(target, -usableLaneLimit, usableLaneLimit);
                IsOvertaking = true;
            }
        }

        ApplySideTrafficLaneGuard();
        CurrentLaneOffset = Mathf.MoveTowards(
            CurrentLaneOffset,
            DesiredLaneOffset,
            laneOffsetMoveSpeed * Time.fixedDeltaTime);
    }

    /// <summary>
    /// Whether this car is still on or beside its starting slot.
    ///
    /// The grid-launch exemption is only valid before the field has actually gone racing.
    /// As a bare speed test it was not a grid rule at all, it was a rule about being slow —
    /// and two cars that have just shunted are also slow, so both would compute the
    /// exemption together and both would accelerate to full target with a car in front of
    /// them and no caution applied.
    ///
    /// Timed rather than measured. Distance would be the more elegant test, and it was the
    /// first attempt, but it depends on the lap tracker actually accumulating — and a car
    /// whose tracker never bound a track reports zero distance forever, which would leave
    /// the exemption permanently on and the caution permanently off. That is the failure
    /// this is meant to prevent, reintroduced by a subtler route. Elapsed time cannot lie
    /// that way: it is measured by the clock, and it ends.
    /// </summary>
    private bool IsStillOnTheGrid()
    {
        return Time.timeSinceLevelLoad < gridLaunchWindowSeconds;
    }

    private void ApplySideTrafficLaneGuard()
    {
        if (perception == null)
            return;

        ApplySideOverlapEscape();

        // A car that is off its line and being steered back to the racing line is the one
        // most likely to meet traffic, because the racing line is where the traffic is. The
        // guard used to stand down entirely for these cars on the assumption that a
        // recovering driver has other problems. It does have other problems; it does not
        // have a spare one for driving into a rival.
        if (IsRecoveringTrack)
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

    /// <summary>
    /// Steers a car that is already touching a rival away from it.
    ///
    /// The lane guard above is a brake, not a steering wheel: it refuses to move a car
    /// <i>towards</i> something it can see beside it. That is right until the two are
    /// already overlapping, at which point it becomes actively harmful. Both cars then
    /// report the other as side-blocked on both sides — the cast starts inside the
    /// neighbour — so both clamp their lane offset to where they already are, both refuse
    /// to move, and the pair is locked together by the very evidence of the contact. There
    /// is no force acting to separate them and no input that would let them try, so the
    /// pack grinds along interpenetrating until something else disturbs it.
    ///
    /// So the very close case is inverted: instead of freezing, the car is told to move
    /// away from the contact. Distance decides which way, because when a car is on top of
    /// us the side casts are both reporting, and "which neighbour is further away" is the
    /// only question that still has an answer. The overtake is abandoned, because a pass
    /// that has become a shunt is not a pass.
    /// </summary>
    private void ApplySideOverlapEscape()
    {
        if (!IsSideOverlapContact())
            return;

        // One side, or both: head for whichever gap is larger, so the car always has a
        // direction that increases the separation rather than trading one contact for
        // another.
        int escapeDirection;
        if (IsRightOverlapClose() && IsLeftOverlapClose())
            escapeDirection = perception.RightDistance >= perception.LeftDistance ? -1 : 1;
        else
            escapeDirection = IsRightOverlapClose() ? -1 : 1;

        // Negative lane offset is to the left, matching the racing line's own sign
        // convention used by SelectOvertakeDirection.
        float escapeOffset = CurrentLaneOffset + escapeDirection * sideOverlapEscapeShift;
        DesiredLaneOffset = Mathf.Clamp(escapeOffset, -LastLaneLimit, LastLaneLimit);

        // Deliberately overrides the guard above for this tick. The guard will re-assert
        // itself next tick if the cars have separated by then, which is the behaviour we
        // want: escape until clear, then resume racing.
        _overtakeDirection = 0;
        _overtakeCommitTimer = 0f;
        IsOvertaking = false;
    }

    /// <summary>
    /// Whether a car is close enough alongside to count as contact rather than company.
    /// </summary>
    private bool IsSideOverlapContact()
    {
        if (perception == null) return false;
        return IsLeftOverlapClose() || IsRightOverlapClose();
    }

    private bool IsLeftOverlapClose()
    {
        return perception != null
            && perception.LeftBlocked
            && perception.LeftDistance < sideOverlapEscapeMetres;
    }

    private bool IsRightOverlapClose()
    {
        return perception != null
            && perception.RightBlocked
            && perception.RightDistance < sideOverlapEscapeMetres;
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
            // Descending ramp: blend is 0 at the far end of the sensor's reach and 1 when the
            // obstacle is on the bumper. The argument order looks inverted and is not —
            // InverseLerp(a, b, v) is Clamp01((v - a) / (b - a)), so passing the FAR distance
            // as `a` and the NEAR one as `b` is what makes the blend rise as the car
            // approaches. Written the other way round it would read the sensor backwards and
            // accelerate into a car 1 m away.
            float distanceBlend = Mathf.InverseLerp(perception.forwardDistance, 3f, perception.FrontDistance);

            float cautionTarget;

            // A standing start is not a traffic jam, and the following law cannot tell the
            // difference on its own. It converts spare gap into a speed advantage, which is
            // the right model for a car in motion and the wrong one for the grid: on a
            // two-wide grid the car ahead is 8 m away and stationary, so the surplus over the
            // desired gap is about 3 m, and 3 m buys 3 km/h. Every car is therefore told it
            // may only crawl, and because the blend term pulls the target toward zero on top
            // of that, the measured target on the line was under 1 km/h. A field that cannot
            // leave the line is not racing, and no amount of tuning the gap fixes it — the
            // model is answering a question nobody asked.
            //
            // So: below a walking pace, with the leader also below a walking pace, there is
            // no closing to manage and no gap being eaten. The car drives at its own target,
            // exactly as it would on an empty track. The exemption is self-cancelling the
            // moment either car moves, so the first car to accelerate hands the whole thing
            // back to the following law while there is still a real gap between them.
            //
            // The "still on the grid" clause is what keeps that exemption honest. As a bare
            // speed test it was not a grid rule at all, it was a rule about being slow, and
            // being slow has other causes: two cars that have just shunted are also below
            // 5 km/h, and both would then compute standingStart together and both would
            // accelerate to full target with a car in front of them and no caution applied.
            // The exemption now requires the car to be one that has not yet left its grid
            // slot, which is the situation it was written for.
            bool standingStart = speedKmh < launchStandingSpeedKmh
                              && perception.LeaderSpeedKmh < launchStandingSpeedKmh
                              && IsStillOnTheGrid();

            if (standingStart)
            {
                cautionTarget = targetSpeedKmh;
                LastFollowTargetKmh = cautionTarget;
            }
            else if (perception.LeaderIsMovable)
            {
                // Car following. This is the part the distance ramp cannot do.
                //
                // The ramp is a function of distance alone, so it is blind to the two things
                // that decide whether a rear-end happens: how fast the car in front is going,
                // and how fast this car is closing on it. Every car in a pack sees the same
                // distances, so every car computes the same target, they all accelerate to it
                // together, and the pack folds up on itself. There was no model of the leader
                // at all — the sensor recorded ClosestObstacle and nothing ever read it.
                //
                // When the leader's speed IS known it is strictly better information than a
                // ramp, so it replaces the ramp rather than merely being an extra term. That
                // also matters for pace: the old ramp pulled any car within sensor range down
                // toward blockedTargetSpeedKmh no matter what the leader was doing, which is
                // safe but means a whole field crawls nose to tail and never actually races.
                // The following target holds the leader's speed and closes only at a capped
                // rate, so a pack forms up and runs instead of forming a conga line.
                //
                // The ramp is still applied, squared, so it only bites over the last few
                // metres — enough to peel the car off a bumper, not enough to strangle the
                // approach for the whole 32 m.
                float followTarget = CalculateFollowTargetKmh(speedKmh);
                cautionTarget = Mathf.Lerp(followTarget, 0f, distanceBlend * distanceBlend);
                LastFollowTargetKmh = cautionTarget;
            }
            else
            {
                // Static scenery has no speed to follow, so the distance ramp is all there is.
                cautionTarget = Mathf.Lerp(blockedTargetSpeedKmh, 0f, distanceBlend * difficulty.avoidanceCaution);
                LastFollowTargetKmh = cautionTarget;
            }

            // An overtake eases the clamp, but it must never RAISE it.
            //
            // This used to blend the caution speed back toward the unblocked target by
            // overtakeTrafficSlowdownScale, which meant the act of deciding to pass undone the
            // very reaction that noticed the car being passed. Worked through on a Hard car
            // with a 15 m gap on a straight: the leader braked to 36 km/h while the overtaker
            // — seeing the same car at the same distance — commanded 196 km/h, a 44 m/s
            // closing speed and contact in about a third of a second. The faster a driver was
            // willing to overtake, the harder it drove into the car it was overtaking, which
            // is backwards.
            //
            // Easing now works the other way: the overtake still relaxes the clamp, but only
            // toward the target it is trying to reach, and never above it. A driver that has
            // committed to a pass therefore closes at a speed it can still shed, and a driver
            // that has NOT committed gets the full caution.
            //
            // The guard used to be Mathf.Min(cautionTarget, eased), which made this whole
            // branch a no-op. Lerp(cautionTarget, targetSpeedKmh, t) with t in [0,1] returns
            // a value inside [cautionTarget, targetSpeedKmh], and targetSpeedKmh is above
            // cautionTarget whenever there is traffic to ease out of — so `eased` was always
            // >= cautionTarget, and Min handed back cautionTarget unchanged. A committed
            // overtaker and a car that gave up were clamped to exactly the same speed; the
            // decision to pass changed the lane and nothing else. The cap that actually
            // matters is the one at the speedTarget assignment below, which already holds the
            // result to targetSpeedKmh.
            if (IsOvertaking)
            {
                float eased = Mathf.Lerp(cautionTarget, targetSpeedKmh, overtakeTrafficSlowdownScale);
                cautionTarget = Mathf.Min(eased, targetSpeedKmh);
            }

            // A car that is already touching a rival is not following it, and must not be
            // treated as though it were.
            //
            // Two cars that come together each see the other as a stationary leader directly
            // ahead, so the following law returns the leader's speed — zero — and the caution
            // collapses to a standstill. Both hold full caution against each other, both stop,
            // and neither can restart: each is waiting for the other. That is the state the
            // lateral escape cannot leave on its own, because separating sideways while
            // commanded to stand still is not separating.
            //
            // So contact overrides the clamp with a floor rather than removing it. A modest
            // speed is enough to slide out from under a car that is on top of you, and it is
            // not enough to convert a shunt into a pile-on. It applies only while a car is
            // genuinely alongside at contact range, so a normal overtake — a metre or two of
            // clearance — is completely unaffected.
            if (IsSideOverlapContact())
                cautionTarget = Mathf.Max(cautionTarget, overlapEscapeMinSpeedKmh);

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

        if (perception != null && perception.FrontBlocked)
        {
            // The emergency trigger used to be a flat 6 m. That number is wrong at racing
            // speed: this drivetrain needs ~130-170 ms to reach full brake (the brake channel
            // spools, and the advanced brake ramps its pressure curve), and a car 40 ms stale
            // on its sensor has already closed. Together that is ~14 m of travel between
            // "the gap is measurable" and "the brake is fully on", so 6 m arrives after
            // contact was already unavoidable — the emergency could only ever describe a
            // collision that had happened.
            //
            // The trigger now scales with speed, so it fires early enough to still do
            // something, and it is capped at the sensor's reach so a fast car falls back on
            // the following law above rather than pretending it can see past its own sensor.
            float emergencyDistance = GetEmergencyDistanceMetres(speedKmh);
            LastEmergencyDistance = emergencyDistance;

            // Scaling the trigger with speed is only half the fix. Above about 40 km/h the
            // speed-scaled distance exceeds the sensor's own reach, so the cap binds and the
            // trigger becomes "is there anything at all inside 32 m" — which is not a hazard
            // test, it is a presence test. It never consulted the leader's speed, so a car
            // running alongside a rival doing 200 km/h on a straight got a full 0.85 brake
            // application, and at the line every stationary car braked for its stationary
            // neighbour and the field never started at all.
            //
            // What decides a contact is closing speed, and the sensor already measures it.
            // LastClosingSpeedKmh was being recorded for exactly this and read by nothing.
            // Scenery keeps the pure distance test, because a wall does not have to move for
            // you to hit it.
            bool needsRiskCheck = perception.LeaderIsMovable;
            bool risky = !needsRiskCheck || perception.ClosingSpeedKmh > emergencyClosingKmh;

            if (perception.FrontDistance < emergencyDistance && risky)
            {
                LastThrottleInput = 0f;
                LastBrakeInput = Mathf.Max(LastBrakeInput, 0.85f);
                LastSpeedClampReason = AISpeedClampReason.EmergencyBrake;
            }
        }
        else
        {
            LastEmergencyDistance = 0f;
        }

        if (IsRecoveringTrack && LastSpeedClampReason == AISpeedClampReason.None)
            LastSpeedClampReason = AISpeedClampReason.TrackRecovery;
    }

    /// <summary>
    /// Speed the follower is allowed to hold given the leader ahead of it.
    ///
    /// The law is deliberately simple and has one hard guarantee: the follower never travels
    /// more than <see cref="followMaxClosingKmh"/> faster than the car in front, and never
    /// faster at all once the gap is at or under the desired headway. Both halves matter —
    /// without the cap a car with room in hand accelerates into the back of a slower one, and
    /// without the headway term the cap alone still lets the pack drift together.
    /// </summary>
    private float CalculateFollowTargetKmh(float speedKmh)
    {
        float desiredGap = followMinGap + followTimeHeadway * (speedKmh / 3.6f);
        float surplus = perception.FrontDistance - desiredGap;
        LastDesiredGap = desiredGap;
        LastLeaderSpeedKmh = perception.LeaderSpeedKmh;
        LastClosingSpeedKmh = perception.ClosingSpeedKmh;

        float allowanceKmh = Mathf.Max(0f, surplus) * followApproachKmhPerMetre;
        allowanceKmh = Mathf.Min(allowanceKmh, followMaxClosingKmh);

        // A leader already rolling backwards relative to the track (a car slowing for a
        // corner, or one that has spun) must not be allowed to drag this car below a crawl
        // and into a standstill, where the brake becomes a reverse request again.
        float leaderFloor = Mathf.Max(0f, perception.LeaderSpeedKmh);
        return Mathf.Max(0f, leaderFloor + allowanceKmh);
    }

    private float GetEmergencyDistanceMetres(float speedKmh)
    {
        float speedMs = Mathf.Max(0f, speedKmh / 3.6f);
        float distance = emergencyMinGap + emergencyTimeHeadway * speedMs + emergencySpeedMargin * speedMs;
        float sensorReach = perception != null ? perception.forwardDistance : 32f;
        return Mathf.Min(distance, sensorReach);
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
