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

    /// <summary>
    /// Every tunable this driver uses, in one asset.
    ///
    /// <para>
    /// This is the only place AI tuning is authored. It used to be fifty-two public fields
    /// on this component, and the AI car prefab serialised overrides for thirty of them, so
    /// the prefab won every time and the values in the code were decorative. See the block
    /// comment above the tuning properties for the full account.
    /// </para>
    /// </summary>
    public AIDriverProfile driverProfile;

    [Header("Difficulty")]
    public AIDifficultyPreset difficultyPreset = AIDifficultyPreset.Medium;

    // ---------------------------------------------------------------------
    // Tuning
    //
    // Every value below is a property, not a field. They used to be public fields on this
    // component, which meant Unity could serialise an override for any of them into
    // AI_F1_Body.prefab — and it had, for thirty of them. Unity deserialisation prefers the
    // prefab, so the code below these declarations was not what the cars ran on, and
    // changing a default here silently did nothing. The proof that this was not
    // hypothetical: trackRecoverySpeedKmh carries a tooltip saying 95 km/h was a bug and
    // this value is 45, while the prefab still said 95. A documented fix that had never
    // been in effect, because a field is a value a prefab is allowed to overrule and a
    // property is not.
    //
    // The values now live in AIDriverProfile and are read through it, so there is exactly
    // one place to tune and nothing left to shadow it. The prefab serialises a single
    // reference to the profile instead.
    //
    // The setters exist for the callers that legitimately want a per-car value: the grid
    // gives each entry its own lane offset and overtake side, and the safety-law tests pin
    // values to build a scenario. Writing sets an override on THIS instance only; reading
    // returns the override when one was set and the profile's value otherwise. That is
    // deliberately not the same as writing to the profile — a shared asset a test could
    // mutate would leak its scenario into every other car in the session, and would dirty
    // the asset on disk.
    //
    // Nullable-per-field rather than one override object holding 52 flags. It is one
    // branch per read and no allocation, and these are read several times per physics
    // tick; a dictionary or a boxed "has override" struct would both cost more than the
    // thing they are optimising.
    //
    // The [Tooltip]s that used to sit on these fields moved with the values, onto the
    // profile, where the inspector now reads them from. They are not repeated here:
    // properties are not serialised, so a tooltip on one would never be displayed.
    // ---------------------------------------------------------------------


    // Path Following
    public float baseLookaheadDistance
    {
        get => _overrideBaseLookaheadDistance ?? Tuning.baseLookaheadDistance;
        set => _overrideBaseLookaheadDistance = value;
    }
    public float lookaheadPerKmh
    {
        get => _overrideLookaheadPerKmh ?? Tuning.lookaheadPerKmh;
        set => _overrideLookaheadPerKmh = value;
    }
    public int curvatureLookaheadSteps
    {
        get => _overrideCurvatureLookaheadSteps ?? Tuning.curvatureLookaheadSteps;
        set => _overrideCurvatureLookaheadSteps = value;
    }
    public float steeringAngleForFullInput
    {
        get => _overrideSteeringAngleForFullInput ?? Tuning.steeringAngleForFullInput;
        set => _overrideSteeringAngleForFullInput = value;
    }
    public float laneOffsetMoveSpeed
    {
        get => _overrideLaneOffsetMoveSpeed ?? Tuning.laneOffsetMoveSpeed;
        set => _overrideLaneOffsetMoveSpeed = value;
    }
    public float waypointReachDistance
    {
        get => _overrideWaypointReachDistance ?? Tuning.waypointReachDistance;
        set => _overrideWaypointReachDistance = value;
    }
    public int forwardProgressSearchSteps
    {
        get => _overrideForwardProgressSearchSteps ?? Tuning.forwardProgressSearchSteps;
        set => _overrideForwardProgressSearchSteps = value;
    }

    // Longitudinal Planner
    public float fallbackBrakeDecelMs2
    {
        get => _overrideFallbackBrakeDecelMs2 ?? Tuning.fallbackBrakeDecelMs2;
        set => _overrideFallbackBrakeDecelMs2 = value;
    }
    public float minimumBrakingHorizonMetres
    {
        get => _overrideMinimumBrakingHorizonMetres ?? Tuning.minimumBrakingHorizonMetres;
        set => _overrideMinimumBrakingHorizonMetres = value;
    }
    public int maxBrakingHorizonWaypoints
    {
        get => _overrideMaxBrakingHorizonWaypoints ?? Tuning.maxBrakingHorizonWaypoints;
        set => _overrideMaxBrakingHorizonWaypoints = value;
    }
    public float brakeMeasurementMinSpeedKmh
    {
        get => _overrideBrakeMeasurementMinSpeedKmh ?? Tuning.brakeMeasurementMinSpeedKmh;
        set => _overrideBrakeMeasurementMinSpeedKmh = value;
    }
    public float brakeTrimSeconds
    {
        get => _overrideBrakeTrimSeconds ?? Tuning.brakeTrimSeconds;
        set => _overrideBrakeTrimSeconds = value;
    }

    // Speed Control
    public float throttleFullErrorKmh
    {
        get => _overrideThrottleFullErrorKmh ?? Tuning.throttleFullErrorKmh;
        set => _overrideThrottleFullErrorKmh = value;
    }
    public float brakeFullErrorKmh
    {
        get => _overrideBrakeFullErrorKmh ?? Tuning.brakeFullErrorKmh;
        set => _overrideBrakeFullErrorKmh = value;
    }
    public float blockedTargetSpeedKmh
    {
        get => _overrideBlockedTargetSpeedKmh ?? Tuning.blockedTargetSpeedKmh;
        set => _overrideBlockedTargetSpeedKmh = value;
    }
    public float steeringThrottleReduction
    {
        get => _overrideSteeringThrottleReduction ?? Tuning.steeringThrottleReduction;
        set => _overrideSteeringThrottleReduction = value;
    }

    // Competitive Pace
    public float baseCornerCautionStrength
    {
        get => _overrideBaseCornerCautionStrength ?? Tuning.baseCornerCautionStrength;
        set => _overrideBaseCornerCautionStrength = value;
    }
    public float competitiveCornerCautionStrength
    {
        get => _overrideCompetitiveCornerCautionStrength ?? Tuning.competitiveCornerCautionStrength;
        set => _overrideCompetitiveCornerCautionStrength = value;
    }
    public float overtakeSpeedBoost
    {
        get => _overrideOvertakeSpeedBoost ?? Tuning.overtakeSpeedBoost;
        set => _overrideOvertakeSpeedBoost = value;
    }
    public float overtakeTrafficSlowdownScale
    {
        get => _overrideOvertakeTrafficSlowdownScale ?? Tuning.overtakeTrafficSlowdownScale;
        set => _overrideOvertakeTrafficSlowdownScale = value;
    }
    public float overtakeSteeringThrottleScale
    {
        get => _overrideOvertakeSteeringThrottleScale ?? Tuning.overtakeSteeringThrottleScale;
        set => _overrideOvertakeSteeringThrottleScale = value;
    }

    // Overtaking
    public bool preferRightOvertake
    {
        get => _overridePreferRightOvertake ?? Tuning.preferRightOvertake;
        set => _overridePreferRightOvertake = value;
    }
    public float overtakeTrigger
    {
        get => _overrideOvertakeTrigger ?? Tuning.overtakeTrigger;
        set => _overrideOvertakeTrigger = value;
    }
    public float overtakeCommitSeconds
    {
        get => _overrideOvertakeCommitSeconds ?? Tuning.overtakeCommitSeconds;
        set => _overrideOvertakeCommitSeconds = value;
    }

    // Race Craft
    public float preferredLaneOffset01
    {
        get => _overridePreferredLaneOffset01 ?? Tuning.preferredLaneOffset01;
        set => _overridePreferredLaneOffset01 = value;
    }
    public float freeTrackLaneUse
    {
        get => _overrideFreeTrackLaneUse ?? Tuning.freeTrackLaneUse;
        set => _overrideFreeTrackLaneUse = value;
    }
    public float waypointLaneInfluence
    {
        get => _overrideWaypointLaneInfluence ?? Tuning.waypointLaneInfluence;
        set => _overrideWaypointLaneInfluence = value;
    }
    public float laneEdgeSafetyMargin
    {
        get => _overrideLaneEdgeSafetyMargin ?? Tuning.laneEdgeSafetyMargin;
        set => _overrideLaneEdgeSafetyMargin = value;
    }
    public float laneVariationStrength
    {
        get => _overrideLaneVariationStrength ?? Tuning.laneVariationStrength;
        set => _overrideLaneVariationStrength = value;
    }
    public float laneVariationFrequency
    {
        get => _overrideLaneVariationFrequency ?? Tuning.laneVariationFrequency;
        set => _overrideLaneVariationFrequency = value;
    }

    // Car Following
    public float followMinGap
    {
        get => _overrideFollowMinGap ?? Tuning.followMinGap;
        set => _overrideFollowMinGap = value;
    }
    public float followTimeHeadway
    {
        get => _overrideFollowTimeHeadway ?? Tuning.followTimeHeadway;
        set => _overrideFollowTimeHeadway = value;
    }
    public float followApproachKmhPerMetre
    {
        get => _overrideFollowApproachKmhPerMetre ?? Tuning.followApproachKmhPerMetre;
        set => _overrideFollowApproachKmhPerMetre = value;
    }
    public float followMaxClosingKmh
    {
        get => _overrideFollowMaxClosingKmh ?? Tuning.followMaxClosingKmh;
        set => _overrideFollowMaxClosingKmh = value;
    }
    public float launchStandingSpeedKmh
    {
        get => _overrideLaunchStandingSpeedKmh ?? Tuning.launchStandingSpeedKmh;
        set => _overrideLaunchStandingSpeedKmh = value;
    }

    // Traffic Safety
    public float sideTrafficLaneHoldBuffer
    {
        get => _overrideSideTrafficLaneHoldBuffer ?? Tuning.sideTrafficLaneHoldBuffer;
        set => _overrideSideTrafficLaneHoldBuffer = value;
    }
    public float sideTrafficTurnThrottleScale
    {
        get => _overrideSideTrafficTurnThrottleScale ?? Tuning.sideTrafficTurnThrottleScale;
        set => _overrideSideTrafficTurnThrottleScale = value;
    }
    public float sideTrafficTurnBrake
    {
        get => _overrideSideTrafficTurnBrake ?? Tuning.sideTrafficTurnBrake;
        set => _overrideSideTrafficTurnBrake = value;
    }
    public float sideTrafficSteeringThreshold
    {
        get => _overrideSideTrafficSteeringThreshold ?? Tuning.sideTrafficSteeringThreshold;
        set => _overrideSideTrafficSteeringThreshold = value;
    }
    public float packCornerCurvatureThreshold
    {
        get => _overridePackCornerCurvatureThreshold ?? Tuning.packCornerCurvatureThreshold;
        set => _overridePackCornerCurvatureThreshold = value;
    }
    public float packCornerSpeedScale
    {
        get => _overridePackCornerSpeedScale ?? Tuning.packCornerSpeedScale;
        set => _overridePackCornerSpeedScale = value;
    }

    // Emergency Braking
    public float emergencyMinGap
    {
        get => _overrideEmergencyMinGap ?? Tuning.emergencyMinGap;
        set => _overrideEmergencyMinGap = value;
    }
    public float emergencyTimeHeadway
    {
        get => _overrideEmergencyTimeHeadway ?? Tuning.emergencyTimeHeadway;
        set => _overrideEmergencyTimeHeadway = value;
    }
    public float emergencySpeedMargin
    {
        get => _overrideEmergencySpeedMargin ?? Tuning.emergencySpeedMargin;
        set => _overrideEmergencySpeedMargin = value;
    }
    public float emergencyClosingKmh
    {
        get => _overrideEmergencyClosingKmh ?? Tuning.emergencyClosingKmh;
        set => _overrideEmergencyClosingKmh = value;
    }

    // Reverse Guard
    public bool requireRearClearanceToReverse
    {
        get => _overrideRequireRearClearanceToReverse ?? Tuning.requireRearClearanceToReverse;
        set => _overrideRequireRearClearanceToReverse = value;
    }

    // Side Overlap Escape
    public float gridLaunchWindowSeconds
    {
        get => _overrideGridLaunchWindowSeconds ?? Tuning.gridLaunchWindowSeconds;
        set => _overrideGridLaunchWindowSeconds = value;
    }
    public float sideOverlapEscapeMetres
    {
        get => _overrideSideOverlapEscapeMetres ?? Tuning.sideOverlapEscapeMetres;
        set => _overrideSideOverlapEscapeMetres = value;
    }
    public float sideOverlapEscapeShift
    {
        get => _overrideSideOverlapEscapeShift ?? Tuning.sideOverlapEscapeShift;
        set => _overrideSideOverlapEscapeShift = value;
    }
    public float overlapEscapeMinSpeedKmh
    {
        get => _overrideOverlapEscapeMinSpeedKmh ?? Tuning.overlapEscapeMinSpeedKmh;
        set => _overrideOverlapEscapeMinSpeedKmh = value;
    }

    // Track Recovery
    public float trackRecoveryMargin
    {
        get => _overrideTrackRecoveryMargin ?? Tuning.trackRecoveryMargin;
        set => _overrideTrackRecoveryMargin = value;
    }
    public float trackRecoverySpeedKmh
    {
        get => _overrideTrackRecoverySpeedKmh ?? Tuning.trackRecoverySpeedKmh;
        set => _overrideTrackRecoverySpeedKmh = value;
    }

    // Stuck Recovery
    public float stuckSpeedThresholdKmh
    {
        get => _overrideStuckSpeedThresholdKmh ?? Tuning.stuckSpeedThresholdKmh;
        set => _overrideStuckSpeedThresholdKmh = value;
    }
    public float stuckDetectionSeconds
    {
        get => _overrideStuckDetectionSeconds ?? Tuning.stuckDetectionSeconds;
        set => _overrideStuckDetectionSeconds = value;
    }
    public float recoveryReverseSeconds
    {
        get => _overrideRecoveryReverseSeconds ?? Tuning.recoveryReverseSeconds;
        set => _overrideRecoveryReverseSeconds = value;
    }
    public float recoveryCooldownSeconds
    {
        get => _overrideRecoveryCooldownSeconds ?? Tuning.recoveryCooldownSeconds;
        set => _overrideRecoveryCooldownSeconds = value;
    }
    public float stuckThrottleDemandThreshold
    {
        get => _overrideStuckThrottleDemandThreshold ?? Tuning.stuckThrottleDemandThreshold;
        set => _overrideStuckThrottleDemandThreshold = value;
    }

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
    [HideInInspector] public float LastPlannedSpeedKmh;
    [HideInInspector] public float LastBrakeDecelMs2;
    [HideInInspector] public float LastBrakingHorizonMetres;
    [HideInInspector] public int LastBrakingHorizonWaypoints;

    private AIDifficultyProfile _runtimeDifficulty;
    private bool _hasProgressIndex;
    private float _laneVariationSeed;
    private float _overtakeCommitTimer;
    private int _overtakeDirection;
    private bool _referencesResolved;
    private float _stuckTimer;
    private float _recoveryTimer;
    private float _recoveryCooldownTimer;

    private AIDriverProfile _runtimeTuning;

    private AILongitudinalPlanner _planner = new AILongitudinalPlanner();

    /// <summary>
    /// Per-instance overrides written through the tuning properties' setters. Null means
    /// "this driver uses the profile's value", which is the state every car starts in.
    /// </summary>
    private float? _overrideBaseLookaheadDistance;
    private float? _overrideLookaheadPerKmh;
    private int? _overrideCurvatureLookaheadSteps;
    private float? _overrideSteeringAngleForFullInput;
    private float? _overrideLaneOffsetMoveSpeed;
    private float? _overrideWaypointReachDistance;
    private int? _overrideForwardProgressSearchSteps;
    private float? _overrideFallbackBrakeDecelMs2;
    private float? _overrideMinimumBrakingHorizonMetres;
    private int? _overrideMaxBrakingHorizonWaypoints;
    private float? _overrideBrakeMeasurementMinSpeedKmh;
    private float? _overrideBrakeTrimSeconds;
    private float? _overrideThrottleFullErrorKmh;
    private float? _overrideBrakeFullErrorKmh;
    private float? _overrideBlockedTargetSpeedKmh;
    private float? _overrideSteeringThrottleReduction;
    private float? _overrideBaseCornerCautionStrength;
    private float? _overrideCompetitiveCornerCautionStrength;
    private float? _overrideOvertakeSpeedBoost;
    private float? _overrideOvertakeTrafficSlowdownScale;
    private float? _overrideOvertakeSteeringThrottleScale;
    private bool? _overridePreferRightOvertake;
    private float? _overrideOvertakeTrigger;
    private float? _overrideOvertakeCommitSeconds;
    private float? _overridePreferredLaneOffset01;
    private float? _overrideFreeTrackLaneUse;
    private float? _overrideWaypointLaneInfluence;
    private float? _overrideLaneEdgeSafetyMargin;
    private float? _overrideLaneVariationStrength;
    private float? _overrideLaneVariationFrequency;
    private float? _overrideFollowMinGap;
    private float? _overrideFollowTimeHeadway;
    private float? _overrideFollowApproachKmhPerMetre;
    private float? _overrideFollowMaxClosingKmh;
    private float? _overrideLaunchStandingSpeedKmh;
    private float? _overrideSideTrafficLaneHoldBuffer;
    private float? _overrideSideTrafficTurnThrottleScale;
    private float? _overrideSideTrafficTurnBrake;
    private float? _overrideSideTrafficSteeringThreshold;
    private float? _overridePackCornerCurvatureThreshold;
    private float? _overridePackCornerSpeedScale;
    private float? _overrideEmergencyMinGap;
    private float? _overrideEmergencyTimeHeadway;
    private float? _overrideEmergencySpeedMargin;
    private float? _overrideEmergencyClosingKmh;
    private bool? _overrideRequireRearClearanceToReverse;
    private float? _overrideGridLaunchWindowSeconds;
    private float? _overrideSideOverlapEscapeMetres;
    private float? _overrideSideOverlapEscapeShift;
    private float? _overrideOverlapEscapeMinSpeedKmh;
    private float? _overrideTrackRecoveryMargin;
    private float? _overrideTrackRecoverySpeedKmh;
    private float? _overrideStuckSpeedThresholdKmh;
    private float? _overrideStuckDetectionSeconds;
    private float? _overrideRecoveryReverseSeconds;
    private float? _overrideRecoveryCooldownSeconds;
    private float? _overrideStuckThrottleDemandThreshold;

    /// <summary>
    /// Cached lap tracker, used only to tell a grid launch from a mid-race pile-up. Null is
    /// legitimate — a car that is not being timed simply keeps the old behaviour.
    /// </summary>
    private LapTracker _tracker;

    /// <summary>
    /// The profile the tunables read, or a cached runtime default when none is assigned.
    ///
    /// <para>
    /// Cached for the same reason <see cref="Difficulty"/> caches its fallback: these reads
    /// happen several times per physics tick per car, and <c>CreateInstance</c> per read
    /// would allocate a ScriptableObject per property per frame. Unlike the difficulty
    /// fallback there is no preset to invalidate against, so the cache is written once and
    /// never rebuilt.
    /// </para>
    ///
    /// <para>
    /// The fallback matters because a car with no profile must still drive. These
    /// properties are read unconditionally, several of them inside
    /// <see cref="CalculateInputs"/>, so a null here would surface as a
    /// <c>NullReferenceException</c> thrown from the middle of a physics step rather than
    /// as an inspector warning someone would notice.
    /// </para>
    /// </summary>
    private AIDriverProfile Tuning
    {
        get
        {
            if (driverProfile != null)
                return driverProfile;

            if (_runtimeTuning == null)
                _runtimeTuning = AIDriverProfile.CreateRuntimeProfile();

            return _runtimeTuning;
        }
    }

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

        // The single waypoint ahead is NOT the speed the car may carry. It is the speed of a
        // point about 11 m away, and the corner that actually governs the next few seconds may
        // be 120 m further on. Taking it directly is why every corner used to arrive as a wall:
        // the AI could not see the braking zone until it was already inside it, so the only
        // available response was to saturate the brake and oscillate.
        //
        // The planner instead takes the MINIMUM over every waypoint within a braking horizon —
        // a distance derived from the speed the car actually has and the braking it can actually
        // do, both measured this tick. The line's own targetSpeedKmh already encodes a backward
        // braking pass from bake time; what this adds is re-solving that pass from the car's real
        // position with a real budget, which is what makes it brake early instead of on arrival.
        float brakeDecelMs2 = _planner.Tick(
            coordinator,
            speedKmh,
            fallbackBrakeDecelMs2,
            brakeMeasurementMinSpeedKmh,
            Time.fixedDeltaTime);

        LastPlannedSpeedKmh = _planner.SolvePlannedSpeedKmh(
            racingLine,
            lookaheadSegmentIndex,
            transform.position,
            speedKmh,
            brakeDecelMs2,
            difficulty.speedMultiplier,
            difficulty.cornerConfidence,
            cautionStrength,
            minimumBrakingHorizonMetres,
            maxBrakingHorizonWaypoints,
            LastTargetSpeedKmh);

        LastBrakeDecelMs2 = _planner.LastBrakeDecelMs2;
        LastBrakingHorizonMetres = _planner.LastHorizonMetres;
        LastBrakingHorizonWaypoints = _planner.LastHorizonWaypointCount;

        UpdateTrackRecoveryState(waypoint);
        UpdateLaneOffset(waypoint, difficulty);
        if (IsOvertaking)
            LastTargetSpeedKmh *= 1f + overtakeSpeedBoost * GetCompetitiveness01(difficulty);

        if (IsRecoveringTrack)
            LastTargetSpeedKmh = Mathf.Min(LastTargetSpeedKmh, trackRecoverySpeedKmh);

        // Applied LAST, so the plan governs every intent above rather than being overridden by
        // it. The overtake boost in particular raises the target by up to 7%, and applying the
        // minimum before it would let a committed pass ask for a corner entry speed the car
        // cannot physically brake for. A driver may want to go faster than the brakes allow; the
        // brakes are what actually arrive.
        LastTargetSpeedKmh = Mathf.Min(LastTargetSpeedKmh, LastPlannedSpeedKmh);

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

            // The speed the follow law settled on, before the overtake branch touches it.
            // The overtake eases toward THIS and is floored by it. It used to ease toward
            // `targetSpeedKmh` — the speed of a road with nothing on it — which is why a
            // committed overtaker aimed 114 km/h into a car doing 36.
            float followCeilingKmh = targetSpeedKmh;

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
                followCeilingKmh = followTarget;
                cautionTarget = Mathf.Lerp(followTarget, 0f, distanceBlend * distanceBlend);
                LastFollowTargetKmh = cautionTarget;
            }
            else
            {
                // Static scenery has no speed to follow, so the distance ramp is all there is.
                cautionTarget = Mathf.Lerp(blockedTargetSpeedKmh, 0f, distanceBlend * difficulty.avoidanceCaution);

                // The ceiling here is the speed the car is allowed past an obstacle at all.
                // Leaving it at the racing-line speed — the default set above this branch —
                // would mean an overtake against scenery has nothing to ease toward and the
                // clamp collapses to the obstacle's own caution, which is the opposite of
                // what committing to a pass is for. A driver who has moved out to pass
                // something should be allowed to get past it.
                followCeilingKmh = blockedTargetSpeedKmh;
                LastFollowTargetKmh = cautionTarget;
            }

            // An overtake eases the clamp, but only toward the car ahead.
            //
            // This used to blend the caution speed back toward the UNBLOCKED target by
            // overtakeTrafficSlowdownScale, which meant the act of deciding to pass undone the
            // very reaction that noticed the car being passed. Worked through on a Hard car
            // with a 15 m gap on a straight: the leader braked to 36 km/h while the overtaker
            // — seeing the same car at the same distance — commanded 114 km/h, contact in about
            // a third of a second. The faster a driver was willing to overtake, the harder it
            // drove into the car it was overtaking, which is backwards.
            //
            // The `Mathf.Min(eased, targetSpeedKmh)` here was a no-op that hid it.
            // Lerp(cautionTarget, targetSpeedKmh, t) with t in [0,1] always returns a value
            // inside [cautionTarget, targetSpeedKmh], and targetSpeedKmh is above cautionTarget
            // whenever there is traffic to ease out of — so `eased` was never above
            // targetSpeedKmh and the Min handed it straight back. The branch raised the clamp.
            //
            // The ease now runs toward `followCeilingKmh` — the speed the following law itself
            // chose for this leader and this gap — and is floored by it. Both bounds are
            // load-bearing:
            //
            //   * the floor holds a committed overtaker at or below the safe following speed,
            //     which is what stops it driving into the car it is passing;
            //   * the blend toward the ceiling is what makes the overtake still DO something.
            //
            // Simply forbidding the branch from raising the clamp would satisfy a
            // "never exceeds the leader" test by making the decision to pass change the lane
            // and nothing else — the earlier bug wearing the other hat. Committing to a pass
            // has to buy back some speed or there is no reason to commit.
            //
            // The `followCeilingKmh > cautionTarget` guard is what makes the two bounds mean
            // different things. When the ceiling is ABOVE the caution there is room to ease
            // upward into, and the branch buys some of it. When it is not — the leader reads
            // zero, so the follow law put the ceiling at 0 as well — there is nothing to ease
            // toward and the branch leaves the caution exactly where the follow law put it.
            // Easing toward a ceiling below the caution would not be an ease at all; it would
            // slow the car further, which is the opposite of what committing to a pass means.
            if (IsOvertaking && followCeilingKmh > cautionTarget)
            {
                float eased = Mathf.Lerp(cautionTarget, followCeilingKmh, overtakeTrafficSlowdownScale);
                cautionTarget = Mathf.Min(eased, followCeilingKmh);
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

        // Feed-forward plus trim, replacing a pure proportional band on speed error.
        //
        // The old law was `throttle = error/16`, `brake = -error/(42 * margin)`. On a Hard car
        // the brake saturates at 31 km/h of error and the throttle at 16, so ANY error between
        // -31 and +16 km/h commanded neither pedal: a ~47 km/h-wide dead band. The controller
        // could not act inside it, so it drifted out of the band, saturated, overshot, and came
        // back — the oscillation, not the corner, was setting the brake rhythm.
        //
        // The trim is a narrow proportional term that closes the residual. The feed-forward is
        // what removes the dead band: it derives the pedal from the distance still available to
        // shed the excess, so the brake ramps in continuously as the corner approaches instead of
        // waiting for an error large enough to escape a band.
        float speedError = speedTarget - speedKmh;
        float speedMs = speedKmh / 3.6f;
        float brakeDecel = Mathf.Max(0.5f, LastBrakeDecelMs2);

        if (speedError > 0f)
        {
            // Below target: throttle. No feed-forward term is meaningful here — there is no
            // schedule of corners to be ahead of, only a shortfall to close.
            LastThrottleInput = Mathf.Clamp01(speedError / throttleFullErrorKmh);
            LastBrakeInput = 0f;
        }
        else
        {
            // Above target. How much brake is needed is set by how quickly the excess has to go,
            // and the only honest answer is the braking horizon: the excess must be shed before
            // the distance runs out, so the demand is spread over the horizon rather than
            // applied as a step.
            //
            // Two guards keep this honest. The horizon is floored so a slow car still has room
            // to work with, and the result is limited to 1 because beyond full brake there is no
            // more deceleration to be had — asking for 4x the brake force buys nothing and only
            // destabilises the tyre model.
            float excessMs = -speedError / 3.6f;
            float horizonSeconds = Mathf.Max(0.05f, LastBrakingHorizonMetres / Mathf.Max(1f, speedMs));
            float requiredDecel = excessMs / horizonSeconds;

            float feedForward = Mathf.Clamp01(requiredDecel / brakeDecel);

            // Trim on top, for the residual the feed-forward cannot see: a car that is off the
            // line, on a surface that grips worse than the budget assumed, or simply not yet
            // responding. Narrow, because the feed-forward is carrying the steady state and a
            // wide trim here would reintroduce exactly the band this replaced.
            float trim = Mathf.Clamp01(excessMs / Mathf.Max(1f, brakeTrimSeconds * brakeDecel));

            LastBrakeInput = Mathf.Clamp01(Mathf.Max(feedForward, trim) * difficulty.brakingMargin);
            LastThrottleInput = 0f;
        }

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

            // Scaling the trigger with speed is only half the fix. A trigger that
            // outgrows the sensor's 32 m reach can never be "not reached" again, so on its
            // own it would fire on anything it can see — which is a presence test, not a
            // hazard test. Two extra terms make it a hazard test at every speed.
            //
            // What decides a contact is closing speed, and the sensor already measures it.
            // LastClosingSpeedKmh was being recorded for exactly this and read by nothing.
            // A closing-rate gate alone is not enough either: a car matching its leader's
            // speed 6 m behind is in exactly the same position as one closing at 40 km/h into
            // it, and the brake that helps is the same brake.
            //
            // So the trigger is two independent conditions, and either one is an emergency:
            //
            //   * CONTACT — inside emergencyMinGap, whatever the closing rate. This is the
            //     proximity term, and it is what makes the brake still fire at 6 m with the
            //     speed differential near zero.
            //   * CLOSING — inside the speed-scaled trigger AND genuinely closing faster than
            //     emergencyClosingKmh. This is what keeps a 30 m queue at 250 km/h, where the
            //     gap lasts ~17 seconds, off the emergency brake entirely. The following law
            //     is the right tool for a queue.
            //
            // The closing gate is a TIME TO CONTACT, not a closing rate. A rate on its own
            // has no idea how much room there is: 6 km/h of closing sounds alarming until
            // you notice the gap is 30 m and therefore lasts eighteen seconds. What makes it
            // dangerous is closing fast AND having little room, and the two have to be
            // multiplied together. emergencyClosingKmh stays as the deadband that keeps
            // sensor noise and a stationary leader out of the division entirely.
            //
            // Scenery is NOT exempt. It used to be — `!needsRiskCheck || ...` handed a wall
            // the emergency brake at any range, on the reasoning that a wall does not have to
            // move for you to hit it. That was true and it was still wrong, because the
            // trigger had been capped at the sensor's reach and this law is not: with the
            // cap gone, "any range" became "any range the car can see", and an obstacle
            // 20 m up the road took a 0.85 brake application off the side-traffic and
            // traffic-caution laws that exist to handle exactly that case. A wall reports no
            // closing speed, so `closingFast` is false for it and the contact term is the
            // only thing that can fire — which is the correct reading anyway. The emergency
            // brake is for "this is now", not for "there is a wall somewhere ahead".
            bool atContactRange = perception.FrontDistance <= emergencyMinGap;
            bool closingFast = perception.ClosingSpeedKmh > emergencyClosingKmh;

            float closingMs = Mathf.Max(0.01f, perception.ClosingSpeedKmh / 3.6f);
            float timeToContact = perception.FrontDistance / closingMs;
            bool imminent = closingFast && timeToContact < emergencyTimeHeadway;

            bool risky = atContactRange || imminent;

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

        // Recorded here, at the very end, so the planner learns from the brake that was actually
        // commanded — including the emergency override above, which is the strongest signal there
        // is. Recorded earlier it would learn from a value the emergency then overwrote, and
        // would attribute to the tyres a deceleration the corner law never asked for.
        _planner.RecordBrakeCommand(LastBrakeInput);
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

        // The old law ended here:
        //
        //     float sensorReach = perception != null ? perception.forwardDistance : 32f;
        //     return Mathf.Min(distance, sensorReach);
        //
        // With the shipped values that is min(6 + 2.3 * v_ms, 32). The cap binds above
        // 26 / 2.3 = 11.3 m/s = 40.7 km/h, so for every speed the game actually races at,
        // emergencyDistance came back as exactly 32 — the sensor's own reach. FrontDistance
        // is a cast TRAVEL distance and is therefore always <= forwardDistance, so
        // `FrontDistance < emergencyDistance` was true on essentially every blocked tick.
        // The distance test was vacuous above 41 km/h.
        //
        // What it was left resting on was the closing-rate gate alone, and that gate is a
        // threshold of 4 km/h. A car holding 250 km/h with a rival doing 246 four seconds up
        // the road was 0.85 braking at full throttle-off, and every car in a train did it to
        // the car in front. That is the "no erratic braking" acceptance criterion failing, and
        // it explains S-A as well: the overtake raised the target, the car closed, closing
        // passed 4 km/h, the emergency fired, speed dropped, closing went negative, the brake
        // released — a full-throttle/full-brake transient for the whole of every pass.
        //
        // The trigger now grows with speed all the way up, so it stays a hazard test across
        // the range the game is played at rather than becoming a presence test at 41 km/h.
        // That alone is not enough — a trigger wider than the sensor is unreachable — so the
        // caller pairs it with a proximity term and a closing-rate term. See the branch above.
        return distance;
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
