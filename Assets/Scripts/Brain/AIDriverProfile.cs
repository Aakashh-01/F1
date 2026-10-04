using UnityEngine;

/// <summary>
/// Every tunable the AI driver uses, in one asset.
///
/// <para>
/// This exists because the values used to live in three places at once: as public fields on
/// <c>AIDriverController</c>, as overrides serialised into <c>AI_F1_Body.prefab</c>, and as
/// defaults in the code. Unity deserialisation prefers the prefab, so of 53 tunables, **30 were
/// shadowed by the prefab and every code-side change to them silently did nothing.** The proof
/// that this is not theoretical: <c>trackRecoverySpeedKmh</c> has a tooltip on this field saying
/// it "used to be 95 km/h" and is set to 45, while the prefab still serialised 95 — a documented
/// fix that had never been in effect.
///
/// </para>
///
/// <para>
/// The field defaults below are the values the AI was <i>actually running</i> — the prefab's,
/// not the code's — so creating this asset changes nothing about how the cars drive. The one
/// deliberate exception is <see cref="trackRecoverySpeedKmh"/>, which takes 45 rather than the
/// prefab's 95 because 95 is the exact value the field's own tooltip identifies as a bug.
/// </para>
///
/// <para>
/// This is a driver profile, not a car profile. Physical limits (mass, downforce, brake force,
/// grip) belong to <c>CarDefinition</c> and are shared by every driver of that car; the AI must
/// not be the thing that defines them.
/// </para>
/// </summary>
[CreateAssetMenu(menuName = "F1/AI Driver Profile", fileName = "AIDriver_")]
public class AIDriverProfile : ScriptableObject
{
    [Header("Path Following")]
    [Tooltip("Distance ahead the driver aims for at a standstill, in metres. Combined with " +
             "lookaheadPerKmh into the distance actually used at speed.")]
    [Range(2f, 60f)] public float baseLookaheadDistance = 8f;

    [Tooltip("Metres of extra lookahead per km/h. This is what keeps the aim point far enough " +
             "ahead at racing speed that the car is not always steering at a point it has already " +
             "passed.")]
    [Range(0f, 0.4f)] public float lookaheadPerKmh = 0.035f;

    [Tooltip("Segments ahead over which corner curvature is measured. Only affects the corner " +
             "confidence and pack-corner terms — NOT the target speed, which is a backward " +
             "braking pass baked into the racing line.")]
    [Range(1, 6)] public int curvatureLookaheadSteps = 3;

    [Tooltip("Steering angle, in degrees, treated as full lock. Pure pursuit divides the angle " +
             "to the aim point by this to get the steering command.")]
    [Range(15f, 90f)] public float steeringAngleForFullInput = 32f;

    [Tooltip("How fast the lateral lane offset moves, in metres per second.")]
    [Range(0.2f, 8f)] public float laneOffsetMoveSpeed = 4.4f;

    [Tooltip("How close the car must come to a waypoint to be considered to have reached it.")]
    [Range(1f, 25f)] public float waypointReachDistance = 8f;

    [Tooltip("Segments searched forward when re-acquiring the racing line after a disturbance.")]
    [Range(1, 10)] public int forwardProgressSearchSteps = 4;

    [Header("Longitudinal Planner")]
    [Tooltip("Fallback braking capability, in m/s^2, used when the car cannot be measured — " +
             "airborne, or a car with no drivetrain. The real budget is measured from the " +
             "drivetrain's own per-wheel force clamp every tick, so this only covers the frames " +
             "where that measurement is meaningless.")]
    [Range(2f, 60f)] public float fallbackBrakeDecelMs2 = 18f;

    [Tooltip("Shortest braking horizon, in metres. The horizon is normally a braking distance, " +
             "which is short in a slow corner; this floor stops a crawling car from seeing only " +
             "the waypoint it is standing on.")]
    [Range(10f, 200f)] public float minimumBrakingHorizonMetres = 45f;

    [Tooltip("Cap on waypoints examined per tick. Bounds the per-tick walk over the line; a " +
             "long horizon at low speed can otherwise cover a third of the track.")]
    [Range(4, 64)] public int maxBrakingHorizonWaypoints = 24;

    [Tooltip("Speed below which an observed deceleration is discarded rather than learned " +
             "from. Near a standstill the reading is dominated by drivetrain spool-up.")]
    [Range(5f, 120f)] public float brakeMeasurementMinSpeedKmh = 35f;

    [Tooltip("Time constant, in seconds, of the brake trim term. This is the width of the band " +
             "over which a speed excess is corrected proportionally. It is deliberately narrow: " +
             "the feed-forward term already carries the steady state, and a wide trim here would " +
             "reproduce the dead band this replaced — a wide one on the old controller meant " +
             "any error inside it commanded neither pedal, so the car coasted out of the band, " +
             "saturated, and oscillated.")]
    [Range(0.05f, 1.5f)] public float brakeTrimSeconds = 0.25f;

    [Header("Speed Control")]
    [Tooltip("Speed error, in km/h, at which throttle reaches full.")]
    [Range(1f, 80f)] public float throttleFullErrorKmh = 16f;

    [Tooltip("Speed error, in km/h, at which brake reaches full, before the difficulty braking " +
             "margin scales it. A large value means the car coasts for a wide band of error " +
             "before it commits to either pedal.")]
    [Range(1f, 100f)] public float brakeFullErrorKmh = 42f;

    [Tooltip("Speed an immovable obstacle holds the car to, at the far end of the sensor. Only " +
             "reachable for scenery, which is currently never seen — see the perception mask.")]
    [Range(10f, 160f)] public float blockedTargetSpeedKmh = 45f;

    [Tooltip("Fraction of throttle removed at full steering lock, so the car does not drive the " +
             "throttle and the steering at the same time in a corner.")]
    [Range(0f, 1f)] public float steeringThrottleReduction = 0.12f;

    [Header("Competitive Pace")]
    [Tooltip("How much a corner slows the car at low competitiveness.")]
    [Range(0f, 0.5f)] public float baseCornerCautionStrength = 0.26f;

    [Tooltip("How much a corner slows the car at high competitiveness.")]
    [Range(0f, 0.5f)] public float competitiveCornerCautionStrength = 0.09f;

    [Tooltip("Fraction added to the target speed while committing to a pass.")]
    [Range(0f, 0.25f)] public float overtakeSpeedBoost = 0.07f;

    [Tooltip("Fraction of the way the traffic speed clamp is eased toward the racing-line target " +
             "once a pass is committed. Never a licence to exceed the leader's speed plus the " +
             "closing allowance — see the leader ceiling in AIDriverController.")]
    [Range(0f, 1f)] public float overtakeTrafficSlowdownScale = 0.4f;

    [Tooltip("Multiplier on the steering-related throttle cut while overtaking, so the car does " +
             "not lift mid-manoeuvre.")]
    [Range(0f, 1f)] public float overtakeSteeringThrottleScale = 0.48f;

    [Header("Overtaking")]
    [Tooltip("Which side to try first when a pass is available. A car only picks the other side " +
             "if the preferred one is blocked.")]
    public bool preferRightOvertake = true;

    [Tooltip("Minimum overtake willingness required before the car will leave its line to pass. " +
             "Lower means more willing.")]
    [Range(0f, 1f)] public float overtakeTrigger = 0.18f;

    [Tooltip("Seconds a chosen pass direction is held once the car has committed to it, so the " +
             "side does not flip while the car is halfway through the manoeuvre.")]
    [Range(0.1f, 4f)] public float overtakeCommitSeconds = 1.1f;

    [Header("Race Craft")]
    [Tooltip("Standing lane bias for this car, -1 hard left to 1 hard right, as a fraction of " +
             "the usable lane. Default 0 means the car prefers the middle of the road.")]
    [Range(-1f, 1f)] public float preferredLaneOffset01;

    [Tooltip("Fraction of the usable lane this car is willing to use on clear track. Low values " +
             "make a car hold a narrow line; high values let it move to defend or overtake.")]
    [Range(0f, 1f)] public float freeTrackLaneUse = 0.74f;

    [Tooltip("Weight given to the racing line's own per-waypoint lane intent over this car's " +
             "standing preference. Currently inert: every baked waypoint has a lane intent of 0.")]
    [Range(0f, 1f)] public float waypointLaneInfluence = 0.78f;

    [Tooltip("Metres kept clear between the car's lateral offset and the edge of the lane.")]
    [Range(0f, 4f)] public float laneEdgeSafetyMargin = 1.4f;

    [Tooltip("Amplitude of the slow lane drift that keeps a car from driving a robotic line.")]
    [Range(0f, 1f)] public float laneVariationStrength = 0.05f;

    [Tooltip("Frequency of that lane drift, in cycles per waypoint.")]
    [Range(0.01f, 0.5f)] public float laneVariationFrequency = 0.08f;

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
             "than it can shed the difference, and it is also the ceiling an overtaker is held " +
             "to while a car is still ahead of it.")]
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
    [Tooltip("Metres of lateral movement that still counts as 'holding lane' rather than " +
             "'leaving lane'. The side guard refuses a lane change while the difference is " +
             "under this, so a car parked exactly on its target offset still counts as moving.")]
    [Range(0f, 2f)] public float sideTrafficLaneHoldBuffer = 0.25f;

    [Tooltip("Fraction of throttle retained at full steering load while another car is beside " +
             "this one in a corner.")]
    [Range(0f, 1f)] public float sideTrafficTurnThrottleScale = 0.62f;

    [Tooltip("Brake applied at full steering load while another car is beside this one in a " +
             "corner.")]
    [Range(0f, 0.5f)] public float sideTrafficTurnBrake = 0.18f;

    [Tooltip("Steering load above which a car beside this one counts as a turn risk.")]
    [Range(0f, 1f)] public float sideTrafficSteeringThreshold = 0.24f;

    [Tooltip("Curvature above which a car treats the corner as a pack corner and slows for the " +
             "cars around it. NOTE: curvature is measured over a fixed segment lookahead, so " +
             "this threshold is only reached in genuinely tight corners.")]
    [Range(0f, 1f)] public float packCornerCurvatureThreshold = 0.38f;

    [Tooltip("Factor applied to the target speed in a pack corner.")]
    [Range(0.2f, 1f)] public float packCornerSpeedScale = 0.74f;

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
             "about 2 m wide, so anything under this is inside it. NOTE: the sensor reports " +
             "cast travel, which already includes the 1.15 m half-extent of the side box, so " +
             "the true lateral gap is this plus that.")]
    [Range(0.5f, 4f)] public float sideOverlapEscapeMetres = 1.8f;

    [Tooltip("How far to move sideways in one tick when escaping a contact, in metres.")]
    [Range(0.1f, 4f)] public float sideOverlapEscapeShift = 1.6f;

    [Tooltip("Speed floor a car in contact is allowed to keep. Enough to slide clear of a " +
             "car on top of it; the contact is mutual, so a follow-the-leader clamp would " +
             "otherwise hold both at a standstill with neither able to restart.")]
    [Range(0f, 80f)] public float overlapEscapeMinSpeedKmh = 40f;

    [Header("Track Recovery")]
    [Tooltip("Metres beyond the lane edge at which a car is judged to have left the road.")]
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

    /// <summary>
    /// A usable profile for a car that has none assigned.
    ///
    /// <para>
    /// Mirrors <c>AIDifficultyProfile.CreateRuntimeProfile</c>, and exists for the same reason:
    /// a missing asset must degrade to the shipped numbers rather than to a
    /// <c>NullReferenceException</c> thrown from inside a physics tick. The field defaults
    /// above are already the values the AI was running, so an unassigned car drives exactly
    /// as it did before this profile existed — the fallback is a safety net, not a second
    /// source of truth.
    /// </para>
    ///
    /// <para>
    /// The caller is responsible for caching the result. Every tunable on
    /// <c>AIDriverController</c> reads through this profile several times per physics step, so
    /// calling this per read would allocate a <c>ScriptableObject</c> per property per frame
    /// per car.
    /// </para>
    /// </summary>
    public static AIDriverProfile CreateRuntimeProfile()
    {
        return CreateInstance<AIDriverProfile>();
    }
}