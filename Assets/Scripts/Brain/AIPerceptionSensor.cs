using UnityEngine;

public class AIPerceptionSensor : MonoBehaviour
{
    [Header("Cast Settings")]
    public LayerMask obstacleLayers = ~0;
    [Range(2f, 80f)] public float forwardDistance = 26f;
    [Range(1f, 20f)] public float sideDistance = 7f;
    public Vector3 forwardHalfExtents = new Vector3(1.3f, 0.8f, 0.8f);
    public Vector3 sideHalfExtents = new Vector3(1.0f, 0.8f, 1.6f);
    [Range(0.1f, 3f)] public float sensorHeight = 0.8f;

    [Header("Rear Cast")]
    [Tooltip("How far behind the car the reverse-guard looks before allowing a reversing manoeuvre.")]
    [Range(2f, 20f)] public float rearDistance = 9f;
    public Vector3 rearHalfExtents = new Vector3(1.3f, 0.8f, 0.8f);

    [Header("Mobile Performance")]
    [Range(1, 12)] public int fixedFrameStride = 3;
    [Range(0, 11)] public int fixedFrameOffset;
    [Range(0.03f, 0.25f)] public float minimumUpdateInterval = 0.07f;
    [Range(3, 24)] public int hitBufferSize = 8;

    [HideInInspector] public bool FrontBlocked;
    [HideInInspector] public bool LeftBlocked;
    [HideInInspector] public bool RightBlocked;
    [HideInInspector] public bool RearBlocked;
    [HideInInspector] public float FrontDistance;
    [HideInInspector] public float RearDistance;

    /// <summary>
    /// Distance to the nearest thing alongside, on each side.
    ///
    /// These were being measured and thrown away (`out _` on the side casts), which left the
    /// driver knowing only *that* a car was abeam and not *how close* it was. That is not a
    /// cosmetic loss: the side readings are the only evidence a driver has about a car that
    /// has drifted out of the forward corridor, and without a distance there is no way to
    /// tell "a rival is one metre away and we are inside it" from "a rival is eight metres
    /// away and we are fine". The first has to be escaped; the second must be ignored.
    /// </summary>
    [HideInInspector] public float LeftDistance;
    [HideInInspector] public float RightDistance;

    [HideInInspector] public Transform ClosestObstacle;
    [HideInInspector] public float LastSensorUpdateTime;
    [HideInInspector] public int SensorUpdateSerial;

    /// <summary>
    /// Rigidbody of the car (or other body) currently ahead, resolved on cast ticks and
    /// kept until the next successful cast. Cached rather than re-resolved per tick so the
    /// following law does not stutter when the cast is throttled by the mobile-performance
    /// stride.
    /// </summary>
    [HideInInspector] public Rigidbody LeaderBody;

    /// <summary>
    /// True when the thing ahead is a physical body rather than static scenery. Static
    /// geometry has no speed to follow, so the following law must not be applied to it.
    /// </summary>
    [HideInInspector] public bool LeaderIsMovable => LeaderBody != null;

    /// <summary>Speed of the body ahead along this car's forward axis, km/h.</summary>
    [HideInInspector] public float LeaderSpeedKmh;

    /// <summary>How fast this car is closing on the body ahead, km/h. Positive means closing.</summary>
    [HideInInspector] public float ClosingSpeedKmh;

    private RaycastHit[] _forwardHits;
    private RaycastHit[] _leftHits;
    private RaycastHit[] _rightHits;
    private RaycastHit[] _rearHits;
    private Collider[] _selfColliders;
    private int _framesSinceUpdate;
    private float _nextUpdateTime;

    private void Awake()
    {
        AllocateBuffers();
        CacheSelfColliders();
        FrontDistance = forwardDistance;
    }

    private void OnValidate()
    {
        hitBufferSize = Mathf.Max(3, hitBufferSize);
        fixedFrameOffset = Mathf.Clamp(fixedFrameOffset, 0, Mathf.Max(0, fixedFrameStride - 1));
    }

    public bool Tick(bool forceUpdate = false)
    {
        EnsureReady();

        _framesSinceUpdate++;
        int stride = Mathf.Max(1, fixedFrameStride);
        int offset = Mathf.Clamp(fixedFrameOffset, 0, stride - 1);
        bool frameReady = _framesSinceUpdate >= stride - offset;
        bool timeReady = Time.time >= _nextUpdateTime;

        if (!forceUpdate && (!frameReady || !timeReady))
            return false;

        _framesSinceUpdate = 0;
        _nextUpdateTime = Time.time + minimumUpdateInterval;
        LastSensorUpdateTime = Time.time;
        SensorUpdateSerial++;

        Vector3 origin = transform.position + Vector3.up * sensorHeight;
        Quaternion rotation = transform.rotation;
        FrontBlocked = CastSensor(origin, transform.forward, forwardHalfExtents, forwardDistance, rotation, _forwardHits, out FrontDistance, out Transform frontObstacle);
        LeftBlocked = CastSensor(origin, -transform.right, sideHalfExtents, sideDistance, rotation, _leftHits, out LeftDistance, out _);
        RightBlocked = CastSensor(origin, transform.right, sideHalfExtents, sideDistance, rotation, _rightHits, out RightDistance, out _);
        RearBlocked = CastSensor(origin, -transform.forward, rearHalfExtents, rearDistance, rotation, _rearHits, out RearDistance, out _);
        ClosestObstacle = frontObstacle;

        if (!FrontBlocked)
            FrontDistance = forwardDistance;

        // Same for the sides: an unblocked cast reports the full reach, so a consumer can
        // treat "blocked" and "distance" as one consistent reading rather than having to
        // remember that a zero distance only means something when the flag is also set.
        if (!LeftBlocked)
            LeftDistance = sideDistance;
        if (!RightBlocked)
            RightDistance = sideDistance;

        // Resolve the leader's body on cast ticks only. The following law samples the
        // resulting speed every physics tick via SampleLeaderState, so a throttled cast
        // costs following nothing.
        LeaderBody = frontObstacle != null
            ? frontObstacle.GetComponentInParent<Rigidbody>()
            : null;

        SampleLeaderState();
        return true;
    }

    /// <summary>
    /// Refreshes the leader's speed and this car's closing rate without casting.
    ///
    /// This is called every physics tick rather than only on cast ticks so the following
    /// law always sees a current leader speed. Without it the follower would steer its
    /// gap control off a leader reading that is up to a full sensor stride stale — which at
    /// racing speed is metres of error in exactly the direction that closes the gap.
    /// </summary>
    public void SampleLeaderState()
    {
        if (LeaderBody == null)
        {
            LeaderSpeedKmh = 0f;
            ClosingSpeedKmh = 0f;
            return;
        }

#if UNITY_6000_0_OR_NEWER
        Vector3 leaderVelocity = LeaderBody.linearVelocity;
        Vector3 ownVelocity = GetOwnVelocity();
#else
        Vector3 leaderVelocity = LeaderBody.velocity;
        Vector3 ownVelocity = GetOwnVelocity();
#endif
        float leaderAlong = Vector3.Dot(leaderVelocity, transform.forward) * 3.6f;
        float ownAlong = Vector3.Dot(ownVelocity, transform.forward) * 3.6f;
        LeaderSpeedKmh = leaderAlong;
        ClosingSpeedKmh = ownAlong - leaderAlong;
    }

    private Vector3 GetOwnVelocity()
    {
        Rigidbody own = GetComponent<Rigidbody>();
        if (own == null)
            return Vector3.zero;
#if UNITY_6000_0_OR_NEWER
        return own.linearVelocity;
#else
        return own.velocity;
#endif
    }

    private bool CastSensor(
        Vector3 origin,
        Vector3 direction,
        Vector3 halfExtents,
        float distance,
        Quaternion rotation,
        RaycastHit[] hits,
        out float closestDistance,
        out Transform closestObstacle)
    {
        closestDistance = distance;
        closestObstacle = null;

        int hitCount = Physics.BoxCastNonAlloc(
            origin,
            halfExtents,
            direction,
            hits,
            rotation,
            distance,
            obstacleLayers,
            QueryTriggerInteraction.Ignore);

        bool blocked = false;
        for (int i = 0; i < hitCount; i++)
        {
            Collider hitCollider = hits[i].collider;
            if (hitCollider == null || IsSelfCollider(hitCollider))
                continue;

            float distanceToHit = Mathf.Max(0f, hits[i].distance);
            if (distanceToHit < closestDistance)
            {
                closestDistance = distanceToHit;
                closestObstacle = hitCollider.transform;
            }

            blocked = true;
        }

        return blocked;
    }

    private bool IsSelfCollider(Collider candidate)
    {
        if (candidate.transform == transform || candidate.transform.IsChildOf(transform))
            return true;

        for (int i = 0; i < _selfColliders.Length; i++)
        {
            if (_selfColliders[i] == candidate)
                return true;
        }

        return false;
    }

    private void EnsureReady()
    {
        if (_forwardHits == null || _forwardHits.Length != hitBufferSize)
            AllocateBuffers();

        if (_selfColliders == null)
            CacheSelfColliders();
    }

    private void AllocateBuffers()
    {
        _forwardHits = new RaycastHit[hitBufferSize];
        _leftHits = new RaycastHit[hitBufferSize];
        _rightHits = new RaycastHit[hitBufferSize];
        _rearHits = new RaycastHit[hitBufferSize];
    }

    private void CacheSelfColliders()
    {
        _selfColliders = GetComponentsInChildren<Collider>();
    }

    private void OnDrawGizmosSelected()
    {
        Vector3 origin = transform.position + Vector3.up * sensorHeight;
        DrawSensor(origin, transform.forward, forwardHalfExtents, forwardDistance, FrontBlocked ? Color.red : Color.green);
        DrawSensor(origin, -transform.right, sideHalfExtents, sideDistance, LeftBlocked ? Color.red : Color.cyan);
        DrawSensor(origin, transform.right, sideHalfExtents, sideDistance, RightBlocked ? Color.red : Color.cyan);
        DrawSensor(origin, -transform.forward, rearHalfExtents, rearDistance, RearBlocked ? Color.red : Color.magenta);
    }

    private void DrawSensor(Vector3 origin, Vector3 direction, Vector3 halfExtents, float distance, Color color)
    {
        Gizmos.color = new Color(color.r, color.g, color.b, 0.35f);
        Vector3 center = origin + direction.normalized * (distance * 0.5f);
        Vector3 size = halfExtents * 2f;
        if (Mathf.Abs(Vector3.Dot(direction.normalized, transform.forward)) > 0.5f)
            size.z += distance;
        else
            size.x += distance;

        Gizmos.matrix = Matrix4x4.TRS(center, transform.rotation, Vector3.one);
        Gizmos.DrawWireCube(Vector3.zero, size);
        Gizmos.matrix = Matrix4x4.identity;
    }
}
