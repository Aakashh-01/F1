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

    [Header("Mobile Performance")]
    [Range(1, 12)] public int fixedFrameStride = 3;
    [Range(0, 11)] public int fixedFrameOffset;
    [Range(0.03f, 0.25f)] public float minimumUpdateInterval = 0.07f;
    [Range(3, 24)] public int hitBufferSize = 8;

    [HideInInspector] public bool FrontBlocked;
    [HideInInspector] public bool LeftBlocked;
    [HideInInspector] public bool RightBlocked;
    [HideInInspector] public float FrontDistance;
    [HideInInspector] public Transform ClosestObstacle;
    [HideInInspector] public float LastSensorUpdateTime;
    [HideInInspector] public int SensorUpdateSerial;

    private RaycastHit[] _forwardHits;
    private RaycastHit[] _leftHits;
    private RaycastHit[] _rightHits;
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
        LeftBlocked = CastSensor(origin, -transform.right, sideHalfExtents, sideDistance, rotation, _leftHits, out _, out _);
        RightBlocked = CastSensor(origin, transform.right, sideHalfExtents, sideDistance, rotation, _rightHits, out _, out _);
        ClosestObstacle = frontObstacle;

        if (!FrontBlocked)
            FrontDistance = forwardDistance;

        return true;
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
