using UnityEngine;

public class AIRacingLine : MonoBehaviour
{
    public bool loop = true;
    public AIRacingWaypoint[] waypoints = new AIRacingWaypoint[0];

    public int Count => waypoints != null ? waypoints.Length : 0;

    private void Awake()
    {
        RefreshWaypoints();
    }

    private void OnValidate()
    {
        RefreshWaypoints();
    }

    public void RefreshWaypoints()
    {
        waypoints = GetComponentsInChildren<AIRacingWaypoint>();
    }

    public AIRacingWaypoint GetWaypoint(int index)
    {
        if (Count == 0)
            return null;

        return waypoints[WrapIndex(index)];
    }

    public Vector3 GetPosition(int index)
    {
        AIRacingWaypoint waypoint = GetWaypoint(index);
        return waypoint != null ? waypoint.Position : transform.position;
    }

    public int FindNearestIndex(Vector3 position)
    {
        if (Count == 0)
            return -1;

        int bestIndex = 0;
        float bestDistance = float.MaxValue;
        for (int i = 0; i < Count; i++)
        {
            if (waypoints[i] == null)
                continue;

            float distance = (waypoints[i].Position - position).sqrMagnitude;
            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex = i;
            }
        }

        return bestIndex;
    }

    public int GetLookaheadIndex(int startIndex, float lookaheadDistance)
    {
        if (Count == 0)
            return -1;

        int index = WrapIndex(startIndex);
        float remaining = Mathf.Max(0f, lookaheadDistance);
        int maxSteps = loop ? Count : Count - index - 1;

        for (int i = 0; i < maxSteps; i++)
        {
            int next = WrapIndex(index + 1);
            float segmentLength = Vector3.Distance(GetPosition(index), GetPosition(next));
            if (remaining <= segmentLength)
                return next;

            remaining -= segmentLength;
            index = next;
        }

        return index;
    }

    public Vector3 GetSegmentForward(int index)
    {
        if (Count < 2)
            return transform.forward;

        Vector3 from = GetPosition(index);
        Vector3 to = GetPosition(index + 1);
        Vector3 forward = Vector3.ProjectOnPlane(to - from, Vector3.up);
        return forward.sqrMagnitude > 0.001f ? forward.normalized : transform.forward;
    }

    public Vector3 GetSegmentRight(int index)
    {
        return Vector3.Cross(Vector3.up, GetSegmentForward(index)).normalized;
    }

    public float CalculateCurvature01(int index, int lookaheadSteps)
    {
        if (Count < 3)
            return 0f;

        int safeSteps = Mathf.Clamp(lookaheadSteps, 1, Count - 1);
        Vector3 currentForward = GetSegmentForward(index);
        Vector3 futureForward = GetSegmentForward(index + safeSteps);
        float angle = Vector3.Angle(currentForward, futureForward);
        return Mathf.InverseLerp(8f, 90f, angle);
    }

    public int WrapIndex(int index)
    {
        if (Count == 0)
            return 0;

        if (loop)
            return (index % Count + Count) % Count;

        return Mathf.Clamp(index, 0, Count - 1);
    }

    private void OnDrawGizmos()
    {
        RefreshWaypoints();
        if (Count < 2)
            return;

        Gizmos.color = new Color(0f, 0.9f, 1f, 0.8f);
        for (int i = 0; i < Count; i++)
        {
            int next = i + 1;
            if (!loop && next >= Count)
                break;

            Gizmos.DrawLine(GetPosition(i), GetPosition(next));
            Vector3 direction = GetSegmentForward(i);
            Gizmos.DrawRay(GetPosition(i), direction * 4f);
        }
    }
}
