using UnityEngine;

public class AIRacingWaypoint : MonoBehaviour
{
    [Range(20f, 360f)] public float targetSpeedKmh = 140f;
    [Range(1f, 25f)] public float laneWidth = 8f;
    [Range(0f, 1f)] public float brakingCaution = 0.35f;
    [Range(0f, 20f)] public float overtakeWidth = 5f;
    [Range(-1f, 1f)] public float preferredLaneOffset01;

    public Vector3 Position => transform.position;

    private void OnDrawGizmos()
    {
        float speedT = Mathf.InverseLerp(60f, 260f, targetSpeedKmh);
        Gizmos.color = Color.Lerp(Color.red, Color.green, speedT);
        Gizmos.DrawWireSphere(transform.position, 1.2f);

        Vector3 right = transform.right;
        Gizmos.color = new Color(0.2f, 0.8f, 1f, 0.55f);
        Gizmos.DrawLine(transform.position - right * laneWidth, transform.position + right * laneWidth);

        if (Mathf.Abs(preferredLaneOffset01) > 0.01f)
        {
            Gizmos.color = new Color(1f, 0.85f, 0.1f, 0.85f);
            Gizmos.DrawWireSphere(transform.position + right * laneWidth * preferredLaneOffset01, 0.75f);
        }
    }
}
