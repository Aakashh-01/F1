using UnityEngine;

public class BasicMotor : MonoBehaviour
{
    public Rigidbody rb;
    public float motorForce;

    [Tooltip("How fast the engine reaches full power. Lower = slower buildup.")]
    public float throttleSpoolSpeed = 1.5f;

    private float _currentThrottle = 0f;

    void FixedUpdate()
    {
        // Using GetAxisRaw to get the exact key press without Unity's hidden delay
        float targetInput = Input.GetAxisRaw("Vertical");

        // Gradually ramp the throttle up or down over time
        _currentThrottle = Mathf.MoveTowards(_currentThrottle, targetInput, throttleSpoolSpeed * Time.fixedDeltaTime);

        if (Mathf.Abs(_currentThrottle) > 0.01f)
        {
            // Apply force at the Center of Mass
            Vector3 forcePos = transform.TransformPoint(rb.centerOfMass);
            rb.AddForceAtPosition(transform.forward * _currentThrottle * motorForce, forcePos, ForceMode.Force);
        }
    }
}