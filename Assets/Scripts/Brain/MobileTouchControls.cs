using UnityEngine;
using UnityEngine.EventSystems;

public class MobileTouchControls : MonoBehaviour
{
    public static float Steering { get; private set; }
    public static float Throttle { get; private set; }
    public static float Brake { get; private set; }

    public static void SetSteering(float value)
    {
        Steering = Mathf.Clamp(value, -1f, 1f);
    }

    public static void SetThrottle(float value)
    {
        Throttle = Mathf.Clamp01(value);
    }

    public static void SetBrake(float value)
    {
        Brake = Mathf.Clamp01(value);
    }

    public static void ResetInputs()
    {
        SetSteering(0f);
        SetThrottle(0f);
        SetBrake(0f);
    }

    private void OnDisable()
    {
        ResetInputs();
    }

    public void SteerLeftDown()
    {
        SetSteering(-1f);
    }

    public void SteerLeftDown(BaseEventData eventData)
    {
        SteerLeftDown();
    }

    public void SteerRightDown()
    {
        SetSteering(1f);
    }

    public void SteerRightDown(BaseEventData eventData)
    {
        SteerRightDown();
    }

    public void SteeringUp()
    {
        SetSteering(0f);
    }

    public void SteeringUp(BaseEventData eventData)
    {
        SteeringUp();
    }

    public void ThrottleDown()
    {
        SetThrottle(1f);
    }

    public void ThrottleDown(BaseEventData eventData)
    {
        ThrottleDown();
    }

    public void ThrottleUp()
    {
        SetThrottle(0f);
    }

    public void ThrottleUp(BaseEventData eventData)
    {
        ThrottleUp();
    }

    public void BrakeDown()
    {
        SetBrake(1f);
    }

    public void BrakeDown(BaseEventData eventData)
    {
        BrakeDown();
    }

    public void BrakeUp()
    {
        SetBrake(0f);
    }

    public void BrakeUp(BaseEventData eventData)
    {
        BrakeUp();
    }
}
