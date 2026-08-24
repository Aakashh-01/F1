using UnityEngine;

/// <summary>
/// Single authority for wheel slot identity and ordering (FL=0, FR=1, RL=2, RR=3).
/// All systems receive resolved RaycastWheel[] arrays from VehiclePhysicsCoordinator;
/// this type owns the legacy name-to-slot mapping and validation so the lookup
/// logic is not duplicated per system.
/// </summary>
public static class WheelSlots
{
    public const int FL = 0;
    public const int FR = 1;
    public const int RL = 2;
    public const int RR = 3;

    public static readonly string[] SlotNames = { "FL", "FR", "RL", "RR" };

    /// <summary>
    /// Fills empty slots by matching child object names ("FL"/"FR"/"RL"/"RR").
    /// Explicitly assigned slots are never overwritten.
    /// </summary>
    public static void ResolveByName(RaycastWheel[] foundWheels, RaycastWheel[] slots)
    {
        if (foundWheels == null || slots == null)
            return;

        int count = Mathf.Min(slots.Length, SlotNames.Length);
        for (int slot = 0; slot < count; slot++)
        {
            if (slots[slot] != null)
                continue;

            for (int i = 0; i < foundWheels.Length; i++)
            {
                RaycastWheel wheel = foundWheels[i];
                if (wheel != null && wheel.name == SlotNames[slot])
                {
                    slots[slot] = wheel;
                    break;
                }
            }
        }
    }

    /// <summary>Logs an error for every unassigned slot. Returns true when all slots are filled.</summary>
    public static bool Validate(RaycastWheel[] slots, string ownerName)
    {
        if (slots == null || slots.Length < SlotNames.Length)
        {
            Debug.LogError($"[WheelSlots] '{ownerName}' needs exactly {SlotNames.Length} wheel slots (FL, FR, RL, RR).");
            return false;
        }

        bool valid = true;
        for (int slot = 0; slot < SlotNames.Length; slot++)
        {
            if (slots[slot] == null)
            {
                Debug.LogError($"[WheelSlots] '{ownerName}' wheel slot {SlotNames[slot]} is not assigned. Physics will be incorrect.");
                valid = false;
            }
        }

        return valid;
    }
}
