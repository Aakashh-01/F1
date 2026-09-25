using UnityEngine;

namespace F1
{
    /// <summary>
    /// Standalone AeroProfile ScriptableObject for wing variants.
    /// Used by CarDefinition for High/Low downforce wing options.
    /// At runtime, values are copied into VehiclePhysicsProfile.aero (which uses the global AeroProfile class).
    /// </summary>
    [CreateAssetMenu(menuName = "F1/Wing Aero Profile", fileName = "WingAeroProfile_")]
    public class WingAeroProfile : ScriptableObject
    {
        [Header("Downforce")]
        [Range(0.5f, 8f)] public float downforceCoeff = 5f;
        [Range(0f, 1f)] public float frontBias = 0.38f;
        [Range(20f, 100f)] public float fullDownforceSpeed = 70f;

        [Header("Drag")]
        [Range(0f, 5f)] public float lateralDragCoeff = 1.8f;
        [Range(10f, 60f)] public float lateralDragOnsetSpeed = 20f;
        [Range(1000f, 40000f)] public float lateralDragCap = 18000f;

        /// <summary>
        /// Copies this WingAeroProfile's values into an AeroProfile (global class used by VehiclePhysicsProfile).
        /// </summary>
        public void ApplyTo(AeroProfile target)
        {
            if (target == null) return;
            target.downforceCoeff = downforceCoeff;
            target.frontBias = frontBias;
            target.fullDownforceSpeed = fullDownforceSpeed;
            target.lateralDragCoeff = lateralDragCoeff;
            target.lateralDragOnsetSpeed = lateralDragOnsetSpeed;
            target.lateralDragCap = lateralDragCap;
        }

        /// <summary>
        /// Creates a new AeroProfile with this profile's values.
        /// </summary>
        public AeroProfile CreateNested()
        {
            var nested = new AeroProfile();
            ApplyTo(nested);
            return nested;
        }
    }
}