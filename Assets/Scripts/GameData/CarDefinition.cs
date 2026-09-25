using UnityEngine;
using WingAeroProfile = F1.WingAeroProfile;

namespace F1.GameData
{
    /// <summary>
    /// Defines a car available in the game. Stored as ScriptableObject for designer-friendly editing.
    /// Generation (Gen1/Gen2/Gen3) determines base performance tier and unlock costs.
    /// </summary>
    [CreateAssetMenu(menuName = "F1/Game Data/Car Definition", fileName = "CarDefinition_")]
    public class CarDefinition : ScriptableObject
    {
        [Header("Identity")]
        [Tooltip("Unique stable ID — used for save data keys. Never change after release.")]
        [SerializeField] private string _carId;
        public string CarId => _carId;

        [Tooltip("Display name shown in UI")]
        [SerializeField] private string _displayName;
        public string DisplayName => _displayName;

        [Tooltip("Generation tier: 1 = starter, 2 = mid, 3 = top")]
        [SerializeField] private int _generation = 1;
        public int Generation => _generation;

        [Header("Visuals")]
        [SerializeField] private Sprite _icon;
        public Sprite Icon => _icon;

        [SerializeField] private GameObject _carPrefab;
        public GameObject CarPrefab => _carPrefab;

        [Header("Physics — Base Profile")]
        [Tooltip("Base physics profile for this car. Wing setup will swap the AeroProfile at runtime.")]
        [SerializeField] private VehiclePhysicsProfile _basePhysicsProfile;
        public VehiclePhysicsProfile BasePhysicsProfile => _basePhysicsProfile;

        [Header("Wing Variants (WingAeroProfile only)")]
        [Tooltip("High downforce aero — used when player selects High Downforce wing")]
        [SerializeField] private WingAeroProfile _highDownforceAero;
        public WingAeroProfile HighDownforceAero => _highDownforceAero;

        [Tooltip("Low downforce aero — used when player selects Low Downforce wing")]
        [SerializeField] private WingAeroProfile _lowDownforceAero;
        public WingAeroProfile LowDownforceAero => _lowDownforceAero;

        [Header("Progression")]
        [Tooltip("Rental variant of the car. Rental cars are never granted on ownership; " +
                 "they must be rented for 24h, and laps set in them do not count toward rankings.")]
        [SerializeField] private bool _isRental;
        public bool IsRental => _isRental;

        [Tooltip("Points required to unlock permanently. 0 = free starter car. " +
                 "Ignored for rental variants, which are unlocked by paying RentalCost24h instead.")]
        [SerializeField] private int _unlockCostPoints = 0;
        public int UnlockCostPoints => _unlockCostPoints;

        [Tooltip("Direct purchase price (USD) — for IAP integration later")]
        [SerializeField] private float _directPurchasePrice = 0f;
        public float DirectPurchasePrice => _directPurchasePrice;

        [Tooltip("24-hour rental cost (USD)")]
        [SerializeField] private float _rentalCost24h = 0f;
        public float RentalCost24h => _rentalCost24h;

        [Header("Description")]
        [TextArea(2, 4)]
        [SerializeField] private string _description;
        public string Description => _description;

#if UNITY_EDITOR
        private void OnValidate()
        {
            if (string.IsNullOrEmpty(_carId))
                _carId = name.Replace("CarDefinition_", "").Replace(" ", "_").ToLowerInvariant();

            if (_generation < 1) _generation = 1;
            if (_generation > 3) _generation = 3;
        }
#endif
    }

    /// <summary>
    /// Runtime-only wrapper for wing selection. Not serialized — created at runtime from CarDefinition.
    /// </summary>
    public enum WingType
    {
        HighDownforce = 0,
        LowDownforce = 1
    }

    public static class CarDefinitionExtensions
    {
        /// <summary>
        /// Gets the WingAeroProfile for the selected wing type.
        /// </summary>
        public static WingAeroProfile GetAeroForWing(this CarDefinition def, WingType wing)
        {
            return wing == WingType.HighDownforce ? def.HighDownforceAero : def.LowDownforceAero;
        }

        /// <summary>
        /// Creates a physics profile with the selected wing's aero applied.
        /// Call this when instantiating the car for qualifying/race.
        /// </summary>
        public static VehiclePhysicsProfile CreateProfileWithWing(this CarDefinition def, WingType wing)
        {
            if (def.BasePhysicsProfile == null) return null;

            var profile = ScriptableObject.CreateInstance<VehiclePhysicsProfile>();
            // Copy all fields from base
            JsonUtility.FromJsonOverwrite(JsonUtility.ToJson(def.BasePhysicsProfile), profile);
            // Swap aero using standalone WingAeroProfile's CreateNested
            var aero = def.GetAeroForWing(wing);
            if (aero != null)
                profile.aero = aero.CreateNested();
            return profile;
        }
    }
}