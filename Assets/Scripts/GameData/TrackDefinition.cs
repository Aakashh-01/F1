using UnityEngine;
using System.Collections.Generic;

namespace F1.GameData
{
    /// <summary>
    /// Defines a track available in the game. Includes sector splits, racing line reference, and unlock data.
    /// </summary>
    [CreateAssetMenu(menuName = "F1/Game Data/Track Definition", fileName = "TrackDefinition_")]
    public class TrackDefinition : ScriptableObject
    {
        [Header("Identity")]
        [Tooltip("Unique stable ID — used for save data keys and leaderboards. Never change after release.")]
        [SerializeField] private string _trackId;
        public string TrackId => _trackId;

        [Tooltip("Display name shown in UI")]
        [SerializeField] private string _displayName;
        public string DisplayName => _displayName;

        [Tooltip("Short code for UI (e.g., \"MON\", \"SPA\", \"MONZA\")")]
        [SerializeField] private string _shortCode;
        public string ShortCode => _shortCode;

        [Header("Scene & Visuals")]
        [SerializeField] private string _sceneName;
        public string SceneName => _sceneName;

        [SerializeField] private Sprite _thumbnail;
        public Sprite Thumbnail => _thumbnail;

        [SerializeField] private Sprite _minimapTexture;
        public Sprite MinimapTexture => _minimapTexture;

        [Tooltip("Marks a circuit that is listed but not yet playable. Needed because a " +
                 "circuit can point at a scene that exists and still not be its own: Spa " +
                 "shares Track_01 with Monaco, so a scene-existence check alone would offer " +
                 "the same corner twice under two names.")]
        [SerializeField] private bool _underConstruction;
        public bool UnderConstruction => _underConstruction;

        [Header("Track Geometry")]
        [Tooltip("Total track length in meters")]
        [SerializeField] private float _trackLengthMeters = 5000f;
        public float TrackLengthMeters => _trackLengthMeters;

        [Tooltip("Number of laps for a standard race")]
        [SerializeField] private int _standardRaceLaps = 15;
        public int StandardRaceLaps => _standardRaceLaps;

        [Header("Sectors — Auto-calculated from racing line if not overridden")]
        [Tooltip("If true, sectors are split evenly by distance (33.3% each). If false, use manual sector distances below.")]
        [SerializeField] private bool _useAutoSectors = true;
        public bool UseAutoSectors => _useAutoSectors;

        [Tooltip("Manual sector start distances (meters from start/finish). Only used if UseAutoSectors = false. Array length must be 3.")]
        [SerializeField] private float[] _sectorStartDistances = new float[3] { 0f, 0f, 0f };
        public float[] SectorStartDistances => _sectorStartDistances;

        [Header("Racing Line")]
        [Tooltip("Reference to the AIRacingLine in the track scene. Used for AI, ghost, track limits, sectors.")]
        [SerializeField] private AIRacingLine _racingLineReference;
        public AIRacingLine RacingLineReference => _racingLineReference;

        [Header("Progression")]
        [Tooltip("Points required to unlock permanently. 0 = free track.")]
        [SerializeField] private int _unlockCostPoints = 0;
        public int UnlockCostPoints => _unlockCostPoints;

        [Tooltip("Direct purchase price (USD) — for IAP integration later")]
        [SerializeField] private float _directPurchasePrice = 0f;
        public float DirectPurchasePrice => _directPurchasePrice;

        [Header("Description")]
        [TextArea(2, 4)]
        [SerializeField] private string _description;
        public string Description => _description;

        [Header("Time Trial / Qualifying")]
        [Tooltip("Developer best lap time (seconds) — shown as \"Track Record\" target")]
        [SerializeField] private float _devBestLapTime = 90f;
        public float DevBestLapTime => _devBestLapTime;

#if UNITY_EDITOR
        private void OnValidate()
        {
            if (string.IsNullOrEmpty(_trackId))
                _trackId = name.Replace("TrackDefinition_", "").Replace(" ", "_").ToLowerInvariant();

            if (string.IsNullOrEmpty(_shortCode))
                _shortCode = _displayName?.Length >= 3 ? _displayName.Substring(0, 3).ToUpperInvariant() : "TRK";

            if (_sectorStartDistances == null || _sectorStartDistances.Length != 3)
                _sectorStartDistances = new float[3] { 0f, 0f, 0f };
        }
#endif
    }

    /// <summary>
    /// Sector identifiers — fixed at 3 sectors per track (F1 standard).
    /// </summary>
    public enum SectorIndex : byte
    {
        Sector1 = 0,
        Sector2 = 1,
        Sector3 = 2
    }

    /// <summary>
    /// Sector timing colors for UI display.
    /// </summary>
    public enum SectorColor
    {
        Purple, // Overall fastest (anyone)
        Green,  // Personal best improvement
        Yellow, // Slower than personal best
        White   // No comparison available
    }

    public static class TrackDefinitionExtensions
    {
        /// <summary>
        /// Gets sector start distances. If UseAutoSectors, calculates evenly spaced splits.
        /// </summary>
        public static float[] GetSectorStartDistances(this TrackDefinition def)
        {
            if (!def.UseAutoSectors && def.SectorStartDistances != null && def.SectorStartDistances.Length == 3)
                return def.SectorStartDistances;

            float third = def.TrackLengthMeters / 3f;
            return new float[3] { 0f, third, third * 2f };
        }

        /// <summary>
        /// Determines which sector a given distance along track falls into.
        /// </summary>
        public static SectorIndex GetSectorAtDistance(this TrackDefinition def, float distanceMeters)
        {
            var starts = def.GetSectorStartDistances();
            if (distanceMeters >= starts[2]) return SectorIndex.Sector3;
            if (distanceMeters >= starts[1]) return SectorIndex.Sector2;
            return SectorIndex.Sector1;
        }

        /// <summary>
        /// Gets the sector color for a sector time compared to personal best and overall best.
        /// </summary>
        public static SectorColor GetSectorColor(float sectorTime, float personalBest, float overallBest)
        {
            if (sectorTime <= overallBest + 0.001f) return SectorColor.Purple;
            if (sectorTime <= personalBest + 0.001f) return SectorColor.Green;
            if (personalBest > 0f) return SectorColor.Yellow;
            return SectorColor.White;
        }
    }
}