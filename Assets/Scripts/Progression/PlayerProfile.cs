using UnityEngine;
using System.Collections.Generic;
using System;
using F1.GameData;

namespace F1.Progression
{
    /// <summary>
    /// Player's persistent progression data. Saved to JSON in persistentDataPath.
    /// Designed for forward compatibility — unknown fields are ignored on load.
    /// </summary>
    [Serializable]
    public class PlayerProfile
    {
        [Header("Identity")]
        public string playerName = "Player";
        public int playerLevel = 1;
        public long totalXP = 0;

        [Header("Currency")]
        public int careerPoints = 0;      // Earned from races, used for unlocks
        public int premiumCurrency = 0;   // For IAP / direct purchases (future)

        [Header("Owned Content — Car IDs")]
        public List<string> ownedCarIds = new List<string>();

        [Header("Owned Content — Track IDs")]
        public List<string> ownedTrackIds = new List<string>();

        [Header("Active Rentals — Car ID -> Expiry Timestamp (Unix UTC)")]
        public SerializableDictionary<string, long> activeRentals = new SerializableDictionary<string, long>();

        [Header("Best Times — Track ID -> Best Lap Time (seconds)")]
        public SerializableDictionary<string, float> bestLapTimes = new SerializableDictionary<string, float>();

        [Header("Best Sector Times — Track ID -> Sector Index -> Time")]
        public SerializableDictionary<string, SerializableDictionary<int, float>> bestSectorTimes = 
            new SerializableDictionary<string, SerializableDictionary<int, float>>();

        [Header("Ghost Data — Track ID -> Ghost Lap Data (JSON)")]
        public SerializableDictionary<string, string> ghostLapData = new SerializableDictionary<string, string>();

        [Header("Wing Preference — Track ID -> Wing Type (0=High, 1=Low)")]
        public SerializableDictionary<string, int> wingPreferencePerTrack = new SerializableDictionary<string, int>();

        [Header("Statistics")]
        public int totalRacesStarted = 0;
        public int totalRacesFinished = 0;
        public int totalWins = 0;
        public int totalPoles = 0;
        public float totalDistanceDrivenKm = 0f;
        public long totalPlayTimeSeconds = 0;

        [Header("Settings")]
        public int steeringAssistLevel = 1; // 0=Low, 1=Medium, 2=High
        public bool autoThrottle = true;
        public bool vibrationEnabled = true;
        public float musicVolume = 0.7f;
        public float sfxVolume = 0.8f;

        // --- Runtime helpers (not serialized) ---
        [NonSerialized] private bool _isDirty;

        public bool IsDirty => _isDirty;

        public void MarkDirty() => _isDirty = true;
        public void MarkClean() => _isDirty = false;

        // --- Car ownership ---
        public bool OwnsCar(string carId) => ownedCarIds.Contains(carId);
        public void GrantCar(string carId) { if (!OwnsCar(carId)) { ownedCarIds.Add(carId); MarkDirty(); } }
        public bool CanAffordCar(int cost) => careerPoints >= cost;
        public bool SpendPoints(int cost) { if (CanAffordCar(cost)) { careerPoints -= cost; MarkDirty(); return true; } return false; }

        // --- Track ownership ---
        public bool OwnsTrack(string trackId) => ownedTrackIds.Contains(trackId);
        public void GrantTrack(string trackId) { if (!OwnsTrack(trackId)) { ownedTrackIds.Add(trackId); MarkDirty(); } }
        public bool CanAffordTrack(int cost) => careerPoints >= cost;

        // --- Rentals ---
        public bool HasActiveRental(string carId)
        {
            if (!activeRentals.TryGetValue(carId, out long expiry)) return false;
            return DateTimeOffset.UtcNow.ToUnixTimeSeconds() < expiry;
        }

        public void StartRental(string carId, int hours = 24)
        {
            long expiry = DateTimeOffset.UtcNow.ToUnixTimeSeconds() + hours * 3600L;
            activeRentals[carId] = expiry;
            MarkDirty();
        }

        public void CleanExpiredRentals()
        {
            long now = DateTimeOffset.UtcNow.ToUnixTimeSeconds();
            var toRemove = new List<string>();
            foreach (var kvp in activeRentals)
                if (kvp.Value <= now) toRemove.Add(kvp.Key);
            foreach (var id in toRemove) activeRentals.Remove(id);
            if (toRemove.Count > 0) MarkDirty();
        }

        public bool IsCarAvailable(string carId) => OwnsCar(carId) || HasActiveRental(carId);

        // --- Best times ---
        public float GetBestLapTime(string trackId) => bestLapTimes.TryGetValue(trackId, out float t) ? t : 0f;
        public bool HasBestLapTime(string trackId) => bestLapTimes.ContainsKey(trackId);

        public void SetBestLapTime(string trackId, float time)
        {
            if (time <= 0f) return;
            float current = GetBestLapTime(trackId);
            if (current == 0f || time < current)
            {
                bestLapTimes[trackId] = time;
                MarkDirty();
            }
        }

        public float GetBestSectorTime(string trackId, int sectorIndex)
        {
            if (bestSectorTimes.TryGetValue(trackId, out var sectors) && sectors.TryGetValue(sectorIndex, out float t))
                return t;
            return 0f;
        }

        public void SetBestSectorTime(string trackId, int sectorIndex, float time)
        {
            if (time <= 0f) return;
            if (!bestSectorTimes.TryGetValue(trackId, out var sectors))
            {
                sectors = new SerializableDictionary<int, float>();
                bestSectorTimes[trackId] = sectors;
            }
            float current = sectors.TryGetValue(sectorIndex, out float existing) ? existing : 0f;
            if (current == 0f || time < current)
            {
                sectors[sectorIndex] = time;
                MarkDirty();
            }
        }

        // --- Ghost ---
        public string GetGhostData(string trackId) => ghostLapData.TryGetValue(trackId, out string data) ? data : null;
        public void SetGhostData(string trackId, string jsonData) { ghostLapData[trackId] = jsonData; MarkDirty(); }
        public bool HasGhostData(string trackId) => ghostLapData.ContainsKey(trackId);

        // --- Wing preference ---
        public int GetWingPreference(string trackId) => wingPreferencePerTrack.TryGetValue(trackId, out int wing) ? wing : 0;
        public void SetWingPreference(string trackId, int wingType) { wingPreferencePerTrack[trackId] = wingType; MarkDirty(); }

        // --- Statistics ---
        public void RecordRaceStart() { totalRacesStarted++; MarkDirty(); }
        public void RecordRaceFinish(int position)
        {
            totalRacesFinished++;
            if (position == 1) { totalWins++; totalPoles++; } // Pole = won quali = P1 grid
            MarkDirty();
        }
        public void AddDistance(float km) { totalDistanceDrivenKm += km; MarkDirty(); }
        public void AddPlayTime(int seconds) { totalPlayTimeSeconds += seconds; MarkDirty(); }
        public void AddXP(int xp) { totalXP += xp; CheckLevelUp(); MarkDirty(); }
        public void AddCareerPoints(int points) { careerPoints += points; MarkDirty(); }

        private void CheckLevelUp()
        {
            // Simple XP curve: level * 1000
            int requiredXP = playerLevel * 1000;
            while (totalXP >= requiredXP)
            {
                playerLevel++;
                requiredXP = playerLevel * 1000;
            }
        }
    }

    /// <summary>
    /// Serializable Dictionary for JSON serialization support.
    /// Unity's JsonUtility doesn't support Dictionary directly.
    /// </summary>
    [Serializable]
    public class SerializableDictionary<TKey, TValue> : Dictionary<TKey, TValue>, ISerializationCallbackReceiver
    {
        [SerializeField] private List<TKey> _keys = new List<TKey>();
        [SerializeField] private List<TValue> _values = new List<TValue>();

        public void OnBeforeSerialize()
        {
            _keys.Clear();
            _values.Clear();
            foreach (var kvp in this)
            {
                _keys.Add(kvp.Key);
                _values.Add(kvp.Value);
            }
        }

        public void OnAfterDeserialize()
        {
            Clear();
            int count = Math.Min(_keys.Count, _values.Count);
            for (int i = 0; i < count; i++)
                Add(_keys[i], _values[i]);
        }
    }

    /// <summary>
    /// Save/Load manager for PlayerProfile. Uses JSON in persistentDataPath.
    /// </summary>
    public static class PlayerProfileManager
    {
        private const string FILE_NAME = "player_profile.json";
        private static string FilePath => System.IO.Path.Combine(Application.persistentDataPath, FILE_NAME);
        private static PlayerProfile _currentProfile;
        private static readonly object _lock = new object();

        public static PlayerProfile Current
        {
            get
            {
                lock (_lock)
                {
                    if (_currentProfile == null) Load();
                    return _currentProfile;
                }
            }
        }

        public static event Action<PlayerProfile> OnProfileLoaded;
        public static event Action<PlayerProfile> OnProfileSaved;

        public static void Load()
        {
            lock (_lock)
            {
                if (System.IO.File.Exists(FilePath))
                {
                    try
                    {
                        string json = System.IO.File.ReadAllText(FilePath);
                        _currentProfile = JsonUtility.FromJson<PlayerProfile>(json);
                        if (_currentProfile == null) _currentProfile = CreateNewProfile();
                        _currentProfile.CleanExpiredRentals();
                    }
                    catch (Exception e)
                    {
                        UnityEngine.Debug.LogError($"[PlayerProfileManager] Failed to load profile: {e.Message}");
                        _currentProfile = CreateNewProfile();
                    }
                }
                else
                {
                    _currentProfile = CreateNewProfile();
                    Save(); // Create file immediately
                }

                OnProfileLoaded?.Invoke(_currentProfile);
            }
        }

        public static void Save()
        {
            lock (_lock)
            {
                if (_currentProfile == null) return;

                try
                {
                    string json = JsonUtility.ToJson(_currentProfile, true);
                    System.IO.File.WriteAllText(FilePath, json);
                    _currentProfile.MarkClean();
                    OnProfileSaved?.Invoke(_currentProfile);
                }
                catch (Exception e)
                {
                    UnityEngine.Debug.LogError($"[PlayerProfileManager] Failed to save profile: {e.Message}");
                }
            }
        }

        public static void SaveAsync()
        {
            // Fire-and-forget for non-blocking saves
            System.Threading.Tasks.Task.Run(() => Save());
        }

        public static void ResetToNew()
        {
            lock (_lock)
            {
                _currentProfile = CreateNewProfile();
                Save();
            }
        }

        private static PlayerProfile CreateNewProfile()
        {
            var profile = new PlayerProfile();
            GrantStarterContent(profile);
            return profile;
        }

        /// <summary>
        /// Grants the GDD's starting content: one free starter car and every free track.
        ///
        /// Without this a fresh profile owns nothing, <c>IsCarAvailable</c> rejects every
        /// car, and the flow dead-ends at car selection. Extracted from
        /// <see cref="CreateNewProfile"/> so it can be exercised without touching the
        /// on-disk save.
        /// </summary>
        public static void GrantStarterContent(PlayerProfile profile)
        {
            if (profile == null)
                return;

            GameDataRegistry.Initialize();

            var starterCars = GameDataRegistry.FreeCars;
            if (starterCars.Count > 0)
            {
                // FreeCars is ordered by generation then display name, so the first
                // entry is the entry-level car.
                profile.GrantCar(starterCars[0].CarId);
            }
            else
            {
                UnityEngine.Debug.LogWarning(
                    "[PlayerProfileManager] No free starter car is defined. New profiles cannot select a car.");
            }

            foreach (var track in GameDataRegistry.FreeTracks)
                profile.GrantTrack(track.TrackId);
        }

        /// <summary>
        /// Call this periodically (e.g., OnApplicationPause, OnApplicationFocus) to auto-save.
        /// </summary>
        public static void AutoSaveIfDirty()
        {
            if (_currentProfile?.IsDirty == true) Save();
        }
    }
}