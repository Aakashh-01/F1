using UnityEngine;
using System;
using System.Collections.Generic;
using System.Linq;
using F1.GameData;

namespace F1.GameData
{
    /// <summary>
    /// Central registry for all game data definitions (Cars, Tracks).
    /// Loads all ScriptableObjects from Resources/GameData at startup.
    /// Provides fast lookup by ID. NO dependency on Progression.
    /// </summary>
    public static class GameDataRegistry
    {
        private static bool _initialized = false;
        private static readonly Dictionary<string, CarDefinition> _carsById = new Dictionary<string, CarDefinition>();
        private static readonly Dictionary<string, TrackDefinition> _tracksById = new Dictionary<string, TrackDefinition>();
        private static readonly List<CarDefinition> _allCars = new List<CarDefinition>();
        private static readonly List<TrackDefinition> _allTracks = new List<TrackDefinition>();

        // Generation-filtered caches
        private static readonly Dictionary<int, List<CarDefinition>> _carsByGeneration = new Dictionary<int, List<CarDefinition>>();
        private static readonly List<CarDefinition> _freeCars = new List<CarDefinition>();
        private static readonly List<CarDefinition> _rentalCars = new List<CarDefinition>();
        private static readonly List<TrackDefinition> _freeTracks = new List<TrackDefinition>();

        public static IReadOnlyList<CarDefinition> AllCars => _allCars;
        public static IReadOnlyList<TrackDefinition> AllTracks => _allTracks;

        /// <summary>
        /// Cars that cost no points to own — excluding rental variants. A rental variant
        /// is never granted on ownership, so including them here would hand every new
        /// profile the whole garage.
        /// </summary>
        public static IReadOnlyList<CarDefinition> FreeCars => _freeCars;

        /// <summary>Cars that can only be used by starting a 24-hour rental.</summary>
        public static IReadOnlyList<CarDefinition> RentalCars => _rentalCars;

        public static IReadOnlyList<TrackDefinition> FreeTracks => _freeTracks;

        public static event Action OnRegistryInitialized;

        /// <summary>
        /// Call once at game startup (e.g., from GameFlowManager.Awake).
        /// </summary>
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        public static void Initialize()
        {
            if (_initialized) return;

            LoadAllDefinitions();
            BuildCaches();
            _initialized = true;
            OnRegistryInitialized?.Invoke();

            UnityEngine.Debug.Log($"[GameDataRegistry] Initialized: {_allCars.Count} cars, {_allTracks.Count} tracks");
        }

        private static void LoadAllDefinitions()
        {
            // Cars
            var carDefs = Resources.LoadAll<CarDefinition>("GameData/Cars");
            foreach (var def in carDefs)
            {
                if (string.IsNullOrEmpty(def.CarId))
                {
                    UnityEngine.Debug.LogWarning($"[GameDataRegistry] Car {def.name} has empty CarId, skipping");
                    continue;
                }
                if (_carsById.ContainsKey(def.CarId))
                {
                    UnityEngine.Debug.LogWarning($"[GameDataRegistry] Duplicate CarId '{def.CarId}' on {def.name}, skipping");
                    continue;
                }
                _carsById[def.CarId] = def;
                _allCars.Add(def);
            }

            // Tracks
            var trackDefs = Resources.LoadAll<TrackDefinition>("GameData/Tracks");
            foreach (var def in trackDefs)
            {
                if (string.IsNullOrEmpty(def.TrackId))
                {
                    UnityEngine.Debug.LogWarning($"[GameDataRegistry] Track {def.name} has empty TrackId, skipping");
                    continue;
                }
                if (_tracksById.ContainsKey(def.TrackId))
                {
                    UnityEngine.Debug.LogWarning($"[GameDataRegistry] Duplicate TrackId '{def.TrackId}' on {def.name}, skipping");
                    continue;
                }
                _tracksById[def.TrackId] = def;
                _allTracks.Add(def);
            }

            // Sort for consistent UI ordering
            _allCars.Sort((a, b) => a.Generation != b.Generation ? a.Generation.CompareTo(b.Generation) : a.DisplayName.CompareTo(b.DisplayName));
            _allTracks.Sort((a, b) => a.DisplayName.CompareTo(b.DisplayName));
        }

        private static void BuildCaches()
        {
            _carsByGeneration.Clear();
            _freeCars.Clear();
            _rentalCars.Clear();
            _freeTracks.Clear();

            foreach (var car in _allCars)
            {
                if (!_carsByGeneration.ContainsKey(car.Generation))
                    _carsByGeneration[car.Generation] = new List<CarDefinition>();
                _carsByGeneration[car.Generation].Add(car);

                if (car.IsRental)
                    _rentalCars.Add(car);
                else if (car.UnlockCostPoints == 0)
                    _freeCars.Add(car);
            }

            foreach (var track in _allTracks)
            {
                if (track.UnlockCostPoints == 0)
                    _freeTracks.Add(track);
            }
        }

        // --- Core lookup methods (no Progression dependency) ---

        public static CarDefinition GetCar(string carId)
        {
            _carsById.TryGetValue(carId, out var def);
            return def;
        }

        public static TrackDefinition GetTrack(string trackId)
        {
            _tracksById.TryGetValue(trackId, out var def);
            return def;
        }

        public static bool TryGetCar(string carId, out CarDefinition def) => _carsById.TryGetValue(carId, out def);
        public static bool TryGetTrack(string trackId, out TrackDefinition def) => _tracksById.TryGetValue(trackId, out def);

        public static IReadOnlyList<CarDefinition> GetCarsByGeneration(int generation)
        {
            return _carsByGeneration.TryGetValue(generation, out var list) ? list : EmptyList<CarDefinition>.Instance;
        }

        // --- Editor utility ---
#if UNITY_EDITOR
        [UnityEditor.MenuItem("F1/Game Data/Log Registry Contents")]
        public static void LogRegistryContents()
        {
            Initialize();
            UnityEngine.Debug.Log($"=== Cars ({_allCars.Count}) ===");
            foreach (var c in _allCars) UnityEngine.Debug.Log($"  {c.CarId} | Gen{c.Generation} | {c.DisplayName} | Unlock: {c.UnlockCostPoints}pts | Rent: ${c.RentalCost24h}");
            UnityEngine.Debug.Log($"=== Tracks ({_allTracks.Count}) ===");
            foreach (var t in _allTracks) UnityEngine.Debug.Log($"  {t.TrackId} | {t.DisplayName} ({t.ShortCode}) | Unlock: {t.UnlockCostPoints}pts");
        }
#endif

        // Helper for empty lists
        internal static class EmptyList<T>
        {
            public static readonly List<T> Instance = new List<T>(0);
        }
    }
}