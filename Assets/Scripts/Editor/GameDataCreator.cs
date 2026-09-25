using UnityEngine;
using UnityEditor;
using F1.GameData;
using WingAeroProfile = F1.WingAeroProfile;

namespace F1.Editor
{
    public static class GameDataCreator
    {
        [MenuItem("F1/Game Data/Create Sample Car Definitions")]
        public static void CreateSampleCars()
        {
            string carsPath = "Assets/Resources/GameData/Cars";
            string tracksPath = "Assets/Resources/GameData/Tracks";

            // Ensure folders exist
            if (!AssetDatabase.IsValidFolder("Assets/Resources")) AssetDatabase.CreateFolder("Assets", "Resources");
            if (!AssetDatabase.IsValidFolder("Assets/Resources/GameData")) AssetDatabase.CreateFolder("Assets/Resources", "GameData");
            if (!AssetDatabase.IsValidFolder(carsPath)) AssetDatabase.CreateFolder("Assets/Resources/GameData", "Cars");
            if (!AssetDatabase.IsValidFolder(tracksPath)) AssetDatabase.CreateFolder("Assets/Resources/GameData", "Tracks");

            // --- Find existing physics profiles to reference ---
            var allPhysicsProfiles = AssetDatabase.FindAssets("t:VehiclePhysicsProfile");
            VehiclePhysicsProfile defaultProfile = null;
            if (allPhysicsProfiles.Length > 0)
            {
                defaultProfile = AssetDatabase.LoadAssetAtPath<VehiclePhysicsProfile>(AssetDatabase.GUIDToAssetPath(allPhysicsProfiles[0]));
            }

            // Create standalone WingAeroProfiles for wing variants
            var highDownforceAero = CreateWingAeroProfile("HighDownforce_Aero", 7.5f, 0.45f, 60f);
            var lowDownforceAero = CreateWingAeroProfile("LowDownforce_Aero", 3.5f, 0.30f, 90f);

            AssetDatabase.CreateAsset(highDownforceAero, $"{carsPath}/HighDownforce_Aero.asset");
            AssetDatabase.CreateAsset(lowDownforceAero, $"{carsPath}/LowDownforce_Aero.asset");

            // --- Car Definitions ---
            var cars = new[]
            {
                new CarData("car_gen1_starter", "F1 Gen 1 - Starter", 1, 0, 0f, 0.50f, 
                    "Entry-level F1 car. Balanced handling, forgiving physics. Free for all players."),
                new CarData("car_gen1_rental", "F1 Gen 1 - Rental", 1, 0, 0f, 0.50f, 
                    "24-hour rental version. Times don't count for rankings."),
                new CarData("car_gen2_unlock", "F1 Gen 2 - Pro", 2, 50000, 5.99f, 1.00f, 
                    "Second generation. More downforce, higher top speed. Requires skill to extract performance."),
                new CarData("car_gen2_rental", "F1 Gen 2 - Rental", 2, 0, 0f, 1.00f, 
                    "24-hour rental version. Times don't count for rankings."),
                new CarData("car_gen3_unlock", "F1 Gen 3 - Elite", 3, 100000, 7.99f, 2.00f, 
                    "Top-tier F1 car. Maximum downforce, highest top speed. Demands precision driving."),
                new CarData("car_gen3_rental", "F1 Gen 3 - Rental", 3, 0, 0f, 2.00f, 
                    "24-hour rental version. Times don't count for rankings.")
            };

            foreach (var carData in cars)
            {
                var def = ScriptableObject.CreateInstance<CarDefinition>();
                def.name = $"CarDefinition_{carData.id}";
                
                // Use reflection to set private fields
                var so = new SerializedObject(def);
                so.FindProperty("_carId").stringValue = carData.id;
                so.FindProperty("_displayName").stringValue = carData.displayName;
                so.FindProperty("_generation").intValue = carData.generation;
                so.FindProperty("_basePhysicsProfile").objectReferenceValue = defaultProfile;
                so.FindProperty("_highDownforceAero").objectReferenceValue = highDownforceAero;
                so.FindProperty("_lowDownforceAero").objectReferenceValue = lowDownforceAero;
                so.FindProperty("_unlockCostPoints").intValue = carData.unlockCost;
                so.FindProperty("_directPurchasePrice").floatValue = carData.purchasePrice;
                so.FindProperty("_rentalCost24h").floatValue = carData.rentalCost;
                so.FindProperty("_description").stringValue = carData.description;
                so.ApplyModifiedProperties();

                AssetDatabase.CreateAsset(def, $"{carsPath}/{def.name}.asset");
            }

            // --- Track Definitions ---
            var tracks = new[]
            {
                new TrackData("track_monaco", "Monaco", "MON", "MonacoGP", 3337f, 78, 0, 3.99f,
                    "Legendary street circuit. Tight corners, zero margin for error. Qualifying is everything here."),
                new TrackData("track_spa", "Spa-Francorchamps", "SPA", "SpaGP", 7004f, 44, 0, 3.99f,
                    "The Ardennes rollercoaster. High-speed sweepers, dramatic elevation. Eau Rouge defines courage."),
                new TrackData("track_monza", "Monza", "MONZA", "MonzaGP", 5793f, 53, 50000, 3.99f,
                    "Temple of Speed. Long straights, heavy braking zones. Low downforce mandatory."),
                new TrackData("track_silverstone", "Silverstone", "SIL", "SilverstoneGP", 5891f, 52, 50000, 3.99f,
                    "Birthplace of F1. Fast flowing corners. High downforce rewards commitment."),
                new TrackData("track_suzuka", "Suzuka", "SUZ", "SuzukaGP", 5807f, 53, 50000, 3.99f,
                    "Figure-eight masterpiece. Sector 1 essess demand rhythm. 130R tests bravery.")
            };

            foreach (var trackData in tracks)
            {
                var def = ScriptableObject.CreateInstance<TrackDefinition>();
                def.name = $"TrackDefinition_{trackData.id}";

                var so = new SerializedObject(def);
                so.FindProperty("_trackId").stringValue = trackData.id;
                so.FindProperty("_displayName").stringValue = trackData.displayName;
                so.FindProperty("_shortCode").stringValue = trackData.shortCode;
                so.FindProperty("_sceneName").stringValue = trackData.sceneName;
                so.FindProperty("_trackLengthMeters").floatValue = trackData.length;
                so.FindProperty("_standardRaceLaps").intValue = trackData.laps;
                so.FindProperty("_unlockCostPoints").intValue = trackData.unlockCost;
                so.FindProperty("_directPurchasePrice").floatValue = trackData.purchasePrice;
                so.FindProperty("_description").stringValue = trackData.description;
                so.FindProperty("_devBestLapTime").floatValue = trackData.length / 50f; // Rough estimate
                so.ApplyModifiedProperties();

                AssetDatabase.CreateAsset(def, $"{tracksPath}/{def.name}.asset");
            }

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            EditorUtility.DisplayDialog("Game Data Created", 
                $"Created {cars.Length} cars and {tracks.Length} tracks in Resources/GameData/", "OK");
        }

        private static WingAeroProfile CreateWingAeroProfile(string name, float downforceCoeff, float frontBias, float fullDownforceSpeed)
        {
            var profile = ScriptableObject.CreateInstance<WingAeroProfile>();
            profile.name = name;
            var so = new SerializedObject(profile);
            so.FindProperty("downforceCoeff").floatValue = downforceCoeff;
            so.FindProperty("frontBias").floatValue = frontBias;
            so.FindProperty("fullDownforceSpeed").floatValue = fullDownforceSpeed;
            so.ApplyModifiedProperties();
            return profile;
        }

        private struct CarData
        {
            public string id;
            public string displayName;
            public int generation;
            public int unlockCost;
            public float purchasePrice;
            public float rentalCost;
            public string description;

            public CarData(string id, string displayName, int generation, int unlockCost, float purchasePrice, float rentalCost, string description)
            {
                this.id = id; this.displayName = displayName; this.generation = generation;
                this.unlockCost = unlockCost; this.purchasePrice = purchasePrice; this.rentalCost = rentalCost;
                this.description = description;
            }
        }

        private struct TrackData
        {
            public string id;
            public string displayName;
            public string shortCode;
            public string sceneName;
            public float length;
            public int laps;
            public int unlockCost;
            public float purchasePrice;
            public string description;

            public TrackData(string id, string displayName, string shortCode, string sceneName, float length, int laps, int unlockCost, float purchasePrice, string description)
            {
                this.id = id; this.displayName = displayName; this.shortCode = shortCode; this.sceneName = sceneName;
                this.length = length; this.laps = laps; this.unlockCost = unlockCost; this.purchasePrice = purchasePrice;
                this.description = description;
            }
        }
    }
}