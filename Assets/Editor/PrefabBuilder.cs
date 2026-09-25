// Place in Assets/Editor/ and run via Unity menu: Tools > BuildPrefabs
using UnityEngine;
using UnityEditor;
using System.Linq;

public class PrefabBuilder
{
    [MenuItem("Tools/BuildPrefabs")]
    public static void BuildAll()
    {
        string folder = "Assets/Prefabs/";

        string[] prefabNames = {
            "BrandingScreen_Prefab",
            "MainMenuScreen_Prefab",
            "CarSelectionScreen_Prefab",
            "TrackSelectionScreen_Prefab",
            "WingSetupScreen_Prefab",
            "ModeSelectionScreen_Prefab",
            "CarCard_Prefab",
            "TrackCard_Prefab"
        };

        string[] scriptNames = {
            "BrandingScreenImpl",
            "MainMenuScreenImpl",
            "CarSelectionScreenImpl",
            "TrackSelectionScreenImpl",
            "WingSetupScreenImpl",
            "ModeSelectionScreenImpl",
            "CarCardImpl",
            "TrackCardImpl"
        };

        for (int i = 0; i < prefabNames.Length; i++)
        {
            string prefabPath = folder + prefabNames[i] + ".prefab";

            // Check existing
            var existing = AssetDatabase.LoadAssetAtPath(prefabPath, typeof(Object));
            if (existing != null)
            {
                Debug.Log("[PrefabBuilder] Exists: " + prefabNames[i]);
                continue;
            }

            // Create GameObject
            var go = new GameObject(prefabNames[i]);

            // Load the assembly and find the type
            var asm = System.AppDomain.CurrentDomain.GetAssemblies()
                .FirstOrDefault(a => a.GetName().Name == "Assembly-CSharp");

            if (asm != null)
            {
                var targetType = asm.GetTypes()
                    .FirstOrDefault(t => t.Name == scriptNames[i]);

                if (targetType != null)
                {
                    go.AddComponent(targetType);
                    Debug.Log("[PrefabBuilder] Added script: " + scriptNames[i] + " to " + prefabNames[i]);
                }
                else
                {
                    Debug.LogWarning("[PrefabBuilder] Type not found: " + scriptNames[i]);
                }
            }
            else
            {
                Debug.LogError("[PrefabBuilder] Assembly-CSharp not found!");
            }

            PrefabUtility.SaveAsPrefabAsset(go, prefabPath);
            Object.DestroyImmediate(go);
            Debug.Log("[PrefabBuilder] Saved: " + prefabPath);
        }

        Debug.Log("[PrefabBuilder] All prefabs processed.");
    }
}
