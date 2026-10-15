#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using GLTFast;
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

namespace F1Editor
{
    // glTFast import is async and must not block the editor main thread,
    // so we start it fire-and-forget and let callers poll the state.
    public static class GarageGlbImporter
    {
        public static volatile bool Done;
        public static string Message = "";

        private static GltfImport _import;
        private static HashSet<int> _rootsBefore;
        public static GameObject Root { get; private set; }

        // Editor-safe defer agent: never defers, so import runs synchronously
        // and GltfImport skips creating the play-mode-only StableFramerate object.
        private sealed class NoDefer : IDeferAgent
        {
            public bool ShouldDefer() => false;
            public bool ShouldDefer(float duration) => false;
            public Task BreakPoint() => Task.CompletedTask;
            public Task BreakPoint(float duration) => Task.CompletedTask;
        }

        public static void Begin(string path)
        {
            Done = false;
            Message = "running";
            Root = null;
            var scene = SceneManager.GetActiveScene();
            _rootsBefore = new HashSet<int>(Array.ConvertAll(
                scene.GetRootGameObjects(), go => go.GetInstanceID()));
            _ = Run(path, scene);
        }

        private static async Task Run(string path, Scene scene)
        {
            try
            {
                _import = new GltfImport(deferAgent: new NoDefer());
                var full = System.IO.Path.GetFullPath(path);
                bool loaded = await _import.LoadFile(full);
                if (!loaded)
                {
                    Message = "Load() returned false";
                    Done = true;
                    return;
                }
                bool ok = _import.InstantiateMainScene((Transform)null);
                Message = ok ? "Imported OK" : "InstantiateMainScene() returned false";
                if (ok)
                    foreach (var go in scene.GetRootGameObjects())
                        if (!_rootsBefore.Contains(go.GetInstanceID())) Root = go;
            }
            catch (Exception e)
            {
                Message = "EX: " + e.Message + "\n" + e.StackTrace;
            }
            _import = null;
            Done = true;
        }

        /// Turns the volatile glTFast import into durable assets:
        /// meshes copied to .asset files, materials swapped to the persistent
        /// M_Gar_* set, light intensities/ranges/shadows authored for the hub.
        public static string Finish()
        {
            if (Root == null) return "no root imported";
            var sb = new StringBuilder();

            const string meshDir = "Assets/Art/Models/Garage/Meshes";
            if (!Directory.Exists(meshDir)) Directory.CreateDirectory(meshDir);
            var meshMap = new Dictionary<Mesh, Mesh>();
            int mi = 0;
            foreach (var mf in Root.GetComponentsInChildren<MeshFilter>(true))
            {
                var src = mf.sharedMesh;
                if (src == null || EditorUtility.IsPersistent(src)) continue;
                if (!meshMap.TryGetValue(src, out var asset) || asset == null)
                {
                    var copy = Object.Instantiate(src);
                    copy.name = src.name;
                    string path = AssetDatabase.GenerateUniqueAssetPath(
                        meshDir + "/" + (string.IsNullOrEmpty(src.name) ? "Mesh_" + mi : src.name) + ".asset");
                    AssetDatabase.CreateAsset(copy, path);
                    meshMap[src] = copy;
                    asset = copy;
                    mi++;
                }
                mf.sharedMesh = asset;
            }
            AssetDatabase.SaveAssets();
            sb.AppendLine("persisted " + mi + " unique meshes");

            int swapped = 0;
            foreach (var rend in Root.GetComponentsInChildren<MeshRenderer>(true))
            {
                var arr = rend.sharedMaterials;
                for (int i = 0; i < arr.Length; i++)
                {
                    if (arr[i] == null || EditorUtility.IsPersistent(arr[i])) continue;
                    // Hand-authored assets use two conventions: M_MAT_Gar_X (CopySerialized
                    // batch) and M_Gar_X (rebuilt materials) — try both before skipping.
                    string matPath = "Assets/Materials/Garage/M_" + arr[i].name + ".mat";
                    var asset = AssetDatabase.LoadAssetAtPath<Material>(matPath);
                    if (asset == null && arr[i].name.StartsWith("MAT_"))
                        asset = AssetDatabase.LoadAssetAtPath<Material>(
                            "Assets/Materials/Garage/M_" + arr[i].name.Substring(4) + ".mat");
                    if (asset == null) continue;
                    arr[i] = asset;
                    swapped++;
                }
                rend.sharedMaterials = arr;
            }
            sb.AppendLine("swapped " + swapped + " material slots");

            foreach (var l in Root.GetComponentsInChildren<Light>(true))
            {
                // Runtime renders these punctual lights ~4x brighter than the editor
                // game-view preview; values below were tuned in play mode.
                if (l.name.StartsWith("LGT_Ceil"))
                {
                    l.intensity = 60f; l.range = 30f; l.shadows = LightShadows.None;
                }
                else if (l.name.StartsWith("LGT_Wall"))
                {
                    l.intensity = 120f; l.range = 40f; l.spotAngle = 70f;
                    l.shadows = LightShadows.Hard; l.shadowStrength = 0.85f;
                }
                else if (l.name.StartsWith("LGT_Table"))
                {
                    l.intensity = 100f; l.range = 40f; l.spotAngle = 80f;
                    l.shadows = LightShadows.Hard; l.shadowStrength = 0.85f;
                }
            }
            sb.AppendLine("lights authored");

            UnityEditor.SceneManagement.EditorSceneManager.MarkAllScenesDirty();
            return sb.ToString();
        }
    }
}
#endif
