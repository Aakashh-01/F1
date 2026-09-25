// Assets/Scripts/GameData/SceneAvailability.cs
//
// Answers whether a circuit a TrackDefinition points at is actually shippable.
//
// The five track definitions name five circuits, but the project only contains one:
// Monaco and Spa both point at Track_01, and Monza, Silverstone and Suzuka point at
// scenes that do not exist. Presenting all five as equally playable is how a demo ends
// up with a client clicking Suzuka and falling off a cliff.
//
// Rather than hardcoding a "coming soon" flag that can drift out of sync with the
// project, this reads the build settings. Drop a real MonzaGP scene in and add it to
// the build list, and that card lights up on its own, with no code change and no second
// place to remember to update.
//
// Lives beside TrackDefinition rather than with the flow code because the selection UI
// needs it and the flow assembly is not referenced from there.
using System.IO;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace F1.GameData
{
    public static class SceneAvailability
    {
        /// <summary>
        /// True when <paramref name="sceneName"/> names a scene that is present and enabled
        /// in the build list. In a player build this defers to Unity's own check.
        /// </summary>
        public static bool IsAvailable(string sceneName)
        {
            if (string.IsNullOrEmpty(sceneName))
                return false;

#if UNITY_EDITOR
            // Checked against the build list rather than the filesystem: a scene that is on
            // disk but absent from the build list cannot be loaded by name at runtime, so it
            // would fail exactly like a missing one.
            foreach (var scene in EditorBuildSettings.scenes)
            {
                if (!scene.enabled)
                    continue;

                if (Path.GetFileNameWithoutExtension(scene.path) == sceneName)
                    return true;
            }

            return false;
#else
            return Application.CanStreamedLevelUnlocked(sceneName);
#endif
        }

        /// <summary>
        /// True when the given definition names a circuit that can actually be raced.
        ///
        /// Both conditions matter. The scene has to exist, which is automatic and cannot
        /// drift. And the circuit must not be flagged as under construction, because a
        /// definition can point at a scene that exists and still not be a distinct circuit:
        /// Spa and Monaco both point at Track_01, so checking the scene alone would offer the
        /// same corner twice under two different names.
        /// </summary>
        public static bool IsAvailable(TrackDefinition track)
        {
            return track != null && !track.UnderConstruction && IsAvailable(track.SceneName);
        }
    }
}
