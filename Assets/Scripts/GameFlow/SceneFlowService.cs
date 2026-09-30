using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace F1.GameFlow
{
    public enum SceneLoadStatus
    {
        Success,

        /// <summary>A null/empty/whitespace scene name was requested.</summary>
        EmptyName,

        /// <summary>The scene is not present in the build's scene list (finding B1).</summary>
        NotInBuild,

        /// <summary>The engine refused or failed the load.</summary>
        LoadFailed,

        /// <summary>The engine refused or failed the unload.</summary>
        UnloadFailed,

        /// <summary>Another load/unload is still running.</summary>
        Busy
    }

    public readonly struct SceneLoadResult
    {
        public readonly string SceneName;
        public readonly SceneLoadStatus Status;
        public readonly string Message;

        public bool Success => Status == SceneLoadStatus.Success;

        private SceneLoadResult(string sceneName, SceneLoadStatus status, string message)
        {
            SceneName = sceneName;
            Status = status;
            Message = message;
        }

        public static SceneLoadResult Ok(string sceneName) =>
            new SceneLoadResult(sceneName, SceneLoadStatus.Success, "OK");

        public static SceneLoadResult Fail(string sceneName, SceneLoadStatus status, string message) =>
            new SceneLoadResult(sceneName, status, message);

        public override string ToString() =>
            Success ? $"OK({SceneName})" : $"{Status}({SceneName}): {Message}";
    }

    /// <summary>
    /// Owns every scene load in the game. Nothing else should call
    /// <c>SceneManager.LoadSceneAsync</c> or <c>UnloadSceneAsync</c>.
    ///
    /// Two kinds of scene are modelled:
    /// <list type="bullet">
    ///   <item><description><b>Flow scenes</b> — exactly one at a time (loading, car
    ///   selection, qualifying, race, results). A transition unloads the previous flow
    ///   scene and loads the next.</description></item>
    ///   <item><description><b>Content scenes</b> — additive, and allowed to survive
    ///   flow transitions. Track content is the intended case: pre-race and race must
    ///   resolve the same live track objects.</description></item>
    /// </list>
    ///
    /// The current flow scene is tracked by <see cref="Scene"/> handle, never by
    /// comparing a requested name against <c>gameObject.scene.name</c>. That comparison
    /// is useless on a persistent object, whose <c>scene.name</c> is permanently
    /// <c>"DontDestroyOnLoad"</c> (finding L2).
    /// </summary>
    public sealed class SceneFlowService
    {
        private readonly MonoBehaviour _host;
        private readonly List<string> _contentScenes = new List<string>();
        private Scene _currentFlowScene;

        public event Action<float> OnProgressChanged;
        public event Action<SceneLoadResult> OnFlowSceneChanged;
        public event Action<SceneLoadResult> OnContentSceneLoaded;
        public event Action<SceneLoadResult> OnSceneLoadFailed;

        /// <summary>
        /// Raised the moment a FLOW scene load begins, with the destination's name.
        ///
        /// Separate from <see cref="OnProgressChanged"/> because progress fires at 0 for
        /// every operation, content loads included — the track-content load inside the
        /// pre-race scene would raise it too, and a transition screen belongs over a scene
        /// change, not over the player still watching the garage. A consumer that wants a
        /// "loading" card between scenes needs to know a flow load has started, which is
        /// this and nothing else.
        /// </summary>
        public event Action<string> OnFlowSceneLoadStarted;

        public Scene CurrentFlowScene => _currentFlowScene;

        public string CurrentFlowSceneName =>
            _currentFlowScene.IsValid() ? _currentFlowScene.name : null;

        public IReadOnlyList<string> ContentScenes => _contentScenes;

        public bool IsBusy { get; private set; }

        /// <summary>Load progress in 0..1. Reset to 0 at the start of each operation.</summary>
        public float Progress { get; private set; }

        public SceneFlowService(MonoBehaviour host)
        {
            _host = host != null ? host : throw new ArgumentNullException(nameof(host));
        }

        /// <summary>
        /// True when this scene can actually be loaded right now.
        ///
        /// This is the check that catches build-settings entries pointing at files that
        /// no longer exist (finding B1). It cannot rely on
        /// <see cref="Application.CanStreamedLevelBeLoaded"/> in the Editor: that API is
        /// only meaningful in a built player and returns false there for every scene,
        /// including ones that are correctly listed in the build settings. Using it
        /// unedited would fail every transition during development.
        /// </summary>
        public static bool IsSceneAvailable(string sceneName)
        {
            if (string.IsNullOrWhiteSpace(sceneName))
                return false;

#if UNITY_EDITOR
            return IsInBuildSettingsForEditor(sceneName);
#else
            return Application.CanStreamedLevelBeLoaded(sceneName);
#endif
        }

#if UNITY_EDITOR
        private static bool IsInBuildSettingsForEditor(string sceneName)
        {
            var scenes = UnityEditor.EditorBuildSettings.scenes;
            for (int i = 0; i < scenes.Length; i++)
            {
                if (!scenes[i].enabled)
                    continue;

                var path = scenes[i].path;
                bool nameMatches =
                    string.Equals(path, sceneName, StringComparison.OrdinalIgnoreCase) ||
                    string.Equals(
                        System.IO.Path.GetFileNameWithoutExtension(path),
                        sceneName,
                        StringComparison.OrdinalIgnoreCase);
                if (!nameMatches)
                    continue;

                // An enabled entry can still point at a deleted file. AssetPathToGUID
                // returns an empty string for a path the asset database cannot resolve.
                return !string.IsNullOrEmpty(UnityEditor.AssetDatabase.AssetPathToGUID(path));
            }

            return false;
        }
#endif

        public bool IsCurrentFlowScene(string sceneName) =>
            !string.IsNullOrWhiteSpace(sceneName) &&
            _currentFlowScene.IsValid() &&
            _currentFlowScene.name == sceneName;

        public bool IsContentLoaded(string sceneName) =>
            !string.IsNullOrWhiteSpace(sceneName) && _contentScenes.Contains(sceneName);

        /// <summary>
        /// Claims the scene the flow was born in as the current flow scene, so the
        /// first transition can unload it correctly. Must be called with the name
        /// captured <i>before</i> <c>DontDestroyOnLoad</c> was applied.
        /// </summary>
        public void AdoptFlowScene(string sceneName)
        {
            if (string.IsNullOrWhiteSpace(sceneName))
            {
                _currentFlowScene = default;
                return;
            }

            var scene = SceneManager.GetSceneByName(sceneName);
            if (scene.IsValid() && !_contentScenes.Contains(sceneName))
                _currentFlowScene = scene;
        }

        public Coroutine LoadFlowScene(string sceneName, Action<SceneLoadResult> onComplete = null) =>
            _host.StartCoroutine(LoadFlowSceneRoutine(sceneName, onComplete));

        public Coroutine LoadContent(string sceneName, Action<SceneLoadResult> onComplete = null) =>
            _host.StartCoroutine(LoadContentRoutine(sceneName, onComplete));

        public Coroutine UnloadContent(string sceneName, Action onComplete = null) =>
            _host.StartCoroutine(UnloadContentRoutine(sceneName, onComplete));

        public Coroutine UnloadAllContent(Action onComplete = null)
        {
            var names = _contentScenes.ToArray();
            return _host.StartCoroutine(UnloadAllContentRoutine(names, onComplete));
        }

        private IEnumerator LoadFlowSceneRoutine(string sceneName, Action<SceneLoadResult> onComplete)
        {
            if (IsBusy)
            {
                Complete(SceneLoadResult.Fail(sceneName, SceneLoadStatus.Busy,
                    "A scene operation is already in progress."), null, onComplete);
                yield break;
            }

            IsBusy = true;
            Progress = 0f;
            OnProgressChanged?.Invoke(0f);
            // Raised after the busy flag is set, so a listener that immediately checks
            // IsBusy sees a consistent state, and before the validity checks, so a load
            // that is about to fail still announces itself and the overlay has something
            // to hide on. Deliberately only here and not in the content-load path: a
            // transition card belongs over a scene change, not over the player still
            // watching the garage while the track streams in.
            OnFlowSceneLoadStarted?.Invoke(sceneName);

            try
            {
                if (string.IsNullOrWhiteSpace(sceneName))
                {
                    Complete(SceneLoadResult.Fail(sceneName, SceneLoadStatus.EmptyName,
                        "No scene name was configured for this step."), null, onComplete);
                    yield break;
                }

                if (!IsSceneAvailable(sceneName))
                {
                    Complete(SceneLoadResult.Fail(sceneName, SceneLoadStatus.NotInBuild,
                        $"'{sceneName}' is not in the build scene list. Check ProjectSettings/EditorBuildSettings.asset."),
                        null, onComplete);
                    yield break;
                }

                // Self-reference guard, by handle rather than by comparing against a
                // DontDestroyOnLoad scene name.
                if (IsCurrentFlowScene(sceneName))
                {
                    Complete(SceneLoadResult.Ok(sceneName), OnFlowSceneChanged, onComplete);
                    yield break;
                }

                // Remember the outgoing scene. Handles can be invalidated by the unload,
                // so capture it before anything changes.
                var outgoing = _currentFlowScene;
                bool willUnloadOutgoing = outgoing.IsValid() && outgoing.isLoaded &&
                                         !_contentScenes.Contains(outgoing.name);

                // LOAD FIRST, UNLOAD AFTER.
                //
                // Unity refuses to unload the last remaining scene ("Unloading the last
                // loaded scene ... is not supported"). At game boot the loading scene is
                // the only scene loaded, so unloading before loading deadlocks the very
                // first transition. Loading additively first guarantees two scenes exist
                // by the time the outgoing one is released.
                //
                // Additive (not Single) is also what keeps content scenes alive across
                // the transition - the track-content lifetime Phase 5 depends on.
                var loadOp = SceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Additive);
                if (loadOp == null)
                {
                    Complete(SceneLoadResult.Fail(sceneName, SceneLoadStatus.LoadFailed,
                        $"The engine refused to load '{sceneName}'."), null, onComplete);
                    yield break;
                }

                yield return ReportProgress(loadOp);

                var loaded = SceneManager.GetSceneByName(sceneName);
                if (!loaded.IsValid() || !loaded.isLoaded)
                {
                    Complete(SceneLoadResult.Fail(sceneName, SceneLoadStatus.LoadFailed,
                        $"'{sceneName}' reported loaded but could not be resolved."),
                        null, onComplete);
                    yield break;
                }

                _currentFlowScene = loaded;
                SceneManager.SetActiveScene(loaded);

                if (willUnloadOutgoing)
                {
                    var unloadOp = SceneManager.UnloadSceneAsync(outgoing);
                    if (unloadOp == null)
                    {
                        // The new scene is already loaded and active, so the transition
                        // itself succeeded. Surface the leftover rather than failing it.
                        OnSceneLoadFailed?.Invoke(SceneLoadResult.Fail(
                            outgoing.name, SceneLoadStatus.UnloadFailed,
                            $"Entered '{sceneName}' but could not unload the previous flow " +
                            $"scene '{outgoing.name}'. It is still loaded."));
                    }
                    else
                    {
                        yield return unloadOp;
                    }
                }

                Complete(SceneLoadResult.Ok(sceneName), OnFlowSceneChanged, onComplete);
            }
            finally
            {
                IsBusy = false;
            }
        }

        private IEnumerator LoadContentRoutine(string sceneName, Action<SceneLoadResult> onComplete)
        {
            if (IsBusy)
            {
                Complete(SceneLoadResult.Fail(sceneName, SceneLoadStatus.Busy,
                    "A scene operation is already in progress."), null, onComplete);
                yield break;
            }

            IsBusy = true;
            Progress = 0f;
            OnProgressChanged?.Invoke(0f);

            try
            {
                if (string.IsNullOrWhiteSpace(sceneName))
                {
                    Complete(SceneLoadResult.Fail(sceneName, SceneLoadStatus.EmptyName,
                        "No content scene name was supplied."), null, onComplete);
                    yield break;
                }

                if (IsContentLoaded(sceneName))
                {
                    Complete(SceneLoadResult.Ok(sceneName), OnContentSceneLoaded, onComplete);
                    yield break;
                }

                if (!IsSceneAvailable(sceneName))
                {
                    Complete(SceneLoadResult.Fail(sceneName, SceneLoadStatus.NotInBuild,
                        $"'{sceneName}' is not in the build scene list."), null, onComplete);
                    yield break;
                }

                var loadOp = SceneManager.LoadSceneAsync(sceneName, LoadSceneMode.Additive);
                if (loadOp == null)
                {
                    Complete(SceneLoadResult.Fail(sceneName, SceneLoadStatus.LoadFailed,
                        $"The engine refused to load content scene '{sceneName}'."), null, onComplete);
                    yield break;
                }

                yield return ReportProgress(loadOp);

                if (!SceneManager.GetSceneByName(sceneName).IsValid())
                {
                    Complete(SceneLoadResult.Fail(sceneName, SceneLoadStatus.LoadFailed,
                        $"'{sceneName}' reported loaded but could not be resolved."), null, onComplete);
                    yield break;
                }

                _contentScenes.Add(sceneName);
                Complete(SceneLoadResult.Ok(sceneName), OnContentSceneLoaded, onComplete);
            }
            finally
            {
                IsBusy = false;
            }
        }

        private IEnumerator UnloadContentRoutine(string sceneName, Action onComplete)
        {
            if (IsBusy)
            {
                OnSceneLoadFailed?.Invoke(SceneLoadResult.Fail(sceneName, SceneLoadStatus.Busy,
                    "A scene operation is already in progress."));
                onComplete?.Invoke();
                yield break;
            }

            IsBusy = true;

            try
            {
                if (!_contentScenes.Remove(sceneName))
                {
                    // Not tracked — treat as a no-op rather than an error.
                    onComplete?.Invoke();
                    yield break;
                }

                var scene = SceneManager.GetSceneByName(sceneName);
                if (!scene.IsValid() || !scene.isLoaded)
                {
                    onComplete?.Invoke();
                    yield break;
                }

                // If the active scene is going away, promote another loaded scene first.
                if (SceneManager.GetActiveScene() == scene)
                {
                    var replacement = _currentFlowScene.IsValid() && _currentFlowScene.isLoaded
                        ? _currentFlowScene
                        : FindAnyLoadedScene();
                    if (replacement.IsValid())
                        SceneManager.SetActiveScene(replacement);
                }

                var unloadOp = SceneManager.UnloadSceneAsync(scene);
                if (unloadOp == null)
                {
                    OnSceneLoadFailed?.Invoke(SceneLoadResult.Fail(sceneName, SceneLoadStatus.UnloadFailed,
                        $"Could not unload content scene '{sceneName}'."));
                    yield break;
                }

                yield return unloadOp;
                onComplete?.Invoke();
            }
            finally
            {
                IsBusy = false;
            }
        }

        private IEnumerator UnloadAllContentRoutine(string[] sceneNames, Action onComplete)
        {
            for (int i = 0; i < sceneNames.Length; i++)
            {
                yield return UnloadContentRoutine(sceneNames[i], null);
            }

            onComplete?.Invoke();
        }

        private IEnumerator ReportProgress(AsyncOperation op)
        {
            while (!op.isDone)
            {
                // Scene activation parks progress at 0.9; map the load itself onto 0..1.
                Progress = Mathf.Clamp01(op.progress / 0.9f);
                OnProgressChanged?.Invoke(Progress);
                yield return null;
            }

            Progress = 1f;
            OnProgressChanged?.Invoke(1f);
        }

        private void Complete(SceneLoadResult result, Action<SceneLoadResult> onSuccess,
            Action<SceneLoadResult> onComplete)
        {
            if (result.Success)
                onSuccess?.Invoke(result);
            else
                OnSceneLoadFailed?.Invoke(result);

            onComplete?.Invoke(result);
        }

        private static Scene FindAnyLoadedScene()
        {
            for (int i = 0; i < SceneManager.sceneCount; i++)
            {
                var scene = SceneManager.GetSceneAt(i);
                if (scene.IsValid() && scene.isLoaded)
                    return scene;
            }

            return default;
        }
    }
}
