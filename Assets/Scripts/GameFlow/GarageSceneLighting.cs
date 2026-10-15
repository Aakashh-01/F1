using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

namespace F1.GameFlow
{
    /// Forces the garage's dark studio lighting while its scene is loaded.
    ///
    /// The hub is additive-loaded over flow shells whose serialized ambient is the
    /// default skybox, and additive loads/unloads re-apply whichever scene's
    /// lighting wins last — which floods the garage and blows out the strips.
    /// This claims the global RenderSettings in OnEnable and hands them back on
    /// disable so other scenes keep their own look. Re-asserted whenever a scene
    /// finishes loading or unloading, because a shell unloading after us would
    /// otherwise restore its bright skybox ambient behind our back.
    [DisallowMultipleComponent]
    public class GarageSceneLighting : MonoBehaviour
    {
        [SerializeField] private AmbientMode _ambientMode = AmbientMode.Flat;
        [ColorUsage(true, true)]
        [SerializeField] private Color _ambientLight = new Color(0.012f, 0.013f, 0.017f);
        [SerializeField] private float _ambientIntensity = 1f;

        private AmbientMode _savedMode;
        private Color _savedColor;
        private float _savedIntensity;
        private Material _savedSkybox;

        private void OnEnable()
        {
            _savedMode = RenderSettings.ambientMode;
            _savedColor = RenderSettings.ambientLight;
            _savedIntensity = RenderSettings.ambientIntensity;
            _savedSkybox = RenderSettings.skybox;

            SceneManager.sceneLoaded += OnSceneLoaded;
            SceneManager.sceneUnloaded += OnSceneUnloaded;
            Apply();
        }

        private void OnDisable()
        {
            SceneManager.sceneLoaded -= OnSceneLoaded;
            SceneManager.sceneUnloaded -= OnSceneUnloaded;

            RenderSettings.ambientMode = _savedMode;
            RenderSettings.ambientLight = _savedColor;
            RenderSettings.ambientIntensity = _savedIntensity;
            RenderSettings.skybox = _savedSkybox;
            DynamicGI.UpdateEnvironment();
        }

        private void OnSceneLoaded(Scene scene, LoadSceneMode mode) => Apply();

        private void OnSceneUnloaded(Scene scene) => Apply();

        private void Apply()
        {
            RenderSettings.ambientMode = _ambientMode;
            RenderSettings.ambientLight = _ambientLight;
            RenderSettings.ambientIntensity = _ambientIntensity;
            RenderSettings.skybox = null;
            DynamicGI.UpdateEnvironment();
        }
    }
}
