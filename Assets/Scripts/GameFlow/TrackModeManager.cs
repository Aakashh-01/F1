using UnityEngine;
using UnityEngine.SceneManagement;
using F1.GameData;
using F1.GameFlow;

namespace F1.GameFlow
{
    /// <summary>
    /// Configures the shared track scene for a race session.
    ///
    /// Once handled qualifying as well, when the track scene doubled as both the qualifying
    /// and the race scene. S4 gave qualifying its own shell — <c>40_PreRaceScene</c>, with
    /// <see cref="PreRaceSceneController"/> — and the qualifying half of this component
    /// moved there with it. That is the correct home for it: the HUD, the car and the ghost
    /// are session concerns, and this component lives in the track *content* scene, which
    /// owns geometry and track references and nothing else (invariant 18).
    ///
    /// The race half stays here for now because the race still routes to this scene, and
    /// moves to <c>50_RaceScene</c> when S5 builds that shell. At that point this component
    /// has no remaining job and can be deleted rather than moved a second time.
    /// </summary>
    public class TrackModeManager : MonoBehaviour
    {
        [Header("Track Identity")] [SerializeField]
        private string _trackId;

        [Header("Race Mode Objects (optional)")]
        [SerializeField] private GameObject _aiOpponentPrefab;

        [SerializeField] private int _maxAIOpponents = 5;

        [Header("Lap Configuration")] [SerializeField]
        private int _raceLapsOverride = -1; // -1 = use track default

        private GameFlowManager _flow;
        private TrackDefinition _trackDefinition;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null)
            {
                Debug.LogError("[TrackModeManager] No GameFlowManager found!");
                return;
            }

            if (_flow.CurrentSessionType != SessionType.Race)
            {
                // The qualifying session is owned by PreRaceSceneController. Doing nothing
                // here is correct, not a gap: this scene is content that the qualifying
                // shell loads underneath itself.
                return;
            }

            // Prefer the track the session actually selected. This scene is reusable track
            // content shared by every track definition, so a hard-coded id here was always
            // going to be wrong for most selections - it resolved to null and the race HUD
            // was configured with no track at all.
            _trackDefinition = _flow.SelectedTrack;
            if (_trackDefinition == null && !string.IsNullOrEmpty(_trackId))
                _trackDefinition = GameDataRegistry.GetTrack(_trackId);

            if (_trackDefinition == null)
                Debug.LogWarning(
                    $"[TrackModeManager] No track definition resolved (fallback id " +
                    $"'{_trackId}'). The HUD will have no track info.");

            InitializeRaceMode();
        }

        private void InitializeRaceMode()
        {
            Debug.Log("[TrackModeManager] Initializing Race mode");

            int totalLaps = _raceLapsOverride > 0
                ? _raceLapsOverride
                : (_trackDefinition?.StandardRaceLaps ?? 10);

            var raceScreen = FindAnyObjectByType<RaceScreen>();
            if (raceScreen != null && _trackDefinition != null)
                raceScreen.SetRaceInfo(_trackDefinition, totalLaps, _flow.RaceGridPosition);

            if (_aiOpponentPrefab == null)
                return;

            int aiCount = Mathf.Min(_maxAIOpponents, 5);
            for (int i = 0; i < aiCount; i++)
            {
                var ai = Instantiate(_aiOpponentPrefab, Vector3.zero, Quaternion.identity);
                ai.name = $"AI_Opponent_{i + 1}";
            }
        }
    }
}
