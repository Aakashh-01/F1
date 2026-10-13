using UnityEngine;

namespace F1.GameFlow
{
    /// <summary>
    /// The menu hub's stage manager. Two hosted screens share one background scene — the
    /// car panel (GameScreen.Lobby) and the track panel (GameScreen.TrackSelection) — and
    /// this decides which is on stage from the flow's current screen, the same signal the
    /// flow itself just committed to. It also takes the showcase car off stage during
    /// track selection, which stops the turntable for free.
    ///
    /// Toggling the host ROOTS (not the screens) is deliberate: each screen's scene
    /// controller sits on its host root, so deactivating the root pauses its behaviour
    /// with it, and GameFlowManager.FindSceneHost matches registered hosts regardless of
    /// active state, so transitions in and out of the hub keep resolving both screens.
    ///
    /// With no flow (raw scene load by tests or art review) this does nothing and the
    /// scene's authored default — car panel on stage — holds.
    /// </summary>
    [DisallowMultipleComponent]
    public class GarageHubView : MonoBehaviour
    {
        private GameFlowManager _flow;
        private FlowSceneHost _carHost;
        private FlowSceneHost _trackHost;
        private GameObject _carSocket;

        private void Start()
        {
            _flow = GameFlowManager.Instance;
            if (_flow == null) return;

            foreach (var host in FindObjectsByType<FlowSceneHost>(
                         FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (host.HostedScreen == GameFlowManager.GameScreen.Lobby) _carHost = host;
                else if (host.HostedScreen == GameFlowManager.GameScreen.TrackSelection) _trackHost = host;
            }

            var swapper = FindAnyObjectByType<ShowcaseCarSwapper>();
            if (swapper != null) _carSocket = swapper.gameObject;

            _flow.OnScreenChanged += Apply;
            Apply(_flow.CurrentScreen);
        }

        private void OnDestroy()
        {
            if (_flow != null) _flow.OnScreenChanged -= Apply;
        }

        private void Apply(GameFlowManager.GameScreen screen)
        {
            SetActive(_carHost, screen == GameFlowManager.GameScreen.Lobby);
            SetActive(_trackHost, screen == GameFlowManager.GameScreen.TrackSelection);
            if (_carSocket != null)
                _carSocket.SetActive(screen == GameFlowManager.GameScreen.Lobby);
        }

        private static void SetActive(FlowSceneHost host, bool active)
        {
            if (host != null) host.gameObject.SetActive(active);
        }
    }
}
