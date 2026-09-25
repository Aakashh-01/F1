using UnityEngine;
using UnityEngine.UI;
using F1.GameFlow;

namespace F1.UI
{
    /// <summary>
    /// Concrete MainMenuScreen — Start, Options, Quit buttons.
    /// </summary>
    public class MainMenuScreenImpl : MainMenuScreen
    {
        [Header("UI")]
        [SerializeField] private Button _startButton;
        [SerializeField] private Button _optionsButton;
        [SerializeField] private Button _quitButton;

        private GameFlowManager _flow;

        public override void Initialize(GameFlowManager flowManager)
        {
            _flow = flowManager;
        }

        public override void Show()
        {
            gameObject.SetActive(true);
        }

        public override void Hide()
        {
            gameObject.SetActive(false);
        }

        public void OnStartClicked()
        {
            TriggerStartPressed();
        }

        public void OnOptionsClicked()
        {
            TriggerOptionsPressed();
        }

        public void OnQuitClicked()
        {
            TriggerQuitPressed();
        }
    }
}
