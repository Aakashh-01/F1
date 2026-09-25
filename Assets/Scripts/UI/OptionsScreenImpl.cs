using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.GameData;
using F1.GameFlow;
using F1.Progression;

namespace F1.UI
{
    /// <summary>
    /// Concrete OptionsScreen — settings controls, audio, video, back.
    /// </summary>
    public class OptionsScreenImpl : OptionsScreen
    {
        [Header("Title")]
        [SerializeField] private TMP_Text _titleText;

        [Header("Audio Settings")]
        [SerializeField] private TMP_Text _audioLabel;
        [SerializeField] private Slider _musicVolumeSlider;
        [SerializeField] private Slider _sfxVolumeSlider;
        [SerializeField] private TMP_Text _musicValueText;
        [SerializeField] private TMP_Text _sfxValueText;

        [Header("Video Settings")]
        [SerializeField] private TMP_Text _videoLabel;
        [SerializeField] private TMP_Dropdown _resolutionDropdown;
        [SerializeField] private TMP_Dropdown _qualityDropdown;

        [Header("Controls")]
        [SerializeField] private TMP_Text _controlsLabel;
        [SerializeField] private Button _resetControlsButton;

        [Header("Buttons")]
        [SerializeField] private Button _backButton;

        private GameFlowManager _flow;

        public override void Initialize(GameFlowManager flowManager)
        {
            _flow = flowManager;
        }

        public override void LoadSettings(PlayerProfile profile)
        {
            if (_titleText != null)
                _titleText.text = "Options";

            // Audio
            if (_musicVolumeSlider != null)
            {
                float musicVol = PlayerPrefs.GetFloat("MusicVolume", 0.7f);
                _musicVolumeSlider.value = musicVol;
                if (_musicValueText != null) _musicValueText.text = Mathf.RoundToInt(musicVol * 100f).ToString();
            }
            if (_sfxVolumeSlider != null)
            {
                float sfxVol = PlayerPrefs.GetFloat("SFXVolume", 0.8f);
                _sfxVolumeSlider.value = sfxVol;
                if (_sfxValueText != null) _sfxValueText.text = Mathf.RoundToInt(sfxVol * 100f).ToString();
            }

            // Video - resolutions
            if (_resolutionDropdown != null)
            {
                _resolutionDropdown.ClearOptions();
                // Basic resolution options — extend based on Display.resolutions
                var options = new System.Collections.Generic.List<string>
                {
                    "1920 x 1080",
                    "1280 x 720",
                    "1024 x 768"
                };
                _resolutionDropdown.AddOptions(options);
                _resolutionDropdown.value = 0;
            }

            // Video - quality
            if (_qualityDropdown != null)
            {
                _qualityDropdown.ClearOptions();
                var qualityOptions = new System.Collections.Generic.List<string>();
                for (int i = 0; i < QualitySettings.count; i++)
                    qualityOptions.Add(QualitySettings.names[i]);
                _qualityDropdown.AddOptions(qualityOptions);
                _qualityDropdown.value = QualitySettings.GetQualityLevel();
            }
        }

        public override void Show()
        {
            gameObject.SetActive(true);
        }

        public override void Hide()
        {
            gameObject.SetActive(false);
        }

        public void OnMusicVolumeChanged(float value)
        {
            if (_musicValueText != null)
                _musicValueText.text = Mathf.RoundToInt(value * 100f).ToString();
            PlayerPrefs.SetFloat("MusicVolume", value);
        }

        public void OnSfxVolumeChanged(float value)
        {
            if (_sfxValueText != null)
                _sfxValueText.text = Mathf.RoundToInt(value * 100f).ToString();
            PlayerPrefs.SetFloat("SFXVolume", value);
        }

        public void OnResolutionChanged(int index)
        {
            // Placeholder — tie to actual Display.resolutions later
            PlayerPrefs.SetInt("SelectedResolution", index);
        }

        public void OnQualityChanged(int index)
        {
            QualitySettings.SetQualityLevel(index, true);
            PlayerPrefs.SetInt("SelectedQuality", index);
        }

        public void OnResetControlsClicked()
        {
            PlayerPrefs.DeleteKey("KeyHorizontal");
            PlayerPrefs.DeleteKey("KeyVertical");
            PlayerPrefs.DeleteKey("KeyBoost");
            PlayerPrefs.DeleteKey("KeyBrake");
            Debug.Log("[OptionsScreen] Controls reset to defaults");
        }

        public void OnBackClicked()
        {
            TriggerBackPressed();
        }
    }
}
