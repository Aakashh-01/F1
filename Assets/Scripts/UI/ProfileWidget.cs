using UnityEngine;
using UnityEngine.UI;
using TMPro;
using F1.Progression;

namespace F1.UI
{
    /// <summary>
    /// The player identity readout — name, team, level, XP bar. Shared hub chrome: both
    /// panels render it identically, so it lives here rather than in either impl, and
    /// swapping a panel prefab never orphans the profile display.
    /// </summary>
    [DisallowMultipleComponent]
    public class ProfileWidget : MonoBehaviour
    {
        [SerializeField] private TMP_Text _playerNameText;
        [SerializeField] private TMP_Text _teamText;
        [SerializeField] private TMP_Text _levelText;
        [SerializeField] private Image _xpFill;

        public void Refresh()
        {
            var profile = PlayerProfileManager.Current;
            if (profile == null) return;

            if (_playerNameText != null) _playerNameText.text = profile.playerName;
            if (_teamText != null) _teamText.text = profile.teamName;
            if (_levelText != null) _levelText.text = $"LEVEL {profile.playerLevel}";
            if (_xpFill != null) _xpFill.fillAmount = profile.XpProgressToNextLevel();
        }
    }
}
