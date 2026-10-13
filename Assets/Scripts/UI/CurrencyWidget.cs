using UnityEngine;
using TMPro;
using F1.Progression;

namespace F1.UI
{
    /// <summary>
    /// The dual-currency readout. Career points lead because unlocks are denominated in
    /// careerPoints; premium sits under it.
    /// </summary>
    [DisallowMultipleComponent]
    public class CurrencyWidget : MonoBehaviour
    {
        [SerializeField] private TMP_Text _careerPointsText;
        [SerializeField] private TMP_Text _premiumText;

        public void SetCareerPoints(int amount)
        {
            if (_careerPointsText != null)
                _careerPointsText.text = amount.ToString("N0");
        }

        public void Refresh()
        {
            var profile = PlayerProfileManager.Current;
            if (profile == null) return;

            SetCareerPoints(profile.careerPoints);
            if (_premiumText != null) _premiumText.text = profile.premiumCurrency.ToString("N0");
        }
    }
}
