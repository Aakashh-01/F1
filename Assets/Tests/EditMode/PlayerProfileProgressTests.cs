using NUnit.Framework;
using UnityEngine;
using F1.Progression;

/// <summary>
/// The XP progress math the garage profile panel renders, and the old-save path for the
/// team-name field.
///
/// The formula is the one <see cref="PlayerProfile.CheckLevelUp"/> implies: level L spans
/// totalXP from (L-1) * XpPerLevel to L * XpPerLevel. A first draft used playerLevel as
/// the base instead of playerLevel - 1, which clamps every legitimate total to zero and
/// renders an empty XP bar forever — these boundary tests are the guard against that
/// exact regression.
/// </summary>
public class PlayerProfileProgressTests
{
    [Test]
    public void XpPerLevel_IsStillOneThousand()
    {
        // The const was extracted from an inline literal. If the curve ever changes,
        // change it here deliberately — the progress formula and CheckLevelUp share it.
        Assert.AreEqual(1000, PlayerProfile.XpPerLevel);
    }

    [Test]
    public void XpProgress_AtTheStartOfLevelOne_IsZero()
    {
        var profile = new PlayerProfile { playerLevel = 1, totalXP = 0 };
        Assert.AreEqual(0f, profile.XpProgressToNextLevel());
    }

    [Test]
    public void XpProgress_AtTheEndOfALevel_IsOne()
    {
        // Level 2 spans 1000..2000. Sitting exactly on the boundary is a full bar — and
        // also the point CheckLevelUp would have promoted past already, which is why the
        // clamp rather than a strict less-than matters here.
        var profile = new PlayerProfile { playerLevel = 2, totalXP = 2000 };
        Assert.AreEqual(1f, profile.XpProgressToNextLevel());
    }

    [Test]
    public void XpProgress_MidLevel_IsHalf()
    {
        var profile = new PlayerProfile { playerLevel = 2, totalXP = 1500 };
        Assert.AreEqual(0.5f, profile.XpProgressToNextLevel(), 1e-5f);
    }

    [Test]
    public void XpProgress_BelowTheLevelWindow_ClampsToZero()
    {
        // Total XP under the level's window happens when the level was raised by
        // something other than AddXP. An empty bar is correct; NaN or a negative fill
        // would corrupt Image.fillAmount.
        var profile = new PlayerProfile { playerLevel = 3, totalXP = 100 };
        Assert.AreEqual(0f, profile.XpProgressToNextLevel());
    }

    [Test]
    public void XpProgress_AboveTheLevelWindow_ClampsToOne()
    {
        var profile = new PlayerProfile { playerLevel = 1, totalXP = 5000 };
        Assert.AreEqual(1f, profile.XpProgressToNextLevel());
    }

    [Test]
    public void XpProgress_AgreesWithCheckLevelUp()
    {
        // Level up through the real path, then confirm the progress bar reads consistently
        // with where the level landed.
        var profile = new PlayerProfile();
        profile.AddXP(2500);
        Assert.AreEqual(3, profile.playerLevel, "2500 XP should reach level 3 on the cumulative curve.");
        Assert.AreEqual(0.5f, profile.XpProgressToNextLevel(), 1e-5f,
            "Level 3 spans 2000..3000, so 2500 XP is half-way.");
    }

    [Test]
    public void OldSave_WithoutTeamName_DefaultsTheTeam()
    {
        // JsonUtility fills missing fields from the field initializers, so a profile
        // saved before teamName existed loads with the default rather than null — and
        // the garage profile panel renders "Apex Racing" instead of an empty string.
        var profile = JsonUtility.FromJson<PlayerProfile>("{\"playerName\":\"Saved Player\"}");
        Assert.IsNotNull(profile);
        Assert.AreEqual("Saved Player", profile.playerName);
        Assert.AreEqual("Apex Racing", profile.teamName);
    }
}
