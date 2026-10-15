using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Guards the separation between what a car *is* and what a *driver* does.
///
/// <para>
/// For months every <see cref="F1.GameData.CarDefinition"/> asset pointed
/// <c>_basePhysicsProfile</c> at <c>F1_AI_Physics</c>, because the AI profile was the only one
/// that had been authored for a race. <c>CarDefinition.CreateProfileWithWing</c> deep-copies
/// that base and swaps only the aero, so every player car silently drove the AI's motor force,
/// throttle spool, brake numbers and assist level. Nothing about the configuration was invalid,
/// every test passed, and the cars drove — the symptom was only ever felt as "the controls are
/// different in race mode".
/// </para>
///
/// <para>
/// The existing gate, <c>GameFlowPhase2GateTests.Registry_EveryCarHasAPhysicsProfileAndBothWings</c>,
/// asserts the profile is non-null. That is why the mis-wiring survived: a wrong profile still
/// satisfies "has a profile". These tests assert *which* profile, and that no car definition
/// reaches into the AI profile directory at all.
/// </para>
///
/// <para>
/// A second, quieter trap lives in <c>CarDefinition._carPrefab</c>: all six assets pointed at
/// <c>AI_F1_Body.prefab</c>, which carries <c>UseExternalInput: 1</c> and the AI physics profile
/// already applied. Nothing read the field, so nothing broke — until the first person wired
/// "spawn the selected car" to it and handed the player a car no keyboard can drive.
/// </para>
/// </summary>
public class CarDefinitionIsolationTests
{
    private const string CarDefinitionDirectory = "Assets/Resources/GameData/Cars";
    private const string AiProfileDirectory = "Assets/Profiles/AI_Profiles/";
    private const string AiBodyPath = "Assets/Prefabs/AI_F1_Body.prefab";

    private static F1.GameData.CarDefinition[] LoadAllCarDefinitions()
    {
        return AssetDatabase.FindAssets("t:CarDefinition", new[] { CarDefinitionDirectory })
            .Select(AssetDatabase.GUIDToAssetPath)
            .Select(AssetDatabase.LoadAssetAtPath<F1.GameData.CarDefinition>)
            .Where(cd => cd != null)
            .ToArray();
    }

    /// <summary>
    /// Whether a serialized asset file names this GUID. Read defensively: the scan walks every
    /// prefab, scene and asset in the project, and one unreadable file must not fail the guard.
    /// </summary>
    private static bool MentionsGuid(string assetPath, string guid)
    {
        try
        {
            string full = System.IO.Path.GetFullPath(assetPath);
            return System.IO.File.Exists(full) && System.IO.File.ReadAllText(full).Contains(guid);
        }
        catch (System.IO.IOException)
        {
            return false;
        }
    }

    [Test]
    public void TheProjectHasCarDefinitionsToCheck()
    {
        Assert.IsNotEmpty(LoadAllCarDefinitions(),
            "No CarDefinition assets were found under " + CarDefinitionDirectory +
            ", so these tests are guarding nothing.");
    }

    [Test]
    public void NoCarDefinitionUsesAnAiPhysicsProfile()
    {
        var definitions = LoadAllCarDefinitions();

        foreach (var cd in definitions)
        {
            var profile = cd.BasePhysicsProfile;
            Assert.IsNotNull(profile, $"'{cd.name}' has no base physics profile.");

            string path = AssetDatabase.GetAssetPath(profile);
            Assert.IsFalse(path.StartsWith(AiProfileDirectory),
                $"'{cd.name}' points _basePhysicsProfile at the AI profile '{path}'. " +
                "CreateProfileWithWing copies every field of that profile into the player's car, " +
                "so the player inherits the AI's motor force, throttle spool and brake numbers. " +
                "A car definition describes the CAR; driver tuning belongs in the AI driver profile.");
        }
    }

    [Test]
    public void EveryCarDefinitionHasTheSameBaseProfile()
    {
        // The base profile is the car's physical limits, not a driver's. If two cars claiming to
        // be different generations resolve to different physics, one of them has been pointed at
        // a profile that describes something other than the car.
        var definitions = LoadAllCarDefinitions();
        var distinct = definitions
            .Select(cd => AssetDatabase.GetAssetPath(cd.BasePhysicsProfile))
            .Distinct()
            .ToArray();

        Assert.LessOrEqual(distinct.Length, 1,
            "Car definitions resolve to " + distinct.Length + " different base profiles (" +
            string.Join(", ", distinct) + "). Expected one.");
    }

    [Test]
    public void NoCarDefinitionPointsAtTheAiBody()
    {
        var definitions = LoadAllCarDefinitions();

        foreach (var cd in definitions)
        {
            var prefab = cd.CarPrefab;
            if (prefab == null)
            {
                // Nothing reads CarPrefab yet, so an unset field is inert. Only a *wrong*
                // reference is a trap, and an empty one cannot hand anyone the wrong car.
                continue;
            }

            string path = AssetDatabase.GetAssetPath(prefab);
            Assert.AreNotEqual(AiBodyPath, path,
                $"'{cd.name}' points CarPrefab at the AI body. That prefab ships with " +
                "UseExternalInput and the AI physics profile already applied, so a player " +
                "spawned from it cannot be driven with the keyboard.");
        }
    }

    [Test]
    public void TheAiBodyIsNotReachableFromAnyCarDefinitionPath()
    {
        // Belt and braces: resolve every CarDefinition reference the project stores and make
        // sure none of them lands on the AI body by any route.
        var aiBodyGuid = AssetDatabase.AssetPathToGUID(AiBodyPath);
        Assert.IsNotEmpty(aiBodyGuid, AiBodyPath + " has no GUID — was the prefab moved?");

        foreach (var cd in LoadAllCarDefinitions())
        {
            string guid = AssetDatabase.AssetPathToGUID(AssetDatabase.GetAssetPath(cd.CarPrefab));
            Assert.AreNotEqual(aiBodyGuid, guid,
                $"'{cd.name}' resolves CarPrefab to the AI body guid.");
        }
    }

    [Test]
    public void TheAiProfileIsOnlyReachableFromTheAiBody()
    {
        // The complementary half of the contract: the AI physics profile should not be drifting
        // into places a player car reads. Load it by path and find every asset that names its
        // GUID in its serialized text.
        const string aiProfilePath = "Assets/Profiles/AI_Profiles/F1_AI_Physics.asset";
        var aiProfile = AssetDatabase.LoadAssetAtPath<VehiclePhysicsProfile>(aiProfilePath);
        Assert.IsNotNull(aiProfile, aiProfilePath + " is missing.");

        string aiGuid = AssetDatabase.AssetPathToGUID(aiProfilePath);
        Assert.IsNotEmpty(aiGuid, aiProfilePath + " has no GUID.");

        string[] referencing = AssetDatabase.FindAssets("")
            .Select(AssetDatabase.GUIDToAssetPath)
            .Where(p => p != null)
            .Where(p => p.EndsWith(".prefab") || p.EndsWith(".unity") || p.EndsWith(".asset"))
            .Where(p => p != aiProfilePath)
            .Where(p => MentionsGuid(p, aiGuid))
            .ToArray();

        Assert.AreEqual(new[] { AiBodyPath }, referencing.OrderBy(p => p).ToArray(),
            "The AI physics profile is referenced by something other than the AI body: " +
            string.Join(", ", referencing) + ". A player car that resolves to it will drive " +
            "the AI's numbers.");
    }
}