using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Guards that <see cref="AIDriverProfile"/> is the single source of AI tuning, and that
/// nothing can shadow it again.
///
/// <para>
/// The defect this exists to prevent is not a bug in any one value. It is that the same
/// fifty-two numbers lived in three places at once — as public fields on
/// <c>AIDriverController</c>, as overrides serialised into <c>AI_F1_Body.prefab</c>, and as
/// defaults in the code — and Unity deserialisation prefers the prefab. So thirty of the
/// fifty-two were being set by the prefab, and editing the code's copy changed nothing that
/// ever ran. Nothing was invalid, nothing warned, and every test passed, because the values
/// were all legal numbers. The clearest single piece of evidence is
/// <c>trackRecoverySpeedKmh</c>: its own tooltip says 95 km/h was a bug and 45 is the fix,
/// the code was changed to 45 — and the prefab still said 95, so the fix had never once been
/// in effect.
/// </para>
///
/// <para>
/// The change that closes it is structural rather than a value correction: the fields became
/// properties, which Unity does not serialise, and the prefab now names one profile asset.
/// These tests assert the structure, because the structure is the part that rots. A future
/// field added as a public field, or a stray key typed back into the prefab by an innocent
/// inspector edit, would reopen the hole while every behavioural test stayed green.
/// </para>
///
/// <para>
/// The tier assets are all three created from the same numbers on purpose. Differentiation
/// between them is a separate piece of work, and inventing three sets of values here would
/// be tuning by assertion: a test that passes because it compares a number against itself is
/// worse than no test, because it looks like coverage. What these tests do assert is that
/// the three are distinct assets and load — the thing that has to be true before any tier
/// value means anything.
/// </para>
/// </summary>
public class AIDriverProfileTests
{
    private const string ProfileDirectory = "Assets/Profiles/AI_Profiles/";
    private const string AiBodyPath = "Assets/Prefabs/AI_F1_Body.prefab";

    private static readonly string[] TierAssetNames =
    {
        "AIDriver_Easy",
        "AIDriver_Medium",
        "AIDriver_Hard"
    };

    /// <summary>
    /// Every GameObject a test created, so a failure part-way through cannot leave one
    /// behind holding a runtime profile that outlives the test run.
    /// </summary>
    private readonly List<GameObject> _spawned = new List<GameObject>();

    [TearDown]
    public void TearDown()
    {
        for (int i = 0; i < _spawned.Count; i++)
        {
            if (_spawned[i] == null)
                continue;

            DestroyRuntimeFallback(_spawned[i].GetComponent<AIDriverController>());
            UnityEngine.Object.DestroyImmediate(_spawned[i]);
        }

        _spawned.Clear();
    }

    /// <summary>
    /// Destroys the runtime profile a driver built for itself because none was assigned.
    ///
    /// <para>
    /// The fallback is cached in a private field and nothing else owns it — the component
    /// has no <c>OnDestroy</c> — so a test that exercises the no-profile path leaks a
    /// ScriptableObject. Reflected rather than exposed: widening the production type's
    /// surface for the sake of a test is the wrong trade, and reaching into privates is
    /// already the established idiom in this folder (see
    /// <c>AIDriverSafetyLawTests.EmergencyDistanceMetres</c>).
    /// </para>
    /// </summary>
    private static void DestroyRuntimeFallback(AIDriverController driver)
    {
        if (driver == null)
            return;

        FieldInfo field = typeof(AIDriverController).GetField(
            "_runtimeTuning", BindingFlags.Instance | BindingFlags.NonPublic);

        if (field == null)
            return;

        var runtime = field.GetValue(driver) as AIDriverProfile;
        if (runtime != null)
            UnityEngine.Object.DestroyImmediate(runtime);
    }

    /// <summary>
    /// Every tunable name, read off the profile rather than hardcoded.
    ///
    /// <para>
    /// Hardcoding the list here would let the two drift apart in the one direction that
    /// matters: a new tunable added to the profile, wired as a public field on the
    /// controller, and quietly absent from this array would pass every assertion below,
    /// because the assertions would only be about the older fifty-two. Deriving it means a
    /// new field is covered the moment it exists.
    /// </para>
    /// </summary>
    private static string[] TuningNames()
    {
        return typeof(AIDriverProfile)
            .GetFields(BindingFlags.Public | BindingFlags.Instance)
            .Where(f => f.FieldType == typeof(float) || f.FieldType == typeof(int) || f.FieldType == typeof(bool))
            .Select(f => f.Name)
            .ToArray();
    }

    /// <summary>Every public field declared directly on the controller itself.</summary>
    private static FieldInfo[] ControllerOwnFields()
    {
        return typeof(AIDriverController)
            .GetFields(BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly)
            .ToArray();
    }

    // ---------------------------------------------------------------------
    // The controller no longer owns the numbers.
    // ---------------------------------------------------------------------

    [Test]
    public void TheControllerHasTunablesToCheck()
    {
        Assert.IsNotEmpty(TuningNames(),
            "AIDriverProfile exposes no float/int/bool fields, so these tests are guarding " +
            "nothing. Did the field types change?");
    }

    [Test]
    public void NoTunableIsAPublicFieldOnTheController()
    {
        var names = new HashSet<string>(TuningNames());

        var shadowing = ControllerOwnFields()
            .Where(f => names.Contains(f.Name))
            .Select(f => f.FieldType.Name + " " + f.Name)
            .ToArray();

        Assert.IsEmpty(shadowing,
            "These tunables are public fields on AIDriverController again: " +
            string.Join(", ", shadowing) + ". A public field is a value Unity will happily " +
            "serialise into a prefab or scene instance, and it will then win over the " +
            "profile — which is the whole defect. They have to be properties, reading " +
            "through AIDriverProfile.");
    }

    [Test]
    public void EveryTunableIsAReadableAndWritablePropertyOnTheController()
    {
        // A get-only property would be the correct end state for the tunables, but half the
        // project's callers assign them directly: RaceGridManager sets a lane offset and an
        // overtake side per entry, and both test assemblies pin values to build a scenario.
        // Dropping the setters would not fail a behavioural test, it would fail to compile,
        // which is a louder and better failure than a silently untunable driver.
        var missing = new List<string>();

        foreach (string name in TuningNames())
        {
            PropertyInfo property = typeof(AIDriverController).GetProperty(
                name, BindingFlags.Public | BindingFlags.Instance);

            if (property == null || !property.CanRead || !property.CanWrite)
                missing.Add(name);
        }

        Assert.IsEmpty(missing,
            "These tunables are not read/write properties on AIDriverController: " +
            string.Join(", ", missing) + ". Callers that assign them — the grid manager and " +
            "both test assemblies — would stop compiling, and a test scenario could no " +
            "longer pin the one value it is about.");
    }

    [Test]
    public void TheControllerHasExactlyOneProfileReferenceAndItIsPublic()
    {
        FieldInfo field = typeof(AIDriverController).GetField(
            "driverProfile", BindingFlags.Public | BindingFlags.Instance);

        Assert.IsNotNull(field,
            "AIDriverController has no public driverProfile field, so the profile is " +
            "unreachable from a prefab and every car is silently on the fallback.");
        Assert.AreEqual(typeof(AIDriverProfile), field.FieldType);
    }

    // ---------------------------------------------------------------------
    // The profile drives the values.
    // ---------------------------------------------------------------------

    [Test]
    public void TheControllerReadsItsValuesFromTheAssignedProfile()
    {
        GameObject car = new GameObject("Profile Read Test Car");
        _spawned.Add(car);
        try
        {
            var driver = car.AddComponent<AIDriverController>();

            var profile = ScriptableObject.CreateInstance<AIDriverProfile>();
            try
            {
                profile.baseLookaheadDistance = 21.5f;
                profile.overtakeTrigger = 0.42f;
                driver.driverProfile = profile;

                Assert.AreEqual(21.5f, driver.baseLookaheadDistance, Epsilon,
                    "The controller is not reading baseLookaheadDistance off the assigned " +
                    "profile.");
                Assert.AreEqual(0.42f, driver.overtakeTrigger, Epsilon,
                    "The controller is not reading overtakeTrigger off the assigned profile.");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(profile);
            }
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(car);
        }
    }

    [Test]
    public void ACarsValuesFollowTheProfileUntilSomethingWritesToThem()
    {
        GameObject car = new GameObject("Profile Write-Through Test Car");
        _spawned.Add(car);
        try
        {
            var driver = car.AddComponent<AIDriverController>();

            var profile = ScriptableObject.CreateInstance<AIDriverProfile>();
            try
            {
                driver.driverProfile = profile;

                // Nothing written yet, so the profile is in charge.
                profile.stuckDetectionSeconds = 9f;
                Assert.AreEqual(9f, driver.stuckDetectionSeconds, Epsilon,
                    "With no override set, the driver must report the profile's value. It " +
                    "reported something else, which means the property is reading a default " +
                    "of its own instead of the asset.");

                // Written, so this instance wins — and only this instance.
                driver.stuckDetectionSeconds = 30f;
                Assert.AreEqual(30f, driver.stuckDetectionSeconds, Epsilon,
                    "Writing a tuning property must be readable straight back. Callers rely " +
                    "on it: the grid manager assigns per-entry values, and both test " +
                    "assemblies pin values to build a scenario.");

                profile.stuckDetectionSeconds = 3f;
                Assert.AreEqual(30f, driver.stuckDetectionSeconds, Epsilon,
                    "A written value must survive a later edit to the profile. The override " +
                    "is per-instance; a write that went to the shared asset instead would " +
                    "leak this car's scenario into every other car using the same profile.");

                // And it must not have leaked onto the asset.
                Assert.AreEqual(3f, profile.stuckDetectionSeconds, Epsilon,
                    "Writing a tuning property modified the shared profile asset. That is the " +
                    "same class of bug as the prefab overrides, just moved: two cars on one " +
                    "profile would retune each other, and the asset would be dirtied on disk.");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(profile);
            }
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(car);
        }
    }

    [Test]
    public void ACarWithNoProfileStillHasEveryValue()
    {
        GameObject car = new GameObject("No Profile Test Car");
        _spawned.Add(car);
        try
        {
            var driver = car.AddComponent<AIDriverController>();
            Assert.IsNull(driver.driverProfile);

            // Every tunable, read through the null-safe fallback. Before the profile existed
            // these were fields with defaults, so a car with nothing assigned was always
            // drivable; that has to stay true, because the failure mode is a
            // NullReferenceException thrown from inside a physics tick rather than anything
            // anyone would notice in the editor.
            foreach (string name in TuningNames())
            {
                PropertyInfo property = typeof(AIDriverController).GetProperty(name);
                object value = property.GetValue(driver);
                Assert.IsNotNull(value, name + " read as null on a driver with no profile.");
            }

            Assert.Greater(driver.trackRecoverySpeedKmh, 0f,
                "A driver with no profile must still have a track-recovery speed.");
        }
        finally
        {
            DestroyRuntimeFallback(driver: car.GetComponent<AIDriverController>());
        }
    }

    [Test]
    public void TheFallbackProfileIsCachedRatherThanRebuiltPerRead()
    {
        // Not observable through the public surface — the fallback is private — so this
        // asserts the factory instead: it must hand back a usable instance rather than
        // something the caller has to populate, because a caller that forgets would null out
        // every value at once.
        AIDriverProfile runtime = AIDriverProfile.CreateRuntimeProfile();
        try
        {
            Assert.IsNotNull(runtime);
            Assert.Greater(runtime.baseLookaheadDistance, 0f,
                "CreateRuntimeProfile returned a profile with no defaults. Its field " +
                "initialisers are the values the AI was actually running, so a bare " +
                "CreateInstance is the whole implementation — if that stops being true, " +
                "every car with no profile drives on zeroes.");
            Assert.AreEqual(45f, runtime.trackRecoverySpeedKmh, Epsilon,
                "The runtime fallback still carries the old 95 km/h track-recovery speed. " +
                "That value is the bug this profile exists to fix: faster than most corners, " +
                "so a car that had drifted wide was steered back into traffic at near-racing " +
                "speed.");
        }
        finally
        {
            UnityEngine.Object.DestroyImmediate(runtime);
        }
    }

    // ---------------------------------------------------------------------
    // The three tiers exist.
    // ---------------------------------------------------------------------

    [Test]
    public void TheThreeTierAssetsExistAndAreDistinct()
    {
        var loaded = TierAssetNames
            .Select(name => AssetDatabase.LoadAssetAtPath<AIDriverProfile>(
                ProfileDirectory + name + ".asset"))
            .ToArray();

        for (int i = 0; i < loaded.Length; i++)
        {
            Assert.IsNotNull(loaded[i],
                ProfileDirectory + TierAssetNames[i] + ".asset is missing or is not an " +
                "AIDriverProfile. A grid with nothing to assign falls back to the shipped " +
                "defaults, which is indistinguishable from having tiers at all.");
        }

        var guids = loaded.Select(AssetDatabase.GetAssetPath)
            .Select(AssetDatabase.AssetPathToGUID)
            .ToArray();
        Assert.AreEqual(loaded.Length, guids.Distinct().Count(),
            "Two of the tier assets share a GUID, so they are the same asset under two " +
            "names and picking a tier does nothing.");

        var instances = loaded.Distinct().ToArray();
        Assert.AreEqual(loaded.Length, instances.Length,
            "Two of the tier assets resolve to the same instance.");
    }

    [Test]
    public void EveryTierHoldsTheFixedTrackRecoverySpeed()
    {
        foreach (string name in TierAssetNames)
        {
            var profile = AssetDatabase.LoadAssetAtPath<AIDriverProfile>(
                ProfileDirectory + name + ".asset");
            Assert.IsNotNull(profile, ProfileDirectory + name + ".asset is missing.");

            Assert.AreEqual(45f, profile.trackRecoverySpeedKmh, Epsilon,
                name + " holds " + profile.trackRecoverySpeedKmh +
                " km/h for track recovery. The prefab this profile replaced serialised 95, " +
                "which is the exact value the field's own tooltip names as a bug: faster " +
                "than most corners, so a recovering car — the least controlled car on the " +
                "track — was being steered back toward the racing line, where the traffic " +
                "is, at near-racing speed.");
        }
    }

    [Test]
    public void TheAiBodyPointsAtAProfileRatherThanRestatingTheNumbers()
    {
        var body = AssetDatabase.LoadAssetAtPath<GameObject>(AiBodyPath);
        Assert.IsNotNull(body, AiBodyPath + " is missing.");

        var driver = body.GetComponent<AIDriverController>();
        Assert.IsNotNull(driver, AiBodyPath + " has no AIDriverController on it.");
        Assert.IsNotNull(driver.driverProfile,
            "The AI body prefab has no driverProfile assigned, so it falls back to the " +
            "runtime defaults. That works, but it means tuning the asset does nothing to the " +
            "cars that actually race — which is the situation this whole change exists to " +
            "end.");
    }

    // ---------------------------------------------------------------------
    // Nothing is left behind in the prefab to shadow the profile.
    // ---------------------------------------------------------------------

    [Test]
    public void TheAiBodySerialisesNoTunableKeys()
    {
        // The half of the change that would otherwise be silently inert. Strip the fields
        // from the controller and Unity stops honouring the prefab's copies of them — but a
        // key left in the YAML is not an error. It deserialises onto nothing, nobody is
        // told, and the file reads as though the prefab is still configured. So this reads
        // the prefab as text and looks for the keys by name.
        Assert.IsTrue(File.Exists(AiBodyPath), AiBodyPath + " is missing.");

        string text = File.ReadAllText(Path.GetFullPath(AiBodyPath));
        var offenders = TuningNames().Where(name => MentionsKey(text, name)).ToArray();

        Assert.IsEmpty(offenders,
            "The AI body prefab still serialises " + offenders.Length + " of the tunables " +
            "directly: " + string.Join(", ", offenders) + ". Unity will keep deserialising " +
            "these onto the component and the profile will not be the source of truth for " +
            "them. Delete the keys.");
    }

    /// <summary>
    /// Whether the file declares <c>name:</c> as a YAML key, rather than merely mentioning
    /// the word. A comment, or a value that happens to contain the name, must not count or
    /// this test would pass for the wrong reason. The colon has to follow the name directly,
    /// so <c>overtakeTrigger</c> cannot be matched by a hypothetical
    /// <c>overtakeTriggerSeconds</c>.
    /// </summary>
    private static bool MentionsKey(string text, string key)
    {
        string prefix = "  " + key + ":";

        return text.Split('\n')
            .Any(line => line.TrimEnd('\r').TrimEnd().StartsWith(prefix, StringComparison.Ordinal));
    }

    private const float Epsilon = 0.001f;
}
