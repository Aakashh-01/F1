using System.IO;
using NUnit.Framework;
using UnityEngine;
using F1.Progression;

/// <summary>
/// Redirects <see cref="PlayerProfileManager"/> at a throwaway file for the duration of a
/// test, and puts the real profile back afterwards.
///
/// Why this is needed
/// ------------------
/// <c>PlayerProfileManager</c> is a plain static holding a lazily-loaded static instance.
/// It is not per-session and not per-test, so every PlayMode fixture in this assembly that
/// touches <c>PlayerProfileManager.Current</c> is touching the same object — and the same
/// file on disk — as the shipping game.
///
/// Two things then write through to the player's real save. <c>SetQualifyingTime</c> saves a
/// best lap whenever the fixture's time beats the stored one, and
/// <c>GameFlowManager</c> calls <c>AutoSaveIfDirty</c> on application pause and focus, so a
/// profile the fixture merely dirtied (by granting starter content) gets flushed on the way
/// out too.
///
/// The result was a saved best lap of 17.999 s and a ghost blob on track_monaco, produced by
/// a test fixture rather than by anyone driving. The qualifying screen then reported that
/// lap for real sessions, the lap tracker looked broken, and the value beat every AI
/// benchmark so the player started on pole. This is the part that is genuinely hard to
/// undo: a poisoned save survives every code fix, because rebuilding the project does not
/// rebuild the file.
///
/// The fix is the seam, not a cleanup. Clearing the JSON by hand treats the symptom and the
/// next test run puts it back; pointing the tests at a temp file means the real save is
/// never opened.
///
/// Usage: call <see cref="Begin"/> from a <c>[UnitySetUp]</c> and <see cref="End"/> from the
/// matching <c>[UnityTearDown]</c>. Both are safe to call more than once.
/// </summary>
public static class ProfileIsolation
{
    private static string _tempPath;
    private static bool _active;

    /// <summary>
    /// Points the manager at an empty temp profile and makes it the loaded one.
    ///
    /// <see cref="PlayerProfileManager.Load"/> is called rather than just assigning the
    /// override, because <c>Current</c> caches the loaded instance: without an explicit load
    /// the fixture would keep writing to the already-cached real profile in memory even
    /// though the file it saves to had changed.
    /// </summary>
    public static void Begin()
    {
        if (_active) return;

        _tempPath = Path.Combine(
            Application.temporaryCachePath,
            $"f1_test_profile_{System.Guid.NewGuid():N}.json");

        PlayerProfileManager.FilePathOverride = _tempPath;

        // The temp file deliberately does not exist yet. Load's missing-file branch creates
        // a fresh profile and writes it, which is the isolation we want — and the fresh file
        // means one test cannot inherit the previous test's granted content.
        PlayerProfileManager.Load();
        _active = true;
    }

    /// <summary>
    /// Restores the real profile and removes the temp file.
    ///
    /// The restore is a load, not a clear: the manager holds no way to drop its cached
    /// instance, and after this the cached object is the real profile again — so a fixture
    /// that runs after this one, or the editor picking play mode up afterwards, reads the
    /// player's own save rather than a test's.
    /// </summary>
    public static void End()
    {
        if (!_active) return;

        PlayerProfileManager.FilePathOverride = null;
        PlayerProfileManager.Load();

        if (!string.IsNullOrEmpty(_tempPath) && File.Exists(_tempPath))
        {
            try { File.Delete(_tempPath); }
            catch (IOException) { /* a locked temp file is not worth failing a teardown over */ }
        }

        _tempPath = null;
        _active = false;
    }
}
