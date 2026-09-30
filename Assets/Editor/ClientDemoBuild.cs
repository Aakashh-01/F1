// Assets/Editor/ClientDemoBuild.cs
//
// Windows client-demo build. Produces Builds/newbuild/F1.exe at the monitor's
// native resolution.
//
// This exists because the project had NO build entry point at all — every previous
// build was made by hand through the Editor GUI, so the player settings that decide
// the window size were whatever someone last clicked. That is how the demo shipped
// opening in a 1024x768 window on a 1920x1080 screen.
//
// TWO SEPARATE THINGS SET THE WINDOW SIZE, and fixing only the first looks like it
// worked right up until the second runs:
//
//   1. ProjectSettings.asset - the defaults below. These are what a fresh clone gets.
//   2. The registry, under HKCU\Software\<company>\<product>. Unity SAVES the last
//      window size, position and fullscreen mode here on exit, and on next launch the
//      saved values WIN over the project defaults. A stale entry here survives a
//      rebuild and re-shrinks the window every time. Diagnose it with:
//
//        reg query "HKCU\Software\DefaultCompany\F1"
//
//      Look for "Resolution Use Native" = 0 and a fixed width/height. This build
//      script does not touch the registry (that would be a surprising thing for a
//      build to do to someone's machine); ClearSavedWindowState() below is available
//      from the menu for when that stale state is the cause.
//
// The garage art and the SlimUI art are purchased Asset Store content and are
// gitignored, so they must already be imported or the lobby will be empty.
using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

public static class ClientDemoBuild
{
    private const string OutputFolder = "Builds/newbuild";
    private const string ExecutableName = "F1";

    [MenuItem("Tools/Client Demo/Build Windows Demo")]
    public static void BuildWindowsDemo()
    {
        ApplyNativeResolutionPlayerSettings();

        var scenes = EditorBuildSettings.scenes
            .Where(s => s.enabled)
            .Select(s => s.path)
            .ToArray();

        if (scenes.Length == 0)
        {
            Debug.LogError("[ClientDemoBuild] No enabled scenes in Build Settings. Nothing to build.");
            return;
        }

        var outputPath = Path.Combine(OutputFolder, ExecutableName + ".exe");
        Directory.CreateDirectory(OutputFolder);

        var options = new BuildPlayerOptions
        {
            scenes = scenes,
            locationPathName = outputPath,
            target = BuildTarget.StandaloneWindows64,
            targetGroup = BuildTargetGroup.Standalone,
            options = BuildOptions.None
        };

        Debug.Log($"[ClientDemoBuild] Building {scenes.Length} scenes to {outputPath}");

        var report = BuildPipeline.BuildPlayer(options);
        var summary = report.summary;

        if (summary.result == BuildResult.Succeeded)
        {
            Debug.Log($"[ClientDemoBuild] SUCCESS  {summary.totalSize / (1024 * 1024)} MB  " +
                      $"in {summary.totalTime.TotalSeconds:F1}s  ->  {outputPath}");
        }
        else
        {
            Debug.LogError($"[ClientDemoBuild] FAILED ({summary.result}) with " +
                           $"{summary.totalErrors} errors. See Editor.log for detail.");
        }
    }

    /// <summary>
    /// Pins the player to the monitor's own resolution, borderless.
    ///
    /// FullScreenWindow rather than ExclusiveFullScreen: borderless fills the screen
    /// but is a normal window, so it survives a resolution change or an Alt-Tab
    /// without dumping the client back to the desktop. Resizable is on so a client
    /// whose monitor is smaller than this machine's can still get a usable window.
    /// </summary>
    private static void ApplyNativeResolutionPlayerSettings()
    {
        PlayerSettings.defaultIsNativeResolution = true;
        PlayerSettings.fullScreenMode = FullScreenMode.FullScreenWindow;
        PlayerSettings.resizableWindow = true;
        PlayerSettings.defaultScreenWidth = 1920;
        PlayerSettings.defaultScreenHeight = 1080;
        PlayerSettings.forceSingleInstance = true;
        PlayerSettings.runInBackground = true;

        AssetDatabase.SaveAssets();
        Debug.Log("[ClientDemoBuild] Player settings set to native resolution, borderless fullscreen.");
    }

    /// <summary>
    /// Deletes the window size/position Unity persisted for this product, so the next
    /// launch falls back to the project defaults instead of a stale saved size.
    ///
    /// Only the geometry keys are removed. The refresh-rate, stereo and session-count
    /// values are left alone — they are not the problem and dropping them buys nothing.
    /// </summary>
    [MenuItem("Tools/Client Demo/Clear Saved Window State")]
    public static void ClearSavedWindowState()
    {
        var company = PlayerSettings.companyName;
        var product = PlayerSettings.productName;
        var keyPath = $@"Software\{company}\{product}";

        using (var key = Microsoft.Win32.Registry.CurrentUser.OpenSubKey(keyPath, true))
        {
            if (key == null)
            {
                Debug.Log($"[ClientDemoBuild] No saved window state at HKCU\\{keyPath} — nothing to clear.");
                return;
            }

            // The hashed suffixes are appended by Unity and are stable per key name.
            string[] prefixes =
            {
                "Screenmanager Resolution Width",
                "Screenmanager Resolution Height",
                "Screenmanager Resolution Use Native",
                "Screenmanager Resolution Window Width",
                "Screenmanager Resolution Window Height",
                "Screenmanager Fullscreen mode",
                "Screenmanager Window Position X",
                "Screenmanager Window Position Y",
                "UnitySelectMonitor"
            };

            var doomed = key.GetValueNames()
                .Where(n => prefixes.Any(p => n.StartsWith(p, StringComparison.Ordinal)))
                .ToArray();

            foreach (var name in doomed)
            {
                key.DeleteValue(name, false);
                Debug.Log($"[ClientDemoBuild] cleared {name}");
            }

            Debug.Log($"[ClientDemoBuild] Cleared {doomed.Length} saved window value(s) from HKCU\\{keyPath}.");
        }
    }
}
