#if UNITY_EDITOR
using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Enters Play Mode and exits the Editor only for an ERC campaign invocation.
/// A fresh Editor process per trial guarantees that serialized scene state is reset.
/// </summary>
[InitializeOnLoad]
public static class ErcCampaignEditorAutomation
{
    private const string AutoPlayArgument = "-ercCampaignAutoPlay";
    private const string SceneArgument = "-ercCampaignScene";
    private const string ShutdownFileArgument = "-ercCampaignShutdownFile";
    private const string AutoPlayConsumedKey = "ERC.Campaign.AutoPlayConsumed";

    private static readonly string[] Arguments = Environment.GetCommandLineArgs();
    private static readonly string ScenePath = ReadArgumentValue(SceneArgument);
    private static readonly string ShutdownFile = ReadArgumentValue(ShutdownFileArgument);
    private static bool autoPlayPending;
    private static bool shutdownLogged;

    static ErcCampaignEditorAutomation()
    {
        bool autoPlayRequested = Arguments.Contains(AutoPlayArgument);
        autoPlayPending =
            autoPlayRequested && !SessionState.GetBool(AutoPlayConsumedKey, false);

        if (!string.IsNullOrEmpty(ShutdownFile) || autoPlayPending)
        {
            EditorApplication.update -= Update;
            EditorApplication.update += Update;
        }

        if (autoPlayRequested)
        {
            Debug.Log(
                "[ERC campaign] Editor automation enabled. " +
                $"scene='{ScenePath}', shutdown_file='{ShutdownFile}'");
        }
    }

    private static string ReadArgumentValue(string name)
    {
        for (int index = 0; index + 1 < Arguments.Length; ++index)
        {
            if (Arguments[index] == name)
            {
                return Arguments[index + 1];
            }
        }
        return string.Empty;
    }

    private static void Update()
    {
        if (ShutdownWasRequested())
        {
            return;
        }

        if (!autoPlayPending)
        {
            return;
        }
        if (EditorApplication.isCompiling || EditorApplication.isUpdating)
        {
            return;
        }
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            autoPlayPending = false;
            return;
        }

        if (!OpenConfiguredScene())
        {
            autoPlayPending = false;
            EditorApplication.update -= Update;
            EditorApplication.Exit(2);
            return;
        }

        autoPlayPending = false;
        SessionState.SetBool(AutoPlayConsumedKey, true);
        Debug.Log("[ERC campaign] Entering Play Mode.");
        EditorApplication.isPlaying = true;
    }

    private static bool OpenConfiguredScene()
    {
        if (string.IsNullOrEmpty(ScenePath))
        {
            Debug.LogError(
                "[ERC campaign] Missing -ercCampaignScene argument.");
            return false;
        }

        string absoluteScenePath = Path.Combine(
            Directory.GetParent(Application.dataPath).FullName,
            ScenePath);
        if (!File.Exists(absoluteScenePath))
        {
            Debug.LogError(
                $"[ERC campaign] Scene does not exist: {absoluteScenePath}");
            return false;
        }

        var activeScene = EditorSceneManager.GetActiveScene();
        if (activeScene.path == ScenePath)
        {
            return true;
        }
        if (activeScene.isDirty)
        {
            Debug.LogError(
                "[ERC campaign] Refusing to discard an unsaved scene.");
            return false;
        }

        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        Debug.Log($"[ERC campaign] Opened scene: {ScenePath}");
        return true;
    }

    private static bool ShutdownWasRequested()
    {
        if (string.IsNullOrEmpty(ShutdownFile) || !File.Exists(ShutdownFile))
        {
            return false;
        }

        autoPlayPending = false;
        if (!shutdownLogged)
        {
            shutdownLogged = true;
            Debug.Log(
                "[ERC campaign] Shutdown requested; leaving Play Mode cleanly.");
        }

        if (EditorApplication.isPlaying)
        {
            EditorApplication.isPlaying = false;
            return true;
        }
        if (EditorApplication.isPlayingOrWillChangePlaymode ||
            EditorApplication.isCompiling)
        {
            return true;
        }

        Debug.Log("[ERC campaign] Play Mode stopped; exiting Editor.");
        EditorApplication.update -= Update;
        EditorApplication.Exit(0);
        return true;
    }
}
#endif
