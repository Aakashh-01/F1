using System.Collections;
using F1.GameFlow;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Bounded waiting for scene teardown, shared by the PlayMode fixtures that unload the flow
/// scene and the additively loaded track content between tests.
/// </summary>
/// <remarks>
/// <para>
/// The PlayMode suite wedged reproducibly. It stopped at 146/155 on
/// <c>SliceS4QualifyingShellTests.OnlyOnePlayerCarIsSpawned</c> — the *first* test in its
/// fixture, so the hang was in the shared teardown rather than in anything the test itself
/// did — and the job latched <c>tests_running</c>, after which the Unity MCP plugin refused
/// to refresh assets, so the project stopped importing files and stopped compiling.
/// </para>
///
/// <para>
/// The wait that could not finish was <c>yield return sceneFlow.UnloadAllContent()</c>.
/// That returns a <see cref="Coroutine"/>, and the two obvious defences do not work:
///
/// <list type="bullet">
/// <item>There is no <c>Coroutine.isDone</c>. Unity exposes no way to ask whether someone
/// else's coroutine has finished, so the only thing you can do is yield it and hope.</item>
/// <item>Wrapping it in a deadline does not work either, because a coroutine that has not
/// finished is not something you can abandon — you cannot stop waiting on it from outside
/// without giving up the wait entirely.</item>
/// </list>
///
/// <para>
/// So the teardown does not yield the coroutine at all. It starts the unload and then polls
/// <see cref="SceneFlowService.IsBusy"/>, which is public and is cleared in a
/// <c>finally</c> by the service itself. That is a real completion signal, and unlike
/// yielding a coroutine it can be given a deadline.
/// </para>
///
/// <para>
/// Nothing here asserts. Teardown that throws replaces a stuck run with a different failure
/// mode and buries the test that was actually running, and the next fixture's setup already
/// asserts a clean flow — so a genuinely dirty teardown is still caught, one test later and
/// with a name attached.
/// </para>
/// </remarks>
public static class SceneOpWait
{
    /// <summary>
    /// Ceiling for a teardown step. Generous on purpose: this is a backstop against a
    /// deadlock, not a performance budget, and a slow-but-correct unload must not be turned
    /// into a failure.
    /// </summary>
    public const float DefaultTimeoutSeconds = 30f;

    /// <summary>
    /// Starts unloading every additively loaded content scene and waits for the service to
    /// go idle, giving up after <paramref name="timeout"/> seconds.
    /// </summary>
    public static IEnumerator UnloadAllContentBounded(SceneFlowService flow, float timeout = DefaultTimeoutSeconds)
    {
        if (flow == null) yield break;

        if (flow.ContentScenes.Count == 0) yield break;

        // Started, deliberately not yielded. See the remarks on why.
        flow.UnloadAllContent();

        float deadline = Time.realtimeSinceStartup + timeout;
        while (flow.IsBusy && Time.realtimeSinceStartup < deadline)
            yield return null;
    }

    /// <summary>
    /// Unloads a scene and waits for the operation to finish, giving up after
    /// <paramref name="timeout"/> seconds.
    ///
    /// <see cref="AsyncOperation"/> does expose <c>isDone</c>, so unlike a coroutine it can
    /// be waited on with a deadline. Yielding it directly would also work, but only in the
    /// sense that "works right up until it doesn't" — which is the failure being fixed here.
    /// </summary>
    public static IEnumerator UnloadSceneBounded(Scene scene, float timeout = DefaultTimeoutSeconds)
    {
        if (!scene.IsValid() || !scene.isLoaded) yield break;

        var op = SceneManager.UnloadSceneAsync(scene);
        if (op == null) yield break;

        float deadline = Time.realtimeSinceStartup + timeout;
        while (!op.isDone && Time.realtimeSinceStartup < deadline)
            yield return null;
    }
}
