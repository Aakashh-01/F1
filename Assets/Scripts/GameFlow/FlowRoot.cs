using UnityEngine;

namespace F1.GameFlow
{
    /// <summary>
    /// Marker for the persistent flow root. Lives on the same GameObject as
    /// <see cref="GameFlowManager"/> and survives every scene transition.
    ///
    /// This class deliberately lives in its own file. When it shared
    /// <c>FlowBootstrap.cs</c>, Unity serialized it into scenes as a bare
    /// <c>m_Script: {fileID: ...}</c> reference with no GUID, and both
    /// <c>00_LoadingScene</c> and <c>LobbyScene</c> ended up carrying a
    /// <em>missing script</em> in that component slot. A MonoBehaviour per file gets a
    /// stable <c>.cs.meta</c> GUID and serializes correctly.
    /// </summary>
    [DisallowMultipleComponent]
    public class FlowRoot : MonoBehaviour
    {
    }
}
