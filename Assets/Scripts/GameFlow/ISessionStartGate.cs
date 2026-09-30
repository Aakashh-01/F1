namespace F1.GameFlow
{
    /// <summary>
    /// Implemented by a scene controller that places the player at the start line and can
    /// hold them there while a countdown plays.
    ///
    /// This exists because of an assembly boundary, not for design taste. The flow
    /// controllers live in F1.GameFlow and the UI lives in Assembly-CSharp, and that
    /// reference only runs in ONE direction — F1.GameFlow cannot see the UI, so a controller
    /// cannot simply reach the overlay and start a countdown. The overlay can see the flow,
    /// so the flow publishes a two-member contract and the overlay drives whoever implements
    /// it.
    ///
    /// The contract is deliberately about the CAR and nothing else. The clock lives in the
    /// overlay and nowhere else, because two clocks started independently would drift and
    /// release the car at a moment the player did not watch.
    /// </summary>
    public interface ISessionStartGate
    {
        /// <summary>True once the session is on track and a countdown should play.</summary>
        bool IsSessionReady { get; }

        /// <summary>
        /// Holds the car at the line, or releases it. The caller is expected to hold it
        /// across a countdown and release on zero.
        /// </summary>
        void SetStartGateLocked(bool locked);
    }
}
