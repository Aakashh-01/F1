using System;
using UnityEngine;

namespace F1.GameFlow
{
    /// <summary>
    /// Works out which grid slot a qualifying result earns.
    ///
    /// A pure function, deliberately. The rule is session *policy* — what a lap time is worth
    /// in grid places — and keeping it out of <see cref="GameFlowManager"/> means it can be
    /// tested directly, without standing up the flow singleton, the player profile and the
    /// registry just to ask a question about arithmetic.
    ///
    /// The rule in one line: start the player on pole, then move them back one place for every
    /// car in the field that is expected to be at least as quick as them.
    ///
    /// Two things follow from that, and both are choices rather than accidents:
    ///
    ///   - **Ties go against the player.** A lap equal to a car's benchmark is not faster than
    ///     it. The alternative silently rewards a player for matching a benchmark exactly.
    ///   - **The player can never start behind the last car.** The field is the AI cars; the
    ///     player is not one of them, so a field of 8 puts even the slowest player 9th.
    ///
    /// A car is "expected" to be quick because <see cref="AIFieldDefinition"/> gives every car
    /// a benchmark lap derived from the difficulty that already describes how hard it pushes.
    /// Qualifying is player-only, so no car has actually run a lap to be measured against —
    /// which is the whole reason a benchmark is needed.
    /// </summary>
    public static class GridPositionResolver
    {
        /// <summary>
        /// Resolves the one-based grid position for a qualifying lap.
        /// </summary>
        /// <param name="playerLapSeconds">The player's best qualifying lap.</param>
        /// <param name="fieldCount">How many AI cars are racing.</param>
        /// <param name="benchmarkFor">
        /// The benchmark lap for the car at an index, in the same order the field spawns.
        /// Null is treated as an empty field.
        /// </param>
        /// <returns>A one-based position: 1 is pole, and at most <c>fieldCount + 1</c>.</returns>
        public static int Resolve(float playerLapSeconds, int fieldCount,
            Func<int, float> benchmarkFor)
        {
            // Nothing to rank against, or nothing to rank. With no field there is no grid
            // order to place the player in, and with no lap there is no result to place them
            // by; pole is the only answer either case supports.
            if (fieldCount <= 0 || benchmarkFor == null || playerLapSeconds <= 0f)
                return 1;

            int position = 1;
            for (int i = 0; i < fieldCount; i++)
            {
                if (benchmarkFor(i) <= playerLapSeconds)
                    position++;
            }

            return Mathf.Min(position, fieldCount + 1);
        }
    }
}
