namespace F1.GameFlow
{
    /// <summary>
    /// What a finishing position is worth in career points.
    ///
    /// A pure table, deliberately, for the same reason <see cref="GridPositionResolver"/> is
    /// one: this is session <i>policy</i> — what the GDD says a race result is worth — and
    /// keeping it out of <see cref="GameFlowManager"/> means it can be tested without standing
    /// up the flow singleton, the player profile and the save file just to ask a question
    /// about a lookup table.
    ///
    /// The numbers are the GDD's "Driver Ranking Rewards" table, verbatim:
    /// 1st 1250, 2nd 1000, 3rd 800, 4th 700, 5th 600, 6th 500, 7th 400, 8th 300, 9th 200,
    /// 10th 100.
    ///
    /// The table is written out rather than interpolated. The GDD's step from 2nd to 3rd is a
    /// 200-point drop while every step after it is 100, so any closed-form fit would have to
    /// special-case that one row anyway — and a special case in an "elegant" formula is worse
    /// to read than the literal list it was hiding.
    /// </summary>
    public static class RacePointsTable
    {
        /// <summary>
        /// Points per one-based finishing position, index 0 being pole. Indexes past the end
        /// of the list are handled by <see cref="PointsForPosition"/> rather than here.
        /// </summary>
        private static readonly int[] PointsByPosition =
        {
            1250, // 1st
            1000, // 2nd
            800,  // 3rd
            700,  // 4th
            600,  // 5th
            500,  // 6th
            400,  // 7th
            300,  // 8th
            200,  // 9th
            100,  // 10th
        };

        /// <summary>How many positions the GDD scores. Anything beyond this is worth nothing.</summary>
        public static int ScoredPositions => PointsByPosition.Length;

        /// <summary>
        /// Career points earned by finishing in a one-based position.
        /// </summary>
        /// <param name="oneBasedPosition">Finishing position, where 1 is the win.</param>
        /// <returns>
        /// The points for that position, or 0 when the position is outside the scored range.
        /// </returns>
        public static int PointsForPosition(int oneBasedPosition)
        {
            // Zero is a real answer here rather than an error case, and it is deliberately not
            // an exception: a position the table does not cover is a finish the GDD does not
            // score, and the caller should be able to award nothing and carry on. Finishing
            // outside the points is a legitimate outcome, not a broken one.
            if (oneBasedPosition < 1 || oneBasedPosition > PointsByPosition.Length)
                return 0;

            return PointsByPosition[oneBasedPosition - 1];
        }
    }
}