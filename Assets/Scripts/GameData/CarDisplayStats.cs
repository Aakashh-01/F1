using UnityEngine;

namespace F1.GameData
{
    /// <summary>
    /// Turns <see cref="CarDefinition"/> display stats into 0..1 garage panel bar fills.
    ///
    /// The ranges are placeholder bounds spanning the current three generations, not real
    /// telemetry — they exist so the panel has stable, comparable bars now and the real
    /// bounds can replace them in one place later.
    /// </summary>
    public static class CarDisplayStats
    {
        public struct Range
        {
            public readonly float Min;
            public readonly float Max;

            public Range(float min, float max)
            {
                Min = min;
                Max = max;
            }
        }

        public static readonly Range PowerHP = new Range(700f, 1050f);
        public static readonly Range TorqueNm = new Range(600f, 850f);
        public static readonly Range TopSpeedKph = new Range(300f, 360f);
        public static readonly Range Accel0100Seconds = new Range(2.4f, 3.6f);
        public static readonly Range WeightKg = new Range(715f, 760f);

        /// <summary>Clamped 0..1 position of <paramref name="value"/> within <paramref name="range"/>.</summary>
        public static float Normalize01(Range range, float value)
        {
            if (range.Max <= range.Min) return 0f;
            return Mathf.Clamp01((value - range.Min) / (range.Max - range.Min));
        }
    }
}
