using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A reusable AI field: how many cars, who drives them, how hard they push, and what
/// numbers they carry.
///
/// The point of this asset is that the project owns exactly one AI car prefab. Every car in
/// the field is that same prefab. So the field size has to be a single number rather than a
/// hand-built list of prefab references, and the cars have to be told apart some other way —
/// which is what <c>DriverIdentifier</c> is for.
///
/// A hand-authored <see cref="roster"/> is optional and takes priority by index. Anything
/// past the end of the roster is generated from the pools, so a named driver keeps their name
/// and number while the cars behind them are filled in automatically. Going from eight cars
/// to twelve is a value change, not new prefabs.
/// </summary>
[CreateAssetMenu(menuName = "F1/AI Field Definition", fileName = "AI_Field_Definition")]
public class AIFieldDefinition : ScriptableObject
{
    /// <summary>One car in the field, and the data that makes it that car rather than a copy.</summary>
    [Serializable]
    public class FieldEntry
    {
        [Tooltip("Shown on the car's number board. Left empty, the name pool is used instead.")]
        public string driverName = "AI Driver";

        [Tooltip("Shown on the number board. Zero takes the next free number from the pool.")]
        public int number;

        [Tooltip("Used when no explicit difficulty profile is assigned.")]
        public AIDifficultyPreset difficulty = AIDifficultyPreset.Medium;

        [Tooltip("Optional. Wins over the preset above when assigned.")]
        public AIDifficultyProfile difficultyProfile;

        [Tooltip("Which side this driver prefers to pass on. Varies so the pack does not all move as one.")]
        public bool preferRightOvertake = true;

        [Range(-1f, 1f)]
        [Tooltip("Preferred racing line offset, 0 = centre. Left at 0, one is assigned automatically.")]
        public float preferredLaneOffset01;

        [Tooltip("Lap time in seconds this driver is expected to set, used to rank the " +
                 "player's qualifying lap into a grid slot. Zero uses the reference time " +
                 "for the difficulty above.")]
        public float benchmarkLapOverride;

        [Tooltip("Explicit grid slot. -1 fills automatically after the player and earlier entries.")]
        public int gridPosition = -1;
    }

    [Header("Field Size")]
    [Tooltip("How many AI cars this field puts on the grid. This is the scalability knob: " +
             "every car is the same prefab, and anything past the roster is generated.")]
    [Range(0, 23)] public int fieldSize = 8;

    [Header("Hand-authored roster (optional)")]
    [Tooltip("Drivers you have named yourself. These win over the pools by index, and any " +
             "car beyond the end of this list is generated to fill the field up to fieldSize.")]
    public FieldEntry[] roster = new FieldEntry[0];

    [Header("Pace benchmarks")]
    [Tooltip("Reference lap time for a Hard car, in seconds. A player's qualifying lap is " +
             "ranked against these, so this is what decides the grid they start from. Set " +
             "them from real laps on this track, not from a guess: a field benchmarked " +
             "wrongly puts every player on the wrong row.")]
    public float hardReferenceLapSeconds = 78f;

    [Tooltip("Reference lap time for a Medium car, in seconds.")]
    public float mediumReferenceLapSeconds = 84f;

    [Tooltip("Reference lap time for an Easy car, in seconds.")]
    public float easyReferenceLapSeconds = 91f;

    [Header("Generated-car pools")]
    [Tooltip("Names for generated cars, cycled. Keep this at least as long as the largest " +
             "fieldSize you expect, or names will repeat.")]
    public string[] driverNamePool =
    {
        "V. Verga", "K. Oduya", "M. Halvorsen", "R. Castellan",
        "J. Moreau", "A. Petrov", "S. Nakamura", "D. Okonkwo",
        "L. Ferreira", "T. Brennan", "N. Kowalski", "C. Duarte",
        "E. Lindqvist", "P. Rasmussen", "H. Yilmaz", "B. Achterberg",
        "F. Mwangi", "G. Rossi", "I. Sandberg", "O. Delacroix",
        "U. Vasquez", "Y. Tanaka", "Z. Hoffmann", "Q. Bhandari"
    };

    [Tooltip("Numbers for generated cars, used in order and never twice in one field.")]
    public int[] numberPool =
    {
        2, 3, 5, 7, 8, 10, 11, 14, 16, 18, 19, 20, 21, 22, 23, 44,
        55, 63, 77, 88, 4, 6, 9, 12
    };

    [Tooltip("Difficulty for generated cars, cycled. A field that is all one difficulty is a " +
             "procession, so this deliberately mixes them.")]
    public AIDifficultyPreset[] difficultyLadder =
    {
        AIDifficultyPreset.Hard, AIDifficultyPreset.Medium, AIDifficultyPreset.Medium,
        AIDifficultyPreset.Easy, AIDifficultyPreset.Medium, AIDifficultyPreset.Hard,
        AIDifficultyPreset.Easy, AIDifficultyPreset.Medium
    };

    /// <summary>How many AI cars this field actually produces. Never negative.</summary>
    public int Count => Mathf.Max(0, fieldSize);

    /// <summary>
    /// The lap time this field's car at <paramref name="index"/> is expected to set.
    ///
    /// This is the field's answer to "how fast is this car?", and it exists so a player's
    /// qualifying lap can be ranked against a grid that has never actually run. Qualifying
    /// is player-only (invariant 6), so there is no on-track result to compare against —
    /// without a benchmark the qualifying result cannot influence the grid at all, and every
    /// race would start from pole.
    ///
    /// The value comes from the car's difficulty, because that is already the thing that
    /// describes how hard it pushes. A hand-authored roster entry can override it, since a
    /// named driver is a specific car rather than a difficulty band.
    ///
    /// <paramref name="index"/> is the same index <see cref="BuildGridEntries"/> uses, so
    /// "the benchmark for car 3" and "the third entry spawned" are the same car.
    /// </summary>
    public float GetBenchmarkLapSeconds(int index)
    {
        FieldEntry authored = (roster != null && index >= 0 && index < roster.Length)
            ? roster[index]
            : null;

        if (authored != null && authored.benchmarkLapOverride > 0f)
            return authored.benchmarkLapOverride;

        AIDifficultyPreset preset = authored != null
            ? authored.difficulty
            : PickDifficulty(index);

        return GetReferenceLapSeconds(preset);
    }

    /// <summary>The reference lap time configured for one difficulty band.</summary>
    public float GetReferenceLapSeconds(AIDifficultyPreset preset)
    {
        switch (preset)
        {
            case AIDifficultyPreset.Hard: return hardReferenceLapSeconds;
            case AIDifficultyPreset.Easy: return easyReferenceLapSeconds;
            default: return mediumReferenceLapSeconds;
        }
    }

    /// <summary>
    /// Turns this definition into the grid entries <c>RaceGridManager</c> already knows how
    /// to spawn. This is the seam: the grid manager never learns about name pools or numbers,
    /// and this never learns about prefabs or grid geometry.
    /// </summary>
    public RaceGridManager.RaceGridEntry[] BuildGridEntries()
    {
        int count = Count;
        var result = new RaceGridManager.RaceGridEntry[count];
        var usedNumbers = new HashSet<int>();

        for (int i = 0; i < count; i++)
        {
            FieldEntry authored = (roster != null && i < roster.Length) ? roster[i] : null;

            string name = authored != null && !string.IsNullOrWhiteSpace(authored.driverName)
                ? authored.driverName
                : PickName(i);

            int number = authored != null && authored.number != 0
                ? authored.number
                : PickNumber(i, usedNumbers);
            usedNumbers.Add(number);

            AIDifficultyPreset preset = authored != null
                ? authored.difficulty
                : PickDifficulty(i);

            bool preferRight = authored != null
                ? authored.preferRightOvertake
                : i % 2 == 0;

            float lane = authored != null && !Mathf.Approximately(authored.preferredLaneOffset01, 0f)
                ? authored.preferredLaneOffset01
                : (i % 2 == 0 ? 0.34f : -0.34f);

            result[i] = new RaceGridManager.RaceGridEntry
            {
                driverName = name,
                driverNumber = number,
                gridPosition = authored != null ? authored.gridPosition : -1,
                difficultyProfile = authored != null ? authored.difficultyProfile : null,
                fallbackDifficulty = preset,
                preferRightOvertake = preferRight,
                preferredLaneOffset01 = lane,
                // The prefab is deliberately left null: every entry resolves to the grid
                // manager's single defaultAICarPrefab, which is the whole point of the field.
                carPrefabOverride = null,
                existingCar = null,
                spawnPoint = null,
                physicsProfileOverride = null
            };
        }

        return result;
    }

    private string PickName(int index)
    {
        if (driverNamePool == null || driverNamePool.Length == 0)
            return $"AI Driver {index + 1}";

        return driverNamePool[index % driverNamePool.Length];
    }

    private int PickNumber(int index, HashSet<int> used)
    {
        if (numberPool != null && numberPool.Length > 0)
        {
            // Walk the pool rather than indexing it, so a hand-authored number earlier in
            // the field does not leave a gap or a duplicate behind.
            for (int i = 0; i < numberPool.Length; i++)
            {
                int candidate = numberPool[(index + i) % numberPool.Length];
                if (!used.Contains(candidate))
                    return candidate;
            }
        }

        // Pool exhausted or absent. Fall back to the first unused number, which keeps the
        // numbers unique even when the pool is smaller than the field.
        int fallback = 1;
        while (used.Contains(fallback)) fallback++;
        return fallback;
    }

    private AIDifficultyPreset PickDifficulty(int index)
    {
        if (difficultyLadder == null || difficultyLadder.Length == 0)
            return AIDifficultyPreset.Medium;

        return difficultyLadder[index % difficultyLadder.Length];
    }
}
