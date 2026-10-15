using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using F1.GameData;

/// <summary>
/// The garage stats panel's math, and the authored display data it renders.
///
/// The asset sweep is the guard for the builder's one-shot authoring pass: an unauthored
/// asset has zero everywhere, which no test fails on until the panel shows "0 HP" on the
/// showroom floor. The range test keeps the authored numbers inside the normalizer's
/// bounds, where a bar pegged at 0 or 1 is a silent loss of comparison.
/// </summary>
public class GarageDisplayStatsTests
{
    // --- The normalizer ---

    [Test]
    public void Normalize01_BelowTheRangeClampsToZero()
    {
        Assert.AreEqual(0f, CarDisplayStats.Normalize01(CarDisplayStats.PowerHP, 100f));
    }

    [Test]
    public void Normalize01_AboveTheRangeClampsToOne()
    {
        Assert.AreEqual(1f, CarDisplayStats.Normalize01(CarDisplayStats.PowerHP, 5000f));
    }

    [Test]
    public void Normalize01_AtTheMidpointIsHalf()
    {
        var range = new CarDisplayStats.Range(100f, 300f);
        Assert.AreEqual(0.5f, CarDisplayStats.Normalize01(range, 200f), 1e-5f);
    }

    [Test]
    public void Normalize01_AtTheRangeEndsMapsToTheEnds()
    {
        var range = new CarDisplayStats.Range(100f, 300f);
        Assert.AreEqual(0f, CarDisplayStats.Normalize01(range, 100f));
        Assert.AreEqual(1f, CarDisplayStats.Normalize01(range, 300f));
    }

    [Test]
    public void Normalize01_ADegenerateRangeIsZeroRatherThanNaN()
    {
        Assert.AreEqual(0f, CarDisplayStats.Normalize01(new CarDisplayStats.Range(5f, 5f), 5f));
    }

    // --- The authored car assets ---

    [Test]
    public void EveryCarAsset_HasGarageDisplayStatsAuthored()
    {
        var cars = LoadAllCars();
        Assert.Greater(cars.Count, 0, "No CarDefinition assets were found.");

        foreach (var car in cars)
        {
            Assert.IsFalse(string.IsNullOrWhiteSpace(car.EngineLabel),
                $"'{car.name}' has no engine label.");
            Assert.Greater(car.PowerHP, 0f, $"'{car.name}' has no power stat.");
            Assert.Greater(car.TorqueNm, 0f, $"'{car.name}' has no torque stat.");
            Assert.Greater(car.TopSpeedKph, 0f, $"'{car.name}' has no top-speed stat.");
            Assert.Greater(car.Accel0100Seconds, 0f, $"'{car.name}' has no 0-100 stat.");
            Assert.Greater(car.WeightKg, 0f, $"'{car.name}' has no weight stat.");
        }
    }

    [Test]
    public void EveryCarAsset_HasAPairwiseDistinctTint()
    {
        var cars = LoadAllCars();
        Assert.Greater(cars.Count, 0, "No CarDefinition assets were found.");

        for (int i = 0; i < cars.Count; i++)
        {
            for (int j = i + 1; j < cars.Count; j++)
            {
                Assert.AreNotEqual(cars[i].DisplayTint, cars[j].DisplayTint,
                    $"'{cars[i].name}' and '{cars[j].name}' share a display tint; while the " +
                    "cars share one body prefab the tint is the only visual difference.");
            }
        }
    }

    [Test]
    public void EveryCarAsset_StatsSitInsideThePanelRanges()
    {
        // A stat outside its range still renders, but its bar pegs at 0 or 1 and the
        // panel loses the comparison it exists for. The authoring pass is expected to
        // stay inside the bounds; if a real bounds change is wanted, change them
        // together, deliberately.
        foreach (var car in LoadAllCars())
        {
            Assert.GreaterOrEqual(CarDisplayStats.Normalize01(CarDisplayStats.PowerHP, car.PowerHP), 0f);
            Assert.LessOrEqual(CarDisplayStats.Normalize01(CarDisplayStats.PowerHP, car.PowerHP), 1f);
            Assert.GreaterOrEqual(CarDisplayStats.Normalize01(CarDisplayStats.TorqueNm, car.TorqueNm), 0f);
            Assert.LessOrEqual(CarDisplayStats.Normalize01(CarDisplayStats.TorqueNm, car.TorqueNm), 1f);
            Assert.GreaterOrEqual(CarDisplayStats.Normalize01(CarDisplayStats.TopSpeedKph, car.TopSpeedKph), 0f);
            Assert.LessOrEqual(CarDisplayStats.Normalize01(CarDisplayStats.TopSpeedKph, car.TopSpeedKph), 1f);
            Assert.GreaterOrEqual(CarDisplayStats.Normalize01(CarDisplayStats.Accel0100Seconds, car.Accel0100Seconds), 0f);
            Assert.LessOrEqual(CarDisplayStats.Normalize01(CarDisplayStats.Accel0100Seconds, car.Accel0100Seconds), 1f);
            Assert.GreaterOrEqual(CarDisplayStats.Normalize01(CarDisplayStats.WeightKg, car.WeightKg), 0f);
            Assert.LessOrEqual(CarDisplayStats.Normalize01(CarDisplayStats.WeightKg, car.WeightKg), 1f);
        }
    }

    private static System.Collections.Generic.List<CarDefinition> LoadAllCars()
    {
        return AssetDatabase.FindAssets("t:CarDefinition")
            .Select(guid => AssetDatabase.LoadAssetAtPath<CarDefinition>(AssetDatabase.GUIDToAssetPath(guid)))
            .Where(car => car != null)
            .ToList();
    }
}
