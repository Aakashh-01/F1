using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UI;
using F1.GameData;
using F1.GameFlow;

/// <summary>
/// Checks on the wing setup screen's tile UI.
///
/// These exist because the screen was once fully "wired" and still showed the player
/// nothing useful: two bare 200x40 Toggles sitting on top of the car, no selected state,
/// and generated artwork assigned to WingAeroProfile.icon that no line of code in the
/// project ever read. Every assertion here is a guard against one of those specific
/// failures coming back.
///
/// The UI impls (F1.UI.WingSetupScreenImpl, WingCardImpl) live in Assembly-CSharp, which
/// neither test assembly references, so they are reached by type name and through the
/// F1.GameFlow base type — the same route the other flow tests take. Adding an asmdef
/// reference to Assembly-CSharp is not possible; an assembly definition cannot reference
/// the predefined one.
/// </summary>
public class WingScreenUiTests
{
    private const string WingCardPath = "Assets/Prefabs/WingCard_Prefab.prefab";
    private const string WingScreenPath = "Assets/Prefabs/WingSetupScreen_Prefab.prefab";

    private GameObject _instance;

    [TearDown]
    public void TearDown()
    {
        if (_instance != null) Object.DestroyImmediate(_instance);
        _instance = null;
    }

    /// <summary>Finds a component by class name, since the type is not referenceable here.</summary>
    private static Component FindByTypeName(GameObject root, string typeName)
    {
        return root.GetComponentsInChildren<MonoBehaviour>(true)
            .FirstOrDefault(mb => mb != null && mb.GetType().Name == typeName);
    }

    private static CarDefinition FirstCarWithBothWingProfiles()
    {
        return AssetDatabase.FindAssets("t:CarDefinition")
            .Select(g => AssetDatabase.LoadAssetAtPath<CarDefinition>(AssetDatabase.GUIDToAssetPath(g)))
            .FirstOrDefault(c => c != null
                                 && c.HighDownforceAero != null
                                 && c.LowDownforceAero != null);
    }

    // ---------------------------------------------------------------------------
    // Artwork
    // ---------------------------------------------------------------------------

    /// <summary>
    /// Every wing profile must carry its artwork.
    ///
    /// A null icon is not a cosmetic problem: the tile renders an empty panel, and because
    /// the profile is a ScriptableObject the emptiness is silent — the screen still
    /// populates, the tests still pass, and the player just sees two blank boxes.
    /// </summary>
    [Test]
    public void EveryWingAeroProfile_HasArtworkAssigned()
    {
        // WingAeroProfile sits in namespace F1, not F1.GameData, despite living in the
        // GameData folder — so it is named in full rather than pulled in with a using.
        var profiles = AssetDatabase.FindAssets("t:WingAeroProfile")
            .Select(g => AssetDatabase.LoadAssetAtPath<F1.WingAeroProfile>(AssetDatabase.GUIDToAssetPath(g)))
            .Where(p => p != null)
            .ToList();

        Assert.IsNotEmpty(profiles, "No WingAeroProfile assets found at all.");

        var missing = profiles.Where(p => p.icon == null).Select(p => p.name).ToList();
        Assert.IsEmpty(missing,
            "Wing profiles with no icon. Their tile will render as an empty panel: "
            + string.Join(", ", missing));
    }

    /// <summary>
    /// Each car's two profiles must resolve, or one of the two tiles has no art and no
    /// physical meaning behind it.
    /// </summary>
    [Test]
    public void EveryCar_ResolvesBothWingProfiles()
    {
        var cars = AssetDatabase.FindAssets("t:CarDefinition")
            .Select(g => AssetDatabase.LoadAssetAtPath<CarDefinition>(AssetDatabase.GUIDToAssetPath(g)))
            .Where(c => c != null)
            .ToList();

        Assert.IsNotEmpty(cars, "No CarDefinition assets found.");

        var bad = cars.Where(c => c.GetAeroForWing(WingType.HighDownforce) == null
                                  || c.GetAeroForWing(WingType.LowDownforce) == null)
                      .Select(c => c.CarId).ToList();
        Assert.IsEmpty(bad, "Cars missing a wing profile: " + string.Join(", ", bad));
    }

    // ---------------------------------------------------------------------------
    // The tile prefab
    // ---------------------------------------------------------------------------

    /// <summary>
    /// The tile must actually be able to display a picture, take a click, and show that it
    /// is selected. Each of these was missing on the first attempt.
    /// </summary>
    [Test]
    public void WingCard_HasArtworkButtonAndSelectedIndicator()
    {
        var card = AssetDatabase.LoadAssetAtPath<GameObject>(WingCardPath);
        Assert.IsNotNull(card, $"Wing card prefab missing at {WingCardPath}.");

        var art = card.transform.Find("WingArt")?.GetComponent<Image>();
        Assert.IsNotNull(art, "WingCard has no WingArt Image, so WingAeroProfile.icon "
                              + "has nowhere to go and the tile is invisible by construction.");
        Assert.IsTrue(art.preserveAspect,
            "WingArt must preserveAspect; the artwork is a wide image and would smear "
            + "into the panel otherwise.");

        Assert.IsNotNull(card.GetComponentInChildren<Button>(true),
            "WingCard has no Button, so the tile cannot be pressed.");

        var indicator = card.transform.Find("SelectedIndicator");
        Assert.IsNotNull(indicator,
            "WingCard has no SelectedIndicator, so there is no way to see which setup is "
            + "chosen. The child name must match CarCardImpl so the existing pattern carries over.");
        Assert.IsFalse(indicator.gameObject.activeSelf,
            "WingCard ships with its selected state on. Every tile would render as chosen "
            + "until the screen configures it.");
    }

    /// <summary>
    /// The artwork Image and the button must not both swallow raycasts in the middle of
    /// the tile, or the press lands on a label and the tile is only clickable around its
    /// edges.
    /// </summary>
    [Test]
    public void WingCard_ArtworkAndLabelsDoNotBlockClicks()
    {
        var card = AssetDatabase.LoadAssetAtPath<GameObject>(WingCardPath);
        Assert.IsNotNull(card);

        var button = card.GetComponentInChildren<Button>(true);
        Assert.IsNotNull(button);
        Assert.IsTrue(button.GetComponent<Image>().raycastTarget,
            "The tile's Button Image must be a raycast target or the tile cannot be clicked.");

        foreach (var name in new[] { "WingArt", "WingArtBacking", "TitleText", "BlurbText" })
        {
            var g = card.transform.Find(name)?.GetComponent<Graphic>();
            Assert.IsNotNull(g, $"WingCard is missing {name}.");
            Assert.IsFalse(g.raycastTarget,
                $"{name} is a raycast target and sits above the tile's Button, so presses in "
                + "the middle of the tile are swallowed.");
        }
    }

    // ---------------------------------------------------------------------------
    // The screen prefab
    // ---------------------------------------------------------------------------

    /// <summary>
    /// The two bare Toggles are gone and the tile strip is there instead. This is the
    /// specific defect the whole pass set out to fix, and it is invisible to a test that
    /// only checks the screen "has a wing UI".
    /// </summary>
    [Test]
    public void WingScreen_HasTileStripAndNoBareToggles()
    {
        var screen = AssetDatabase.LoadAssetAtPath<GameObject>(WingScreenPath);
        Assert.IsNotNull(screen, $"Wing screen prefab missing at {WingScreenPath}.");

        var strip = screen.transform.Find("TileStrip");
        Assert.IsNotNull(strip, "Wing screen has no TileStrip, so it has nowhere to put tiles.");
        Assert.IsNotNull(strip.GetComponent<HorizontalLayoutGroup>(),
            "TileStrip has no HorizontalLayoutGroup to lay the tiles out along.");

        foreach (var dead in new[] { "HighDownforceToggle", "LowDownforceToggle" })
        {
            Assert.IsNull(screen.transform.Find(dead),
                $"{dead} is still in the prefab. It is a 200x40 control that sits on top of "
                + "the car and was replaced by the tile strip.");
        }
    }

    /// <summary>
    /// Guards the known UnityEventTools trap: a persistent listener added to a throwaway
    /// scene object before the prefab's first save is silently dropped, producing a screen
    /// whose buttons look right and do nothing.
    /// </summary>
    [Test]
    public void WingScreen_ContinueAndBackAreWiredToTheImpl()
    {
        var screen = AssetDatabase.LoadAssetAtPath<GameObject>(WingScreenPath);
        Assert.IsNotNull(screen);

        foreach (var name in new[] { "ContinueButton", "BackButton" })
        {
            var button = screen.transform.Find(name)?.GetComponent<Button>();
            Assert.IsNotNull(button, $"Wing screen has no {name}.");
            Assert.AreEqual(1, button.onClick.GetPersistentEventCount(),
                $"{name} has {button.onClick.GetPersistentEventCount()} persistent listeners; "
                + "expected exactly 1. Zero means the button is inert, more than one means a "
                + "press fires the handler twice.");
        }
    }

    [Test]
    public void WingScreen_WiresTheWingCardPrefab()
    {
        var screen = AssetDatabase.LoadAssetAtPath<GameObject>(WingScreenPath);
        var impl = FindByTypeName(screen, "WingSetupScreenImpl");
        Assert.IsNotNull(impl, "No WingSetupScreenImpl on the wing screen prefab.");

        var so = new SerializedObject(impl);
        var prop = so.FindProperty("_wingCardPrefab");
        Assert.IsNotNull(prop, "WingSetupScreenImpl has no _wingCardPrefab field.");
        Assert.IsNotNull(prop.objectReferenceValue,
            "The screen's wing card prefab is unassigned, so the strip will stay empty.");
    }

    // ---------------------------------------------------------------------------
    // Behaviour: the screen populates two tiles
    // ---------------------------------------------------------------------------

    /// <summary>
    /// The end-to-end one: configure the screen with a real car and check it produces one
    /// tile per WingType, each showing that wing's own artwork and marking the current
    /// choice. This is the assertion that would have failed on the original screen, which
    /// populated nothing and read no icon at all.
    /// </summary>
    [Test]
    public void WingScreen_SpawnsOneTilePerWingType_CarryingThatWingsArtwork()
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(WingScreenPath);
        var car = FirstCarWithBothWingProfiles();
        Assert.IsNotNull(prefab);
        Assert.IsNotNull(car, "No CarDefinition has both wing profiles assigned.");

        _instance = (GameObject)Object.Instantiate(prefab);
        try
        {
            // Reached through the F1.GameFlow base type, which the test assembly does
            // reference, rather than through the Assembly-CSharp impl.
            var screen = (WingSetupScreen)_instance.GetComponent("WingSetupScreenImpl");
            Assert.IsNotNull(screen, "Instantiated screen has no WingSetupScreen component.");

            screen.Setup(car, null, WingType.LowDownforce);

            var strip = _instance.transform.Find("TileStrip");
            Assert.IsNotNull(strip, "The instantiated screen has no TileStrip.");

            var tiles = strip.Cast<Transform>().ToList();
            Assert.AreEqual(2, tiles.Count,
                $"Expected one tile per WingType, found {tiles.Count}: "
                + string.Join(", ", tiles.Select(t => t.name)));

            foreach (var wing in new[] { WingType.HighDownforce, WingType.LowDownforce })
            {
                var named = tiles.FirstOrDefault(t => t.name == $"Tile_{wing}");
                Assert.IsNotNull(named, $"No tile named Tile_{wing}.");

                var art = named.Find("WingArt")?.GetComponent<Image>();
                Assert.IsNotNull(art, $"{named.name} has no WingArt Image.");
                Assert.AreEqual(car.GetAeroForWing(wing).icon, art.sprite,
                    $"{named.name} is not showing its own profile's artwork. Both profiles "
                    + "carry different sprites, so a mix-up here is invisible in a count of "
                    + "tiles but obvious to the player.");

                var title = named.Find("TitleText")?.GetComponent<TMPro.TextMeshProUGUI>();
                Assert.IsNotNull(title, $"{named.name} has no TitleText.");
                Assert.IsNotEmpty(title.text, $"{named.name} has an empty title.");

                var blurb = named.Find("BlurbText")?.GetComponent<TMPro.TextMeshProUGUI>();
                Assert.IsNotNull(blurb, $"{named.name} has no BlurbText.");
                Assert.IsNotEmpty(blurb.text,
                    $"{named.name} has no consequence line. Both profiles share "
                    + "downforceCoeff, so a number would read as no difference at all — the "
                    + "plain-English line is what tells the two tiles apart.");
            }

            // The session came in on Low, so Low must be the marked one and High must not.
            Assert.IsTrue(tiles.First(t => t.name == "Tile_LowDownforce")
                              .Find("SelectedIndicator").gameObject.activeSelf,
                "The tile for the current wing (Low) is not marked as selected.");
            Assert.IsFalse(tiles.First(t => t.name == "Tile_HighDownforce")
                               .Find("SelectedIndicator").gameObject.activeSelf,
                "The tile for the wing NOT in use (High) is marked as selected.");
        }
        finally
        {
            Object.DestroyImmediate(_instance);
            _instance = null;
        }
    }
}
