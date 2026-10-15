// Assets/Editor/GarageUiArtBuilder.cs
//
// Procedural 2D art for the garage screen's reference look
// (Assets/Scenes/ReferenceImageCarselection.png): glass panels with rounded corners,
// red stat bars, chevron arrows, currency icons and the orange parallelogram Continue.
//
// Everything is rasterized from signed-distance / polygon coverage at 4x supersampling
// and written as PNGs under Assets/Art/UI/Garage with 9-slice borders set in the
// importer. The shape sprites ship white so a tint drives their colour, the same
// convention the SlimUI pack uses; only the icons and the parallelogram bake colour in.
//
// Re-runnable: overwrites the PNGs in place. Run Tools > Garage > Build UI Art.
using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using TMPro;

public static class GarageUiArtBuilder
{
    private const string Dir = "Assets/Art/UI/Garage";
    private const string FontDir = "Assets/TextMesh Pro/Fonts";
    private const string DefaultFontPath = "Assets/TextMesh Pro/Resources/Fonts & Materials/LiberationSans SDF.asset";

    internal delegate float Coverage(float x, float y);
    internal delegate Color ColorAt(float x, float y);

    [MenuItem("Tools/Garage/Build UI Art", priority = 20)]
    public static void BuildAll()
    {
        EnsureFolder();

        // --- Structural shapes: white, tinted at use site. ---
        Write("Panel_Rounded", 96, 96, RoundedRect(48, 48, 48, 48, 24),
            (x, y) => Color.white, new Vector4(28, 28, 28, 28));
        Write("Panel_Outline", 96, 96, (x, y) =>
            RoundedRect(48, 48, 48, 48, 24)(x, y) * (1f - RoundedRect(48, 48, 47, 47, 23)(x, y)),
            (x, y) => Color.white, new Vector4(28, 28, 28, 28));
        Write("Pill_Rounded", 64, 16, RoundedRect(32, 8, 32, 8, 8),
            (x, y) => Color.white, new Vector4(8, 8, 8, 8));
        Write("Chevron_Right", 128, 128, Stroke(new[] { new Vector2(46, 24), new Vector2(92, 64), new Vector2(46, 104) }, 13f),
            (x, y) => Color.white, Vector4.zero);
        Write("Chevron_Left", 128, 128, (x, y) => Stroke(new[] { new Vector2(46, 24), new Vector2(92, 64), new Vector2(46, 104) }, 13f)(128f - x, y),
            (x, y) => Color.white, Vector4.zero);

        // --- Icons: colour baked in. ---
        Write("Coin", 128, 128, Disc(64, 64, 60), CoinColor, Vector4.zero);
        Write("Gem", 128, 128, Poly(new[] { new Vector2(44, 96), new Vector2(84, 96), new Vector2(110, 72), new Vector2(64, 22), new Vector2(18, 72) }),
            GemColor, Vector4.zero);
        Write("Sparkle", 256, 256, Star4(128, 128, 122, 16), (x, y) => Color.white, Vector4.zero);
        Write("Helmet_Avatar", 256, 256, HelmetCoverage, HelmetColor, Vector4.zero);

        // --- The Continue button: parallelogram with gradient + gloss baked in. ---
        Write("Button_Parallelogram", 288, 128,
            Poly(new[] { new Vector2(24, 0), new Vector2(238, 0), new Vector2(264, 128), new Vector2(50, 128) }),
            ParallelogramColor, new Vector4(60, 40, 60, 40));

        AssetDatabase.SaveAssets();
        Debug.Log($"[GarageUiArt] UI sprites written to {Dir}.");
    }

    /// <summary>
    /// Creates the TMP font assets for Titillium Web (OFL, downloaded into the Fonts
    /// folder). Each falls back to LiberationSans SDF so any glyph Titillium lacks still
    /// renders. Dynamic atlas: characters are baked on first use, no bake step to run.
    /// </summary>
    [MenuItem("Tools/Garage/Build F1 Font Assets", priority = 21)]
    public static void BuildFontAssets()
    {
        var fallback = AssetDatabase.LoadAssetAtPath<TMP_FontAsset>(DefaultFontPath);
        if (fallback == null)
            Debug.LogError($"[GarageUiArt] No default font at {DefaultFontPath}; fallbacks will be empty.");

        foreach (var face in new[] { "SemiBold", "Bold", "BoldItalic" })
        {
            var ttfPath = $"{FontDir}/TitilliumWeb-{face}.ttf";
            var font = AssetDatabase.LoadAssetAtPath<Font>(ttfPath);
            if (font == null)
            {
                Debug.LogError($"[GarageUiArt] {ttfPath} missing; download it from the google/fonts repo (ofl/titilliumweb).");
                continue;
            }

            var assetPath = $"{FontDir}/TitilliumWeb-{face} SDF.asset";
            if (AssetDatabase.LoadAssetAtPath<TMP_FontAsset>(assetPath) != null) continue;

            var tmpAsset = TMP_FontAsset.CreateFontAsset(font, 66, 5,
                UnityEngine.TextCore.LowLevel.GlyphRenderMode.SDFAA, 2048, 2048,
                AtlasPopulationMode.Dynamic, true);
            tmpAsset.name = $"TitilliumWeb-{face} SDF";
            if (fallback != null)
                tmpAsset.fallbackFontAssetTable = new List<TMP_FontAsset> { fallback };
            AssetDatabase.CreateAsset(tmpAsset, assetPath);

            // CreateFontAsset leaves the material and atlas as in-memory objects; without
            // sub-assetting them the next domain reload finds "Font Atlas Texture ... is
            // missing" on every label and renders clipped glyphs.
            tmpAsset.material.name = $"{tmpAsset.name} Material";
            AssetDatabase.AddObjectToAsset(tmpAsset.material, tmpAsset);
            if (tmpAsset.atlasTexture != null)
            {
                tmpAsset.atlasTexture.name = $"{tmpAsset.name} Atlas";
                AssetDatabase.AddObjectToAsset(tmpAsset.atlasTexture, tmpAsset);
            }
            EditorUtility.SetDirty(tmpAsset);
        }

        AssetDatabase.SaveAssets();
        Debug.Log("[GarageUiArt] Titillium Web SDF font assets ready.");
    }

    // ==================================================================================
    // Rasterizer
    // ==================================================================================

    private const int Supersample = 4;

    internal static void Write(string name, int w, int h, Coverage coverage, ColorAt colorAt,
        Vector4 border, string dir = Dir)
    {
        var tex = new Texture2D(w, h, TextureFormat.RGBA32, false);
        var pixels = new Color32[w * h];
        float inv = 1f / Supersample;

        for (int py = 0; py < h; py++)
        {
            for (int px = 0; px < w; px++)
            {
                float a = 0f;
                for (int sy = 0; sy < Supersample; sy++)
                    for (int sx = 0; sx < Supersample; sx++)
                        a += coverage(px + (sx + 0.5f) * inv, py + (sy + 0.5f) * inv);
                a /= Supersample * Supersample;
                if (a <= 0.004f) { pixels[py * w + px] = new Color(0, 0, 0, 0); continue; }

                var c = colorAt(px, py);
                c.a = Mathf.Clamp01(a);
                pixels[py * w + px] = c;
            }
        }

        tex.SetPixels32(pixels);
        tex.Apply();
        WriteRaw(name, tex, border, dir);
        UnityEngine.Object.DestroyImmediate(tex);
    }

    /// <summary>Writes a pre-rasterized texture as a sprite PNG and applies the importer
    /// settings shared by every generated asset.</summary>
    internal static void WriteRaw(string name, Texture2D tex, Vector4 border, string dir = Dir)
    {
        File.WriteAllBytes($"{dir}/{name}.png", tex.EncodeToPNG());

        AssetDatabase.ImportAsset($"{dir}/{name}.png");
        var importer = AssetImporter.GetAtPath($"{dir}/{name}.png") as TextureImporter;
        importer.textureType = TextureImporterType.Sprite;
        // Unity 6 defaults fresh PNGs to Multiple; with an empty sprite sheet that
        // produces NO sprite sub-asset at all and every LoadAssetAtPath<Sprite> is null.
        importer.spriteImportMode = SpriteImportMode.Single;
        importer.spriteBorder = border;
        importer.alphaIsTransparency = true;
        importer.isReadable = true;
        importer.mipmapEnabled = false;
        importer.npotScale = TextureImporterNPOTScale.None;
        importer.maxTextureSize = Mathf.NextPowerOfTwo(Mathf.Max(tex.width, tex.height));
        importer.textureCompression = TextureImporterCompression.Uncompressed;
        importer.SaveAndReimport();
    }

    // --- Coverage primitives. Coordinates are pixels, y up, outside = 0, edge = soft. ---

    internal static Coverage RoundedRect(float cx, float cy, float hw, float hh, float r)
    {
        return (x, y) =>
        {
            var q = new Vector2(Mathf.Abs(x - cx) - (hw - r), Mathf.Abs(y - cy) - (hh - r));
            float d = new Vector2(Mathf.Max(q.x, 0f), Mathf.Max(q.y, 0f)).magnitude
                       + Mathf.Min(Mathf.Max(q.x, q.y), 0f) - r;
            return Mathf.Clamp01(0.5f - d);
        };
    }

    internal static Coverage Disc(float cx, float cy, float r)
    {
        return (x, y) => Mathf.Clamp01(0.5f - (new Vector2(x - cx, y - cy).magnitude - r));
    }

    internal static Coverage Rect(float x0, float y0, float x1, float y1)
    {
        return (x, y) => (x >= x0 && x <= x1 && y >= y0 && y <= y1) ? 1f : 0f;
    }

    internal static Coverage Stroke(Vector2[] points, float width)
    {
        return (x, y) =>
        {
            float best = float.MaxValue;
            for (int i = 0; i < points.Length - 1; i++)
                best = Mathf.Min(best, SegmentDistance(new Vector2(x, y), points[i], points[i + 1]));
            return Mathf.Clamp01(0.5f - (best - width * 0.5f));
        };
    }

    private static float SegmentDistance(Vector2 p, Vector2 a, Vector2 b)
    {
        var ab = b - a;
        float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / ab.sqrMagnitude);
        return (p - (a + ab * t)).magnitude;
    }

    /// <summary>Even-odd polygon fill with a soft 1px edge.</summary>
    internal static Coverage Poly(Vector2[] verts)
    {
        return (x, y) =>
        {
            bool inside = false;
            for (int i = 0, j = verts.Length - 1; i < verts.Length; j = i++)
                if ((verts[i].y > y) != (verts[j].y > y) &&
                    x < (verts[j].x - verts[i].x) * (y - verts[i].y) / (verts[j].y - verts[i].y) + verts[i].x)
                    inside = !inside;

            if (inside)
            {
                float nearest = float.MaxValue;
                for (int i = 0, j = verts.Length - 1; i < verts.Length; j = i++)
                    nearest = Mathf.Min(nearest, SegmentDistance(new Vector2(x, y), verts[i], verts[j]));
                return Mathf.Clamp01(nearest - 0.5f);
            }
            return 0f;
        };
    }

    /// <summary>4-point star (the sparkle) plus a faint round glow underneath.</summary>
    private static Coverage Star4(float cx, float cy, float outer, float inner)
    {
        var verts = new Vector2[8];
        for (int i = 0; i < 8; i++)
        {
            float r = (i % 2 == 0) ? outer : inner;
            float ang = i * Mathf.PI * 0.25f;
            verts[i] = new Vector2(cx + Mathf.Cos(ang) * r, cy + Mathf.Sin(ang) * r);
        }
        var star = Poly(verts);
        var glow = Disc(cx, cy, outer * 0.55f);
        return (x, y) => Mathf.Max(star(x, y), glow(x, y) * 0.18f);
    }

    // ==================================================================================
    // Composed sprites
    // ==================================================================================

    private static Color CoinColor(float x, float y)
    {
        float d = new Vector2(x - 64, y - 64).magnitude;
        var face = Color.Lerp(new Color(1f, 0.84f, 0.25f), new Color(0.93f, 0.71f, 0.13f), d / 60f);
        if (d > 54f) face = new Color(0.78f, 0.55f, 0.08f);           // rim
        if (d > 60f) face = Color.Lerp(face, new Color(0.55f, 0.38f, 0.05f), (d - 60f));

        // The "C": an arc ring with the gap facing right.
        float ang = Mathf.Atan2(y - 64, x - 64) * Mathf.Rad2Deg;
        if (ang < 0) ang += 360f;
        bool ring = d > 22f && d < 38f;
        bool gap = (ang < 42f || ang > 318f);
        if (ring && !gap) face = new Color(0.45f, 0.30f, 0.03f);
        return face;
    }

    private static Color GemColor(float x, float y)
    {
        if (y > 72f) return new Color(0.42f, 0.85f, 0.98f);            // crown, lighter
        if (x < 64f) return new Color(0.10f, 0.55f, 0.78f);            // pavilion left, shadow
        return new Color(0.16f, 0.68f, 0.91f);                          // pavilion right
    }

    private static readonly Coverage HelmetShell = Disc(128, 142, 98);
    private static readonly Coverage HelmetVisor = RoundedRect(130, 132, 74, 26, 14);
    private static readonly Coverage HelmetChin = Rect(0, 0, 256, 60);

    private static float HelmetCoverage(float x, float y)
    {
        return Mathf.Max(HelmetShell(x, y) * (HelmetChin(x, y) > 0f ? 0f : 1f),
                         HelmetVisor(x, y) * (HelmetShell(x, y) > 0.5f ? 1f : 0f));
    }

    private static Color HelmetColor(float x, float y)
    {
        if (HelmetVisor(x, y) > 0.5f) return new Color(0.10f, 0.11f, 0.14f);
        if (y > 200f) return new Color(0.94f, 0.95f, 0.97f);           // white crown band
        return new Color(0.82f, 0.12f, 0.12f);                          // red shell
    }

    private static Color ParallelogramColor(float x, float y)
    {
        float v = y / 128f;
        var top = new Color(1f, 0.62f, 0.18f);
        var bottom = new Color(0.88f, 0.27f, 0.00f);
        var c = Color.Lerp(bottom, top, v);
        if (v > 0.55f) c = Color.Lerp(c, Color.white, (v - 0.55f) / 0.45f * 0.35f); // gloss
        return c;
    }

    // ==================================================================================

    internal static void EnsureFolder()
    {
        if (!AssetDatabase.IsValidFolder("Assets/Art")) AssetDatabase.CreateFolder("Assets", "Art");
        if (!AssetDatabase.IsValidFolder("Assets/Art/UI")) AssetDatabase.CreateFolder("Assets/Art", "UI");
        if (!AssetDatabase.IsValidFolder(Dir)) AssetDatabase.CreateFolder("Assets/Art/UI", "Garage");
    }
}
