// Assets/Editor/TrackUiArtBuilder.cs
//
// Procedural art for the track selection screen
// (Assets/Scenes/ReferenceImageTrack.png): national flags, the orange SELECTED chip,
// the hero-card glow border, the dark BACK parallelogram, and the neon circuit maps.
//
// The circuit maps are RANDOM stylized layouts for now — seeded per track id so the
// same track always regenerates the same shape — drawn as a red neon line with a soft
// glow, matching the reference's look. When real track geometry lands, regenerate the
// map for that track id from its racing line and nothing else changes: the
// TrackDefinition._thumbnail reference is rewritten here either way.
//
// Re-runnable: overwrites the PNGs and the thumbnail references in place.
// Run Tools > Garage > Build Track UI Art.
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using F1.GameData;

public static class TrackUiArtBuilder
{
    private const string Dir = "Assets/Art/UI/Tracks";

    private static readonly Color Red1 = new Color(0.808f, 0.067f, 0.149f);   // F1 red #CE1126

    [MenuItem("Tools/Garage/Build Track UI Art", priority = 22)]
    public static void BuildAll()
    {
        EnsureFolder();

        BuildFlag("Flag_Monaco", (x, y) => y > 32f ? Color.white : Red1);
        BuildFlag("Flag_Italy", (x, y) => x < 32f ? new Color(0f, 0.56f, 0.31f)
            : x < 64f ? Color.white : new Color(0.8f, 0.12f, 0.15f));
        BuildFlag("Flag_Belgium", (x, y) => x < 32f ? new Color(0.11f, 0.11f, 0.12f)
            : x < 64f ? new Color(0.99f, 0.8f, 0.15f) : Red1);
        BuildFlag("Flag_Japan", (x, y) =>
            (new Vector2(x - 48f, y - 32f).magnitude < 13f) ? Red1 : Color.white);
        BuildFlag("Flag_GreatBritain", UnionJack);

        BuildSelectedChip();
        BuildDarkParallelogram();
        BuildHeroBorder();
        BuildCircuitMaps();

        AssetDatabase.SaveAssets();
        Debug.Log("[TrackUiArt] Track sprites written and thumbnails assigned.");
    }

    // ==================================================================================
    // Flags — 96x64 with a hairline light border.
    // ==================================================================================

    private static void BuildFlag(string name, GarageUiArtBuilder.ColorAt colorAt)
    {
        GarageUiArtBuilder.Write(name, 96, 64,
            GarageUiArtBuilder.Rect(1f, 1f, 95f, 63f),
            (x, y) =>
            {
                if (x < 2f || x > 94f || y < 2f || y > 62f) return new Color(0.85f, 0.87f, 0.9f);
                return colorAt(x - 2f, y - 2f);
            }, Vector4.zero, Dir);
    }

    private static Color UnionJack(float x, float y)
    {
        const float w = 92f, h = 60f;
        var navy = new Color(0.08f, 0.18f, 0.48f);
        var white = Color.white;

        // Diagonals: white band with a red core.
        float d1 = LineDistance(x, y, 0, 0, w, h);
        float d2 = LineDistance(x, y, 0, h, w, 0);
        if (d1 < 5f || d2 < 5f) return white;
        if (d1 < 2.2f || d2 < 2.2f) return Red1;

        // Cross: white field with a red centre.
        if (Mathf.Abs(x - w / 2f) < 11f || Mathf.Abs(y - h / 2f) < 11f) return white;
        if (Mathf.Abs(x - w / 2f) < 6.5f || Mathf.Abs(y - h / 2f) < 6.5f) return Red1;
        return navy;
    }

    private static float LineDistance(float px, float py, float ax, float ay, float bx, float by)
    {
        var ab = new Vector2(bx - ax, by - ay);
        float t = Mathf.Clamp01(Vector2.Dot(new Vector2(px - ax, py - ay), ab) / ab.sqrMagnitude);
        return (new Vector2(px, py) - (new Vector2(ax, ay) + ab * t)).magnitude;
    }

    // ==================================================================================
    // Chrome pieces.
    // ==================================================================================

    /// <summary>Orange banner chip, 256x64, with the reference's notched left edge.</summary>
    private static void BuildSelectedChip()
    {
        GarageUiArtBuilder.Write("Chip_Selected", 256, 64,
            GarageUiArtBuilder.Poly(new[]
            {
                new Vector2(0f, 32f), new Vector2(18f, 64f), new Vector2(256f, 64f),
                new Vector2(256f, 0f), new Vector2(18f, 0f),
            }),
            (x, y) => Color.Lerp(new Color(0.95f, 0.42f, 0.05f), new Color(1f, 0.62f, 0.15f), y / 64f),
            new Vector4(24, 12, 12, 12), Dir);
    }

    /// <summary>The BACK button: same parallelogram geometry as Continue, dark glass.</summary>
    private static void BuildDarkParallelogram()
    {
        GarageUiArtBuilder.Write("Button_Parallelogram_Dark", 288, 128,
            GarageUiArtBuilder.Poly(new[]
            {
                new Vector2(24f, 0f), new Vector2(238f, 0f), new Vector2(264f, 128f), new Vector2(50f, 128f),
            }),
            (x, y) =>
            {
                float v = y / 128f;
                var c = Color.Lerp(new Color(0.07f, 0.075f, 0.09f), new Color(0.24f, 0.26f, 0.30f), v);
                if (v > 0.55f) c = Color.Lerp(c, Color.white, (v - 0.55f) / 0.45f * 0.18f);
                return c;
            },
            new Vector4(60, 40, 60, 40), Dir);
    }

    /// <summary>Hero-card frame: rounded orange stroke with an inner and outer glow
    /// falloff. 9-slice with generous borders so the glow survives scaling.</summary>
    private static void BuildHeroBorder()
    {
        var outer = GarageUiArtBuilder.RoundedRect(64f, 64f, 64f, 64f, 18f);
        var inner = GarageUiArtBuilder.RoundedRect(64f, 64f, 58f, 58f, 14f);
        GarageUiArtBuilder.Write("Border_Hero", 128, 128,
            (x, y) => Mathf.Max(0f, outer(x, y) - inner(x, y)),
            (x, y) =>
            {
                var stroke = GarageUiArtBuilder.RoundedRect(64f, 64f, 61.5f, 61.5f, 16f)(x, y)
                           - GarageUiArtBuilder.RoundedRect(64f, 64f, 60f, 60f, 15f)(x, y);
                var glow = Mathf.Clamp01(1f - Mathf.Abs(stroke) * 1.4f);
                return Color.Lerp(new Color(1f, 0.45f, 0.08f), new Color(1f, 0.85f, 0.55f), glow);
            },
            new Vector4(40, 40, 40, 40), Dir);
    }

    // ==================================================================================
    // Circuit maps — seeded random neon layouts, assigned as track thumbnails.
    // ==================================================================================

    private const int MapW = 512, MapH = 360;

    private static void BuildCircuitMaps()
    {
        var tracks = AssetDatabase.FindAssets("t:TrackDefinition")
            .Select(g => AssetDatabase.LoadAssetAtPath<TrackDefinition>(AssetDatabase.GUIDToAssetPath(g)))
            .Where(t => t != null)
            .ToList();

        foreach (var track in tracks)
        {
            var points = RandomCircuit(track.TrackId);
            var tex = RenderNeonMap(points);
            GarageUiArtBuilder.WriteRaw($"Map_{track.TrackId}", tex, Vector4.zero, Dir);
            UnityEngine.Object.DestroyImmediate(tex);

            var sprite = AssetDatabase.LoadAssetAtPath<Sprite>($"{Dir}/Map_{track.TrackId}.png");
            var so = new SerializedObject(track);
            so.FindProperty("_thumbnail").objectReferenceValue = sprite;
            so.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(track);
        }
        AssetDatabase.SaveAssets();
    }

    /// <summary>A closed loop of jittered polar control points smoothed with Catmull-Rom —
    /// reads as a plausible street/permanent circuit and is stable per track id.</summary>
    private static List<Vector2> RandomCircuit(string trackId)
    {
        uint hash = 2166136261;
        foreach (char c in trackId) { hash ^= c; hash *= 16777619; }
        var rng = new System.Random((int)(hash & 0x7fffffff));

        int n = 9 + rng.Next(5);
        var control = new List<Vector2>();
        for (int i = 0; i < n; i++)
        {
            float ang = (i / (float)n) * Mathf.PI * 2f + (float)(rng.NextDouble() - 0.5) * 0.5f;
            float r = 0.42f + (float)rng.NextDouble() * 0.58f;
            control.Add(new Vector2(Mathf.Cos(ang) * r * 1.25f, Mathf.Sin(ang) * r));
        }

        var samples = new List<Vector2>();
        for (int i = 0; i < n; i++)
        {
            Vector2 p0 = control[(i + n - 1) % n], p1 = control[i],
                    p2 = control[(i + 1) % n], p3 = control[(i + 2) % n];
            for (int s = 0; s < 48; s++)
            {
                float t = s / 48f;
                samples.Add(CatmullRom(p0, p1, p2, p3, t));
            }
        }

        // Normalize into the canvas with margin, preserving aspect.
        float minX = float.MaxValue, maxX = float.MinValue, minY = minX, maxY = maxX;
        foreach (var p in samples)
        {
            minX = Mathf.Min(minX, p.x); maxX = Mathf.Max(maxX, p.x);
            minY = Mathf.Min(minY, p.y); maxY = Mathf.Max(maxY, p.y);
        }
        float scale = Mathf.Min((MapW - 90f) / (maxX - minX), (MapH - 90f) / (maxY - minY));
        for (int i = 0; i < samples.Count; i++)
        {
            samples[i] = new Vector2(
                (samples[i].x - (minX + maxX) / 2f) * scale + MapW / 2f,
                (samples[i].y - (minY + maxY) / 2f) * scale + MapH / 2f);
        }
        return samples;
    }

    private static Vector2 CatmullRom(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3, float t)
    {
        float t2 = t * t, t3 = t2 * t;
        return 0.5f * ((2f * p1) + (-p0 + p2) * t
            + (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2
            + (-p0 + 3f * p1 - 3f * p2 + p3) * t3);
    }

    private static Texture2D RenderNeonMap(List<Vector2> samples)
    {
        var intensity = new float[MapW * MapH];

        foreach (var p in samples)
            Stamp(intensity, p, 16f, 0.055f);          // wide soft glow
        foreach (var p in samples)
            Stamp(intensity, p, 5.5f, 0.35f);           // halo
        foreach (var p in samples)
            Stamp(intensity, p, 2.4f, 1.4f);            // core line

        var tex = new Texture2D(MapW, MapH, TextureFormat.RGBA32, false);
        var pixels = new Color32[MapW * MapH];
        for (int i = 0; i < pixels.Length; i++)
        {
            float v = intensity[i];
            if (v <= 0.004f) { pixels[i] = new Color(0, 0, 0, 0); continue; }
            var c = Color.Lerp(new Color(0.85f, 0.06f, 0.08f), new Color(1f, 0.75f, 0.65f),
                Mathf.Clamp01((v - 0.8f) * 0.9f));
            c.a = Mathf.Clamp01(v * 1.6f);
            pixels[i] = c;
        }
        tex.SetPixels32(pixels);
        tex.Apply();
        return tex;
    }

    private static void Stamp(float[] buffer, Vector2 center, float radius, float strength)
    {
        int x0 = Mathf.Max(0, Mathf.FloorToInt(center.x - radius));
        int x1 = Mathf.Min(MapW - 1, Mathf.CeilToInt(center.x + radius));
        int y0 = Mathf.Max(0, Mathf.FloorToInt(center.y - radius));
        int y1 = Mathf.Min(MapH - 1, Mathf.CeilToInt(center.y + radius));
        float r2 = radius * radius;

        for (int y = y0; y <= y1; y++)
        {
            float dy = y + 0.5f - center.y;
            for (int x = x0; x <= x1; x++)
            {
                float dx = x + 0.5f - center.x;
                float d2 = dx * dx + dy * dy;
                if (d2 >= r2) continue;
                float fall = 1f - d2 / r2;
                buffer[y * MapW + x] += strength * fall * fall;
            }
        }
    }

    private static void EnsureFolder()
    {
        if (!AssetDatabase.IsValidFolder("Assets/Art/UI"))
        {
            GarageUiArtBuilder.EnsureFolder();
            return;
        }
        if (!AssetDatabase.IsValidFolder(Dir)) AssetDatabase.CreateFolder("Assets/Art/UI", "Tracks");
    }
}
