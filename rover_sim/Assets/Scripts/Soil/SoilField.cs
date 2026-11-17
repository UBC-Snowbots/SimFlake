using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>Where the soil sits before any wheel touched it: a Unity Terrain (<see cref="SoilTerrain"/>) or a test plane.</summary>
public interface ISoilBase
{
    bool Contains(float x, float z);
    /// <summary>World height (m) of the undisturbed surface, without small-scale relief.</summary>
    float BaseHeight(float x, float z);
    GroundSpec SpecAt(float x, float z);
    /// <summary>Amplitudes (m) of the small-scale relief at a point.</summary>
    void Roughness(float x, float z, out float fine, out float coarse);
}

/// <summary>
/// Tileable band-limited noise shared by physics (array) and the terrain shader (texture), so wheels feel exactly the
/// bumps you see. Two uses of the same map: fine (clods, pebbles) and coarse (undulations).
/// </summary>
public static class SoilNoise
{
    public const int N = 256;
    public const float FineLength = 2.56f;     // m per repeat: 1 cm texels, features 15-20 cm
    public const float CoarseLength = 9.47f;   // m per repeat: 3.7 cm texels, features 0.3-1 m
    /// <summary>Scales all relief (0 = perfectly smooth ground). Set before the soil grid is built (SoilWorld does).</summary>
    public static float Scale = 1f;
    static float[] map;
    static Texture2D tex;

    public static float[] Map
    {
        get
        {
            if (map != null) return map;
            var rnd = new System.Random(20270531);
            var a = new float[N * N];
            var b = new float[N * N];
            for (int i = 0; i < a.Length; i++) a[i] = (float)rnd.NextDouble() * 2f - 1f;
            // Blurred white noise, no sharp grit: bumps shorter than a tyre's contact patch are swallowed by the rubber
            // in reality, but would make a wheel hop here (1.5 mm bumps 3 cm apart at 1 m/s need > 2 g to follow).
            // Fine visual grain is the terrain layers' normal maps' job.
            var lo = Blur(a, b, 3, 3);
            map = new float[N * N];
            float peak = 0;
            for (int i = 0; i < map.Length; i++) peak = Mathf.Max(peak, Mathf.Abs(lo[i]));
            for (int i = 0; i < map.Length; i++) map[i] = lo[i] / peak;
            return map;
        }
    }

    static float[] Blur(float[] src, float[] tmp, int radius, int passes)
    {
        float[] a = src, b = tmp;
        for (int p = 0; p < passes; p++)
        {
            for (int y = 0; y < N; y++) for (int x = 0; x < N; x++)
            {
                float s = 0;
                for (int k = -radius; k <= radius; k++) s += a[y * N + ((x + k) & (N - 1))];
                b[y * N + x] = s;
            }
            for (int y = 0; y < N; y++) for (int x = 0; x < N; x++)
            {
                float s = 0;
                for (int k = -radius; k <= radius; k++) s += b[((y + k) & (N - 1)) * N + x];
                a[y * N + x] = s;
            }
        }
        return a;
    }

    /// <summary>Bilinear, wrapping; u,v in repeats. Matches a Repeat/Bilinear texture lookup.</summary>
    public static float Sample(float u, float v)
    {
        var m = Map;
        float fx = u * N - 0.5f, fy = v * N - 0.5f;
        int x0 = Mathf.FloorToInt(fx), y0 = Mathf.FloorToInt(fy);
        float tx = fx - x0, ty = fy - y0;
        int x1 = (x0 + 1) & (N - 1), y1 = (y0 + 1) & (N - 1);
        x0 &= N - 1; y0 &= N - 1;
        float a = m[y0 * N + x0], b = m[y0 * N + x1], c = m[y1 * N + x0], d = m[y1 * N + x1];
        return (a + (b - a) * tx) * (1 - ty) + (c + (d - c) * tx) * ty;
    }

    public static float Relief(float x, float z, float fine, float coarse)
    {
        float r = 0;
        if (fine > 0) r += fine * Sample(x / FineLength, z / FineLength);
        if (coarse > 0) r += coarse * Sample(x / CoarseLength + 0.37f, z / CoarseLength + 0.11f);
        return r * Scale;
    }

    public static Texture2D Texture
    {
        get
        {
            if (tex != null) return tex;
            tex = new Texture2D(N, N, TextureFormat.RFloat, false, true) { name = "SoilNoise", wrapMode = TextureWrapMode.Repeat, filterMode = FilterMode.Bilinear, hideFlags = HideFlags.HideAndDontSave };
            tex.SetPixelData(Map, 0);
            tex.Apply(false, true);
            return tex;
        }
    }
}

/// <summary>
/// The deformable soil surface: a fine height grid stored as sparse tiles (only where a wheel has been), on top of an
/// <see cref="ISoilBase"/>. Per cell: the undisturbed height (base + relief, cached), how far it has been pushed down or
/// piled up, the highest pressure it has carried (soil that was pressed once is firmer the second time: wheels
/// following in a rut sink less), and a "disturbed" mark for the shader.
/// Cell (ix, iz) is centred on ((ix + 0.5) cell, (iz + 0.5) cell) in world x/z.
/// </summary>
public class SoilField
{
    public const int TileShift = 6, TileSize = 1 << TileShift, TileMask = TileSize - 1;

    public class Tile
    {
        public int tx, tz;
        public float[] b = new float[TileSize * TileSize];     // undisturbed height, m (world)
        public float[] d = new float[TileSize * TileSize];     // deformation, m (+ up)
        public float[] sy = new float[TileSize * TileSize];    // yield pressure, Pa
        public float[] mark = new float[TileSize * TileSize];  // 0..1 disturbed
        public bool gpuDirty, touched;
        public int lastUsed;
        public ISoilBase soil;
    }

    public readonly float cell;
    public int maxTiles = 6000;
    public int frame;
    public event Action<Tile> TileChanged;
    readonly Dictionary<long, Tile> tiles = new Dictionary<long, Tile>();
    readonly Func<float, float, ISoilBase> findBase;
    Tile lastTile; long lastKey = long.MinValue;

    public SoilField(float cellSize, Func<float, float, ISoilBase> findBase) { cell = cellSize; this.findBase = findBase; }

    public int TileCount => tiles.Count;
    public IEnumerable<Tile> Tiles => tiles.Values;
    static long Key(int tx, int tz) => ((long)tx << 32) ^ (uint)tz;

    public Tile Find(int tx, int tz)
    {
        long k = Key(tx, tz);
        if (k == lastKey) return lastTile;
        tiles.TryGetValue(k, out var t);
        if (t != null) { lastKey = k; lastTile = t; }
        return t;
    }

    Tile GetOrCreate(int tx, int tz)
    {
        var t = Find(tx, tz);
        if (t != null) { t.lastUsed = frame; return t; }
        float x0 = tx * TileSize * cell, z0 = tz * TileSize * cell;
        var soil = findBase(x0 + TileSize * cell * 0.5f, z0 + TileSize * cell * 0.5f);
        if (soil == null) return null;
        if (tiles.Count >= maxTiles) Evict();
        t = new Tile { tx = tx, tz = tz, soil = soil, lastUsed = frame };
        float crustKey = -1; GroundSpec spec = null; float crust = 0;
        for (int j = 0; j < TileSize; j++)
        {
            float z = z0 + (j + 0.5f) * cell;
            for (int i = 0; i < TileSize; i++)
            {
                float x = x0 + (i + 0.5f) * cell;
                soil.Roughness(x, z, out float rf, out float rc);
                t.b[j * TileSize + i] = soil.BaseHeight(x, z) + SoilNoise.Relief(x, z, rf, rc);
                if ((i & 15) == 0 && (j & 15) == 0)
                {
                    spec = soil.SpecAt(x, z);
                    crust = spec != null ? spec.crustStrength * 1000f : 0;
                    crustKey = crust;
                }
                t.sy[j * TileSize + i] = crust;
            }
        }
        tiles[Key(tx, tz)] = t;
        lastKey = Key(tx, tz); lastTile = t;
        return t;
    }

    void Evict()
    {
        var old = new List<Tile>(tiles.Values);
        old.Sort((a, b) => a.lastUsed.CompareTo(b.lastUsed));
        for (int i = 0; i < old.Count / 8; i++)
        {
            tiles.Remove(Key(old[i].tx, old[i].tz));
            old[i].d = null;   // marks it dead for the renderer
            TileChanged?.Invoke(old[i]);
        }
        lastKey = long.MinValue; lastTile = null;
    }

    public void Clear()
    {
        foreach (var t in tiles.Values) { t.d = null; TileChanged?.Invoke(t); }
        tiles.Clear(); lastKey = long.MinValue; lastTile = null;
    }

    /// <summary>A rectangular working copy of the grid around a wheel.</summary>
    public class Patch
    {
        public int ix0, iz0, nx, nz;
        public float[] b, h, sy, mark;     // h = b + d (current surface)
        public float[] h0, sy0, mark0;     // as fetched: only cells that changed are written back
        public bool valid, modified;
        public float maxH, minH;
        public void Ensure(int n)
        {
            if (b != null && b.Length >= n) return;
            b = new float[n]; h = new float[n]; sy = new float[n]; mark = new float[n];
            h0 = new float[n]; sy0 = new float[n]; mark0 = new float[n];
        }
    }

    public int CellIndex(float x) => Mathf.FloorToInt(x / cell);

    /// <summary>Copy cells [ix0, ix0+nx) x [iz0, iz0+nz) into the patch. False if any part has no soil under it.</summary>
    public bool Fetch(Patch p, int ix0, int iz0, int nx, int nz)
    {
        p.ix0 = ix0; p.iz0 = iz0; p.nx = nx; p.nz = nz; p.modified = false; p.valid = false;
        p.Ensure(nx * nz);
        float maxH = float.MinValue, minH = float.MaxValue;
        for (int j = 0; j < nz; j++)
        {
            int iz = iz0 + j, tz = iz >> TileShift, lz = iz & TileMask;
            int i = 0;
            while (i < nx)
            {
                int ix = ix0 + i, tx = ix >> TileShift, lx = ix & TileMask;
                int run = Mathf.Min(nx - i, TileSize - lx);
                var t = GetOrCreate(tx, tz);
                if (t == null) return false;
                int src = lz * TileSize + lx, dst = j * nx + i;
                Array.Copy(t.b, src, p.b, dst, run);
                Array.Copy(t.sy, src, p.sy, dst, run);
                Array.Copy(t.mark, src, p.mark, dst, run);
                for (int k = 0; k < run; k++)
                {
                    float h = t.b[src + k] + t.d[src + k];
                    p.h[dst + k] = h;
                    if (h > maxH) maxH = h;
                    if (h < minH) minH = h;
                }
                i += run;
            }
        }
        int total = nx * nz;
        Array.Copy(p.h, p.h0, total); Array.Copy(p.sy, p.sy0, total); Array.Copy(p.mark, p.mark0, total);
        p.maxH = maxH; p.minH = minH; p.valid = true;
        return true;
    }

    public void Store(Patch p)
    {
        if (!p.valid) return;
        for (int j = 0; j < p.nz; j++)
        {
            int iz = p.iz0 + j, tz = iz >> TileShift, lz = iz & TileMask;
            int i = 0;
            while (i < p.nx)
            {
                int ix = p.ix0 + i, tx = ix >> TileShift, lx = ix & TileMask;
                int run = Mathf.Min(p.nx - i, TileSize - lx);
                var t = Find(tx, tz);
                if (t != null)
                {
                    int dst = lz * TileSize + lx, src = j * p.nx + i;
                    bool changed = false;
                    for (int k = 0; k < run; k++)
                    {
                        int s = src + k;
                        if (p.sy[s] != p.sy0[s]) t.sy[dst + k] = p.sy[s];
                        if (p.h[s] != p.h0[s] || p.mark[s] != p.mark0[s]) { t.d[dst + k] = p.h[s] - p.b[s]; t.mark[dst + k] = p.mark[s]; changed = true; }
                    }
                    if (changed) { t.gpuDirty = true; t.touched = true; TileChanged?.Invoke(t); }
                }
                i += run;
            }
        }
    }

    /// <summary>Current surface height at a point (bilinear), or NaN where there is no soil. For cameras/tools, not the hot path.</summary>
    public float Height(float x, float z, Patch scratch)
    {
        float fx = x / cell - 0.5f, fz = z / cell - 0.5f;
        int ix = Mathf.FloorToInt(fx), iz = Mathf.FloorToInt(fz);
        if (!Fetch(scratch, ix, iz, 2, 2)) return float.NaN;
        float tx = fx - ix, tz = fz - iz;
        return (scratch.h[0] + (scratch.h[1] - scratch.h[0]) * tx) * (1 - tz) + (scratch.h[2] + (scratch.h[3] - scratch.h[2]) * tx) * tz;
    }
}
