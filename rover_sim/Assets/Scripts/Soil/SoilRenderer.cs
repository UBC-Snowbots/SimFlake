using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Shows the soil: keeps a window of the <see cref="SoilField"/> around a target (the rover) in a texture and gives the
/// soil terrains a shader that displaces (tessellated presets) and shades the ground with it, plus the same small-scale
/// relief the wheels feel. Sits next to <see cref="SoilWorld"/>. Does nothing without a graphics device (headless).
///
/// The window texture wraps around (cell (ix, iz) lives at texel (ix mod N, iz mod N)), so moving the window only
/// uploads the tiles that came into view. Ruts outside the window stay in the SoilField and reappear when you return.
/// </summary>
[DefaultExecutionOrder(-150)]
public class SoilRenderer : MonoBehaviour
{
    [Tooltip("Window follows this (default: the first DriveWheel's robot, else the main camera)")]
    public Transform target;
    [Tooltip("m: beyond this distance from the camera the ground is not tessellated")]
    public float tessellationDistance = 22f;
    public int maxUploadsPerFrame = 48;

    SoilWorld world;
    Texture2D window, staging;
    float[] buf;
    int n, version = -1, tilesPerSide;
    long[] slotKey;                       // which tile each window slot shows (long.MinValue = empty)
    int cx = int.MinValue, cz;            // window centre, in tiles
    readonly HashSet<SoilField.Tile> dirty = new HashSet<SoilField.Tile>();
    readonly List<SoilField.Tile> work = new List<SoilField.Tile>();
    readonly Dictionary<Terrain, (Material mat, bool instanced)> original = new Dictionary<Terrain, (Material, bool)>();
    Material matPlain, matTess;
    SoilField hooked;
    bool noGraphics;

    static readonly int SoilTex = Shader.PropertyToID("_SoilTex"), SoilWindow = Shader.PropertyToID("_SoilWindow"),
        SoilParams = Shader.PropertyToID("_SoilParams"), NoiseTex = Shader.PropertyToID("_SoilNoiseTex"),
        NoiseScale = Shader.PropertyToID("_SoilNoiseScale"), ReliefTex = Shader.PropertyToID("_SoilReliefTex"),
        TerrainRect = Shader.PropertyToID("_SoilTerrainRect");

    void OnEnable()
    {
        world = GetComponent<SoilWorld>();
        noGraphics = SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null;
        Shader.SetGlobalVector(SoilParams, Vector4.zero);
    }

    void OnDisable()
    {
        Unhook();
        Restore();
        Shader.SetGlobalVector(SoilParams, Vector4.zero);
        if (window != null) Destroy(window);
        if (staging != null) Destroy(staging);
        window = staging = null; version = -1;
    }

    void Unhook() { if (hooked != null) hooked.TileChanged -= OnTile; hooked = null; }
    void OnTile(SoilField.Tile t) { dirty.Add(t); }

    void Restore()
    {
        foreach (var kv in original)
            if (kv.Key != null) { kv.Key.materialTemplate = kv.Value.mat; kv.Key.drawInstanced = kv.Value.instanced; kv.Key.SetSplatMaterialPropertyBlock(null); }
        original.Clear();
    }

    void Rebuild()
    {
        version = world.Version;
        Unhook();
        dirty.Clear();
        Restore();
        if (!world.Active) { Shader.SetGlobalVector(SoilParams, Vector4.zero); return; }
        var p = world.Current;
        n = p.window;
        tilesPerSide = n / SoilField.TileSize;
        if (window == null || window.width != n)
        {
            if (window != null) Destroy(window);
            window = new Texture2D(n, n, TextureFormat.RGFloat, false, true) { name = "SoilWindow", wrapMode = TextureWrapMode.Repeat, filterMode = FilterMode.Bilinear };
        }
        var clear = new float[n * n * 2];
        window.SetPixelData(clear, 0);
        window.Apply(false);
        if (staging == null)
        {
            staging = new Texture2D(SoilField.TileSize, SoilField.TileSize, TextureFormat.RGFloat, false, true) { name = "SoilTileUpload" };
            buf = new float[SoilField.TileSize * SoilField.TileSize * 2];
        }
        slotKey = new long[tilesPerSide * tilesPerSide];
        for (int i = 0; i < slotKey.Length; i++) slotKey[i] = long.MinValue;
        cx = int.MinValue;
        hooked = world.Field;
        hooked.TileChanged += OnTile;

        if (matPlain == null)
        {
            // loaded from Resources so player builds include the shaders (Shader.Find fails for unreferenced shaders)
            matPlain = Resources.Load<Material>("SoilTerrain"); matTess = Resources.Load<Material>("SoilTerrainTess");
            if (matPlain == null || matTess == null) { Debug.LogError("[Soil] Resources/SoilTerrain(.Tess).mat missing: run SimFlake > Soil > Create Utah Ground Specs"); enabled = false; return; }
        }
        bool tess = p.tessellation > 0 && SystemInfo.supportsTessellationShaders;
        foreach (var b in world.Bases)
        {
            if (!(b is SoilTerrain st) || st.Terrain == null) continue;
            st.Init();
            var t = st.Terrain;
            original[t] = (t.materialTemplate, t.drawInstanced);
            t.drawInstanced = false;
            t.materialTemplate = tess ? matTess : matPlain;
            var block = new MaterialPropertyBlock();
            Vector3 pos = t.transform.position, size = t.terrainData.size;
            block.SetVector(TerrainRect, new Vector4(pos.x, pos.z, 1f / size.x, 1f / size.z));
            if (st.ReliefTexture != null) block.SetTexture(ReliefTex, st.ReliefTexture);
            t.SetSplatMaterialPropertyBlock(block);
        }
        Shader.SetGlobalTexture(SoilTex, window);
        Shader.SetGlobalTexture(NoiseTex, SoilNoise.Texture);
        Shader.SetGlobalVector(NoiseScale, new Vector4(1f / SoilNoise.FineLength, 1f / SoilNoise.CoarseLength, SoilNoise.Scale, 0));
        Shader.SetGlobalVector(SoilParams, new Vector4(p.cell, Mathf.Max(p.tessellation, 1), tessellationDistance, 1));
    }

    void LateUpdate()
    {
        if (world == null || noGraphics) return;
        if (world.Version != version || (world.Active && original.Count != CountTerrains())) Rebuild();
        if (!world.Active || window == null) return;
        if (target == null)
        {
            // the rover's moving body (the prefab root itself stays where it was placed)
            var dw = FindObjectOfType<DriveWheel>();
            if (dw != null) foreach (var b in dw.transform.root.GetComponentsInChildren<ArticulationBody>()) if (b.isRoot) target = b.transform;
            if (target == null) target = Camera.main != null ? Camera.main.transform : null;
            if (target == null) return;
        }
        var field = world.Field;
        float tileM = SoilField.TileSize * field.cell;
        Vector3 f = Focus();
        int tcx = Mathf.RoundToInt(f.x / tileM), tcz = Mathf.RoundToInt(f.z / tileM);
        int half = tilesPerSide / 2, uploads = 0;
        if (tcx != cx || tcz != cz)
        {
            cx = tcx; cz = tcz;
            // slots whose tile changed: show the new tile (or flat ground)
            for (int j = 0; j < tilesPerSide; j++)
            for (int i = 0; i < tilesPerSide; i++)
            {
                int tx = cx - half + i, tz = cz - half + j;
                int slot = Mod(tz, tilesPerSide) * tilesPerSide + Mod(tx, tilesPerSide);
                long key = ((long)tx << 32) ^ (uint)tz;
                if (slotKey[slot] == key) continue;
                bool had = slotKey[slot] != long.MinValue && slotKey[slot] != long.MinValue + 1;
                var tile = field.Find(tx, tz);
                if (tile != null && tile.touched) { Upload(tile); slotKey[slot] = key; }
                else { if (had) UploadEmpty(tx, tz); slotKey[slot] = long.MinValue + 1; }
            }
            Shader.SetGlobalVector(SoilWindow, new Vector4(cx * tileM, cz * tileM, 1f / (n * field.cell), (half - 0.5f) * tileM));
        }
        if (dirty.Count > 0)
        {
            work.Clear(); work.AddRange(dirty);
            foreach (var t in work)
            {
                if (uploads >= maxUploadsPerFrame) break;
                dirty.Remove(t);
                if (t.tx < cx - half || t.tx >= cx - half + tilesPerSide || t.tz < cz - half || t.tz >= cz - half + tilesPerSide) continue;
                int slot = Mod(t.tz, tilesPerSide) * tilesPerSide + Mod(t.tx, tilesPerSide);
                if (t.d == null) { UploadEmpty(t.tx, t.tz); slotKey[slot] = long.MinValue + 1; }
                else { Upload(t); slotKey[slot] = ((long)t.tx << 32) ^ (uint)t.tz; }
                uploads++;
            }
        }
    }

    int CountTerrains() { int c = 0; foreach (var b in world.Bases) if (b is SoilTerrain) c++; return c; }
    static int Mod(int a, int m) { int r = a % m; return r < 0 ? r + m : r; }

    Vector3 Focus() => target.position;

    void Upload(SoilField.Tile t)
    {
        int m = SoilField.TileSize * SoilField.TileSize;
        for (int i = 0; i < m; i++) { buf[2 * i] = t.d[i]; buf[2 * i + 1] = t.mark[i]; }
        t.gpuDirty = false;
        Push(t.tx, t.tz);
    }

    void UploadEmpty(int tx, int tz)
    {
        System.Array.Clear(buf, 0, buf.Length);
        Push(tx, tz);
    }

    void Push(int tx, int tz)
    {
        staging.SetPixelData(buf, 0);
        staging.Apply(false);
        Graphics.CopyTexture(staging, 0, 0, 0, 0, SoilField.TileSize, SoilField.TileSize, window, 0, 0,
            Mod(tx, tilesPerSide) * SoilField.TileSize, Mod(tz, tilesPerSide) * SoilField.TileSize);
    }
}
