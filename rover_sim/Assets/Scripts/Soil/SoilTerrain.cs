using UnityEngine;

/// <summary>
/// Makes a Unity Terrain soil: wheels with a <see cref="DriveWheel"/> stop using its PhysX collider and run the
/// wheel-soil model on it instead (everything else still collides with the terrain as usual).
/// Each terrain layer gets a <see cref="GroundSpec"/> (same order as the terrain's layers); the layer with the most
/// weight at a spot decides the soil there. Needs a <see cref="SoilWorld"/> in the scene.
/// </summary>
[RequireComponent(typeof(Terrain))]
public class SoilTerrain : MonoBehaviour, ISoilBase
{
    [Tooltip("One per terrain layer, in the terrain's layer order")]
    public GroundSpec[] layerSpecs;
    [Tooltip("Used for layers without a spec")]
    public GroundSpec defaultSpec;

    public Terrain Terrain { get; private set; }
    public TerrainCollider Collider { get; private set; }
    /// <summary>RG = fine / coarse relief amplitude (m) over the terrain, for the shader.</summary>
    public Texture2D ReliefTexture { get; private set; }

    Vector3 pos, size;
    int res, aw, ah;
    float[] heights;        // world m, [z * res + x]
    byte[] dominant;        // layer index per alphamap texel
    float[] ampF, ampC;     // per alphamap texel
    bool ready;

    void OnEnable()
    {
        Terrain = GetComponent<Terrain>();
        Collider = GetComponent<TerrainCollider>();
        if (SoilWorld.Instance != null) SoilWorld.Instance.Register(this);
    }

    void Start() { if (SoilWorld.Instance != null) SoilWorld.Instance.Register(this); }
    void OnDisable() { if (SoilWorld.Instance != null) SoilWorld.Instance.Unregister(this); }

    public void Init()
    {
        if (ready) return;
        ready = true;
        if (Terrain == null) Terrain = GetComponent<Terrain>();
        var td = Terrain.terrainData;
        pos = transform.position; size = td.size; res = td.heightmapResolution;
        var h = td.GetHeights(0, 0, res, res);
        heights = new float[res * res];
        for (int z = 0; z < res; z++) for (int x = 0; x < res; x++) heights[z * res + x] = pos.y + h[z, x] * size.y;

        // Relief amplitudes and soil per point, from the splat weights. Kept at <= 1024 texels per side; the shader
        // gets the same grid as a texture so what you see is what the wheels feel.
        int sw = td.alphamapWidth, sh = td.alphamapHeight, layers = td.alphamapLayers;
        aw = Mathf.Min(sw, 1024); ah = Mathf.Min(sh, 1024);
        dominant = new byte[aw * ah]; ampF = new float[aw * ah]; ampC = new float[aw * ah];
        var rgba = new float[aw * ah * 4];
        for (int z = 0; z < ah; z++)
        {
            int sz = ah > 1 ? Mathf.RoundToInt(z * (sh - 1f) / (ah - 1f)) : 0;
            var a = td.GetAlphamaps(0, sz, sw, 1);
            for (int x = 0; x < aw; x++)
            {
                int sx = aw > 1 ? Mathf.RoundToInt(x * (sw - 1f) / (aw - 1f)) : 0;
                int best = 0; float bw = -1, f = 0, c = 0, dk = 0, wsum = 0;
                for (int l = 0; l < layers; l++)
                {
                    float w = a[0, sx, l];
                    if (w > bw) { bw = w; best = l; }
                    var s = Spec(l);
                    if (s != null) { f += w * s.roughnessFine; c += w * s.roughnessCoarse; dk += w * s.disturbedDarkening; wsum += w; }
                }
                if (layers == 0 && defaultSpec != null) { f = defaultSpec.roughnessFine; c = defaultSpec.roughnessCoarse; dk = defaultSpec.disturbedDarkening; wsum = 1; }
                if (wsum > 1e-4f) { f /= wsum; c /= wsum; dk /= wsum; }
                int k = z * aw + x;
                dominant[k] = (byte)best; ampF[k] = f; ampC[k] = c;
                rgba[k * 4] = f; rgba[k * 4 + 1] = c; rgba[k * 4 + 2] = dk; rgba[k * 4 + 3] = 1;
            }
        }
        if (SystemInfo.graphicsDeviceType != UnityEngine.Rendering.GraphicsDeviceType.Null)
        {
            if (ReliefTexture != null) Destroy(ReliefTexture);
            ReliefTexture = new Texture2D(aw, ah, TextureFormat.RGBAFloat, false, true) { name = "SoilRelief", wrapMode = TextureWrapMode.Clamp, filterMode = FilterMode.Bilinear, hideFlags = HideFlags.HideAndDontSave };
            ReliefTexture.SetPixelData(rgba, 0);
            ReliefTexture.Apply(false, true);
        }
    }

    /// <summary>Re-read the terrain (heights, layers, specs) on next use.</summary>
    public void Reinit() { ready = false; }

    GroundSpec Spec(int layer) => layerSpecs != null && layer < layerSpecs.Length && layerSpecs[layer] != null ? layerSpecs[layer] : defaultSpec;

    public bool Contains(float x, float z)
    {
        if (!ready) Init();
        return x >= pos.x && z >= pos.z && x <= pos.x + size.x && z <= pos.z + size.z;
    }

    // Same triangle interpolation as the terrain mesh and its collider (diagonal from (0,0) to (1,1)).
    public float BaseHeight(float x, float z)
    {
        if (!ready) Init();
        float fx = Mathf.Clamp((x - pos.x) / size.x * (res - 1), 0, res - 1.001f), fz = Mathf.Clamp((z - pos.z) / size.z * (res - 1), 0, res - 1.001f);
        int ix = (int)fx, iz = (int)fz;
        float u = fx - ix, v = fz - iz;
        int k = iz * res + ix;
        float h00 = heights[k], h10 = heights[k + 1], h01 = heights[k + res], h11 = heights[k + res + 1];
        return u > v ? h00 + (h10 - h00) * u + (h11 - h10) * v : h00 + (h11 - h01) * u + (h01 - h00) * v;
    }

    public GroundSpec SpecAt(float x, float z)
    {
        if (!ready) Init();
        int ix = Mathf.Clamp(Mathf.RoundToInt((x - pos.x) / size.x * (aw - 1)), 0, aw - 1), iz = Mathf.Clamp(Mathf.RoundToInt((z - pos.z) / size.z * (ah - 1)), 0, ah - 1);
        return Spec(dominant[iz * aw + ix]);
    }

    public void Roughness(float x, float z, out float fine, out float coarse)
    {
        if (!ready) Init();
        float fx = Mathf.Clamp((x - pos.x) / size.x * (aw - 1), 0, aw - 1.001f), fz = Mathf.Clamp((z - pos.z) / size.z * (ah - 1), 0, ah - 1.001f);
        int ix = (int)fx, iz = (int)fz;
        float u = fx - ix, v = fz - iz;
        int k = iz * aw + ix;
        fine = Mathf.Lerp(Mathf.Lerp(ampF[k], ampF[k + 1], u), Mathf.Lerp(ampF[k + aw], ampF[k + aw + 1], u), v);
        coarse = Mathf.Lerp(Mathf.Lerp(ampC[k], ampC[k + 1], u), Mathf.Lerp(ampC[k + aw], ampC[k + aw + 1], u), v);
    }
}
