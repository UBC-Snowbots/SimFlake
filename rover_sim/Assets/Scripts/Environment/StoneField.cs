using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Stones on and in the ground, everywhere, without tens of thousands of objects in the scene: they exist only around
/// the rovers (and the main camera) and are the same every run. Each patch of ground works out its stones from its
/// position and the seed, so driving away and back finds them where they were; loose stones that were pushed stay
/// where they ended up.
///
/// How many and how big: the rock size-frequency law used for Mars landing sites (Golombek and Rapp 1997; Golombek et
/// al. 2021): the fraction of ground covered by rocks of diameter D or more is F(D) = k exp(-q D), q = 1.79 + 0.152 / k,
/// where k is the rock abundance (2 % Phoenix, 3 % InSight, 5 % Spirit, 7 % Viking 1). k and the largest stone are set
/// per soil (<see cref="soils"/>); stones under <see cref="minDiameter"/> are left to the ground texture.
///
/// Two kinds: embedded stones (most of them: a fixed collider, part buried) and loose ones (a Rigidbody that wakes
/// when something touches it: wheels kick them, ride over them, or push them along).
/// Wheels meet stones with their PhysX sphere collider, like any rock (see CLAUDE.md, Soil model).
/// </summary>
public class StoneField : MonoBehaviour
{
    [Serializable]
    public class SoilStones
    {
        public GroundSpec soil;
        [Tooltip("Fraction of the ground covered by rocks (k). Mars sites: 0.02-0.07 plains, 0.2 rocky.")]
        public float rockAbundance = 0.03f;
        [Tooltip("Largest stone on this soil (m). Bigger rocks are the scene's boulders.")]
        public float maxDiameter = 0.15f;
        [Range(0, 1), Tooltip("Share of stones lying loose on the surface (the rest are embedded).")]
        public float looseFraction = 0.3f;
    }

    public SoilStones[] soils;
    [Tooltip("Stone meshes about 1 m across with a convex MeshCollider: small set for stones, big set from bigMeshFrom up.")]
    public GameObject[] stonePrefabs, rockPrefabs;
    public float minDiameter = 0.05f;
    [Tooltip("Stones up to this size appear within nearRadius; larger ones within farRadius (cameras and lidar see them from further).")]
    public float bigFrom = 0.15f;
    public float nearRadius = 9f, farRadius = 30f;
    public float bigMeshFrom = 0.3f;
    public int seed = 2027;
    [Tooltip("Upper limit of stones alive at once.")]
    public int maxStones = 3000;
    public float rockDensity = 2300f;
    [Tooltip("x, z, radius (m): no stones here (start lane, task points).")]
    public Vector3[] clearZones;
    [Tooltip("Also keep stones around the main camera (for looking around with the spectator camera).")]
    public bool aroundMainCamera = true;

    public int Alive { get; private set; }

    class Stone { public GameObject go; public Rigidbody rb; public int pool; public Vector3 home; }
    class Cell { public List<Stone> stones = new List<Stone>(); public bool big; public int cx, cz; }
    struct Moved { public Vector3 p; public Quaternion r; }

    readonly Dictionary<long, Cell> cells = new Dictionary<long, Cell>();
    readonly Dictionary<long, Dictionary<int, Moved>> moved = new Dictionary<long, Dictionary<int, Moved>>();
    readonly Dictionary<int, Stack<GameObject>> pools = new Dictionary<int, Stack<GameObject>>();
    readonly List<Vector3> focus = new List<Vector3>();
    readonly List<long> scratch = new List<long>();
    SoilTerrain[] terrains;
    MotorController[] rovers;
    float[] smallDensity, bigDensity;       // stones per m2, per soil
    float smallMax, bigMax;
    float[] stoneH, rockH;                  // mesh height for 1 m width
    float nextScan;
    const float SmallCell = 3f, BigCell = 10f;

    void Start()
    {
        // --no-stones: switch the stones off in a player (to compare), --stone-abundance 2: twice as many
        var args = new List<string>(Environment.GetCommandLineArgs());
        if (args.Contains("--no-stones")) { enabled = false; return; }
        int ai = args.IndexOf("--stone-abundance");
        if (ai >= 0 && ai + 1 < args.Count && float.TryParse(args[ai + 1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float mul))
            foreach (var so in soils) so.rockAbundance *= mul;
        terrains = FindObjectsOfType<SoilTerrain>();
        rovers = FindObjectsOfType<MotorController>();
        if (soils == null || soils.Length == 0 || stonePrefabs == null || stonePrefabs.Length == 0 || terrains.Length == 0) { enabled = false; return; }
        if (rockPrefabs == null || rockPrefabs.Length == 0) rockPrefabs = stonePrefabs;
        smallDensity = new float[soils.Length]; bigDensity = new float[soils.Length];
        for (int i = 0; i < soils.Length; i++)
        {
            smallDensity[i] = Count(soils[i].rockAbundance, minDiameter, Mathf.Min(bigFrom, soils[i].maxDiameter));
            bigDensity[i] = Count(soils[i].rockAbundance, bigFrom, soils[i].maxDiameter);
            smallMax = Mathf.Max(smallMax, smallDensity[i]); bigMax = Mathf.Max(bigMax, bigDensity[i]);
        }
        stoneH = Heights(stonePrefabs); rockH = Heights(rockPrefabs);
        Scan(true);
    }

    static float[] Heights(GameObject[] prefabs)
    {
        var h = new float[prefabs.Length];
        for (int i = 0; i < h.Length; i++) { var mf = prefabs[i].GetComponentInChildren<MeshFilter>(); h[i] = mf != null ? mf.sharedMesh.bounds.size.y : 0.5f; }
        return h;
    }

    static float Q(float k) => 1.79f + 0.152f / Mathf.Max(k, 1e-4f);

    /// <summary>Stones per m2 with diameter between d0 and d1 for rock abundance k: -dF/dD divided by one stone's area.</summary>
    public static float Count(float k, float d0, float d1)
    {
        if (d1 <= d0 || k <= 0) return 0;
        float q = Q(k), sum = 0; const int n = 64;
        for (int i = 0; i < n; i++)
        {
            float d = d0 + (d1 - d0) * (i + 0.5f) / n;
            sum += k * q * Mathf.Exp(-q * d) / (Mathf.PI * d * d / 4f);
        }
        return sum * (d1 - d0) / n;
    }

    /// <summary>Look for rovers again (one was swapped for another at runtime).</summary>
    public void FindRovers() => rovers = FindObjectsOfType<MotorController>();

    void Update()
    {
        if (Time.time < nextScan) return;
        nextScan = Time.time + 0.2f;
        Scan(false);
    }

    void Scan(bool all)
    {
        focus.Clear();
        foreach (var r in rovers) if (r != null && r.isActiveAndEnabled) focus.Add(RoverPosition(r));
        if (aroundMainCamera && Camera.main != null) focus.Add(Camera.main.transform.position);
        // drop patches nobody is near (a little further out than where they appear, so they don't flicker)
        scratch.Clear();
        foreach (var kv in cells) if (!Near(kv.Value, 1.25f)) scratch.Add(kv.Key);
        foreach (long key in scratch) Release(key);
        int budget = all ? int.MaxValue : 250;      // stones per scan, so a fast drive doesn't hitch
        foreach (var f in focus)
        {
            budget = Fill(f, false, budget);
            budget = Fill(f, true, budget);
        }
    }

    static Vector3 RoverPosition(MotorController r)
    {
        foreach (var b in r.GetComponentsInChildren<ArticulationBody>()) if (b.isRoot) return b.transform.position;
        return r.transform.position;
    }

    bool Near(Cell c, float factor)
    {
        float size = c.big ? BigCell : SmallCell, radius = (c.big ? farRadius : nearRadius) * factor + size;
        var p = new Vector2((c.cx + 0.5f) * size, (c.cz + 0.5f) * size);
        foreach (var f in focus) if ((new Vector2(f.x, f.z) - p).sqrMagnitude < radius * radius) return true;
        return false;
    }

    static long Key(bool big, int cx, int cz) => ((long)(big ? 1 : 0) << 62) | ((long)(cx & 0x3FFFFFFF) << 30) | (long)(cz & 0x3FFFFFFF);

    int Fill(Vector3 f, bool big, int budget)
    {
        float size = big ? BigCell : SmallCell, radius = big ? farRadius : nearRadius;
        if ((big ? bigMax : smallMax) <= 0) return budget;
        int x0 = Mathf.FloorToInt((f.x - radius) / size), x1 = Mathf.FloorToInt((f.x + radius) / size);
        int z0 = Mathf.FloorToInt((f.z - radius) / size), z1 = Mathf.FloorToInt((f.z + radius) / size);
        for (int cz = z0; cz <= z1 && budget > 0; cz++)
        for (int cx = x0; cx <= x1 && budget > 0; cx++)
        {
            float dx = (cx + 0.5f) * size - f.x, dz = (cz + 0.5f) * size - f.z;
            if (dx * dx + dz * dz > radius * radius) continue;
            long key = Key(big, cx, cz);
            if (cells.ContainsKey(key)) continue;
            budget -= Build(key, big, cx, cz);
        }
        return budget;
    }

    SoilTerrain TerrainAt(float x, float z)
    {
        foreach (var t in terrains) if (t != null && t.Contains(x, z)) return t;
        return null;
    }

    int SoilIndex(GroundSpec s)
    {
        for (int i = 0; i < soils.Length; i++) if (soils[i].soil == s) return i;
        return -1;
    }

    /// <summary>Height of the ground as the wheels and the shader see it: terrain + the soil's small-scale relief.</summary>
    static float Surface(SoilTerrain t, float x, float z)
    {
        float h = t.BaseHeight(x, z);
        if (SoilWorld.Instance != null && SoilWorld.Instance.Active)
        {
            t.Roughness(x, z, out float fine, out float coarse);
            h += SoilNoise.Relief(x, z, fine, coarse);
        }
        return h;
    }

    int Build(long key, bool big, int cx, int cz)
    {
        var cell = new Cell { big = big, cx = cx, cz = cz };
        cells[key] = cell;
        float size = big ? BigCell : SmallCell, dMin = big ? bigFrom : minDiameter;
        float[] density = big ? bigDensity : smallDensity;
        float maxDensity = big ? bigMax : smallMax;
        var rnd = new System.Random(unchecked(seed * 73856093 ^ cx * 19349663 ^ cz * 83492791 ^ (big ? 0x5bd1e995 : 0)));
        float R() => (float)rnd.NextDouble();
        float expect = maxDensity * size * size;
        int candidates = (int)expect + (R() < expect - (int)expect ? 1 : 0);
        for (int i = 0; i < candidates; i++)
        {
            // every candidate uses the same number of random draws, so stone i is the same stone every run
            float x = (cx + R()) * size, z = (cz + R()) * size;
            float accept = R(), u = R(), keep = R(), loose = R(), yaw = R() * 360f, tiltA = R(), tiltB = R(), sinkR = R();
            int mesh = rnd.Next(1 << 20);
            var t = TerrainAt(x, z);
            if (t == null) continue;
            int si = SoilIndex(t.SpecAt(x, z));
            if (si < 0 || accept * maxDensity > density[si]) continue;
            var s = soils[si];
            float dMax = big ? s.maxDiameter : Mathf.Min(bigFrom, s.maxDiameter);
            // diameter: numbers fall as exp(-q D) / D^2. Draw from 1 / D^2, keep with exp(-q (D - dMin)).
            float d = 1f / (1f / dMin - u * (1f / dMin - 1f / dMax));
            if (keep > Mathf.Exp(-Q(s.rockAbundance) * (d - dMin))) continue;
            if (!Clear(x, z, d)) continue;
            if (Alive >= maxStones) break;

            float h0 = Surface(t, x, z);
            var n = new Vector3(Surface(t, x - 0.25f, z) - Surface(t, x + 0.25f, z), 0.5f, Surface(t, x, z - 0.25f) - Surface(t, x, z + 0.25f)).normalized;
            bool isLoose = loose < s.looseFraction && n.y > 0.87f;      // nothing lies loose on a slope over ~30 deg
            bool useBig = d >= bigMeshFrom;
            var set = useBig ? rockPrefabs : stonePrefabs;
            int mi = mesh % set.Length;
            float height = (useBig ? rockH : stoneH)[mi] * d;
            float sink = isLoose ? 0.05f + 0.1f * sinkR : 0.3f + 0.35f * sinkR;
            float tilt = isLoose ? 20f : 9f;
            var rot = Quaternion.FromToRotation(Vector3.up, n) * Quaternion.Euler((tiltA - 0.5f) * 2 * tilt, yaw, (tiltB - 0.5f) * 2 * tilt);
            var pos = new Vector3(x, h0, z) + n * (height * (0.5f - sink));

            var st = Take((useBig ? 2 : 0) + (isLoose ? 1 : 0), set[mi], mi);
            st.home = pos;
            st.go.transform.localScale = Vector3.one * d;
            if (st.rb != null)
            {
                st.rb.isKinematic = true;       // asleep on the surface until something touches it (LooseStone)
                st.rb.mass = Mathf.Max(0.05f, rockDensity * 0.5f * d * d * height);
            }
            st.go.transform.SetPositionAndRotation(pos, rot);
            st.go.name = isLoose ? "stone (loose)" : "stone";
            st.go.SetActive(true);
            cell.stones.Add(st);
            Alive++;
        }
        // loose stones that were pushed before this patch was dropped go back to where they ended up
        if (moved.TryGetValue(key, out var was))
            foreach (var m in was)
                if (m.Key < cell.stones.Count && cell.stones[m.Key].rb != null)
                    cell.stones[m.Key].go.transform.SetPositionAndRotation(m.Value.p, m.Value.r);
        return cell.stones.Count + 1;
    }

    bool Clear(float x, float z, float d)
    {
        if (clearZones == null) return true;
        foreach (var c in clearZones) { float r = c.z + d; if ((x - c.x) * (x - c.x) + (z - c.y) * (z - c.y) < r * r) return false; }
        return true;
    }

    // pools: one per (mesh set, loose or not, mesh), so a stone keeps its collider and never needs AddComponent later
    Stone Take(int kind, GameObject prefab, int mi)
    {
        int pool = kind * 1024 + mi;
        if (!pools.TryGetValue(pool, out var stack)) pools[pool] = stack = new Stack<GameObject>();
        GameObject go;
        if (stack.Count > 0) go = stack.Pop();
        else
        {
            go = Instantiate(prefab, transform);
            if ((kind & 1) == 1)
            {
                var rb = go.AddComponent<Rigidbody>();
                rb.isKinematic = true; rb.angularDrag = 0.6f; rb.drag = 0.05f;
                go.AddComponent<LooseStone>();
            }
        }
        return new Stone { go = go, rb = go.GetComponent<Rigidbody>(), pool = pool };
    }

    void Release(long key)
    {
        var cell = cells[key];
        cells.Remove(key);
        Dictionary<int, Moved> keep = null;
        for (int i = 0; i < cell.stones.Count; i++)
        {
            var st = cell.stones[i];
            if (st.rb != null && (st.go.transform.position - st.home).sqrMagnitude > 0.0009f)
            {
                // remember where a pushed stone ended up, by its place in this patch's list (rebuilt in the same order)
                if (keep == null) keep = new Dictionary<int, Moved>();
                keep[i] = new Moved { p = st.go.transform.position, r = st.go.transform.rotation };
            }
            st.go.SetActive(false);
            pools[st.pool].Push(st.go);
            Alive--;
        }
        if (keep != null) moved[key] = keep; else moved.Remove(key);
    }
}
