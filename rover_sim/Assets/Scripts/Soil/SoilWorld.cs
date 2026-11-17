using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

public enum SoilQuality { Off, Low, Medium, High, Ultra }

/// <summary>
/// One per scene: owns the deformable soil grid (<see cref="SoilField"/>) and the realism/performance preset.
/// Terrains join with a <see cref="SoilTerrain"/> component; wheels (<see cref="DriveWheel"/>) find it through
/// <see cref="Instance"/>. Without a SoilWorld (or with quality Off) wheels use plain PhysX contacts.
///
/// Quality: inspector, F9 in Play (cycles), SimFlake > Soil menu, or <c>--soil-quality off|low|medium|high|ultra</c>.
/// Changing it rebuilds the grid (ruts are cleared).
/// </summary>
[DefaultExecutionOrder(-200)]
public class SoilWorld : MonoBehaviour
{
    [Serializable]
    public struct Preset
    {
        public float cell;            // m, soil grid cell
        public int nTheta, nY;        // tyre contact elements around / across
        public int window;            // rendered soil window, cells per side (power of two)
        public float tessellation;    // target triangle edge on screen, pixels (0 = no tessellation: ruts are shading only)
        public int relaxPasses;
        public bool sideForces;
        public float fixedDt;         // physics step, 0 = project setting
    }

    public static Preset Get(SoilQuality q)
    {
        switch (q)
        {
            case SoilQuality.Low:    return new Preset { cell = 0.025f, nTheta = 60,  nY = 3,  window = 512,  tessellation = 0,  relaxPasses = 1, sideForces = false };
            case SoilQuality.Medium: return new Preset { cell = 0.015f, nTheta = 90,  nY = 4,  window = 1024, tessellation = 20, relaxPasses = 1, sideForces = true };
            case SoilQuality.Ultra:  return new Preset { cell = 0.005f, nTheta = 360, nY = 32, window = 2048, tessellation = 6,  relaxPasses = 2, sideForces = true, fixedDt = 0.0025f };
            default:                 return new Preset { cell = 0.01f,  nTheta = 120, nY = 8,  window = 2048, tessellation = 10, relaxPasses = 2, sideForces = true };
        }
    }

    public SoilQuality quality = SoilQuality.High;
    [Tooltip("Ruts on/off (off: the soil still gives way under load, but springs back).")]
    public bool ruts = true;
    [Tooltip("Limit on contact stiffness = factor x wheel mass / dt². Higher = stiffer tyres on hard ground; too high and wheels buzz.")]
    public float stabilityFactor = 1.0f;
    [Tooltip("Scales the small-scale relief of all soils (0 = smooth ground)")]
    [Range(0, 2)] public float relief = 1f;
    [Tooltip("Compute the wheels' soil contacts on several CPU cores")]
    public bool parallel = true;
    public KeyCode cycleKey = KeyCode.F9;

    /// <summary>Time spent in the soil model (Stopwatch ticks) and physics steps counted, for profiling.</summary>
    public static long SoilTicks, SoilSteps;

    public static SoilWorld Instance { get; private set; }
    public SoilField Field { get; private set; }
    public Preset Current { get; private set; }
    /// <summary>Goes up whenever the grid is rebuilt (quality change): wheels and the renderer re-read their settings.</summary>
    public int Version { get; private set; }
    public bool Active => quality != SoilQuality.Off && Field != null;
    public event Action Rebuilt;

    readonly List<ISoilBase> bases = new List<ISoilBase>();
    ISoilBase lastBase;
    float defaultFixedDt;

    void Awake()
    {
        Instance = this;
        defaultFixedDt = Time.fixedDeltaTime;
        var args = Environment.GetCommandLineArgs();
        int i = Array.IndexOf(args, "--soil-quality");
        if (i >= 0 && i + 1 < args.Length && Enum.TryParse(args[i + 1], true, out SoilQuality q)) quality = q;
        if (Array.IndexOf(args, "--no-ruts") >= 0) ruts = false;
        if (Array.IndexOf(args, "--soil-serial") >= 0) parallel = false;
        i = Array.IndexOf(args, "--soil-stability");
        if (i >= 0 && i + 1 < args.Length && float.TryParse(args[i + 1], NumberStyles.Float, CultureInfo.InvariantCulture, out float sf)) stabilityFactor = sf;
        i = Array.IndexOf(args, "--soil-relief");
        if (i >= 0 && i + 1 < args.Length && float.TryParse(args[i + 1], NumberStyles.Float, CultureInfo.InvariantCulture, out float rl)) relief = rl;
        Build();
    }

    void OnDestroy() { if (Instance == this) Instance = null; }

    void Update()
    {
        if (Input.GetKeyDown(cycleKey)) SetQuality((SoilQuality)(((int)quality + 1) % 5));
        if (Field != null) Field.frame = Time.frameCount;
    }

    public void SetQuality(SoilQuality q)
    {
        if (q == quality && Field != null) return;
        quality = q;
        Build();
        Debug.Log($"[Soil] quality {quality}" + (Active ? $": {Current.cell * 100:0.#} cm cells, {Current.nTheta}x{Current.nY} tyre elements" : ""));
    }

    void Build()
    {
        Field?.Clear();
        SoilNoise.Scale = relief;
        Current = Get(quality);
        Field = quality == SoilQuality.Off ? null : new SoilField(Current.cell, FindBase);
        if (Application.isPlaying) Time.fixedDeltaTime = Current.fixedDt > 0 ? Mathf.Min(Current.fixedDt, defaultFixedDt) : defaultFixedDt;
        Version++;
        Rebuilt?.Invoke();
    }

    /// <summary>Forget all ruts (the grid is rebuilt from the terrains).</summary>
    public void ClearRuts() { Build(); }

    public WheelSoilModel.Settings WheelSettings => new WheelSoilModel.Settings
    {
        nTheta = Current.nTheta, nY = Current.nY, stabilityFactor = stabilityFactor, deform = ruts,
        sideForces = Current.sideForces, relaxPasses = Current.relaxPasses,
    };

    readonly List<DriveWheel> wheels = new List<DriveWheel>();
    bool[] begun = new bool[8];
    public void Register(DriveWheel w) { if (!wheels.Contains(w)) wheels.Add(w); }
    public int TerrainCount { get { int c = 0; foreach (var b in bases) if (b is SoilTerrain) c++; return c; } }

    // All wheels' soil contacts for this physics step, before the DriveWheels' own FixedUpdate (execution order).
    void FixedUpdate()
    {
        if (!Active || wheels.Count == 0) return;
        long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
        wheels.RemoveAll(w => w == null);
        int n = wheels.Count, work = 0;
        if (begun.Length < n) begun = new bool[n];
        for (int i = 0; i < n; i++) { begun[i] = wheels[i].SoilBegin(); if (begun[i]) work++; }
        float dt = Time.fixedDeltaTime;
        if (parallel && work > 1)
            System.Threading.Tasks.Parallel.For(0, n, i => { if (begun[i]) wheels[i].SoilCompute(dt); });
        else
            for (int i = 0; i < n; i++) if (begun[i]) wheels[i].SoilCompute(dt);
        for (int i = 0; i < n; i++) wheels[i].SoilEnd();
        SoilTicks += System.Diagnostics.Stopwatch.GetTimestamp() - t0; SoilSteps++;
    }

    public void Register(ISoilBase b) { if (!bases.Contains(b)) bases.Add(b); }
    public void Unregister(ISoilBase b) { bases.Remove(b); if (lastBase == b) lastBase = null; }
    public IReadOnlyList<ISoilBase> Bases => bases;

    public ISoilBase FindBase(float x, float z)
    {
        if (lastBase != null && lastBase.Contains(x, z)) return lastBase;
        for (int i = 0; i < bases.Count; i++)
            if (bases[i].Contains(x, z)) return lastBase = bases[i];
        return null;
    }

    /// <summary>Current soil surface height under a world point (ruts included), or NaN off the soil.</summary>
    public float SurfaceHeight(Vector3 p)
    {
        if (!Active) return float.NaN;
        return Field.Height(p.x, p.z, scratch);
    }
    readonly SoilField.Patch scratch = new SoilField.Patch();
}

/// <summary>A flat (or tilted) plane of one soil: for the wheel test rig and unit checks.</summary>
public class FlatSoil : ISoilBase
{
    public GroundSpec spec;
    public float height, slopeX, slopeZ;      // height at the origin, rise per metre along x and z
    public bool relief;
    public bool Contains(float x, float z) => true;
    public float BaseHeight(float x, float z) => height + slopeX * x + slopeZ * z;
    public GroundSpec SpecAt(float x, float z) => spec;
    public void Roughness(float x, float z, out float fine, out float coarse)
    {
        fine = relief && spec != null ? spec.roughnessFine : 0; coarse = relief && spec != null ? spec.roughnessCoarse : 0;
    }
}
