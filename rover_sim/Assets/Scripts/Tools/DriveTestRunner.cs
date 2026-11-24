using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;

/// <summary>
/// Drivetrain test bench (scene Assets/Scenes/Tests/DriveTest). Builds a flat test ground with 10/20/30 deg ramps at
/// runtime, then for every rover x ground material: accelerate + cruise, top speed, point turn, arc, command-timeout
/// stop, and slope climbs (with a hold on the slope). Results go to the Console as a table, plus JSON with
/// --drive-test-out. No ROS needed (it calls MotorController directly), and nothing is published.
///
/// Editor: open the scene and press Play (watch with F8 for the drivetrain HUD).
/// Headless (player built with SimFlakeTools.BuildLinux -scenes Assets/Scenes/Tests/DriveTest.unity):
///   rover_sim.x86_64 -batchmode -nographics --drive-test [--drive-test-out r.json] [--drive-test-timescale 20]
///                    [--drive-test-rovers chassis_2024,mini_rover] [--drive-test-grounds CustomDirt,Asphalt]
///                    [--drive-test-only accel,top,spin,arc,stop,slope] [--fixed-dt 0.005] [--turnslip 1.6]
/// Exit code 0 when all runs finished (the numbers are for people to judge; nothing is pass/fail except crashes).
/// </summary>
public class DriveTestRunner : MonoBehaviour
{
    [Serializable]
    public class RoverCase
    {
        public string label;
        public GameObject prefab;
        [Tooltip("m/s used for cruise, arc and slope tests")]
        public float testSpeed = 1f;
        [Tooltip("m up the slope that counts as 'climbed'")]
        public float climbDistance = 5f;
        public bool enabled = true;
        [Tooltip("kg added to the root body (e.g. the arm, ~15 kg on chassis_2024), at its centre of mass")]
        public float payloadKg = 0f;
    }

    public RoverCase[] rovers = new RoverCase[0];
    public PhysicMaterial[] grounds = new PhysicMaterial[0];
    [Tooltip("Soils for the wheel-soil model: each is run on a soil terrain (flat + 10/20/30 deg slopes in the heightmap)")]
    public GroundSpec[] soils = new GroundSpec[0];
    public float[] slopes = { 10, 20, 30 };
    [Tooltip("Speed-up in the editor (headless default is 20)")]
    public float timeScale = 1f;
    public bool followCamera = true;
    [Tooltip("Flat test ground: a TerrainCollider (heightfield, like the real worlds) or a box")]
    public bool terrainGround = true;

    readonly List<Dictionary<string, object>> results = new List<Dictionary<string, object>>();
    readonly List<GameObject> arena = new List<GameObject>();
    GameObject ground;
    Transform followTarget;
    Rover currentRover;
    bool headless;
    string outPath;
    HashSet<string> only;
    double turnSlipOverride = double.NaN;
    bool trace;
    float lastTrace;
    // Experiment switches (headless): --legacy-drive, --speed-gain <N·m per rad/s>, --no-rolling-resistance,
    // --solver-iterations <pos,vel>
    bool legacyDrive, noRolling;
    string wheelCollider;
    float speedGain = -1, bogieDamping = -1, contactOffset = -1, integralTime = -1;
    int solverPos = -1, solverVel = -1;
    const float RampLength = 30f, RampWidth = 8f;

    // ---------------------------------------------------------------- setup

    void Start()
    {
        var args = Environment.GetCommandLineArgs();
        string Arg(string n) { for (int i = 0; i < args.Length - 1; i++) if (args[i] == n) return args[i + 1]; return null; }
        headless = args.Contains("--drive-test");
        outPath = Arg("--drive-test-out");
        if (float.TryParse(Arg("--drive-test-timescale"), NumberStyles.Float, CultureInfo.InvariantCulture, out var ts)) timeScale = ts;
        else if (headless) timeScale = 20f;
        if (float.TryParse(Arg("--fixed-dt"), NumberStyles.Float, CultureInfo.InvariantCulture, out var fdt)) Time.fixedDeltaTime = fdt;
        if (double.TryParse(Arg("--turnslip"), NumberStyles.Float, CultureInfo.InvariantCulture, out var tsf)) turnSlipOverride = tsf;
        var roverFilter = Arg("--drive-test-rovers")?.Split(',');
        if (roverFilter != null) foreach (var r in rovers) r.enabled = roverFilter.Contains(r.label);
        var groundFilter = Arg("--drive-test-grounds")?.Split(',');
        if (groundFilter != null) grounds = grounds.Where(g => g != null && groundFilter.Contains(g.name)).ToArray();
        // --soils LooseSand,Hardpack (part of the spec name) or --soils none
        var soilFilter = Arg("--soils")?.Split(',');
        if (soilFilter != null) soils = soils.Where(g => g != null && soilFilter.Any(f => g.name.IndexOf(f, StringComparison.OrdinalIgnoreCase) >= 0)).ToArray();
        trace = args.Contains("--drive-test-trace");
        DriveWheel.DebugSettle = trace;
        legacyDrive = args.Contains("--legacy-drive");
        wheelCollider = Arg("--wheel-collider");
        if (Arg("--ground-shape") == "box") terrainGround = false;
        noRolling = args.Contains("--no-rolling-resistance");
        if (float.TryParse(Arg("--speed-gain"), NumberStyles.Float, CultureInfo.InvariantCulture, out var sg)) speedGain = sg;
        if (float.TryParse(Arg("--integral-time"), NumberStyles.Float, CultureInfo.InvariantCulture, out var it)) integralTime = it;
        if (float.TryParse(Arg("--contact-offset"), NumberStyles.Float, CultureInfo.InvariantCulture, out var co)) contactOffset = co;
        if (float.TryParse(Arg("--bogie-damping"), NumberStyles.Float, CultureInfo.InvariantCulture, out var bd)) bogieDamping = bd;
        var si = Arg("--solver-iterations")?.Split(',');
        if (si != null && si.Length == 2) { solverPos = int.Parse(si[0]); solverVel = int.Parse(si[1]); }
        var o = Arg("--drive-test-only");
        if (o != null) only = new HashSet<string>(o.Split(','));

        Application.targetFrameRate = -1;
        QualitySettings.vSyncCount = 0;
        Time.maximumDeltaTime = 1f;
        Time.timeScale = timeScale;
        BuildArena();
        StartCoroutine(RunAll());
    }

    bool Want(string test) => only == null || only.Contains(test);

    void BuildArena()
    {
        if (terrainGround)
        {
            var td = new TerrainData { heightmapResolution = 1025 };
            td.size = new Vector3(600, TerrainHeight, 600);
            ground = Terrain.CreateTerrainGameObject(td);
            ground.name = "Test Ground (terrain)";
            ground.transform.position = new Vector3(-300, 0, -300);
        }
        else
        {
            ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
            ground.name = "Test Ground (box)";
            ground.transform.localScale = new Vector3(600, 1, 600);
            ground.transform.position = new Vector3(0, -0.5f, 0);
        }
        arena.Add(ground);
        for (int i = 0; i < slopes.Length; i++)
        {
            var ramp = GameObject.CreatePrimitive(PrimitiveType.Cube);
            ramp.name = $"Ramp {slopes[i]:0} deg";
            float t = 0.5f;
            var rot = Quaternion.Euler(-slopes[i], 0, 0);
            ramp.transform.localScale = new Vector3(RampWidth, t, RampLength);
            ramp.transform.rotation = rot;
            Vector3 edge = RampStart(i);
            ramp.transform.position = edge - rot * new Vector3(0, t / 2, -RampLength / 2);
            arena.Add(ramp);
        }
    }

    Vector3 RampStart(int i) => new Vector3(60 + 20 * i, 0, 0);

    const float TerrainHeight = 20f;
    SoilWorld soilWorld;
    SoilTerrain soilTerrain;

    /// <summary>Turn the arena into a soil terrain of one soil: the box ramps go, the same slopes are cut into the heightmap.</summary>
    void SetSoil(GroundSpec spec)
    {
        var terrain = ground.GetComponent<Terrain>();
        if (terrain == null) { Debug.LogError("[DriveTest] soils need the terrain ground"); return; }
        if (soilWorld == null)
        {
            foreach (var g in arena) if (g != ground) g.SetActive(false);
            var td = terrain.terrainData;
            int res = td.heightmapResolution;
            var h = new float[res, res];
            float step = td.size.x / (res - 1);
            for (int zi = 0; zi < res; zi++)
            for (int xi = 0; xi < res; xi++)
            {
                float x = ground.transform.position.x + xi * step, z = ground.transform.position.z + zi * step;
                for (int i = 0; i < slopes.Length; i++)
                    if (Mathf.Abs(x - RampStart(i).x) <= RampWidth * 0.5f + 0.3f)
                        h[zi, xi] = Mathf.Tan(slopes[i] * Mathf.Deg2Rad) * Mathf.Clamp(z, 0, RampLength) / TerrainHeight;
            }
            td.SetHeights(0, 0, h);
            soilWorld = new GameObject("Soil").AddComponent<SoilWorld>();
            soilTerrain = ground.AddComponent<SoilTerrain>();
        }
        soilTerrain.defaultSpec = spec;
        soilTerrain.Reinit();
        soilWorld.ClearRuts();
    }

    void SetGround(PhysicMaterial m)
    {
        foreach (var g in arena) g.GetComponent<Collider>().sharedMaterial = m;
    }

    // ---------------------------------------------------------------- runner

    IEnumerator RunAll()
    {
        float wall0 = Time.realtimeSinceStartup;
        Debug.Log($"[DriveTest] fixed dt {Time.fixedDeltaTime * 1000:F1} ms, timescale {Time.timeScale}, ground {(terrainGround ? "terrain" : "box")}, " +
                  $"rovers {string.Join(",", rovers.Where(r => r.enabled).Select(r => r.label))}, grounds {string.Join(",", grounds.Select(g => g.name))}");
        foreach (var rc in rovers.Where(r => r.enabled && r.prefab != null))
            foreach (var g in grounds)
            {
                SetGround(g);
                yield return RunRover(rc, g);
            }
        foreach (var rc in rovers.Where(r => r.enabled && r.prefab != null))
            foreach (var soil in soils.Where(x => x != null))
            {
                SetSoil(soil);
                yield return null;
                Debug.Log($"[DriveTest] soil {soil.name}, quality {soilWorld.quality} ({soilWorld.Current.cell * 100:0.#} cm cells, {soilWorld.Current.nTheta}x{soilWorld.Current.nY} elements), dt {Time.fixedDeltaTime * 1000:F1} ms");
                yield return RunRover(rc, new PhysicMaterial(soil.name.Replace("Utah_", "")));
            }
        Debug.Log($"[DriveTest] done in {Time.realtimeSinceStartup - wall0:F0} s wall time");
        PrintTable();
        if (!string.IsNullOrEmpty(outPath))
        {
            File.WriteAllText(outPath, ToJson(results));
            Debug.Log("[DriveTest] wrote " + outPath);
        }
        if (headless) Application.Quit(0);
    }

    class Rover
    {
        public GameObject go;
        public ArticulationBody root;
        public MotorController mc;
        public DriveWheel[] wheels;
        public float restPitch;
        public Vector3 Pos => root.transform.position;
        public Vector3 Fwd => root.transform.forward;
        // From positions: ArticulationBody.velocity reads several % high with the TGS solver.
        public float Speed => Vector3.Dot(vel, Fwd);
        public float SpeedPhysx => Vector3.Dot(root.velocity, Fwd);
        public Vector3 vel; Vector3 lastPos; float lastT = -1;
        public void Sample()
        {
            float t = Time.fixedTime;
            if (lastT >= 0 && t > lastT) vel = (Pos - lastPos) / (t - lastT);
            lastPos = Pos; lastT = t;
        }
        public float YawRateCcw => -root.angularVelocity.y;          // rad/s, + = left, about world up
        public float YawCcw => -Mathf.Atan2(Fwd.x, Fwd.z);
    }

    IEnumerator Spawn(RoverCase rc, Vector3 pos, Action<Rover> done)
    {
        var go = Instantiate(rc.prefab, pos + Vector3.up * 3f, Quaternion.identity);
        go.name = rc.label;
        if (rc.payloadKg > 0)
        {
            var rootBody = go.GetComponentsInChildren<ArticulationBody>().First(b => b.isRoot);
            float m0 = rootBody.mass;
            rootBody.mass = m0 + rc.payloadKg;
            rootBody.inertiaTensor *= rootBody.mass / m0;
        }
        foreach (var b in go.GetComponentsInChildren<Behaviour>(true))
        {
            string t = b.GetType().Name;
            if (b is Camera || b is AudioListener || t.Contains("Lidar") || t.Contains("CameraCapture") || t == "LocationStuff"
                || t == "ImuSensor" || t == "CameraSwapper" || t == "CameraOrbit")
                b.enabled = false;
        }
        foreach (var dw in go.GetComponentsInChildren<DriveWheel>(true))
        {
            if (legacyDrive) { DestroyImmediate(dw); continue; }
            if ((speedGain >= 0 || integralTime >= 0) && dw.motor != null)
            {
                dw.motor = Instantiate(dw.motor);
                if (speedGain >= 0) dw.motor.speedLoopGain = speedGain;
                if (integralTime >= 0) dw.motor.speedLoopIntegralTime = integralTime;
            }
            if (noRolling) dw.rollingResistance = false;
        }
        var r = new Rover
        {
            go = go,
            root = go.GetComponentsInChildren<ArticulationBody>().First(b => b.isRoot),
            mc = go.GetComponentInChildren<MotorController>(),
        };
        if (wheelCollider == "sphere")
            foreach (var dw in go.GetComponentsInChildren<DriveWheel>(true))
                foreach (var mc in dw.GetComponentsInChildren<MeshCollider>(true))
                {
                    float rad = mc.transform.lossyScale.x * 0.5f;
                    var sc = mc.gameObject.AddComponent<SphereCollider>();
                    sc.radius = 0.5f; sc.center = Vector3.zero; sc.sharedMaterial = mc.sharedMaterial;
                    // non-uniform scale: sphere uses the largest axis; set scale uniform to the radius
                    mc.transform.localScale = Vector3.one * (2 * rad / Mathf.Max(mc.transform.parent.lossyScale.x, 1e-6f));
                    DestroyImmediate(mc);
                }
        if (contactOffset > 0)
        {
            foreach (var c in go.GetComponentsInChildren<Collider>(true)) c.contactOffset = contactOffset;
            foreach (var g in arena) g.GetComponent<Collider>().contactOffset = contactOffset;
        }
        if (bogieDamping >= 0)
            foreach (var b in go.GetComponentsInChildren<ArticulationBody>())
                if (!b.isRoot && b.GetComponent<DriveWheel>() == null && b.jointType == ArticulationJointType.RevoluteJoint)
                {
                    var d = b.xDrive; d.driveType = ArticulationDriveType.Force; d.stiffness = 0; d.damping = bogieDamping;
                    d.targetVelocity = 0; d.forceLimit = float.MaxValue; b.xDrive = d;
                    Debug.Log($"[DriveTest] {b.name}: joint damping {bogieDamping}");
                }
        if (solverPos > 0) { r.root.solverIterations = solverPos; r.root.solverVelocityIterations = solverVel; }   // after RobotPhysicsSetup.Awake
        Physics.SyncTransforms();
        float minY = go.GetComponentsInChildren<Collider>().Where(c => !c.isTrigger && c.enabled).Min(c => c.bounds.min.y);
        r.root.TeleportRoot(r.root.transform.position + Vector3.up * (pos.y + 0.02f - minY), r.root.transform.rotation);
        r.root.velocity = Vector3.zero; r.root.angularVelocity = Vector3.zero;
        yield return new WaitForFixedUpdate();
        r.wheels = r.mc.DriveWheels.Where(w => w != null).ToArray();
        if (!double.IsNaN(turnSlipOverride)) r.mc.TurnSlipFactor = turnSlipOverride;
        followTarget = r.root.transform;
        currentRover = r;
        // settle
        yield return Drive(r, 0, 0, 1.5f, null);
        r.restPitch = RestPitch(r);
        if (trace)
            foreach (var c in go.GetComponentsInChildren<Collider>().Where(c => c.enabled && !c.isTrigger && c.GetComponentInParent<DriveWheel>() == null)
                                 .OrderBy(c => c.bounds.min.y).Take(4))
                Debug.Log($"[Trace] {go.name}: lowest non-wheel collider {c.name} on {c.attachedArticulationBody?.name}: {c.bounds.min.y * 1000:F0} mm above ground");
        done(r);
    }

    /// <summary>Body pitch on flat ground after settling (deg, + = nose up), logged with the rocker angles: how far the
    /// body sags on its rocker-bogie (the differential's stiffness is all that holds it).</summary>
    static float RestPitch(Rover r)
    {
        var fwd = r.root.transform.forward;
        float pitch = Mathf.Atan2(fwd.y, new Vector2(fwd.x, fwd.z).magnitude) * Mathf.Rad2Deg;
        var diff = r.go.GetComponentInChildren<RockerDifferential>();
        string rockers = diff != null && diff.leftRocker != null && diff.rightRocker != null
            ? $", rockers L {diff.leftRocker.jointPosition[0] * Mathf.Rad2Deg:F2} R {diff.rightRocker.jointPosition[0] * Mathf.Rad2Deg:F2} deg"
            : "";
        Debug.Log($"[DriveTest] {r.go.name}: settled, body pitch {pitch:F2} deg{rockers}");
        return pitch;
    }

    // Per-wheel state, 4 times per simulated second (--drive-test-trace).
    void Trace(Rover r, string what)
    {
        if (!trace || Time.fixedTime - lastTrace < 0.25f) return;
        lastTrace = Time.fixedTime;
        var sb = new StringBuilder($"[Trace] {r.go.name} {what} t={Time.fixedTime:F2} v={r.Speed:F3} yawrate={r.YawRateCcw:F3}\n");
        foreach (var w in r.wheels)
            sb.Append($"   {w.name,-24} set {w.SetpointSpeed,7:F2} spd {w.Speed,7:F2} tq {w.OutputTorque,7:F2} avail {w.TorqueAvailable,6:F2} I {w.MotorCurrent,5:F1} " +
                      $"load {w.Contact.normalForce,6:F0} vlong {w.Contact.longitudinalVelocity,6:F3} surf {w.Contact.surfaceSpeed,6:F3} slip {w.Contact.slipRatio,6:F3} rr {w.RollingResistanceTorque,5:F2}" +
                      $" low {(w.HubWorld.y - w.Radius) * 1000,5:F1}mm" +
                      (w.Soil.handled ? $" | sink {w.Soil.sinkage * 1000,5:F1} mm spring {w.Soil.spring * 1000,4:F1} scale {w.Soil.stiffnessScale,4:F2} trac {w.Soil.traction,6:F1}/{w.Soil.tractionLimit,5:F1} patch {w.Soil.patchLength * 1000,3:F0}x{w.Soil.patchWidth * 1000,2:F0} el {w.Soil.elements} nrm {Mathf.Atan2(w.Soil.normal.z, w.Soil.normal.y) * Mathf.Rad2Deg,5:F1} F {w.Soil.force.y,5:F1} {w.Soil.force.z,5:F1}" : "") + "\n");
        Debug.Log(sb.ToString());
    }

    IEnumerator Drive(Rover r, float v, float w, float seconds, Action<float> eachStep)
    {
        var tw = new geometry_msgs.msg.Twist();
        tw.linear.x = v; tw.angular.z = w;
        float t0 = Time.fixedTime;
        while (Time.fixedTime - t0 < seconds)
        {
            r.mc.CmdVelCallback(tw);
            yield return new WaitForFixedUpdate();
            r.Sample();
            Trace(r, $"cmd v={v} w={w}");
            eachStep?.Invoke(Time.fixedTime - t0);
        }
    }

    IEnumerator Idle(float seconds, Action<float> eachStep)
    {
        float t0 = Time.fixedTime;
        while (Time.fixedTime - t0 < seconds) { yield return new WaitForFixedUpdate(); currentRover?.Sample(); eachStep?.Invoke(Time.fixedTime - t0); }
    }

    // Deactivate first: Destroy only happens at the end of the frame, and the next rover's MotorController finds wheels
    // by name (GameObject.Find skips inactive objects).
    void Despawn(Rover r) { if (r?.go != null) { r.go.SetActive(false); Destroy(r.go); } }

    Dictionary<string, object> NewResult(RoverCase rc, PhysicMaterial g, string test)
    {
        var d = new Dictionary<string, object> { ["rover"] = rc.label, ["ground"] = g.name, ["test"] = test };
        results.Add(d);
        return d;
    }

    // Per-step electrical stats over all wheels.
    class Stats
    {
        public float peakCurrent, peakTemp, sumAbsTorque, sumSlip, sumLoad, sumSink, maxSink; public int n, nSlip, nAir, nSoil;
        public void Add(Rover r)
        {
            foreach (var w in r.wheels)
            {
                peakCurrent = Mathf.Max(peakCurrent, w.MotorCurrent);
                peakTemp = Mathf.Max(peakTemp, w.WindingTemperature);
                sumAbsTorque += Mathf.Abs(w.OutputTorque); n++;
                if (w.Contact.grounded && w.Contact.normalForce > 0.5f) { sumSlip += Mathf.Abs(w.Contact.slipRatio); sumLoad += w.Contact.normalForce; nSlip++; }
                else nAir++;
                if (w.Soil.grounded) { sumSink += w.Soil.sinkage; maxSink = Mathf.Max(maxSink, w.Soil.sinkage); nSoil++; }
            }
        }
        public void Into(Dictionary<string, object> d)
        {
            if (n == 0) return;
            d["peak_current_A"] = peakCurrent; d["peak_winding_C"] = peakTemp; d["mean_wheel_torque_Nm"] = sumAbsTorque / n;
            d["contact_loss_pct"] = 100f * nAir / n;
            if (nSlip > 0) { d["mean_abs_slip"] = sumSlip / nSlip; d["mean_wheel_load_N"] = sumLoad / nSlip; }
            if (nSoil > 0) { d["mean_sink_mm"] = 1000f * sumSink / nSoil; d["max_sink_mm"] = 1000f * maxSink; }
        }
    }

    IEnumerator RunRover(RoverCase rc, PhysicMaterial g)
    {
        float wall0 = Time.realtimeSinceStartup, sim0 = Time.fixedTime;
        Rover r = null;

        // ---- accelerate + cruise
        if (Want("accel"))
        {
            yield return Spawn(rc, new Vector3(0, 0, -150), x => r = x);
            var d = NewResult(rc, g, "accel");
            var st = new Stats();
            Vector3 p0 = r.Pos; double ox = r.mc.OdomX, oy = r.mc.OdomY;
            float t63 = -1; var speeds = new List<float>();
            float lastVy = r.root.velocity.y; double az2 = 0; int naz = 0;
            Vector3 p5 = Vector3.zero; float t5 = 0, tLast = 0;
            yield return Drive(r, rc.testSpeed, 0, 8f, t =>
            {
                float s = r.Speed; speeds.Add(r.SpeedPhysx); st.Add(r);
                float vy = r.root.velocity.y;
                if (t > 4f) { float az = (vy - lastVy) / Time.fixedDeltaTime; az2 += az * az; naz++; }
                if (t5 == 0 && t >= 5f) { p5 = r.Pos; t5 = t; }
                tLast = t;
                lastVy = vy;
                if (t63 < 0 && s >= 0.63f * rc.testSpeed) t63 = t;
            });
            float trueDist = Vector3.Distance(p0, r.Pos);
            float odomDist = (float)Math.Sqrt((r.mc.OdomX - ox) * (r.mc.OdomX - ox) + (r.mc.OdomY - oy) * (r.mc.OdomY - oy));
            d["cmd_speed"] = rc.testSpeed;
            d["rest_pitch_deg"] = r.restPitch;
            d["t63_s"] = t63;
            d["cruise_speed"] = Vector3.Distance(p5, r.Pos) / Mathf.Max(tLast - t5, 1e-3f);   // from positions, last 3 s
            d["cruise_speed_physx"] = speeds.Skip(speeds.Count * 5 / 8).Average();             // ArticulationBody.velocity
            d["vert_accel_rms"] = naz > 0 ? (float)Math.Sqrt(az2 / naz) : 0f;   // m/s², body vibration while cruising
            d["true_dist_m"] = trueDist;
            d["odom_dist_m"] = odomDist;
            d["odom_err_pct"] = trueDist > 0.01f ? 100f * (odomDist - trueDist) / trueDist : 0f;
            st.Into(d);
            Despawn(r);
        }

        // ---- top speed
        if (Want("top"))
        {
            yield return Spawn(rc, new Vector3(-30, 0, -150), x => r = x);
            var d = NewResult(rc, g, "top");
            var st = new Stats(); var speeds = new List<float>();
            yield return Drive(r, 50f, 0, 8f, t => { speeds.Add(r.Speed); st.Add(r); });
            d["top_speed"] = speeds.Skip(speeds.Count * 5 / 8).Average();
            var w0 = r.wheels.FirstOrDefault();
            if (w0 != null && w0.motor != null) d["no_load_speed"] = w0.motor.NoLoadWheelSpeed * w0.Radius;
            st.Into(d);
            Despawn(r);
        }

        // ---- point turn
        if (Want("spin"))
        {
            yield return Spawn(rc, new Vector3(-60, 0, -150), x => r = x);
            var d = NewResult(rc, g, "spin");
            var st = new Stats(); var rates = new List<float>();
            Vector3 p0 = r.Pos;
            float yawAcc = 0, lastYaw = 0; double odomAcc = 0, lastOdom = 0, physxAcc = 0; int nPhysx = 0;
            const float cmdW = 0.5f;
            bool started = false;
            yield return Drive(r, 0, cmdW, 8f, t =>
            {
                rates.Add(r.YawRateCcw); st.Add(r);
                if (t >= 3f)
                {
                    if (!started) { started = true; lastYaw = r.YawCcw; lastOdom = r.mc.OdomYaw; }
                    physxAcc += r.YawRateCcw; nPhysx++;
                    yawAcc += Mathf.DeltaAngle(lastYaw * Mathf.Rad2Deg, r.YawCcw * Mathf.Rad2Deg) * Mathf.Deg2Rad; lastYaw = r.YawCcw;
                    odomAcc += Math.IEEERemainder(r.mc.OdomYaw - lastOdom, 2 * Math.PI); lastOdom = r.mc.OdomYaw;
                }
            });
            float trueRate = yawAcc / 5f;
            d["cmd_yaw_rate"] = cmdW;
            d["true_yaw_rate"] = trueRate;
            d["odom_yaw_rate"] = odomAcc / 5.0;
            d["physx_yaw_rate"] = nPhysx > 0 ? physxAcc / nPhysx : 0;   // ArticulationBody.angularVelocity (what the IMU gyro used)
            d["turn_slip_factor"] = r.mc.TurnSlipFactor;
            if (Mathf.Abs(trueRate) > 1e-3f)
            {
                // wheel speed difference ∝ cmd x factor; actual yaw ∝ wheel difference. To make actual = cmd:
                d["slip_factor_for_cmd_match"] = r.mc.TurnSlipFactor * cmdW / trueRate;
            }
            d["drift_m"] = Vector3.Distance(p0, r.Pos);
            st.Into(d);
            Despawn(r);
        }

        // ---- arc
        if (Want("arc"))
        {
            yield return Spawn(rc, new Vector3(-90, 0, -150), x => r = x);
            var d = NewResult(rc, g, "arc");
            var st = new Stats(); var rates = new List<float>(); var speeds = new List<float>();
            float v = rc.testSpeed * 0.5f, w = 0.3f;
            yield return Drive(r, v, w, 8f, t => { if (t > 3) { rates.Add(r.YawRateCcw); speeds.Add(r.Speed); } st.Add(r); });
            d["cmd_speed"] = v; d["cmd_yaw_rate"] = w;
            d["speed"] = speeds.Average(); d["true_yaw_rate"] = rates.Average();
            st.Into(d);
            Despawn(r);
        }

        // ---- command timeout: commands stop, wheels must stop by themselves
        if (Want("stop"))
        {
            yield return Spawn(rc, new Vector3(-120, 0, -150), x => r = x);
            var d = NewResult(rc, g, "stop");
            yield return Drive(r, rc.testSpeed, 0, 4f, null);
            Vector3 p0 = r.Pos; float v0 = r.Speed, tStop = -1;
            yield return Idle(3f, t => { if (tStop < 0 && Mathf.Abs(r.Speed) < 0.02f) tStop = t; });
            d["speed_before"] = v0;
            d["time_to_stop_s"] = tStop;
            d["stop_dist_m"] = Vector3.Distance(p0, r.Pos);
            d["final_speed"] = r.Speed;
            Despawn(r);
        }

        // ---- slopes
        if (Want("slope"))
            for (int i = 0; i < slopes.Length; i++)
            {
                Vector3 start = RampStart(i);
                yield return Spawn(rc, start + new Vector3(0, 0, -3f), x => r = x);
                var d = NewResult(rc, g, $"slope{slopes[i]:0}");
                var st = new Stats();
                var up = Quaternion.Euler(-slopes[i], 0, 0) * Vector3.forward;
                float Progress() => Vector3.Dot(r.Pos - start, up);
                float best = -1e9f, stuckSince = -1, minSpeed = 1e9f; string outcome = "timeout"; float tEnd = 0;
                float timeLimit = 3f / rc.testSpeed + rc.climbDistance / rc.testSpeed * 3f + 5f;
                bool finished = false;
                var tw = new geometry_msgs.msg.Twist(); tw.linear.x = rc.testSpeed;
                float t0 = Time.fixedTime;
                while (!finished && Time.fixedTime - t0 < timeLimit)
                {
                    r.mc.CmdVelCallback(tw);
                    yield return new WaitForFixedUpdate();
                    r.Sample();
                    float t = Time.fixedTime - t0, p = Progress();
                    Trace(r, $"slope{slopes[i]:0} p={p:F2}");
                    st.Add(r);
                    if (p > 0.5f) minSpeed = Mathf.Min(minSpeed, r.Speed);
                    if (Vector3.Dot(r.root.transform.up, Vector3.up) < 0.3f) { outcome = "tipped"; finished = true; }
                    else if (p >= rc.climbDistance) { outcome = "climbed"; finished = true; }
                    if (p > best + 0.02f) { best = p; stuckSince = t; }
                    else if (t - stuckSince > 3f && p > 0) { outcome = "stalled"; finished = true; }
                    tEnd = t;
                }
                d["outcome"] = outcome;
                d["progress_m"] = Progress();
                d["time_s"] = tEnd;
                d["min_speed_on_slope"] = minSpeed < 1e8f ? minSpeed : 0f;
                st.Into(d);
                if (outcome == "climbed")
                {
                    // Hold: command zero on the slope; does it creep back?
                    float p0 = Progress();
                    yield return Drive(r, 0, 0, 3f, _ => Trace(r, $"hold p={Progress():F3}"));
                    d["hold_creep_m"] = p0 - Progress();
                }
                Despawn(r);
            }

        float wall = Time.realtimeSinceStartup - wall0, sim = Time.fixedTime - sim0;
        var perf = NewResult(rc, g, "perf");
        perf["sim_s"] = sim; perf["wall_s"] = wall; perf["realtime_factor"] = sim / Mathf.Max(wall, 1e-3f);
        perf["fixed_dt_ms"] = Time.fixedDeltaTime * 1000f;
        if (SoilWorld.SoilSteps > 0)
        {
            perf["soil_ms_per_step"] = (float)(SoilWorld.SoilTicks * 1e3 / System.Diagnostics.Stopwatch.Frequency / SoilWorld.SoilSteps);
            if (soilWorld != null && soilWorld.Field != null) perf["soil_tiles"] = soilWorld.Field.TileCount;
            SoilWorld.SoilTicks = 0; SoilWorld.SoilSteps = 0;
        }
    }

    // ---------------------------------------------------------------- output

    void PrintTable()
    {
        var sb = new StringBuilder("[DriveTest] RESULTS\n");
        foreach (var d in results)
        {
            sb.Append($"  {d["rover"],-13} {d["ground"],-11} {d["test"],-8}");
            foreach (var kv in d.Where(k => k.Key != "rover" && k.Key != "ground" && k.Key != "test"))
                sb.Append($" {kv.Key}={Fmt(kv.Value)}");
            sb.AppendLine();
        }
        Debug.Log(sb.ToString());
    }

    static string Fmt(object v) => v is float f ? f.ToString("0.###", CultureInfo.InvariantCulture)
                                 : v is double x ? x.ToString("0.###", CultureInfo.InvariantCulture) : v.ToString();

    static string ToJson(List<Dictionary<string, object>> rows)
    {
        var sb = new StringBuilder("[\n");
        for (int i = 0; i < rows.Count; i++)
        {
            sb.Append("  {");
            sb.Append(string.Join(", ", rows[i].Select(kv => $"\"{kv.Key}\": " +
                (kv.Value is string s ? $"\"{s}\"" : Fmt(kv.Value)))));
            sb.Append(i < rows.Count - 1 ? "},\n" : "}\n");
        }
        return sb.Append("]\n").ToString();
    }

    void LateUpdate()
    {
        if (!followCamera || followTarget == null) return;
        var cam = Camera.main;
        if (cam == null) return;
        Vector3 f = Vector3.ProjectOnPlane(followTarget.forward, Vector3.up).normalized;
        Vector3 want = followTarget.position - f * 4f + Vector3.up * 2f;
        cam.transform.position = Vector3.Lerp(cam.transform.position, want, 0.1f);
        cam.transform.LookAt(followTarget.position + Vector3.up * 0.3f);
    }
}
