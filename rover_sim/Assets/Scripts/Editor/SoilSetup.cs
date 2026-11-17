using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Soil model setup: the Utah desert ground specs (Assets/Robots/Drivetrain/Grounds/Utah_*.asset) and which terrain
    /// layer is which soil. Values are literature numbers for similar soils (Wong, "Theory of Ground Vehicles"; dry
    /// sand / sandy loam / clayey soil), adjusted for a dry desert; nobody has measured the URC site with a bevameter,
    /// so treat them as a starting point and tune in the inspector.
    /// </summary>
    public static class SoilSetup
    {
        const string Dir = DrivetrainSetup.Root + "/Grounds";

        public static GroundSpec LooseSand => Load("Utah_LooseSand");
        public static GroundSpec Hardpack => Load("Utah_Hardpack");
        public static GroundSpec Bentonite => Load("Utah_BentoniteCrust");
        public static GroundSpec Gravel => Load("Utah_Gravel");
        public static GroundSpec Slickrock => Load("Utah_Slickrock");
        static GroundSpec Load(string n) => AssetDatabase.LoadAssetAtPath<GroundSpec>($"{Dir}/{n}.asset");

        public static IEnumerable<GroundSpec> All => new[] { LooseSand, Hardpack, Bentonite, Gravel, Slickrock };

        static GroundSpec Ensure(string name, Action<GroundSpec> init)
        {
            string path = $"{Dir}/{name}.asset";
            var a = AssetDatabase.LoadAssetAtPath<GroundSpec>(path);
            if (a != null) return a;
            a = ScriptableObject.CreateInstance<GroundSpec>();
            init(a);
            AssetDatabase.CreateAsset(a, path);
            Debug.Log("[Soil] created " + path);
            return a;
        }

        [MenuItem("SimFlake/Soil/Create Utah Ground Specs", priority = 260)]
        public static void EnsureAssets()
        {
            Ensure("Utah_LooseSand", g =>
            {
                g.notes = "Dry loose sand: wash beds, dunes, blow sand. Wheels sink and dig in when they spin. Bekker/shear: dry sand (Wong, LLL).";
                g.kc = 0.99f; g.kphi = 1528f; g.n = 1.1f; g.crustStrength = 0; g.cohesion = 1.04f; g.frictionAngle = 30f; g.shearModulus = 0.02f;
                g.rubberFriction = 0.42f; g.deformable = true; g.density = 1550; g.reposeAngle = 33; g.compaction = 0.35f; g.excavation = 0.5f;
                g.roughnessFine = 0.001f; g.roughnessCoarse = 0.01f; g.disturbedDarkening = 0.14f; g.rollingResistance = 0.2f;
            });
            Ensure("Utah_Hardpack", g =>
            {
                g.notes = "Dry silty desert soil with a thin crust: flats and tracks. Firm: a light rover leaves lug prints, a heavy one shallow ruts. " +
                          "Bekker/shear: sandy loam (Wong), stiffened for a dry compact soil; crust strength is an estimate.";
                g.kc = 20f; g.kphi = 20000f; g.n = 0.9f; g.crustStrength = 60f; g.cohesion = 4f; g.frictionAngle = 33f; g.shearModulus = 0.012f;
                g.rubberFriction = 0.65f; g.deformable = true; g.density = 1650; g.reposeAngle = 55; g.compaction = 0.7f; g.excavation = 0.15f;
                g.roughnessFine = 0.003f; g.roughnessCoarse = 0.012f; g.disturbedDarkening = 0.3f; g.rollingResistance = 0.06f;
            });
            Ensure("Utah_BentoniteCrust", g =>
            {
                g.notes = "Dry bentonite clay hills ('popcorn' crust over powder, Mancos/Morrison badlands). Weak fluffy crust, powder underneath, " +
                          "holds crisp prints. Bekker values are an estimate for a fluffy dry powder (a boot sinks 1-3 cm). (Wet bentonite is a different, " +
                          "far worse soil: not modelled.)";
                g.kc = 2f; g.kphi = 800f; g.n = 0.8f; g.crustStrength = 12f; g.cohesion = 3f; g.frictionAngle = 24f; g.shearModulus = 0.008f;
                g.rubberFriction = 0.45f; g.deformable = true; g.density = 1200; g.reposeAngle = 62; g.compaction = 0.75f; g.excavation = 0.2f;
                g.roughnessFine = 0.006f; g.roughnessCoarse = 0.015f; g.disturbedDarkening = 0.35f; g.rollingResistance = 0.1f;
            });
            Ensure("Utah_Gravel", g =>
            {
                g.notes = "Gravelly / pebbly soil and desert pavement: stones over silt. Rough, firm, little cohesion.";
                g.kc = 0.1f; g.kphi = 8000f; g.n = 0.9f; g.crustStrength = 20f; g.cohesion = 0.3f; g.frictionAngle = 38f; g.shearModulus = 0.02f;
                g.rubberFriction = 0.5f; g.deformable = true; g.density = 1750; g.reposeAngle = 38; g.compaction = 0.4f; g.excavation = 0.25f;
                g.roughnessFine = 0.005f; g.roughnessCoarse = 0.015f; g.disturbedDarkening = 0.18f; g.rollingResistance = 0.08f;
            });
            Ensure("Utah_Slickrock", g =>
            {
                g.notes = "Bare sandstone (slickrock), a little dusty. Rigid: no ruts; grip is rubber friction. shearModulus here is the tread rubber's own shear give.";
                g.deformable = false; g.rubberFriction = 0.8f; g.shearModulus = 0.002f; g.roughnessFine = 0.0015f; g.roughnessCoarse = 0.02f;
                g.disturbedDarkening = 0; g.rollingResistance = 0.02f;
            });
            AssetDatabase.SaveAssets();
        }

        /// <summary>
        /// Materials for the soil terrain shaders in Resources, so player builds include the shaders (nothing else
        /// references them) and SoilRenderer can load them.
        /// </summary>
        public static void EnsureMaterials()
        {
            foreach (var (file, shader) in new[] { ("SoilTerrain", "SimFlake/SoilTerrain"), ("SoilTerrainTess", "SimFlake/SoilTerrainTess"),
                                                   ("SoilTerrainAdd", "Hidden/SimFlake/SoilTerrainAdd"), ("SoilTerrainTessAdd", "Hidden/SimFlake/SoilTerrainTessAdd") })
            {
                string path = $"Assets/Resources/{file}.mat";
                var sh = Shader.Find(shader);
                if (sh == null) { Debug.LogError("[Soil] shader missing: " + shader); continue; }
                foreach (var m in UnityEditor.ShaderUtil.GetShaderMessages(sh))
                    Debug.Log($"[Soil] shader {shader}: {m.severity} {m.message} ({m.file}:{m.line})");
                var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
                if (mat == null) AssetDatabase.CreateAsset(new Material(sh) { name = file }, path);
                else if (mat.shader != sh) { mat.shader = sh; EditorUtility.SetDirty(mat); }
            }
            AssetDatabase.SaveAssets();
        }

        /// <summary>Soil for a terrain layer, by layer name (the Badlands layers and the soil yard use these names).</summary>
        public static GroundSpec ForLayer(string layerName)
        {
            string n = layerName.ToLowerInvariant();
            if (n.Contains("sand") && !n.Contains("stone")) return LooseSand;
            if (n.Contains("sandstone") || n.Contains("rock") && !n.Contains("soil")) return Slickrock;
            if (n.Contains("clay") || n.Contains("bentonite")) return Bentonite;
            if (n.Contains("pebble") || n.Contains("gravel") || n.Contains("rocky")) return Gravel;
            return Hardpack;
        }

        /// <summary>Add/refresh the SoilTerrain on a terrain, soils assigned by layer name.</summary>
        public static SoilTerrain MakeSoil(Terrain t)
        {
            EnsureAssets();
            var st = t.GetComponent<SoilTerrain>() ?? t.gameObject.AddComponent<SoilTerrain>();
            st.layerSpecs = t.terrainData.terrainLayers.Select(l => l != null ? ForLayer(l.name) : Hardpack).ToArray();
            st.defaultSpec = Hardpack;
            EditorUtility.SetDirty(st);
            return st;
        }
    }

    /// <summary>
    /// Single-wheel test rig, like a terramechanics lab's: one wheel under a fixed load is driven across a flat bed of
    /// one soil at a fixed forward speed and a fixed slip, and we record what it pulls (drawbar pull), the torque it
    /// needs, how deep it sinks. Runs the same <see cref="WheelSoilModel"/> as the rover, without PhysX, in seconds.
    /// Use it to compare tyres/treads/rubbers and to sanity-check soil numbers.
    ///   Unity -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.WheelRig.RunFromCommandLine
    ///         [-wheel MiniRoverWheel] [-ground Utah_LooseSand,...] [-load 18] [-speed 0.3] [-quality high] [-out rig.csv]
    /// </summary>
    public static class WheelRig
    {
        public struct Row { public string wheel, ground; public float load, slip, pull, torque, sinkage, rut, efficiency, patch, pressure, sideForce; }

        [MenuItem("SimFlake/Soil/Run Wheel Test Rig (mini rover wheel, all Utah soils)", priority = 261)]
        public static void RunMenu() => Debug.Log(Report(Sweep(DrivetrainSetup.MiniRoverWheel, SoilSetup.All, 18f, 0.3f, SoilQuality.High)));

        public static void RunFromCommandLine()
        {
            try
            {
                SoilSetup.EnsureAssets();
                string wn = Arg("-wheel") ?? "MiniRoverWheel";
                var wheel = AssetDatabase.LoadAssetAtPath<WheelSpec>($"{DrivetrainSetup.Root}/Wheels/{wn}.asset");
                if (wheel == null) throw new Exception("no wheel spec " + wn);
                var grounds = Arg("-ground") != null
                    ? Arg("-ground").Split(',').Select(n => AssetDatabase.LoadAssetAtPath<GroundSpec>($"{DrivetrainSetup.Root}/Grounds/{n}.asset")).Where(g => g != null).ToList()
                    : SoilSetup.All.ToList();
                float load = F("-load", 18f), speed = F("-speed", 0.3f);
                Enum.TryParse(Arg("-quality") ?? "high", true, out SoilQuality q);
                // -variants "name:key=value;key=value,name2:..." tries changed copies of the wheel next to the original.
                // keys: lugs (scale lug height, 0 = smooth tyre), shore, thickness (m), grip, width (scale)
                var wheelsToRun = new List<WheelSpec> { wheel };
                foreach (string v in (Arg("-variants") ?? "").Split(new[] { ',' }, StringSplitOptions.RemoveEmptyEntries))
                    wheelsToRun.Add(Variant(wheel, v));
                var rows = new List<Row>();
                SlopeDeg = F("-slope", 0f); Relax = (int)F("-relax", -1); NoDig = Arg("-nodig") != null; NoAside = Arg("-noaside") != null;
                foreach (var w in wheelsToRun) rows.AddRange(Sweep(w, grounds, load, speed, q, F("-angle", 0f)));
                Debug.Log(Arg("-summary") != null ? Summary(rows) : Report(rows));
                if (Arg("-out") != null) System.IO.File.WriteAllText(Arg("-out"), Csv(rows));
                EditorApplication.Exit(0);
            }
            catch (Exception e) { Debug.LogException(e); EditorApplication.Exit(1); }
        }

        static string Arg(string name) { var a = Environment.GetCommandLineArgs(); int i = Array.IndexOf(a, name); return i >= 0 && i + 1 < a.Length ? a[i + 1] : null; }
        static float F(string name, float def) => float.TryParse(Arg(name), NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : def;

        /// <summary>A changed copy of a wheel spec, e.g. "tall:lugs=2" or "soft:shore=20;thickness=0.015".</summary>
        public static WheelSpec Variant(WheelSpec src, string desc)
        {
            var w = UnityEngine.Object.Instantiate(src);
            var parts = desc.Split(':');
            w.name = parts[0];
            w.tread = JsonUtility.FromJson<TreadData>(JsonUtility.ToJson(src.tread));
            foreach (string kv in (parts.Length > 1 ? parts[1] : "").Split(new[] { ';' }, StringSplitOptions.RemoveEmptyEntries))
            {
                var p = kv.Split('=');
                float v = float.Parse(p[1], CultureInfo.InvariantCulture);
                switch (p[0])
                {
                    case "shore": w.shoreA = v; break;
                    case "thickness": w.rubberThickness = v; break;
                    case "grip": w.gripScale = v; break;
                    case "width": w.tread.width *= v; break;
                    case "lugs":
                        var c = w.tread.Carcass();
                        for (int t = 0; t < w.tread.nTheta; t++) for (int y = 0; y < w.tread.nY; y++)
                        { int k = t * w.tread.nY + y; w.tread.radius[k] = c[y] + Mathf.Max(0, w.tread.radius[k] - c[y]) * v; }
                        w.tread.Analyse();
                        break;
                    default: throw new Exception("unknown wheel variant key " + p[0]);
                }
            }
            return w;
        }

        /// <summary>One line per wheel x ground: the numbers you compare tyres by.</summary>
        public static string Summary(List<Row> rows)
        {
            var sb = new StringBuilder("[WheelRig] summary (DP = drawbar pull / load)\n  wheel            ground                 DP@20%  DP max (at slip)  sink@20% mm  torque@20% N·m  best efficiency  rolling resist.\n");
            foreach (var g in rows.GroupBy(r => (r.wheel, r.ground)))
            {
                var at20 = g.First(r => Mathf.Abs(r.slip - 0.2f) < 0.01f);
                var best = g.OrderByDescending(r => r.pull).First();
                var zero = g.First(r => Mathf.Abs(r.slip) < 0.01f);
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,-16} {1,-20} {2,7:F3} {3,8:F3} ({4:F2}) {5,11:F1} {6,14:F3} {7,15:F2} {8,15:F3}",
                    g.Key.wheel, g.Key.ground.Replace("Utah_", ""), at20.pull / at20.load, best.pull / best.load, best.slip, at20.sinkage * 1000, at20.torque,
                    g.Max(r => r.efficiency), -(zero.pull - zero.torque / 0.0655f) / zero.load));
            }
            return sb.ToString();
        }

        /// <summary>Experiment switches: run up a slope of this many degrees (pull is then along the slope, per load normal to it).</summary>
        public static float SlopeDeg;
        public static int Relax = -1;
        public static bool NoDig, NoAside;

        public static readonly float[] Slips = { -0.2f, 0f, 0.05f, 0.1f, 0.2f, 0.3f, 0.45f, 0.6f, 0.8f };

        public static List<Row> Sweep(WheelSpec wheel, IEnumerable<GroundSpec> grounds, float load, float speed, SoilQuality q, float slipAngleDeg = 0)
        {
            var rows = new List<Row>();
            foreach (var g in grounds)
                foreach (float s in Slips)
                    rows.Add(Run(wheel, g, load, speed, s, q, slipAngleDeg));
            return rows;
        }

        /// <summary>One run: 3 m at constant speed and slip; averages over the last 60 %.</summary>
        public static Row Run(WheelSpec wheel, GroundSpec ground, float load, float speed, float slip, SoilQuality q, float slipAngleDeg = 0, float dt = 0.005f, float wheelMass = 0.3f)
        {
            var preset = SoilWorld.Get(q);
            if (NoDig || NoAside)
            {
                ground = UnityEngine.Object.Instantiate(ground); ground.name = ground.name.Replace("(Clone)", "");
                if (NoDig) ground.excavation = 0;
                if (NoAside) ground.compaction = 1;
            }
            float sa0 = SlopeDeg * Mathf.Deg2Rad;
            Vector3 along = new Vector3(0, Mathf.Sin(sa0), Mathf.Cos(sa0)), normal = new Vector3(0, Mathf.Cos(sa0), -Mathf.Sin(sa0));
            var soil = new FlatSoil { spec = ground, height = 0, slopeZ = Mathf.Tan(sa0) };
            var field = new SoilField(preset.cell, (x, z) => soil);
            var model = new WheelSoilModel();
            float radius = wheel.tread.Valid ? wheel.tread.tipRadius : wheel.radius;
            model.Configure(wheel, radius, wheel.width, new WheelSoilModel.Settings
            { nTheta = preset.nTheta, nY = preset.nY, stabilityFactor = 1f, deform = true, sideForces = preset.sideForces, relaxPasses = Relax >= 0 ? Relax : preset.relaxPasses });
            radius = model.Radius;

            Vector3 axis = Vector3.right;                       // wheel rolls along +z
            WheelSoilModel.Frame(axis, out var e1, out var e2);
            float omega = speed / (radius * Mathf.Max(1f - slip, 0.05f));   // rad/s (slip = 1 - v / (omega r))
            float sa = slipAngleDeg * Mathf.Deg2Rad;
            Vector3 vel = (along * Mathf.Cos(sa) + Vector3.right * Mathf.Sin(sa)) * speed;
            float mass = load / 9.81f, vy = 0, angle = 0;       // vy: speed along the slope normal
            Vector3 pos = normal * (radius + 0.002f);
            float distance = 3f, t = 0, tEnd = distance / speed;
            double sPull = 0, sTorque = 0, sSink = 0, sPatch = 0, sPress = 0, sSide = 0; int n = 0;
            while (t < tEnd)
            {
                var rot = Quaternion.AngleAxis(angle * Mathf.Rad2Deg, axis);
                var pose = new WheelSoilModel.Pose
                {
                    hub = pos, axis = axis, e1 = rot * e1, e2 = rot * e2,
                    hubVelocity = vel + normal * vy, angularVelocity = axis * omega, spin = omega, stabilityMass = wheelMass,
                };
                var r = model.Step(field, soil, pose, dt);
                vy += (Vector3.Dot(r.force, normal) / mass - 9.81f * Mathf.Cos(sa0)) * dt;
                pos += (vel + normal * vy) * dt;
                angle += omega * dt; t += dt;
                if (t > 0.4f * tEnd)
                {
                    sPull += Vector3.Dot(r.force, along); sTorque += -Vector3.Dot(r.torque, axis); sSink += radius - Vector3.Dot(pos - along * Vector3.Dot(pos, along), normal);
                    sPatch += r.patchLength; sPress += r.pressure; sSide += r.force.x; n++;
                }
            }
            float pull = (float)(sPull / n), torque = (float)(sTorque / n);
            // rut left behind: lowest soil across the track 0.5 m back from the wheel
            var scratch = new SoilField.Patch();
            float rut = 0;
            for (float x = -0.03f; x <= 0.03f; x += 0.005f) rut = Mathf.Max(rut, -field.Height(pos.x + x - vel.x / speed * 0.5f, pos.z - vel.z / speed * 0.5f, scratch));
            return new Row
            {
                wheel = wheel.name, ground = ground.name, slip = slip, pull = pull, torque = torque, sinkage = (float)(sSink / n), rut = rut,
                load = load * Mathf.Cos(sa0),
                efficiency = torque * omega > 1e-4f ? Mathf.Clamp(pull * speed / (torque * omega), -9, 1) : 0,
                patch = (float)(sPatch / n), pressure = (float)(sPress / n), sideForce = (float)(sSide / n),
            };
        }

        public static string Report(List<Row> rows)
        {
            var sb = new StringBuilder("[WheelRig] drawbar pull / load vs slip (torque N·m, hub sinkage mm, rut left mm, tractive efficiency)\n");
            foreach (var grp in rows.GroupBy(r => r.wheel + " on " + r.ground))
            {
                sb.AppendLine($"{grp.Key}, load {grp.First().load:F0} N");
                sb.AppendLine("   slip   DP/W  torque   sink    rut    eff  patch mm  kPa   side/W");
                foreach (var r in grp)
                    sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "  {0,5:F2} {1,6:F3} {2,7:F3} {3,6:F1} {4,6:F1} {5,6:F2} {6,8:F0} {7,5:F0} {8,7:F3}",
                        r.slip, r.pull / r.load, r.torque, r.sinkage * 1000, r.rut * 1000, r.efficiency, r.patch * 1000, r.pressure / 1000, r.sideForce / r.load));
            }
            return sb.ToString();
        }

        public static string Csv(List<Row> rows)
        {
            var sb = new StringBuilder("wheel,ground,load_N,slip,pull_N,pull_over_load,torque_Nm,sinkage_mm,rut_mm,efficiency,patch_mm,pressure_kPa,side_N\n");
            foreach (var r in rows)
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "{0},{1},{2:F1},{3:F2},{4:F3},{5:F4},{6:F4},{7:F2},{8:F2},{9:F3},{10:F1},{11:F1},{12:F3}",
                    r.wheel, r.ground, r.load, r.slip, r.pull, r.pull / r.load, r.torque, r.sinkage * 1000, r.rut * 1000, r.efficiency, r.patch * 1000, r.pressure / 1000, r.sideForce));
            return sb.ToString();
        }
    }
}
