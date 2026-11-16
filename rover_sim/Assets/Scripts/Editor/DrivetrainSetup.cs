using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Drivetrain model setup (see Scripts/Robot/Drivetrain):
    ///  - spec assets in Assets/Robots/Drivetrain/{Motors,Wheels,Grounds} and the ground lookup Resources/GroundSpecs.asset
    ///    (created once with datasheet values; edit them in the inspector afterwards, re-running doesn't overwrite);
    ///  - physics project settings (fixed timestep, TGS solver, improved patch friction);
    ///  - chassis_2024.prefab: smooth wheel colliders from its WheelSpec, DriveWheel + motor, RobotPhysicsSetup, HUD
    ///    (mini_rover gets the same from MiniRoverBuilder);
    ///  - the test bench scene Assets/Scenes/Tests/DriveTest.unity.
    ///   Unity -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.DrivetrainSetup.SetUpFromCommandLine
    /// </summary>
    public static class DrivetrainSetup
    {
        public const string Root = "Assets/Robots/Drivetrain";
        public const string CylinderMeshPath = Root + "/Wheels/unit_cylinder_collider.asset";
        public const string Chassis2024 = "Assets/Robots/Chassis/chassis_2024.prefab";
        public const string MiniRover = "Assets/Robots/Chassis/mini_rover.prefab";
        public const string TestScene = "Assets/Scenes/Tests/DriveTest.unity";
        public const float FixedTimestep = 0.005f;

        [MenuItem("SimFlake/Drivetrain/Set Up Everything (assets, physics, chassis_2024, test scene)", priority = 250)]
        public static void SetUpMenu()
        {
            if (!EditorUtility.DisplayDialog("Drivetrain setup", $"Rewrite the wheel colliders and drive components of {Chassis2024}, set physics project settings, rebuild {TestScene}?", "Go", "Cancel")) return;
            SetUp();
        }

        public static void SetUpFromCommandLine() => RunAndExit(SetUp);

        [MenuItem("SimFlake/Drivetrain/Apply To chassis_2024", priority = 251)]
        public static void ApplyChassisMenu() => ApplyToChassis2024();

        [MenuItem("SimFlake/Drivetrain/Apply To mini_rover (keeps sensor moves)", priority = 252)]
        public static void ApplyMiniMenu() => ApplyToMiniRover();

        [MenuItem("SimFlake/Drivetrain/Build Drive Test Scene", priority = 252)]
        public static void BuildTestSceneMenu() => BuildTestScene();

        public static void BuildTestSceneFromCommandLine() => RunAndExit(BuildTestScene);

        static void RunAndExit(Func<bool> f)
        {
            bool ok = false;
            try { ok = f(); }
            catch (Exception e) { Debug.LogException(e); }
            EditorApplication.Exit(ok ? 0 : 1);
        }

        public static bool SetUp()
        {
            EnsureAssets();
            SetPhysicsSettings();
            bool ok = ApplyToChassis2024();
            ok &= ApplyToMiniRover();
            ok &= BuildTestScene();
            AssetDatabase.SaveAssets();
            return ok;
        }

        // ------------------------------------------------------------------ assets

        public static DriveMotorSpec MaxonMotor => Load<DriveMotorSpec>(Root + "/Motors/Maxon_ECXFL42M_GPX42_26.asset");
        public static DriveMotorSpec MiniMotor => Load<DriveMotorSpec>(Root + "/Motors/ES_36PG_3429_100.asset");
        public static WheelSpec Chassis2024Wheel => Load<WheelSpec>(Root + "/Wheels/Chassis2024Wheel.asset");
        public static WheelSpec MiniRoverWheel => Load<WheelSpec>(Root + "/Wheels/MiniRoverWheel.asset");

        static T Load<T>(string path) where T : UnityEngine.Object => AssetDatabase.LoadAssetAtPath<T>(path);

        static T Ensure<T>(string path, Action<T> init) where T : ScriptableObject
        {
            var a = Load<T>(path);
            if (a != null) return a;
            System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(path));
            a = ScriptableObject.CreateInstance<T>();
            init(a);
            AssetDatabase.CreateAsset(a, path);
            Debug.Log("[Drivetrain] created " + path);
            return a;
        }

        public static void EnsureAssets()
        {
            Ensure<DriveMotorSpec>(Root + "/Motors/Maxon_ECXFL42M_GPX42_26.asset", m =>
            {
                m.notes = "chassis_2024 drive: maxon ECXFL42M KL A HTQ 24V + GPX42 C 26:1 (part B84441BAF7EF, datasheet 06.05.2026).\n" +
                          "Motor/gearhead values are from the datasheet. ESTIMATES: currentLimit 20 A (controller peak, unknown), " +
                          "maxWheelAcceleration 50 rad/s² (RoverFlake2 drive_control MAX_ACCEL_RADS), speedLoopGain.";
                m.nominalVoltage = 24; m.noLoadSpeedRpm = 8050; m.noLoadCurrent = 0.363f; m.stallTorque = 1.11f;
                m.torqueConstant = 0.0282f; m.terminalResistance = 0.233f; m.nominalCurrent = 7.31f; m.rotorInertia = 86.2f;
                m.gearRatio = 26; m.gearEfficiency = 0.81f; m.gearheadMaxTorque = 11.3f; m.gearheadInertia = 15f;
                m.thermalResistanceWinding = 3.04f; m.thermalResistanceHousing = 1.66f;
                m.windingTimeConstant = 20.2f; m.motorTimeConstant = 92.3f; m.maxWindingTemperature = 155f;
                m.currentLimit = 20f; m.supplyVoltage = 24f; m.maxWheelAcceleration = 50f; m.speedLoopGain = 40f; m.speedLoopIntegralTime = 0f;
            });
            Ensure<DriveMotorSpec>(Root + "/Motors/ES_36PG_3429_100.asset", m =>
            {
                m.notes = "mini_rover drive: E-S Motor 36PG-3429-100-EN, 12 V brushed, 100:1 planetary, encoder.\n" +
                          "Retailer spec (RobotShop): no-load 95 rpm / <450 mA, rated 15 kg·cm @ 47 rpm / <2.5 A, stall 25 kg·cm (2.45 N·m) / <5 A.\n" +
                          "Motor-side values are DERIVED from those output figures with an ASSUMED 70 % gear efficiency " +
                          "(stall 2.45/(100x0.7) = 0.035 N·m, kt = 0.035/5 = 7 mN·m/A, R = 12 V/5 A = 2.4 Ω). " +
                          "ESTIMATES: thermal values (none published; a generic 34 mm can), gearhead limit 3 N·m, controller current limit 5 A, " +
                          "acceleration (firmware unknown).";
                m.nominalVoltage = 12; m.noLoadSpeedRpm = 9500; m.noLoadCurrent = 0.45f; m.stallTorque = 0.035f;
                m.torqueConstant = 0.007f; m.terminalResistance = 2.4f; m.nominalCurrent = 2.5f; m.rotorInertia = 25f;
                m.gearRatio = 100; m.gearEfficiency = 0.7f; m.gearheadMaxTorque = 3f; m.gearheadInertia = 2f;
                m.thermalResistanceWinding = 6f; m.thermalResistanceHousing = 12f;
                m.windingTimeConstant = 15f; m.motorTimeConstant = 600f; m.maxWindingTemperature = 125f;
                m.currentLimit = 5f; m.supplyVoltage = 12f; m.maxWheelAcceleration = 30f; m.speedLoopGain = 2f; m.speedLoopIntegralTime = 0f;
            });

            var rubber = EnsurePhysicMaterial(Root + "/Wheels/RoverTread.physicMaterial", "RoverTread", 1f, 1f, PhysicMaterialCombine.Multiply);
            Ensure<WheelSpec>(Root + "/Wheels/Chassis2024Wheel.asset", w =>
            {
                w.notes = "chassis_2024 wheel. Size measured from the imported mesh (radius/width 0). Tread friction 1.0 with Multiply, " +
                          "so the ground material's friction applies as-is. Sphere collider: the mesh cylinder jittered on the ground " +
                          "(25 % contact loss, 0.5 g vibration at 5 ms steps).";
                w.shape = WheelColliderShape.Sphere; w.material = rubber; w.mass = 0.9f; w.rollingResistanceScale = 1f;
            });
            var miniMat = Load<PhysicMaterial>("Assets/Materials/MiniRoverWheel.physicMaterial");
            Ensure<WheelSpec>(Root + "/Wheels/MiniRoverWheel.asset", w =>
            {
                w.notes = "mini_rover grousered 3D-printed wheel, collides as a sphere of the tread radius (VHACD hulls snagged; " +
                          "the mesh cylinder rolls with some contact jitter). " +
                          "Friction capped at 0.35 (Minimum combine): the bogie pivot is higher than half the bogie wheel spacing, " +
                          "so skid turns at ~0.5 friction flip the bogies.";
                w.shape = WheelColliderShape.Sphere; w.material = miniMat; w.rollingResistanceScale = 1.3f;
            });

            // Ground specs for the materials the worlds use.
            var dirt = Load<PhysicMaterial>("Assets/Materials/CustomDirt.physicMaterial");
            var asphalt = Load<PhysicMaterial>("Assets/Materials/Asphalt.physicMaterial");
            var gDirt = Ensure<GroundSpec>(Root + "/Grounds/PackedDirt.asset", g =>
            {
                g.notes = "CustomDirt: packed dirt / desert crust (most terrain and the Badlands rocks). Bekker values: dry sandy loam (Wong).";
                g.material = dirt; g.rollingResistance = 0.06f;
                g.kc = 0.9f; g.kphi = 1523.4f; g.n = 1.1f; g.cohesion = 1.04f; g.frictionAngle = 28f; g.shearModulus = 0.025f; g.deformable = true;
            });
            var gAsphalt = Ensure<GroundSpec>(Root + "/Grounds/Asphalt.asset", g =>
            {
                g.notes = "Asphalt / concrete. Rigid: no sinkage.";
                g.material = asphalt; g.rollingResistance = 0.02f; g.deformable = false;
            });
            var gDefault = Ensure<GroundSpec>(Root + "/Grounds/Default.asset", g =>
            {
                g.notes = "Anything without a known material (Unity's default 0.6 friction): treated as firm ground.";
                g.rollingResistance = 0.04f;
            });
            var table = Ensure<GroundSpecTable>("Assets/Resources/GroundSpecs.asset", t => { });
            if (table.defaultGround == null || table.grounds.Count == 0)
            {
                table.defaultGround = gDefault;
                table.grounds = new List<GroundSpec> { gDirt, gAsphalt };
                EditorUtility.SetDirty(table);
            }
            FixGroundMaterials();
            CylinderMesh();
            AssetDatabase.SaveAssets();
        }

        static PhysicMaterial EnsurePhysicMaterial(string path, string name, float stat, float dyn, PhysicMaterialCombine combine)
        {
            var m = Load<PhysicMaterial>(path);
            if (m != null) return m;
            System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(path));
            m = new PhysicMaterial(name) { staticFriction = stat, dynamicFriction = dyn, frictionCombine = combine };
            AssetDatabase.CreateAsset(m, path);
            return m;
        }

        // CustomDirt had static friction 0.03 < dynamic 0.4: wheels broke loose almost at once, then grabbed again.
        // Rubber/treads on packed dirt ~0.6 static, ~0.5 sliding; on asphalt ~0.8 / 0.7.
        public static void FixGroundMaterials()
        {
            void Set(string path, float s, float d)
            {
                var m = Load<PhysicMaterial>(path);
                if (m == null || (Mathf.Approximately(m.staticFriction, s) && Mathf.Approximately(m.dynamicFriction, d))) return;
                Debug.Log($"[Drivetrain] {path}: friction {m.staticFriction}/{m.dynamicFriction} -> {s}/{d}");
                m.staticFriction = s; m.dynamicFriction = d;
                EditorUtility.SetDirty(m);
            }
            Set("Assets/Materials/CustomDirt.physicMaterial", 0.6f, 0.5f);
            Set("Assets/Materials/Asphalt.physicMaterial", 0.8f, 0.7f);
        }

        // ------------------------------------------------------------------ physics settings

        public static void SetPhysicsSettings()
        {
            var time = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TimeManager.asset").FirstOrDefault();
            if (time != null)
            {
                var so = new SerializedObject(time);
                var p = so.FindProperty("Fixed Timestep");
                if (p != null && p.propertyType == SerializedPropertyType.Float) { p.floatValue = FixedTimestep; so.ApplyModifiedPropertiesWithoutUndo(); }
                else Time.fixedDeltaTime = FixedTimestep;
                Debug.Log($"[Drivetrain] fixed timestep {FixedTimestep * 1000} ms");
            }
            var dyn = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/DynamicsManager.asset").FirstOrDefault();
            if (dyn != null)
            {
                var so = new SerializedObject(dyn);
                void SetInt(string n, int v) { var p = so.FindProperty(n); if (p == null) Debug.LogWarning("[Drivetrain] no physics setting " + n); else { p.intValue = v; Debug.Log($"[Drivetrain] {n} = {v}"); } }
                void SetBool(string n, bool v) { var p = so.FindProperty(n); if (p == null) Debug.LogWarning("[Drivetrain] no physics setting " + n); else { p.boolValue = v; Debug.Log($"[Drivetrain] {n} = {v}"); } }
                SetInt("m_SolverType", 1);              // Temporal Gauss-Seidel: much better for articulations and stiff contacts
                SetBool("m_ImprovedPatchFriction", true); // friction matches the analytical Coulomb model on patches
                so.ApplyModifiedPropertiesWithoutUndo();
            }
            AssetDatabase.SaveAssets();
        }

        // ------------------------------------------------------------------ robots

        // turnSlipFactor measured on CustomDirt with the drive test bench (point turn at 0.5 rad/s: commanded = actual yaw).
        // Skid steer depends on the ground and load: chassis_2024 2.05-2.15 (35-50 kg) on dirt; mini_rover's friction is capped so ~2.2 everywhere.
        public static bool ApplyToChassis2024() => ApplyToPrefab(Chassis2024, MaxonMotor, Chassis2024Wheel, 2.1, Chassis2024MassKg);

        // Real rover (team, 2026-11): wheels 0.9 kg each, under 50 kg with the arm, arm ~15 kg -> 35 kg without it.
        // The URDF said 86 kg. The chassis/bogie split is scaled from the URDF (not measured). The arm is a bench payload.
        public const float Chassis2024MassKg = 35f;

        /// <summary>Same as MiniRoverBuilder does on a rebuild, but on the existing prefab (sensor moves stay).</summary>
        public static bool ApplyToMiniRover() => ApplyToPrefab(MiniRover, MiniMotor, MiniRoverWheel, 2.2);

        /// <summary>Puts the drivetrain model on a robot prefab whose MotorController lists its six wheels.</summary>
        public static bool ApplyToPrefab(string prefabPath, DriveMotorSpec motor, WheelSpec wheel, double turnSlipFactor = 0, float totalMassKg = 0)
        {
            EnsureAssets();
            motor = motor ?? (prefabPath == MiniRover ? MiniMotor : MaxonMotor);
            wheel = wheel ?? (prefabPath == MiniRover ? MiniRoverWheel : Chassis2024Wheel);
            var root = PrefabUtility.LoadPrefabContents(prefabPath);
            try
            {
                var mc = root.GetComponentInChildren<MotorController>(true);
                if (mc == null) throw new Exception("no MotorController in " + prefabPath);
                var so = new SerializedObject(mc);
                var wheels = new List<ArticulationBody>();
                for (int i = 0; i < 6; i++)
                {
                    var w = so.FindProperty("wheelJoints").GetArrayElementAtIndex(i).objectReferenceValue as ArticulationBody;
                    if (w == null)
                    {
                        string n = so.FindProperty("wheelJointNames").GetArrayElementAtIndex(i).stringValue;
                        w = root.GetComponentsInChildren<ArticulationBody>(true).FirstOrDefault(b => b.name == n);
                    }
                    if (w == null) throw new Exception("wheel " + i + " not found");
                    wheels.Add(w);
                }
                AttachDrivetrain(root, wheels, motor, wheel);
                if (totalMassKg > 0) ScaleBodyMass(root, wheels, totalMassKg);
                if (turnSlipFactor > 0) { so.FindProperty("turnSlipFactor").doubleValue = turnSlipFactor; so.ApplyModifiedPropertiesWithoutUndo(); }
                if (root.GetComponent<RobotPhysicsSetup>() == null) root.AddComponent<RobotPhysicsSetup>();
                PrefabUtility.SaveAsPrefabAsset(root, prefabPath, out bool ok);
                Debug.Log(ok ? "[Drivetrain] saved " + prefabPath : "[Drivetrain] FAILED to save " + prefabPath);
                return ok;
            }
            finally { PrefabUtility.UnloadPrefabContents(root); }
        }

        /// <summary>Wheel colliders from the WheelSpec, a DriveWheel on each wheel, and the HUD on the robot root.</summary>
        public static void AttachDrivetrain(GameObject robot, IEnumerable<ArticulationBody> wheels, DriveMotorSpec motor, WheelSpec spec)
        {
            var mesh = CylinderMesh();
            foreach (var w in wheels)
            {
                ApplyWheelSpec(w, spec, mesh);
                var dw = w.GetComponent<DriveWheel>() ?? w.gameObject.AddComponent<DriveWheel>();
                dw.motor = motor; dw.wheel = spec;
                // Initial drive: the DriveWheel rewrites it every step; this is what an editor (not playing) shows.
                var d = w.xDrive;
                d.driveType = ArticulationDriveType.Force;   // Velocity/Target types ignore damping (Unity computes its own gains)
                d.stiffness = 0; d.damping = motor.speedLoopGain; d.forceLimit = motor.PeakWheelTorque;
                w.xDrive = d;
                w.jointFriction = 0; w.angularDamping = 0.05f;
            }
            if (robot.GetComponent<DrivetrainHud>() == null) robot.AddComponent<DrivetrainHud>();
            Debug.Log($"[Drivetrain] {robot.name}: {motor.name}: {motor.Summary}");
        }

        public static void ApplyWheelSpec(ArticulationBody w, WheelSpec spec, Mesh cylinder)
        {
            if (spec == null || spec.shape == WheelColliderShape.KeepImported) return;
            Fit(w, out Vector3 axis, out Vector3 centre, out float rMesh, out float widthMesh);
            float r = spec.radius > 0 ? spec.radius : rMesh;
            float width = spec.width > 0 ? spec.width : widthMesh;

            foreach (var c in w.GetComponentsInChildren<Collider>(true))
            {
                // Only the wheel's own colliders (not those of child links, if any).
                if (c.GetComponentInParent<ArticulationBody>() != w) continue;
                var go = c.gameObject;
                UnityEngine.Object.DestroyImmediate(c, true);
                if (go != w.gameObject && go.name.StartsWith("Tread") && go.GetComponents<Component>().Length == 1)
                    UnityEngine.Object.DestroyImmediate(go, true);
            }
            if (spec.shape == WheelColliderShape.Sphere)
            {
                var sg = new GameObject("TreadCollider");
                sg.transform.SetParent(w.transform, false);
                sg.transform.localPosition = centre;
                var sc = sg.AddComponent<SphereCollider>();
                sc.radius = r; sc.sharedMaterial = spec.material;
                SetMass(w, spec.mass);
                Debug.Log($"[Drivetrain] {w.name}: Sphere r {r:F4} (tread width {width:F4})");
                return;
            }
            var holder = new GameObject("TreadCollider");
            holder.transform.SetParent(w.transform, false);
            holder.transform.localPosition = centre;
            holder.transform.localRotation = Quaternion.FromToRotation(Vector3.up, axis);
            float core = spec.shape == WheelColliderShape.Grousers ? r - spec.grouserHeight : r;
            holder.transform.localScale = new Vector3(2 * core, width, 2 * core);    // unit cylinder: radius 0.5, height 1 along y
            var mc = holder.AddComponent<MeshCollider>();
            mc.sharedMesh = cylinder; mc.convex = true; mc.sharedMaterial = spec.material;

            if (spec.shape == WheelColliderShape.Grousers)
            {
                for (int i = 0; i < spec.grouserCount; i++)
                {
                    var g = new GameObject($"TreadGrouser{i}");
                    g.transform.SetParent(w.transform, false);
                    Quaternion around = Quaternion.AngleAxis(360f * i / spec.grouserCount, axis);
                    Vector3 radial = around * Vector3.ProjectOnPlane(Vector3.forward, axis).normalized;
                    g.transform.localPosition = centre + radial * (core + spec.grouserHeight / 2);
                    g.transform.localRotation = Quaternion.LookRotation(Vector3.Cross(axis, radial), radial);
                    var box = g.AddComponent<BoxCollider>();
                    box.size = new Vector3(width, spec.grouserHeight, Mathf.Max(0.004f, spec.grouserHeight * 0.6f));
                    box.sharedMaterial = spec.material;
                }
            }
            SetMass(w, spec.mass);
            Debug.Log($"[Drivetrain] {w.name}: {spec.shape} r {r:F4} width {width:F4} (mesh r {rMesh:F4} w {widthMesh:F4})");
        }

        /// <summary>Sets a body's mass and scales its inertia tensor with it (the URDF tensors are explicit, not computed).</summary>
        public static void SetMass(ArticulationBody b, float mass)
        {
            if (mass <= 0 || Mathf.Approximately(b.mass, mass)) return;
            b.inertiaTensor *= mass / b.mass;
            b.mass = mass;
        }

        /// <summary>Scales every non-wheel body (mass and inertia) so the whole robot weighs totalKg. Keeps the
        /// URDF's mass distribution between chassis and suspension; only the total changes.</summary>
        public static void ScaleBodyMass(GameObject robot, ICollection<ArticulationBody> wheels, float totalKg)
        {
            var bodies = robot.GetComponentsInChildren<ArticulationBody>(true);
            float wheelKg = bodies.Where(wheels.Contains).Sum(b => b.mass);
            var rest = bodies.Where(b => !wheels.Contains(b)).ToArray();
            float restKg = rest.Sum(b => b.mass);
            float k = (totalKg - wheelKg) / restKg;
            if (k <= 0) throw new Exception($"[Drivetrain] {robot.name}: wheels alone weigh {wheelKg} kg > {totalKg} kg");
            if (Mathf.Abs(k - 1) < 1e-4f) return;
            foreach (var b in rest) SetMass(b, b.mass * k);
            Debug.Log($"[Drivetrain] {robot.name}: total mass {wheelKg + restKg:F1} -> {totalKg:F1} kg (wheels {wheelKg:F1} kg; " +
                      string.Join(", ", rest.Select(b => $"{b.name} {b.mass:F2}")) + ")");
        }

        /// <summary>Tread cylinder fitted to the wheel's visual mesh: axle axis, tread centre (wheel-local), radius, width.</summary>
        public static void Fit(ArticulationBody w, out Vector3 axis, out Vector3 centre, out float radius, out float width)
        {
            axis = (w.anchorRotation * Vector3.right).normalized;
            Vector3 pivot = w.anchorPosition, ax = axis;
            var verts = w.GetComponentsInChildren<MeshFilter>(true)
                         .Where(f => f.sharedMesh != null && f.GetComponentInParent<ArticulationBody>() == w)
                         .SelectMany(f => f.sharedMesh.vertices.Select(v => w.transform.InverseTransformPoint(f.transform.TransformPoint(v)) - pivot))
                         .ToArray();
            if (verts.Length == 0) throw new Exception($"[Drivetrain] {w.name}: no visual mesh to fit the tread to");
            float rMax = verts.Max(v => Vector3.ProjectOnPlane(v, ax).magnitude);
            var tread = verts.Where(v => Vector3.ProjectOnPlane(v, ax).magnitude > 0.9f * rMax).Select(v => Vector3.Dot(v, ax)).ToArray();
            float a0 = tread.Min(), a1 = tread.Max();
            radius = rMax; width = a1 - a0;
            centre = pivot + axis * (0.5f * (a0 + a1));
        }

        // Sides of the tread collider. PhysX convex hulls take at most 255 vertices, so 120 sides (240 vertices). With 32
        // sides the 0.6 mm facet bumps made chassis_2024 buzz on flat ground (contacts breaking every few steps).
        const int CylinderSides = 120;

        /// <summary>Unit cylinder (radius 0.5, height 1 along y) used for every tread collider. Rebuilt in place (same GUID) if the side count changes.</summary>
        public static Mesh CylinderMesh()
        {
            var m = Load<Mesh>(CylinderMeshPath);
            if (m != null && m.vertexCount == 2 * CylinderSides) return m;
            const int n = CylinderSides;
            var v = new List<Vector3>(); var tris = new List<int>();
            for (int i = 0; i < n; i++)
            {
                float t = 2 * Mathf.PI * i / n;
                v.Add(new Vector3(0.5f * Mathf.Cos(t), -0.5f, 0.5f * Mathf.Sin(t)));
                v.Add(new Vector3(0.5f * Mathf.Cos(t), 0.5f, 0.5f * Mathf.Sin(t)));
            }
            for (int i = 0; i < n; i++)
            {
                int a = 2 * i, b = 2 * ((i + 1) % n);
                tris.AddRange(new[] { a, a + 1, b, b, a + 1, b + 1 });
                if (i > 0 && i < n - 1) { tris.AddRange(new[] { 0, b, a }); tris.AddRange(new[] { 1, a + 1, b + 1 }); }
            }
            bool create = m == null;
            if (create) m = new Mesh();
            m.Clear();
            m.name = "unit_cylinder_collider";
            m.SetVertices(v); m.SetTriangles(tris, 0);
            m.RecalculateNormals(); m.RecalculateBounds();
            if (create)
            {
                System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(CylinderMeshPath));
                AssetDatabase.CreateAsset(m, CylinderMeshPath);
            }
            else EditorUtility.SetDirty(m);
            AssetDatabase.SaveAssets();
            Debug.Log($"[Drivetrain] tread collider mesh: {n} sides");
            return m;
        }

        // ------------------------------------------------------------------ test scene

        public static bool BuildTestScene()
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            var ros = new GameObject("ROS Connection").AddComponent<SimFlake.Ros.RosConnection>();
            ros.autoLaunchRosbridge = false;   // the bench never talks to ROS
            ros.showStatusOverlay = false;

            var runner = new GameObject("Drive Test").AddComponent<DriveTestRunner>();
            runner.rovers = new[]
            {
                new DriveTestRunner.RoverCase { label = "chassis_2024", prefab = Load<GameObject>(Chassis2024), testSpeed = 1.0f, climbDistance = 5f },
                new DriveTestRunner.RoverCase { label = "chassis_2024_arm", prefab = Load<GameObject>(Chassis2024), testSpeed = 1.0f, climbDistance = 5f, payloadKg = 15f },
                new DriveTestRunner.RoverCase { label = "mini_rover", prefab = Load<GameObject>(MiniRover), testSpeed = 0.3f, climbDistance = 2f },
            };
            runner.grounds = new[]
            {
                Load<PhysicMaterial>("Assets/Materials/CustomDirt.physicMaterial"),
                Load<PhysicMaterial>("Assets/Materials/Asphalt.physicMaterial"),
            };
            System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(TestScene));
            bool ok = EditorSceneManager.SaveScene(scene, TestScene);
            Debug.Log(ok ? "[Drivetrain] saved " + TestScene : "[Drivetrain] FAILED to save " + TestScene);
            return ok;
        }
    }
}
