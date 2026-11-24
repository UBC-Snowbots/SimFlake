using System;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using UnityEditor;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Builds rover prefabs from RoverFlake2 rover_description URDFs: Assets/Robots/Chassis/mini_rover.prefab (mini_rover.urdf)
    /// and chassis_2026.prefab (chassis.urdf, the 2026 big rover). Both have the same structure: base + rockers coupled by a
    /// differential bar + bogies + six wheels, with the same joint names. Each rover is one <see cref="Def"/>.
    ///
    /// The URDF's sensor links (LiDAR, IMU, GPS antenna) are replaced by <see cref="SensorBlock"/>s: coloured,
    /// labelled boxes that carry the sensor script, so a sensor can be moved by dragging its block. A camera block
    /// is added. Topics and frame ids are the same as chassis_2024's, so only one of these rovers per scene.
    ///
    /// Re-running overwrites the prefab (and any sensor moves saved in it). Scene instances keep their overrides.
    ///   Unity -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.MiniRoverBuilder.BuildFromCommandLine
    ///   Unity -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.MiniRoverBuilder.BuildChassis2026FromCommandLine
    /// </summary>
    public static class MiniRoverBuilder
    {
        const string BlockMatPath = "Assets/Materials/SensorBlock.mat";
        const string LabelMatPath = "Assets/Materials/SensorLabelOverlay.mat";
        const string ReferenceChassis = "Assets/Robots/Chassis/chassis_2024.prefab";
        const string CatalogPath = "Assets/Resources/RoverCatalog.asset";
        const string ArmPrefab = "Assets/Robots/Arm/Arm.prefab";

        // Same order/names as chassis_2024's MotorController (= /motor_commands order).
        static readonly string[] SlotNames = {
            "back_left_wheel_link", "back_right_wheel_link", "left_front_wheel_link",
            "left_mid_wheel_link", "right_front_wheel_link", "right_mid_wheel_link",
        };

        static readonly string[] WheelLinks = { "WFL_Link", "WML_Link", "WBL_Link", "WFR_Link", "WMR_Link", "WBR_Link" };

        static Vector3 Flu(float x, float y, float z) => new Vector3(-y, z, x);   // ROS FLU -> Unity link frame

        /// <summary>One rover. Sensor positions are base_link-local (Unity axes); <c>rel(link)</c> gives a URDF link's origin there.</summary>
        public class Def
        {
            public string name, urdf, prefab, catalogLabel;
            public string[] dropLinks;                                   // sensor links replaced by blocks (their mass goes to base_link)
            public Func<Func<string, Vector3>, Vector3> lidar, imu, gps, camera;
            public Vector3 lidarSize, lidarCentre, imuSize, imuCentre, gpsSize, gpsCentre, cameraSize;
            public string cameraLabel = "Camera";
            public float lidarRange = 40f;
            public Func<WheelSpec> wheel;
            public Func<DriveMotorSpec> motor;
            public double turnSlipFactor;
            public float differentialMultiplier;                         // URDF <mimic> multiplier of Differential_joint
            public float totalMassKg;                                    // 0 = keep the URDF masses
            public int maxHullsPerLink;                                  // 0 = keep the importer's VHACD (up to 1024 hulls per mesh)
            public float diffStiffness = 50f, diffDamping = 2f;          // RockerDifferential per rocker (N·m/rad, N·m·s/rad)
            public Action<string, GameObject, Transform> decorate;      // (tag, robot, base_link): looks, extra colliders
            public Vector3? armMount;                                    // link_0 of the stepper arm, base_link-local (Unity); null = no arm
        }

        public static readonly Def MiniRover = new Def
        {
            name = "mini_rover", catalogLabel = "MINI ROVER",
            urdf = "Assets/Robots/Descriptions/mini_rover/mini_rover.urdf",
            prefab = "Assets/Robots/Chassis/mini_rover.prefab",
            dropLinks = new[] { "LiDAR_Link", "IMU_Link", "GPS1_Link", "GPS2_Link" },   // GPS1 = receiver board: only the antenna matters
            // LiDAR (Scanse Sweep): frame on the scan plane; the housing reaches 41.5 mm below / 20.4 mm above it.
            lidar = rel => rel("lidar_scan_link"), lidarSize = Flu(0.065f, 0.065f, 0.0619f), lidarCentre = Flu(0, 0, -0.01055f),
            imu = rel => rel("IMU_Link"), imuSize = Flu(0.04f, 0.04f, 0.014f), imuCentre = Flu(0, 0, -0.007f),
            gps = rel => rel("GPS2_Link"), gpsSize = Flu(0.025f, 0.025f, 0.005f), gpsCentre = Flu(0, 0, -0.0025f),
            // Camera: not in the URDF. On the top plate behind the lidar, looking forward. RealSense-sized block.
            camera = rel => Flu(0.20f, 0f, 0.1025f), cameraSize = Flu(0.025f, 0.09f, 0.025f),
            wheel = () => DrivetrainSetup.MiniRoverWheel, motor = () => DrivetrainSetup.MiniMotor,
            // Skid-steer slip, tuned so a commanded turn matches the IMU on CustomDirt.
            turnSlipFactor = 2.2, differentialMultiplier = 0.25f,
        };

        /// <summary>The 2026 big rover. See RoverFlake2 rover_description/CLAUDE.md ("Big rover") for what in its URDF is
        /// measured and what is a placeholder (wheels r = 0.1 m, inertials, limits).</summary>
        public static readonly Def Chassis2026 = new Def
        {
            name = "chassis_2026", catalogLabel = "CHASSIS 2026",
            urdf = "Assets/Robots/Descriptions/chassis_2026/chassis_2026.urdf",
            prefab = DrivetrainSetup.Chassis2026,
            dropLinks = new[] { "LiDAR_Link", "IMU_Link", "GPS_Left_Link", "GPS_Right_Link" },
            // Same Scanse Sweep as the mini rover, standing on LiDAR_Link's origin (its mesh: 65 mm housing centred 22 mm
            // behind the origin, 61.9 mm tall), so the scan plane is 41.5 mm up.
            lidar = rel => rel("LiDAR_Link") + Flu(-0.022f, 0, 0.0415f), lidarSize = Flu(0.065f, 0.065f, 0.0619f), lidarCentre = Flu(0, 0, -0.01055f),
            // IMU and GPS meshes are 19 x 25 x 19 mm boxes with a corner on the link origin: block on the box.
            imu = rel => rel("IMU_Link") + Flu(0.0095f, 0.0125f, 0.0095f), imuSize = Flu(0.019f, 0.025f, 0.019f), imuCentre = Vector3.zero,
            // One /gps/fix: the left antenna (the right one is for heading on the real rover; not simulated).
            gps = rel => rel("GPS_Left_Link") + Flu(0.0095f, -0.0125f, 0.019f), gpsSize = Flu(0.019f, 0.025f, 0.019f), gpsCentre = Flu(0, 0, -0.0095f),
            // Camera: on the front of the PTZ head at the top of the mast (the PTZ link stays as the mast's visual + collider),
            // looking straight forward. Pan/tilt is not simulated.
            camera = rel => Flu(-0.555f, 0.24f, 0.955f), cameraSize = Flu(0.025f, 0.06f, 0.03f), cameraLabel = "Camera (PTZ)",
            wheel = () => DrivetrainSetup.Chassis2026Wheel, motor = () => DrivetrainSetup.MaxonMotor,
            turnSlipFactor = DrivetrainSetup.Chassis2026TurnSlip, differentialMultiplier = -0.3f,
            totalMassKg = DrivetrainSetup.Chassis2026MassKg,
            // The importer gave 527 hulls (base_link 160, rockers 69-93): PhysX ran at half chassis_2024's speed and
            // RobotPhysicsSetup's self-collision pairs grow with the square of that.
            maxHullsPerLink = 16,
            // The real differential is a rigid linkage. At the default 50 N·m/rad the body (centre of mass ~12 cm behind the
            // rocker pivot) sagged 11.6 deg nose-up; 20000 leaves ~0.1 deg.
            diffStiffness = 20000f, diffDamping = 200f,
            decorate = Chassis2026Body.Apply,
            // Arm on the top panel's front-right corner (photo): motor hole 10 cm behind the front, 19 cm right of centre. The base
            // plate (x +7.5 / -19.5 cm, y +-7.5 cm around the motor hole) then clears the LiDAR, IMU and right GPS antenna; its
            // underside (link_0 + 10 cm) stands on the panel; the motor goes down into the box.
            armMount = Flu(-0.10f, -0.19f, Chassis2026Body.TopSurface - 0.10f),
        };

        [MenuItem("SimFlake/Build Mini Rover Prefab (from URDF)", priority = 310)]
        public static void BuildMenu() => BuildWithDialog(MiniRover);

        [MenuItem("SimFlake/Build Chassis 2026 Prefab (from URDF)", priority = 311)]
        public static void BuildChassis2026Menu() => BuildWithDialog(Chassis2026);

        static void BuildWithDialog(Def def)
        {
            if (AssetDatabase.LoadAssetAtPath<GameObject>(def.prefab) != null &&
                !EditorUtility.DisplayDialog("Rebuild " + def.name, $"Overwrite {def.prefab}? Sensor positions saved in the prefab are reset.", "Rebuild", "Cancel"))
                return;
            Build(def);
        }

        public static void BuildFromCommandLine() => RunAndExit(MiniRover);
        public static void BuildChassis2026FromCommandLine() => RunAndExit(Chassis2026);

        static void RunAndExit(Def def)
        {
            bool ok = false;
            try { ok = Build(def); }
            catch (Exception e) { Debug.LogException(e); }
            EditorApplication.Exit(ok ? 0 : 1);
        }

        public static bool Build() => Build(MiniRover);

        public static bool Build(Def def)
        {
            string tag = $"[{def.name}]";
            DrivetrainSetup.EnsureAssets();
            var settings = new Unity.Robotics.UrdfImporter.ImportSettings
            {
                chosenAxis = Unity.Robotics.UrdfImporter.ImportSettings.axisType.yAxis,
                convexMethod = Unity.Robotics.UrdfImporter.ImportSettings.convexDecomposer.vHACD,
            };
            GameObject robot = null;
            // Absolute path: the importer's GetRelativeAssetPath treats any path longer than Application.dataPath as absolute
            // and cuts that many characters off (a relative "Assets/..." path only survives if it is short enough).
            string urdfAbs = System.IO.Path.GetFullPath(System.IO.Path.Combine(Application.dataPath, "..", def.urdf));
            var it = Unity.Robotics.UrdfImporter.UrdfRobotExtensions.Create(urdfAbs, settings);
            while (it.MoveNext()) robot = it.Current ?? robot;
            if (robot == null) { Debug.LogError(tag + " URDF import failed"); return false; }
            robot.name = def.name;

            // The importer's keyboard joint controller overwrites drive settings at Start: not wanted.
            foreach (var c in robot.GetComponentsInChildren<Unity.Robotics.UrdfImporter.Control.Controller>(true))
                UnityEngine.Object.DestroyImmediate(c);

            Transform Find(string n) => robot.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => t.name == n)
                                        ?? throw new Exception($"{tag} link not found: " + n);
            var baseLink = Find("base_link");
            var baseBody = baseLink.GetComponent<ArticulationBody>();
            baseBody.immovable = false;

            // Sensor frames from the URDF, relative to base_link, before the links are removed.
            Vector3 Rel(string n) => baseLink.InverseTransformPoint(Find(n).position);
            Vector3 lidarPos = def.lidar(Rel), imuPos = def.imu(Rel), gpsPos = def.gps(Rel), camPos = def.camera(Rel);
            float removedMass = 0;
            foreach (var n in def.dropLinks)
            {
                var t = Find(n);
                removedMass += t.GetComponent<ArticulationBody>()?.mass ?? 0;
                UnityEngine.Object.DestroyImmediate(t.gameObject);
            }
            baseBody.mass += removedMass; // keep the total mass; the blocks themselves are massless colliders

            // Pose each side so all three wheels are level on flat ground (the mini rover's URDF zero pose has the front
            // wheels ~15 cm up); that pose becomes the joint zero, so the body stands level and the limits are centred on it.
            StandOnFlatGround(tag, Find("RockerL_Link"), Find("BogieL_Link"), Find("WFL_Link"), Find("WML_Link"), Find("WBL_Link"));
            StandOnFlatGround(tag, Find("RockerR_Link"), Find("BogieR_Link"), Find("WFR_Link"), Find("WMR_Link"), Find("WBR_Link"));

            // ---------- sensors ----------
            var blockMat = EnsureBlockMaterial();
            var labelMat = EnsureLabelMaterial();
            var sensors = new GameObject("Sensors").transform;
            sensors.SetParent(baseLink, false);

            var lidar = MakeSensor(sensors, "LiDAR", "LiDAR", new Color(0.95f, 0.25f, 0.2f), lidarPos, def.lidarSize, def.lidarCentre, blockMat, labelMat);
            var scanner = lidar.AddComponent<HorizontalLidarScanner>();
            scanner.topic = "/scan"; scanner.frameid = "lidar";
            scanner.numberOfRays = 720; scanner.scanInterval = 0.1f;
            scanner.scanRadius = def.lidarRange; scanner.max = (int)def.lidarRange; scanner.min = 0; // Sweep V1: 40 m

            var imu = MakeSensor(sensors, "IMU", "IMU", new Color(0.2f, 0.8f, 0.3f), imuPos, def.imuSize, def.imuCentre, blockMat, labelMat);
            var imuS = imu.AddComponent<ImuSensor>();
            imuS.topic = "/imu/data"; imuS.frameId = "imu_link";

            var gps = MakeSensor(sensors, "GPS", "GPS", new Color(0.2f, 0.45f, 1f), gpsPos, def.gpsSize, def.gpsCentre, blockMat, labelMat);
            var gpsS = gps.AddComponent<LocationStuff>();
            gpsS.topic = "/gps/fix"; gpsS.frameId = "gnss"; // object_to_track empty = the block itself

            var cam = MakeSensor(sensors, "Camera", def.cameraLabel, new Color(1f, 0.8f, 0.1f), camPos, def.cameraSize, Vector3.zero, blockMat, labelMat);
            var camera = cam.AddComponent<Camera>();
            camera.fieldOfView = 58f; camera.nearClipPlane = 0.02f; camera.farClipPlane = 500f; camera.depth = -5;
            var raw = cam.AddComponent<RGBCameraCapture>();
            raw.rgbCamera = camera; raw.frameid = "camera_link";
            raw.imageTopic = "/camera/image_raw"; raw.cameraInfoTopic = "/camera/camera_info";
            var jpg = cam.AddComponent<CompressedCameraCapture>();
            jpg.rgbCamera = camera; jpg.frameid = "camera_link"; jpg.topic = "/camera/image/compressed";
            jpg.enabled = false; // same as the scenes (team choice); enable for a JPEG feed

            def.decorate?.Invoke(tag, robot, baseLink);
            if (def.maxHullsPerLink > 0) CapHulls(tag, robot, def);

            // ---------- drive ----------
            var wheelSpec = def.wheel();
            var wheelMesh = DrivetrainSetup.CylinderMesh();
            var wheels = WheelLinks
                .Select(n => Find(n).GetComponent<ArticulationBody>()).ToArray();
            foreach (var w in wheels) DrivetrainSetup.ApplyWheelSpec(w, wheelSpec, wheelMesh);

            float refAxisSign = ReferenceForwardAxisSign(tag);
            var slots = new ArticulationBody[6];
            var isLeft = new bool[6];
            var dirs = new int[6];
            // Front / mid / back by order along the body on each side (base_link's origin isn't always mid-body).
            var placed = wheels.Select(w => (w, p: baseLink.InverseTransformPoint(WheelCentre(w)))).ToArray();
            foreach (var side in placed.GroupBy(x => x.p.x < 0))     // Unity +x = right
            {
                var rows = side.OrderByDescending(x => x.p.z).ToArray();
                if (rows.Length != 3) throw new Exception($"{tag} expected 3 wheels per side, got {rows.Length}");
                for (int r = 0; r < 3; r++)
                {
                    var (w, p) = rows[r];
                    bool left = side.Key;
                    string row = new[] { "front", "mid", "back" }[r];
                    string slot = row == "back" ? $"back_{(left ? "left" : "right")}_wheel_link" : $"{(left ? "left" : "right")}_{row}_wheel_link";
                    int i = Array.IndexOf(SlotNames, slot);
                    slots[i] = w; isLeft[i] = left;
                    // Positive joint speed moves the rover forward when the axis points the same way as chassis_2024's left wheels.
                    int fwd = Math.Sign(Vector3.Dot(JointAxisWorld(w), baseLink.right)) == Math.Sign(refAxisSign) ? 1 : -1;
                    dirs[i] = fwd * (left ? 1 : -1);
                    Debug.Log($"{tag} {slot} = {w.name} at {p:F3} dir {dirs[i]}");
                }
            }

            var mc = robot.AddComponent<MotorController>();
            var so = new SerializedObject(mc);
            for (int i = 0; i < 6; i++)
            {
                so.FindProperty("wheelJointNames").GetArrayElementAtIndex(i).stringValue = SlotNames[i];
                so.FindProperty("wheelJoints").GetArrayElementAtIndex(i).objectReferenceValue = slots[i];
                so.FindProperty("isLeftWheel").GetArrayElementAtIndex(i).boolValue = isLeft[i];
                so.FindProperty("motor_dirs").GetArrayElementAtIndex(i).intValue = dirs[i];
            }
            so.FindProperty("odomRateHz").floatValue = 50f;
            so.FindProperty("turnSlipFactor").doubleValue = def.turnSlipFactor;
            so.ApplyModifiedPropertiesWithoutUndo();

            // Motor model on every wheel, HUD on the root.
            DrivetrainSetup.AttachDrivetrain(robot, wheels, def.motor(), wheelSpec);
            if (def.totalMassKg > 0) DrivetrainSetup.ScaleBodyMass(robot, wheels, def.totalMassKg);
            robot.AddComponent<RobotPhysicsSetup>();
            var diff = robot.AddComponent<RockerDifferential>();
            diff.leftRocker = Find("RockerL_Link").GetComponent<ArticulationBody>();
            diff.rightRocker = Find("RockerR_Link").GetComponent<ArticulationBody>();
            diff.differential = Find("Differential_Link").GetComponent<ArticulationBody>();
            diff.differentialMultiplier = def.differentialMultiplier;
            diff.stiffness = def.diffStiffness; diff.damping = def.diffDamping;

            // Stepper arm mount (Esc menu > ROVER AND WORLD > ROVER > CONFIGURATION, --rover-config arm), like chassis_2024's.
            if (def.armMount.HasValue)
            {
                var mount = new GameObject("ArmMount").transform;
                mount.SetParent(baseLink, false);
                mount.localPosition = def.armMount.Value;
                mount.localRotation = Quaternion.identity;   // same axes as chassis_2024's chassis_link: the arm faces forward
                var payload = robot.AddComponent<RoverPayload>();
                payload.armMount = mount;
                payload.armPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(ArmPrefab) ?? throw new Exception($"{tag} {ArmPrefab} not found");
            }

            foreach (var b in robot.GetComponentsInChildren<ArticulationBody>())
                Debug.Log($"{tag} body {b.name}: {b.jointType} mass {b.mass:F3} immovable {b.immovable} drive k{b.xDrive.stiffness} d{b.xDrive.damping} f{b.xDrive.forceLimit}");

            PrefabUtility.SaveAsPrefabAsset(robot, def.prefab, out bool ok);
            UnityEngine.Object.DestroyImmediate(robot);
            if (ok) AddToCatalog(def);
            if (ok && def.maxHullsPerLink > 0) DeleteUnusedHulls(tag, def);
            AssetDatabase.SaveAssets();
            Debug.Log(ok ? $"{tag} saved {def.prefab}" : $"{tag} FAILED to save prefab");
            return ok;
        }

        static string HullDir(Def def) => System.IO.Path.GetDirectoryName(def.urdf).Replace('\\', '/') + "/Hulls";

        /// <summary>Re-decomposes every link (not wheels: their colliders are replaced by the WheelSpec) that has more than
        /// <see cref="Def.maxHullsPerLink"/> convex hulls, from its visual mesh, with VHACD capped at that many hulls.</summary>
        static void CapHulls(string tag, GameObject robot, Def def)
        {
            string dir = HullDir(def);
            if (AssetDatabase.IsValidFolder(dir)) AssetDatabase.DeleteAsset(dir);
            AssetDatabase.CreateFolder(System.IO.Path.GetDirectoryName(dir).Replace('\\', '/'), "Hulls");
            int before = 0, after = 0;
            foreach (var body in robot.GetComponentsInChildren<ArticulationBody>(true))
            {
                if (WheelLinks.Contains(body.name)) continue;
                var cols = body.GetComponentsInChildren<MeshCollider>(true).Where(c => c.GetComponentInParent<ArticulationBody>() == body).ToList();
                before += cols.Count;
                if (cols.Count <= def.maxHullsPerLink) { after += cols.Count; continue; }
                var host = cols[0].transform;
                var sensorsT = body.transform.Find("Sensors");   // sensor blocks and labels are not part of the link
                var filters = body.GetComponentsInChildren<MeshFilter>(true)
                                  .Where(f => f.sharedMesh != null && f.GetComponentInParent<ArticulationBody>() == body
                                              && (sensorsT == null || !f.transform.IsChildOf(sensorsT))).ToArray();
                var verts = new List<Vector3>(); var tris = new List<int>();
                foreach (var f in filters)
                {
                    int o = verts.Count;
                    verts.AddRange(f.sharedMesh.vertices.Select(v => host.InverseTransformPoint(f.transform.TransformPoint(v))));
                    tris.AddRange(f.sharedMesh.triangles.Select(i => i + o));
                }
                if (verts.Count == 0) { Debug.LogWarning($"{tag} {body.name}: no visual mesh, keeping {cols.Count} hulls"); after += cols.Count; continue; }
                var src = new Mesh { indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
                src.SetVertices(verts); src.SetTriangles(tris, 0);
                var tmp = new GameObject("vhacd");
                var vh = tmp.AddComponent<MeshProcess.VHACD>();
                vh.m_parameters.m_maxConvexHulls = (uint)def.maxHullsPerLink;
                var hulls = vh.GenerateConvexMeshes(src);
                UnityEngine.Object.DestroyImmediate(tmp);
                if (hulls == null || hulls.Count == 0) { Debug.LogWarning($"{tag} {body.name}: VHACD gave nothing, keeping {cols.Count} hulls"); after += cols.Count; continue; }
                var mat = cols[0].sharedMaterial;
                foreach (var c in cols) UnityEngine.Object.DestroyImmediate(c);
                for (int k = 0; k < hulls.Count; k++)
                {
                    AssetDatabase.CreateAsset(hulls[k], $"{dir}/{body.name}_{k}.asset");
                    var mc = host.gameObject.AddComponent<MeshCollider>();
                    mc.sharedMesh = hulls[k]; mc.convex = true; mc.sharedMaterial = mat;
                }
                Debug.Log($"{tag} {body.name}: {cols.Count} -> {hulls.Count} convex hulls");
                after += hulls.Count;
            }
            Debug.Log($"{tag} collision hulls {before} -> {after}");
        }

        /// <summary>The importer's own hull assets (<c>&lt;stl&gt;_1.asset</c>, <c>_2</c>, ...) for the re-decomposed links are no
        /// longer referenced: delete them. Keep everything the rover prefab or an STL prefab (<c>&lt;stl&gt;.prefab</c> +
        /// <c>&lt;stl&gt;_0.asset</c>, made by the importer, needed by the next import) uses.</summary>
        static void DeleteUnusedHulls(string tag, Def def)
        {
            string meshes = System.IO.Path.GetDirectoryName(def.urdf).Replace('\\', '/') + "/rover_description/meshes";
            var roots = AssetDatabase.FindAssets("t:Prefab", new[] { meshes }).Select(AssetDatabase.GUIDToAssetPath).Append(def.prefab).ToArray();
            var used = new HashSet<string>(AssetDatabase.GetDependencies(roots, true));
            var unused = AssetDatabase.FindAssets("t:Mesh", new[] { meshes }).Select(AssetDatabase.GUIDToAssetPath)
                                      .Where(p => p.EndsWith(".asset") && !used.Contains(p)).Distinct().ToList();
            foreach (var p in unused) AssetDatabase.DeleteAsset(p);
            Debug.Log($"{tag} deleted {unused.Count} unused importer hull assets");
        }

        // The Esc menu's rover swap list (and what puts the prefab into player builds).
        static void AddToCatalog(Def def)
        {
            var cat = AssetDatabase.LoadAssetAtPath<RoverCatalog>(CatalogPath);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(def.prefab);
            if (cat == null || prefab == null || cat.rovers.Any(e => e.prefab == prefab)) return;
            cat.rovers.Add(new RoverCatalog.Entry { label = def.catalogLabel, prefab = prefab });
            EditorUtility.SetDirty(cat);
            Debug.Log($"[{def.name}] added to {CatalogPath} as {def.catalogLabel}");
        }

        static GameObject MakeSensor(Transform parent, string name, string label, Color color, Vector3 pos,
                                     Vector3 size, Vector3 centre, Material blockMat, Material labelMat)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.localPosition = pos;
            go.transform.localRotation = Quaternion.identity;

            var blockGo = GameObject.CreatePrimitive(PrimitiveType.Cube); // MeshFilter + MeshRenderer + BoxCollider
            blockGo.name = "Block";
            blockGo.transform.SetParent(go.transform, false);
            blockGo.GetComponent<MeshRenderer>().sharedMaterial = blockMat;

            var labelGo = new GameObject("Label");
            labelGo.transform.SetParent(go.transform, false);
            var tmp = labelGo.AddComponent<TextMeshPro>();
            tmp.font = TMP_Settings.defaultFontAsset;
            tmp.fontSharedMaterial = labelMat;
            tmp.alignment = TextAlignmentOptions.Bottom;
            tmp.enableWordWrapping = false;
            tmp.fontStyle = FontStyles.Bold;
            tmp.rectTransform.sizeDelta = new Vector2(0.3f, 0.05f);
            labelGo.GetComponent<MeshRenderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;

            var sb = go.AddComponent<SensorBlock>();
            sb.label = label; sb.color = color; sb.size = new Vector3(Mathf.Abs(size.x), Mathf.Abs(size.y), Mathf.Abs(size.z));
            sb.center = centre; sb.block = blockGo.transform; sb.labelText = tmp;
            sb.Apply();
            labelGo.transform.localPosition = centre + Vector3.up * (sb.size.y / 2 + sb.labelGap);
            return go;
        }

        static void StandOnFlatGround(string tag, Transform rocker, Transform bogie, Transform front, Transform mid, Transform back)
        {
            var rb = rocker.GetComponent<ArticulationBody>();
            var bb = bogie.GetComponent<ArticulationBody>();
            Vector3 rPos = rocker.localPosition, bPos = bogie.localPosition;
            Quaternion rRot = rocker.localRotation, bRot = bogie.localRotation;

            // Wheel centres (joint origins) are what must be level; all wheels have the same radius.
            float LevelBogie(float beta)
            {
                bogie.localPosition = bPos; bogie.localRotation = bRot;
                bogie.RotateAround(bogie.TransformPoint(bb.anchorPosition), JointAxisWorld(bb), beta);
                return mid.position.y - back.position.y;
            }
            float PoseRocker(float alpha)
            {
                rocker.localPosition = rPos; rocker.localRotation = rRot;
                rocker.RotateAround(rocker.TransformPoint(rb.anchorPosition), JointAxisWorld(rb), alpha);
                Solve(LevelBogie, -60, 60);
                return front.position.y - 0.5f * (mid.position.y + back.position.y);
            }
            float a = Solve(PoseRocker, -60, 60);
            PoseRocker(a);
            float b = Vector3.SignedAngle(bogie.localRotation * Vector3.forward, bRot * Vector3.forward, Vector3.right);
            Debug.Log($"{tag} {rocker.name} posed {a:F2} deg (bogie {-b:F2} deg); wheel heights {front.position.y:F4} {mid.position.y:F4} {back.position.y:F4}");

            foreach (var body in new[] { rb, bb })
            {
                var parent = body.transform.parent;
                body.matchAnchors = true;
                body.parentAnchorPosition = parent.InverseTransformPoint(body.transform.TransformPoint(body.anchorPosition));
                body.parentAnchorRotation = Quaternion.Inverse(parent.rotation) * body.transform.rotation * body.anchorRotation;
            }
        }

        // Bisection for f(x) = 0 on [lo, hi] (f changes sign there).
        static float Solve(Func<float, float> f, float lo, float hi)
        {
            float flo = f(lo);
            for (int i = 0; i < 50; i++)
            {
                float m = 0.5f * (lo + hi), fm = f(m);
                if (Math.Sign(fm) == Math.Sign(flo)) { lo = m; flo = fm; } else hi = m;
            }
            return 0.5f * (lo + hi);
        }

        static Vector3 WheelCentre(ArticulationBody w)
        {
            var cols = w.GetComponentsInChildren<Collider>();
            if (cols.Length == 0) return w.transform.position;
            var b = cols[0].bounds;
            foreach (var c in cols) b.Encapsulate(c.bounds);
            return b.center;
        }

        // An articulation's joint axis is the x axis of its anchor frame.
        static Vector3 JointAxisWorld(ArticulationBody b) => b.transform.rotation * (b.anchorRotation * Vector3.right);

        // chassis_2024 drives with motor_dirs = 1, so its left wheels' axis direction means "positive = forward".
        static float ReferenceForwardAxisSign(string tag)
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(ReferenceChassis);
            var inst = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            try
            {
                var w = inst.GetComponentsInChildren<ArticulationBody>().First(b => b.name == "left_front_wheel_link");
                var body = inst.GetComponentsInChildren<ArticulationBody>().First(b => b.isRoot);
                // chassis_link's forward is Unity +z like every imported robot; right = +x.
                float s = Vector3.Dot(JointAxisWorld(w), body.transform.right);
                Debug.Log($"{tag} reference left wheel axis . right = {s:F2}");
                return s;
            }
            finally { UnityEngine.Object.DestroyImmediate(inst); }
        }

        static Material EnsureBlockMaterial()
        {
            var m = AssetDatabase.LoadAssetAtPath<Material>(BlockMatPath);
            if (m != null) return m;
            m = new Material(Shader.Find("Standard")) { color = Color.white };
            m.SetFloat("_Glossiness", 0.3f);
            AssetDatabase.CreateAsset(m, BlockMatPath);
            return m;
        }

        static Material EnsureLabelMaterial()
        {
            var m = AssetDatabase.LoadAssetAtPath<Material>(LabelMatPath);
            if (m != null) return m;
            m = new Material(TMP_Settings.defaultFontAsset.material) { shader = Shader.Find("TextMeshPro/Distance Field Overlay") };
            m.EnableKeyword("OUTLINE_ON");
            m.SetColor("_OutlineColor", Color.black);
            m.SetFloat("_OutlineWidth", 0.2f);
            AssetDatabase.CreateAsset(m, LabelMatPath);
            return m;
        }
    }
}
