using System;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using UnityEditor;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Builds Assets/Robots/Chassis/mini_rover.prefab from the mini rover URDF (RoverFlake2 rover_description).
    ///
    /// The URDF's sensor links (LiDAR, IMU, GPS antenna) are replaced by <see cref="SensorBlock"/>s: coloured,
    /// labelled boxes that carry the sensor script, so a sensor can be moved by dragging its block. A camera block
    /// is added (the URDF has none). GPS1 (the receiver board) is dropped: only the antenna position matters.
    /// Topics and frame ids are the same as chassis_2024's.
    ///
    /// Re-running overwrites the prefab (and any sensor moves saved in it). Scene instances keep their overrides.
    ///   Unity -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.MiniRoverBuilder.BuildFromCommandLine
    /// </summary>
    public static class MiniRoverBuilder
    {
        const string Urdf = "Assets/Robots/Descriptions/mini_rover/mini_rover.urdf";
        const string PrefabPath = "Assets/Robots/Chassis/mini_rover.prefab";
        const string BlockMatPath = "Assets/Materials/SensorBlock.mat";
        const string LabelMatPath = "Assets/Materials/SensorLabelOverlay.mat";
        const string WheelMeshPath = "Assets/Robots/Descriptions/mini_rover/wheel_tread_collider.asset";
        const string WheelPhysicsPath = "Assets/Materials/MiniRoverWheel.physicMaterial";
        const string ReferenceChassis = "Assets/Robots/Chassis/chassis_2024.prefab";

        // Same order/names as chassis_2024's MotorController (= /motor_commands order).
        static readonly string[] SlotNames = {
            "back_left_wheel_link", "back_right_wheel_link", "left_front_wheel_link",
            "left_mid_wheel_link", "right_front_wheel_link", "right_mid_wheel_link",
        };

        [MenuItem("SimFlake/Build Mini Rover Prefab (from URDF)", priority = 310)]
        public static void BuildMenu()
        {
            if (AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath) != null &&
                !EditorUtility.DisplayDialog("Rebuild mini rover", $"Overwrite {PrefabPath}? Sensor positions saved in the prefab are reset.", "Rebuild", "Cancel"))
                return;
            Build();
        }

        public static void BuildFromCommandLine()
        {
            bool ok = false;
            try { ok = Build(); }
            catch (Exception e) { Debug.LogException(e); }
            EditorApplication.Exit(ok ? 0 : 1);
        }

        static Vector3 Flu(float x, float y, float z) => new Vector3(-y, z, x);   // ROS FLU -> Unity link frame

        public static bool Build()
        {
            var settings = new Unity.Robotics.UrdfImporter.ImportSettings
            {
                chosenAxis = Unity.Robotics.UrdfImporter.ImportSettings.axisType.yAxis,
                convexMethod = Unity.Robotics.UrdfImporter.ImportSettings.convexDecomposer.vHACD,
            };
            GameObject robot = null;
            var it = Unity.Robotics.UrdfImporter.UrdfRobotExtensions.Create(Urdf, settings);
            while (it.MoveNext()) robot = it.Current ?? robot;
            if (robot == null) { Debug.LogError("[MiniRover] URDF import failed"); return false; }
            robot.name = "mini_rover";

            // The importer's keyboard joint controller overwrites drive settings at Start: not wanted.
            foreach (var c in robot.GetComponentsInChildren<Unity.Robotics.UrdfImporter.Control.Controller>(true))
                UnityEngine.Object.DestroyImmediate(c);

            Transform Find(string n) => robot.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => t.name == n)
                                        ?? throw new Exception("[MiniRover] link not found: " + n);
            var baseLink = Find("base_link");
            var baseBody = baseLink.GetComponent<ArticulationBody>();
            baseBody.immovable = false;

            // Sensor frames from the URDF, relative to base_link, before the links are removed.
            Vector3 Rel(string n) => baseLink.InverseTransformPoint(Find(n).position);
            Vector3 lidarPos = Rel("lidar_scan_link"), imuPos = Rel("IMU_Link"), gpsPos = Rel("GPS2_Link");
            float removedMass = 0;
            foreach (var n in new[] { "LiDAR_Link", "IMU_Link", "GPS1_Link", "GPS2_Link" })
            {
                var t = Find(n);
                removedMass += t.GetComponent<ArticulationBody>()?.mass ?? 0;
                UnityEngine.Object.DestroyImmediate(t.gameObject);
            }
            baseBody.mass += removedMass; // keep the total mass; the blocks themselves are massless colliders

            // The URDF's zero pose isn't a standing pose (front wheels ~15 cm above the bogie wheels). Pose each side
            // so all three wheels are level on flat ground; that pose becomes the joint zero, so the body stands level
            // and the rocker/bogie limits are centred on it.
            StandOnFlatGround(Find("RockerL_Link"), Find("BogieL_Link"), Find("WFL_Link"), Find("WML_Link"), Find("WBL_Link"));
            StandOnFlatGround(Find("RockerR_Link"), Find("BogieR_Link"), Find("WFR_Link"), Find("WMR_Link"), Find("WBR_Link"));

            // ---------- sensors ----------
            var blockMat = EnsureBlockMaterial();
            var labelMat = EnsureLabelMaterial();
            var sensors = new GameObject("Sensors").transform;
            sensors.SetParent(baseLink, false);

            // LiDAR (Scanse Sweep): frame on the scan plane; the housing reaches 41.5 mm below / 20.4 mm above it.
            var lidar = MakeSensor(sensors, "LiDAR", "LiDAR", new Color(0.95f, 0.25f, 0.2f), lidarPos,
                                   Flu(0.065f, 0.065f, 0.0619f), Flu(0, 0, -0.01055f), blockMat, labelMat);
            var scanner = lidar.AddComponent<HorizontalLidarScanner>();
            scanner.topic = "/scan"; scanner.frameid = "lidar";
            scanner.numberOfRays = 720; scanner.scanInterval = 0.1f;
            scanner.scanRadius = 40f; scanner.max = 40; scanner.min = 0; // Sweep V1: 40 m

            var imu = MakeSensor(sensors, "IMU", "IMU", new Color(0.2f, 0.8f, 0.3f), imuPos,
                                 Flu(0.04f, 0.04f, 0.014f), Flu(0, 0, -0.007f), blockMat, labelMat);
            var imuS = imu.AddComponent<ImuSensor>();
            imuS.topic = "/imu/data"; imuS.frameId = "imu_link";

            var gps = MakeSensor(sensors, "GPS", "GPS", new Color(0.2f, 0.45f, 1f), gpsPos,
                                 Flu(0.025f, 0.025f, 0.005f), Flu(0, 0, -0.0025f), blockMat, labelMat);
            var gpsS = gps.AddComponent<LocationStuff>();
            gpsS.topic = "/gps/fix"; gpsS.frameId = "gnss"; // object_to_track empty = the block itself

            // Camera: not in the URDF. On the top plate behind the lidar, looking forward. RealSense-sized block.
            var cam = MakeSensor(sensors, "Camera", "Camera", new Color(1f, 0.8f, 0.1f), Flu(0.20f, 0f, 0.1025f),
                                 Flu(0.025f, 0.09f, 0.025f), Vector3.zero, blockMat, labelMat);
            var camera = cam.AddComponent<Camera>();
            camera.fieldOfView = 58f; camera.nearClipPlane = 0.02f; camera.farClipPlane = 500f; camera.depth = -5;
            var raw = cam.AddComponent<RGBCameraCapture>();
            raw.rgbCamera = camera; raw.frameid = "camera_link";
            raw.imageTopic = "/camera/image_raw"; raw.cameraInfoTopic = "/camera/camera_info";
            var jpg = cam.AddComponent<CompressedCameraCapture>();
            jpg.rgbCamera = camera; jpg.frameid = "camera_link"; jpg.topic = "/camera/image/compressed";
            jpg.enabled = false; // same as the scenes (team choice); enable for a JPEG feed

            // ---------- drive ----------
            var wheelMesh = EnsureCylinderMesh();
            var wheelPhysics = EnsureWheelPhysics();
            var wheels = new[] { "WFL_Link", "WML_Link", "WBL_Link", "WFR_Link", "WMR_Link", "WBR_Link" }
                .Select(n => Find(n).GetComponent<ArticulationBody>()).ToArray();
            foreach (var w in wheels)
            {
                var d = w.xDrive;
                // Torque limit ~ a small gearmotor (N·m). chassis_2024 uses 50, which here flips the 70 g bogies
                // onto their limits when a skid turn starts. Rolling/scrub needs ~0.4 N·m per wheel; 30 deg climb ~0.4.
                d.stiffness = 0; d.damping = 1000f; d.forceLimit = 3f;
                w.xDrive = d;
                SmoothWheelCollider(w, wheelMesh, wheelPhysics);
            }

            float refAxisSign = ReferenceForwardAxisSign();
            var slots = new ArticulationBody[6];
            var isLeft = new bool[6];
            var dirs = new int[6];
            foreach (var w in wheels)
            {
                Vector3 p = baseLink.InverseTransformPoint(WheelCentre(w));
                bool left = p.x < 0;                    // Unity +x = right
                string row = p.z > 0.15f ? "front" : p.z > -0.1f ? "mid" : "back";
                string slot = row == "back" ? $"back_{(left ? "left" : "right")}_wheel_link" : $"{(left ? "left" : "right")}_{row}_wheel_link";
                int i = Array.IndexOf(SlotNames, slot);
                if (i < 0 || slots[i] != null) throw new Exception($"[MiniRover] can't place wheel {w.name} at {p} (slot {slot})");
                slots[i] = w; isLeft[i] = left;
                // Positive joint speed moves the rover forward when the axis points the same way as chassis_2024's left wheels.
                int fwd = Math.Sign(Vector3.Dot(JointAxisWorld(w), baseLink.right)) == Math.Sign(refAxisSign) ? 1 : -1;
                dirs[i] = fwd * (left ? 1 : -1);
                Debug.Log($"[MiniRover] {slot} = {w.name} at {p:F3} dir {dirs[i]}");
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
            // Skid-steer slip, tuned so a commanded turn matches the IMU on CustomDirt (chassis_2024 uses 2.3).
            so.FindProperty("turnSlipFactor").doubleValue = 2.2;
            so.ApplyModifiedPropertiesWithoutUndo();

            robot.AddComponent<RobotPhysicsSetup>();
            var diff = robot.AddComponent<RockerDifferential>();
            diff.leftRocker = Find("RockerL_Link").GetComponent<ArticulationBody>();
            diff.rightRocker = Find("RockerR_Link").GetComponent<ArticulationBody>();
            diff.differential = Find("Differential_Link").GetComponent<ArticulationBody>();

            foreach (var b in robot.GetComponentsInChildren<ArticulationBody>())
                Debug.Log($"[MiniRover] body {b.name}: {b.jointType} mass {b.mass:F3} immovable {b.immovable} drive k{b.xDrive.stiffness} d{b.xDrive.damping} f{b.xDrive.forceLimit}");

            PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath, out bool ok);
            UnityEngine.Object.DestroyImmediate(robot);
            AssetDatabase.SaveAssets();
            Debug.Log(ok ? $"[MiniRover] saved {PrefabPath}" : "[MiniRover] FAILED to save prefab");
            return ok;
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

        static void StandOnFlatGround(Transform rocker, Transform bogie, Transform front, Transform mid, Transform back)
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
            Debug.Log($"[MiniRover] {rocker.name} posed {a:F2} deg (bogie {-b:F2} deg); wheel heights {front.position.y:F4} {mid.position.y:F4} {back.position.y:F4}");

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

        // VHACD hulls of the grousered wheel are faceted and catch on the ground (skid turns stalled the drives).
        // Replace them with one smooth convex cylinder fitted to the tread (radius and width from the visual mesh).
        static void SmoothWheelCollider(ArticulationBody w, Mesh cylinder, PhysicMaterial physics)
        {
            Vector3 axis = (w.anchorRotation * Vector3.right).normalized;            // wheel-local joint axis
            Vector3 pivot = w.anchorPosition;
            var verts = w.GetComponentsInChildren<MeshFilter>()
                         .SelectMany(f => f.sharedMesh.vertices.Select(v => w.transform.InverseTransformPoint(f.transform.TransformPoint(v)) - pivot))
                         .ToArray();
            float rMax = verts.Max(v => Vector3.ProjectOnPlane(v, axis).magnitude);
            var tread = verts.Where(v => Vector3.ProjectOnPlane(v, axis).magnitude > 0.9f * rMax).Select(v => Vector3.Dot(v, axis)).ToArray();
            float a0 = tread.Min(), a1 = tread.Max();

            foreach (var c in w.GetComponentsInChildren<Collider>(true)) UnityEngine.Object.DestroyImmediate(c);
            var go = new GameObject("TreadCollider");
            go.transform.SetParent(w.transform, false);
            go.transform.localPosition = pivot + axis * (0.5f * (a0 + a1));
            go.transform.localRotation = Quaternion.FromToRotation(Vector3.up, axis);
            go.transform.localScale = new Vector3(2 * rMax, a1 - a0, 2 * rMax);   // unit cylinder: radius 0.5, height 1
            var mc = go.AddComponent<MeshCollider>();
            mc.sharedMesh = cylinder; mc.convex = true; mc.sharedMaterial = physics;
            Debug.Log($"[MiniRover] {w.name} tread collider r {rMax:F4} width {a1 - a0:F4}");
        }

        // Grip cap. The bogie pivot sits 14 cm above the ground but only 12.7 cm from each bogie wheel, so skid-steer
        // traction tips the bogies over ("wheelie") once friction gets near ~0.5: in-place turns flipped them onto
        // their limits and rolled the rover. Minimum combine: the terrain can be slipperier, never grippier.
        static PhysicMaterial EnsureWheelPhysics()
        {
            var m = AssetDatabase.LoadAssetAtPath<PhysicMaterial>(WheelPhysicsPath);
            if (m != null) return m;
            m = new PhysicMaterial("MiniRoverWheel") { dynamicFriction = 0.35f, staticFriction = 0.35f, frictionCombine = PhysicMaterialCombine.Minimum };
            AssetDatabase.CreateAsset(m, WheelPhysicsPath);
            return m;
        }

        static Mesh EnsureCylinderMesh()
        {
            var m = AssetDatabase.LoadAssetAtPath<Mesh>(WheelMeshPath);
            if (m != null) return m;
            const int n = 32;
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
            m = new Mesh { name = "wheel_tread_collider", vertices = v.ToArray(), triangles = tris.ToArray() };
            m.RecalculateNormals(); m.RecalculateBounds();
            AssetDatabase.CreateAsset(m, WheelMeshPath);
            return m;
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
        static float ReferenceForwardAxisSign()
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(ReferenceChassis);
            var inst = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            try
            {
                var w = inst.GetComponentsInChildren<ArticulationBody>().First(b => b.name == "left_front_wheel_link");
                var body = inst.GetComponentsInChildren<ArticulationBody>().First(b => b.isRoot);
                // chassis_link's forward is Unity +z like every imported robot; right = +x.
                float s = Vector3.Dot(JointAxisWorld(w), body.transform.right);
                Debug.Log($"[MiniRover] reference left wheel axis . right = {s:F2}");
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
