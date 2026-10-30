using System;
using System.IO;
using System.Linq;
using SimFlake.Ros;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Editor helpers. Also usable headless, e.g.
    ///   Unity -batchmode -quit -projectPath rover_sim -executeMethod SimFlake.EditorTools.SimFlakeTools.BuildLinux \
    ///         -scenes Assets/Scenes/ChassisPlayground.unity -out Builds/sim/rover_sim.x86_64
    /// </summary>
    public static class SimFlakeTools
    {
        [MenuItem("SimFlake/Build Linux Player (scenes in Build Settings)")]
        public static void BuildLinuxMenu()
        {
            var scenes = EditorBuildSettings.scenes.Where(s => s.enabled).Select(s => s.path).ToArray();
            if (scenes.Length == 0) scenes = new[] { UnityEngine.SceneManagement.SceneManager.GetActiveScene().path };
            Build(scenes, "Builds/SimFlake/rover_sim.x86_64", false);
        }

        public static void BuildLinux()
        {
            string scenesArg = Arg("-scenes");
            string outPath = Arg("-out") ?? "Builds/SimFlake/rover_sim.x86_64";
            string[] scenes = string.IsNullOrEmpty(scenesArg)
                ? EditorBuildSettings.scenes.Where(s => s.enabled).Select(s => s.path).ToArray()
                : scenesArg.Split(',');
            bool ok = Build(scenes, outPath, Environment.GetCommandLineArgs().Contains("-development"));
            if (Application.isBatchMode) EditorApplication.Exit(ok ? 0 : 1);
        }

        static bool Build(string[] scenes, string outPath, bool development)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(outPath)));
            var opts = new BuildPlayerOptions
            {
                scenes = scenes,
                locationPathName = outPath,
                target = BuildTarget.StandaloneLinux64,
                options = development ? BuildOptions.Development : BuildOptions.None,
            };
            Debug.Log("[SimFlake] Building " + string.Join(", ", scenes) + " -> " + outPath);
            BuildReport report = BuildPipeline.BuildPlayer(opts);
            Debug.Log($"[SimFlake] Build {report.summary.result}: {report.summary.totalSize / 1e6:F0} MB, {report.summary.totalErrors} errors");
            return report.summary.result == BuildResult.Succeeded;
        }

        static string Arg(string name)
        {
            var a = Environment.GetCommandLineArgs();
            for (int i = 0; i < a.Length - 1; i++) if (a[i] == name) return a[i + 1];
            return null;
        }

        static double TurnSlip => double.TryParse(Arg("-turnslip"), out var v) ? v : 2.3;

        static readonly string[] RoverScenes = {
            "Assets/Scenes/ChassisPlayground.unity",
            "Assets/Scenes/TaskWorlds/WaypointTask.unity",
            "Assets/Scenes/SampleScene.unity",
        };

        /// <summary>
        /// One-off migration of the rover scenes to the Jazzy sensor set. Pass -dryrun to only print.
        /// Adds an IMU to base_link, sets sane camera/lidar rates. Safe to run twice.
        /// </summary>
        public static void UpgradeScenes()
        {
            bool dry = Environment.GetCommandLineArgs().Contains("-dryrun");
            foreach (var path in RoverScenes)
            {
                var scene = UnityEditor.SceneManagement.EditorSceneManager.OpenScene(path);
                Debug.Log($"[Upgrade] === {path}");
                var all = Resources.FindObjectsOfTypeAll<Transform>().Where(t => t.gameObject.scene == scene).ToArray();
                var bases = all.Where(t => t.name == "base_link").ToArray();
                foreach (var b in bases)
                    Debug.Log($"[Upgrade] base_link at {GetPath(b)} articulation={b.GetComponent<ArticulationBody>() != null} imu={b.GetComponent<ImuSensor>() != null}");
                foreach (var mc in all.Select(t => t.GetComponent<MotorController>()).Where(c => c != null))
                {
                    Debug.Log($"[Upgrade] MotorController on {GetPath(mc.transform)}");
                    if (!dry)
                    {
                        var so = new SerializedObject(mc);
                        so.FindProperty("odomRateHz").floatValue = 50f;
                        so.FindProperty("turnSlipFactor").doubleValue = TurnSlip;
                        so.ApplyModifiedPropertiesWithoutUndo();
                    }
                }
                foreach (var g in all.Select(t => t.GetComponent<LocationStuff>()).Where(c => c != null))
                    Debug.Log($"[Upgrade] GPS on {GetPath(g.transform)} tracking {(g.object_to_track ? GetPath(g.object_to_track) : "self")}");
                foreach (var l in all.Select(t => t.GetComponent<HorizontalLidarScanner>()).Where(c => c != null))
                {
                    Debug.Log($"[Upgrade] Lidar on {GetPath(l.transform)} interval {l.scanInterval}s rays {l.numberOfRays}");
                    if (!dry) { l.scanInterval = 0.1f; EditorUtility.SetDirty(l); }
                }
                foreach (var c in all.Select(t => t.GetComponent<RGBCameraCapture>()).Where(c => c != null))
                {
                    Debug.Log($"[Upgrade] Raw cam on {GetPath(c.transform)} {c.width}x{c.height}@{c.fps} camera={(c.rgbCamera ? GetPath(c.rgbCamera.transform) : "none")}");
                    if (!dry) { c.width = 640; c.height = 480; c.fps = 30; if (c.frameid == "cam_unused") c.frameid = "camera_link"; EditorUtility.SetDirty(c); }
                }
                foreach (var c in all.Select(t => t.GetComponent<CompressedCameraCapture>()).Where(c => c != null))
                {
                    Debug.Log($"[Upgrade] JPEG cam on {GetPath(c.transform)} {c.width}x{c.height}@{c.fps}");
                    if (!dry) { c.width = 1280; c.height = 720; c.fps = 30; EditorUtility.SetDirty(c); }
                }
                // The moving rover body is the root ArticulationBody (chassis_link); base_link is a static parent.
                var root = all.Select(t => t.GetComponent<ArticulationBody>())
                              .FirstOrDefault(a => a != null && (a.transform.parent == null || a.transform.parent.GetComponentInParent<ArticulationBody>() == null));
                if (root != null)
                {
                    Debug.Log($"[Upgrade] rover body = {GetPath(root.transform)}");
                    if (root.GetComponent<ImuSensor>() == null)
                    {
                        Debug.Log($"[Upgrade] adding ImuSensor to {GetPath(root.transform)}");
                        if (!dry) root.gameObject.AddComponent<ImuSensor>();
                    }
                    var gps = all.Select(t => t.GetComponent<LocationStuff>()).FirstOrDefault(c => c != null);
                    if (gps != null && gps.object_to_track == null)
                    {
                        Debug.Log($"[Upgrade] GPS now tracks {GetPath(root.transform)}");
                        if (!dry) { gps.object_to_track = root.transform; EditorUtility.SetDirty(gps); }
                    }
                }
                if (!dry) UnityEditor.SceneManagement.EditorSceneManager.SaveScene(scene);
            }
            if (Application.isBatchMode) EditorApplication.Exit(0);
        }

        /// <summary>Creates Assets/Resources/ArucoUnlit.mat (unlit texture material used by the ArUco scripts).</summary>
        public static void EnsureArucoMaterial()
        {
            const string path = "Assets/Resources/ArucoUnlit.mat";
            if (AssetDatabase.LoadAssetAtPath<Material>(path) == null)
            {
                AssetDatabase.CreateAsset(new Material(Shader.Find("Unlit/Texture")), path);
                AssetDatabase.SaveAssets();
                Debug.Log("[SimFlake] created " + path);
            }
            if (Application.isBatchMode) EditorApplication.Exit(0);
        }

        /// <summary>
        /// Smoke test for the URDF -> prefab workflow. Imports -urdf (a .urdf under Assets) with VHACD colliders
        /// and saves it as -prefab. Run it on a scratch copy of a description package (it writes mesh assets).
        /// </summary>
        public static void TestUrdfImport()
        {
            string urdf = Arg("-urdf"), prefab = Arg("-prefab");
            var settings = new Unity.Robotics.UrdfImporter.ImportSettings
            {
                chosenAxis = Unity.Robotics.UrdfImporter.ImportSettings.axisType.yAxis,
                convexMethod = Unity.Robotics.UrdfImporter.ImportSettings.convexDecomposer.vHACD,
            };
            GameObject robot = null;
            var it = Unity.Robotics.UrdfImporter.UrdfRobotExtensions.Create(urdf, settings);
            while (it.MoveNext()) robot = it.Current ?? robot;
            bool ok = robot != null;
            if (ok)
            {
                int bodies = robot.GetComponentsInChildren<ArticulationBody>().Length;
                int cols = robot.GetComponentsInChildren<Collider>().Length;
                int meshes = robot.GetComponentsInChildren<MeshFilter>().Length;
                Debug.Log($"[UrdfTest] imported '{robot.name}': {bodies} articulation bodies, {meshes} meshes, {cols} colliders");
                PrefabUtility.SaveAsPrefabAsset(robot, prefab, out ok);
                Debug.Log($"[UrdfTest] prefab saved: {ok} -> {prefab}");
                ok &= bodies > 1 && meshes > 0 && cols > 0;
            }
            Debug.Log(ok ? "[UrdfTest] PASSED" : "[UrdfTest] FAILED");
            if (Application.isBatchMode) EditorApplication.Exit(ok ? 0 : 1);
        }

        static string GetPath(Transform t) => t.parent == null ? t.name : GetPath(t.parent) + "/" + t.name;

        /// <summary>Checks the JSON layer used to talk to rosbridge. Run from the menu or with -executeMethod.</summary>
        [MenuItem("SimFlake/Run ROS Message Self-Test")]
        public static void SelfTest()
        {
            int fails = 0;
            void Check(bool cond, string what) { if (!cond) { fails++; Debug.LogError("[SelfTest] FAIL " + what); } else Debug.Log("[SelfTest] ok   " + what); }

            var scan = new sensor_msgs.msg.LaserScan { ranges = new[] { 1.5f, float.PositiveInfinity } };
            string s = JsonUtility.ToJson(scan);
            Debug.Log("[SelfTest] LaserScan json: " + s);
            Check(s.Contains("Infinity"), "float +inf serializes as Infinity (ujson accepts it)");

            var fix = new sensor_msgs.msg.NavSatFix { latitude = 49.123456789012345, longitude = -123.98765432109876 };
            string f = JsonUtility.ToJson(fix);
            var back = JsonUtility.FromJson<sensor_msgs.msg.NavSatFix>(f);
            Check(Math.Abs(back.latitude - fix.latitude) < 1e-12, "double precision kept: " + f.Substring(f.IndexOf("\"latitude\""), 40));
            Check(f.Contains("\"position_covariance\":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]") || f.Contains("\"position_covariance\":[0,0,0,0,0,0,0,0,0]"), "fixed-size array has 9 entries");
            Check(f.Contains("\"status\":{\"status\":0"), "sbyte field serializes as number");

            string incoming = "{\"op\": \"publish\", \"topic\": \"/cmd_vel\", \"msg\": {\"linear\": {\"x\": 1.5, \"y\": 0.0, \"z\": 0.0}, \"angular\": {\"x\": 0.0, \"y\": 0.0, \"z\": -0.25}}}";
            string body = RosJson.ExtractField(incoming, "msg");
            var tw = JsonUtility.FromJson<geometry_msgs.msg.Twist>(body);
            Check(tw.linear.x == 1.5 && tw.angular.z == -0.25, "incoming Twist parsed");
            Check(RosJson.ExtractField("{\"msg\": \"a}\\\"b\", \"op\": \"x\"}", "op") == "\"x\"", "string-aware field scan");

            Check(RosJson.Unquote(RosJson.ExtractField("{\"op\":\"publish\",\"topic\":\"\\/cmd_vel\"}", "topic")) == "/cmd_vel", "ujson-escaped topic \\/cmd_vel decoded");
            Check(RosJson.Unquote("\"a\\u0041\\n\\\"b\"") == "aA\n\"b", "unicode / quote escapes decoded");

            Check(JsonUtility.FromJson<std_msgs.msg.String>("{\"data\":\"a\\/b\"}").data == "a/b", "JsonUtility decodes \\/ inside message bodies");

            var arr = JsonUtility.FromJson<std_msgs.msg.Float64MultiArray>("{\"layout\": {\"dim\": [], \"data_offset\": 0}, \"data\": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]}");
            Check(arr.data.Length == 7 && arr.data[6] == 7.0, "Float64MultiArray parsed");

            // What rosbridge (ujson) sends for the HMI's jog / position commands: char as a number, bare NaN.
            string armJson = RosJson.ExtractField("{\"op\": \"publish\", \"topic\": \"\\/arm\\/command\", \"msg\": {\"cmd_type\": 80, \"positions\": [NaN, 0.25, NaN, NaN, NaN, NaN, NaN], \"velocities\": [NaN, -5.0, NaN, NaN, NaN, NaN, NaN], \"end_effector\": 0.0, \"cmd_value\": 0}}", "msg");
            var arm = JsonUtility.FromJson<rover_msgs.msg.ArmCommand>(armJson);
            Check(arm.cmd_type == (byte)'P' && arm.positions.Length == 7 && arm.positions[1] == 0.25 && arm.velocities[1] == -5.0,
                  "ArmCommand parsed (char cmd_type, arrays)");
            Check(double.IsNaN(arm.positions[0]) && double.IsNaN(arm.velocities[6]), "bare NaN in incoming JSON parsed as NaN");
            var inf = JsonUtility.FromJson<sensor_msgs.msg.LaserScan>("{\"ranges\": [Infinity, -Infinity, 1.0], \"header\": {\"frame_id\": \"NaN Infinity\"}}");
            Check(float.IsPositiveInfinity(inf.ranges[0]) && float.IsNegativeInfinity(inf.ranges[1]) && inf.ranges[2] == 1f && inf.header.frame_id == "NaN Infinity",
                  "Infinity parsed, strings untouched");

            var q = RosFrames.UnityRotationToEnu(Quaternion.identity); // facing Unity +z = north
            Check(Math.Abs(q.z - Math.Sin(Math.PI / 4)) < 1e-6 && Math.Abs(q.w - Math.Cos(Math.PI / 4)) < 1e-6, $"facing north -> yaw +90deg in ENU ({q.x:F3},{q.y:F3},{q.z:F3},{q.w:F3})");
            var q2 = RosFrames.UnityRotationToEnu(Quaternion.Euler(0, 90, 0)); // facing Unity +x = east
            Check(Math.Abs(q2.w) > 0.9999, $"facing east -> yaw 0 ({q2.x:F3},{q2.y:F3},{q2.z:F3},{q2.w:F3})");

            Debug.Log(fails == 0 ? "[SelfTest] ALL PASSED" : $"[SelfTest] {fails} FAILED");
            if (Application.isBatchMode) EditorApplication.Exit(fails == 0 ? 0 : 1);
        }
    }
}
