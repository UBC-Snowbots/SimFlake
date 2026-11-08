using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Mountain skybox (Shaders/MountainSkybox) with time of day and weather for the open scene. The scene gets a
    /// "Sky Environment" object with a SkyEnvironment component, which sets the sky, sun, ambient light and haze.
    /// The sky material holds the scene's look (mountain colours etc.) and is only created if missing, so inspector
    /// tweaks to it survive re-runs.
    /// </summary>
    [InitializeOnLoad]
    public static class SkyBuilder
    {
        const string SkyDir = "Assets/Worlds/Skies";
        const string Menu = "SimFlake/Environment/";

        /// <summary>What differs between scenes: the sky material's look, the haze and the default time of day.</summary>
        public class Preset
        {
            public string name = "SunsetMountains";
            public float fogDensity = 0.0032f;
            public SkyEnvironment.TimeOfDay defaultTime = SkyEnvironment.TimeOfDay.Sunset;
            /// <summary>Called once when the sky material is first created, to set shader properties.</summary>
            public System.Action<Material> initMaterial;
        }

        /// <summary>Snowy mountain ranges on the horizon (ChassisPlayground).</summary>
        public static readonly Preset Mountains = new Preset();

        /// <summary>Desert: red-brown mesas instead of snowy peaks (BadlandsTraversal).</summary>
        public static readonly Preset Desert = new Preset
        {
            name = "DesertMesas",
            fogDensity = 0.0011f,
            defaultTime = SkyEnvironment.TimeOfDay.Midday,
            initMaterial = m =>
            {
                m.SetFloat("_MountainHeight", 5f);
                m.SetColor("_FarMountainColor", new Color(0.66f, 0.58f, 0.58f));
                m.SetColor("_NearMountainColor", new Color(0.52f, 0.38f, 0.31f));
                m.SetColor("_AlpenGlow", new Color(0.85f, 0.62f, 0.50f));
                m.SetFloat("_SnowLine", 1.2f);
                m.SetFloat("_Seed", 11.3f);
            },
        };

        // ------------------------------------------------------------------ set up

        public static void Apply() => Apply(Mountains);

        /// <summary>Adds (or updates) the scene's SkyEnvironment. An existing one keeps its time of day and weather.</summary>
        public static SkyEnvironment Apply(Preset p)
        {
            string matPath = $"{SkyDir}/{p.name}.mat";
            Directory.CreateDirectory(SkyDir);
            var shader = AssetDatabase.LoadAssetAtPath<Shader>("Assets/Shaders/MountainSkybox.shader");
            var mat = AssetDatabase.LoadAssetAtPath<Material>(matPath);
            if (!mat)
            {
                mat = new Material(shader) { name = p.name };
                p.initMaterial?.Invoke(mat);
                AssetDatabase.CreateAsset(mat, matPath);
            }
            mat.shader = shader;
            // reflections are rendered at runtime now; the old baked cubemaps aren't needed
            AssetDatabase.DeleteAsset($"{SkyDir}/{p.name}_Reflection.asset");

            var env = Object.FindObjectOfType<SkyEnvironment>();
            if (!env)
            {
                env = new GameObject("Sky Environment").AddComponent<SkyEnvironment>();
                env.timeOfDay = p.defaultTime;
                env.weather = SkyEnvironment.Weather.Clear;
            }
            env.skyMaterial = mat;
            env.baseFogDensity = p.fogDensity;
            if (!env.sun)
            {
                foreach (var l in Object.FindObjectsOfType<Light>()) if (l.type == LightType.Directional) { env.sun = l; break; }
                if (!env.sun) { env.sun = new GameObject("Directional Light").AddComponent<Light>(); env.sun.type = LightType.Directional; }
            }
            env.Apply();
            EditorUtility.SetDirty(env);
            Debug.Log($"[SkyBuilder] {p.name} sky: {env.timeOfDay}, {env.weather}");
            return env;
        }

        // Scenes are saved with the sky material asset (not the runtime copy) and default reflections; SkyEnvironment
        // puts its copy back when the scene opens. Without this the scene file would point at nothing.
        static SkyBuilder()
        {
            EditorSceneManager.sceneSaving += (scene, path) =>
            {
                var env = Object.FindObjectOfType<SkyEnvironment>();
                if (!env || !env.skyMaterial) return;
                RenderSettings.skybox = env.skyMaterial;
                RenderSettings.defaultReflectionMode = DefaultReflectionMode.Skybox;
                RenderSettings.customReflection = null;
            };
            EditorSceneManager.sceneSaved += scene =>
            {
                var env = Object.FindObjectOfType<SkyEnvironment>();
                if (env && env.isActiveAndEnabled) env.Apply();
            };
        }

        // ------------------------------------------------------------------ menu

        [MenuItem(Menu + "Set Up Mountain Sky", priority = 200)]
        static void SetUpMenu() { Apply(Object.FindObjectOfType<SkyEnvironment>() ? CurrentPreset() : Mountains); Dirty(); }

        static Preset CurrentPreset()
        {
            var env = Object.FindObjectOfType<SkyEnvironment>();
            return env && env.skyMaterial && env.skyMaterial.name == Desert.name ? Desert : Mountains;
        }

        static void Set(SkyEnvironment.TimeOfDay? t, SkyEnvironment.Weather? w)
        {
            var env = Object.FindObjectOfType<SkyEnvironment>() ?? Apply(Mountains);
            Undo.RecordObject(env, "Change time of day / weather");
            env.Set(t ?? env.timeOfDay, w ?? env.weather);
            Dirty();
        }

        static bool Check(string path, bool on) { UnityEditor.Menu.SetChecked(path, on); return true; }
        static SkyEnvironment Env => Object.FindObjectOfType<SkyEnvironment>();
        static void Dirty() => EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());

        const string T = Menu + "Time of Day/", W = Menu + "Weather/";
        [MenuItem(T + "Sunrise", priority = 210)] static void Sunrise() => Set(SkyEnvironment.TimeOfDay.Sunrise, null);
        [MenuItem(T + "Midday", priority = 211)] static void Midday() => Set(SkyEnvironment.TimeOfDay.Midday, null);
        [MenuItem(T + "Sunset", priority = 212)] static void Sunset() => Set(SkyEnvironment.TimeOfDay.Sunset, null);
        [MenuItem(T + "Night", priority = 213)] static void Night() => Set(SkyEnvironment.TimeOfDay.Night, null);
        [MenuItem(W + "Clear", priority = 220)] static void Clear() => Set(null, SkyEnvironment.Weather.Clear);
        [MenuItem(W + "Cloudy", priority = 221)] static void Cloudy() => Set(null, SkyEnvironment.Weather.Cloudy);
        [MenuItem(T + "Sunrise", true)] static bool SunriseOn() => Check(T + "Sunrise", Env && Env.timeOfDay == SkyEnvironment.TimeOfDay.Sunrise);
        [MenuItem(T + "Midday", true)] static bool MiddayOn() => Check(T + "Midday", Env && Env.timeOfDay == SkyEnvironment.TimeOfDay.Midday);
        [MenuItem(T + "Sunset", true)] static bool SunsetOn() => Check(T + "Sunset", Env && Env.timeOfDay == SkyEnvironment.TimeOfDay.Sunset);
        [MenuItem(T + "Night", true)] static bool NightOn() => Check(T + "Night", Env && Env.timeOfDay == SkyEnvironment.TimeOfDay.Night);
        [MenuItem(W + "Clear", true)] static bool ClearOn() => Check(W + "Clear", Env && Env.weather == SkyEnvironment.Weather.Clear);
        [MenuItem(W + "Cloudy", true)] static bool CloudyOn() => Check(W + "Cloudy", Env && Env.weather == SkyEnvironment.Weather.Cloudy);

        // ------------------------------------------------------------------ command line

        /// <summary>
        /// Adds SkyEnvironment to ChassisPlayground (sunset) and BadlandsTraversal (midday) and saves them.
        /// -screenshots dir also renders every time of day x weather from a fixed view in each scene (needs a GPU).
        /// </summary>
        public static void SetUpScenesFromCommandLine()
        {
            string shots = null;
            var a = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < a.Length - 1; i++) if (a[i] == "-screenshots") shots = a[i + 1];
            foreach (var (path, preset, pos, look) in new[]
            {
                ("Assets/Scenes/ChassisPlayground.unity", Mountains, Vector3.zero, Vector3.zero), // views relative to the terrain centre
                ("Assets/Scenes/TaskWorlds/BadlandsTraversal.unity", Desert, new Vector3(-30, 6, -232), new Vector3(-10, 4, -150)),
            })
            {
                var scene = EditorSceneManager.OpenScene(path);
                var env = Apply(preset);
                var (t0, w0) = (env.timeOfDay, env.weather);
                if (shots != null) RenderAll(env, Path.Combine(shots, Path.GetFileNameWithoutExtension(path)), pos, look);
                env.Set(t0, w0);
                EditorSceneManager.SaveScene(scene);
            }
            AssetDatabase.SaveAssets();
            EditorApplication.Exit(0);
        }

        static void RenderAll(SkyEnvironment env, string dir, Vector3 pos, Vector3 look)
        {
            Directory.CreateDirectory(dir);
            Physics.SyncTransforms();
            float Ground(Vector3 p) => Physics.Raycast(new Vector3(p.x, 1000, p.z), Vector3.down, out var h, 2000) ? h.point.y : 0;
            var cam = new GameObject("shotcam").AddComponent<Camera>();
            cam.farClipPlane = 3000; cam.fieldOfView = 70;
            var rt = new RenderTexture(1280, 720, 24) { antiAliasing = 4 };
            var tex = new Texture2D(1280, 720, TextureFormat.RGB24, false);
            if (pos == Vector3.zero && Terrain.activeTerrain) // a view across the middle of the terrain
            {
                var t = Terrain.activeTerrain; var c = t.transform.position + t.terrainData.size / 2;
                pos = new Vector3(c.x, 3, c.z - 30); look = new Vector3(c.x, 1, c.z + 30);
            }
            var from = pos + Vector3.up * Ground(pos);
            foreach (SkyEnvironment.TimeOfDay t in System.Enum.GetValues(typeof(SkyEnvironment.TimeOfDay)))
            foreach (SkyEnvironment.Weather w in System.Enum.GetValues(typeof(SkyEnvironment.Weather)))
            {
                env.Set(t, w);
                // one view of the course, one toward the sun/moon
                var toLight = -env.sun.transform.forward; toLight.y = Mathf.Max(toLight.y, 0.15f);
                foreach (var (view, target) in new[] { ("course", look + Vector3.up * Ground(look)), ("sun", from + toLight * 100) })
                {
                    cam.transform.position = from;
                    cam.transform.LookAt(target);
                    cam.targetTexture = rt;
                    cam.Render();
                    RenderTexture.active = rt;
                    tex.ReadPixels(new Rect(0, 0, 1280, 720), 0, 0); tex.Apply();
                    File.WriteAllBytes(Path.Combine(dir, $"{t}_{w}_{view}.png".ToLower()), tex.EncodeToPNG());
                }
            }
            RenderTexture.active = null;
            Object.DestroyImmediate(cam.gameObject);
            Debug.Log("[SkyBuilder] screenshots in " + dir);
        }
    }
}
