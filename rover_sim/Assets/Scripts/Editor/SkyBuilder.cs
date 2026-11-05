using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Late-afternoon look for the open scene: mountain skybox (Shaders/MountainSkybox), a low warm sun in the
    /// west-southwest, matching ambient light, light haze and a reflection cubemap rendered from the sky.
    /// The sky material is only created if missing, so inspector tweaks to it survive re-runs.
    /// </summary>
    public static class SkyBuilder
    {
        const string SkyDir = "Assets/Worlds/Skies";

        // Sun position: degrees above the horizon, and compass bearing (0 = north/+z, 90 = east/+x).
        public const float SunElevation = 7f, SunBearing = 250f;

        [MenuItem("SimFlake/Apply Sunset Mountain Sky")]
        public static void ApplyMenu()
        {
            Apply();
            EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        }

        /// <summary>Everything that differs between sky looks. Sunset is the ChassisPlayground default.</summary>
        public class Preset
        {
            public string name = "SunsetMountains";
            public float elevation = SunElevation, bearing = SunBearing;
            public Color sunColor = new Color(1f, 0.70f, 0.45f);
            public float sunIntensity = 1.15f;
            public Color ambientSky = new Color(0.42f, 0.42f, 0.52f), ambientEquator = new Color(0.55f, 0.43f, 0.36f),
                ambientGround = new Color(0.20f, 0.16f, 0.13f);
            public float fogDensity = 0.0032f;
            public Color fogColor = new Color(0.80f, 0.62f, 0.50f);
            /// <summary>Called once when the sky material is first created, to set shader properties.</summary>
            public System.Action<Material> initMaterial;
        }

        /// <summary>Midday-ish desert: high SW sun, blue sky, red-brown mesas instead of snowy peaks.</summary>
        public static readonly Preset Desert = new Preset
        {
            name = "DesertMesas",
            elevation = 38f, bearing = 215f,
            sunColor = new Color(1f, 0.95f, 0.88f), sunIntensity = 1.05f,
            ambientSky = new Color(0.42f, 0.49f, 0.62f), ambientEquator = new Color(0.50f, 0.46f, 0.42f),
            ambientGround = new Color(0.26f, 0.20f, 0.15f),
            fogDensity = 0.0011f, fogColor = new Color(0.78f, 0.74f, 0.70f),
            initMaterial = m =>
            {
                m.SetColor("_ZenithColor", new Color(0.16f, 0.36f, 0.68f));
                m.SetColor("_HorizonColor", new Color(0.80f, 0.79f, 0.75f));
                m.SetColor("_GroundColor", new Color(0.45f, 0.36f, 0.28f));
                m.SetColor("_SunColor", new Color(1f, 0.95f, 0.86f));
                m.SetFloat("_SunGlow", 0.6f);
                m.SetFloat("_MountainHeight", 5f);
                m.SetColor("_FarMountainColor", new Color(0.66f, 0.58f, 0.58f));
                m.SetColor("_NearMountainColor", new Color(0.52f, 0.38f, 0.31f));
                m.SetColor("_AlpenGlow", new Color(0.85f, 0.62f, 0.50f));
                m.SetFloat("_SnowLine", 1.2f);
                m.SetFloat("_CloudCover", 0.22f);
                m.SetColor("_CloudShadow", new Color(0.66f, 0.68f, 0.76f));
                m.SetColor("_CloudLit", new Color(0.98f, 0.97f, 0.95f));
                m.SetFloat("_Seed", 11.3f);
            },
        };

        public static void Apply() => Apply(new Preset());

        public static void Apply(Preset p)
        {
            string SkyMatPath = $"{SkyDir}/{p.name}.mat", ReflectionPath = $"{SkyDir}/{p.name}_Reflection.asset";
            Directory.CreateDirectory(SkyDir);
            var mat = AssetDatabase.LoadAssetAtPath<Material>(SkyMatPath);
            if (!mat)
            {
                var shader = AssetDatabase.LoadAssetAtPath<Shader>("Assets/Shaders/MountainSkybox.shader");
                mat = new Material(shader) { name = p.name };
                p.initMaterial?.Invoke(mat);
                AssetDatabase.CreateAsset(mat, SkyMatPath);
            }
            mat.shader = AssetDatabase.LoadAssetAtPath<Shader>("Assets/Shaders/MountainSkybox.shader");

            // Sun
            var light = Object.FindObjectsOfType<Light>() is var lights && lights.Length > 0
                ? System.Array.Find(lights, l => l.type == LightType.Directional) : null;
            if (!light) light = new GameObject("Directional Light").AddComponent<Light>();
            light.type = LightType.Directional;
            float el = p.elevation * Mathf.Deg2Rad, br = p.bearing * Mathf.Deg2Rad;
            var toSun = new Vector3(Mathf.Cos(el) * Mathf.Sin(br), Mathf.Sin(el), Mathf.Cos(el) * Mathf.Cos(br));
            light.transform.rotation = Quaternion.LookRotation(-toSun);
            light.color = p.sunColor;
            light.intensity = p.sunIntensity;
            light.shadows = LightShadows.Soft;
            light.shadowStrength = 0.85f;

            RenderSettings.skybox = mat;
            RenderSettings.sun = light;

            // Ambient: cool sky above, warm bounce at the horizon
            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = p.ambientSky;
            RenderSettings.ambientEquatorColor = p.ambientEquator;
            RenderSettings.ambientGroundColor = p.ambientGround;

            // Haze: hides the terrain's far edge against the mountains, barely visible within ~50 m
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.ExponentialSquared;
            RenderSettings.fogDensity = p.fogDensity;
            RenderSettings.fogColor = p.fogColor;

            // Reflections from the new sky instead of Unity's default grey one
            var cam = new GameObject("reflection_cam").AddComponent<Camera>();
            cam.cullingMask = 0;
            cam.clearFlags = CameraClearFlags.Skybox;
            var cube = new Cubemap(128, TextureFormat.RGBA32, true);
            bool ok = cam.RenderToCubemap(cube);
            Object.DestroyImmediate(cam.gameObject);
            if (ok)
            {
                var old = AssetDatabase.LoadAssetAtPath<Cubemap>(ReflectionPath);
                if (old) AssetDatabase.DeleteAsset(ReflectionPath);
                AssetDatabase.CreateAsset(cube, ReflectionPath);
                RenderSettings.defaultReflectionMode = DefaultReflectionMode.Custom;
                RenderSettings.customReflection = cube;
                RenderSettings.reflectionIntensity = 0.6f;
            }
            else Debug.LogWarning("[SkyBuilder] couldn't render reflection cubemap (no GPU?), keeping default reflections");

            Debug.Log($"[SkyBuilder] {p.name} sky applied, sun {p.elevation} deg up at bearing {p.bearing}");
        }
    }
}
