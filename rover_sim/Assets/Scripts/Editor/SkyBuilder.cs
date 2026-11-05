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
        const string SkyMatPath = SkyDir + "/SunsetMountains.mat";
        const string ReflectionPath = SkyDir + "/SunsetMountains_Reflection.asset";

        // Sun position: degrees above the horizon, and compass bearing (0 = north/+z, 90 = east/+x).
        public const float SunElevation = 7f, SunBearing = 250f;

        [MenuItem("SimFlake/Apply Sunset Mountain Sky")]
        public static void ApplyMenu()
        {
            Apply();
            EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        }

        public static void Apply()
        {
            Directory.CreateDirectory(SkyDir);
            var mat = AssetDatabase.LoadAssetAtPath<Material>(SkyMatPath);
            if (!mat)
            {
                var shader = AssetDatabase.LoadAssetAtPath<Shader>("Assets/Shaders/MountainSkybox.shader");
                mat = new Material(shader) { name = "SunsetMountains" };
                AssetDatabase.CreateAsset(mat, SkyMatPath);
            }
            mat.shader = AssetDatabase.LoadAssetAtPath<Shader>("Assets/Shaders/MountainSkybox.shader");

            // Sun
            var light = Object.FindObjectsOfType<Light>() is var lights && lights.Length > 0
                ? System.Array.Find(lights, l => l.type == LightType.Directional) : null;
            if (!light) light = new GameObject("Directional Light").AddComponent<Light>();
            light.type = LightType.Directional;
            float el = SunElevation * Mathf.Deg2Rad, br = SunBearing * Mathf.Deg2Rad;
            var toSun = new Vector3(Mathf.Cos(el) * Mathf.Sin(br), Mathf.Sin(el), Mathf.Cos(el) * Mathf.Cos(br));
            light.transform.rotation = Quaternion.LookRotation(-toSun);
            light.color = new Color(1f, 0.70f, 0.45f);
            light.intensity = 1.15f;
            light.shadows = LightShadows.Soft;
            light.shadowStrength = 0.85f;

            RenderSettings.skybox = mat;
            RenderSettings.sun = light;

            // Ambient: cool sky above, warm bounce at the horizon
            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = new Color(0.42f, 0.42f, 0.52f);
            RenderSettings.ambientEquatorColor = new Color(0.55f, 0.43f, 0.36f);
            RenderSettings.ambientGroundColor = new Color(0.20f, 0.16f, 0.13f);

            // Haze: hides the terrain's far edge against the mountains, barely visible within ~50 m
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.ExponentialSquared;
            RenderSettings.fogDensity = 0.0032f;
            RenderSettings.fogColor = new Color(0.80f, 0.62f, 0.50f);

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

            Debug.Log($"[SkyBuilder] sunset sky applied, sun {SunElevation} deg up at bearing {SunBearing}");
        }
    }
}
