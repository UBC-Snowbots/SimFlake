using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Time of day and weather for a scene with the mountain skybox (Shaders/MountainSkybox). The two are independent.
/// Change them in the inspector (also during Play), from the SimFlake > Environment menu, or in a player build with
/// --time-of-day sunrise|midday|sunset|night and --weather clear|cloudy.
///
/// skyMaterial is the scene's base look (mountain colours, cloud scale, seed). This works on a copy of it, so the
/// asset never changes; it also drives the sun light, ambient light, fog and the reflection cubemap.
/// </summary>
[ExecuteAlways]
public class SkyEnvironment : MonoBehaviour
{
    public enum TimeOfDay { Sunrise, Midday, Sunset, Night }
    public enum Weather { Clear, Cloudy }

    public TimeOfDay timeOfDay = TimeOfDay.Sunset;
    public Weather weather = Weather.Clear;
    [Tooltip("The scene's mountain sky material. Left unchanged; a copy gets the time-of-day settings.")]
    public Material skyMaterial;
    [Tooltip("Directional light used as the sun (moon at night). Found automatically if empty.")]
    public Light sun;
    [Tooltip("Haze density on a clear day. Cloudy days are hazier.")]
    public float baseFogDensity = 0.002f;

    struct Look
    {
        public float elevation, bearing;             // sun/moon: degrees above horizon, compass bearing (0 = +z north)
        public Color lightColor; public float lightIntensity;
        public Color ambientSky, ambientEquator, ambientGround, fog;
        public Color zenith, horizon, below, sunColor, disc, cloudLit, cloudShadow, landTint;
        public float sunGlow, sunSize, stars, alpenglow;
    }

    static Look Get(TimeOfDay t)
    {
        switch (t)
        {
            case TimeOfDay.Sunrise: return new Look
            {
                elevation = 8, bearing = 80,
                lightColor = new Color(1f, 0.78f, 0.58f), lightIntensity = 1.0f,
                ambientSky = new Color(0.45f, 0.47f, 0.58f), ambientEquator = new Color(0.60f, 0.50f, 0.43f), ambientGround = new Color(0.20f, 0.17f, 0.14f),
                fog = new Color(0.84f, 0.72f, 0.64f),
                zenith = new Color(0.24f, 0.38f, 0.64f), horizon = new Color(0.98f, 0.76f, 0.58f), below = new Color(0.40f, 0.33f, 0.29f),
                sunColor = new Color(1f, 0.78f, 0.52f), disc = new Color(2f, 1.85f, 1.5f), sunGlow = 1.0f, sunSize = 0.0004f, alpenglow = 1f,
                cloudLit = new Color(1f, 0.80f, 0.66f), cloudShadow = new Color(0.48f, 0.44f, 0.56f), landTint = new Color(1f, 0.93f, 0.88f),
            };
            case TimeOfDay.Midday: return new Look
            {
                elevation = 55, bearing = 190,
                lightColor = new Color(1f, 0.96f, 0.90f), lightIntensity = 1.05f,
                ambientSky = new Color(0.42f, 0.49f, 0.62f), ambientEquator = new Color(0.50f, 0.46f, 0.42f), ambientGround = new Color(0.26f, 0.20f, 0.15f),
                fog = new Color(0.78f, 0.77f, 0.75f),
                zenith = new Color(0.16f, 0.36f, 0.68f), horizon = new Color(0.80f, 0.80f, 0.78f), below = new Color(0.45f, 0.38f, 0.31f),
                sunColor = new Color(1f, 0.95f, 0.86f), disc = new Color(2f, 1.95f, 1.8f), sunGlow = 0.6f, sunSize = 0.0004f, alpenglow = 0.2f,
                cloudLit = new Color(0.98f, 0.97f, 0.95f), cloudShadow = new Color(0.66f, 0.68f, 0.76f), landTint = Color.white,
            };
            case TimeOfDay.Night: return new Look
            {
                elevation = 35, bearing = 150, // the light is the moon
                lightColor = new Color(0.62f, 0.70f, 0.95f), lightIntensity = 0.32f, // dim, but the course stays visible to cameras
                ambientSky = new Color(0.09f, 0.11f, 0.19f), ambientEquator = new Color(0.07f, 0.08f, 0.12f), ambientGround = new Color(0.03f, 0.03f, 0.04f),
                fog = new Color(0.04f, 0.05f, 0.09f),
                zenith = new Color(0.008f, 0.012f, 0.035f), horizon = new Color(0.05f, 0.07f, 0.13f), below = new Color(0.03f, 0.03f, 0.04f),
                sunColor = new Color(0.55f, 0.62f, 0.85f), disc = new Color(1.5f, 1.55f, 1.7f), sunGlow = 0.25f, sunSize = 0.0007f, stars = 1f,
                cloudLit = new Color(0.16f, 0.18f, 0.25f), cloudShadow = new Color(0.05f, 0.06f, 0.09f), landTint = new Color(0.10f, 0.12f, 0.18f),
            };
            default: return new Look // sunset
            {
                elevation = 7, bearing = 250,
                lightColor = new Color(1f, 0.70f, 0.45f), lightIntensity = 1.15f,
                ambientSky = new Color(0.42f, 0.42f, 0.52f), ambientEquator = new Color(0.55f, 0.43f, 0.36f), ambientGround = new Color(0.20f, 0.16f, 0.13f),
                fog = new Color(0.80f, 0.62f, 0.50f),
                zenith = new Color(0.20f, 0.30f, 0.55f), horizon = new Color(0.95f, 0.66f, 0.45f), below = new Color(0.36f, 0.30f, 0.27f),
                sunColor = new Color(1f, 0.72f, 0.42f), disc = new Color(2f, 1.84f, 1.5f), sunGlow = 1.1f, sunSize = 0.0004f, alpenglow = 1f,
                cloudLit = new Color(1f, 0.70f, 0.52f), cloudShadow = new Color(0.42f, 0.36f, 0.48f), landTint = new Color(1f, 0.90f, 0.85f),
            };
        }
    }

    static Color Grey(Color c, float amount, float scale = 1f)
    {
        float l = c.r * 0.3f + c.g * 0.55f + c.b * 0.15f;
        return Color.Lerp(c, new Color(l, l, l * 1.03f), amount) * scale;
    }

    Material instance;
    Cubemap reflection;

    void Awake()
    {
        if (!Application.isEditor) ReadCommandLine();
    }

    void OnEnable() => Apply();

    void OnDisable()
    {
        // hand the scene back its plain sky material, then free the copies
        if (skyMaterial && RenderSettings.skybox == instance) RenderSettings.skybox = skyMaterial;
        if (instance) DestroyImmediate(instance);
        if (reflection) DestroyImmediate(reflection);
    }

#if UNITY_EDITOR
    // OnValidate can't render, so apply right after it
    void OnValidate() => UnityEditor.EditorApplication.delayCall += () => { if (this && isActiveAndEnabled) Apply(); };
#endif

    void ReadCommandLine()
    {
        var a = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < a.Length - 1; i++)
        {
            if (a[i] == "--time-of-day" && System.Enum.TryParse(a[i + 1], true, out TimeOfDay t)) timeOfDay = t;
            if (a[i] == "--weather" && System.Enum.TryParse(a[i + 1], true, out Weather w)) weather = w;
        }
    }

    /// <summary>Sets time of day and weather and applies them.</summary>
    public void Set(TimeOfDay t, Weather w) { timeOfDay = t; weather = w; Apply(); }

    public void Apply()
    {
        var look = Get(timeOfDay);
        bool cloudy = weather == Weather.Cloudy;

        // --- sky material
        if (skyMaterial)
        {
            if (!instance || instance.shader != skyMaterial.shader)
            {
                if (instance) DestroyImmediate(instance);
                instance = new Material(skyMaterial) { name = skyMaterial.name + " (time of day)", hideFlags = HideFlags.DontSave };
            }
            else instance.CopyPropertiesFromMaterial(skyMaterial);
            instance.SetColor("_ZenithColor", cloudy ? Grey(look.zenith, 0.6f, 1.1f) : look.zenith);
            instance.SetColor("_HorizonColor", cloudy ? Grey(look.horizon, 0.6f, 0.95f) : look.horizon);
            instance.SetColor("_GroundColor", look.below);
            instance.SetColor("_SunColor", look.sunColor);
            instance.SetColor("_DiscColor", look.disc);
            instance.SetFloat("_SunGlow", look.sunGlow);
            instance.SetFloat("_SunSize", look.sunSize);
            instance.SetFloat("_StarIntensity", look.stars);
            instance.SetFloat("_GlowAmount", look.alpenglow * (cloudy ? 0.3f : 1f));
            instance.SetColor("_LandTint", cloudy ? Grey(look.landTint, 0.3f, 0.85f) : look.landTint);
            instance.SetColor("_CloudLit", cloudy ? Grey(look.cloudLit, 0.6f, 0.85f) : look.cloudLit);
            instance.SetColor("_CloudShadow", cloudy ? Grey(look.cloudShadow, 0.5f, 0.8f) : look.cloudShadow);
            instance.SetFloat("_CloudCover", cloudy ? 0.85f : 0.15f);
            instance.SetFloat("_Overcast", cloudy ? 0.75f : 0f);
            RenderSettings.skybox = instance;
        }

        // --- sun (or moon)
        if (!sun) sun = RenderSettings.sun;
        if (!sun) foreach (var l in FindObjectsOfType<Light>()) if (l.type == LightType.Directional) { sun = l; break; }
        if (sun)
        {
            float el = look.elevation * Mathf.Deg2Rad, br = look.bearing * Mathf.Deg2Rad;
            var toSun = new Vector3(Mathf.Cos(el) * Mathf.Sin(br), Mathf.Sin(el), Mathf.Cos(el) * Mathf.Cos(br));
            sun.transform.rotation = Quaternion.LookRotation(-toSun);
            sun.color = cloudy ? Grey(look.lightColor, 0.5f) : look.lightColor;
            sun.intensity = look.lightIntensity * (cloudy ? 0.45f : 1f);
            sun.shadows = LightShadows.Soft;
            sun.shadowStrength = cloudy ? 0.35f : 0.85f;
            RenderSettings.sun = sun;
        }

        // --- ambient light and haze. Overcast light is flatter: less sun, a little more (grey) sky.
        RenderSettings.ambientMode = AmbientMode.Trilight;
        RenderSettings.ambientSkyColor = cloudy ? Grey(look.ambientSky, 0.6f, 1.1f) : look.ambientSky;
        RenderSettings.ambientEquatorColor = cloudy ? Grey(look.ambientEquator, 0.6f, 1.05f) : look.ambientEquator;
        RenderSettings.ambientGroundColor = cloudy ? Grey(look.ambientGround, 0.5f) : look.ambientGround;
        RenderSettings.fog = true;
        RenderSettings.fogMode = FogMode.ExponentialSquared;
        RenderSettings.fogDensity = baseFogDensity * (cloudy ? 1.8f : 1f);
        RenderSettings.fogColor = cloudy ? Grey(look.fog, 0.6f, 0.9f) : look.fog;

        UpdateReflections();
    }

    /// <summary>Renders the sky into the reflection cubemap so shiny things reflect the current sky.</summary>
    void UpdateReflections()
    {
        if (!reflection) reflection = new Cubemap(64, TextureFormat.RGBA32, true) { name = "Sky reflection", hideFlags = HideFlags.DontSave };
        var go = new GameObject("sky reflection camera") { hideFlags = HideFlags.HideAndDontSave };
        var cam = go.AddComponent<Camera>();
        cam.enabled = false;
        cam.cullingMask = 0;
        cam.clearFlags = CameraClearFlags.Skybox;
        bool ok = cam.RenderToCubemap(reflection);
        DestroyImmediate(go);
        if (!ok) return; // no GPU (batchmode -nographics): keep whatever reflections the scene had
        RenderSettings.defaultReflectionMode = DefaultReflectionMode.Custom;
        RenderSettings.customReflection = reflection;
        RenderSettings.reflectionIntensity = timeOfDay == TimeOfDay.Night ? 0.3f : 0.6f;
    }
}
