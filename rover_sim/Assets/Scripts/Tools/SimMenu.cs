using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// In-game menu ("field PDA"), Esc in Play mode or a player build. Tabs: CONTROLS (every hotkey, greyed out when the
/// scene doesn't have that feature), CAMERA (free / follow camera, view graphics quality, rover sensor camera resolution),
/// SOIL (soil model quality, ruts), ENVIRONMENT (time of day, weather), WORLD (load another scene of the build). V toggles the follow camera without opening the menu.
///
/// Nothing to add to scenes: it creates itself after the first scene loads (not in -batchmode, so benches, PlayShots
/// and the soil demo are unaffected). Settings are runtime only; in the editor, graphics quality settings are put back
/// when Play stops (Unity would otherwise keep them in ProjectSettings). The simulation keeps running while it is open.
/// Look: original IMGUI drawing, textures generated in code (no image assets).
/// Check it headless: a player with graphics, <c>-batchmode -screen-width 1600 -screen-height 900 --sim-menu-shots &lt;dir&gt;</c>
/// turns the follow camera on, saves one PNG per tab and quits.
/// </summary>
[DefaultExecutionOrder(-100)]
public class SimMenu : MonoBehaviour
{
    public static SimMenu Instance { get; private set; }
    public static bool IsOpen => Instance != null && Instance.open;

    public KeyCode menuKey = KeyCode.Escape;
    public KeyCode followKey = KeyCode.V;
    public KeyCode nextRoverKey = KeyCode.T;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Create()
    {
        string shots = Arg("--sim-menu-shots");
        if ((Application.isBatchMode && shots == null) || Instance != null) return;
        var go = new GameObject("SimFlake Menu");
        DontDestroyOnLoad(go);
        var menu = go.AddComponent<SimMenu>();
        if (shots != null) menu.StartCoroutine(menu.Shots(shots));
    }

    static string Arg(string n) { var a = Environment.GetCommandLineArgs(); int i = Array.IndexOf(a, n); return i >= 0 && i + 1 < a.Length ? a[i + 1] : null; }

    System.Collections.IEnumerator Shots(string dir)
    {
        System.IO.Directory.CreateDirectory(dir);
        for (int i = 0; i < 120; i++) yield return null; // let the rover settle
        SetFollow(true);
        for (int i = 0; i < 60; i++) yield return null;
        yield return Grab(dir, "follow");
        SetOpen(true);
        for (int t = 0; t < Tabs.Length; t++)
        {
            tab = t;
            for (int i = 0; i < 5; i++) yield return null;
            yield return Grab(dir, "menu_" + Tabs[t].ToLowerInvariant());
        }
        if (SceneManager.sceneCountInBuildSettings > 1)
        {
            int next = (SceneManager.GetActiveScene().buildIndex + 1) % SceneManager.sceneCountInBuildSettings;
            LoadWorld(next);
            for (int i = 0; i < 300; i++) yield return null;
            yield return Grab(dir, "world_" + SceneName(next).ToLowerInvariant());
        }
        Debug.Log($"[SimMenu] shots saved to {dir}");
        Application.Quit(0);
    }

    // A batchmode player's backbuffer can't be read back, so for shots OnGUI draws the view camera and the menu into a texture.
    string shotPath;
    System.Collections.IEnumerator Grab(string dir, string name)
    {
        shotPath = System.IO.Path.Combine(dir, name + ".png");
        while (shotPath != null) yield return null;
    }

    void BeginShot()
    {
        var rt = RenderTexture.GetTemporary(Screen.width, Screen.height, 24);
        var cam = ViewCamera();
        if (cam != null) { cam.targetTexture = rt; cam.Render(); cam.targetTexture = null; }
        RenderTexture.active = rt;
    }

    void EndShot()
    {
        var rt = RenderTexture.active;
        var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        System.IO.File.WriteAllBytes(shotPath, tex.EncodeToPNG());
        Destroy(tex);
        RenderTexture.active = null;
        RenderTexture.ReleaseTemporary(rt);
        shotPath = null;
    }

    // ---------------------------------------------------------------- state
    bool open;
    int tab;
    static readonly string[] Tabs = { "CONTROLS", "CAMERA", "SOIL", "ENVIRONMENT", "WORLD" };
    Vector2 scroll;
    string toast; float toastUntil;

    bool following;
    Camera followCam;
    readonly List<Transform> rovers = new List<Transform>();
    int roverIndex;

    // Scene features, refreshed when the menu opens
    DrivetrainHud hud;
    CameraSwapper swapper;
    CameraOrbit orbit;
    RGBCameraCapture[] rawCams = new RGBCameraCapture[0];
    CompressedCameraCapture[] jpegCams = new CompressedCameraCapture[0];

    float baseFixedDt;
    long soilTicks0, soilSteps0; float soilSampleT; float soilMsPerStep = -1;
    float fps;

    void Awake()
    {
        if (Instance != null && Instance != this) { Destroy(gameObject); return; }
        Instance = this;
        FixDownscaledFullscreen();
        SceneManager.sceneLoaded += OnSceneLoaded;
        CaptureBaseFixedDt();
    }

    // Fullscreen below the desktop resolution ("downscaled viewport") on Linux: the mouse positions Unity reports don't
    // match the scaled picture, so clicks miss the menu. Go to the desktop resolution instead.
    static void FixDownscaledFullscreen()
    {
        if (Application.isEditor || Screen.fullScreenMode == FullScreenMode.Windowed) return;
        int w = Display.main.systemWidth, h = Display.main.systemHeight;
        if (w <= 0 || h <= 0 || (Screen.width == w && Screen.height == h)) return;
        Debug.Log($"[SimMenu] fullscreen at {Screen.width}x{Screen.height} on a {w}x{h} desktop: switching to {w}x{h} (mouse input is offset when downscaled)");
        Screen.SetResolution(w, h, Screen.fullScreenMode);
    }

    void OnDestroy()
    {
        SceneManager.sceneLoaded -= OnSceneLoaded;
        if (Instance == this) Instance = null;
    }

    void OnSceneLoaded(Scene s, LoadSceneMode m)
    {
        following = false; followCam = null; rovers.Clear(); terrainOriginals.Clear(); terrainDetail = -1;
        CaptureBaseFixedDt();
        if (followAfterLoad) StartCoroutine(FollowSoon());
        followAfterLoad = false;
        if (open) RefreshScene();
    }

    bool followAfterLoad;
    System.Collections.IEnumerator FollowSoon()
    {
        yield return null; // let the rovers' Start run
        SetFollow(true);
    }

    // ---------------------------------------------------------------- tab: environment

    static readonly string[] TimeNames = { "SUNRISE", "MIDDAY", "SUNSET", "NIGHT" };
    static readonly string[] WeatherNames = { "CLEAR", "CLOUDY" };
    SkyEnvironment sky;

    void DrawEnvironment()
    {
        if (sky == null) sky = FindObjectOfType<SkyEnvironment>();
        Section("SKY");
        if (sky == null)
        {
            Hint("This scene has no Sky Environment object, so time of day and weather are fixed. " +
                 "Scenes with one: BadlandsTraversal, UrcAutonomySite, SoilYard, ChassisPlayground.");
            return;
        }
        Row("TIME OF DAY");
        int t = Seg((int)sky.timeOfDay, TimeNames);
        EndRow();
        Row("WEATHER");
        int w = Seg((int)sky.weather, WeatherNames);
        EndRow();
        if (t != (int)sky.timeOfDay || w != (int)sky.weather)
            sky.Set((SkyEnvironment.TimeOfDay)t, (SkyEnvironment.Weather)w);
        Hint("Moves the sun (or moon) and changes sky, ambient light, fog and reflections. Rover cameras see it too. " +
             "Players also take --time-of-day and --weather on the command line.");
    }

    // ---------------------------------------------------------------- worlds

    static string SceneName(int buildIndex) => System.IO.Path.GetFileNameWithoutExtension(SceneUtility.GetScenePathByBuildIndex(buildIndex));

    void LoadWorld(int buildIndex)
    {
        // Leave the physics step as the project has it (soil Ultra halves it), drop the old scene's ROS
        // subscriptions and camera sockets (rosbridge keeps running), then load.
        Time.fixedDeltaTime = baseFixedDt;
        SimFlake.Ros.RosConnection.ResetForSceneChange();
        followAfterLoad = following;
        SetOpen(false);
        Toast("LOADING " + SceneName(buildIndex).ToUpperInvariant());
        SceneManager.LoadScene(buildIndex);
    }

    void DrawWorld()
    {
        Section("WORLDS IN THIS BUILD");
        int n = SceneManager.sceneCountInBuildSettings;
        int current = SceneManager.GetActiveScene().buildIndex;
        if (n <= 1)
        {
            Hint(Application.isEditor
                ? "In the editor, open another scene from the Project window. In a player build, every scene in the build is listed here."
                : "This build has only one world.");
            return;
        }
        for (int i = 0; i < n; i++)
        {
            GUILayout.BeginHorizontal(GUILayout.MinHeight(34));
            bool here = i == current;
            if (GUILayout.Button(here ? "CURRENT" : "LOAD", here ? sSegOn : sSegOff, GUILayout.Width(110), GUILayout.Height(30)) && !here) LoadWorld(i);
            GUILayout.Space(12);
            GUILayout.Label($"{SceneName(i)}  <color=#7a9499><size=13>{WorldBlurb(SceneName(i))}</size></color>", sLabel, GUILayout.Height(30));
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
        }
        Hint("Loading a world resets it (ruts, rover position). The ROS connection stays up; topics come back once the new world starts.");
    }

    static string WorldBlurb(string scene)
    {
        switch (scene)
        {
            case "BadlandsTraversal": return "1.5 km URC/CIRC-style traversal: escarpment, wash, hoodoos, dunes. Mini rover, soil centre tile.";
            case "UrcAutonomySite": return "The real URC 2027 autonomy square mile from lidar + aerial photo. chassis_24, stones, soil.";
            case "SoilYard": return "Mini rover on test lanes of Utah soils: ruts, sinkage, ramps.";
            case "ChassisPlayground": return "chassis_2024 with drops, slopes and a sand rock pit.";
            case "WaypointTask": return "chassis_2024 with GPS, lidar and cameras: waypoint task.";
            case "ArmScene": return "The arm on a table.";
            case "SampleScene": return "Older all-in-one playground.";
            default: return "";
        }
    }

    void CaptureBaseFixedDt()
    {
        var soil = SoilWorld.Instance;
        baseFixedDt = soil != null && soil.Current.fixedDt > 0 ? 0.005f : Time.fixedDeltaTime;
    }

    void Update()
    {
        fps = Mathf.Lerp(fps, 1f / Mathf.Max(Time.unscaledDeltaTime, 1e-4f), 0.05f);

        if (Input.GetKeyDown(menuKey)) SetOpen(!open);
        if (open)
        {
            if (Input.GetKeyDown(KeyCode.Q)) tab = (tab + Tabs.Length - 1) % Tabs.Length;
            if (Input.GetKeyDown(KeyCode.E)) tab = (tab + 1) % Tabs.Length;
            for (int i = 0; i < Tabs.Length; i++) if (Input.GetKeyDown(KeyCode.Alpha1 + i)) tab = i;
        }
        else
        {
            if (Input.GetKeyDown(followKey)) SetFollow(!following);
            if (following && Input.GetKeyDown(nextRoverKey)) NextRover();
        }

        // Follow target vanished (rover respawned/destroyed): fall back to free camera.
        if (following && (followCam == null || !followCam.TryGetComponent(out FollowCamera f) || f.target == null)) SetFollow(false);

        if (Time.unscaledTime > soilSampleT)
        {
            long dt = SoilWorld.SoilTicks - soilTicks0, ds = SoilWorld.SoilSteps - soilSteps0;
            soilMsPerStep = ds > 0 ? (float)(dt * 1000.0 / System.Diagnostics.Stopwatch.Frequency / ds) : -1;
            soilTicks0 = SoilWorld.SoilTicks; soilSteps0 = SoilWorld.SoilSteps;
            soilSampleT = Time.unscaledTime + 0.5f;
        }
    }

    void SetOpen(bool o)
    {
        open = o;
        if (open)
        {
            RefreshScene();
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }
    }

    void RefreshScene()
    {
        hud = FindObjectOfType<DrivetrainHud>();
        swapper = FindObjectOfType<CameraSwapper>();
        orbit = FindObjectOfType<CameraOrbit>();
        rawCams = FindObjectsOfType<RGBCameraCapture>();
        jpegCams = FindObjectsOfType<CompressedCameraCapture>();
        FindRovers();
    }

    void Toast(string msg) { toast = msg; toastUntil = Time.unscaledTime + 3f; }

    // ---------------------------------------------------------------- cameras

    /// <summary>The camera drawing the Game view: the MainCamera, else the deepest enabled on-screen camera.</summary>
    static Camera ViewCamera()
    {
        var main = Camera.main;
        if (main != null) return main;
        Camera best = null;
        foreach (var c in Camera.allCameras)
            if (c.targetTexture == null && (best == null || c.depth > best.depth)) best = c;
        return best;
    }

    void FindRovers()
    {
        rovers.Clear();
        foreach (var mc in FindObjectsOfType<MotorController>())
            foreach (var b in mc.GetComponentsInChildren<ArticulationBody>())
                if (b.isRoot && !rovers.Contains(b.transform)) rovers.Add(b.transform);
        if (rovers.Count == 0)
            foreach (var b in FindObjectsOfType<ArticulationBody>())
                if (b.isRoot && !rovers.Contains(b.transform)) rovers.Add(b.transform);
        if (roverIndex >= rovers.Count) roverIndex = 0;
    }

    static string RoverName(Transform body) => body.root != null ? body.root.name : body.name;

    void SetFollow(bool on)
    {
        var cam = following && followCam != null ? followCam : ViewCamera();
        if (cam == null) { Toast("NO VIEW CAMERA IN THIS SCENE"); following = false; return; }

        if (on)
        {
            FindRovers();
            if (rovers.Count == 0) { Toast("NO ROVER TO FOLLOW"); return; }
            // Anything else steering this camera stands down while following.
            foreach (var b in cam.GetComponents<MonoBehaviour>())
                if (b is FreeFlyCamera || b is CameraOrbit || b is RGBCameraObject) b.enabled = false;
            if (!cam.TryGetComponent(out FollowCamera f)) f = cam.gameObject.AddComponent<FollowCamera>();
            f.SetTarget(rovers[roverIndex]);
            f.enabled = true;
            followCam = cam;
            following = true;
            Toast("FOLLOWING " + RoverName(rovers[roverIndex]).ToUpperInvariant());
        }
        else
        {
            if (cam.TryGetComponent(out FollowCamera f)) f.enabled = false;
            foreach (var b in cam.GetComponents<MonoBehaviour>())
                if (b is CameraOrbit || b is RGBCameraObject) b.enabled = false;
            if (!cam.TryGetComponent(out FreeFlyCamera ff)) ff = cam.gameObject.AddComponent<FreeFlyCamera>();
            ff.enabled = true;
            following = false;
            followCam = null;
            Toast("FREE CAMERA");
        }
    }

    void NextRover()
    {
        FindRovers();
        if (rovers.Count == 0) return;
        roverIndex = (roverIndex + 1) % rovers.Count;
        if (following) SetFollow(true);
    }

    // ---------------------------------------------------------------- graphics quality (restored when Play stops)

    struct QualitySnapshot { public int aa, vsync; public ShadowQuality shadows; public float shadowDistance; }
    readonly Dictionary<int, QualitySnapshot> touchedLevels = new Dictionary<int, QualitySnapshot>();
    int originalLevel = -1, originalFrameRate;

    void BeforeQualityChange(bool levelChange = false)
    {
        if (originalLevel < 0) { originalLevel = QualitySettings.GetQualityLevel(); originalFrameRate = Application.targetFrameRate; }
        if (levelChange) return;
        int l = QualitySettings.GetQualityLevel();
        if (!touchedLevels.ContainsKey(l))
            touchedLevels[l] = new QualitySnapshot
            {
                aa = QualitySettings.antiAliasing, vsync = QualitySettings.vSyncCount,
                shadows = QualitySettings.shadows, shadowDistance = QualitySettings.shadowDistance,
            };
    }

    void OnApplicationQuit()
    {
#if UNITY_EDITOR
        if (originalLevel < 0) return;
        foreach (var kv in touchedLevels)
        {
            QualitySettings.SetQualityLevel(kv.Key, false);
            QualitySettings.antiAliasing = kv.Value.aa;
            QualitySettings.vSyncCount = kv.Value.vsync;
            QualitySettings.shadows = kv.Value.shadows;
            QualitySettings.shadowDistance = kv.Value.shadowDistance;
        }
        QualitySettings.SetQualityLevel(originalLevel, false);
        Application.targetFrameRate = originalFrameRate;
#endif
    }

    // Terrain detail presets; -1 = as the scene has it.
    static readonly string[] TerrainDetailNames = { "SCENE", "LOW", "MEDIUM", "HIGH", "ULTRA" };
    struct TerrainLook { public float pixelError, basemap, detail, trees; }
    static readonly TerrainLook[] TerrainLooks =
    {
        new TerrainLook { pixelError = 16, basemap = 200,  detail = 40,  trees = 300 },
        new TerrainLook { pixelError = 8,  basemap = 500,  detail = 80,  trees = 800 },
        new TerrainLook { pixelError = 4,  basemap = 1000, detail = 150, trees = 2000 },
        new TerrainLook { pixelError = 1,  basemap = 2000, detail = 250, trees = 5000 },
    };
    readonly Dictionary<Terrain, TerrainLook> terrainOriginals = new Dictionary<Terrain, TerrainLook>();
    int terrainDetail = -1;

    void SetTerrainDetail(int preset)
    {
        terrainDetail = preset;
        foreach (var t in Terrain.activeTerrains)
        {
            if (t == null) continue;
            if (!terrainOriginals.ContainsKey(t))
                terrainOriginals[t] = new TerrainLook { pixelError = t.heightmapPixelError, basemap = t.basemapDistance, detail = t.detailObjectDistance, trees = t.treeDistance };
            var l = preset < 0 ? terrainOriginals[t] : TerrainLooks[preset];
            t.heightmapPixelError = l.pixelError;
            t.basemapDistance = l.basemap;
            t.detailObjectDistance = l.detail;
            t.treeDistance = l.trees;
        }
    }

    // ---------------------------------------------------------------- theme (all generated, no image assets)

    static readonly Color Cyan = new Color(0.36f, 0.86f, 0.95f);
    static readonly Color CyanDim = new Color(0.20f, 0.48f, 0.56f);
    static readonly Color Orange = new Color(1.00f, 0.68f, 0.18f);
    static readonly Color Ink = new Color(0.02f, 0.07f, 0.10f, 0.95f);
    static readonly Color Text = new Color(0.84f, 0.96f, 1.00f);
    static readonly Color TextDim = new Color(0.45f, 0.58f, 0.62f);
    static readonly Color Red = new Color(1.00f, 0.38f, 0.30f);

    bool themeReady;
    Texture2D texWhite, texGrid, texStripes;
    GUIStyle sPanel, sTabOn, sTabOff, sSegOn, sSegOff, sLabel, sDim, sHead, sTitle, sSub, sKey, sSection, sSlider, sThumb, sWarn, sToast, sStatus;

    static Font OsFont(int size, params string[] names)
    {
        try { return Font.CreateDynamicFontFromOSFont(names, size); } catch { return null; }
    }

    /// <summary>A 9-sliceable box with cut (chamfered) corners and an edge line.</summary>
    static Texture2D Chamfer(Color fill, Color edge, int cut = 10, bool tl = true, bool tr = false, bool bl = false, bool br = true, int edgeWidth = 2)
    {
        const int S = 48;
        var t = new Texture2D(S, S, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Clamp, filterMode = FilterMode.Bilinear, hideFlags = HideFlags.DontSave };
        var px = new Color[S * S];
        for (int y = 0; y < S; y++)
            for (int x = 0; x < S; x++)
            {
                int top = S - 1 - y; // distance from the top row
                float corner = float.MaxValue;
                if (tl) corner = Mathf.Min(corner, x + top - cut);
                if (tr) corner = Mathf.Min(corner, (S - 1 - x) + top - cut);
                if (bl) corner = Mathf.Min(corner, x + y - cut);
                if (br) corner = Mathf.Min(corner, (S - 1 - x) + y - cut);
                Color c;
                if (corner < 0) c = Color.clear;
                else
                {
                    float d = Mathf.Min(Mathf.Min(x, S - 1 - x), Mathf.Min(y, S - 1 - y));
                    d = Mathf.Min(d, corner * 0.7071f);
                    c = d < edgeWidth ? edge : fill;
                }
                px[y * S + x] = c;
            }
        t.SetPixels(px);
        t.Apply();
        return t;
    }

    static Texture2D Solid(Color c)
    {
        var t = new Texture2D(1, 1, TextureFormat.RGBA32, false) { hideFlags = HideFlags.DontSave };
        t.SetPixel(0, 0, c); t.Apply();
        return t;
    }

    void BuildTheme()
    {
        themeReady = true;
        texWhite = Solid(Color.white);

        // Grid + scanlines, tiled over the panel.
        texGrid = new Texture2D(32, 32, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Repeat, filterMode = FilterMode.Point, hideFlags = HideFlags.DontSave };
        for (int y = 0; y < 32; y++)
            for (int x = 0; x < 32; x++)
            {
                float a = (y % 3 == 0) ? 0.05f : 0f;
                if (x == 0 || y == 0) a = 0.07f;
                texGrid.SetPixel(x, y, new Color(Cyan.r, Cyan.g, Cyan.b, a));
            }
        texGrid.Apply();

        // Diagonal hazard stripes.
        texStripes = new Texture2D(16, 16, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Repeat, filterMode = FilterMode.Bilinear, hideFlags = HideFlags.DontSave };
        for (int y = 0; y < 16; y++)
            for (int x = 0; x < 16; x++)
                texStripes.SetPixel(x, y, ((x + y) % 16) < 7 ? Orange : Color.clear);
        texStripes.Apply();

        Font body = OsFont(15, "Ubuntu Condensed", "Roboto Condensed", "DejaVu Sans Condensed", "Liberation Sans Narrow", "Ubuntu", "DejaVu Sans", "Arial");
        Font heavy = OsFont(30, "Ubuntu Condensed Bold", "Ubuntu Condensed", "DejaVu Sans Condensed Bold", "DejaVu Sans Condensed", "Ubuntu", "Arial");
        Font mono = OsFont(14, "Ubuntu Mono", "DejaVu Sans Mono", "Liberation Mono", "Monospace");

        GUIStyle Box(Texture2D bg, Texture2D hover = null, int border = 12)
        {
            var s = new GUIStyle { border = new RectOffset(border, border, border, border), alignment = TextAnchor.MiddleCenter, font = body, fontSize = 15 };
            s.normal.background = bg; s.normal.textColor = Text;
            s.hover.background = hover ?? bg; s.hover.textColor = Color.white;
            s.active.background = hover ?? bg; s.active.textColor = Color.white;
            return s;
        }

        sPanel = Box(Chamfer(Ink, Cyan, 18, edgeWidth: 2), border: 20);
        sTabOff = Box(Chamfer(new Color(0.05f, 0.14f, 0.18f, 0.95f), CyanDim, 10, true, true, false, false), Chamfer(new Color(0.08f, 0.22f, 0.27f, 0.95f), Cyan, 10, true, true, false, false));
        sTabOff.fontStyle = FontStyle.Bold; sTabOff.normal.textColor = Cyan;
        sTabOn = Box(Chamfer(Cyan, Cyan, 10, true, true, false, false));
        sTabOn.fontStyle = FontStyle.Bold; sTabOn.normal.textColor = sTabOn.hover.textColor = sTabOn.active.textColor = Ink;
        sSegOff = Box(Chamfer(new Color(0.04f, 0.12f, 0.16f, 0.95f), CyanDim, 6, true, false, false, true), Chamfer(new Color(0.08f, 0.22f, 0.27f, 0.95f), Cyan, 6, true, false, false, true), 8);
        sSegOff.fontSize = 14; sSegOff.padding = new RectOffset(10, 10, 4, 4); sSegOff.margin = new RectOffset(0, 6, 2, 2);
        sSegOn = Box(Chamfer(Orange, new Color(1f, 0.88f, 0.55f), 6, true, false, false, true), null, 8);
        sSegOn.fontSize = 14; sSegOn.fontStyle = FontStyle.Bold; sSegOn.padding = sSegOff.padding; sSegOn.margin = sSegOff.margin;
        sSegOn.normal.textColor = sSegOn.hover.textColor = sSegOn.active.textColor = Ink;

        sLabel = new GUIStyle { font = body, fontSize = 15, wordWrap = true, richText = true, alignment = TextAnchor.MiddleLeft, padding = new RectOffset(0, 0, 2, 2) };
        sLabel.normal.textColor = Text;
        sDim = new GUIStyle(sLabel) { fontSize = 13 };
        sDim.normal.textColor = TextDim;
        sWarn = new GUIStyle(sDim);
        sWarn.normal.textColor = Orange;
        sHead = new GUIStyle(sLabel) { fontStyle = FontStyle.Bold, wordWrap = false };
        sHead.normal.textColor = Cyan;
        sTitle = new GUIStyle(sLabel) { font = heavy, fontSize = 30, fontStyle = FontStyle.Bold, wordWrap = false };
        sTitle.normal.textColor = Orange;
        sSub = new GUIStyle(sLabel) { font = mono, fontSize = 13, wordWrap = false };
        sSub.normal.textColor = CyanDim;
        sSection = new GUIStyle(sLabel) { fontSize = 16, fontStyle = FontStyle.Bold, wordWrap = false, padding = new RectOffset(0, 0, 10, 2) };
        sSection.normal.textColor = Orange;
        sKey = Box(Chamfer(new Color(0.06f, 0.10f, 0.12f, 1f), Orange, 5, true, false, false, true, 1), null, 7);
        sKey.font = mono; sKey.fontSize = 13; sKey.fontStyle = FontStyle.Bold; sKey.padding = new RectOffset(6, 6, 3, 3);
        sKey.normal.textColor = sKey.hover.textColor = Orange;
        sToast = Box(Chamfer(new Color(0.02f, 0.07f, 0.10f, 0.9f), Orange, 8), null, 10);
        sToast.font = mono; sToast.fontSize = 15; sToast.fontStyle = FontStyle.Bold; sToast.padding = new RectOffset(18, 18, 8, 8);
        sToast.normal.textColor = Orange;

        // Slider: a thin line with ticks at the ends, orange thumb.
        var bar = new Texture2D(8, 18, TextureFormat.RGBA32, false) { hideFlags = HideFlags.DontSave, filterMode = FilterMode.Point };
        for (int y = 0; y < 18; y++)
            for (int x = 0; x < 8; x++)
                bar.SetPixel(x, y, (y >= 8 && y <= 9) || ((x == 0 || x == 7) && y >= 5 && y <= 12) ? CyanDim : Color.clear);
        bar.Apply();
        sSlider = new GUIStyle { border = new RectOffset(3, 3, 0, 0), fixedHeight = 18, margin = new RectOffset(4, 4, 6, 6) };
        sSlider.normal.background = bar;
        sThumb = new GUIStyle { fixedWidth = 12, fixedHeight = 18, border = new RectOffset(4, 4, 4, 4) };
        sThumb.normal.background = sThumb.hover.background = sThumb.active.background = Chamfer(Orange, new Color(1f, 0.9f, 0.6f), 4, true, false, false, true, 1);
        sStatus = new GUIStyle(sSub) { alignment = TextAnchor.UpperRight, richText = true };
    }

    // ---------------------------------------------------------------- drawing

    float uiScale;
    float viewW, viewH;

    void OnGUI()
    {
        bool shot = shotPath != null && Event.current.type == EventType.Repaint;
        if (shot) BeginShot();
        bool showToast = toast != null && Time.unscaledTime < toastUntil;
        if (!open && !showToast) { if (shot) EndShot(); return; }
        if (!themeReady) BuildTheme();

        GUI.depth = -100;
        uiScale = Mathf.Clamp(Screen.height / 900f, 0.6f, 3f);
        viewW = Screen.width / uiScale; viewH = Screen.height / uiScale;
        var oldMatrix = GUI.matrix;
        GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(uiScale, uiScale, 1));

        if (open) DrawMenu();
        if (showToast)
        {
            var c = new GUIContent(toast);
            var size = sToast.CalcSize(c);
            GUI.Label(new Rect((viewW - size.x) / 2, open ? 24 : viewH * 0.12f, size.x, size.y), c, sToast);
        }
        GUI.matrix = oldMatrix;
        if (shot) EndShot();
    }

    void Fill(Rect r, Color c)
    {
        var old = GUI.color; GUI.color = c;
        GUI.DrawTexture(r, texWhite);
        GUI.color = old;
    }

    void DrawMenu()
    {
        // Dim the world behind, with faint scanlines.
        Fill(new Rect(0, 0, viewW, viewH), new Color(0, 0.02f, 0.03f, 0.55f));

        float w = Mathf.Min(1060, viewW - 40), h = Mathf.Min(720, viewH - 40);
        var panel = new Rect((viewW - w) / 2, (viewH - h) / 2, w, h);
        GUI.Box(panel, GUIContent.none, sPanel);
        var inner = new Rect(panel.x + 4, panel.y + 4, panel.width - 8, panel.height - 8);
        GUI.DrawTextureWithTexCoords(inner, texGrid, new Rect(0, 0, inner.width / 32f, inner.height / 32f));

        // Header: title, hazard stripes, status readout.
        float x0 = panel.x + 28;
        GUI.Label(new Rect(x0, panel.y + 14, 300, 40), "SIMFLAKE", sTitle);
        GUI.Label(new Rect(x0 + 2, panel.y + 50, 600, 18), $"FIELD PDA  //  {SceneManager.GetActiveScene().name.ToUpperInvariant()}", sSub);
        var stripes = new Rect(x0 + 175, panel.y + 24, 120, 16);
        GUI.DrawTextureWithTexCoords(stripes, texStripes, new Rect(0, 0, stripes.width / 16f, 1));

        string status = $"{DateTime.Now:HH:mm:ss}   {fps:0} FPS   ROS {(SimFlake.Ros.Ros.Connected ? "<color=#5ddcf2>LINKED</color>" : "<color=#ff6050>OFFLINE</color>")}";
        GUI.Label(new Rect(panel.xMax - 430, panel.y + 22, 400, 18), status, sStatus);
        string cam = following && rovers.Count > 0 ? $"FOLLOW CAM: {RoverName(rovers[roverIndex]).ToUpperInvariant()}" : "FREE CAM";
        GUI.Label(new Rect(panel.xMax - 430, panel.y + 42, 400, 18), cam, sStatus);

        // Tabs.
        float tabY = panel.y + 78, tabW = 140;
        for (int i = 0; i < Tabs.Length; i++)
            if (GUI.Button(new Rect(x0 + i * (tabW + 6), tabY, tabW, 32), $"{Tabs[i]}", i == tab ? sTabOn : sTabOff)) { tab = i; scroll = Vector2.zero; }
        GUI.Label(new Rect(panel.xMax - 228, tabY + 8, 200, 20), "[Q] [E] SWITCH", sStatus);
        Fill(new Rect(panel.x + 12, tabY + 32, panel.width - 24, 2), Cyan);

        // Content.
        var content = new Rect(x0, tabY + 46, panel.width - 56, panel.yMax - (tabY + 46) - 46);
        GUILayout.BeginArea(content);
        scroll = GUILayout.BeginScrollView(scroll, false, false, GUIStyle.none, GUI.skin.verticalScrollbar);
        switch (tab)
        {
            case 0: DrawControls(content.width - 20); break;
            case 1: DrawCamera(); break;
            case 2: DrawSoil(); break;
            case 3: DrawEnvironment(); break;
            case 4: DrawWorld(); break;
        }
        GUILayout.EndScrollView();
        GUILayout.EndArea();

        // Footer.
        Fill(new Rect(panel.x + 12, panel.yMax - 40, panel.width - 24, 1), CyanDim);
        GUI.Label(new Rect(x0, panel.yMax - 34, 700, 22), "[ESC] CLOSE      SIMULATION KEEPS RUNNING      SETTINGS LAST UNTIL PLAY STOPS", sSub);
        var stripes2 = new Rect(panel.xMax - 150, panel.yMax - 30, 110, 12);
        GUI.DrawTextureWithTexCoords(stripes2, texStripes, new Rect(0, 0, stripes2.width / 16f, 1));
    }

    // ---------------------------------------------------------------- tab: controls

    struct Hotkey { public string keys, action; public bool available; }

    List<(string, List<Hotkey>)> Hotkeys()
    {
        bool swap = swapper != null && swapper.cam1 != null && swapper.cam2 != null;
        bool orbitOn = orbit != null && orbit.enabled && orbit.target != null;
        bool multi = rovers.Count > 1;
        var soil = SoilWorld.Instance;
        string Key(KeyCode k) => k.ToString().Replace("Alpha", "").ToUpperInvariant();

        return new List<(string, List<Hotkey>)>
        {
            ("VIEW", new List<Hotkey>
            {
                new Hotkey { keys = Key(followKey), action = "Follow camera on / off (off = free camera)", available = rovers.Count > 0 },
                new Hotkey { keys = "C", action = "Swap the rover's two cameras", available = swap },
                new Hotkey { keys = "F8", action = "Drivetrain HUD (wheel torque, slip, sinkage)", available = hud != null },
                new Hotkey { keys = soil != null ? Key(soil.cycleKey) : "F9", action = "Cycle soil quality (clears ruts)", available = soil != null },
            }),
            ("FREE CAMERA", new List<Hotkey>
            {
                new Hotkey { keys = "RMB + MOUSE", action = "Look around", available = true },
                new Hotkey { keys = "W A S D", action = "Move", available = true },
                new Hotkey { keys = "Q / E", action = "Down / up", available = true },
                new Hotkey { keys = "SHIFT", action = "Move fast (x5)", available = true },
                new Hotkey { keys = "SCROLL", action = "Change move speed", available = true },
            }),
            ("FOLLOW CAMERA", new List<Hotkey>
            {
                new Hotkey { keys = "RMB + MOUSE", action = "Orbit around the rover", available = rovers.Count > 0 },
                new Hotkey { keys = "SCROLL", action = "Zoom", available = rovers.Count > 0 },
                new Hotkey { keys = "R", action = "Snap back behind the rover", available = rovers.Count > 0 },
                new Hotkey { keys = Key(nextRoverKey), action = "Follow the next rover", available = multi },
            }),
            ("ORBIT CAMERA (SAMPLESCENE)", new List<Hotkey>
            {
                new Hotkey { keys = "ARROWS", action = "Orbit", available = orbitOn },
                new Hotkey { keys = "Z / X", action = "Zoom in / out", available = orbitOn },
            }),
        };
    }

    void DrawControls(float width)
    {
        var groups = Hotkeys();
        float colW = (width - 30) / 2;
        GUILayout.BeginHorizontal();
        for (int col = 0; col < 2; col++)
        {
            GUILayout.BeginVertical(GUILayout.Width(colW));
            for (int g = col; g < groups.Count; g += 2)
            {
                Section(groups[g].Item1);
                foreach (var k in groups[g].Item2)
                {
                    GUILayout.BeginHorizontal();
                    var old = GUI.color;
                    if (!k.available) GUI.color = new Color(1, 1, 1, 0.35f);
                    GUILayout.Label(k.keys, sKey, GUILayout.Width(118), GUILayout.Height(24));
                    GUILayout.Space(10);
                    GUILayout.Label(k.action + (k.available ? "" : "  <size=12>(not in this scene)</size>"), sLabel, GUILayout.Width(colW - 130));
                    GUI.color = old;
                    GUILayout.EndHorizontal();
                    GUILayout.Space(2);
                }
            }
            if (col == 1)
            {
                Section("DRIVING");
                GUILayout.Label("The rover is driven over ROS, not the keyboard: publish geometry_msgs/Twist on /cmd_vel " +
                                "(e.g. <b>ros2 run teleop_twist_keyboard teleop_twist_keyboard</b>). The arm listens on /arm/command.", sDim, GUILayout.Width(colW));
            }
            GUILayout.EndVertical();
            GUILayout.Space(30);
        }
        GUILayout.EndHorizontal();
    }

    // ---------------------------------------------------------------- tab: camera

    static readonly string[] AaNames = { "OFF", "2X", "4X", "8X" };
    static readonly int[] AaValues = { 0, 2, 4, 8 };
    static readonly string[] ShadowNames = { "OFF", "HARD", "SOFT" };
    static readonly string[] FrameNames = { "UNLIMITED", "VSYNC", "60 FPS", "30 FPS" };
    static readonly Vector2Int[] Resolutions = { new Vector2Int(320, 240), new Vector2Int(640, 480), new Vector2Int(1280, 720), new Vector2Int(1920, 1080) };
    static readonly string[] ResolutionNames = { "320x240", "640x480", "1280x720", "1920x1080" };
    static readonly int[] FpsValues = { 5, 10, 15, 30 };
    static readonly string[] FpsNames = { "5", "10", "15", "30" };

    void DrawCamera()
    {
        Section("VIEW MODE");
        Row("CAMERA");
        int mode = Seg(following ? 1 : 0, new[] { "FREE", "FOLLOW ROVER" });
        if (mode != (following ? 1 : 0)) SetFollow(mode == 1);
        EndRow();
        if (rovers.Count > 1)
        {
            Row("ROVER");
            var names = new string[rovers.Count];
            for (int i = 0; i < rovers.Count; i++) names[i] = RoverName(rovers[i]).ToUpperInvariant();
            int r = Seg(roverIndex, names);
            if (r != roverIndex) { roverIndex = r; if (following) SetFollow(true); }
            EndRow();
        }
        var cam = following && followCam != null ? followCam : ViewCamera();
        if (following && followCam != null && followCam.TryGetComponent(out FollowCamera f))
        {
            Row("DISTANCE");
            f.distance = Slider(f.distance, f.minDistance, f.maxDistance, "{0:0.0} m");
            EndRow();
            Row("AUTO RE-CENTRE");
            f.autoRecenter = Seg(f.autoRecenter ? 1 : 0, new[] { "OFF", "ON" }) == 1;
            EndRow();
        }
        if (cam != null)
        {
            Row("FIELD OF VIEW");
            cam.fieldOfView = Slider(cam.fieldOfView, 30, 110, "{0:0}°");
            EndRow();
        }

        Section("GRAPHICS (GAME VIEW)");
        Row("PRESET");
        var levels = QualitySettings.names;
        var levelNames = new string[levels.Length];
        for (int i = 0; i < levels.Length; i++) levelNames[i] = levels[i].ToUpperInvariant();
        int lvl = QualitySettings.GetQualityLevel();
        int nl = Seg(lvl, levelNames);
        if (nl != lvl) { BeforeQualityChange(true); QualitySettings.SetQualityLevel(nl, true); }
        EndRow();
        Hint("Unity quality level: texture size, shadows, LOD and anti-aliasing together. The rows below fine-tune it.");

        Row("ANTI-ALIASING");
        int aa = Array.IndexOf(AaValues, QualitySettings.antiAliasing);
        int na = Seg(aa, AaNames);
        if (na != aa) { BeforeQualityChange(); QualitySettings.antiAliasing = AaValues[na]; }
        EndRow();

        Row("SHADOWS");
        int sh = (int)QualitySettings.shadows;
        int ns = Seg(sh, ShadowNames);
        if (ns != sh) { BeforeQualityChange(); QualitySettings.shadows = (ShadowQuality)ns; }
        EndRow();
        if (QualitySettings.shadows != ShadowQuality.Disable)
        {
            Row("SHADOW DISTANCE");
            float sd = Slider(QualitySettings.shadowDistance, 10, 500, "{0:0} m");
            if (!Mathf.Approximately(sd, QualitySettings.shadowDistance)) { BeforeQualityChange(); QualitySettings.shadowDistance = sd; }
            EndRow();
        }

        if (Terrain.activeTerrains.Length > 0)
        {
            Row("TERRAIN DETAIL");
            int td = Seg(terrainDetail + 1, TerrainDetailNames) - 1;
            if (td != terrainDetail) SetTerrainDetail(td);
            EndRow();
            Hint("Ground mesh accuracy and how far grass, shrubs and the full-resolution ground texture are drawn. SCENE = as the world was built.");
        }

        Row("FRAME RATE");
        int fr = QualitySettings.vSyncCount > 0 ? 1 : Application.targetFrameRate == 60 ? 2 : Application.targetFrameRate == 30 ? 3 : 0;
        int nf = Seg(fr, FrameNames);
        if (nf != fr)
        {
            BeforeQualityChange();
            QualitySettings.vSyncCount = nf == 1 ? 1 : 0;
            Application.targetFrameRate = nf == 2 ? 60 : nf == 3 ? 30 : -1;
        }
        EndRow();

        Section("ROVER CAMERAS (WHAT ROS SEES)");
        if (rawCams.Length == 0 && jpegCams.Length == 0)
            Hint("No rover cameras in this scene.");
        if (rawCams.Length > 0)
        {
            var c0 = rawCams[0];
            GUILayout.Label($"RAW IMAGE   <color=#5ddcf2>{string.Join(", ", Topics(rawCams))}</color>", sLabel);
            Row("RESOLUTION");
            int ri = IndexOfRes(c0.width, c0.height);
            int nr = Seg(ri, ResolutionNames);
            if (nr != ri) foreach (var c in rawCams) { c.width = Resolutions[nr].x; c.height = Resolutions[nr].y; }
            EndRow();
            Row("RATE");
            int fi = Array.IndexOf(FpsValues, c0.fps);
            int nfi = Seg(fi, FpsNames);
            if (nfi != fi) foreach (var c in rawCams) c.fps = FpsValues[nfi];
            EndRow();
            long mbps = (long)c0.width * c0.height * 3 * c0.fps / 1_000_000;
            if (mbps > 30) GUILayout.Label($"~{mbps} MB/s per camera: more than rosbridge carries (~30 MB/s). Expect dropped frames; use the JPEG stream for high resolutions.", sWarn);
            else Hint($"~{mbps} MB/s per camera through rosbridge. camera_info follows the resolution.");
        }
        if (jpegCams.Length > 0)
        {
            var c0 = jpegCams[0];
            bool anyOn = false;
            foreach (var c in jpegCams) anyOn |= c.enabled;
            GUILayout.Label($"JPEG   <color=#5ddcf2>{string.Join(", ", Topics(jpegCams))}</color>{(anyOn ? "" : "  <color=#7a9499>(disabled in this scene)</color>")}", sLabel);
            Row("STREAM");
            bool on = Seg(anyOn ? 1 : 0, new[] { "OFF", "ON" }) == 1;
            if (on != anyOn) foreach (var c in jpegCams) c.enabled = on;
            EndRow();
            Row("RESOLUTION");
            int ri = IndexOfRes(c0.width, c0.height);
            int nr = Seg(ri, ResolutionNames);
            if (nr != ri) foreach (var c in jpegCams) { c.width = Resolutions[nr].x; c.height = Resolutions[nr].y; }
            EndRow();
            Row("RATE");
            int fi = Array.IndexOf(FpsValues, c0.fps);
            int nfi = Seg(fi, FpsNames);
            if (nfi != fi) foreach (var c in jpegCams) c.fps = FpsValues[nfi];
            EndRow();
            Row("JPEG QUALITY");
            int q = Mathf.RoundToInt(Slider(c0.jpegQuality, 10, 100, "{0:0}"));
            if (q != c0.jpegQuality) foreach (var c in jpegCams) c.jpegQuality = q;
            EndRow();
        }
    }

    static int IndexOfRes(int w, int h)
    {
        for (int i = 0; i < Resolutions.Length; i++) if (Resolutions[i].x == w && Resolutions[i].y == h) return i;
        return -1;
    }

    static IEnumerable<string> Topics(RGBCameraCapture[] cams) { foreach (var c in cams) yield return c.imageTopic; }
    static IEnumerable<string> Topics(CompressedCameraCapture[] cams) { foreach (var c in cams) yield return c.topic; }

    // ---------------------------------------------------------------- tab: soil

    static readonly string[] SoilNames = { "OFF", "LOW", "MEDIUM", "HIGH", "ULTRA" };

    void DrawSoil()
    {
        var soil = SoilWorld.Instance;
        Section("SOIL MECHANICS");
        if (soil == null)
        {
            Hint("This scene has no deformable soil (no SoilWorld): wheels use plain PhysX contacts. " +
                 "Soil scenes: SoilYard, BadlandsTraversal (centre tile), UrcAutonomySite.");
            return;
        }
        Row("QUALITY");
        int q = Seg((int)soil.quality, SoilNames);
        if (q != (int)soil.quality) soil.SetQuality((SoilQuality)q);
        EndRow();
        GUILayout.Label(SoilDescription(soil.quality), sLabel);
        Hint("Changing quality clears the ruts. Results agree across Low-Ultra within ~10 %; Low/Medium shake more on sand.");

        GUILayout.Space(6);
        Row("RUTS");
        bool ruts = Seg(soil.ruts ? 1 : 0, new[] { "OFF", "ON" }) == 1;
        if (ruts != soil.ruts) { soil.ruts = ruts; soil.ClearRuts(); }
        EndRow();
        Hint("Off: the soil still gives way under the wheels but springs back (no tracks left behind).");
        Row("");
        if (GUILayout.Button("CLEAR RUTS", sSegOff, GUILayout.Width(140), GUILayout.Height(28))) { soil.ClearRuts(); Toast("RUTS CLEARED"); }
        EndRow();

        Section("STATUS");
        string ms = soilMsPerStep >= 0 ? $"{soilMsPerStep:0.00} ms per physics step" : "idle (no wheel on soil)";
        GUILayout.Label($"Soil model: {(soil.Active ? soil.quality.ToString().ToUpperInvariant() : "OFF")}    {ms}    physics {1f / Time.fixedDeltaTime:0} Hz    " +
                        $"{soil.TerrainCount} soil terrain{(soil.TerrainCount == 1 ? "" : "s")}", sLabel);
        Hint("Per-wheel sinkage, contact patch and grip: F8 drivetrain HUD.");
    }

    string SoilDescription(SoilQuality q)
    {
        if (q == SoilQuality.Off) return "Plain PhysX contacts: no sinkage, no ruts, no tread. Fastest.";
        var p = SoilWorld.Get(q);
        float hz = 1f / (p.fixedDt > 0 ? Mathf.Min(p.fixedDt, baseFixedDt) : baseFixedDt);
        string draw = p.tessellation <= 0 ? "ruts drawn as shading only (light, good for iGPUs)" : $"ruts drawn with tessellation (~{p.tessellation:0} px triangles)";
        string speed = q == SoilQuality.Low ? "~10-12x" : q == SoilQuality.Medium ? "~9-11x" : q == SoilQuality.High ? "~9-10x" : "~3x";
        return $"{p.cell * 100:0.#} cm soil cells  ·  {p.nTheta}x{p.nY} tyre elements  ·  physics {hz:0} Hz  ·  {draw}  ·  " +
               $"<color=#5ddcf2>{speed} faster than real time</color> (mini rover, headless)";
    }

    // ---------------------------------------------------------------- widgets

    void Section(string title)
    {
        GUILayout.Label(title, sSection);
        var r = GUILayoutUtility.GetLastRect();
        Fill(new Rect(r.x, r.yMax, Mathf.Min(r.width, 420), 1), Orange);
        Fill(new Rect(r.x, r.yMax - 1, 30, 3), Orange);
        GUILayout.Space(6);
    }

    void Row(string label)
    {
        GUILayout.BeginHorizontal(GUILayout.MinHeight(30));
        GUILayout.Label(label, sHead, GUILayout.Width(190), GUILayout.Height(28));
    }

    static void EndRow()
    {
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();
    }

    void Hint(string text) => GUILayout.Label(text, sDim);

    int Seg(int selected, string[] options)
    {
        int result = selected;
        for (int i = 0; i < options.Length; i++)
            if (GUILayout.Button(options[i], i == selected ? sSegOn : sSegOff, GUILayout.Height(28), GUILayout.MinWidth(56))) result = i;
        return result;
    }

    float Slider(float value, float min, float max, string format)
    {
        float v = GUILayout.HorizontalSlider(value, min, max, sSlider, sThumb, GUILayout.Width(320));
        GUILayout.Space(12);
        GUILayout.Label(string.Format(format, v), sHead, GUILayout.Width(90), GUILayout.Height(28));
        return v;
    }
}
