using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Builds the URC autonomy world (Scenes/TaskWorlds/UrcAutonomySite) from real data of the real place: the Utah
    /// state-owned square mile north of MDRS that the URC 2027 rules name for the Autonomy Mission's route-finding
    /// sub-mission (rule 1.e.xiii, between 38.411N -110.786E and 38.425N -110.768E).
    ///
    ///   ground shape   USGS 3DEP 1 m lidar, used as is (2048 m square = the square mile + ~240 m margin, 2 x 2 terrains)
    ///   ground colour  USDA NAIP aerial photo (0.6 m) with sun shading, shadows and shrubs taken out
    ///   soils          one of six per 1 m cell, worked out from colour, slope, drainage and shrub cover (an interpretation)
    ///   shrubs, big boulders   where the aerial photo shows them
    ///   backdrop       16 km of 16 m DEM with Sentinel-2 colour (Factory Butte, Skyline Rim, the Fremont River valley)
    ///
    /// The source files in Assets/Worlds/UrcSite/Source come from tools/urc_site/fetch_data.py + process_data.py.
    /// What is NOT real: the task layout (URC publishes the targets at the competition), rocks smaller than the photo can
    /// show (scattered by soil class near the task area), and everything under 1 m (the soil model's relief).
    ///
    /// World frame: x = east, z = north, metres from the centre of the square mile (UTM 12N grid, so north is grid north,
    /// 0.14 deg off true north). y = elevation - 1375 m.
    ///   Menu: SimFlake > World Generation > Build URC Autonomy Site
    ///   Headless: Unity -batchmode -projectPath rover_sim -executeMethod SimFlake.EditorTools.UrcSiteBuilder.BuildFromCommandLine [-screenshots dir]
    /// A rebuild replaces "Generated World" and Assets/Worlds/UrcSite/Generated; rovers, camera and anything you added stay.
    /// </summary>
    public static class UrcSiteBuilder
    {
        public const string ScenePath = "Assets/Scenes/TaskWorlds/UrcAutonomySite.unity";
        const string Dir = "Assets/Worlds/UrcSite", Src = Dir + "/Source", Gen = Dir + "/Generated";
        const string RootName = "Generated World";
        const string ArucoPrefabPath = "Assets/Prefabs/Props/ArucoMarker_75cm.prefab";

        // ------------------------------------------------------------------ georeference (must match tools/urc_site)
        const double OriginLat = 38.418, OriginLon = -110.777;      // centre of the rules' square mile
        const double OriginE = 519467, OriginN = 4252218;           // the same point in UTM 12N (NAD83), whole metres
        const int Core = 2048, N = Core + 1;                        // metres / height samples (1 m)
        const float ElevBase = 1340f, ElevRange = 100f;             // heights_1m.bytes encoding
        const float Y0 = 1375f;                                     // elevation of Unity y = 0
        // Splat maps at 2 m: they are stored as text in the terrain assets (1 m would add ~50 MB to the repo), and the soil
        // map is an interpretation anyway. Heights stay at the lidar's 1 m.
        const int Tiles = 2, TileSize = Core / Tiles, TileRes = TileSize + 1, SplatRes = TileSize / 2;
        const int FarN = 1025; const float FarSize = 16384f, FarBase = 1250f, FarRange = 400f;
        /// <summary>The rules' square mile in world metres (for the boundary markers and the report).</summary>
        static readonly Rect SquareMile = Rect.MinMaxRect(-785f, -777f, 785f, 777f);

        // ------------------------------------------------------------------ task layout (world metres)
        // URC only publishes the real targets at the competition. These follow the rules' description (1.e.xii-xvi): a start
        // on flat easy ground, one target on a hill that can't be climbed from every side, one behind the hill without
        // radio line of sight to the C2 antenna. The C2 is where the aerial photo shows the vehicle tracks of the real
        // URC staging area. Each point is moved to the flattest ground within SnapRadius when the world is built.
        const float SnapRadius = 10f;
        static Vector2 C2 = new Vector2(-610, 592), Gate = new Vector2(-594, 590), DronePad = new Vector2(-586, 572);
        static Vector2 MissionStart = new Vector2(-560, 585);
        static Vector2 Astronaut = new Vector2(-535, 545), FollowTo = new Vector2(-500, 505), Hammer = new Vector2(-512, 552);
        const float MastHeight = 3f;

        struct Target { public string name; public Vector2 p; public int aruco; public bool snap; public string note; }
        static Target[] Targets =
        {
            new Target { name = "T1 hill", p = new Vector2(-259, 300), aruco = 1, snap = false,
                note = "Rule 1.e.xv. Top of a clay ridge, 16 m above the start. North-east and south-east faces are 25-35 deg; the way up is from the south-west." },
            new Target { name = "T2 hidden", p = new Vector2(-185, 240), aruco = 2, snap = true,
                note = "Rule 1.e.xvi. Behind the T1 ridge: no radio line of sight to the C2 mast." },
        };

        enum Soil { Hardpack, WashSand, BentoniteClay, Gravel, RockyTalus, Sandstone }

        /// <summary>
        /// What the ground is like under 1 m, per soil class: the part no public data set shows. Small-scale relief
        /// (peak amplitude in m; the RMS is about a quarter of it: 60 mm = 1.5 cm RMS) and stones (StoneField: rock abundance k of the Mars
        /// rock size law, the largest stone, the share lying loose). Judgment, anchored on: profile roughness measured on
        /// desert surfaces (Greeley et al., Death Valley: 0.5-1.5 cm RMS at particle scale, 4-6 cm on bouldery fans),
        /// rock abundances of 2-7 % on Mars plains and ~20 % on rocky ones, and what this ground looks like: crusted
        /// silt flats with stones set in them, clay hills with popcorn crust and rills, lag gravel on the crests,
        /// blocky talus under the sandstone ledges.
        /// </summary>
        static readonly (Soil soil, string spec, string from, float fine, float coarse, float k, float maxStone, float loose, string notes)[] GroundTable =
        {
            (Soil.Hardpack, "Mdrs_Hardpack", "Utah_Hardpack", 0.004f, 0.06f, 0.02f, 0.12f, 0.35f,
                "MDRS flats: crusted silty sand, hummocky between the shrubs, fist-size stones set in the crust and a few loose."),
            (Soil.WashSand, "Mdrs_WashSand", "Utah_LooseSand", 0.003f, 0.04f, 0.03f, 0.20f, 0.6f,
                "MDRS wash beds: loose sand with ripples and bars, cobbles lying loose."),
            (Soil.BentoniteClay, "Mdrs_BentoniteCrust", "Utah_BentoniteCrust", 0.008f, 0.10f, 0.01f, 0.12f, 0.5f,
                "MDRS clay hills (Brushy Basin): popcorn crust, rills down the slopes (as roughness, not as channels), few stones."),
            (Soil.Gravel, "Mdrs_GravelLag", "Utah_Gravel", 0.006f, 0.05f, 0.10f, 0.15f, 0.4f,
                "MDRS crests and gully beds: lag gravel, stones 5-15 cm close together."),
            (Soil.RockyTalus, "Mdrs_Talus", "Utah_Gravel", 0.008f, 0.16f, 0.20f, 0.5f, 0.3f,
                "MDRS talus under sandstone ledges: blocks in loose rubble, very rough. Meant to be hard or impossible to drive."),
            (Soil.Sandstone, "Mdrs_Sandstone", "Utah_Slickrock", 0.003f, 0.10f, 0.08f, 0.4f, 0.5f,
                "MDRS sandstone ledges: rigid, stepped along the bedding, broken blocks lying on it."),
        };

        // ------------------------------------------------------------------ state
        static ushort[] heights;        // [z * N + x], row 0 = south
        static byte[] soil;             // [z * Core + x], 1 m cells, row 0 = south
        const int ColorN = 512;
        static Color32[] groundColor;   // [z * ColorN + x], row 0 = south
        static Terrain[,] tiles;
        static Transform root;
        static PhysicMaterial dirt;
        static readonly StringBuilder report = new StringBuilder();
        static readonly List<Vector3> keepClear = new List<Vector3>();    // x, z, radius
        static readonly Dictionary<string, Material> mats = new Dictionary<string, Material>();

        [MenuItem("SimFlake/World Generation/Build URC Autonomy Site (real MDRS terrain)", priority = 102)]
        public static void BuildMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            Build();
        }

        public static void BuildFromCommandLine()
        {
            try
            {
                Build();
                string shots = Arg("-screenshots");
                if (!string.IsNullOrEmpty(shots)) Screenshots(shots);
                EditorApplication.Exit(0);
            }
            catch (System.Exception e) { Debug.LogException(e); EditorApplication.Exit(1); }
        }

        static void Build()
        {
            var t0 = System.DateTime.Now;
            if (!File.Exists(Src + "/heights_1m.bytes")) throw new System.Exception($"{Src} is missing: run tools/urc_site/fetch_data.py and process_data.py");
            SoilSetup.EnsureAssets();
            SoilSetup.EnsureMaterials();
            ImportSources();
            var scene = File.Exists(ScenePath)
                ? EditorSceneManager.OpenScene(ScenePath)
                : EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            foreach (var old in scene.GetRootGameObjects().Where(g => g.name == RootName)) Object.DestroyImmediate(old);
            if (AssetDatabase.IsValidFolder(Gen)) AssetDatabase.DeleteAsset(Gen);
            foreach (var d in new[] { Gen, Gen + "/Materials", Gen + "/Prefabs", Gen + "/Meshes" }) Directory.CreateDirectory(d);
            AssetDatabase.Refresh();
            mats.Clear(); report.Clear(); keepClear.Clear();
            Random.InitState(2027);
            dirt = AssetDatabase.LoadAssetAtPath<PhysicMaterial>("Assets/Materials/CustomDirt.physicMaterial");

            LoadData();
            SnapLayout();
            root = new GameObject(RootName).transform;
            BuildTerrain();
            PlantShrubs();
            BuildRocks();
            BuildBackdrop();
            BuildSite();
            BuildTargets();

            MakeSiteSoils();
            BuildStoneField();
            SoilYardBuilder.EnsureSoilObject();
            var env = SkyBuilder.Apply(Sky);
            if (env.sun) env.sun.shadows = LightShadows.Soft;
            var ls = new LightingSettings { name = "UrcSiteLighting", bakedGI = false, realtimeGI = false };
            AssetDatabase.CreateAsset(ls, Gen + "/UrcSiteLighting.lighting");
            Lightmapping.lightingSettings = ls;
            EnsureRovers();
            EnsureCamera();

            AssetDatabase.SaveAssets();
            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene, ScenePath);
            WriteWaypointFile();
            Verify();
            Debug.Log($"[UrcSite] Done in {(System.DateTime.Now - t0).TotalSeconds:F0} s\n{report}");
        }

        /// <summary>Clear desert air: you can see Factory Butte 10 km away. Low far ranges on the horizon (the Henry Mountains).</summary>
        static readonly SkyBuilder.Preset Sky = new SkyBuilder.Preset
        {
            name = "UtahDesert",
            fogDensity = 0.00011f,
            defaultTime = SkyEnvironment.TimeOfDay.Midday,
            initMaterial = m =>
            {
                m.SetFloat("_MountainHeight", 3.5f);
                m.SetColor("_FarMountainColor", new Color(0.62f, 0.60f, 0.66f));
                m.SetColor("_NearMountainColor", new Color(0.55f, 0.47f, 0.44f));
                m.SetColor("_AlpenGlow", new Color(0.85f, 0.62f, 0.50f));
                m.SetFloat("_SnowLine", 1.2f);
                m.SetFloat("_Seed", 5.2f);
            },
        };

        // ------------------------------------------------------------------ source data

        static void ImportSources()
        {
            AssetDatabase.Refresh();
            void Tex(string path, int max, TextureWrapMode wrap, bool normal = false, bool cutout = false)
            {
                var ti = (TextureImporter)AssetImporter.GetAtPath(path);
                if (ti == null) throw new System.Exception($"missing {path} (run tools/urc_site/process_data.py and make_assets.py)");
                var type = normal ? TextureImporterType.NormalMap : TextureImporterType.Default;
                if (ti.maxTextureSize == max && ti.wrapMode == wrap && ti.textureType == type && ti.anisoLevel == 8 && ti.mipMapsPreserveCoverage == cutout) return;
                ti.textureType = type; ti.maxTextureSize = max; ti.wrapMode = wrap; ti.anisoLevel = 8; ti.mipmapEnabled = true;
                ti.textureCompression = TextureImporterCompression.CompressedHQ;
                // cut-out cards: keep the plants as full at a distance as close up
                ti.alphaIsTransparency = cutout; ti.mipMapsPreserveCoverage = cutout; ti.alphaTestReferenceValue = 0.4f;
                ti.SaveAndReimport();
            }
            Tex(Src + "/ground_color.jpg", 4096, TextureWrapMode.Clamp);
            Tex(Src + "/far_color.jpg", 2048, TextureWrapMode.Clamp);
            foreach (var l in System.Enum.GetNames(typeof(Soil)))
            {
                Tex($"{Src}/Layers/{l}_Detail.jpg", 1024, TextureWrapMode.Repeat);
                Tex($"{Src}/Layers/{l}_Normal.jpg", 1024, TextureWrapMode.Repeat, normal: true);
            }
            Tex(Src + "/Props/Rock_Albedo.jpg", 1024, TextureWrapMode.Repeat);
            Tex(Src + "/Props/Rock_Normal.jpg", 1024, TextureWrapMode.Repeat, normal: true);
            Tex(Src + "/Props/ShrubAtlas.png", 1024, TextureWrapMode.Clamp, cutout: true);
        }

        static void LoadData()
        {
            var b = File.ReadAllBytes(Src + "/heights_1m.bytes");
            if (b.Length != N * N * 2) throw new System.Exception("heights_1m.bytes has the wrong size");
            heights = new ushort[N * N];
            System.Buffer.BlockCopy(b, 0, heights, 0, b.Length);

            var tex = new Texture2D(2, 2, TextureFormat.RGBA32, false, true);
            tex.LoadImage(File.ReadAllBytes(Src + "/soil_classes.png"));
            if (tex.width != Core || tex.height != Core) throw new System.Exception("soil_classes.png has the wrong size");
            var px = tex.GetPixels32();      // bottom row first = south first
            soil = new byte[Core * Core];
            for (int i = 0; i < soil.Length; i++) soil[i] = (byte)Mathf.Min(px[i].r, 5);
            // ground colour at 4 m, to pick each boulder's rock colour
            tex.LoadImage(File.ReadAllBytes(Src + "/ground_color.jpg"));
            px = tex.GetPixels32();
            int step = tex.width / ColorN;
            groundColor = new Color32[ColorN * ColorN];
            for (int z = 0; z < ColorN; z++) for (int x = 0; x < ColorN; x++) groundColor[z * ColorN + x] = px[(z * step + step / 2) * tex.width + x * step + step / 2];
            Object.DestroyImmediate(tex);
        }

        /// <summary>Ground height (Unity y) at world x, z from the DEM (same triangles as Unity's terrain).</summary>
        static float Ground(float x, float z)
        {
            float fx = Mathf.Clamp(x + Core / 2, 0, Core - 0.001f), fz = Mathf.Clamp(z + Core / 2, 0, Core - 0.001f);
            int xi = (int)fx, zi = (int)fz;
            float u = fx - xi, v = fz - zi;
            float h00 = heights[zi * N + xi], h10 = heights[zi * N + xi + 1], h01 = heights[(zi + 1) * N + xi], h11 = heights[(zi + 1) * N + xi + 1];
            // the terrain mesh splits each cell along the (0,0)-(1,1) diagonal
            float h = u > v ? h00 + (h10 - h00) * u + (h11 - h10) * v : h00 + (h11 - h01) * u + (h01 - h00) * v;
            return ElevBase + h / 65535f * ElevRange - Y0;
        }
        static float Ground(Vector2 p) => Ground(p.x, p.y);
        static Vector3 OnGround(Vector2 p, float up = 0) => new Vector3(p.x, Ground(p) + up, p.y);

        static float SlopeDeg(float x, float z)
        {
            float gx = (Ground(x + 1, z) - Ground(x - 1, z)) / 2, gz = (Ground(x, z + 1) - Ground(x, z - 1)) / 2;
            return Mathf.Atan(Mathf.Sqrt(gx * gx + gz * gz)) * Mathf.Rad2Deg;
        }

        /// <summary>Steepest slope within r metres.</summary>
        static float MaxSlope(Vector2 p, float r)
        {
            float m = 0;
            for (float dz = -r; dz <= r; dz += 1) for (float dx = -r; dx <= r; dx += 1) m = Mathf.Max(m, SlopeDeg(p.x + dx, p.y + dz));
            return m;
        }

        static Color GroundColorAt(float x, float z)
        {
            int xi = Mathf.Clamp((int)((x / Core + 0.5f) * ColorN), 0, ColorN - 1), zi = Mathf.Clamp((int)((z / Core + 0.5f) * ColorN), 0, ColorN - 1);
            return groundColor[zi * ColorN + xi];
        }

        static Soil SoilAt(float x, float z)
        {
            int xi = Mathf.Clamp(Mathf.FloorToInt(x + Core / 2), 0, Core - 1), zi = Mathf.Clamp(Mathf.FloorToInt(z + Core / 2), 0, Core - 1);
            return (Soil)soil[zi * Core + xi];
        }

        /// <summary>Move each task point to the flattest spot nearby (the layout was picked on a map, not on the ground).</summary>
        static void SnapLayout()
        {
            Vector2 Snap(Vector2 p, float flat)
            {
                Vector2 best = p; float bs = MaxSlope(p, flat);
                for (float dz = -SnapRadius; dz <= SnapRadius; dz += 2) for (float dx = -SnapRadius; dx <= SnapRadius; dx += 2)
                {
                    var q = p + new Vector2(dx, dz);
                    float s = MaxSlope(q, flat) + 0.15f * new Vector2(dx, dz).magnitude;
                    if (s < bs) { bs = s; best = q; }
                }
                return best;
            }
            C2 = Snap(C2, 4); Gate = Snap(Gate, 2); DronePad = Snap(DronePad, 2); MissionStart = Snap(MissionStart, 2);
            Astronaut = Snap(Astronaut, 2); FollowTo = Snap(FollowTo, 2); Hammer = Snap(Hammer, 1);
            for (int i = 0; i < Targets.Length; i++) if (Targets[i].snap) Targets[i].p = Snap(Targets[i].p, 1);
            foreach (var p in new[] { C2, Gate, DronePad, MissionStart, Astronaut, FollowTo, Hammer }) keepClear.Add(new Vector3(p.x, p.y, 4f));
            keepClear.Add(new Vector3(C2.x, C2.y, 9f));
            // an open lane from the rover start through the gate to the mission start
            for (float t = -0.15f; t <= 1f; t += 0.05f) { var q = Vector2.Lerp(Gate, MissionStart, t); keepClear.Add(new Vector3(q.x, q.y, 3f)); }
            foreach (var t in Targets) keepClear.Add(new Vector3(t.p.x, t.p.y, 2.5f));
        }

        static bool Clear(float x, float z, float margin = 0)
        {
            foreach (var k in keepClear) if (Sq(x - k.x) + Sq(z - k.y) < Sq(k.z + margin)) return false;
            return true;
        }
        static float Sq(float v) => v * v;

        // ------------------------------------------------------------------ terrain

        // Close-up textures drawn for this site by tools/urc_site/make_assets.py (Source/Layers). tile = metres per repeat.
        static readonly (Soil soil, string layerName, float tile, float normalScale)[] LayerSpec =
        {
            // layerName decides the soil (SoilSetup.ForLayer): keep the key words
            (Soil.Hardpack, "Hardpack", 2f, 0.8f),
            (Soil.WashSand, "WashSand", 3f, 0.9f),
            (Soil.BentoniteClay, "BentoniteClay", 2f, 1f),
            (Soil.Gravel, "Gravel", 2f, 1f),
            (Soil.RockyTalus, "TalusGravel", 3f, 1.1f),
            (Soil.Sandstone, "Sandstone", 6f, 1.2f),
        };

        static void BuildTerrain()
        {
            var layers = LayerSpec.Select(s =>
            {
                var tl = new TerrainLayer
                {
                    name = s.layerName,
                    diffuseTexture = AssetDatabase.LoadAssetAtPath<Texture2D>($"{Src}/Layers/{s.soil}_Detail.jpg"),
                    normalMapTexture = AssetDatabase.LoadAssetAtPath<Texture2D>($"{Src}/Layers/{s.soil}_Normal.jpg"),
                    tileSize = new Vector2(s.tile, s.tile), normalScale = s.normalScale, smoothness = 0.03f, metallic = 0,
                };
                if (!tl.diffuseTexture) throw new System.Exception($"missing {Src}/Layers/{s.soil}_Detail.jpg");
                AssetDatabase.CreateAsset(tl, $"{Gen}/Materials/{s.layerName}.terrainlayer");
                return tl;
            }).ToArray();
            var shader = Shader.Find("SimFlake/SiteTerrain");
            if (shader == null) throw new System.Exception("shader SimFlake/SiteTerrain not found");
            var macro = AssetDatabase.LoadAssetAtPath<Texture2D>(Src + "/ground_color.jpg");
            var bushes = MakeShrubPrefabs();
            var group = new GameObject("Terrain (USGS 1 m lidar)").transform;
            group.SetParent(root, false);
            tiles = new Terrain[Tiles, Tiles];
            int nl = layers.Length;

            for (int j = 0; j < Tiles; j++)
            for (int i = 0; i < Tiles; i++)
            {
                int x0 = i * TileSize, z0 = j * TileSize;      // in samples from the south-west corner
                var td = new TerrainData { heightmapResolution = TileRes };
                td.size = new Vector3(TileSize, ElevRange, TileSize);
                var hm = new float[TileRes, TileRes];
                for (int z = 0; z < TileRes; z++) for (int x = 0; x < TileRes; x++) hm[z, x] = heights[(z0 + z) * N + x0 + x] / 65535f;
                td.SetHeights(0, 0, hm);
                td.alphamapResolution = SplatRes;
                td.baseMapResolution = 512;
                td.terrainLayers = layers;
                td.treePrototypes = bushes.Select(b => new TreePrototype { prefab = b.prefab }).ToArray();
                AssetDatabase.CreateAsset(td, $"{Gen}/Terrain_{i}_{j}.asset");     // before SetAlphamaps (else the splat maps are lost)

                // soil classes -> layer weights: each texel covers 2 x 2 cells, softened by the ring of cells around them
                var a = new float[SplatRes, SplatRes, nl];
                for (int z = 0; z < SplatRes; z++)
                for (int x = 0; x < SplatRes; x++)
                {
                    for (int dz = -1; dz <= 2; dz++)
                    for (int dx = -1; dx <= 2; dx++)
                    {
                        int sx = Mathf.Clamp(x0 + 2 * x + dx, 0, Core - 1), sz = Mathf.Clamp(z0 + 2 * z + dz, 0, Core - 1);
                        bool inner = dx >= 0 && dx <= 1 && dz >= 0 && dz <= 1;
                        a[z, x, soil[sz * Core + sx]] += (inner ? 3f : 1f) / 24f;
                    }
                }
                td.SetAlphamaps(0, 0, a);
                EditorUtility.SetDirty(td);

                var mat = new Material(shader) { name = $"SiteTerrain_{i}_{j}" };
                mat.SetTexture("_MacroTex", macro);
                mat.SetVector("_MacroST", new Vector4(1f / Tiles, 1f / Tiles, (float)i / Tiles, (float)j / Tiles));
                mat.enableInstancing = true;
                AssetDatabase.CreateAsset(mat, $"{Gen}/Materials/SiteTerrain_{i}_{j}.mat");

                var go = Terrain.CreateTerrainGameObject(td);
                go.name = $"Terrain {(j == 0 ? "south" : "north")}-{(i == 0 ? "west" : "east")}";
                go.transform.SetParent(group, false);
                go.transform.position = new Vector3(x0 - Core / 2, ElevBase - Y0, z0 - Core / 2);
                var t = go.GetComponent<Terrain>();
                t.materialTemplate = mat;
                t.heightmapPixelError = 4;
                t.basemapDistance = 20000;          // always the full shader: the base map has no ground colour
                t.treeDistance = 450;
                t.treeBillboardDistance = 450;      // the shrub prefabs have no billboards
                t.drawInstanced = true;
                t.groupingID = 9;
                go.GetComponent<TerrainCollider>().sharedMaterial = dirt;
                GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic | StaticEditorFlags.OccluderStatic | StaticEditorFlags.OccludeeStatic);
                tiles[i, j] = t;
            }
            for (int j = 0; j < Tiles; j++)
            for (int i = 0; i < Tiles; i++)
                tiles[i, j].SetNeighbors(i > 0 ? tiles[i - 1, j] : null, j < Tiles - 1 ? tiles[i, j + 1] : null,
                    i < Tiles - 1 ? tiles[i + 1, j] : null, j > 0 ? tiles[i, j - 1] : null);
            shrubProtos = bushes;
        }

        // ------------------------------------------------------------------ shrubs (positions from the aerial photo)

        static (GameObject prefab, UrcSiteProps.ShrubKind kind, bool solid)[] shrubProtos;

        /// <summary>Generated shrubs (UrcSiteProps): every species as a solid plant (collider) and as a small one.</summary>
        static (GameObject prefab, UrcSiteProps.ShrubKind kind, bool solid)[] MakeShrubPrefabs()
        {
            var mat = new Material(Shader.Find("SimFlake/DesertShrub")) { name = "DesertShrub" };
            mat.mainTexture = AssetDatabase.LoadAssetAtPath<Texture2D>(Src + "/Props/ShrubAtlas.png");
            mat.enableInstancing = true;
            AssetDatabase.CreateAsset(mat, Gen + "/Materials/DesertShrub.mat");
            var list = new List<(GameObject, UrcSiteProps.ShrubKind, bool)>();
            int seed = 100;
            foreach (bool solid in new[] { true, false })
            foreach (var k in UrcSiteProps.Shrubs)
            {
                var mesh = UrcSiteProps.ShrubMesh(k, seed++);
                AssetDatabase.CreateAsset(mesh, $"{Gen}/Meshes/{mesh.name}.asset");
                var go = new GameObject(k.name + (solid ? "" : "_small"));
                go.AddComponent<MeshFilter>().sharedMesh = mesh;
                go.AddComponent<MeshRenderer>().sharedMaterial = mat;
                if (solid)
                {
                    // woody shrubs stop a rover and show on lidar; the small ones can be driven over
                    var cap = go.AddComponent<CapsuleCollider>();
                    cap.radius = 0.3f; cap.height = Mathf.Max(k.height, 0.6f); cap.center = new Vector3(0, cap.height / 2, 0);
                }
                list.Add((PrefabUtility.SaveAsPrefabAsset(go, $"{Gen}/Prefabs/{go.name}.prefab"), k, solid));
                Object.DestroyImmediate(go);
            }
            return list.ToArray();
        }

        static IEnumerable<float[]> Csv(string path)
        {
            foreach (var line in File.ReadLines(path).Skip(1))
            {
                var p = line.Split(',');
                if (p.Length < 3) continue;
                yield return p.Select(s => float.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            }
        }

        static void PlantShrubs()
        {
            var lists = new List<TreeInstance>[Tiles, Tiles];
            for (int j = 0; j < Tiles; j++) for (int i = 0; i < Tiles; i++) lists[i, j] = new List<TreeInstance>();
            int nKinds = UrcSiteProps.Shrubs.Length, total = 0, solidCount = 0;
            var perKind = new int[nKinds];
            foreach (var s in Csv(Src + "/shrubs.csv"))
            {
                float x = s[0], z = s[1], r = s[2];
                if (Mathf.Abs(x) > Core / 2 - 1 || Mathf.Abs(z) > Core / 2 - 1 || !Clear(x, z, r)) continue;
                int i = Mathf.Clamp((int)((x + Core / 2) / TileSize), 0, Tiles - 1), j = Mathf.Clamp((int)((z + Core / 2) / TileSize), 0, Tiles - 1);
                bool solid = r >= 0.55f;
                // which plant: the photo only says "a shrub this wide is here". Species by where it grows and its size.
                bool clay = SoilAt(x, z) == Soil.BentoniteClay;
                float pick = Random.value * UrcSiteProps.Shrubs.Sum(k => clay ? k.weightOnClay : solid ? k.weight : k.weightSmall);
                int kind = 0;
                for (; kind < nKinds - 1; kind++)
                {
                    var k = UrcSiteProps.Shrubs[kind];
                    pick -= clay ? k.weightOnClay : solid ? k.weight : k.weightSmall;
                    if (pick <= 0) break;
                }
                int proto = kind + (solid ? 0 : nKinds);      // prototypes: all kinds solid, then all kinds small
                // the photo's 0.6 m pixels make every shrub look at least 0.8 m wide: shrink the small ones
                float width = Mathf.Clamp(2f * r * (solid ? 0.8f : 0.7f), 0.3f, 1.6f);
                float tall = width * Random.Range(0.85f, 1.15f);
                tall = Mathf.Min(tall, 1.1f / UrcSiteProps.Shrubs[kind].height);      // nothing here grows much over a metre
                var o = tiles[i, j].transform.position;
                lists[i, j].Add(new TreeInstance
                {
                    position = new Vector3((x - o.x) / TileSize, 0, (z - o.z) / TileSize), prototypeIndex = proto,
                    widthScale = width, heightScale = tall,
                    rotation = Random.Range(0, Mathf.PI * 2), color = Color.white, lightmapColor = Color.white,
                });
                total++; perKind[kind]++; if (solid) solidCount++;
            }
            for (int j = 0; j < Tiles; j++) for (int i = 0; i < Tiles; i++) tiles[i, j].terrainData.SetTreeInstances(lists[i, j].ToArray(), true);
            report.AppendLine($"shrubs from the aerial photo: {total} ({solidCount} big enough to block the rover / show on lidar): " +
                              string.Join(", ", UrcSiteProps.Shrubs.Select((k, n) => $"{k.name} {perKind[n]}")));
        }

        // ------------------------------------------------------------------ rocks

        // generated sandstone blocks (UrcSiteProps.BoulderMesh): [colour][variant], big = boulders, small = stones
        static GameObject[][] rockBig, rockSmall;
        static Color[] rockColors;
        static readonly List<Vector3> placedRocks = new List<Vector3>();

        static void MakeRockPrefabs()
        {
            // rock colours measured in the aerial photo on the sandstone ledges and talus (Source/Props/palette.csv)
            rockColors = File.ReadLines(Src + "/Props/palette.csv").Where(l => l.StartsWith("rock,")).Select(l =>
            {
                var p = l.Split(',').Skip(1).Select(v => float.Parse(v, CultureInfo.InvariantCulture)).ToArray();
                return new Color(p[0], p[1], p[2]);
            }).ToArray();
            if (rockColors.Length == 0) throw new System.Exception("no rock colours in palette.csv");
            var albedo = AssetDatabase.LoadAssetAtPath<Texture2D>(Src + "/Props/Rock_Albedo.jpg");
            var normal = AssetDatabase.LoadAssetAtPath<Texture2D>(Src + "/Props/Rock_Normal.jpg");
            var rockMats = rockColors.Select((c, i) =>
            {
                // the texture is light grey (0.76): scale so texture x colour = the measured colour. Loose blocks are a
                // little darker than the ground they sit on (shaded sides, desert varnish).
                var m = new Material(Shader.Find("Standard")) { name = $"Sandstone_{i}", color = c * (0.95f / 0.76f) };
                m.mainTexture = albedo; m.SetTexture("_BumpMap", normal); m.EnableKeyword("_NORMALMAP"); m.SetFloat("_BumpScale", 1.2f);
                m.SetFloat("_Glossiness", 0.04f); m.SetFloat("_SpecularHighlights", 0f); m.EnableKeyword("_SPECULARHIGHLIGHTS_OFF");
                m.enableInstancing = true;
                AssetDatabase.CreateAsset(m, $"{Gen}/Materials/{m.name}.mat");
                return m;
            }).ToArray();

            GameObject[][] Make(string prefix, int count, int subdiv, int seed0)
            {
                var meshes = Enumerable.Range(0, count).Select(k =>
                {
                    // most blocks are slabs (they split along the bedding), a few are chunky
                    var m = UrcSiteProps.BoulderMesh(seed0 + k, subdiv, k % 4 == 3 ? 0.85f : Mathf.Lerp(0.38f, 0.62f, k / (float)count));
                    m.name = $"{prefix}_{k}";
                    AssetDatabase.CreateAsset(m, $"{Gen}/Meshes/{m.name}.asset");
                    return m;
                }).ToArray();
                return rockMats.Select((mat, ci) => meshes.Select(mesh =>
                {
                    var go = new GameObject($"{mesh.name}_c{ci}");
                    go.AddComponent<MeshFilter>().sharedMesh = mesh;
                    go.AddComponent<MeshRenderer>().sharedMaterial = mat;
                    var mc = go.AddComponent<MeshCollider>();
                    mc.sharedMesh = mesh; mc.convex = true; mc.sharedMaterial = dirt;
                    var prefab = PrefabUtility.SaveAsPrefabAsset(go, $"{Gen}/Prefabs/{go.name}.prefab");
                    Object.DestroyImmediate(go);
                    return prefab;
                }).ToArray()).ToArray();
            }
            rockBig = Make("Boulder", 8, 2, 500);
            rockSmall = Make("Stone", 6, 1, 700);
        }

        static bool PlaceRock(Transform parent, float x, float z, float size, float sink)
        {
            if (!Clear(x, z, size)) return false;
            float rad = size * 0.5f;
            foreach (var q in placedRocks) if (Sq(x - q.x) + Sq(z - q.y) < Sq((rad + q.z) * 0.8f)) return false;
            placedRocks.Add(new Vector3(x, z, rad));
            // rock colour: the measured colour nearest to the ground right here (talus is the same rock as its slope),
            // now and then another one
            var gc = GroundColorAt(x, z);
            int ci = 0; float bestD = float.MaxValue;
            for (int k = 0; k < rockColors.Length; k++)
            {
                float d = Sq(rockColors[k].r - gc.r) + Sq(rockColors[k].g - gc.g) + Sq(rockColors[k].b - gc.b);
                if (d < bestD) { bestD = d; ci = k; }
            }
            if (Random.value < 0.25f) ci = Random.Range(0, rockColors.Length);
            var set = size >= 0.45f ? rockBig : rockSmall;
            var rock = (GameObject)PrefabUtility.InstantiatePrefab(set[ci][Random.Range(0, set[ci].Length)], parent);
            rock.name = $"rock_{size:F2}m";
            // the meshes are 1 m across and already slab-shaped
            rock.transform.localScale = new Vector3(size * Random.Range(0.9f, 1.1f), size * Random.Range(0.8f, 1.15f), size * Random.Range(0.9f, 1.1f));
            // slabs lie roughly flat; on a slope they lie on the slope
            rock.transform.rotation = Quaternion.Euler(Random.Range(-14f, 14f), Random.Range(0, 360f), Random.Range(-14f, 14f));
            rock.transform.position = new Vector3(x, Ground(x, z), z);
            var b = rock.GetComponentInChildren<Renderer>().bounds;
            float gmin = Mathf.Min(Ground(x, z), Mathf.Min(Mathf.Min(Ground(x + rad, z), Ground(x - rad, z)), Mathf.Min(Ground(x, z + rad), Ground(x, z - rad))));
            rock.transform.position += Vector3.up * (gmin - b.min.y - sink * b.size.y);
            GameObjectUtility.SetStaticEditorFlags(rock, StaticEditorFlags.BatchingStatic);
            return true;
        }

        static void BuildRocks()
        {
            MakeRockPrefabs();
            placedRocks.Clear();
            var big = new GameObject("Boulders (from the aerial photo)").transform; big.SetParent(root, false);
            int nb = 0;
            foreach (var b in Csv(Src + "/boulders.csv"))
                if (Mathf.Abs(b[0]) < Core / 2 - 3 && Mathf.Abs(b[1]) < Core / 2 - 3 && PlaceRock(big, b[0], b[1], Mathf.Clamp(b[2] * 0.8f, 0.6f, 3f), 0.25f)) nb++;

            report.AppendLine($"boulders from the aerial photo: {nb} (smaller stones come from the StoneField at run time)");
        }

        // ------------------------------------------------------------------ ground under 1 m: soils and stones

        static GroundSpec[] siteSpecs;

        /// <summary>The site's own GroundSpecs (copies of the Utah ones with this site's roughness). Existing assets are kept: edit them in the inspector.</summary>
        static void MakeSiteSoils()
        {
            siteSpecs = GroundTable.Select(g =>
            {
                string path = $"{DrivetrainSetup.Root}/Grounds/{g.spec}.asset";
                var a = AssetDatabase.LoadAssetAtPath<GroundSpec>(path);
                if (a != null) return a;
                a = Object.Instantiate(AssetDatabase.LoadAssetAtPath<GroundSpec>($"{DrivetrainSetup.Root}/Grounds/{g.from}.asset"));
                a.name = g.spec;
                a.roughnessFine = g.fine; a.roughnessCoarse = g.coarse;
                a.notes = $"{g.notes} Copy of {g.from} with rougher small-scale relief (UrcSiteBuilder.GroundTable). Not measured on site.";
                AssetDatabase.CreateAsset(a, path);
                return a;
            }).ToArray();
            report.AppendLine("soils by layer: " + string.Join(", ", siteSpecs.Select((sp, i) => $"{LayerSpec[i].layerName} = {sp.name} (relief {sp.roughnessFine * 1000:F0} / {sp.roughnessCoarse * 1000:F0} mm)")));
            foreach (var t in tiles)
            {
                var st = SoilSetup.MakeSoil(t);
                st.layerSpecs = siteSpecs;          // layer order = GroundTable order = Soil enum
                st.defaultSpec = siteSpecs[0];
                EditorUtility.SetDirty(st);
            }
        }

        static void BuildStoneField()
        {
            var go = new GameObject("Stones (StoneField: appear around the rover in Play)");
            go.transform.SetParent(root, false);
            var sf = go.AddComponent<StoneField>();
            sf.soils = GroundTable.Select((g, i) => new StoneField.SoilStones { soil = siteSpecs[i], rockAbundance = g.k, maxDiameter = g.maxStone, looseFraction = g.loose }).ToArray();
            sf.stonePrefabs = rockSmall.SelectMany(a => a).ToArray();
            sf.rockPrefabs = rockBig.SelectMany(a => a).ToArray();
            sf.clearZones = keepClear.ToArray();
            sf.farRadius = 25f; sf.maxStones = 3500;
            report.AppendLine("stones per m2 (5-15 cm / over 15 cm), from the rock abundance per soil: " + string.Join(", ", GroundTable.Select(g =>
                $"{g.soil} {StoneField.Count(g.k, sf.minDiameter, Mathf.Min(sf.bigFrom, g.maxStone)):F1} / {StoneField.Count(g.k, sf.bigFrom, g.maxStone):F2}")));
        }

        // ------------------------------------------------------------------ backdrop (16 km around the site)

        static void BuildBackdrop()
        {
            var b = File.ReadAllBytes(Src + "/far_heights.bytes");
            var far = new ushort[FarN * FarN];
            System.Buffer.BlockCopy(b, 0, far, 0, b.Length);
            float FarY(float x, float z)
            {
                float fx = Mathf.Clamp((x / FarSize + 0.5f) * (FarN - 1), 0, FarN - 1.001f), fz = Mathf.Clamp((z / FarSize + 0.5f) * (FarN - 1), 0, FarN - 1.001f);
                int xi = (int)fx, zi = (int)fz; float u = fx - xi, v = fz - zi;
                float h = Mathf.Lerp(Mathf.Lerp(far[zi * FarN + xi], far[zi * FarN + xi + 1], u), Mathf.Lerp(far[(zi + 1) * FarN + xi], far[(zi + 1) * FarN + xi + 1], u), v);
                return FarBase + h / 65535f * FarRange - Y0;
            }
            // grid lines: 16 m near the site, 128 m far away
            var g = new List<float>();
            float half = FarSize / 2, fine = 2304;
            for (float v = -half; v < -fine; v += 128) g.Add(v);
            for (float v = -fine; v < fine; v += 32) g.Add(v);
            for (float v = fine; v <= half; v += 128) g.Add(v);
            int n = g.Count;
            float core = Core / 2;
            var verts = new Vector3[n * n]; var uvs = new Vector2[n * n];
            for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                float x = g[i], z = g[j];
                float y = FarY(x, z);
                // under the real terrain and just outside it, sink so the coarse mesh never pokes through the 1 m terrain
                float d = Mathf.Max(Mathf.Abs(x), Mathf.Abs(z)) - core;
                y -= d < 0 ? 8f : Mathf.Lerp(2.5f, 0f, Mathf.Clamp01(d / 400f));
                verts[j * n + i] = new Vector3(x, y, z);
                uvs[j * n + i] = new Vector2(x / FarSize + 0.5f, z / FarSize + 0.5f);
            }
            var tris = new List<int>();
            for (int j = 0; j < n - 1; j++)
            for (int i = 0; i < n - 1; i++)
            {
                // leave a hole under the site (one cell of overlap)
                if (Mathf.Max(Mathf.Abs(g[i]), Mathf.Abs(g[i + 1])) < core - 40 && Mathf.Max(Mathf.Abs(g[j]), Mathf.Abs(g[j + 1])) < core - 40) continue;
                int a = j * n + i;
                tris.AddRange(new[] { a, a + n, a + 1, a + 1, a + n, a + n + 1 });
            }
            var mesh = new Mesh { name = "Backdrop", indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            mesh.vertices = verts; mesh.uv = uvs; mesh.triangles = tris.ToArray();
            mesh.RecalculateNormals(); mesh.RecalculateBounds();
            AssetDatabase.CreateAsset(mesh, Gen + "/Meshes/Backdrop.asset");
            var mat = new Material(Shader.Find("Standard")) { name = "Backdrop" };
            mat.mainTexture = AssetDatabase.LoadAssetAtPath<Texture2D>(Src + "/far_color.jpg");
            mat.SetFloat("_Glossiness", 0f); mat.SetFloat("_SpecularHighlights", 0f); mat.SetFloat("_GlossyReflections", 0f);
            mat.EnableKeyword("_SPECULARHIGHLIGHTS_OFF"); mat.EnableKeyword("_GLOSSYREFLECTIONS_OFF");
            AssetDatabase.CreateAsset(mat, Gen + "/Materials/Backdrop.mat");
            var go = new GameObject("Backdrop (16 km, 16 m DEM + Sentinel-2, no collider)");
            go.transform.SetParent(root, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterial = mat; mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            report.AppendLine($"backdrop: {n} x {n} vertices, {tris.Count / 3} triangles");
        }

        // ------------------------------------------------------------------ props

        static Material M(string name, Color c, float smooth = 0.15f)
        {
            if (mats.TryGetValue(name, out var m)) return m;
            m = new Material(Shader.Find("Standard")) { name = name, color = c };
            m.SetFloat("_Glossiness", smooth);
            AssetDatabase.CreateAsset(m, $"{Gen}/Materials/{name}.mat");
            return mats[name] = m;
        }

        static GameObject Prim(PrimitiveType type, Transform parent, string name, Vector3 localPos, Vector3 scale, Material mat, Vector3 euler = default, bool collider = true)
        {
            var go = GameObject.CreatePrimitive(type);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = localPos; go.transform.localEulerAngles = euler; go.transform.localScale = scale;
            go.GetComponent<Renderer>().sharedMaterial = mat;
            if (!collider) Object.DestroyImmediate(go.GetComponent<Collider>());
            return go;
        }

        static Transform Group(Transform parent, string name, Vector3 pos, float yaw = 0)
        {
            var t = new GameObject(name).transform;
            t.SetParent(parent, false);
            t.position = pos; t.rotation = Quaternion.Euler(0, yaw, 0);
            return t;
        }

        static void Label(Transform parent, string text, Vector3 localPos, float size, Color color)
        {
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            var t = new GameObject("Label: " + text).AddComponent<TextMesh>();
            t.transform.SetParent(parent, false);
            t.transform.localPosition = localPos;
            t.text = text; t.font = font; t.fontSize = 80; t.characterSize = size; t.anchor = TextAnchor.MiddleCenter; t.color = color;
            t.GetComponent<MeshRenderer>().sharedMaterial = font.material;
        }

        /// <summary>A thin pole with a small flag, e.g. to mark a point for people looking at the scene.</summary>
        static void Flag(Transform parent, string name, Vector2 p, Material mat, float h = 1.6f)
        {
            var g = Group(parent, name, OnGround(p));
            Prim(PrimitiveType.Cylinder, g, "pole", new Vector3(0, h / 2, 0), new Vector3(0.025f, h / 2, 0.025f), M("PoleGrey", new Color(0.75f, 0.75f, 0.75f), 0.4f), default, false);
            Prim(PrimitiveType.Cube, g, "flag", new Vector3(0.2f, h - 0.14f, 0), new Vector3(0.4f, 0.26f, 0.01f), mat, default, false);
        }

        static void BuildSite()
        {
            var site = new GameObject("Site (C2, start gate, astronaut)").transform; site.SetParent(root, false);
            var white = M("CanopyWhite", new Color(0.93f, 0.93f, 0.9f)); var steel = M("PoleGrey", new Color(0.75f, 0.75f, 0.75f), 0.4f);
            var orange = M("SafetyOrange", new Color(1f, 0.42f, 0.05f)); var dark = M("DarkPlastic", new Color(0.12f, 0.12f, 0.13f), 0.3f);
            var wood = M("TableTop", new Color(0.55f, 0.45f, 0.33f)); var blue = M("FlagBlue", new Color(0.1f, 0.35f, 0.9f));

            // command and control station: a canopy facing the course, two tables, the antenna mast
            Vector2 toStart = (MissionStart - C2).normalized;
            float yaw = Mathf.Atan2(toStart.x, toStart.y) * Mathf.Rad2Deg;
            var c2 = Group(site, "C2 station", OnGround(C2), yaw);
            foreach (var (sx, sz) in new[] { (-1.5f, -1.5f), (1.5f, -1.5f), (-1.5f, 1.5f), (1.5f, 1.5f) })
                Prim(PrimitiveType.Cylinder, c2, "leg", new Vector3(sx, 1.1f, sz), new Vector3(0.04f, 1.1f, 0.04f), steel);
            Prim(PrimitiveType.Cube, c2, "canopy", new Vector3(0, 2.3f, 0), new Vector3(3.3f, 0.06f, 3.3f), white);
            Prim(PrimitiveType.Cube, c2, "back wall", new Vector3(0, 1.25f, -1.55f), new Vector3(3.1f, 2.0f, 0.03f), white);
            foreach (float tx in new[] { -0.8f, 0.8f })
            {
                Prim(PrimitiveType.Cube, c2, "table", new Vector3(tx, 0.74f, 0.2f), new Vector3(1.4f, 0.04f, 0.7f), wood);
                Prim(PrimitiveType.Cube, c2, "laptop", new Vector3(tx, 0.87f, 0.3f), new Vector3(0.34f, 0.22f, 0.02f), dark, new Vector3(-15, 0, 0), false);
                foreach (float lx in new[] { -0.6f, 0.6f }) foreach (float lz in new[] { -0.28f, 0.28f })
                    Prim(PrimitiveType.Cube, c2, "table leg", new Vector3(tx + lx, 0.36f, 0.2f + lz), new Vector3(0.03f, 0.72f, 0.03f), steel, default, false);
            }
            Prim(PrimitiveType.Cylinder, c2, "antenna mast", new Vector3(2.3f, MastHeight / 2, -0.5f), new Vector3(0.05f, MastHeight / 2, 0.05f), steel);
            Prim(PrimitiveType.Cube, c2, "sector antenna", new Vector3(2.3f, MastHeight - 0.2f, -0.42f), new Vector3(0.14f, 0.5f, 0.06f), white, default, false);
            Label(c2, "C2", new Vector3(0, 2.75f, 1.66f), 0.07f, Color.white);

            // start gate: two cones' worth of posts 2 m apart, the rover leaves through it
            Vector2 gdir = (MissionStart - Gate).normalized;
            var gate = Group(site, "Start gate", OnGround(Gate), Mathf.Atan2(gdir.x, gdir.y) * Mathf.Rad2Deg);
            foreach (float sx in new[] { -1f, 1f })
            {
                Prim(PrimitiveType.Cylinder, gate, "post", new Vector3(sx, 0.6f, 0), new Vector3(0.05f, 0.6f, 0.05f), orange);
                Prim(PrimitiveType.Cylinder, gate, "base", new Vector3(sx, 0.03f, 0), new Vector3(0.3f, 0.03f, 0.3f), dark);
            }

            // drone landing pad (1.e.xiv: the drone must be back on it before the rover leaves the mission start)
            var pad = Group(site, "Drone landing pad", OnGround(DronePad, 0.02f));
            Prim(PrimitiveType.Cylinder, pad, "pad", Vector3.zero, new Vector3(1.5f, 0.01f, 1.5f), orange, default, false);
            Prim(PrimitiveType.Cylinder, pad, "centre", new Vector3(0, 0.004f, 0), new Vector3(0.9f, 0.01f, 0.9f), white, default, false);

            Flag(site, "Mission start (route finding, GNSS given)", MissionStart, blue);
            Flag(site, "Astronaut 'follow me' destination", FollowTo, orange);

            // astronaut waiting at a GNSS point (1.e.v): a suit-sized figure the cameras and lidar can see
            Vector2 look = (MissionStart - Astronaut).normalized;
            var a = Group(site, "Astronaut (GNSS point, 1.e.v)", OnGround(Astronaut), Mathf.Atan2(look.x, look.y) * Mathf.Rad2Deg);
            var suit = M("SuitWhite", new Color(0.9f, 0.9f, 0.88f), 0.3f);
            Prim(PrimitiveType.Capsule, a, "torso", new Vector3(0, 1.15f, 0), new Vector3(0.48f, 0.38f, 0.34f), suit);
            foreach (float sx in new[] { -0.12f, 0.12f }) Prim(PrimitiveType.Capsule, a, "leg", new Vector3(sx, 0.43f, 0), new Vector3(0.19f, 0.43f, 0.19f), suit);
            foreach (float sx in new[] { -0.31f, 0.31f }) Prim(PrimitiveType.Capsule, a, "arm", new Vector3(sx, 1.12f, 0), new Vector3(0.13f, 0.34f, 0.13f), suit, new Vector3(0, 0, sx > 0 ? 8 : -8));
            Prim(PrimitiveType.Sphere, a, "helmet", new Vector3(0, 1.68f, 0), new Vector3(0.36f, 0.36f, 0.36f), suit);
            Prim(PrimitiveType.Sphere, a, "visor", new Vector3(0, 1.69f, 0.08f), new Vector3(0.27f, 0.25f, 0.25f), M("VisorGold", new Color(0.75f, 0.55f, 0.15f), 0.9f), default, false);
            Prim(PrimitiveType.Cube, a, "life support pack", new Vector3(0, 1.22f, -0.23f), new Vector3(0.4f, 0.55f, 0.2f), M("PackGrey", new Color(0.6f, 0.6f, 0.62f)));

            // rock pick hammer to fetch (1.e.viii): ~33 cm, 0.65 kg, loose on the ground
            var h = Group(site, "Rock pick hammer (fetch, 1.e.viii)", OnGround(Hammer, 0.03f), 40);
            var rb = h.gameObject.AddComponent<Rigidbody>(); rb.mass = 0.65f;
            Prim(PrimitiveType.Cylinder, h, "handle", new Vector3(0, 0, 0), new Vector3(0.032f, 0.14f, 0.032f), blue, new Vector3(90, 0, 0));
            Prim(PrimitiveType.Cube, h, "head", new Vector3(0, 0, 0.15f), new Vector3(0.17f, 0.028f, 0.03f), steel);

            // corner posts of the rules' square mile
            var corners = Group(site, "Square mile corners (rule 1.e.xiii)", Vector3.zero);
            foreach (var c in new[] { new Vector2(SquareMile.xMin, SquareMile.yMin), new Vector2(SquareMile.xMax, SquareMile.yMin), new Vector2(SquareMile.xMin, SquareMile.yMax), new Vector2(SquareMile.xMax, SquareMile.yMax) })
                Prim(PrimitiveType.Cylinder, corners, "corner post", OnGround(c, 1.5f), new Vector3(0.08f, 1.5f, 0.08f), orange);
        }

        static void BuildTargets()
        {
            var group = new GameObject("Targets (AR posts)").transform; group.SetParent(root, false);
            var markerPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(ArucoPrefabPath);
            if (markerPrefab == null) Debug.LogWarning($"[UrcSite] {ArucoPrefabPath} not found, targets get no ArUco posts");
            foreach (var w in Targets)
            {
                var go = new GameObject($"{w.name} (ArUco {w.aruco})");
                go.transform.SetParent(group, false);
                go.transform.position = OnGround(w.p);
                if (markerPrefab == null) continue;
                // the team's post prefab: three 20 cm faces (4x4_50), the texture is set from 'id' when Play starts
                var marker = (GameObject)PrefabUtility.InstantiatePrefab(markerPrefab, go.transform);
                marker.GetComponent<TripleAruco>().id = w.aruco;
                PrefabUtility.RecordPrefabInstancePropertyModifications(marker.GetComponent<TripleAruco>());
                var post = marker.GetComponentsInChildren<Renderer>().FirstOrDefault(r => r.name == "Post");
                if (post != null)
                {
                    var b = post.bounds;
                    marker.transform.position += go.transform.position - new Vector3(b.center.x, b.min.y, b.center.z);
                }
            }
        }

        // ------------------------------------------------------------------ rovers, camera

        /// <summary>WGS84 latitude / longitude of a world point (inverse UTM 12N).</summary>
        public static void LatLon(float x, float z, out double lat, out double lon)
        {
            const double a = 6378137.0, f = 1 / 298.257222101, k0 = 0.9996;
            double e2 = f * (2 - f), ep2 = e2 / (1 - e2);
            double E = OriginE + x - 500000.0, Nn = OriginN + z;
            double mu = Nn / k0 / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
            double e1 = (1 - System.Math.Sqrt(1 - e2)) / (1 + System.Math.Sqrt(1 - e2));
            double p = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * System.Math.Sin(2 * mu) + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * System.Math.Sin(4 * mu)
                       + 151 * e1 * e1 * e1 / 96 * System.Math.Sin(6 * mu) + 1097 * e1 * e1 * e1 * e1 / 512 * System.Math.Sin(8 * mu);
            double sp = System.Math.Sin(p), cp = System.Math.Cos(p), tp = System.Math.Tan(p);
            double N1 = a / System.Math.Sqrt(1 - e2 * sp * sp), T = tp * tp, C = ep2 * cp * cp;
            double R1 = a * (1 - e2) / System.Math.Pow(1 - e2 * sp * sp, 1.5), D = E / (N1 * k0);
            lat = p - N1 * tp / R1 * (D * D / 2 - (5 + 3 * T + 10 * C - 4 * C * C - 9 * ep2) * D * D * D * D / 24
                                      + (61 + 90 * T + 298 * C + 45 * T * T - 252 * ep2 - 3 * C * C) * System.Math.Pow(D, 6) / 720);
            lon = (D - (1 + 2 * T + C) * D * D * D / 6 + (5 - 2 * C + 28 * T - 3 * C * C + 8 * ep2 + 24 * T * T) * System.Math.Pow(D, 5) / 120) / cp;
            lat *= 180 / System.Math.PI; lon = -111.0 + lon * 180 / System.Math.PI;
        }

        /// <summary>Where the rovers start: just behind the start gate, facing through it.</summary>
        static Vector2 RoverStart => Gate - (MissionStart - Gate).normalized * 2.5f;

        static void EnsureRovers()
        {
            var start = RoverStart;
            Vector2 dir = (MissionStart - Gate).normalized;
            LatLon(start.x, start.y, out double lat, out double lon);
            GameObject Rover(string path, string name, bool active)
            {
                var existing = Object.FindObjectsOfType<Transform>(true).FirstOrDefault(t => t.parent == null && t.name == name);
                var go = existing ? existing.gameObject : (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(path));
                if (!existing) { go.name = name; go.SetActive(active); }
                // put it on the ground: lowest renderer 5 cm above the terrain
                go.transform.SetPositionAndRotation(new Vector3(start.x, 0, start.y), Quaternion.Euler(0, Mathf.Atan2(dir.x, dir.y) * Mathf.Rad2Deg, 0));
                var rs = go.GetComponentsInChildren<Renderer>(true).Where(r => !(r is ParticleSystemRenderer) && r.GetComponent<TextMesh>() == null && r.GetComponent<TMPro.TMP_Text>() == null).ToArray();
                float minY = rs.Length > 0 ? rs.Min(r => r.bounds.min.y) : 0;
                float g = Enumerable.Range(0, 9).Max(k => Ground(start.x + (k % 3 - 1) * 0.5f, start.y + (k / 3 - 1) * 0.5f));
                go.transform.position += Vector3.up * (g + 0.05f - minY);
                // the GNSS receiver reports real coordinates: its origin is the rover's start
                foreach (var gps in go.GetComponentsInChildren<LocationStuff>(true))
                {
                    gps.startLatitude = lat; gps.startLongitude = lon; gps.startAltitude = Ground(start) + Y0;
                    PrefabUtility.RecordPrefabInstancePropertyModifications(gps);
                }
                PrefabUtility.RecordPrefabInstancePropertyModifications(go.transform);
                return go;
            }
            // one rover at a time: both use the same topics
            Rover(DrivetrainSetup.Chassis2024, "chassis_24", true);
            Rover(DrivetrainSetup.MiniRover, "mini_rover (off: same topics as chassis_24)", false);
            report.AppendLine(string.Format(CultureInfo.InvariantCulture, "rover start ({0:F1}, {1:F1}) = {2:F7}, {3:F7}, {4:F1} m (GNSS origin of both rovers)", start.x, start.y, lat, lon, Ground(start) + Y0));
        }

        static void EnsureCamera()
        {
            var cam = Object.FindObjectsOfType<Camera>().FirstOrDefault(c => c.GetComponent<FreeFlyCamera>());
            if (cam == null)
            {
                cam = new GameObject("Spectator Camera (right-drag + WASD)").AddComponent<Camera>();
                cam.tag = "MainCamera";
                cam.gameObject.AddComponent<AudioListener>();
                cam.gameObject.AddComponent<FreeFlyCamera>();
                var p = C2 + new Vector2(-9, -7);
                cam.transform.position = OnGround(p, 3.2f);
                cam.transform.LookAt(OnGround(MissionStart, 1f));
            }
            cam.nearClipPlane = 0.2f; cam.farClipPlane = 14000;
        }

        // ------------------------------------------------------------------ waypoints, checks

        static void WriteWaypointFile()
        {
            // what the sim's GNSS will report at a point: LocationStuff's flat-earth formula from the rover start
            const double R = 6378137.0;
            var start = RoverStart;
            LatLon(start.x, start.y, out double lat0, out double lon0);
            var sb = new StringBuilder();
            sb.AppendLine("# URC autonomy site: task points (generated by UrcSiteBuilder, don't edit). x = east, z = north, metres from the");
            sb.AppendLine("# centre of the URC 2027 square mile (38.418, -110.777; UTM 12N 519467 E 4252218 N). The targets are our own choice:");
            sb.AppendLine("# URC publishes the real ones at the competition. sim_lat/sim_lon = what /gps/fix reports there (flat-earth from the");
            sb.AppendLine("# rover start); true_lat/true_lon = the real place (they differ by < 1.5 m inside the task area).");
            sb.AppendLine("name, unity_x, unity_z, sim_lat, sim_lon, true_lat, true_lon, elevation_m, soil, max_slope_within_2m_deg, note");
            void Row(string name, Vector2 p, string note)
            {
                double e = p.x - start.x, n = p.y - start.y;
                double slat = lat0 + n / R * 180 / System.Math.PI, slon = lon0 + e / (R * System.Math.Cos(lat0 * System.Math.PI / 180)) * 180 / System.Math.PI;
                LatLon(p.x, p.y, out double tlat, out double tlon);
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture, "{0}, {1:F1}, {2:F1}, {3:F7}, {4:F7}, {5:F7}, {6:F7}, {7:F1}, {8}, {9:F0}, {10}",
                    name, p.x, p.y, slat, slon, tlat, tlon, Ground(p) + Y0, SoilAt(p.x, p.y), MaxSlope(p, 2), note));
            }
            Row("ROVER START", start, "Behind the start gate. GNSS origin of the rovers.");
            Row("C2", C2, $"Command and control station, antenna mast {MastHeight:F0} m. On the real URC staging area.");
            Row("DRONE PAD", DronePad, "Rule 1.e.xiv.");
            Row("MISSION START", MissionStart, "Rule 1.e.xii: flat easy ground, teleoperation allowed up to here.");
            Row("ASTRONAUT", Astronaut, "Rule 1.e.v: drive to the astronaut, stop within 3 m.");
            Row("FOLLOW TO", FollowTo, "Rule 1.e.vi: where the astronaut walks to (the figure in the scene does not move).");
            Row("HAMMER", Hammer, "Rule 1.e.viii: rock pick hammer to fetch.");
            foreach (var t in Targets) Row($"{t.name} (ArUco {t.aruco})", t.p, t.note);
            File.WriteAllText(Dir + "/waypoints.csv", sb.ToString());
            AssetDatabase.ImportAsset(Dir + "/waypoints.csv");
            report.Append(sb);
        }

        /// <summary>Highest the ground rises above the straight line between two points (m): > 0 means no line of sight.</summary>
        static float Blocking(Vector3 from, Vector3 to)
        {
            float worst = float.NegativeInfinity;
            int n = Mathf.CeilToInt(Vector3.Distance(from, to));
            for (int k = 1; k < n; k++)
            {
                var p = Vector3.Lerp(from, to, (float)k / n);
                worst = Mathf.Max(worst, Ground(p.x, p.z) - p.y);
            }
            return worst;
        }

        static readonly string[] Compass = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };

        /// <summary>
        /// Smallest "steepest slope on the way" over all paths (1 m grid) from 'range' metres out on a bearing to within 2 m
        /// of the top, staying within +-halfWidth degrees of that bearing as seen from the top.
        /// </summary>
        static float WedgeBottleneck(Vector2 top, float bearingDeg, float range, float halfWidth)
        {
            int r = Mathf.CeilToInt(range) + 2, w = 2 * r + 1;
            var best = new float[w * w];
            for (int k = 0; k < best.Length; k++) best[k] = float.MaxValue;
            bool Inside(int ix, int iz)
            {
                float dx = ix - r, dz = iz - r, d = Mathf.Sqrt(dx * dx + dz * dz);
                if (d > range + 1) return false;
                if (d < 4) return true;
                return Mathf.Abs(Mathf.DeltaAngle(Mathf.Atan2(dx, dz) * Mathf.Rad2Deg, bearingDeg)) <= halfWidth;
            }
            float Slope(int ix, int iz) => SlopeDeg(top.x + ix - r, top.y + iz - r);
            var open = new SortedSet<(float cost, int idx)>();
            int sx = r + Mathf.RoundToInt(Mathf.Sin(bearingDeg * Mathf.Deg2Rad) * range), sz = r + Mathf.RoundToInt(Mathf.Cos(bearingDeg * Mathf.Deg2Rad) * range);
            best[sz * w + sx] = Slope(sx, sz);
            open.Add((best[sz * w + sx], sz * w + sx));
            while (open.Count > 0)
            {
                var cur = open.Min; open.Remove(cur);
                int ix = cur.idx % w, iz = cur.idx / w;
                if (Sq(ix - r) + Sq(iz - r) <= 4) return cur.cost;
                for (int dz = -1; dz <= 1; dz++) for (int dx = -1; dx <= 1; dx++)
                {
                    int nx = ix + dx, nz = iz + dz;
                    if ((dx == 0 && dz == 0) || nx < 0 || nz < 0 || nx >= w || nz >= w || !Inside(nx, nz)) continue;
                    float c = Mathf.Max(cur.cost, Slope(nx, nz));
                    int ni = nz * w + nx;
                    if (c < best[ni]) { open.Remove((best[ni], ni)); best[ni] = c; open.Add((c, ni)); }
                }
            }
            return 90;
        }

        static void Verify()
        {
            var sb = new StringBuilder("checks:\n");
            bool ok = true;
            void Check(bool pass, string what) { sb.AppendLine($"  {(pass ? "ok  " : "FAIL")} {what}"); ok &= pass; }

            // the DEM in the scene is the DEM in the file
            float worst = 0;
            var rnd = new System.Random(5);
            for (int k = 0; k < 4000; k++)
            {
                float x = (float)(rnd.NextDouble() - 0.5) * (Core - 4), z = (float)(rnd.NextDouble() - 0.5) * (Core - 4);
                var t = tiles[x < 0 ? 0 : 1, z < 0 ? 0 : 1];
                worst = Mathf.Max(worst, Mathf.Abs(t.SampleHeight(new Vector3(x, 0, z)) + t.transform.position.y - Ground(x, z)));
            }
            Check(worst < 0.02f, $"terrain height = lidar DEM (worst difference {worst * 1000:F1} mm; the 16-bit heightmap step is {ElevRange / 65535 * 1000:F1} mm)");
            LatLon(0, 0, out double lat, out double lon);
            Check(System.Math.Abs(lat - OriginLat) < 2e-5 && System.Math.Abs(lon - OriginLon) < 2e-5, $"georeference: world origin = {lat:F6}, {lon:F6} (expected {OriginLat}, {OriginLon})");
            Check(MaxSlope(MissionStart, 3) < 8 && MaxSlope(Gate, 3) < 8 && MaxSlope(RoverStart, 2) < 8,
                $"start is flat easy ground (max slope: rover start {MaxSlope(RoverStart, 2):F0}, gate {MaxSlope(Gate, 3):F0}, mission start {MaxSlope(MissionStart, 3):F0} deg)");
            var mast = OnGround(C2, MastHeight);
            foreach (var t in Targets)
            {
                float b = Blocking(mast, OnGround(t.p, 1.0f));
                float dist = Vector2.Distance(MissionStart, t.p), climb = Ground(t.p) - Ground(MissionStart);
                sb.AppendLine($"       {t.name}: {dist:F0} m from the mission start, {climb:+0.0;-0.0} m, soil {SoilAt(t.p.x, t.p.y)}, " +
                              (b > 0 ? $"NO radio line of sight to the C2 mast (ground {b:F1} m above the line)" : $"line of sight to the C2 mast (clear by {-b:F1} m)"));
            }
            Check(Blocking(mast, OnGround(Targets[0].p, 1f)) < 0, "T1 (hill) is in view of the C2 mast");
            Check(Blocking(mast, OnGround(Targets[1].p, 1f)) > 1, "T2 is hidden from the C2 mast (rule 1.e.xvi)");
            // "not all approach directions will be navigable": the easiest line to the top from 70 m out, kept inside a
            // 45 deg wedge on each bearing; the number is the steepest slope that line can't avoid
            var hill = Targets[0].p;
            var runs = new List<string>(); int steep = 0;
            for (int az = 0; az < 360; az += 45)
            {
                float m = WedgeBottleneck(hill, az, 70f, 22.5f);
                runs.Add($"{Compass[az / 45]} {m:F0}"); if (m > 24) steep++;
            }
            float any = WedgeBottleneck(hill, 0, 70f, 180f);
            Check(steep >= 2 && any < 20, $"T1 hill can't be climbed from every side. Steepest unavoidable slope coming straight in from: {string.Join(", ", runs)} deg; " +
                                          $"the easiest winding route needs {any:F0} deg");
            // soils reach the soil model
            foreach (var t in tiles) t.GetComponent<SoilTerrain>().Reinit();
            var counts = new int[6];
            foreach (byte s in soil) counts[s]++;
            sb.AppendLine("       soil map: " + string.Join(", ", Enumerable.Range(0, 6).Select(k => $"{(Soil)k} {100f * counts[k] / soil.Length:F1} %")));
            var st = tiles[0, 1].GetComponent<SoilTerrain>();
            Check(st.SpecAt(MissionStart.x, MissionStart.y) != null, $"soil model sees the terrain (mission start = {st.SpecAt(MissionStart.x, MissionStart.y)?.name})");
            report.Append(sb);
            if (!ok) Debug.LogError("[UrcSite] some checks FAILED, see the report");
        }

        // ------------------------------------------------------------------ screenshots

        static string Arg(string name)
        {
            var a = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < a.Length - 1; i++) if (a[i] == name) return a[i + 1];
            return null;
        }

        static void Screenshots(string dir)
        {
            Directory.CreateDirectory(dir);
            var cam = new GameObject("shotcam").AddComponent<Camera>();
            cam.nearClipPlane = 0.3f; cam.farClipPlane = 14000;
            float shadow = QualitySettings.shadowDistance;
            void Shot(string name, int w, int h)
            {
                var rt = new RenderTexture(w, h, 24) { antiAliasing = 4 };
                cam.targetTexture = rt; cam.Render();
                RenderTexture.active = rt;
                var tex = new Texture2D(w, h, TextureFormat.RGB24, false);
                tex.ReadPixels(new Rect(0, 0, w, h), 0, 0); tex.Apply();
                File.WriteAllBytes(Path.Combine(dir, name + ".jpg"), tex.EncodeToJPG(88));
                RenderTexture.active = null; cam.targetTexture = null;
                Object.DestroyImmediate(rt); Object.DestroyImmediate(tex);
            }
            cam.orthographic = true; cam.aspect = 1;
            cam.transform.rotation = Quaternion.Euler(90, 0, 0);
            QualitySettings.shadowDistance = 3000;
            cam.orthographicSize = Core / 2; cam.transform.position = new Vector3(0, 600, 0); Shot("map_whole", 1400, 1400);
            cam.orthographicSize = 260; cam.transform.position = new Vector3(-400, 600, 440); Shot("map_task_area", 1400, 1400);
            QualitySettings.shadowDistance = 250;
            cam.orthographic = false; cam.fieldOfView = 55; cam.ResetAspect();
            var hill = Targets[0].p; var hid = Targets[1].p;
            var views = new (string, Vector3, Vector3)[]
            {
                ("c2_station", OnGround(C2 + new Vector2(-8, -9), 2.6f), OnGround(Gate, 1f)),
                ("rover_eye_start", OnGround(RoverStart, 0.9f), OnGround(MissionStart + (MissionStart - Gate).normalized * 300, 6f)),
                ("start_towards_hill", OnGround(MissionStart, 1.6f), OnGround(hill, 2f)),
                ("astronaut", OnGround(Astronaut + new Vector2(3.5f, 3.5f), 1.3f), OnGround(Astronaut, 1.1f)),
                ("hammer", OnGround(Hammer + new Vector2(0.7f, -0.7f), 0.45f), OnGround(Hammer, 0.02f)),
                ("hill_from_southwest", OnGround(hill + new Vector2(-70, -55), 4f), OnGround(hill, 0f)),
                ("hill_from_northeast", OnGround(hill + new Vector2(60, 50), 3f), OnGround(hill, 0f)),
                ("hill_post", OnGround(hill + new Vector2(-4.5f, -3.5f), 1.2f), OnGround(hill, 0.9f)),
                ("hill_top_view_to_c2", OnGround(hill, 1.8f), OnGround(C2, 2f)),
                ("hidden_target", OnGround(hid + new Vector2(9, -7), 1.6f), OnGround(hid, 0.8f)),
                ("clay_ground_close", OnGround(hid + new Vector2(14, 6), 0.5f), OnGround(hid + new Vector2(22, 10), 0f)),
                ("shrubs_close", OnGround(MissionStart + new Vector2(28, -30), 0.8f), OnGround(MissionStart + new Vector2(40, -40), 0.2f)),
                ("shrubs_rover_camera_height", OnGround(Astronaut + new Vector2(-30, 25), 1.0f), OnGround(Astronaut + new Vector2(10, 0), 0.5f)),
                ("boulders_below_caprock", OnGround(new Vector2(545, 588), 1.6f), OnGround(new Vector2(575, 615), 1.5f)),
                ("talus_ground_close", OnGround(new Vector2(-235, 318), 0.6f), OnGround(new Vector2(-242, 312), 0.0f)),
                ("overview_from_west", new Vector3(-900, 260, 450), OnGround(new Vector2(-300, 400), 0)),
                ("overview_high", new Vector3(-1100, 520, -200), OnGround(new Vector2(0, 300), 0)),
                ("mesa_scarp_northeast", OnGround(new Vector2(420, 480), 3f), OnGround(new Vector2(700, 640), 20f)),
                ("brushy_basin_hills_southwest", OnGround(new Vector2(-640, -330), 3f), OnGround(new Vector2(-900, -520), 15f)),
                ("arroyo_south", OnGround(new Vector2(-150, -640), 3f), OnGround(new Vector2(60, -700), 0f)),
                ("horizon_west_factory_butte", OnGround(new Vector2(-560, 718), 3f), OnGround(new Vector2(-560, 718), 3f) + new Vector3(-1000, 60, 250)),
            };
            foreach (var (name, pos, look) in views)
            {
                cam.transform.position = pos; cam.transform.LookAt(look);
                Shot(name, 1600, 900);
            }
            QualitySettings.shadowDistance = shadow;
            Object.DestroyImmediate(cam.gameObject);
            Debug.Log("[UrcSite] screenshots in " + dir);
        }
    }
}
