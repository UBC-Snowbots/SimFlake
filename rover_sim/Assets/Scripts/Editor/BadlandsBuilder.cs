using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Generates the Badlands traversal world (Scenes/TaskWorlds/BadlandsTraversal): a desert / badlands mix in the
    /// spirit of URC (MDRS, Utah) and CIRC (Drumheller, Alberta). The map is 3 x 3 terrain tiles of 500 m (1.5 km
    /// square). Only the centre tile (-250..250 m) is detailed; the outer ring is coarse and rises into mountains at
    /// the map edge.
    ///
    /// Everything comes from the analytic height function H(x, z) and the layout constants below. Change them and
    /// re-run: the "Generated World" root and Assets/Worlds/Badlands are deleted and rebuilt. Anything else you add to
    /// the scene (robots etc.) is kept.
    ///   Menu: SimFlake > World Generation > Build Badlands Traversal World
    ///   Headless: Unity -batchmode -projectPath rover_sim -executeMethod SimFlake.EditorTools.BadlandsBuilder.BuildFromCommandLine [-screenshots dir]
    /// </summary>
    public static class BadlandsBuilder
    {
        const string ScenePath = "Assets/Scenes/TaskWorlds/BadlandsTraversal.unity";
        const string AssetDir = "Assets/Worlds/Badlands";
        const string TexDir = "Assets/Worlds/BadlandsTextures"; // tinted albedos from tools/make_badlands_textures.py
        const string RootName = "Generated World";
        const string ArucoPrefabPath = "Assets/Prefabs/Props/ArucoMarker_75cm.prefab";

        const float TileSize = 500f, BaseY = -20f, HeightRange = 150f;
        // Cliff faces run right to the edge of the 1.5 km map (the cliffs keep going up the boundary mountains).
        const float MapEdge = 1.5f * TileSize - 2f;
        // All tiles share one heightmap resolution: Unity only stitches neighbouring tiles cleanly when they match,
        // otherwise cracks open along the seams at x/z = +-250.
        const int CenterRes = 1025, OuterRes = 1025, CenterSplat = 1024, OuterSplat = 256, CenterDetail = 512;

        // ------------------------------------------------------------------ layout (world metres, +x east, +z north)

        static readonly Vector2 StationCenter = new Vector2(-20f, -205f);
        const float StationRadius = 22f;
        static Vector2 StartPos => StationCenter + new Vector2(0, 14f);

        // Escarpment: a ~12 m cliff band across the middle of the tile with three ways up.
        const float WestRampX = -165f, EastTalusX = 150f, SlotX = 15f, SlotHalf = 3.3f, SlotLen = 40f;
        // Horizontal width of the slot's side step. Wide enough for the 0.49 m heightmap grid to draw it without stair-steps.
        const float SlotSoft = 1.2f;
        // Lane through the slot kept clear of rocks bigger than pebbles (the rock faces leave ~3.8 m).
        const float SlotLane = 2.4f;
        // Ramps: breaches in the escarpment. Each is a notch cut into the plateau (rock walls on both sides, side step
        // RampCutSoft wide) with an earth/debris fan below it (side slopes RampFillSoft wide). w = horizontal length.
        static readonly (float x, float half, float w, bool rough)[] Ramps =
        {
            (WestRampX, 14f, 50f, false), // west: long, even ramp (~15 deg)
            (EastTalusX, 22f, 34f, true), // east: steeper rockfall slope (~22 deg) + bumps
        };
        const float CliffW = 1.6f, RampCutSoft = 1.2f, RampFillSoft = 10f;
        // the ledge band's ramp, built the same way (the ledge then dies out east of x = -45)
        static readonly (float x, float half, float w, bool rough) LedgeRamp = (-68f, 9f, 30f, false);
        // Dry wash with two graded fords.
        const float FordW = -110f, FordE = 95f;
        // Ledge band (3.5 m step) on the plateau, west half; it ramps out at its east end.
        const float TerraceStep = 3.5f;

        struct Butte { public Vector2 c; public float r, h, seed; }
        static readonly Butte[] Buttes =
        {
            new Butte { c = new Vector2(70, 150), r = 22, h = 16, seed = 1.3f },
            new Butte { c = new Vector2(-70, -40), r = 14, h = 18, seed = 4.1f },
            new Butte { c = new Vector2(-215, 120), r = 10, h = 11, seed = 7.7f },
        };
        static readonly Vector2 CraterC = new Vector2(-150, 210);
        const float CraterR = 22f, CraterDepth = 5f, CraterRim = 2.2f;

        // Zones (ellipses: centre, radii)
        static readonly Vector4 BadlandsZone = new Vector4(140, -55, 95, 50);
        static readonly Vector4 BoulderZone = new Vector4(-160, -55, 75, 45);
        static readonly Vector4 DuneZone = new Vector4(180, 165, 65, 80);

        struct Waypoint { public string name; public Vector2 p; public int aruco; public string note; }
        static readonly Waypoint[] Waypoints =
        {
            new Waypoint { name = "WP1", p = new Vector2(55, -80), aruco = -1, note = "GNSS only. Edge of the hoodoo field, across the dry wash." },
            new Waypoint { name = "WP2", p = new Vector2(-40, 105), aruco = 1, note = "Top of the escarpment. Slot canyon, west ramp or east talus." },
            new Waypoint { name = "WP3", p = CraterC, aruco = 2, note = "Crater floor, above the ledge band." },
            new Waypoint { name = "WP4", p = new Vector2(185, 205), aruco = 3, note = "Dune field behind the big butte." },
        };

        // Dirt track: the long, safe way from the station to the plateau (west ford, west ramp).
        static Vector2[] road;

        // ------------------------------------------------------------------ state

        static Terrain[,] tiles;
        static PhysicMaterial dirt;
        static float padHeight;
        static Transform root;
        static readonly Dictionary<string, Material> mats = new Dictionary<string, Material>();
        static List<(GameObject prefab, float size)> rockPrefabs;
        static readonly List<Vector3> placedRocks = new List<Vector3>(); // x, z, radius
        static readonly StringBuilder report = new StringBuilder();

        [MenuItem("SimFlake/World Generation/Build Badlands Traversal World", priority = 101)]
        public static void BuildMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            Build();
        }

        public static void BuildFromCommandLine()
        {
            Build();
            Verify();
            string shots = Arg("-screenshots");
            if (!string.IsNullOrEmpty(shots)) Screenshots(shots);
            EditorApplication.Exit(0);
        }

        static void Build()
        {
            var t0 = System.DateTime.Now;
            AssetDatabase.Refresh();
            var scene = File.Exists(ScenePath)
                ? EditorSceneManager.OpenScene(ScenePath)
                : EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            foreach (var old in scene.GetRootGameObjects().Where(g => g.name == RootName)) Object.DestroyImmediate(old);
            if (AssetDatabase.IsValidFolder(AssetDir)) AssetDatabase.DeleteAsset(AssetDir);
            foreach (var d in new[] { AssetDir, AssetDir + "/Meshes", AssetDir + "/Prefabs", AssetDir + "/Textures", AssetDir + "/Materials" })
                Directory.CreateDirectory(d);
            AssetDatabase.Refresh();
            mats.Clear(); placedRocks.Clear(); report.Clear();

            dirt = AssetDatabase.LoadAssetAtPath<PhysicMaterial>("Assets/Materials/CustomDirt.physicMaterial");
            Random.InitState(1337);
            padHeight = HRaw(StationCenter.x, StationCenter.y);
            road = new[]
            {
                StartPos, new Vector2(-45, -170), new Vector2(-88, -152), new Vector2(FordW, ArZ(FordW)),
                new Vector2(-114, -100), new Vector2(-106, -35), new Vector2(-140, 0),
                new Vector2(WestRampX, EscZ(WestRampX) - 30), new Vector2(WestRampX + 2, EscZ(WestRampX) + 30),
                new Vector2(-120, EscZ(-120) + 60),
            };

            root = new GameObject(RootName).transform;
            BuildTerrain();
            BuildCliffWalls();
            BuildHoodoos();
            BuildRocks();
            BuildControlStation();
            BuildWaypoints();

            SkyBuilder.Apply(SkyBuilder.Desert);
            var ls = new LightingSettings { name = "BadlandsLighting", bakedGI = false, realtimeGI = false };
            AssetDatabase.CreateAsset(ls, AssetDir + "/BadlandsLighting.lighting");
            Lightmapping.lightingSettings = ls;
            EnsureCamera();

            AssetDatabase.SaveAssets();
            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene, ScenePath);
            WriteWaypointFile();
            Debug.Log($"[Badlands] Done in {(System.DateTime.Now - t0).TotalSeconds:F0} s\n{report}");
        }

        // ------------------------------------------------------------------ noise helpers

        static float N(float x, float z) => Mathf.PerlinNoise(x + 317.3f, z + 911.7f) * 2f - 1f;

        static float Fbm(float x, float z, int oct)
        {
            float s = 0, a = 0.5f, f = 1, norm = 0;
            for (int i = 0; i < oct; i++) { s += a * N(x * f + i * 19.1f, z * f + i * 7.3f); norm += a; a *= 0.5f; f *= 2.03f; }
            return s / norm;
        }

        static float SmoothStep(float a, float b, float x) { float t = Mathf.Clamp01((x - a) / (b - a)); return t * t * (3 - 2 * t); }
        /// <summary>1 within 'half' of c, fading to 0 over 'soft'.</summary>
        static float Band(float v, float c, float half, float soft) => 1 - SmoothStep(half, half + soft, Mathf.Abs(v - c));
        static float Ellipse(float x, float z, Vector4 e) => Mathf.Sqrt(Sq((x - e.x) / e.z) + Sq((z - e.y) / e.w));
        static float Zone(float x, float z, Vector4 e) => 1 - SmoothStep(0.65f, 1f, Ellipse(x, z, e) + 0.15f * N(x / 30f, z / 30f));
        static float Sq(float v) => v * v;
        /// <summary>1 in the detailed centre tile, fading to 0 at its edges (so the coarse tiles match at the seams).</summary>
        static float DetailMask(float x, float z) => 1 - SmoothStep(225f, 246f, Mathf.Max(Mathf.Abs(x), Mathf.Abs(z)));

        // ------------------------------------------------------------------ height function

        static float EscZ(float x) => 55f + 18f * Mathf.Sin(x / 70f + 0.6f) + 10f * N(x / 35f, 3.1f) + 3f * N(x / 9f, 8.7f);
        static float EscStep(float x) => 12f + 2f * N(x / 90f, 5.5f);
        static float SlotCenter(float z) => SlotX + 1.8f * Mathf.Sin(z / 7f);
        static float SlotBand(float x, float z) => Band(x, SlotCenter(z), SlotHalf, SlotSoft);

        /// <summary>Signed horizontal distance from the escarpment edge (positive = up on the plateau).</summary>
        static float EscD(float x, float z)
        {
            float dz = EscZ(x + 0.5f) - EscZ(x - 0.5f);
            return (z - EscZ(x)) / Mathf.Sqrt(1 + dz * dz);
        }

        /// <summary>
        /// Blends a ramp into a cliff of height h. Where the ramp lies below the cliff surface it is a notch with sharp
        /// sides (covered by rock walls); where it lies above, it is a debris fan with wide side slopes. The two meet
        /// where the heights are equal, so there's no seam.
        /// </summary>
        static float WithRamp(float h, float x, float z, float d, float step, (float x, float half, float w, bool rough) r)
        {
            if (Mathf.Abs(x - r.x) > r.half + RampFillSoft) return h;
            float hr = step * Rise(d, r.w, out float t);
            if (r.rough) hr += 0.7f * Fbm(x / 2.5f, z / 2.5f, 2) * 4 * t * (1 - t); // rockfall bumps on the talus
            return Mathf.Lerp(h, hr, Band(x, r.x, r.half, hr < h ? RampCutSoft : RampFillSoft));
        }
        static float CutBand(float x, (float x, float half, float w, bool rough) r) => Band(x, r.x, r.half, RampCutSoft);

        /// <summary>True where a wall column sits on the side step of a ramp or the slot (narrower face than the cliff).</summary>
        static bool OnCutSide(Vector2 p)
        {
            if (SlotBand(p.x, p.y) > 0.01f) return true;
            float d = EscD(p.x, p.y), dt = TerD(p.x, p.y);
            return d > 0 && Ramps.Any(r => CutBand(p.x, r) > 0.01f) || dt > 0 && CutBand(p.x, LedgeRamp) > 0.01f;
        }

        /// <summary>Rise profile: 0 at the bottom, 1 at the top, with a talus apron at the foot.</summary>
        static float Rise(float d, float w, out float t)
        {
            t = Mathf.Clamp01((d + w / 2) / w);
            float sm = t * t * (3 - 2 * t);
            float p = Mathf.Lerp(sm, 0.3f * sm + 0.7f * t, SmoothStep(3f, 10f, w)); // ramps are closer to linear
            float apron = 0.1f * Mathf.Exp(-Mathf.Max(0, -d - w / 2) / 4f);
            return apron + (1 - apron) * p;
        }

        static float EscHeight(float x, float z)
        {
            float d = EscD(x, z), step = EscStep(x);
            float h = step * Rise(d, CliffW, out _);
            foreach (var r in Ramps) h = WithRamp(h, x, z, d, step, r);
            float slot = SlotBand(x, z);
            if (slot > 0)
            {
                // slot canyon: a ramp (~19 deg) cut down into the plateau, from the cliff foot to the top over SlotLen
                float floor = EscStep(x) * Rise(d - SlotLen * 0.5f, SlotLen, out _);
                h = Mathf.Lerp(h, Mathf.Min(h, floor), slot);
            }
            return h;
        }

        static float TerZ(float x) => 165f + 8f * Mathf.Sin(x / 40f) + 5f * N(x / 20f, 1.7f);

        /// <summary>Signed horizontal distance from the ledge band's edge (positive = up).</summary>
        static float TerD(float x, float z)
        {
            float dz = TerZ(x + 0.5f) - TerZ(x - 0.5f);
            return (z - TerZ(x)) / Mathf.Sqrt(1 + dz * dz);
        }

        static float TerraceHeight(float x, float z)
        {
            float fade = 1 - SmoothStep(-45f, -10f, x);
            if (fade <= 0) return 0;
            float d = TerD(x, z);
            float h = WithRamp(TerraceStep * Rise(d, CliffW, out _), x, z, d, TerraceStep, LedgeRamp);
            return fade * h;
        }

        static float ButteR(Butte b, float ang) =>
            b.r * (1 + 0.16f * N(Mathf.Cos(ang) * 1.3f + b.seed, Mathf.Sin(ang) * 1.3f)
                     + 0.05f * N(Mathf.Cos(ang) * 5f + b.seed * 2, Mathf.Sin(ang) * 5f));

        static float ButteHeight(float x, float z)
        {
            float h = 0;
            foreach (var b in Buttes)
            {
                float dx = x - b.c.x, dz = z - b.c.y, r = Mathf.Sqrt(dx * dx + dz * dz);
                if (r > b.r * 1.5f + 40f) continue;
                float dd = ButteR(b, Mathf.Atan2(dz, dx)) - r; // positive inside
                float p = SmoothStep(-1f, 1f, dd);
                float apron = 0.15f * Mathf.Exp(-Mathf.Max(0, -dd - 1) / 6f);
                h += b.h * (apron + (1 - apron) * p) + p * 0.6f * N(x / 12f, z / 12f);
            }
            return h;
        }

        static float ArZ(float x) => -125f + 20f * Mathf.Sin(x / 55f) + 8f * Mathf.Sin(x / 23f + 1f);

        /// <summary>Dry wash channel: 1 on the bed, 0 outside. Banks are ~60 deg except at the two fords.</summary>
        static float Arroyo(float x, float z, out float dist, out float hw)
        {
            float s = ArZ(x + 0.5f) - ArZ(x - 0.5f);
            dist = Mathf.Abs(z - ArZ(x)) / Mathf.Sqrt(1 + s * s);
            hw = 6f + 2f * N(x / 30f, 4.4f);
            float soft = Mathf.Lerp(0.7f, 12f, Mathf.Max(Band(x, FordW, 6, 6), Band(x, FordE, 6, 6)));
            return 1 - SmoothStep(hw - soft, hw + soft, dist);
        }

        static float CraterHeight(float x, float z)
        {
            float r = Vector2.Distance(new Vector2(x, z), CraterC) / CraterR;
            if (r > 3) return 0;
            return r < 1 ? -CraterDepth + (CraterDepth + CraterRim) * r * r : CraterRim * Mathf.Exp(-Sq((r - 1) / 0.4f));
        }

        static float DuneHeight(float x, float z)
        {
            float m = Zone(x, z, DuneZone);
            if (m <= 0) return 0;
            // transverse ridges, warped and broken up into separate dunes
            float phase = (x * 0.8f + z * 0.6f) / 22f + 0.8f * N(x / 45f, z / 45f) + 0.3f * N(x / 15f, z / 15f);
            float f = phase - Mathf.Floor(phase);
            float prof = f < 0.72f ? SmoothStep(0, 0.72f, f) : 1 - SmoothStep(0.72f, 1f, f); // gentle stoss, steep lee
            float amp = (2.4f + 0.8f * N(x / 40f, z / 40f)) * SmoothStep(-0.35f, 0.35f, N(x / 38f + 4, z / 38f - 2));
            return m * (amp * prof + 0.6f * (N(x / 25f, z / 25f) + 1));
        }

        static float HRaw(float x, float z)
        {
            float dm = DetailMask(x, z);
            float h = 7f * Fbm(x / 220f, z / 220f, 4) + 1.5f * Fbm(x / 45f + 3, z / 45f + 7, 3) + 0.25f * dm * Fbm(x / 7f, z / 7f, 2);
            h += 0.012f * z;
            h += EscHeight(x, z) + TerraceHeight(x, z);
            h -= (2.2f + 0.4f * N(x / 50f, 2.2f)) * Arroyo(x, z, out _, out _);
            h += ButteHeight(x, z) * dm + CraterHeight(x, z) * dm + DuneHeight(x, z) * dm;
            float bad = Zone(x, z, BadlandsZone) * dm;
            if (bad > 0)
                h += bad * (3.2f * Mathf.Pow(1 - Mathf.Abs(N(x / 24f, z / 24f)), 3) + 0.5f * Mathf.Pow(1 - Mathf.Abs(N(x / 6f + 5, z / 6f)), 3) - 0.8f);
            float boulder = Zone(x, z, BoulderZone) * dm;
            if (boulder > 0) h += boulder * 0.5f * Fbm(x / 5f, z / 5f, 2);
            float edge = SmoothStep(560f, 745f, Mathf.Max(Mathf.Abs(x), Mathf.Abs(z)));
            if (edge > 0) h += edge * (60f + 40f * Fbm(x / 150f, z / 150f, 3));
            return h;
        }

        static float H(float x, float z)
        {
            float pad = 1 - SmoothStep(StationRadius, StationRadius + 12f, Vector2.Distance(new Vector2(x, z), StationCenter));
            float h = pad >= 1 ? padHeight : HRaw(x, z);
            return Mathf.Lerp(h, padHeight, pad);
        }

        static float RoadDist(float x, float z)
        {
            var p = new Vector2(x, z);
            float best = float.MaxValue;
            for (int i = 0; i < road.Length - 1; i++)
            {
                Vector2 a = road[i], ab = road[i + 1] - a;
                float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / ab.sqrMagnitude);
                best = Mathf.Min(best, Vector2.Distance(p, a + t * ab));
            }
            return best;
        }

        // ------------------------------------------------------------------ terrain

        enum L { DesertSoil, Pebbles, Sandstone, WashSand, DryGrass, GreyClay, RockySoil, Gravel }

        static TerrainLayer[] MakeLayers()
        {
            var spec = new (L layer, string normal, float tile)[]
            {
                (L.DesertSoil, "Pebbles_B", 5), (L.Pebbles, "Pebbles_A", 4), (L.Sandstone, "Rock", 8), (L.WashSand, "Sand", 5),
                (L.DryGrass, "Grass_Dry", 4), (L.GreyClay, "Rock", 6), (L.RockySoil, "Soil_Rocks", 5), (L.Gravel, "Pebbles_C", 3),
            };
            return spec.Select(s =>
            {
                var tl = new TerrainLayer
                {
                    name = s.layer.ToString(),
                    diffuseTexture = AssetDatabase.LoadAssetAtPath<Texture2D>($"{TexDir}/{s.layer}_Albedo.jpg"),
                    normalMapTexture = AssetDatabase.LoadAssetAtPath<Texture2D>($"Assets/ThirdParty/TerrainSampleAssets/Textures/Terrain/{s.normal}_Normal.tif"),
                    tileSize = new Vector2(s.tile, s.tile),
                    normalScale = 1, smoothness = 0.05f, metallic = 0,
                };
                if (!tl.diffuseTexture) Debug.LogError($"[Badlands] missing {TexDir}/{s.layer}_Albedo.jpg (run tools/make_badlands_textures.py)");
                AssetDatabase.CreateAsset(tl, $"{AssetDir}/Materials/{s.layer}.terrainlayer");
                return tl;
            }).ToArray();
        }

        static void BuildTerrain()
        {
            var layers = MakeLayers();
            var bushes = MakeBushPrefabs();
            var grasses = MakeGrassPrefabs();
            var group = new GameObject("Terrain").transform;
            group.SetParent(root, false);
            tiles = new Terrain[3, 3];

            for (int j = 0; j < 3; j++)
            for (int i = 0; i < 3; i++)
            {
                bool center = i == 1 && j == 1;
                int res = center ? CenterRes : OuterRes;
                var origin = new Vector3((i - 1) * TileSize - TileSize / 2, BaseY, (j - 1) * TileSize - TileSize / 2);
                var td = new TerrainData { heightmapResolution = res };
                td.size = new Vector3(TileSize, HeightRange, TileSize);

                var hm = new float[res, res];
                float step = TileSize / (res - 1);
                for (int zi = 0; zi < res; zi++)
                for (int xi = 0; xi < res; xi++)
                    hm[zi, xi] = Mathf.Clamp01((H(origin.x + xi * step, origin.z + zi * step) - BaseY) / HeightRange);
                td.SetHeights(0, 0, hm);

                td.alphamapResolution = center ? CenterSplat : OuterSplat;
                td.baseMapResolution = center ? 1024 : 256;
                td.terrainLayers = layers;
                td.SetAlphamaps(0, 0, Splat(td.alphamapResolution, origin, hm, res));

                td.treePrototypes = bushes.Select(b => new TreePrototype { prefab = b }).ToArray();
                if (center)
                {
                    td.SetDetailResolution(CenterDetail, 32);
                    td.SetDetailScatterMode(DetailScatterMode.InstanceCountMode);
                    td.detailPrototypes = grasses.Select(g => new DetailPrototype
                    {
                        prototype = g, usePrototypeMesh = true, useInstancing = true, renderMode = DetailRenderMode.VertexLit,
                        minWidth = 0.7f, maxWidth = 1.4f, minHeight = 0.6f, maxHeight = 1.3f, noiseSpread = 0.4f,
                        healthyColor = Color.white, dryColor = new Color(0.9f, 0.85f, 0.75f),
                    }).ToArray();
                    PaintGrass(td, origin);
                }

                string name = $"Terrain_{i}_{j}";
                AssetDatabase.CreateAsset(td, $"{AssetDir}/{name}.asset");
                var go = Terrain.CreateTerrainGameObject(td);
                go.name = center ? "Terrain (centre, detailed)" : $"Terrain ({i - 1},{j - 1}) coarse";
                go.transform.SetParent(group, false);
                go.transform.position = origin;
                var terrain = go.GetComponent<Terrain>();
                terrain.heightmapPixelError = center ? 2 : 5;
                terrain.basemapDistance = 300;
                terrain.treeDistance = 500;
                terrain.treeBillboardDistance = 500; // the shrub prefabs have no billboards
                terrain.detailObjectDistance = 90;
                terrain.drawInstanced = true;
                terrain.groupingID = 7;
                go.GetComponent<TerrainCollider>().sharedMaterial = dirt;
                GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic | StaticEditorFlags.OccluderStatic | StaticEditorFlags.OccludeeStatic);
                tiles[i, j] = terrain;
            }
            for (int j = 0; j < 3; j++)
            for (int i = 0; i < 3; i++)
                tiles[i, j].SetNeighbors(i > 0 ? tiles[i - 1, j] : null, j < 2 ? tiles[i, j + 1] : null,
                    i < 2 ? tiles[i + 1, j] : null, j > 0 ? tiles[i, j - 1] : null);

            // trees after all tiles exist (placement checks slopes on the final terrain)
            for (int j = 0; j < 3; j++)
            for (int i = 0; i < 3; i++)
                PlantBushes(tiles[i, j], i == 1 && j == 1);
        }

        static float[,,] Splat(int ares, Vector3 origin, float[,] hm, int hres)
        {
            int nl = System.Enum.GetValues(typeof(L)).Length;
            var a = new float[ares, ares, nl];
            var w = new float[nl];
            float hstep = TileSize / (hres - 1);
            for (int az = 0; az < ares; az++)
            for (int ax = 0; ax < ares; ax++)
            {
                float x = origin.x + (ax + 0.5f) / ares * TileSize, z = origin.z + (az + 0.5f) / ares * TileSize;
                // slope from the heightmap
                int xi = Mathf.Clamp(Mathf.RoundToInt((x - origin.x) / hstep), 1, hres - 2);
                int zi = Mathf.Clamp(Mathf.RoundToInt((z - origin.z) / hstep), 1, hres - 2);
                float gx = (hm[zi, xi + 1] - hm[zi, xi - 1]) * HeightRange / (2 * hstep);
                float gz = (hm[zi + 1, xi] - hm[zi - 1, xi]) * HeightRange / (2 * hstep);
                float slope = Mathf.Atan(Mathf.Sqrt(gx * gx + gz * gz)) * Mathf.Rad2Deg;

                float dm = DetailMask(x, z);
                float wash = Arroyo(x, z, out _, out _);
                float bad = Zone(x, z, BadlandsZone) * dm;
                float boulder = Zone(x, z, BoulderZone) * dm;
                float dune = Zone(x, z, DuneZone) * dm;
                float plateau = SmoothStep(-2f, 6f, EscD(x, z));
                float pad = 1 - SmoothStep(StationRadius - 4, StationRadius + 2, Vector2.Distance(new Vector2(x, z), StationCenter));
                float roadW = 1 - SmoothStep(1.2f, 2.4f, RoadDist(x, z));
                float crater = 1 - SmoothStep(0.9f, 1.4f, Vector2.Distance(new Vector2(x, z), CraterC) / CraterR);
                float escD = EscD(x, z);
                float talus = Band(x, EastTalusX, 22f, 10f) * Band(escD, 0, 20f, 6f);
                float cliffFoot = escD < 0 ? 1 - SmoothStep(0, 10f, -escD) : 0;

                System.Array.Clear(w, 0, nl);
                w[(int)L.DesertSoil] = 1f;
                w[(int)L.Pebbles] = 0.9f * SmoothStep(0.05f, 0.45f, N(x / 25f, z / 25f)) + 1.2f * boulder + 0.8f * crater
                                    + 1.5f * talus + 2f * wash * SmoothStep(0.1f, 0.5f, N(x / 8f, z / 8f));
                w[(int)L.RockySoil] = 0.8f * SmoothStep(0f, 0.4f, N(x / 40f + 9, z / 40f)) + 0.4f * plateau
                                      + 3f * talus + 1.5f * cliffFoot;
                w[(int)L.DryGrass] = 0.9f * SmoothStep(0.15f, 0.5f, N(x / 30f + 3, z / 30f - 5)) * (1 - bad) * (1 - dune)
                                     + 0.6f * SmoothStep(0.1f, 0.4f, wash) * (1 - SmoothStep(0.6f, 0.9f, wash));
                w[(int)L.WashSand] = 2.5f * SmoothStep(0.5f, 0.85f, wash) + 4f * dune;
                w[(int)L.GreyClay] = 3f * bad;
                w[(int)L.Gravel] = 5f * pad + 2.5f * roadW * (1 - wash);
                float rock = SmoothStep(30f, 45f, slope);
                for (int k = 0; k < nl; k++) w[k] *= 1 - rock;
                w[(int)L.Sandstone] = 6f * rock + 0.0001f;

                float sum = w.Sum();
                for (int k = 0; k < nl; k++) a[az, ax, k] = w[k] / sum;
            }
            return a;
        }

        static void PaintGrass(TerrainData td, Vector3 origin)
        {
            int n = td.detailResolution, layersN = td.detailPrototypes.Length;
            var maps = Enumerable.Range(0, layersN).Select(_ => new int[n, n]).ToArray();
            long total = 0;
            for (int zi = 0; zi < n; zi++)
            for (int xi = 0; xi < n; xi++)
            {
                float x = origin.x + (xi + 0.5f) / n * TileSize, z = origin.z + (zi + 0.5f) / n * TileSize;
                float steep = td.GetSteepness((xi + 0.5f) / n, (zi + 0.5f) / n);
                if (steep > 28) continue;
                float wash = Arroyo(x, z, out _, out _);
                float density = 2.2f * SmoothStep(0.05f, 0.5f, N(x / 30f + 3, z / 30f - 5)) + 0.35f;
                density *= 1 - 0.85f * Zone(x, z, BadlandsZone);
                density *= 1 - 0.9f * Zone(x, z, DuneZone);
                density *= 1 - SmoothStep(0.6f, 0.9f, wash);
                density *= SmoothStep(StationRadius - 2, StationRadius + 4, Vector2.Distance(new Vector2(x, z), StationCenter));
                density *= SmoothStep(1.5f, 3f, RoadDist(x, z));
                density *= DetailMask(x, z);
                float r = Random.value * density;
                int k = Random.Range(0, layersN);
                int count = Mathf.FloorToInt(r);
                if (count > 0) { maps[k][zi, xi] = count; total += count; }
            }
            for (int k = 0; k < layersN; k++) td.SetDetailLayer(0, 0, k, maps[k]);
            report.AppendLine($"grass clumps: {total}");
        }

        static void PlantBushes(Terrain t, bool center)
        {
            var td = t.terrainData;
            var origin = t.transform.position;
            var list = new List<TreeInstance>();
            int attempts = center ? 60000 : 6000;
            // prototype weights: sage A, sage B, dry A, dry B, heather
            float[] weight = { 0.3f, 0.25f, 0.18f, 0.12f, 0.15f };
            for (int k = 0; k < attempts; k++)
            {
                float u = Random.value, v = Random.value;
                float x = origin.x + u * TileSize, z = origin.z + v * TileSize;
                if (td.GetSteepness(u, v) > 24) continue;
                float wash = Arroyo(x, z, out float dist, out float hw);
                float p = 0.07f + 0.05f * N(x / 50f, z / 50f);
                p += 0.25f * Band(dist, hw + 3f, 3f, 3f) * (1 - SmoothStep(0.3f, 0.6f, wash)); // shrubs line the wash banks
                p *= 1 - wash;
                p *= 1 - 0.5f * SmoothStep(-2f, 6f, EscD(x, z)); // sparser on the plateau
                p *= 1 - 0.85f * Zone(x, z, BadlandsZone);
                p *= 1 - 0.9f * Zone(x, z, DuneZone);
                p *= SmoothStep(StationRadius, StationRadius + 6, Vector2.Distance(new Vector2(x, z), StationCenter));
                p *= SmoothStep(2.5f, 4f, RoadDist(x, z));
                p *= SlotBand(x, z) > 0.01f && EscD(x, z) > -2 && EscD(x, z) < SlotLen + 2 ? 0 : 1;
                if (Waypoints.Any(w => Vector2.Distance(w.p, new Vector2(x, z)) < 4)) p = 0;
                if (!center) p *= 0.5f * (1 - SmoothStep(560f, 700f, Mathf.Max(Mathf.Abs(x), Mathf.Abs(z))));
                if (Random.value > p) continue;

                float r = Random.value * weight.Sum();
                int proto = 0;
                while (proto < weight.Length - 1 && r > weight[proto]) { r -= weight[proto]; proto++; }
                float s = Random.Range(0.7f, 1.8f);
                list.Add(new TreeInstance
                {
                    position = new Vector3(u, 0, v), prototypeIndex = proto,
                    widthScale = s * Random.Range(0.85f, 1.15f), heightScale = s,
                    rotation = Random.Range(0, Mathf.PI * 2), color = Color.white, lightmapColor = Color.white,
                });
            }
            td.SetTreeInstances(list.ToArray(), true);
            report.AppendLine($"{t.name}: {list.Count} shrubs");
        }

        // ------------------------------------------------------------------ vegetation prefabs

        static Material FoliageMaterial(Material src, string name, Color tint, float saturation)
        {
            string key = "foliage_" + name;
            if (mats.TryGetValue(key, out var m)) return m;
            m = new Material(AssetDatabase.LoadAssetAtPath<Shader>("Assets/Shaders/Foliage.shader")) { name = name, color = tint };
            m.mainTexture = src.GetTexture("Base_Map");
            m.SetTexture("_BumpMap", src.GetTexture("Normal_Map"));
            m.SetFloat("_Saturation", saturation);
            m.enableInstancing = true;
            AssetDatabase.CreateAsset(m, $"{AssetDir}/Materials/{name}.mat");
            mats[key] = m;
            return m;
        }

        static GameObject FoliagePrefab(string src, string matName, Color tint, float sat, bool collider)
        {
            var srcPrefab = AssetDatabase.LoadAssetAtPath<GameObject>($"Assets/ThirdParty/TerrainSampleAssets/Prefabs/{src}.prefab");
            var mesh = srcPrefab.GetComponentInChildren<MeshFilter>().sharedMesh;
            var mat = FoliageMaterial(srcPrefab.GetComponentInChildren<MeshRenderer>().sharedMaterial, matName, tint, sat);
            var go = new GameObject(src);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterial = mat;
            if (collider)
            {
                // shrubs are solid enough to stop a rover and show up on lidar
                var b = mesh.bounds;
                var cap = go.AddComponent<CapsuleCollider>();
                cap.center = b.center;
                cap.radius = 0.4f * Mathf.Max(b.size.x, b.size.z);
                cap.height = Mathf.Max(b.size.y, 2 * cap.radius);
            }
            var prefab = PrefabUtility.SaveAsPrefabAsset(go, $"{AssetDir}/Prefabs/{src}.prefab");
            Object.DestroyImmediate(go);
            return prefab;
        }

        static GameObject[] MakeBushPrefabs()
        {
            var sage = new Color(1.12f, 1.1f, 1.0f);
            return new[]
            {
                FoliagePrefab("Bush_A", "Sagebrush", sage, 0.45f, true),
                FoliagePrefab("Bush_B", "Sagebrush", sage, 0.45f, true),
                FoliagePrefab("BushDry_A", "DryBush", Color.white, 0.85f, true),
                FoliagePrefab("BushDry_B", "DryBush", Color.white, 0.85f, true),
                FoliagePrefab("Heather_A", "Saltbush", new Color(1.05f, 1.0f, 0.95f), 0.55f, true),
            };
        }

        static GameObject[] MakeGrassPrefabs() => new[]
        {
            FoliagePrefab("GrassDry_A", "DryGrass", new Color(1.05f, 1f, 0.9f), 0.9f, false),
            FoliagePrefab("GrassDry_B", "DryGrass", new Color(1.05f, 1f, 0.9f), 0.9f, false),
            FoliagePrefab("GrassDry_C", "DryGrass", new Color(1.05f, 1f, 0.9f), 0.9f, false),
        };

        // ------------------------------------------------------------------ cliff walls

        /// <summary>Terrain height at world (x, z), from whichever tile holds it.</summary>
        static float Ground(float x, float z)
        {
            var t = TileAt(x, z);
            return t.SampleHeight(new Vector3(x, 0, z)) + t.transform.position.y;
        }
        static float Ground(Vector2 p) => Ground(p.x, p.y);

        static float Steepness(float x, float z)
        {
            var t = TileAt(x, z);
            var o = t.transform.position;
            return t.terrainData.GetSteepness((x - o.x) / TileSize, (z - o.z) / TileSize);
        }

        static Terrain TileAt(float x, float z) =>
            tiles[Mathf.Clamp(Mathf.FloorToInt((x + 1.5f * TileSize) / TileSize), 0, 2), Mathf.Clamp(Mathf.FloorToInt((z + 1.5f * TileSize) / TileSize), 0, 2)];

        static void BuildCliffWalls()
        {
            var group = new GameObject("Cliff Walls").transform;
            group.SetParent(root, false);
            // mostly red Entrada-style sandstone, the odd pale or grey bed
            Color red = new Color(0.70f, 0.42f, 0.28f), orange = new Color(0.74f, 0.49f, 0.33f), maroon = new Color(0.64f, 0.37f, 0.26f);
            var sandstone = StrataMaterial("Sandstone", new[]
            {
                red, red, red, orange, orange, maroon, maroon, new Color(0.82f, 0.66f, 0.50f), new Color(0.60f, 0.45f, 0.38f),
            }, 0.03f);

            // Escarpment and ledge band. Where a ramp or the slot canyon breaks a cliff, the line turns in, runs up one side
            // of the notch and back down the other, so one rock face wraps the corners instead of ending in cut-off slabs.
            var pts = new List<Vector2>(); var low = new List<Vector2>();
            void Corner(Vector2 p, Vector2 from, Vector2 to) // a fan of columns around a convex corner
            {
                for (int k = 0; k <= 6; k++) { pts.Add(p); low.Add((Vector2)Vector3.Slerp(from, to, k / 6f)); }
            }
            Vector2 SideLow(System.Func<float, float> c, float z, int side) // perpendicular to a notch wall, pointing into it
            {
                float dc = (c(z + 0.3f) - c(z - 0.3f)) / 0.6f;
                return -side * new Vector2(1, -dc).normalized;
            }
            // notches: (centre line x(z), wall offset from it, how far back into the plateau the walls run), west to east
            void CliffPath(System.Func<float, float> lineZ, float xStart, float xEnd, params (System.Func<float, float> c, float off, float len)[] notches)
            {
                pts.Clear(); low.Clear();
                Vector2 Low(float x) => new Vector2(lineZ(x + 0.5f) - lineZ(x - 0.5f), -1).normalized;
                void Run(float a, float b) { for (float x = a; x < b; x += 0.8f) { pts.Add(new Vector2(x, lineZ(x))); low.Add(Low(x)); } }
                float x0 = xStart;
                foreach (var (c, off, len) in notches)
                {
                    float cx = c(lineZ(c(0))), xw = cx - off, xe = cx + off, zTop = lineZ(cx) + len;
                    // corner pivots sit on the plateau corner: beside the notch's side step and 1.2 m behind the cliff line
                    // there (the line can run diagonally), so the fan columns measure the full cliff height
                    var wc = new Vector2(xw - 0.7f, lineZ(xw - 0.7f) + 1.2f); var ec = new Vector2(xe + 0.7f, lineZ(xe + 0.7f) + 1.2f);
                    Run(x0, xw - 0.8f);
                    Corner(wc, Low(xw), SideLow(c, wc.y, -1));
                    for (float z = wc.y + 0.6f; z <= zTop; z += 0.6f) { pts.Add(new Vector2(c(z) - off, z)); low.Add(SideLow(c, z, -1)); }
                    for (float z = zTop; z >= ec.y + 0.6f; z -= 0.6f) { pts.Add(new Vector2(c(z) + off, z)); low.Add(SideLow(c, z, 1)); }
                    Corner(ec, SideLow(c, ec.y, 1), Low(xe));
                    x0 = xe + 0.8f;
                }
                Run(x0, xEnd);
            }
            (System.Func<float, float>, float, float) RampNotch((float x, float half, float w, bool rough) r) =>
                (_ => r.x, r.half + RampCutSoft / 2, r.w / 2 + 6f);

            CliffPath(EscZ, -MapEdge, MapEdge, RampNotch(Ramps[0]), (SlotCenter, SlotHalf + SlotSoft / 2, SlotLen + 6f), RampNotch(Ramps[1]));
            // low minHeight: notch walls shrink to nothing at the back, and should tuck into the ground, not stop as slabs
            Wall(group, "Escarpment", pts, low, p => OnCutSide(p) ? SlotSoft : CliffW, false, sandstone, null, minHeight: 0.4f);

            // ledge band: dies out toward x = -10, its face shrinking with it
            CliffPath(TerZ, -MapEdge, -8f, RampNotch(LedgeRamp));
            Wall(group, "Ledge Band", pts, low, p => OnCutSide(p) ? SlotSoft : CliffW, false, sandstone, null, minHeight: 0.4f);

            // buttes
            foreach (var b in Buttes)
            {
                pts.Clear(); low.Clear();
                int n = Mathf.CeilToInt(2 * Mathf.PI * b.r / 0.8f);
                for (int k = 0; k < n; k++)
                {
                    float ang = k * 2 * Mathf.PI / n;
                    pts.Add(b.c + ButteR(b, ang) * new Vector2(Mathf.Cos(ang), Mathf.Sin(ang)));
                }
                for (int k = 0; k < n; k++)
                {
                    var tng = pts[(k + 1) % n] - pts[(k + n - 1) % n];
                    low.Add(new Vector2(tng.y, -tng.x).normalized); // outward for a counter-clockwise loop
                }
                Wall(group, $"Butte ({b.c.x:F0}, {b.c.y:F0})", pts, low, _ => 2f, true, sandstone, null);
            }
        }

        /// <summary>
        /// A rock face covering a steep terrain step (the heightmap alone smears texture down cliffs). pts run along the
        /// middle of the step, low points toward the bottom. The face sits 0.7-1.2 m in front of the terrain, follows it from
        /// below ground level up to the rim and tucks back into the ground on top. Columns where the step is lower than
        /// minHeight (or 'include' says no) are left out.
        /// </summary>
        static void Wall(Transform parent, string name, List<Vector2> pts, List<Vector2> low, System.Func<Vector2, float> faceWidth, bool closed,
            Material mat, System.Func<Vector2, bool> include, float minHeight = 1.2f)
        {
            const int Rows = 12, R = Rows + 2;
            int n = pts.Count;
            var verts = new Vector3[n * R]; var uvs = new Vector2[n * R]; var valid = new bool[n]; var outDir = new Vector3[n];
            float along = 0;
            for (int i = 0; i < n; i++)
            {
                // measured a little out on the face, so corner fans (same point, turning normal) still advance the texture
                if (i > 0) along += Vector2.Distance(pts[i] + low[i].normalized * 1.2f, pts[i - 1] + low[i - 1].normalized * 1.2f);
                Vector2 p = pts[i], nn = low[i].normalized;
                outDir[i] = new Vector3(nn.x, 0, nn.y);
                float faceW = faceWidth(p);
                float buttress = 0.8f * (0.5f + 0.5f * N(along / 9f, 0.5f));
                // the top comes from behind the step, or from p itself for corner pivots (which sit on the top already)
                float lowH = Ground(p + nn * (faceW / 2 + 1.5f)), highH = Mathf.Max(Ground(p - nn * (faceW / 2 + 1.5f)), Ground(p));
                valid[i] = highH - lowH > minHeight && (include == null || include(p));
                for (int r = 0; r <= Rows; r++)
                {
                    float y = Mathf.Lerp(lowH - 0.6f, highH + 0.05f, r / (float)Rows);
                    float tf = Mathf.Clamp01((y - lowH) / Mathf.Max(highH - lowH, 0.1f));
                    float rough = 0.5f + 0.5f * Fbm(along / 2.3f, y / 1.4f, 2);
                    float ledges = 0.15f * Mathf.Sin(y * 3.1f + 2f * N(along / 6f, y / 3f));
                    float s = faceW / 2 - faceW * tf + 0.7f + 0.6f * rough + buttress + ledges;
                    var q = p + nn * s;
                    if (r == 0) y = Mathf.Min(y, Ground(q) - 0.8f); // always start below the ground right there
                    verts[i * R + r] = new Vector3(q.x, y, q.y);
                    uvs[i * R + r] = new Vector2(along / 4f, y / 16f);
                }
                var lip = p - nn * (faceW / 2 + 1.0f);
                verts[i * R + Rows + 1] = new Vector3(lip.x, Ground(lip) - 0.25f, lip.y);
                uvs[i * R + Rows + 1] = new Vector2(along / 4f, (highH + 1f) / 16f);
            }
            var tris = new List<int>();
            int cols = closed ? n : n - 1, kept = 0;
            for (int i = 0; i < cols; i++)
            {
                int j = (i + 1) % n;
                if (!valid[i] || !valid[j]) continue;
                kept++;
                for (int r = 0; r < R - 1; r++)
                {
                    int a = i * R + r, b = j * R + r, c = i * R + r + 1, d = j * R + r + 1;
                    var want = r == R - 2 ? Vector3.up : outDir[i];
                    bool flip = Vector3.Dot(Vector3.Cross(verts[b] - verts[a], verts[c] - verts[a]), want) < 0;
                    tris.AddRange(flip ? new[] { a, c, b, b, c, d } : new[] { a, b, c, b, d, c });
                }
            }
            if (kept == 0) { Debug.LogWarning($"[Badlands] wall {name}: nothing to cover"); return; }
            var mesh = new Mesh { name = name, indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            mesh.vertices = verts; mesh.uv = uvs; mesh.triangles = tris.ToArray();
            mesh.RecalculateNormals(); mesh.RecalculateTangents(); mesh.RecalculateBounds();
            AssetDatabase.CreateAsset(mesh, $"{AssetDir}/Meshes/{Safe(name)}.asset");
            var go = MeshObject(parent, name, mesh, mat);
            var mc = go.AddComponent<MeshCollider>();
            mc.sharedMesh = mesh; mc.sharedMaterial = dirt;
            report.AppendLine($"wall {name}: {kept} columns");
        }

        static GameObject MeshObject(Transform parent, string name, Mesh mesh, Material mat)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = mat;
            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic | StaticEditorFlags.OccluderStatic | StaticEditorFlags.OccludeeStatic);
            return go;
        }

        /// <summary>Layered-rock material: horizontal bands from the palette (one texture = 4 m wide x 16 m tall).</summary>
        static Material StrataMaterial(string name, Color[] palette, float coalChance)
        {
            const int W = 256, Hh = 1024;
            var bandColor = new Color[Hh]; var bandRecess = new float[Hh];
            int y0 = 0;
            while (y0 < Hh)
            {
                bool coal = Random.value < coalChance;
                int th = coal ? Random.Range(3, 8) : Random.Range(12, 80);
                if (Hh - (y0 + th) < 12) th = Hh - y0;
                var c = coal ? new Color(0.25f, 0.22f, 0.20f) : palette[Random.Range(0, palette.Length)] * Random.Range(0.95f, 1.05f);
                float rec = coal ? 1 : Random.value;
                for (int y = y0; y < y0 + th; y++) { bandColor[y] = c; bandRecess[y] = rec; }
                y0 += th;
            }
            // noise that tiles in both directions
            float TN(float px, float py, float sx, float sy)
            {
                float u = px / sx, v = py / sy, U = W / sx, V = Hh / sy, wx = px / W, wy = py / Hh;
                float a = Mathf.PerlinNoise(u + 50, v + 50), b = Mathf.PerlinNoise(u - U + 50, v + 50);
                float c = Mathf.PerlinNoise(u + 50, v - V + 50), d = Mathf.PerlinNoise(u - U + 50, v - V + 50);
                return Mathf.Lerp(Mathf.Lerp(a, b, wx), Mathf.Lerp(c, d, wx), wy);
            }
            var albedo = new Texture2D(W, Hh, TextureFormat.RGB24, true);
            var height = new float[W, Hh];
            for (int y = 0; y < Hh; y++)
            for (int x = 0; x < W; x++)
            {
                float big = TN(x, y, 40, 10), streak = TN(x, y, 6, 90), fine = Random.value;
                // blend the band edges over a few pixels, with a wavy boundary
                int yy = Mathf.Clamp(y + Mathf.RoundToInt((TN(x, y, 30, 30) - 0.5f) * 6), 0, Hh - 1);
                var c = bandColor[yy] * (0.86f + 0.26f * big) * (0.9f + 0.12f * streak) * (0.95f + 0.08f * fine);
                c.a = 1;
                albedo.SetPixel(x, y, c);
                height[x, y] = (1 - bandRecess[yy]) * 0.6f + 0.3f * big + 0.15f * streak + 0.06f * fine;
            }
            var normal = new Texture2D(W, Hh, TextureFormat.RGB24, true);
            for (int y = 0; y < Hh; y++)
            for (int x = 0; x < W; x++)
            {
                float dx = height[(x + 1) % W, y] - height[(x + W - 1) % W, y];
                float dy = height[x, (y + 1) % Hh] - height[x, (y + Hh - 1) % Hh];
                var nrm = new Vector3(-dx * 3f, -dy * 3f, 1).normalized;
                normal.SetPixel(x, y, new Color(nrm.x * 0.5f + 0.5f, nrm.y * 0.5f + 0.5f, nrm.z * 0.5f + 0.5f));
            }
            var alb = SaveTexture(albedo, $"{name}_Strata_Albedo", false);
            var nor = SaveTexture(normal, $"{name}_Strata_Normal", true);
            var m = NewMaterial(name + "Strata", Color.white, 0.06f);
            m.mainTexture = alb;
            m.SetTexture("_BumpMap", nor);
            m.EnableKeyword("_NORMALMAP");
            // real rock grain on top of the bands (detail tile = 2 m)
            m.SetTexture("_DetailAlbedoMap", AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/ThirdParty/TerrainSampleAssets/Textures/Terrain/Rock_BaseColor.tif"));
            m.SetTexture("_DetailNormalMap", AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/ThirdParty/TerrainSampleAssets/Textures/Terrain/Rock_Normal.tif"));
            m.SetTextureScale("_DetailAlbedoMap", new Vector2(2, 8));
            m.SetFloat("_DetailNormalMapScale", 1.2f);
            m.EnableKeyword("_DETAIL_MULX2");
            return m;
        }

        static Texture2D SaveTexture(Texture2D tex, string name, bool normalMap)
        {
            string path = $"{AssetDir}/Textures/{name}.png";
            File.WriteAllBytes(path, tex.EncodeToPNG());
            AssetDatabase.ImportAsset(path);
            var imp = (TextureImporter)AssetImporter.GetAtPath(path);
            imp.wrapMode = TextureWrapMode.Repeat;
            imp.anisoLevel = 4;
            if (normalMap) imp.textureType = TextureImporterType.NormalMap;
            imp.SaveAndReimport();
            return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        // ------------------------------------------------------------------ hoodoos

        static void BuildHoodoos()
        {
            var group = new GameObject("Hoodoos").transform;
            group.SetParent(root, false);
            // Drumheller-style grey/buff mudstone and sandstone with thin coal seams
            Color buff = new Color(0.70f, 0.65f, 0.58f), grey = new Color(0.64f, 0.59f, 0.53f), pale = new Color(0.76f, 0.70f, 0.62f);
            var clay = StrataMaterial("Hoodoo", new[]
            {
                buff, buff, buff, grey, grey, pale, pale, new Color(0.58f, 0.50f, 0.44f), new Color(0.66f, 0.55f, 0.44f),
            }, 0.06f);
            var capMat = AssetDatabase.LoadAssetAtPath<Material>("Assets/ThirdParty/HQ Rocks/Materials/Rock_3_v2.mat") ?? NewMaterial("Caprock", new Color(0.4f, 0.3f, 0.25f), 0.1f);

            // variants 0-1: capless eroded cones, 2-7: capped, 8-9: tall capped
            const int Variants = 10;
            var columns = new (Mesh col, Mesh cap, float h, float capY)[Variants];
            for (int v = 0; v < Variants; v++)
            {
                bool tall = v >= 8, capped = v >= 2;
                float h = tall ? Random.Range(8f, 10f) : Random.Range(3f, 6.5f), r0 = Random.Range(0.7f, 1.2f) * (tall ? 1.4f : 1);
                var col = HoodooColumn(v, h, r0, capped, out float topR, out Vector3 top);
                AssetDatabase.CreateAsset(col, $"{AssetDir}/Meshes/Hoodoo_{v}.asset");
                Mesh cap = null;
                if (capped)
                {
                    cap = HoodooCap(v, topR * Random.Range(1.5f, 2.1f), Random.Range(0.4f, 0.8f));
                    AssetDatabase.CreateAsset(cap, $"{AssetDir}/Meshes/HoodooCap_{v}.asset");
                }
                columns[v] = (col, cap, h, top.y - 0.15f);
                capOffsets[v] = top;
            }

            var placed = new List<Vector2>();
            int count = 0;
            for (int tries = 0; tries < 4000 && count < 42; tries++)
            {
                var p = new Vector2(Random.Range(BadlandsZone.x - BadlandsZone.z, BadlandsZone.x + BadlandsZone.z),
                    Random.Range(BadlandsZone.y - BadlandsZone.w, BadlandsZone.y + BadlandsZone.w));
                if (Zone(p.x, p.y, BadlandsZone) < 0.6f || Steepness(p.x, p.y) > 14 || Arroyo(p.x, p.y, out _, out _) > 0.01f) continue;
                if (placed.Any(q => Vector2.Distance(p, q) < 7f)) continue;
                if (Waypoints.Any(w => Vector2.Distance(w.p, p) < 8f)) continue;
                placed.Add(p);
                int v = count < 4 ? 8 + count % 2 : Random.Range(0, 8); // a few tall ones
                float s = Random.Range(0.85f, 1.2f);
                var go = MeshObject(group, $"Hoodoo_{count:00} ({columns[v].h * s:F1} m)", columns[v].col, clay);
                go.transform.position = new Vector3(p.x, Ground(p) - 0.2f, p.y);
                go.transform.rotation = Quaternion.Euler(0, Random.Range(0, 360f), 0);
                go.transform.localScale = Vector3.one * s;
                go.AddComponent<MeshCollider>().sharedMesh = columns[v].col;
                placedRocks.Add(new Vector3(p.x, p.y, 2f * s));
                count++;
                if (!columns[v].cap) continue;
                var cap = MeshObject(go.transform, "Caprock", columns[v].cap, capMat);
                cap.transform.localPosition = new Vector3(capOffsets[v].x, columns[v].capY, capOffsets[v].z);
                cap.transform.localRotation = Quaternion.Euler(Random.Range(-6f, 6f), Random.Range(0, 360f), Random.Range(-6f, 6f));
                var cc = cap.AddComponent<MeshCollider>();
                cc.sharedMesh = columns[v].cap; cc.convex = true;
            }
            report.AppendLine($"hoodoos: {count}");
        }

        static readonly Vector3[] capOffsets = new Vector3[16];

        /// <summary>
        /// Lathe column: flared base, tapering, lumpy and slightly leaning, with strata ribs. Capped ones keep a neck for
        /// the cap rock; capless ones erode to a blunt point. Origin at ground level; 'top' is the centre of the top.
        /// </summary>
        static Mesh HoodooColumn(int seed, float h, float r0, bool capped, out float topR, out Vector3 top)
        {
            const int Na = 24, Ny = 28;
            var verts = new List<Vector3>(); var uvs = new List<Vector2>(); var tris = new List<int>();
            float Profile(float t) => capped
                ? 1.55f - 0.95f * Mathf.Pow(t, 0.8f) + 0.12f * SmoothStep(0.9f, 1f, t)
                : 1.6f - 1.35f * Mathf.Pow(t, 0.9f);
            float leanAng = seed * 2.4f;
            var lean = new Vector3(Mathf.Cos(leanAng), 0, Mathf.Sin(leanAng)) * (0.03f + 0.02f * (seed % 3));
            topR = r0 * Profile(1);
            top = lean * h + Vector3.up * h;
            for (int k = 0; k < Ny; k++)
            {
                float y = -0.6f + k / (Ny - 1f) * (h + 0.6f), t = Mathf.Clamp01(y / h);
                float bulge = 1 + 0.14f * N(t * 3f + seed * 2.1f, seed * 1.7f);
                for (int a = 0; a <= Na; a++)
                {
                    float ang = a * 2 * Mathf.PI / Na;
                    float ca = Mathf.Cos(ang), sa = Mathf.Sin(ang);
                    float nz = 0.22f * N(ca * 1.2f + seed * 3.3f, sa * 1.2f + t * 2f) + 0.08f * N(ca * 3f + seed, sa * 3f + t * 6f);
                    float r = r0 * (y < 0 ? 1.7f : Profile(t)) * bulge * (1 + nz + 0.04f * Mathf.Sin(y * 7f + seed));
                    verts.Add(new Vector3(ca * r, y, sa * r) + lean * Mathf.Max(y, 0));
                    uvs.Add(new Vector2(a / (float)Na * 2 * Mathf.PI * r0 / 4f, y / 8f + seed * 0.13f));
                }
            }
            for (int k = 0; k < Ny - 1; k++)
            for (int a = 0; a < Na; a++)
            {
                int i0 = k * (Na + 1) + a, i1 = i0 + 1, i2 = i0 + Na + 1, i3 = i2 + 1;
                tris.AddRange(new[] { i0, i2, i1, i1, i2, i3 });
            }
            int topI = verts.Count;
            verts.Add(top + Vector3.up * (capped ? 0.02f : 0.25f)); uvs.Add(new Vector2(0, h / 8f));
            int ring = (Ny - 1) * (Na + 1);
            for (int a = 0; a < Na; a++) tris.AddRange(new[] { ring + a, topI, ring + a + 1 });
            var mesh = new Mesh { name = "Hoodoo_" + seed, vertices = verts.ToArray(), uv = uvs.ToArray() };
            mesh.triangles = FixWindingLathe(mesh.vertices, tris, lean);
            mesh.RecalculateNormals(); mesh.RecalculateTangents(); mesh.RecalculateBounds();
            return mesh;
        }

        /// <summary>Like FixWinding, but 'inside' is the leaning axis of the column at each triangle's height.</summary>
        static int[] FixWindingLathe(Vector3[] v, List<int> tris, Vector3 lean)
        {
            var t = tris.ToArray();
            for (int i = 0; i < t.Length; i += 3)
            {
                Vector3 a = v[t[i]], b = v[t[i + 1]], c = v[t[i + 2]];
                var center = (a + b + c) / 3;
                var axisPt = lean * Mathf.Max(center.y, 0) + Vector3.up * center.y;
                var n = Vector3.Cross(b - a, c - a);
                var want = center - axisPt;
                if (want.sqrMagnitude < 1e-4f || Mathf.Abs(n.normalized.y) > 0.9f) want = Vector3.up;
                if (Vector3.Dot(n, want) < 0) { int s = t[i + 1]; t[i + 1] = t[i + 2]; t[i + 2] = s; }
            }
            return t;
        }

        /// <summary>Flat irregular cap rock slab.</summary>
        static Mesh HoodooCap(int seed, float r, float thick)
        {
            const int Na = 16;
            var verts = new List<Vector3> { new Vector3(0, thick + 0.1f, 0), new Vector3(0, -0.05f, 0) };
            for (int a = 0; a < Na; a++)
            {
                float ang = a * 2 * Mathf.PI / Na;
                float rr = r * (1 + 0.25f * N(Mathf.Cos(ang) + seed * 5.1f, Mathf.Sin(ang)));
                verts.Add(new Vector3(Mathf.Cos(ang) * rr, thick * Random.Range(0.85f, 1.1f), Mathf.Sin(ang) * rr));
                verts.Add(new Vector3(Mathf.Cos(ang) * rr * 0.85f, 0, Mathf.Sin(ang) * rr * 0.85f));
            }
            var tris = new List<int>();
            for (int a = 0; a < Na; a++)
            {
                int t0 = 2 + 2 * a, b0 = t0 + 1, t1 = 2 + 2 * ((a + 1) % Na), b1 = t1 + 1;
                tris.AddRange(new[] { 0, t1, t0, 1, b0, b1, t0, t1, b0, b0, t1, b1 });
            }
            var v = verts.ToArray();
            var mesh = new Mesh { name = "HoodooCap_" + seed, vertices = v, uv = v.Select(p => new Vector2(p.x, p.z) / 2f).ToArray() };
            mesh.triangles = FixWinding(v, tris, new Vector3(0, thick / 2, 0));
            mesh.RecalculateNormals(); mesh.RecalculateTangents(); mesh.RecalculateBounds();
            return mesh;
        }

        /// <summary>Orients every triangle to face away from the given interior point (fine for star-shaped meshes).</summary>
        static int[] FixWinding(Vector3[] v, List<int> tris, Vector3 inside)
        {
            var t = tris.ToArray();
            for (int i = 0; i < t.Length; i += 3)
            {
                Vector3 a = v[t[i]], b = v[t[i + 1]], c = v[t[i + 2]];
                var center = (a + b + c) / 3;
                var axis = new Vector3(center.x - inside.x, 0, center.z - inside.z); // lathe: radial direction
                if (axis.sqrMagnitude < 1e-4f) axis = center - inside;
                // top/bottom fans: use the vertical direction
                if (Mathf.Abs(Vector3.Cross(b - a, c - a).normalized.y) > 0.7f) axis = new Vector3(0, center.y - inside.y, 0);
                if (Vector3.Dot(Vector3.Cross(b - a, c - a), axis) < 0) { int s = t[i + 1]; t[i + 1] = t[i + 2]; t[i + 2] = s; }
            }
            return t;
        }

        // ------------------------------------------------------------------ rocks

        static void MakeRockPrefabs()
        {
            rockPrefabs = new List<(GameObject, float)>();
            foreach (var guid in AssetDatabase.FindAssets("t:Prefab", new[] { "Assets/ThirdParty/HQ Rocks/Prefabs" }))
            {
                var src = AssetDatabase.LoadAssetAtPath<GameObject>(AssetDatabase.GUIDToAssetPath(guid));
                var inst = (GameObject)PrefabUtility.InstantiatePrefab(src);
                inst.transform.position = Vector3.zero;
                foreach (var f in inst.GetComponentsInChildren<MeshFilter>())
                    if (!f.GetComponent<Collider>())
                    {
                        var mc = f.gameObject.AddComponent<MeshCollider>();
                        mc.sharedMesh = f.sharedMesh; mc.convex = true; mc.sharedMaterial = dirt;
                    }
                var b = inst.GetComponentInChildren<Renderer>().bounds;
                float size = Mathf.Max(b.size.x, b.size.y, b.size.z);
                var prefab = PrefabUtility.SaveAsPrefabAsset(inst, $"{AssetDir}/Prefabs/{src.name}.prefab");
                Object.DestroyImmediate(inst);
                rockPrefabs.Add((prefab, size));
            }
        }

        static bool Reserved(Vector2 p, float margin)
        {
            if (Vector2.Distance(p, StationCenter) < StationRadius + 4 + margin) return true;
            if (RoadDist(p.x, p.y) < 2.2f + margin) return true;
            if (Waypoints.Any(w => Vector2.Distance(w.p, p) < 3f + margin)) return true;
            return false;
        }

        static int PlaceRock(Transform parent, Vector2 p, float size, float sink = 0.3f, bool allowSteep = false, float maxSteep = 32f)
        {
            if (!allowSteep && Steepness(p.x, p.y) > maxSteep) return 0;
            float rad = size * 0.5f;
            foreach (var q in placedRocks)
                if (Sq(p.x - q.x) + Sq(p.y - q.y) < Sq((rad + q.z) * 0.9f)) return 0;
            placedRocks.Add(new Vector3(p.x, p.y, rad));
            var (prefab, meshSize) = rockPrefabs[Random.Range(0, rockPrefabs.Count)];
            var rock = (GameObject)PrefabUtility.InstantiatePrefab(prefab, parent);
            rock.name = $"rock_{size:F2}m";
            var sc = size / Mathf.Max(meshSize, 1e-3f);
            rock.transform.localScale = new Vector3(sc * Random.Range(0.85f, 1.15f), sc * Random.Range(0.6f, 1.0f), sc * Random.Range(0.85f, 1.15f));
            rock.transform.rotation = Quaternion.Euler(Random.Range(-12f, 12f), Random.Range(0, 360f), Random.Range(-12f, 12f));
            float g = Ground(p);
            rock.transform.position = new Vector3(p.x, g, p.y);
            var b = rock.GetComponentInChildren<Renderer>().bounds;
            // sit on the lowest ground under the rock so it never floats on slopes
            float gmin = Mathf.Min(g, Ground(p.x + rad, p.y), Ground(p.x - rad, p.y), Ground(p.x, p.y + rad), Ground(p.x, p.y - rad));
            rock.transform.position += Vector3.up * (gmin - b.min.y - sink * b.size.y);
            return 1;
        }

        /// <summary>Power-law size between min and max: mostly small, a few big.</summary>
        static float RockSize(float min, float max, float k = 2.2f) => min + (max - min) * Mathf.Pow(Random.value, k);

        static void BuildRocks()
        {
            MakeRockPrefabs();
            var group = new GameObject("Rocks").transform;
            group.SetParent(root, false);
            Transform Sub(string n) { var t = new GameObject(n).transform; t.SetParent(group, false); return t; }
            int c;

            // boulder field: the dense one, big stuff a rover has to weave through
            var bf = Sub("Boulder Field");
            c = 0;
            for (int k = 0; k < 6000 && c < 420; k++)
            {
                var p = new Vector2(Random.Range(BoulderZone.x - BoulderZone.z, BoulderZone.x + BoulderZone.z),
                    Random.Range(BoulderZone.y - BoulderZone.w, BoulderZone.y + BoulderZone.w));
                float z = Zone(p.x, p.y, BoulderZone);
                if (Random.value > z || Reserved(p, 0.5f)) continue;
                c += PlaceRock(bf, p, RockSize(0.25f, 2.4f, 2.0f));
            }
            report.AppendLine($"boulder field rocks: {c}");

            // rockfall along the foot of every cliff, heavy on the east talus slope
            var talus = Sub("Talus");
            c = 0;
            for (int k = 0; k < 30000 && c < 800; k++)
            {
                float x = Random.Range(-240f, 240f);
                float onTalus = Band(x, EastTalusX, 22f, 6f);
                float d = onTalus > 0.5f ? Random.Range(-22f, 18f) : -Random.Range(1.5f, 10f) * Random.Range(0.3f, 1f);
                float z = EscZ(x) + d;
                var p = new Vector2(x, z);
                if (SlotBand(x, z) > 0.01f && d > -3 || Reserved(p, 0.5f)) continue;
                if (d > -1f && Mathf.Abs(x - EastTalusX) > Ramps[1].half - 2.5f && onTalus > 0.5f) continue; // clear of the notch walls
                if (onTalus < 0.5f && Random.value > 0.3f) continue;
                c += PlaceRock(talus, p, onTalus > 0.5f ? RockSize(0.3f, 1.9f) : RockSize(0.3f, 2.6f, 1.6f), allowSteep: onTalus > 0.5f, maxSteep: 60);
            }
            foreach (var b in Buttes)
                for (int k = 0; k < 60; k++)
                {
                    float ang = Random.Range(0, 2 * Mathf.PI);
                    var p = b.c + (ButteR(b, ang) + Random.Range(1.5f, 9f)) * new Vector2(Mathf.Cos(ang), Mathf.Sin(ang));
                    if (!Reserved(p, 0.5f)) c += PlaceRock(talus, p, RockSize(0.3f, 2.2f, 1.8f), maxSteep: 40);
                }
            report.AppendLine($"talus rocks: {c}");

            // slot canyon: rubble heaped against both walls, pebbles on the floor, a debris fan and big blocks at the mouth,
            // loose blocks on the rim. A winding lane (SlotLane wide) stays clear of anything bigger than a pebble.
            var slot = Sub("Slot Canyon");
            c = 0;
            Physics.SyncTransforms();
            float zs = EscZ(SlotX), off = SlotHalf + SlotSoft / 2;
            float LaneC(float z) => SlotCenter(z) + 0.5f * Mathf.Sin(z / 9f);
            for (int k = 0; k < 3000 && c < 90; k++)
            {
                float z = zs + Random.Range(0.5f, SlotLen - 2f);
                int side = Random.value < 0.5f ? -1 : 1;
                float lc = LaneC(z), y = Ground(lc, z) + 0.35f;
                // distance from the lane centre to this wall's rock face
                if (!Physics.Raycast(new Vector3(lc, y, z), new Vector3(side, 0, 0), out var hit, 8f)) continue;
                float u = Random.Range(0.2f, 0.7f); // how far the rock tucks into the face
                float rad = Mathf.Min(RockSize(0.2f, 1.1f, 1.6f) / 2, (hit.distance - SlotLane / 2) / (1 + u));
                if (rad < 0.1f) continue;
                c += PlaceRock(slot, new Vector2(lc + side * (hit.distance - rad * u), z), rad * 2, allowSteep: true);
            }
            for (int k = 0, n = 0; k < 400 && n < 120; k++) // pebbles the rover can drive over
            {
                float z = zs + Random.Range(-2f, SlotLen - 2f);
                int ok = PlaceRock(slot, new Vector2(SlotCenter(z) + Random.Range(-1.8f, 1.8f), z), RockSize(0.08f, 0.2f), allowSteep: true);
                n += ok; c += ok;
            }
            for (int k = 0, n = 0; k < 2000 && n < 150; k++) // debris fan spreading out of the mouth
            {
                float back = Random.Range(0.5f, 18f) * Random.Range(0.4f, 1f), z = zs - back;
                float x = SlotX + Random.Range(-1f, 1f) * (off + 0.7f * back);
                float size = RockSize(0.12f, 1.1f, 2.4f);
                if (size > 0.3f && Mathf.Abs(x - SlotCenter(zs)) - size / 2 < SlotLane / 2) continue;
                if (Reserved(new Vector2(x, z), 0.3f)) continue;
                int ok = PlaceRock(slot, new Vector2(x, z), size, maxSteep: 40);
                n += ok; c += ok;
            }
            foreach (int side in new[] { -1, 1 }) // big fallen blocks flanking the entrance
                for (int k = 0, n = 0; k < 40 && n < 3; k++)
                {
                    float x = SlotCenter(zs) + side * (off + Random.Range(1.5f, 6f));
                    int ok = PlaceRock(slot, new Vector2(x, EscZ(x) - Random.Range(2.5f, 7f)), RockSize(1.2f, 2.3f, 1f), maxSteep: 40);
                    n += ok; c += ok;
                }
            for (int k = 0, n = 0; k < 800 && n < 40; k++) // loose blocks on the plateau along the rim
            {
                float z = zs + Random.Range(1f, SlotLen + 4f);
                int side = Random.value < 0.5f ? -1 : 1;
                float size = RockSize(0.25f, 1.4f, 1.8f);
                var p = new Vector2(SlotCenter(z) + side * (off + 2.2f + size / 2 + Random.Range(0f, 5f)), z);
                if (EscD(p.x, p.y) < 3f + size / 2) continue; // stay back from the main cliff edge
                int ok = PlaceRock(slot, p, size, maxSteep: 25);
                n += ok; c += ok;
            }
            report.AppendLine($"slot canyon rocks: {c}");

            // crater ejecta
            var cr = Sub("Crater Ejecta");
            c = 0;
            for (int k = 0; k < 600 && c < 80; k++)
            {
                float ang = Random.Range(0, 2 * Mathf.PI), r = CraterR * Random.Range(0.5f, 2.2f);
                var p = CraterC + r * new Vector2(Mathf.Cos(ang), Mathf.Sin(ang));
                if (!Reserved(p, 0.5f)) c += PlaceRock(cr, p, RockSize(0.2f, 1.4f, 2.4f), maxSteep: 35);
            }
            report.AppendLine($"crater rocks: {c}");

            // general scatter: lots of small rocks everywhere, a few big ones
            var sc = Sub("Scatter");
            c = 0;
            for (int k = 0; k < 9000 && c < 900; k++)
            {
                var p = new Vector2(Random.Range(-240f, 240f), Random.Range(-240f, 240f));
                if (Reserved(p, 0.3f) || Arroyo(p.x, p.y, out _, out _) > 0.3f && Random.value < 0.7f) continue;
                if (Zone(p.x, p.y, DuneZone) > 0.3f && Random.value < 0.9f) continue;
                float big = Random.value < 0.08f ? 1 : 0;
                c += PlaceRock(sc, p, big > 0 ? RockSize(0.6f, 1.3f) : RockSize(0.12f, 0.5f));
            }
            report.AppendLine($"scattered rocks: {c}");
        }

        // ------------------------------------------------------------------ control station

        static Material M(string name, Color c, float smooth = 0.2f, Color? emission = null)
        {
            if (mats.TryGetValue(name, out var m)) return m;
            m = NewMaterial(name, c, smooth);
            if (emission.HasValue)
            {
                m.EnableKeyword("_EMISSION");
                m.SetColor("_EmissionColor", emission.Value);
                m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.None;
            }
            mats[name] = m;
            return m;
        }

        static Material NewMaterial(string name, Color color, float smooth)
        {
            var m = new Material(Shader.Find("Standard")) { name = name, color = color };
            m.SetFloat("_Glossiness", smooth);
            AssetDatabase.CreateAsset(m, $"{AssetDir}/Materials/{name}.mat");
            return m;
        }

        static GameObject Prim(PrimitiveType type, Transform parent, string name, Vector3 pos, Vector3 scale, Material mat, Vector3 euler = default, bool collider = true)
        {
            var go = GameObject.CreatePrimitive(type);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = pos;
            go.transform.localRotation = Quaternion.Euler(euler);
            go.transform.localScale = scale;
            go.GetComponent<Renderer>().sharedMaterial = mat;
            if (!collider) Object.DestroyImmediate(go.GetComponent<Collider>());
            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic);
            return go;
        }

        static GameObject Box(Transform p, string n, Vector3 pos, Vector3 size, Material m, float yaw = 0, bool col = true) =>
            Prim(PrimitiveType.Cube, p, n, pos, size, m, new Vector3(0, yaw, 0), col);

        /// <summary>Cylinder between two local points.</summary>
        static GameObject Rod(Transform p, string n, Vector3 a, Vector3 b, float radius, Material m)
        {
            var go = Prim(PrimitiveType.Cylinder, p, n, (a + b) / 2, new Vector3(2 * radius, Vector3.Distance(a, b) / 2, 2 * radius), m);
            go.transform.localRotation = Quaternion.FromToRotation(Vector3.up, b - a);
            return go;
        }

        static Transform Group(Transform parent, string name, Vector3 localPos, float yaw = 0)
        {
            var t = new GameObject(name).transform;
            t.SetParent(parent, false);
            t.localPosition = localPos;
            t.localRotation = Quaternion.Euler(0, yaw, 0);
            return t;
        }

        static Mesh pyramid, cone;

        static Mesh Pyramid()
        {
            if (pyramid) return pyramid;
            var v = new[] { new Vector3(-0.5f, 0, -0.5f), new Vector3(0.5f, 0, -0.5f), new Vector3(0.5f, 0, 0.5f), new Vector3(-0.5f, 0, 0.5f), new Vector3(0, 1, 0) };
            var verts = new List<Vector3>(); var tris = new List<int>();
            for (int k = 0; k < 4; k++) { int i = verts.Count; verts.AddRange(new[] { v[k], v[(k + 1) % 4], v[4] }); tris.AddRange(new[] { i, i + 2, i + 1 }); }
            int b = verts.Count; verts.AddRange(v.Take(4)); tris.AddRange(new[] { b, b + 1, b + 2, b, b + 2, b + 3 });
            pyramid = new Mesh { name = "CanopyRoof", vertices = verts.ToArray() };
            pyramid.triangles = FixWinding(pyramid.vertices, tris, new Vector3(0, 0.3f, 0));
            pyramid.RecalculateNormals();
            AssetDatabase.CreateAsset(pyramid, $"{AssetDir}/Meshes/CanopyRoof.asset");
            return pyramid;
        }

        static Mesh Cone()
        {
            if (cone) return cone;
            const int n = 16;
            var verts = new List<Vector3> { new Vector3(0, 1, 0), Vector3.zero }; var tris = new List<int>();
            for (int k = 0; k < n; k++) { float a = k * 2 * Mathf.PI / n; verts.Add(new Vector3(Mathf.Cos(a) * 0.5f, 0, Mathf.Sin(a) * 0.5f)); }
            for (int k = 0; k < n; k++) { int a = 2 + k, b = 2 + (k + 1) % n; tris.AddRange(new[] { 0, b, a, 1, a, b }); }
            cone = new Mesh { name = "Cone", vertices = verts.ToArray() };
            cone.triangles = FixWinding(cone.vertices, tris, new Vector3(0, 0.3f, 0));
            cone.RecalculateNormals();
            AssetDatabase.CreateAsset(cone, $"{AssetDir}/Meshes/Cone.asset");
            return cone;
        }

        static void Canopy(Transform parent, Vector3 pos, Material fabric)
        {
            var t = Group(parent, "Canopy Tent 3x3", pos);
            var metal = M("Aluminium", new Color(0.75f, 0.75f, 0.78f), 0.6f);
            foreach (var (x, z) in new[] { (-1.45f, -1.45f), (1.45f, -1.45f), (1.45f, 1.45f), (-1.45f, 1.45f) })
                Rod(t, "Leg", new Vector3(x, 0, z), new Vector3(x, 2.3f, z), 0.03f, metal);
            var roof = MeshObject(t, "Roof", Pyramid(), fabric);
            roof.transform.localPosition = new Vector3(0, 2.3f, 0);
            roof.transform.localScale = new Vector3(3.1f, 0.65f, 3.1f);
            for (int k = 0; k < 4; k++)
            {
                var side = Group(t, "Valance", Vector3.zero, k * 90);
                Box(side, "Valance", new Vector3(0, 2.18f, 1.55f), new Vector3(3.1f, 0.25f, 0.01f), fabric, col: false);
            }
        }

        static void Table(Transform parent, Vector3 pos, float yaw)
        {
            var t = Group(parent, "Folding Table", pos, yaw);
            var top = M("TableTop", new Color(0.92f, 0.92f, 0.9f), 0.3f);
            var leg = M("DarkMetal", new Color(0.15f, 0.15f, 0.16f), 0.4f);
            Box(t, "Top", new Vector3(0, 0.72f, 0), new Vector3(1.8f, 0.04f, 0.75f), top);
            foreach (var (x, z) in new[] { (-0.8f, -0.3f), (0.8f, -0.3f), (0.8f, 0.3f), (-0.8f, 0.3f) })
                Rod(t, "Leg", new Vector3(x, 0, z), new Vector3(x, 0.7f, z), 0.015f, leg);
        }

        static void Laptop(Transform parent, Vector3 pos, float yaw)
        {
            var t = Group(parent, "Laptop", pos, yaw);
            var body = M("LaptopGrey", new Color(0.2f, 0.2f, 0.22f), 0.5f);
            var screen = M("Screen", new Color(0.05f, 0.08f, 0.12f), 0.9f, new Color(0.25f, 0.45f, 0.7f));
            Box(t, "Base", new Vector3(0, 0.01f, 0), new Vector3(0.34f, 0.02f, 0.24f), body, col: false);
            var lid = Group(t, "Lid", new Vector3(0, 0.02f, 0.12f));
            lid.localRotation = Quaternion.Euler(-15, 0, 0);
            Box(lid, "Lid", new Vector3(0, 0.12f, 0), new Vector3(0.34f, 0.24f, 0.01f), body, col: false);
            Box(lid, "Display", new Vector3(0, 0.12f, -0.006f), new Vector3(0.31f, 0.2f, 0.002f), screen, col: false);
        }

        static void Chair(Transform parent, Vector3 pos, float yaw)
        {
            var t = Group(parent, "Camp Chair", pos, yaw);
            var fabric = M("ChairFabric", new Color(0.1f, 0.25f, 0.45f), 0.1f);
            var leg = M("DarkMetal", new Color(0.15f, 0.15f, 0.16f), 0.4f);
            Box(t, "Seat", new Vector3(0, 0.45f, 0), new Vector3(0.5f, 0.03f, 0.48f), fabric);
            Box(t, "Back", new Vector3(0, 0.75f, -0.24f), new Vector3(0.5f, 0.55f, 0.03f), fabric, col: false);
            Rod(t, "Leg", new Vector3(-0.25f, 0, -0.25f), new Vector3(0.25f, 0.45f, 0.24f), 0.012f, leg);
            Rod(t, "Leg", new Vector3(0.25f, 0, -0.25f), new Vector3(-0.25f, 0.45f, 0.24f), 0.012f, leg);
        }

        static void Truck(Transform parent, Vector3 pos, float yaw)
        {
            var t = Group(parent, "Pickup Truck", pos, yaw);
            var paint = M("TruckWhite", new Color(0.9f, 0.9f, 0.88f), 0.7f);
            var glass = M("Glass", new Color(0.05f, 0.07f, 0.09f), 0.95f);
            var tire = M("Tire", new Color(0.06f, 0.06f, 0.06f), 0.1f);
            var trim = M("DarkMetal", new Color(0.15f, 0.15f, 0.16f), 0.4f);
            Box(t, "Body", new Vector3(0, 0.85f, 0), new Vector3(1.95f, 0.65f, 5.6f), paint);
            Box(t, "Cab", new Vector3(0, 1.6f, 0.75f), new Vector3(1.85f, 0.9f, 2.4f), paint);
            Box(t, "Windshield", new Vector3(0, 1.65f, 1.96f), new Vector3(1.7f, 0.7f, 0.02f), glass, col: false);
            Box(t, "Side windows", new Vector3(0, 1.7f, 0.75f), new Vector3(1.87f, 0.55f, 2.1f), glass, col: false);
            Box(t, "Bed floor", new Vector3(0, 1.0f, -1.7f), new Vector3(1.8f, 0.05f, 2.1f), trim, col: false);
            Box(t, "Bumper", new Vector3(0, 0.55f, 2.82f), new Vector3(1.98f, 0.22f, 0.1f), trim);
            foreach (var (x, z) in new[] { (-0.95f, 1.8f), (0.95f, 1.8f), (-0.95f, -1.75f), (0.95f, -1.75f) })
                Prim(PrimitiveType.Cylinder, t, "Wheel", new Vector3(x, 0.4f, z), new Vector3(0.8f, 0.14f, 0.8f), tire, new Vector3(0, 0, 90));
        }

        static void Mast(Transform parent, Vector3 pos)
        {
            var t = Group(parent, "Antenna Mast", pos);
            var metal = M("Aluminium", new Color(0.75f, 0.75f, 0.78f), 0.6f);
            var white = M("AntennaWhite", new Color(0.95f, 0.95f, 0.95f), 0.3f);
            var dark = M("DarkMetal", new Color(0.15f, 0.15f, 0.16f), 0.4f);
            for (int k = 0; k < 3; k++)
            {
                float a = k * 120 * Mathf.Deg2Rad;
                Rod(t, "Tripod Leg", new Vector3(Mathf.Sin(a) * 0.9f, 0, Mathf.Cos(a) * 0.9f), new Vector3(0, 1.5f, 0), 0.02f, metal);
            }
            Rod(t, "Mast", new Vector3(0, 0, 0), new Vector3(0, 6.5f, 0), 0.03f, metal);
            // two sector panels aimed at the course (north), slight downtilt
            foreach (float yaw in new[] { -25f, 25f })
            {
                var g = Group(t, "Sector Panel", new Vector3(0, 6.0f, 0), yaw);
                var panel = Box(g, "Panel", new Vector3(0, 0, 0.12f), new Vector3(0.18f, 0.7f, 0.08f), white, col: false);
                panel.transform.localRotation = Quaternion.Euler(5, 0, 0);
            }
            // yagi
            var yagi = Group(t, "Yagi", new Vector3(0, 5.2f, 0));
            Box(yagi, "Boom", new Vector3(0, 0, 0.6f), new Vector3(0.025f, 0.025f, 1.3f), metal, col: false);
            for (int k = 0; k < 8; k++)
                Box(yagi, "Element", new Vector3(0, 0, 0.05f + k * 0.17f), new Vector3(0.36f - k * 0.015f, 0.008f, 0.008f), metal, col: false);
            Rod(t, "Omni", new Vector3(0, 6.5f, 0), new Vector3(0, 7.4f, 0), 0.012f, white);
            Box(t, "Radio Box", new Vector3(0, 1.2f, 0.06f), new Vector3(0.2f, 0.3f, 0.08f), dark, col: false);
        }

        static void BuildControlStation()
        {
            float g = padHeight;
            var st = Group(root, "Control Station", new Vector3(StationCenter.x, g, StationCenter.y));
            var blue = M("CanopyBlue", new Color(0.08f, 0.2f, 0.55f), 0.15f);
            var gold = M("CanopyGold", new Color(0.95f, 0.72f, 0.1f), 0.15f);
            var black = M("CaseBlack", new Color(0.07f, 0.07f, 0.07f), 0.35f);
            var yellow = M("CaseYellow", new Color(0.95f, 0.75f, 0.05f), 0.35f);
            var red = M("GeneratorRed", new Color(0.7f, 0.08f, 0.06f), 0.4f);
            var orange = M("ConeOrange", new Color(1f, 0.38f, 0.05f), 0.3f);
            var tape = M("TapeWhite", new Color(0.95f, 0.95f, 0.95f), 0.2f);
            var wood = M("Plywood", new Color(0.75f, 0.62f, 0.42f), 0.1f);

            // operator tents: comms/driving under the blue one, pit/repair under the gold one
            Canopy(st, new Vector3(-1.6f, 0, 0), blue);
            Canopy(st, new Vector3(1.6f, 0, 0), gold);
            Table(st, new Vector3(-1.6f, 0, 0.5f), 0);
            Laptop(st, new Vector3(-2.1f, 0.74f, 0.45f), 180);
            Laptop(st, new Vector3(-1.4f, 0.74f, 0.45f), 180);
            var mon = Group(st, "Monitor", new Vector3(-0.85f, 0.74f, 0.65f), 180);
            Box(mon, "Stand", new Vector3(0, 0.12f, 0), new Vector3(0.05f, 0.24f, 0.05f), black, col: false);
            Box(mon, "Bezel", new Vector3(0, 0.38f, 0), new Vector3(0.6f, 0.36f, 0.03f), black, col: false);
            Box(mon, "Display", new Vector3(0, 0.38f, 0.016f), new Vector3(0.57f, 0.33f, 0.002f), M("Screen", Color.black), col: false);
            Box(st, "Gamepad", new Vector3(-1.75f, 0.76f, 0.2f), new Vector3(0.16f, 0.04f, 0.1f), black, col: false);
            Box(st, "Router", new Vector3(-2.35f, 0.76f, 0.75f), new Vector3(0.25f, 0.04f, 0.18f), tape, col: false);
            Chair(st, new Vector3(-2.1f, 0, -0.2f), 0);
            Chair(st, new Vector3(-1.2f, 0, -0.25f), 10);
            Table(st, new Vector3(1.6f, 0, 0.5f), 0);
            Box(st, "Toolbox", new Vector3(1.1f, 0.87f, 0.5f), new Vector3(0.5f, 0.25f, 0.25f), red);
            Box(st, "Battery", new Vector3(1.8f, 0.8f, 0.6f), new Vector3(0.2f, 0.12f, 0.15f), black, col: false);
            Box(st, "Battery", new Vector3(2.05f, 0.8f, 0.6f), new Vector3(0.2f, 0.12f, 0.15f), black, col: false);
            Box(st, "Charger", new Vector3(2.3f, 0.78f, 0.35f), new Vector3(0.22f, 0.08f, 0.16f), yellow, col: false);
            Chair(st, new Vector3(2.2f, 0, -0.4f), -20);

            // cases and spares behind the tents
            for (int k = 0; k < 5; k++)
                Box(st, "Pelican Case", new Vector3(-2.5f + k * 1.2f, 0.18f, -2.4f), new Vector3(0.62f, 0.36f, 0.48f), k % 2 == 0 ? black : yellow, Random.Range(-10f, 10f));
            Box(st, "Spare Wheel Crate", new Vector3(3.6f, 0.3f, -2.2f), new Vector3(0.9f, 0.6f, 0.9f), wood, 12);

            var gen = Group(st, "Generator", new Vector3(7.5f, 0, -3f), 20);
            Box(gen, "Body", new Vector3(0, 0.25f, 0), new Vector3(0.6f, 0.45f, 0.45f), red);
            Box(gen, "Panel", new Vector3(0.301f, 0.28f, 0), new Vector3(0.01f, 0.2f, 0.3f), black, col: false);
            Box(gen, "Fuel Can", new Vector3(0.7f, 0.2f, 0.1f), new Vector3(0.18f, 0.38f, 0.3f), red);

            Mast(st, new Vector3(4.6f, 0, 1.2f));
            Truck(st, new Vector3(-9f, 0, -2.5f), 100);

            var potty = Group(st, "Porta Potty", new Vector3(11f, 0, -6f), -30);
            Box(potty, "Booth", new Vector3(0, 1.15f, 0), new Vector3(1.1f, 2.3f, 1.1f), M("PottyBlue", new Color(0.1f, 0.35f, 0.7f), 0.3f));

            // flag + sign
            var metal = M("Aluminium", new Color(0.75f, 0.75f, 0.78f), 0.6f);
            Rod(st, "Flag Pole", new Vector3(-4.5f, 0, 2.5f), new Vector3(-4.5f, 5f, 2.5f), 0.025f, metal);
            Box(st, "Flag", new Vector3(-4.0f, 4.6f, 2.5f), new Vector3(1.0f, 0.6f, 0.01f), blue, col: false);
            Box(st, "Flag Stripe", new Vector3(-4.0f, 4.6f, 2.505f), new Vector3(1.0f, 0.15f, 0.01f), gold, col: false);
            var sign = Group(st, "Sign", new Vector3(3.2f, 0, 3.3f));
            Rod(sign, "Post", Vector3.zero, new Vector3(0, 1.6f, 0), 0.03f, metal);
            Box(sign, "Board", new Vector3(0, 1.45f, 0.03f), new Vector3(1.7f, 0.4f, 0.02f), tape, col: false);
            Text(sign, "BASE STATION", new Vector3(0, 1.45f, 0.05f), 180, Color.black, 0.03f);

            // rover start box (2.4 m square of tape) and a coned lane heading north
            var start = Group(st, "Rover Start Box", new Vector3(0, 0.005f, 14f));
            foreach (var (x, z, sx, sz) in new[] { (0f, -1.2f, 2.45f, 0.05f), (0f, 1.2f, 2.45f, 0.05f), (-1.2f, 0f, 0.05f, 2.45f), (1.2f, 0f, 0.05f, 2.45f) })
                Box(start, "Tape", new Vector3(x, 0, z), new Vector3(sx, 0.01f, sz), tape, col: false);
            var startText = Text(start, "START", new Vector3(0, 0.01f, -1.7f), 0, Color.white, 0.05f);
            startText.transform.localRotation = Quaternion.Euler(90, 0, 0);
            for (int k = 0; k < 4; k++)
                foreach (float x in new[] { -2.2f, 2.2f })
                {
                    var c = MeshObject(st, "Cone", Cone(), orange);
                    c.transform.localPosition = new Vector3(x, 0, 17f + k * 3f);
                    c.transform.localScale = new Vector3(0.3f, 0.7f, 0.3f);
                    var cc = c.AddComponent<MeshCollider>();
                    cc.sharedMesh = Cone(); cc.convex = true;
                }
            report.AppendLine($"control station at ({StationCenter.x}, {g:F2}, {StationCenter.y}), rover start ({StartPos.x}, {StartPos.y})");
        }

        static GameObject Text(Transform parent, string text, Vector3 pos, float yaw, Color color, float charSize)
        {
            var go = new GameObject("Text " + text);
            go.transform.SetParent(parent, false);
            go.transform.localPosition = pos;
            go.transform.localRotation = Quaternion.Euler(0, yaw, 0);
            var tm = go.AddComponent<TextMesh>();
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            tm.font = font;
            go.GetComponent<MeshRenderer>().sharedMaterial = font.material;
            tm.text = text; tm.fontSize = 64; tm.characterSize = charSize; tm.anchor = TextAnchor.MiddleCenter; tm.color = color;
            return go;
        }

        // ------------------------------------------------------------------ waypoints

        static void BuildWaypoints()
        {
            var group = new GameObject("Waypoints").transform;
            group.SetParent(root, false);
            var icon = EditorGUIUtility.IconContent("sv_label_1").image as Texture2D;
            var startGo = new GameObject("START (GNSS origin)");
            startGo.transform.SetParent(group, false);
            startGo.transform.position = new Vector3(StartPos.x, Ground(StartPos), StartPos.y);
            if (icon) EditorGUIUtility.SetIconForObject(startGo, icon);

            var markerPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(ArucoPrefabPath);
            if (markerPrefab == null) Debug.LogWarning($"[Badlands] {ArucoPrefabPath} not found, waypoints get no ArUco posts");
            foreach (var w in Waypoints)
            {
                var go = new GameObject($"{w.name}{(w.aruco >= 0 ? $" (ArUco {w.aruco})" : " (GNSS only)")}");
                go.transform.SetParent(group, false);
                go.transform.position = new Vector3(w.p.x, Ground(w.p), w.p.y);
                if (icon) EditorGUIUtility.SetIconForObject(go, icon);
                if (w.aruco < 0 || markerPrefab == null) continue;

                // the team's ArUco post prefab, linked (edits to the prefab show up here). TripleAruco picks the marker
                // texture from 'id' when Play starts, so the faces look blank in edit mode.
                var marker = (GameObject)PrefabUtility.InstantiatePrefab(markerPrefab, go.transform);
                marker.GetComponent<TripleAruco>().id = w.aruco;
                PrefabUtility.RecordPrefabInstancePropertyModifications(marker.GetComponent<TripleAruco>());
                // the prefab's pivot is not at its post, so move it until the bottom of "Post" sits on the ground here
                var post = marker.GetComponentsInChildren<Renderer>().FirstOrDefault(r => r.name == "Post");
                if (post != null)
                {
                    var b = post.bounds;
                    marker.transform.position += go.transform.position - new Vector3(b.center.x, b.min.y, b.center.z);
                }
                else Debug.LogWarning("[Badlands] ArUco prefab has no child called 'Post', can't place it on the ground");
            }
        }

        static void WriteWaypointFile()
        {
            const double R = 6378137.0, Lat0 = 49.0, Lon0 = -123.0;
            var sb = new StringBuilder();
            sb.AppendLine("# Badlands traversal waypoints (generated by BadlandsBuilder, don't edit)");
            sb.AppendLine("# GNSS origin = rover start box, (49.0, -123.0), the LocationStuff default. Unity +x = east, +z = north.");
            sb.AppendLine("name, unity_x, unity_z, east_m, north_m, latitude, longitude, ground_alt_rel_m, note");
            var start = StartPos;
            float g0 = Ground(start);
            foreach (var w in new[] { new Waypoint { name = "START", p = start, aruco = -1, note = "Rover start box at the control station." } }.Concat(Waypoints))
            {
                double e = w.p.x - start.x, n = w.p.y - start.y;
                double lat = Lat0 + n / R * 180 / System.Math.PI;
                double lon = Lon0 + e / (R * System.Math.Cos(Lat0 * System.Math.PI / 180)) * 180 / System.Math.PI;
                sb.AppendLine(string.Format(System.Globalization.CultureInfo.InvariantCulture, "{0}, {1:F1}, {2:F1}, {3:F1}, {4:F1}, {5:F7}, {6:F7}, {7:F1}, {8}",
                    w.name + (w.aruco >= 0 ? $" (ArUco {w.aruco})" : ""), w.p.x, w.p.y, e, n, lat, lon, Ground(w.p) - g0, w.note));
            }
            File.WriteAllText(AssetDir + "/waypoints.csv", sb.ToString());
            AssetDatabase.ImportAsset(AssetDir + "/waypoints.csv");
            report.Append(sb);
        }

        static void EnsureCamera()
        {
            if (Object.FindObjectOfType<Camera>()) return;
            var cam = new GameObject("Spectator Camera (right-drag + WASD)").AddComponent<Camera>();
            cam.tag = "MainCamera";
            cam.farClipPlane = 2500;
            cam.gameObject.AddComponent<AudioListener>();
            cam.gameObject.AddComponent<FreeFlyCamera>();
            var p = StationCenter + new Vector2(-8, -12);
            cam.transform.position = new Vector3(p.x, padHeight + 5, p.y);
            cam.transform.LookAt(new Vector3(StationCenter.x, padHeight + 2, StationCenter.y + 40));
        }

        // ------------------------------------------------------------------ checks + screenshots

        static void Verify()
        {
            Physics.SyncTransforms();
            float Hit(float x, float z) => Physics.Raycast(new Vector3(x, 200, z), Vector3.down, out var h, 400) ? h.point.y : float.NaN;
            var sb = new StringBuilder("[Badlands] checks\n");
            (float maxDeg, float rise) Profile(System.Func<float, Vector2> path, float t0, float t1)
            {
                float worst = 0, prev = float.NaN, y0 = float.NaN, y1 = 0;
                for (float t = t0; t <= t1; t += 1f)
                {
                    var p = path(t); float y = Hit(p.x, p.y);
                    if (float.IsNaN(y0)) y0 = y;
                    if (!float.IsNaN(prev)) worst = Mathf.Max(worst, Mathf.Atan(Mathf.Abs(y - prev)) * Mathf.Rad2Deg);
                    prev = y; y1 = y;
                }
                return (worst, y1 - y0);
            }
            float ze(float x) => EscZ(x);
            var west = Profile(t => new Vector2(WestRampX, ze(WestRampX) + t), -35, 35);
            var east = Profile(t => new Vector2(EastTalusX, ze(EastTalusX) + t), -25, 25);
            var slot = Profile(t => new Vector2(SlotCenter(ze(SlotX) + t), ze(SlotX) + t), -5, SlotLen + 5);
            sb.AppendLine($"west ramp: rise {west.rise:F1} m, steepest 1 m step {west.maxDeg:F0} deg");
            sb.AppendLine($"east talus: rise {east.rise:F1} m, steepest {east.maxDeg:F0} deg (plus boulders)");
            sb.AppendLine($"slot canyon floor: rise {slot.rise:F1} m, steepest {slot.maxDeg:F0} deg");
            float zm = ze(SlotX) + 12;
            float floor = Hit(SlotCenter(zm), zm);
            float wl = 0, wr = 0;
            for (float dx = 0; dx < 8; dx += 0.1f) { if (wl == 0 && Hit(SlotCenter(zm) - dx, zm) > floor + 2) wl = dx; if (wr == 0 && Hit(SlotCenter(zm) + dx, zm) > floor + 2) wr = dx; }
            sb.AppendLine($"slot canyon width 12 m in: {wl + wr:F1} m, walls {Hit(SlotCenter(zm) + 5, zm) - floor:F1} m high");
            float gap = float.MaxValue, gapAt = 0;
            for (float z = ze(SlotX) + 1; z < ze(SlotX) + SlotLen - 3; z += 0.5f)
            {
                var o = new Vector3(SlotCenter(z), Hit(SlotCenter(z), z) + 0.5f, z);
                if (Physics.Raycast(o, Vector3.left, out var a, 10) && Physics.Raycast(o, Vector3.right, out var b, 10) && a.distance + b.distance < gap)
                { gap = a.distance + b.distance; gapAt = z - ze(SlotX); }
            }
            sb.AppendLine($"slot canyon narrowest gap 0.5 m above the floor (walls + rocks): {gap:F1} m, {gapAt:F0} m in");
            foreach (float x in new[] { -80f, 60f, 220f })
                sb.AppendLine($"escarpment at x={x}: cliff {Hit(x, ze(x) + 3) - Hit(x, ze(x) - 3):F1} m over 6 m");
            sb.AppendLine($"ledge band at x=-150: {Hit(-150, TerZ(-150) + 3) - Hit(-150, TerZ(-150) - 3):F1} m");
            foreach (var (nme, x) in new[] { ("west ford", FordW), ("east ford", FordE), ("wash at x=0", 0f), ("wash at x=-200", -200f) })
            {
                var pr = Profile(t => new Vector2(x, ArZ(x) + t), -16, 16);
                sb.AppendLine($"{nme}: steepest {pr.maxDeg:F0} deg, bed {Hit(x, ArZ(x)) - Hit(x, ArZ(x) + 16):F1} m vs bank");
            }
            foreach (var b in Buttes) sb.AppendLine($"butte at {b.c}: {Hit(b.c.x, b.c.y) - Hit(b.c.x + b.r + 8, b.c.y):F1} m tall");
            sb.AppendLine($"crater: floor {Hit(CraterC.x, CraterC.y) - Hit(CraterC.x + CraterR, CraterC.y):F1} m below rim");
            float pmin = float.MaxValue, pmax = float.MinValue;
            for (float dx = -10; dx <= 10; dx += 1) for (float dz = -10; dz <= 18; dz += 1)
            { float y = Hit(StationCenter.x + dx, StationCenter.y + dz); if (y < padHeight + 0.5f) { pmin = Mathf.Min(pmin, y); pmax = Mathf.Max(pmax, y); } }
            sb.AppendLine($"station pad: ground {pmin:F2}..{pmax:F2} (target {padHeight:F2})");
            Debug.Log(sb.ToString());
        }

        static string Arg(string name)
        {
            var a = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < a.Length - 1; i++) if (a[i] == name) return a[i + 1];
            return null;
        }

        static string Safe(string s) => new string(s.Select(ch => char.IsLetterOrDigit(ch) ? ch : '_').ToArray());

        static void Screenshots(string dir)
        {
            Directory.CreateDirectory(dir);
            var cam = new GameObject("shotcam").AddComponent<Camera>();
            cam.farClipPlane = 3000;
            const int W = 1600, Hh = 900;
            var rt = new RenderTexture(W, Hh, 24) { antiAliasing = 4 };
            var tex = new Texture2D(W, Hh, TextureFormat.RGB24, false);
            void Shot(string name)
            {
                cam.targetTexture = rt;
                cam.Render();
                RenderTexture.active = rt;
                tex.ReadPixels(new Rect(0, 0, W, Hh), 0, 0);
                tex.Apply();
                File.WriteAllBytes(Path.Combine(dir, name + ".png"), tex.EncodeToPNG());
            }
            // top-down map of the detailed tile
            cam.orthographic = true;
            cam.orthographicSize = 260;
            cam.aspect = 1;
            var rtSq = new RenderTexture(1400, 1400, 24);
            cam.transform.position = new Vector3(0, 400, 0);
            cam.transform.rotation = Quaternion.Euler(90, 0, 0);
            cam.targetTexture = rtSq;
            var shadow = QualitySettings.shadowDistance;
            QualitySettings.shadowDistance = 600;
            cam.Render();
            RenderTexture.active = rtSq;
            var sq = new Texture2D(1400, 1400, TextureFormat.RGB24, false);
            sq.ReadPixels(new Rect(0, 0, 1400, 1400), 0, 0); sq.Apply();
            File.WriteAllBytes(Path.Combine(dir, "map_topdown.png"), sq.EncodeToPNG());
            QualitySettings.shadowDistance = shadow;
            cam.orthographic = false;
            cam.fieldOfView = 55;
            cam.ResetAspect();

            float zs = EscZ(SlotX), zw = EscZ(WestRampX), zt = EscZ(EastTalusX);
            var views = new (string, Vector3, Vector3)[]
            {
                ("overview_south", new Vector3(0, 160, -430), new Vector3(0, 0, 0)),
                ("overview_east", new Vector3(420, 120, -120), new Vector3(0, 0, 20)),
                ("station", new Vector3(-36, 5, -224), new Vector3(-20, 1, -200)),
                ("rover_eye_start", new Vector3(StartPos.x, 1.1f, StartPos.y), new Vector3(StartPos.x + 10, 0.5f, StartPos.y + 60)),
                ("escarpment", new Vector3(-30, 4, -5), new Vector3(10, 8, 60)),
                ("slot_entrance", new Vector3(SlotX - 3, 3, zs - 22), new Vector3(SlotX, 4, zs + 10)),
                ("slot_inside", new Vector3(SlotCenter(zs + 4), 1.3f, zs + 4), new Vector3(SlotCenter(zs + 30), 1.5f, zs + 30)),
                ("slot_mouth_above", new Vector3(SlotX + 6, 22, zs - 16), new Vector3(SlotX, 0, zs + 3)),
                ("slot_back", new Vector3(SlotCenter(zs + SlotLen + 8), 3, zs + SlotLen + 8), new Vector3(SlotCenter(zs + 22), 0, zs + 22)),
                ("slot_corner_west", new Vector3(SlotX - 12, 14, zs - 10), new Vector3(SlotCenter(zs) - SlotHalf - 0.6f, 0, zs + 0.5f)),
                ("slot_corner_east", new Vector3(SlotX + 12, 14, zs - 10), new Vector3(SlotCenter(zs) + SlotHalf + 0.6f, 0, zs + 0.5f)),
                ("seam_east", new Vector3(225, 15, -40), new Vector3(275, 0, 10)),
                ("escarpment_east_end", new Vector3(470, 30, -20), new Vector3(540, 10, 40)),
                ("west_ramp", new Vector3(WestRampX + 45, 8, zw - 35), new Vector3(WestRampX, 6, zw + 5)),
                ("east_talus", new Vector3(EastTalusX - 40, 6, zt - 30), new Vector3(EastTalusX, 6, zt)),
                ("west_ramp_east_corner", new Vector3(WestRampX + 32, 5, zw - 16), new Vector3(WestRampX + 15, 4, zw + 2)),
                ("west_ramp_from_top", new Vector3(WestRampX, 4, zw + 32), new Vector3(WestRampX + 5, -4, zw - 10)),
                ("east_talus_west_corner", new Vector3(EastTalusX - 42, 5, zt - 14), new Vector3(EastTalusX - 23, 4, zt + 2)),
                ("hoodoos", new Vector3(95, 4, -90), new Vector3(150, 3, -50)),
                ("boulder_field", new Vector3(-115, 3, -100), new Vector3(-165, 0, -50)),
                ("arroyo", new Vector3(-60, 4, ArZ(-60) - 25), new Vector3(-30, -1, ArZ(-30))),
                ("west_ford", new Vector3(FordW + 18, 2.5f, ArZ(FordW) - 18), new Vector3(FordW, -1, ArZ(FordW))),
                ("dunes", new Vector3(130, 8, 110), new Vector3(190, 2, 170)),
                ("crater", new Vector3(-120, 10, 180), new Vector3(-150, -1, 210)),
                ("ledge_band", new Vector3(-120, 3, TerZ(-120) - 25), new Vector3(-140, 3, TerZ(-140))),
                ("ledge_ramp", new Vector3(-48, 4, TerZ(-48) - 22), new Vector3(-68, 1, TerZ(-68))),
                ("ledge_end", new Vector3(-20, 5, TerZ(-20) - 18), new Vector3(-45, 0, TerZ(-45))),
                ("station_sign", new Vector3(-20 + 3.2f, 1.6f, -205 + 6f), new Vector3(-20 + 3.2f, 1.45f, -205 + 3.3f)),
                ("butte", new Vector3(10, 4, 105), new Vector3(70, 10, 150)),
                ("waypoint_post", new Vector3(-37, 1.5f, 101), new Vector3(-40, 1.2f, 105)),
            };
            foreach (var (name, pos, look) in views)
            {
                cam.transform.position = pos + Vector3.up * Ground(pos.x, pos.z);
                cam.transform.LookAt(look + Vector3.up * Ground(look.x, look.z));
                Shot(name);
            }
            RenderTexture.active = null;
            Object.DestroyImmediate(cam.gameObject);
            Debug.Log("[Badlands] screenshots in " + dir);
        }
    }
}
