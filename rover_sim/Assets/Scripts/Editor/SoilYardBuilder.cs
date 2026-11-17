using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Soil test yard (Assets/Scenes/SoilYard.unity): five 12 m lanes of the Utah soils side by side (loose sand, hardpack,
    /// bentonite crust, gravel, slickrock), each with a flat run, a row of bumps, and a 15 deg (west half) / 25 deg (east
    /// half) ramp onto a plateau. A mini_rover starts on the sand lane. For trying tyres and soil settings.
    ///   menu SimFlake > World Generation > Build Soil Yard, or
    ///   Unity -batchmode -projectPath rover_sim -executeMethod SimFlake.EditorTools.SoilYardBuilder.BuildFromCommandLine
    /// Also: <see cref="AddSoilToBadlands"/> turns the Badlands centre terrain into soil without rebuilding that world.
    /// </summary>
    public static class SoilYardBuilder
    {
        public const string ScenePath = "Assets/Scenes/SoilYard.unity";
        const string AssetDir = "Assets/Worlds/SoilYard";
        const string TexDir = "Assets/Worlds/BadlandsTextures";
        const float Size = 80f, Height = 10f, LaneWidth = 12f;
        const int Res = 513, Splat = 1024;
        public static readonly (string layer, string normal, float tile, string label)[] Lanes =
        {
            ("WashSand", "Sand", 3f, "LOOSE SAND"), ("DesertSoil", "Pebbles_B", 3f, "HARDPACK"), ("GreyClay", "Rock", 4f, "BENTONITE CRUST"),
            ("Gravel", "Pebbles_C", 2.5f, "GRAVEL"), ("Sandstone", "Rock", 6f, "SLICKROCK"),
        };
        public static float LaneX(int i) => (i - 2) * LaneWidth;
        public const float StartZ = -30f, RampZ = 8f, RampRun = 8f, PlateauRun = 5f;

        [MenuItem("SimFlake/World Generation/Build Soil Yard", priority = 104)]
        public static void BuildMenu()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            Build();
        }

        public static void BuildFromCommandLine()
        {
            try { Build(); AddSoilToBadlands(); EditorApplication.Exit(0); }
            catch (System.Exception e) { Debug.LogException(e); EditorApplication.Exit(1); }
        }

        static float Smooth(float a, float b, float x) => Mathf.SmoothStep(0, 1, Mathf.InverseLerp(a, b, x));

        /// <summary>Ground height (m) at world x, z.</summary>
        public static float H(float x, float z)
        {
            float h = 0;
            if (Mathf.Abs(x) < 2.5f * LaneWidth + 2f)
            {
                // which half of its lane: west = 15 deg, east = 25 deg, 1 m blend
                float local = Mathf.Repeat(x + LaneWidth * 0.5f, LaneWidth) - LaneWidth * 0.5f;
                float steep = Smooth(-0.5f, 0.5f, local) * (1f - Smooth(LaneWidth * 0.5f - 0.5f, LaneWidth * 0.5f, Mathf.Abs(local)));
                float slope = Mathf.Lerp(Mathf.Tan(15 * Mathf.Deg2Rad), Mathf.Tan(25 * Mathf.Deg2Rad), steep);
                float top = slope * RampRun;
                float up = Mathf.Clamp01((z - RampZ) / RampRun), down = Mathf.Clamp01((z - RampZ - RampRun - PlateauRun) / (top / Mathf.Tan(20 * Mathf.Deg2Rad)));
                h = top * (up - down);
                // bumps before the ramp: 5 cm high, 0.6 m long, staggered left/right so the bogies work
                if (z > -6f && z < 0f)
                {
                    float phase = (z + 6f + (local > 0 ? 0.75f : 0f)) % 1.5f;
                    if (phase < 0.6f) h += 0.05f * Mathf.Sin(phase / 0.6f * Mathf.PI) * Mathf.Sin(phase / 0.6f * Mathf.PI);
                }
                h *= 1f - Smooth(2.5f * LaneWidth, 2.5f * LaneWidth + 2f, Mathf.Abs(x));
            }
            return h;
        }

        public static void Build()
        {
            SoilSetup.EnsureAssets();
            SoilSetup.EnsureMaterials();
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            if (AssetDatabase.IsValidFolder(AssetDir)) AssetDatabase.DeleteAsset(AssetDir);
            Directory.CreateDirectory(AssetDir);
            AssetDatabase.Refresh();

            var layers = Lanes.Select(l =>
            {
                var tl = new TerrainLayer
                {
                    name = l.layer,
                    diffuseTexture = AssetDatabase.LoadAssetAtPath<Texture2D>($"{TexDir}/{l.layer}_Albedo.jpg"),
                    normalMapTexture = AssetDatabase.LoadAssetAtPath<Texture2D>($"Assets/ThirdParty/TerrainSampleAssets/Textures/Terrain/{l.normal}_Normal.tif"),
                    tileSize = new Vector2(l.tile, l.tile), normalScale = 1, smoothness = 0.05f, metallic = 0,
                };
                AssetDatabase.CreateAsset(tl, $"{AssetDir}/{l.layer}.terrainlayer");
                return tl;
            }).ToArray();

            var td = new TerrainData { heightmapResolution = Res };
            td.size = new Vector3(Size, Height, Size);
            var origin = new Vector3(-Size / 2, 0, -Size / 2);
            var hm = new float[Res, Res];
            float step = Size / (Res - 1);
            for (int zi = 0; zi < Res; zi++) for (int xi = 0; xi < Res; xi++) hm[zi, xi] = H(origin.x + xi * step, origin.z + zi * step) / Height;
            td.SetHeights(0, 0, hm);
            td.alphamapResolution = Splat;
            td.baseMapResolution = 1024;
            td.terrainLayers = layers;
            var a = new float[Splat, Splat, layers.Length];
            for (int zi = 0; zi < Splat; zi++)
            for (int xi = 0; xi < Splat; xi++)
            {
                float x = origin.x + xi * Size / (Splat - 1f);
                float sum = 0;
                for (int l = 0; l < layers.Length; l++)
                {
                    float w = 1f - Smooth(LaneWidth * 0.5f - 0.25f, LaneWidth * 0.5f + 0.25f, Mathf.Abs(x - LaneX(l)));
                    if (l == 1 && Mathf.Abs(x) > 2.5f * LaneWidth) w = 1;      // outside the lanes: hardpack
                    a[zi, xi, l] = w; sum += w;
                }
                for (int l = 0; l < layers.Length; l++) a[zi, xi, l] /= Mathf.Max(sum, 1e-4f);
            }
            // the splat maps are sub-assets: set them once the TerrainData is an asset (set before, they come back blank)
            AssetDatabase.CreateAsset(td, $"{AssetDir}/Terrain.asset");
            td.SetAlphamaps(0, 0, a);
            EditorUtility.SetDirty(td);

            var go = Terrain.CreateTerrainGameObject(td);
            go.name = "Soil Yard Terrain";
            go.transform.position = origin;
            var terrain = go.GetComponent<Terrain>();
            terrain.heightmapPixelError = 1;
            terrain.basemapDistance = 400;
            terrain.drawInstanced = false;
            go.GetComponent<TerrainCollider>().sharedMaterial = AssetDatabase.LoadAssetAtPath<PhysicMaterial>("Assets/Materials/CustomDirt.physicMaterial");
            SoilSetup.MakeSoil(terrain);
            EnsureSoilObject();

            // lane labels
            var labels = new GameObject("Lane Labels").transform;
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            for (int i = 0; i < Lanes.Length; i++)
            {
                var t = new GameObject(Lanes[i].label).AddComponent<TextMesh>();
                t.transform.SetParent(labels, false);
                t.text = Lanes[i].label; t.font = font; t.fontSize = 96; t.characterSize = 0.08f; t.anchor = TextAnchor.MiddleCenter; t.color = Color.white;
                t.GetComponent<MeshRenderer>().sharedMaterial = font.material;
                t.transform.position = new Vector3(LaneX(i), 2.6f, StartZ - 5f);
            }

            SkyBuilder.Apply(SkyBuilder.Desert);
            var ls = new LightingSettings { name = "SoilYardLighting", bakedGI = false, realtimeGI = false };
            AssetDatabase.CreateAsset(ls, AssetDir + "/SoilYardLighting.lighting");
            Lightmapping.lightingSettings = ls;

            var rover = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(DrivetrainSetup.MiniRover));
            rover.transform.position = new Vector3(LaneX(0), 0.35f, StartZ);
            var cam = new GameObject("Spectator Camera (right-drag + WASD)").AddComponent<Camera>();
            cam.tag = "MainCamera"; cam.farClipPlane = 1500; cam.nearClipPlane = 0.05f;
            cam.gameObject.AddComponent<AudioListener>();
            cam.gameObject.AddComponent<FreeFlyCamera>();
            cam.transform.position = new Vector3(LaneX(0) - 1.2f, 0.9f, StartZ - 2.2f);
            cam.transform.LookAt(new Vector3(LaneX(0), 0.15f, StartZ + 1f));
            new GameObject("Soil Demo (--soil-demo <dir>)").AddComponent<SoilDemo>();

            AssetDatabase.SaveAssets();
            EditorSceneManager.SaveScene(scene, ScenePath);
            Debug.Log("[SoilYard] saved " + ScenePath);
            Verify(terrain);
        }

        /// <summary>Checks: each lane reports its soil, and SoilTerrain's height matches the terrain's own.</summary>
        static void Verify(Terrain terrain)
        {
            var st = terrain.GetComponent<SoilTerrain>();
            st.Reinit();
            var lanes = string.Join(", ", Enumerable.Range(0, Lanes.Length).Select(i => $"{Lanes[i].label} = {st.SpecAt(LaneX(i), StartZ)?.name}"));
            float worst = 0;
            var rnd = new System.Random(3);
            for (int i = 0; i < 2000; i++)
            {
                float x = (float)(rnd.NextDouble() - 0.5) * (Size - 2), z = (float)(rnd.NextDouble() - 0.5) * (Size - 2);
                worst = Mathf.Max(worst, Mathf.Abs(st.BaseHeight(x, z) - (terrain.SampleHeight(new Vector3(x, 0, z)) + terrain.transform.position.y)));
            }
            Debug.Log($"[SoilYard] lanes: {lanes}; SoilTerrain height vs Terrain.SampleHeight: worst difference {worst * 1000:F2} mm");
        }

        /// <summary>The scene's SoilWorld + SoilRenderer object.</summary>
        public static SoilWorld EnsureSoilObject()
        {
            var w = Object.FindObjectOfType<SoilWorld>();
            if (w == null) w = new GameObject("Soil (quality: F9)").AddComponent<SoilWorld>();
            if (w.GetComponent<SoilRenderer>() == null) w.gameObject.AddComponent<SoilRenderer>();
            return w;
        }

        /// <summary>Make the Badlands centre terrain soil (adds SoilTerrain + the Soil object, saves the scene). No rebuild needed.</summary>
        [MenuItem("SimFlake/Soil/Add Soil To Badlands (centre terrain)", priority = 262)]
        public static void AddSoilToBadlands()
        {
            const string path = "Assets/Scenes/TaskWorlds/BadlandsTraversal.unity";
            if (!File.Exists(path)) return;
            SoilSetup.EnsureMaterials();
            var scene = EditorSceneManager.OpenScene(path);
            var centre = Object.FindObjectsOfType<Terrain>().OrderByDescending(t => t.terrainData.heightmapResolution).FirstOrDefault(t => t.name.Contains("centre"));
            if (centre == null) { Debug.LogWarning("[Soil] no centre terrain in Badlands"); return; }
            var st = SoilSetup.MakeSoil(centre);
            EnsureSoilObject();
            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            Debug.Log("[Soil] Badlands centre terrain is soil: " + string.Join(", ", centre.terrainData.terrainLayers.Zip(st.layerSpecs, (l, s) => l.name + " = " + s.name.Replace("Utah_", ""))));
        }
    }
}
