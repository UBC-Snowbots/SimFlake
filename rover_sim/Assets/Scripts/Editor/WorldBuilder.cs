using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Generates the test features in ChassisPlayground: a sandy rock pit, a drop course (0.3/0.6/1.0 m)
    /// and a slope course (10/20/30 deg). Everything goes under one root object "Generated Features",
    /// which is deleted and rebuilt on every run, so tweak the numbers here and re-run.
    ///   Menu: SimFlake > Build Playground Features
    ///   Headless: Unity -batchmode -projectPath rover_sim -executeMethod SimFlake.EditorTools.WorldBuilder.BuildFromCommandLine
    /// Add -screenshots <dir> (without -nographics) to also render preview PNGs.
    /// </summary>
    public static class WorldBuilder
    {
        const string ScenePath = "Assets/Scenes/ChassisPlayground.unity";
        const string AssetDir = "Assets/Worlds/ChassisPlayground";
        const string RootName = "Generated Features";

        // Layout (world coordinates, rover spawns near (60.6, 47.3) facing +x / east).
        static readonly Vector3 PitCenter = new Vector3(66f, 0f, 24f);
        const float PitHalf = 10f;     // 20 x 20 m inner area
        const float BermWidth = 2.5f;  // sloped sand rim outside the inner area
        const float PitRes = 0.25f;    // mesh grid spacing (m)

        static Terrain terrain;
        static PhysicMaterial groundPhysics;

        [MenuItem("SimFlake/Build Playground Features")]
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
            var scene = EditorSceneManager.OpenScene(ScenePath);
            foreach (var old in scene.GetRootGameObjects().Where(g => g.name == RootName)) Object.DestroyImmediate(old);
            if (AssetDatabase.IsValidFolder(AssetDir)) AssetDatabase.DeleteAsset(AssetDir);
            Directory.CreateDirectory(AssetDir);
            AssetDatabase.Refresh();

            terrain = Object.FindObjectOfType<Terrain>();
            groundPhysics = terrain.GetComponent<TerrainCollider>().sharedMaterial;

            var root = new GameObject(RootName);
            Random.InitState(2024);
            BuildSandPit(root.transform);
            BuildDropCourse(root.transform);
            BuildSlopeCourse(root.transform);
            SkyBuilder.Apply();

            AssetDatabase.SaveAssets();
            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            Debug.Log("[WorldBuilder] Done");
        }

        // ------------------------------------------------------------------ sand pit

        struct Mound { public float u, v, r, h; public Mound(float u, float v, float r, float h) { this.u = u; this.v = v; this.r = r; this.h = h; } }

        static readonly Mound[] Mounds =
        {
            new Mound(-4.5f, 3.5f, 2.6f, 0.9f),   // round hill
            new Mound(5f, -4f, 3.2f, 1.3f),       // big hill
            new Mound(3.5f, 5f, 1.4f, 0.45f),     // small bump
            new Mound(-5.5f, -5f, 2.0f, 0.7f),
            new Mound(-1f, -7f, 1.2f, 0.35f),
            new Mound(0.5f, -0.5f, 1.8f, -0.28f), // crater
            new Mound(-6.5f, 6.5f, 1.2f, -0.2f),  // small crater
        };

        /// <summary>Sand height above the terrain at pit-local (u, v).</summary>
        static float PitHeight(float u, float v)
        {
            const float floor = 0.3f, bermTop = 0.75f;
            float e = Mathf.Max(Mathf.Abs(u), Mathf.Abs(v));
            bool entry = v > 0 && Mathf.Abs(u) < 2.5f;   // gap in the north berm, facing the rover spawn
            float h;
            if (e > PitHalf) // outer slope down to the terrain
            {
                float t = Mathf.SmoothStep(0, 1, (e - PitHalf) / BermWidth);
                h = Mathf.Lerp(entry ? floor : bermTop, 0.02f, t);
            }
            else
            {
                float rim = entry ? 0 : Mathf.SmoothStep(0, 1, (e - (PitHalf - 1.5f)) / 1.5f);
                h = Mathf.Lerp(floor, bermTop, rim);
                float fade = 1 - rim;
                foreach (var m in Mounds)
                {
                    float d2 = ((u - m.u) * (u - m.u) + (v - m.v) * (v - m.v)) / (m.r * m.r);
                    h += m.h * Mathf.Exp(-2.2f * d2) * fade;
                }
                // wind ripples + lumpy noise
                h += fade * (0.04f * Mathf.Sin(0.9f * u + 0.5f * v + 3f * Mathf.PerlinNoise(u * 0.15f, v * 0.15f))
                           + 0.12f * (Mathf.PerlinNoise(u * 0.25f + 10f, v * 0.25f + 5f) - 0.5f));
            }
            return h;
        }

        static float Ground(float x, float z) => terrain.SampleHeight(new Vector3(x, 0, z)) + terrain.transform.position.y;
        static float PitWorldY(float u, float v) => Ground(PitCenter.x + u, PitCenter.z + v) + PitHeight(u, v);

        static void BuildSandPit(Transform parent)
        {
            var go = new GameObject("Sand Rock Pit (20x20 m)");
            go.transform.SetParent(parent, false);
            go.transform.position = new Vector3(PitCenter.x, 0, PitCenter.z);

            float ext = PitHalf + BermWidth;
            int n = Mathf.RoundToInt(2 * ext / PitRes) + 1;
            var verts = new Vector3[n * n];
            var uvs = new Vector2[n * n];
            for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                float u = -ext + i * PitRes, v = -ext + j * PitRes;
                verts[j * n + i] = new Vector3(u, PitWorldY(u, v), v);
                uvs[j * n + i] = new Vector2(u, v) / 2f; // sand texture tile = 2 m
            }
            var tris = new List<int>();
            for (int j = 0; j < n - 1; j++)
            for (int i = 0; i < n - 1; i++)
            {
                int a = j * n + i, b = a + 1, c = a + n, d = c + 1;
                tris.AddRange(new[] { a, c, b, b, c, d });
            }
            var mesh = new Mesh { name = "SandPit", vertices = verts, uv = uvs, triangles = tris.ToArray() };
            mesh.RecalculateNormals();
            mesh.RecalculateTangents();
            mesh.RecalculateBounds();
            AssetDatabase.CreateAsset(mesh, AssetDir + "/SandPit.asset");

            var sandLayer = AssetDatabase.LoadAssetAtPath<TerrainLayer>("Assets/TerrainSampleAssets/TerrainLayers/Sand_TerrainLayer.terrainlayer");
            var sand = NewMaterial("Sand", new Color(0.93f, 0.85f, 0.70f), sandLayer ? sandLayer.diffuseTexture : null,
                sandLayer ? sandLayer.normalMapTexture : null, 0.1f);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = sand;
            var col = go.AddComponent<MeshCollider>();
            col.sharedMesh = mesh;
            col.sharedMaterial = groundPhysics;
            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic | StaticEditorFlags.ContributeGI | StaticEditorFlags.OccluderStatic | StaticEditorFlags.OccludeeStatic);

            ScatterRocks(go.transform);
            Label(parent, "SAND ROCK PIT", new Vector3(PitCenter.x, PitWorldY(0, ext) + 1.6f, PitCenter.z + ext + 0.5f), 180);
        }

        static void ScatterRocks(Transform pit)
        {
            var prefabs = AssetDatabase.FindAssets("t:Prefab", new[] { "Assets/HQ Rocks/Prefabs" })
                .Select(g => AssetDatabase.LoadAssetAtPath<GameObject>(AssetDatabase.GUIDToAssetPath(g))).ToArray();
            if (prefabs.Length == 0) { Debug.LogWarning("[WorldBuilder] no HQ Rocks prefabs found"); return; }
            var rocks = new GameObject("Rocks").transform;
            rocks.SetParent(pit, false);

            // (count, min size, max size) in metres. Mostly small stuff the rover can climb, a few it can't.
            var bins = new[] { (60, 0.08f, 0.18f), (30, 0.2f, 0.35f), (12, 0.4f, 0.6f), (5, 0.8f, 1.3f) };
            var placed = new List<Vector3>();
            int idx = 0;
            foreach (var (count, minS, maxS) in bins)
            for (int k = 0; k < count; k++)
            {
                float size = Random.Range(minS, maxS);
                Vector2 p = Vector2.zero;
                bool ok = false;
                for (int tries = 0; tries < 50 && !ok; tries++)
                {
                    p = new Vector2(Random.Range(-PitHalf + 0.6f, PitHalf - 0.6f), Random.Range(-PitHalf + 0.6f, PitHalf - 0.6f));
                    ok = !(p.y > PitHalf - 3.5f && Mathf.Abs(p.x) < 2.8f) // keep the entrance clear
                         // q.z holds the other rock's size
                         && placed.All(q => Vector2.Distance(p, new Vector2(q.x, q.y)) > (q.z + size) * 0.6f + 0.1f);
                }
                if (!ok) continue;
                placed.Add(new Vector3(p.x, p.y, size));

                var prefab = prefabs[Random.Range(0, prefabs.Length)];
                var rock = (GameObject)PrefabUtility.InstantiatePrefab(prefab, rocks);
                rock.name = $"rock_{idx++:000}_{size:F2}m";
                var mf = rock.GetComponentInChildren<MeshFilter>();
                var bs = mf.sharedMesh.bounds.size;
                float meshSize = Mathf.Max(bs.x, bs.y, bs.z);
                rock.transform.localScale = Vector3.one * (size / Mathf.Max(meshSize, 1e-3f))
                    * Random.Range(0.85f, 1.15f);
                rock.transform.rotation = Quaternion.Euler(Random.Range(-15f, 15f), Random.Range(0, 360f), Random.Range(-15f, 15f));
                // sit it on the sand, sunk by ~30% of its size
                rock.transform.position = new Vector3(PitCenter.x + p.x, PitWorldY(p.x, p.y), PitCenter.z + p.y);
                var b = rock.GetComponentInChildren<Renderer>().bounds;
                rock.transform.position += Vector3.up * (PitWorldY(p.x, p.y) - b.min.y - 0.3f * b.size.y);

                foreach (var f in rock.GetComponentsInChildren<MeshFilter>())
                    if (!f.GetComponent<Collider>())
                    {
                        var mc = f.gameObject.AddComponent<MeshCollider>();
                        mc.sharedMesh = f.sharedMesh;
                        mc.sharedMaterial = groundPhysics;
                    }
                GameObjectUtility.SetStaticEditorFlags(rock, StaticEditorFlags.BatchingStatic);
            }
            Debug.Log($"[WorldBuilder] placed {placed.Count} rocks");
        }

        // ------------------------------------------------------------------ drops and slopes

        static Material concrete, hazard;

        static void Materials()
        {
            if (concrete) return;
            concrete = NewMaterial("Concrete", new Color(0.62f, 0.62f, 0.60f), GridTexture(), null, 0.15f);
            hazard = NewMaterial("HazardYellow", new Color(1f, 0.78f, 0.05f), null, null, 0.3f);
        }

        /// <summary>
        /// A block extruded along +z: profile points are (distance along lane, height above the lane's reference ground).
        /// The lane runs along world +x starting at 'start'.
        /// </summary>
        static GameObject Extrude(Transform parent, string name, Vector3 start, Vector2[] profile, float width, Material mat)
        {
            var verts = new List<Vector3>(); var uvs = new List<Vector2>(); var tris = new List<int>();
            void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 n, float s0, float s1)
            {
                int i0 = verts.Count;
                verts.AddRange(new[] { a, b, c, d });
                uvs.AddRange(new[] { new Vector2(s0, a.z), new Vector2(s1, b.z), new Vector2(s1, c.z), new Vector2(s0, d.z) });
                void T(int x, int y, int z)
                {
                    var cr = Vector3.Cross(verts[y] - verts[x], verts[z] - verts[x]);
                    if (Vector3.Dot(cr, n) < 0) { int t = y; y = z; z = t; }
                    tris.AddRange(new[] { x, y, z });
                }
                T(i0, i0 + 1, i0 + 2); T(i0, i0 + 2, i0 + 3);
            }
            float hw = width / 2;
            // sides (top/ends) along each profile edge; profile must be convex and listed counter-clockwise
            float along = 0;
            for (int k = 0; k < profile.Length; k++)
            {
                var p = profile[k]; var q = profile[(k + 1) % profile.Length];
                var n = new Vector3(q.y - p.y, -(q.x - p.x), 0).normalized;
                float next = along + Vector2.Distance(p, q);
                Quad(new Vector3(p.x, p.y, -hw), new Vector3(q.x, q.y, -hw), new Vector3(q.x, q.y, hw), new Vector3(p.x, p.y, hw), n, along, next);
                along = next;
            }
            // caps
            foreach (float z in new[] { -hw, hw })
            {
                var n = new Vector3(0, 0, Mathf.Sign(z));
                for (int k = 1; k < profile.Length - 1; k++)
                {
                    int i0 = verts.Count;
                    foreach (var p in new[] { profile[0], profile[k], profile[k + 1] }) { verts.Add(new Vector3(p.x, p.y, z)); uvs.Add(p); }
                    var cr = Vector3.Cross(verts[i0 + 1] - verts[i0], verts[i0 + 2] - verts[i0]);
                    tris.AddRange(Vector3.Dot(cr, n) >= 0 ? new[] { i0, i0 + 1, i0 + 2 } : new[] { i0, i0 + 2, i0 + 1 });
                }
            }
            var mesh = new Mesh { name = name, vertices = verts.ToArray(), uv = uvs.ToArray(), triangles = tris.ToArray() };
            mesh.RecalculateNormals(); mesh.RecalculateBounds();
            AssetDatabase.CreateAsset(mesh, $"{AssetDir}/{name.Replace(' ', '_').Replace('.', '_')}.asset");

            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.position = start;
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = mat;
            var col = go.AddComponent<MeshCollider>();
            col.sharedMesh = mesh;
            col.convex = true;
            col.sharedMaterial = groundPhysics;
            GameObjectUtility.SetStaticEditorFlags(go, StaticEditorFlags.BatchingStatic);
            return go;
        }

        static float MaxGround(float x0, float x1, float z, float hw)
        {
            float m = float.MinValue;
            for (float x = x0; x <= x1; x += 0.5f)
            for (float dz = -hw; dz <= hw; dz += 0.5f)
                m = Mathf.Max(m, Ground(x, z + dz));
            return m;
        }

        static void BuildDropCourse(Transform parent)
        {
            Materials();
            var group = new GameObject("Drop Course (0.3 / 0.6 / 1.0 m)").transform;
            group.SetParent(parent, false);
            float[] drops = { 0.3f, 0.6f, 1.0f };
            const float x0 = 58f, width = 2.6f, deck = 2.5f, rampAngle = 14f;
            for (int k = 0; k < drops.Length; k++)
            {
                float h = drops[k], z = 60f + 4.5f * k;
                float ramp = h / Mathf.Tan(rampAngle * Mathf.Deg2Rad);
                float xEdge = x0 + ramp + deck;
                // the drop is measured to the ground right after the edge
                float landing = MaxGround(xEdge, xEdge + 1.5f, z, width / 2);
                float startY = Ground(x0, z);
                float top = landing + h;
                float bottom = Mathf.Min(landing, startY) - 0.6f; // buried below the terrain
                var profile = new[]
                {
                    new Vector2(0, startY - 0.03f - startY), new Vector2(ramp, top - startY), new Vector2(ramp + deck, top - startY),
                    new Vector2(ramp + deck, bottom - startY), new Vector2(0, bottom - startY),
                };
                // reverse to counter-clockwise order (x right, y up)
                System.Array.Reverse(profile);
                var block = Extrude(group, $"Drop {h:F1} m", new Vector3(x0, startY, z), profile, width, concrete);
                // yellow strip on the lip so it shows up on camera
                Extrude(block.transform, $"Drop {h:F1} m edge", new Vector3(xEdge - 0.2f, top - 0.01f, z),
                    new[] { new Vector2(0, 0), new Vector2(0.2f, 0), new Vector2(0.2f, 0.02f), new Vector2(0, 0.02f) }, width, hazard)
                    .GetComponent<Collider>().enabled = false;
                Label(group, $"{h:F1} m DROP", new Vector3(x0 - 0.5f, startY + 1.3f, z), 90);
            }
        }

        static void BuildSlopeCourse(Transform parent)
        {
            Materials();
            var group = new GameObject("Slope Course (10 / 20 / 30 deg)").transform;
            group.SetParent(parent, false);
            float[] angles = { 10f, 20f, 30f };
            const float x0 = 36f, width = 2.6f, flat = 2f, h = 0.8f;
            for (int k = 0; k < angles.Length; k++)
            {
                float a = angles[k], z = 60f + 4.5f * k;
                float run = h / Mathf.Tan(a * Mathf.Deg2Rad);
                float len = 2 * run + flat;
                float y0 = Ground(x0, z), y1 = Ground(x0 + len, z);
                float baseY = Mathf.Min(y0, y1, MaxGround(x0, x0 + len, z, width / 2) - 0.3f);
                float top = Mathf.Max(y0, y1) + h;
                var profile = new[]
                {
                    new Vector2(0, y0 - 0.03f), new Vector2(run, top), new Vector2(run + flat, top), new Vector2(len, y1 - 0.03f),
                    new Vector2(len, baseY - 0.6f), new Vector2(0, baseY - 0.6f),
                }.Select(p => new Vector2(p.x, p.y - y0)).Reverse().ToArray();
                Extrude(group, $"Slope {a:F0} deg", new Vector3(x0, y0, z), profile, width, concrete);
                Label(group, $"{a:F0}° SLOPE", new Vector3(x0 - 0.5f, y0 + 1.3f, z), 90);
            }
        }

        // ------------------------------------------------------------------ helpers

        static Material NewMaterial(string name, Color color, Texture2D albedo, Texture2D normal, float smooth)
        {
            var m = new Material(Shader.Find("Standard")) { name = name, color = color };
            if (albedo) { m.mainTexture = albedo; m.color = Color.Lerp(color, Color.white, 0.6f); }
            if (normal) { m.SetTexture("_BumpMap", normal); m.EnableKeyword("_NORMALMAP"); }
            m.SetFloat("_Glossiness", smooth);
            AssetDatabase.CreateAsset(m, $"{AssetDir}/{name}.mat");
            return m;
        }

        /// <summary>Light concrete with a dark line every metre (1 UV unit), so cameras get texture and scale.</summary>
        static Texture2D GridTexture()
        {
            const int n = 128;
            var tex = new Texture2D(n, n, TextureFormat.RGB24, true);
            for (int y = 0; y < n; y++)
            for (int x = 0; x < n; x++)
            {
                bool line = x < 2 || y < 2 || x >= n - 2 || y >= n - 2;
                float noise = 0.06f * (Mathf.PerlinNoise(x * 0.15f, y * 0.15f) - 0.5f) + 0.03f * (Random.value - 0.5f);
                float g = line ? 0.35f : 0.92f + noise;
                tex.SetPixel(x, y, new Color(g, g, g * 0.97f));
            }
            string path = AssetDir + "/ConcreteGrid.png";
            File.WriteAllBytes(path, tex.EncodeToPNG());
            AssetDatabase.ImportAsset(path);
            var imp = (TextureImporter)AssetImporter.GetAtPath(path);
            imp.wrapMode = TextureWrapMode.Repeat;
            imp.anisoLevel = 4;
            imp.SaveAndReimport();
            return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        static void Label(Transform parent, string text, Vector3 pos, float yaw)
        {
            var go = new GameObject("Label " + text);
            go.transform.SetParent(parent, false);
            go.transform.position = pos;
            go.transform.rotation = Quaternion.Euler(0, yaw, 0);
            var tm = go.AddComponent<TextMesh>();
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            tm.font = font;
            go.GetComponent<MeshRenderer>().sharedMaterial = font.material;
            tm.text = text;
            tm.fontSize = 64;
            tm.characterSize = 0.04f;
            tm.anchor = TextAnchor.MiddleCenter;
            tm.color = new Color(1f, 0.85f, 0.1f);
        }

        /// <summary>Raycasts against the new colliders and logs measured drop heights.</summary>
        static void Verify()
        {
            Physics.SyncTransforms();
            float Hit(float x, float z) => Physics.Raycast(new Vector3(x, 50, z), Vector3.down, out var h, 100) ? h.point.y : float.NaN;
            foreach (var t in Object.FindObjectsOfType<Transform>().Where(t => t.name.StartsWith("Drop ") && t.GetComponent<Renderer>() && !t.name.EndsWith("edge")))
            {
                var b = t.GetComponent<Renderer>().bounds;
                float z = b.center.z, xEdge = b.max.x;
                Debug.Log($"[WorldBuilder] {t.name}: deck {Hit(xEdge - 0.3f, z):F2}, landing {Hit(xEdge + 0.5f, z):F2}, measured drop {Hit(xEdge - 0.3f, z) - Hit(xEdge + 0.5f, z):F2} m");
            }
            Debug.Log($"[WorldBuilder] pit: centre surface {Hit(PitCenter.x, PitCenter.z):F2} (terrain {Ground(PitCenter.x, PitCenter.z):F2}), big hill {Hit(PitCenter.x + 5, PitCenter.z - 4):F2}");
        }

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
            cam.fieldOfView = 55; cam.farClipPlane = 1000;
            var views = new (string, Vector3, Vector3)[]
            {
                ("overview", new Vector3(45, 22, 18), new Vector3(75, 0, 45)),
                ("pit", new Vector3(66, 9, 41), new Vector3(66, 0, 24)),
                ("pit_low", new Vector3(58, 2.2f, 31), new Vector3(70, 0.5f, 20)),
                ("drops", new Vector3(76, 3.5f, 56), new Vector3(63, 0.5f, 65)),
                ("slopes", new Vector3(30, 3.5f, 52), new Vector3(42, 0.5f, 65)),
                ("sky_toward_sun", new Vector3(62, 1.8f, 44), new Vector3(20, 5, 30)),
                ("sky_east", new Vector3(55, 1.8f, 45), new Vector3(100, 5, 40)),
                ("sky_north", new Vector3(60, 1.8f, 30), new Vector3(70, 5, 80)),
            };
            var rt = new RenderTexture(1600, 900, 24);
            var tex = new Texture2D(1600, 900, TextureFormat.RGB24, false);
            foreach (var (name, pos, look) in views)
            {
                cam.transform.position = pos + Vector3.up * Ground(pos.x, pos.z);
                cam.transform.LookAt(look + Vector3.up * Ground(look.x, look.z));
                cam.targetTexture = rt;
                cam.Render();
                RenderTexture.active = rt;
                tex.ReadPixels(new Rect(0, 0, 1600, 900), 0, 0);
                tex.Apply();
                File.WriteAllBytes(Path.Combine(dir, name + ".png"), tex.EncodeToPNG());
            }
            RenderTexture.active = null;
            Object.DestroyImmediate(cam.gameObject);
            Debug.Log("[WorldBuilder] screenshots in " + dir);
        }
    }
}
