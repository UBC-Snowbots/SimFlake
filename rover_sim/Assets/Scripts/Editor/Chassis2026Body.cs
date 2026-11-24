using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// chassis_2026's looks and body collider, added by <see cref="MiniRoverBuilder"/> (Def.decorate), from photos of the real rover:
    /// blue powder-coated frame, white panels on all six faces of the frame box (the internals are not modelled), three 120 mm fans
    /// on each side panel, silver rockers/bogies/differential, black carbon mast, e-stop on the top panel near the back-left.
    ///
    /// Frame box (measured from base_link.STL): x -0.6..0 (front), y +-0.305, z 0..0.21, 25.4 mm square tube. The panels sit 3 mm
    /// inside the tubes' outer faces, so the blue tubes frame every panel as on the real rover. The fans are shallow (7 mm) because
    /// the rockers run ~2 cm outside the side panels.
    ///
    /// Collision: base_link's 160 VHACD hulls (the open frame) are replaced by one box over the closed shell; nothing goes inside it.
    /// Materials and fan meshes are generated assets (Robots/Chassis/Materials, Robots/Chassis/Parts), refreshed in place on a rebuild.
    /// </summary>
    public static class Chassis2026Body
    {
        const string MatDir = "Assets/Robots/Chassis/Materials";
        const string PartDir = "Assets/Robots/Chassis/Parts";

        // Frame box (base_link, ROS FLU, metres).
        public const float X0 = -0.6f, X1 = 0f, HalfWidth = 0.305f, Z0 = 0f, Z1 = 0.21f, Tube = 0.0254f;
        const float Inset = 0.003f, Panel = 0.002f;
        /// <summary>Top surface of the top panel (where payloads stand).</summary>
        public const float TopSurface = Z1 - Inset;

        const float FanSize = 0.12f, FanDepth = 0.007f, FanHole = 0.057f;
        static readonly float[] FanX = { -0.10f, -0.33f, -0.50f };   // clear of the rocker pivot boss (x -0.262..-0.237)
        const float FanZ = 0.105f;

        static Vector3 Flu(float x, float y, float z) => new Vector3(-y, z, x);
        static Vector3 FluSize(float x, float y, float z) => new Vector3(Mathf.Abs(y), Mathf.Abs(z), Mathf.Abs(x));

        public static void Apply(string tag, GameObject robot, Transform baseLink)
        {
            EnsureFolder(MatDir); EnsureFolder(PartDir);
            var frame = Mat("Chassis2026_Frame", new Color(0.06f, 0.20f, 0.62f), 0.15f, 0.55f);      // blue powder coat
            var panel = Mat("Chassis2026_Panel", new Color(0.93f, 0.93f, 0.91f), 0f, 0.15f);         // white panels, matte (0.35 mirrored the sky)
            var alu = Mat("Chassis2026_Aluminium", new Color(0.78f, 0.79f, 0.80f), 0.85f, 0.55f);
            var carbon = Mat("Chassis2026_Carbon", new Color(0.05f, 0.05f, 0.055f), 0.3f, 0.65f);
            var fanPlastic = Mat("Chassis2026_FanPlastic", new Color(0.045f, 0.045f, 0.05f), 0f, 0.25f);
            var fanVoid = Mat("Chassis2026_FanVoid", new Color(0.01f, 0.01f, 0.01f), 0f, 0f);
            var red = Mat("Chassis2026_EStopRed", new Color(0.80f, 0.06f, 0.04f), 0f, 0.6f);

            // ---------- colours of the URDF parts ----------
            var bodies = robot.GetComponentsInChildren<ArticulationBody>(true).ToDictionary(b => b.name);
            Recolor(baseLink.GetComponent<ArticulationBody>(), frame);
            foreach (var n in new[] { "RockerL_Link", "RockerR_Link", "BogieL_Link", "BogieR_Link", "Differential_Link" })
                if (bodies.TryGetValue(n, out var b)) Recolor(b, alu);
            if (bodies.TryGetValue("PTZ_Link", out var ptz)) Recolor(ptz, carbon);

            // ---------- shell ----------
            var shell = new GameObject("Shell").transform;
            shell.SetParent(baseLink, false);
            float cx = (X0 + X1) / 2, cz = (Z0 + Z1) / 2, len = X1 - X0 - Tube, wid = 2 * HalfWidth - Tube, hgt = Z1 - Z0 - Tube;
            Box(shell, "Top Panel", Flu(cx, 0, Z1 - Inset - Panel / 2), FluSize(len, wid, Panel), panel);
            Box(shell, "Bottom Panel", Flu(cx, 0, Z0 + Inset + Panel / 2), FluSize(len, wid, Panel), panel);
            Box(shell, "Left Panel", Flu(cx, HalfWidth - Inset - Panel / 2, cz), FluSize(len, Panel, hgt), panel);
            Box(shell, "Right Panel", Flu(cx, -(HalfWidth - Inset - Panel / 2), cz), FluSize(len, Panel, hgt), panel);
            Box(shell, "Front Panel", Flu(X1 - Inset - Panel / 2, 0, cz), FluSize(Panel, wid, hgt), panel);
            Box(shell, "Back Panel", Flu(X0 + Inset + Panel / 2, 0, cz), FluSize(Panel, wid, hgt), panel);

            // ---------- fans: 3 per side, housing on the panel, blades recessed ----------
            var housing = SaveMesh("Fan120_Housing", FanHousing);
            var rotor = SaveMesh("Fan120_Rotor", FanRotor);
            var back = SaveMesh("Fan120_Back", m => Disc(m, FanHole, 0.0004f));
            foreach (int side in new[] { 1, -1 })
                for (int i = 0; i < FanX.Length; i++)
                {
                    var fan = new GameObject($"Fan {(side > 0 ? "L" : "R")}{i + 1}").transform;
                    fan.SetParent(shell, false);
                    fan.localPosition = Flu(FanX[i], side * (HalfWidth - Inset), FanZ);
                    fan.localRotation = Quaternion.LookRotation(Flu(0, side, 0), Vector3.up);   // local +z = out of the panel
                    Part(fan, "Housing", housing, fanPlastic);
                    Part(fan, "Back", back, fanVoid);
                    Part(fan, "Rotor", rotor, fanPlastic);
                }

            // ---------- e-stop (top panel, back-left, as on the real rover) ----------
            var estop = new GameObject("E-Stop").transform;
            estop.SetParent(shell, false);
            estop.localPosition = Flu(-0.47f, 0.20f, TopSurface);
            Box(estop, "Box", Flu(0, 0, 0.0175f), FluSize(0.045f, 0.045f, 0.035f), frame);
            Cylinder(estop, "Stem", Flu(0, 0, 0.040f), 0.024f, 0.010f, carbon);
            Cylinder(estop, "Button", Flu(0, 0, 0.050f), 0.040f, 0.012f, red);

            // ---------- collider: one box over the shell instead of the frame's VHACD hulls ----------
            var baseBody = baseLink.GetComponent<ArticulationBody>();
            var hulls = baseLink.GetComponentsInChildren<MeshCollider>(true).Where(c => c.GetComponentInParent<ArticulationBody>() == baseBody).ToList();
            var physMat = hulls.Select(c => c.sharedMaterial).FirstOrDefault(m => m != null);
            foreach (var c in hulls) UnityEngine.Object.DestroyImmediate(c);
            var colGo = new GameObject("Shell Collider");
            colGo.transform.SetParent(baseLink, false);
            var box = colGo.AddComponent<BoxCollider>();
            box.center = Flu(cx, 0, cz);
            box.size = FluSize(X1 - X0, 2 * HalfWidth, Z1 - Z0);
            box.sharedMaterial = physMat;
            Debug.Log($"{tag} body: shell, {2 * FanX.Length} fans, e-stop; base_link collision {hulls.Count} hulls -> 1 box {box.size}");
        }

        // ------------------------------------------------------------------ helpers

        static void Recolor(ArticulationBody body, Material m)
        {
            if (body == null) return;
            foreach (var r in body.GetComponentsInChildren<MeshRenderer>(true))
                if (r.GetComponentInParent<ArticulationBody>() == body && r.GetComponentInParent<SensorBlock>() == null)
                    r.sharedMaterials = Enumerable.Repeat(m, r.sharedMaterials.Length).ToArray();
        }

        static GameObject Box(Transform parent, string name, Vector3 pos, Vector3 size, Material m)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            UnityEngine.Object.DestroyImmediate(go.GetComponent<Collider>());
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = pos;
            go.transform.localScale = size;
            go.GetComponent<MeshRenderer>().sharedMaterial = m;
            return go;
        }

        static void Cylinder(Transform parent, string name, Vector3 pos, float diameter, float height, Material m)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);   // height 2 along y
            UnityEngine.Object.DestroyImmediate(go.GetComponent<Collider>());
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = pos;
            go.transform.localScale = new Vector3(diameter, height / 2, diameter);
            go.GetComponent<MeshRenderer>().sharedMaterial = m;
        }

        static void Part(Transform parent, string name, Mesh mesh, Material m)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = m;
        }

        static void EnsureFolder(string path)
        {
            if (AssetDatabase.IsValidFolder(path)) return;
            int i = path.LastIndexOf('/');
            AssetDatabase.CreateFolder(path.Substring(0, i), path.Substring(i + 1));
        }

        // Created once, then updated in place (same GUID), so tweaking a colour here and rebuilding just works.
        static Material Mat(string name, Color c, float metallic, float smoothness)
        {
            string path = $"{MatDir}/{name}.mat";
            var m = AssetDatabase.LoadAssetAtPath<Material>(path);
            bool create = m == null;
            if (create) m = new Material(Shader.Find("Standard"));
            m.color = c;
            m.SetFloat("_Metallic", metallic);
            m.SetFloat("_Glossiness", smoothness);
            if (create) AssetDatabase.CreateAsset(m, path); else EditorUtility.SetDirty(m);
            return m;
        }

        static Mesh SaveMesh(string name, Action<Mesh> fill)
        {
            string path = $"{PartDir}/{name}.asset";
            var m = AssetDatabase.LoadAssetAtPath<Mesh>(path);
            bool create = m == null;
            if (create) m = new Mesh { name = name }; else m.Clear();
            fill(m);
            m.RecalculateBounds();
            if (create) AssetDatabase.CreateAsset(m, path); else EditorUtility.SetDirty(m);
            return m;
        }

        // ------------------------------------------------------------------ fan meshes (local +z = out of the panel, 0 = panel surface)

        const int Seg = 32;   // multiple of 8: the square's corners fall on segment boundaries

        static Vector2 Dir(int i) { float a = 2 * Mathf.PI * i / Seg; return new Vector2(Mathf.Cos(a), Mathf.Sin(a)); }
        static Vector2 OnSquare(Vector2 d, float half) => d * (half / Mathf.Max(Mathf.Abs(d.x), Mathf.Abs(d.y)));

        /// <summary>Square frame with a round hole: front face, hole wall, four outer sides (flat-shaded).</summary>
        static void FanHousing(Mesh m)
        {
            var v = new List<Vector3>(); var n = new List<Vector3>(); var t = new List<int>();
            void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 normal)
            {
                int o = v.Count; v.AddRange(new[] { a, b, c, d }); n.AddRange(new[] { normal, normal, normal, normal });
                // Unity draws triangle (a, b, c) on the side Cross(b - a, c - a) points to: wind so that is the normal.
                if (Vector3.Dot(Vector3.Cross(b - a, c - a), normal) >= 0) t.AddRange(new[] { o, o + 1, o + 2, o, o + 2, o + 3 });
                else t.AddRange(new[] { o, o + 2, o + 1, o, o + 3, o + 2 });
            }
            float h = FanSize / 2, z = FanDepth;
            for (int i = 0; i < Seg; i++)
            {
                Vector2 d0 = Dir(i), d1 = Dir(i + 1);
                Vector2 c0 = d0 * FanHole, c1 = d1 * FanHole, s0 = OnSquare(d0, h), s1 = OnSquare(d1, h);
                Quad(new Vector3(c0.x, c0.y, z), new Vector3(s0.x, s0.y, z), new Vector3(s1.x, s1.y, z), new Vector3(c1.x, c1.y, z), Vector3.forward);
                Vector3 inward = -(Vector3)((d0 + d1) / 2).normalized;
                Quad(new Vector3(c0.x, c0.y, 0), new Vector3(c0.x, c0.y, z), new Vector3(c1.x, c1.y, z), new Vector3(c1.x, c1.y, 0), inward);
            }
            Vector2[] corner = { new Vector2(h, h), new Vector2(-h, h), new Vector2(-h, -h), new Vector2(h, -h) };
            for (int i = 0; i < 4; i++)
            {
                Vector2 a = corner[i], b = corner[(i + 1) % 4];
                Vector3 outward = ((Vector3)(a + b)).normalized;
                Quad(new Vector3(a.x, a.y, z), new Vector3(a.x, a.y, 0), new Vector3(b.x, b.y, 0), new Vector3(b.x, b.y, z), outward);
            }
            m.SetVertices(v); m.SetNormals(n); m.SetTriangles(t, 0);
        }

        /// <summary>Hub (closed disc) and seven pitched blades, double-sided, inside the housing.</summary>
        static void FanRotor(Mesh m)
        {
            var v = new List<Vector3>(); var n = new List<Vector3>(); var t = new List<int>();
            float hub = 0.021f, tip = FanHole - 0.003f, zc = FanDepth * 0.55f, pitch = 28f * Mathf.Deg2Rad;
            // hub face
            int c = v.Count; v.Add(new Vector3(0, 0, FanDepth * 0.8f)); n.Add(Vector3.forward);
            for (int i = 0; i <= Seg; i++) { var d = Dir(i) * hub; v.Add(new Vector3(d.x, d.y, FanDepth * 0.8f)); n.Add(Vector3.forward); }
            for (int i = 0; i < Seg; i++) t.AddRange(new[] { c, c + 1 + i, c + 2 + i });   // faces +z
            for (int b = 0; b < 7; b++)
            {
                float a0 = 2 * Mathf.PI * b / 7, sweep = 0.55f;   // blade spans 0.55 rad at the tip, a bit less at the root
                var q = new List<Vector3>();
                foreach (var (r, a, side) in new[] { (hub, a0, -1), (tip, a0 + 0.12f, -1), (tip, a0 + 0.12f + sweep, 1), (hub, a0 + sweep * 0.7f, 1) })
                    q.Add(new Vector3(r * Mathf.Cos(a), r * Mathf.Sin(a), zc + side * Mathf.Sin(pitch) * 0.004f));
                var normal = Vector3.Cross(q[1] - q[0], q[2] - q[0]).normalized;   // (0, 1, 2) faces this way
                foreach (var s in new[] { 1, -1 })
                {
                    int o = v.Count; v.AddRange(q); n.AddRange(Enumerable.Repeat(normal * s, 4));
                    if (s > 0) t.AddRange(new[] { o, o + 1, o + 2, o, o + 2, o + 3 }); else t.AddRange(new[] { o, o + 2, o + 1, o, o + 3, o + 2 });
                }
            }
            m.SetVertices(v); m.SetNormals(n); m.SetTriangles(t, 0);
        }

        static void Disc(Mesh m, float r, float z)
        {
            var v = new List<Vector3> { new Vector3(0, 0, z) }; var t = new List<int>();
            for (int i = 0; i <= Seg; i++) { var d = Dir(i) * r; v.Add(new Vector3(d.x, d.y, z)); }
            for (int i = 0; i < Seg; i++) t.AddRange(new[] { 0, 1 + i, 2 + i });   // faces +z
            m.SetVertices(v); m.SetNormals(Enumerable.Repeat(Vector3.forward, v.Count).ToList()); m.SetTriangles(t, 0);
        }
    }
}
