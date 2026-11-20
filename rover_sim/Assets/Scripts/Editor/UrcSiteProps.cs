using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Meshes made by code for the URC site world (so we own them and they fit the place): low desert shrubs built from
    /// a few cut-out cards (texture: Worlds/UrcSite/Source/Props/ShrubAtlas.png from tools/urc_site/make_assets.py) and
    /// angular bedded-sandstone boulders. Everything is seeded: the same call gives the same mesh.
    /// </summary>
    public static class UrcSiteProps
    {
        // ------------------------------------------------------------------ shrubs

        /// <summary>Cells of the 4 x 2 shrub atlas.</summary>
        public enum Card { Shadscale, Saltbush, Ephedra, Grass, DeadTwigs, Bark, HalfDead, ShadscaleDark }

        public class ShrubKind
        {
            public string name;
            public Card card, cap;          // side cards, and the flat one seen from above
            public float height;            // for a plant 1 m wide
            public float lean;              // how far the outer cards lean out (deg from vertical)
            public int star, outer;         // upright cards through the middle, leaning cards around
            public bool hasCap;
            public float weight, weightOnClay, weightSmall;     // how common: on normal ground, on bare clay, among small plants
        }

        /// <summary>
        /// The plants of the MDRS flats (Morrison / Mancos desert shrub community). Proportions from the species, not
        /// from the aerial photo: shadscale 20-60 cm rounded and spiny, mat saltbush a 10-20 cm mat on clay, Mormon tea
        /// a 50-100 cm broom of green stems, galleta / ricegrass bunches, and a share of dead or dormant shrubs.
        /// </summary>
        public static readonly ShrubKind[] Shrubs =
        {
            new ShrubKind { name = "Shadscale_A", card = Card.Shadscale, cap = Card.Shadscale, height = 0.58f, lean = 45, star = 6, outer = 7, hasCap = true, weight = 0.26f, weightOnClay = 0.05f, weightSmall = 0.15f },
            new ShrubKind { name = "Shadscale_B", card = Card.ShadscaleDark, cap = Card.ShadscaleDark, height = 0.5f, lean = 50, star = 5, outer = 8, hasCap = true, weight = 0.2f, weightOnClay = 0.05f, weightSmall = 0.12f },
            new ShrubKind { name = "Shadscale_HalfDead", card = Card.HalfDead, cap = Card.HalfDead, height = 0.55f, lean = 45, star = 6, outer = 6, hasCap = true, weight = 0.14f, weightOnClay = 0.1f, weightSmall = 0.1f },
            new ShrubKind { name = "MatSaltbush", card = Card.Saltbush, cap = Card.Saltbush, height = 0.3f, lean = 68, star = 5, outer = 8, hasCap = true, weight = 0.1f, weightOnClay = 0.6f, weightSmall = 0.2f },
            new ShrubKind { name = "MormonTea", card = Card.Ephedra, cap = Card.Ephedra, height = 0.95f, lean = 24, star = 6, outer = 6, hasCap = false, weight = 0.08f, weightOnClay = 0.0f, weightSmall = 0.03f },
            new ShrubKind { name = "BunchGrass", card = Card.Grass, cap = Card.Grass, height = 0.7f, lean = 30, star = 5, outer = 5, hasCap = false, weight = 0.12f, weightOnClay = 0.05f, weightSmall = 0.32f },
            new ShrubKind { name = "DeadShrub", card = Card.DeadTwigs, cap = Card.DeadTwigs, height = 0.55f, lean = 40, star = 6, outer = 6, hasCap = false, weight = 0.1f, weightOnClay = 0.15f, weightSmall = 0.08f },
        };

        static Rect Cell(Card c)
        {
            const float m = 0.008f;
            int k = (int)c, col = k % 4, row = k / 4;
            return Rect.MinMaxRect(col / 4f + m, 1f - (row + 1) / 2f + m, (col + 1) / 4f - m, 1f - row / 2f - m);
        }

        class Buf
        {
            public readonly List<Vector3> v = new List<Vector3>(), n = new List<Vector3>();
            public readonly List<Vector2> uv = new List<Vector2>();
            public readonly List<Color> c = new List<Color>();
            public readonly List<int> t = new List<int>();

            /// <summary>A card: bottom centre p, 'right' and 'up' unit directions, w x h, textured with an atlas cell.</summary>
            public void AddCard(Vector3 p, Vector3 right, Vector3 up, float w, float h, Rect cell, Color tint, float plantHeight, bool flip)
            {
                int i0 = v.Count;
                Vector3 a = p - right * (w / 2), b = p + right * (w / 2);
                var pts = new[] { a, b, b + up * h, a + up * h };
                float u0 = flip ? cell.xMax : cell.xMin, u1 = flip ? cell.xMin : cell.xMax;
                var uvs = new[] { new Vector2(u0, cell.yMin), new Vector2(u1, cell.yMin), new Vector2(u1, cell.yMax), new Vector2(u0, cell.yMax) };
                for (int k = 0; k < 4; k++)
                {
                    v.Add(pts[k]); uv.Add(uvs[k]);
                    // dome normal: away from a point low in the middle of the plant, so the cards shade as one mound
                    n.Add((new Vector3(pts[k].x, 0, pts[k].z) * 0.9f + Vector3.up * (0.35f + pts[k].y / Mathf.Max(plantHeight, 0.01f))).normalized);
                    // darker towards the ground (the plant shades its own base)
                    float shade = Mathf.Lerp(0.62f, 1.05f, Mathf.Clamp01(pts[k].y / Mathf.Max(plantHeight, 0.01f)));
                    c.Add(new Color(tint.r * shade, tint.g * shade, tint.b * shade, 1));
                }
                t.AddRange(new[] { i0, i0 + 2, i0 + 1, i0, i0 + 3, i0 + 2 });
            }

            public Mesh ToMesh(string name)
            {
                var m = new Mesh { name = name };
                m.SetVertices(v); m.SetNormals(n); m.SetUVs(0, uv); m.SetColors(c); m.SetTriangles(t, 0);
                m.RecalculateBounds();
                return m;
            }
        }

        /// <summary>One shrub, 1 m wide, base at the origin. ~24-30 triangles.</summary>
        public static Mesh ShrubMesh(ShrubKind k, int seed)
        {
            var rnd = new System.Random(seed);
            float R(float a, float b) => a + (b - a) * (float)rnd.NextDouble();
            var buf = new Buf();
            Rect cell = Cell(k.card);
            float h = k.height;
            // upright cards through the middle: the plant's outline from every side
            float yaw0 = R(0, 180);
            for (int i = 0; i < k.star; i++)
            {
                float yaw = (yaw0 + i * 180f / k.star + R(-8, 8)) * Mathf.Deg2Rad;
                var right = new Vector3(Mathf.Cos(yaw), 0, Mathf.Sin(yaw));
                var off = new Vector3(R(-0.05f, 0.05f), 0, R(-0.05f, 0.05f));
                float t = R(0.9f, 1.08f);
                buf.AddCard(off, right, Vector3.up, R(0.88f, 1.0f), h * R(0.85f, 1.0f), cell, new Color(t, t, t * R(0.96f, 1.02f)), h, rnd.Next(2) == 0);
            }
            // cards leaning out all around: fill the mound, and give it something to show from above
            for (int i = 0; i < k.outer; i++)
            {
                float yaw = (i * 360f / k.outer + R(-15, 15)) * Mathf.Deg2Rad;
                var outDir = new Vector3(Mathf.Cos(yaw), 0, Mathf.Sin(yaw));
                var right = new Vector3(-outDir.z, 0, outDir.x);
                float lean = (k.lean + R(-10, 10)) * Mathf.Deg2Rad;
                var up = (Vector3.up * Mathf.Cos(lean) + outDir * Mathf.Sin(lean)).normalized;
                float len = Mathf.Min(0.5f / Mathf.Max(Mathf.Sin(lean), 0.3f), h / Mathf.Max(Mathf.Cos(lean), 0.3f)) * R(0.8f, 1.0f);
                float t = R(0.88f, 1.06f);
                buf.AddCard(outDir * R(0.0f, 0.06f), right, up, R(0.55f, 0.75f), len, cell, new Color(t, t, t), h, rnd.Next(2) == 0);
            }
            if (k.hasCap)
            {
                // a flat card near the top: the crown seen from a mast camera or a drone
                float yaw = R(0, 360) * Mathf.Deg2Rad, s = R(0.6f, 0.72f);
                var right = new Vector3(Mathf.Cos(yaw), 0, Mathf.Sin(yaw)); var fwd = new Vector3(-right.z, 0, right.x);
                buf.AddCard(-fwd * (s / 2) + Vector3.up * (h * 0.62f), right, (fwd + Vector3.up * 0.05f).normalized, s, s, Cell(k.cap), Color.white, h, false);
            }
            return buf.ToMesh($"{k.name}_{seed}");
        }

        // ------------------------------------------------------------------ boulders

        static void Icosphere(int subdiv, out List<Vector3> verts, out List<int> tris)
        {
            float t = (1 + Mathf.Sqrt(5)) / 2;
            verts = new[]
            {
                new Vector3(-1, t, 0), new Vector3(1, t, 0), new Vector3(-1, -t, 0), new Vector3(1, -t, 0), new Vector3(0, -1, t), new Vector3(0, 1, t),
                new Vector3(0, -1, -t), new Vector3(0, 1, -t), new Vector3(t, 0, -1), new Vector3(t, 0, 1), new Vector3(-t, 0, -1), new Vector3(-t, 0, 1),
            }.Select(p => p.normalized).ToList();
            tris = new List<int> { 0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11, 1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
                                   3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9, 4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1 };
            for (int s = 0; s < subdiv; s++)
            {
                var cache = new Dictionary<long, int>();
                var vs = verts;
                int Mid(int a, int b)
                {
                    long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                    if (cache.TryGetValue(key, out int i)) return i;
                    vs.Add(((vs[a] + vs[b]) * 0.5f).normalized);
                    return cache[key] = vs.Count - 1;
                }
                var nt = new List<int>(tris.Count * 4);
                for (int i = 0; i < tris.Count; i += 3)
                {
                    int a = tris[i], b = tris[i + 1], c = tris[i + 2], ab = Mid(a, b), bc = Mid(b, c), ca = Mid(c, a);
                    nt.AddRange(new[] { a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca });
                }
                tris = nt;
            }
        }

        /// <summary>
        /// A sandstone block, 1 m across (largest horizontal size), centred on the origin. Blocks break off bedded caprock:
        /// flat top and bottom along the bedding, a few fracture faces around, worn corners. 'slab' = height / width.
        /// subdiv 2 = 320 triangles (boulders), 1 = 80 (stones).
        /// </summary>
        public static Mesh BoulderMesh(int seed, int subdiv, float slab)
        {
            var rnd = new System.Random(seed);
            float R(float a, float b) => a + (b - a) * (float)rnd.NextDouble();
            Icosphere(subdiv, out var verts, out var tris);
            // between a box and a ball: flat-ish faces, worn corners
            float round = R(0.3f, 0.55f);
            var scale = new Vector3(1, slab, R(0.65f, 1f));
            for (int i = 0; i < verts.Count; i++)
            {
                var p = verts[i];
                var box = p / Mathf.Max(Mathf.Abs(p.x), Mathf.Max(Mathf.Abs(p.y), Mathf.Abs(p.z)));
                verts[i] = Vector3.Scale(Vector3.Lerp(box, p * 1.2f, round) * 0.5f, scale);
            }
            // fracture planes: bedding (top, bottom) and 4-7 others. Vertices beyond a plane are pushed back onto it.
            var planes = new List<(Vector3 n, float d)>
            {
                ((Vector3.up + new Vector3(R(-0.08f, 0.08f), 0, R(-0.08f, 0.08f))).normalized, 0.5f * slab * R(0.72f, 0.9f)),
                ((Vector3.down + new Vector3(R(-0.08f, 0.08f), 0, R(-0.08f, 0.08f))).normalized, 0.5f * slab * R(0.72f, 0.9f)),
            };
            int extra = rnd.Next(4, 8);
            for (int k = 0; k < extra; k++)
            {
                float yaw = R(0, Mathf.PI * 2), pitch = R(-0.5f, 0.5f);
                var n = new Vector3(Mathf.Cos(yaw) * Mathf.Cos(pitch), Mathf.Sin(pitch), Mathf.Sin(yaw) * Mathf.Cos(pitch));
                float reach = verts.Max(p => Vector3.Dot(p, n));
                planes.Add((n, reach * R(0.62f, 0.9f)));
            }
            for (int i = 0; i < verts.Count; i++)
            {
                var p = verts[i];
                foreach (var (n, d) in planes) { float over = Vector3.Dot(p, n) - d; if (over > 0) p -= n * over; }
                // weathering: a little roughness everywhere
                p += p.normalized * 0.018f * (Mathf.PerlinNoise(p.x * 7f + seed, p.z * 7f + p.y * 5f) - 0.5f) * 2f;
                verts[i] = p;
            }
            float width = Mathf.Max(verts.Max(p => p.x) - verts.Min(p => p.x), verts.Max(p => p.z) - verts.Min(p => p.z));
            for (int i = 0; i < verts.Count; i++) verts[i] /= width;

            // face normals; a vertex is smooth across faces within 32 deg of each other, sharp across fracture edges
            int nTri = tris.Count / 3;
            var fn = new Vector3[nTri];
            var touching = new List<int>[verts.Count];
            for (int i = 0; i < verts.Count; i++) touching[i] = new List<int>();
            for (int f = 0; f < nTri; f++)
            {
                Vector3 a = verts[tris[f * 3]], b = verts[tris[f * 3 + 1]], c = verts[tris[f * 3 + 2]];
                fn[f] = Vector3.Cross(b - a, c - a).normalized;
                for (int k = 0; k < 3; k++) touching[tris[f * 3 + k]].Add(f);
            }
            var v = new Vector3[nTri * 3]; var nrm = new Vector3[nTri * 3]; var uv = new Vector2[nTri * 3]; var idx = new int[nTri * 3];
            float cos = Mathf.Cos(32 * Mathf.Deg2Rad);
            Vector2 uvOff = new Vector2(R(0, 1), R(0, 1));
            for (int f = 0; f < nTri; f++)
            {
                var n = fn[f];
                float ax = Mathf.Abs(n.x), ay = Mathf.Abs(n.y), az = Mathf.Abs(n.z);
                for (int k = 0; k < 3; k++)
                {
                    int vi = tris[f * 3 + k], o = f * 3 + k;
                    var p = verts[vi];
                    v[o] = p; idx[o] = o;
                    Vector3 sum = Vector3.zero;
                    foreach (int g in touching[vi]) if (Vector3.Dot(fn[g], n) > cos) sum += fn[g];
                    nrm[o] = sum.normalized;
                    // box mapping; on the sides v = height, so the texture's bedding lines lie flat like real strata
                    uv[o] = (ay >= ax && ay >= az ? new Vector2(p.x, p.z) : ax >= az ? new Vector2(p.z, p.y) : new Vector2(p.x, p.y)) * 0.8f + uvOff;
                }
            }
            var m = new Mesh { name = $"Boulder_{seed}" };
            m.vertices = v; m.normals = nrm; m.uv = uv; m.triangles = idx;
            m.RecalculateTangents(); m.RecalculateBounds();
            return m;
        }
    }
}
