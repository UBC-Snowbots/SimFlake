using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace SimFlake.EditorTools
{
    /// <summary>
    /// Measures a tyre's tread from the wheel's visual mesh (the STL from the URDF) and stores it in the
    /// <see cref="WheelSpec"/> as a height map (<see cref="TreadData"/>): radius of the outer surface over angle x width.
    /// Also finds where the tyre sits along the axle for each wheel (the STL can include a hub/motor tube) and whether a
    /// wheel is the mirror image of the reference one.
    /// Runs as part of <see cref="DrivetrainSetup.AttachDrivetrain"/>; menu: SimFlake > Drivetrain > Bake Tread From Mesh.
    /// To try a new tread: replace the wheel STL (or point a copy of the WheelSpec at another prefab) and bake.
    /// </summary>
    public static class TreadBaker
    {
        public const int NTheta = 180, NY = 16, Super = 2;

        struct Unwrapped { public float[] y, r, th; public float yMin, yMax, rMax; }

        static Unwrapped Unwrap(ArticulationBody wheel, int samples)
        {
            Vector3 axis = (wheel.anchorRotation * Vector3.right).normalized, pivot = wheel.anchorPosition;
            WheelSoilModel.Frame(axis, out var e1, out var e2);
            var tris = new List<Vector3>();
            foreach (var mf in wheel.GetComponentsInChildren<MeshFilter>(true))
            {
                // only this wheel's own visuals (not another body's below it)
                if (mf.sharedMesh == null || mf.GetComponentInParent<ArticulationBody>() != wheel) continue;
                var v = mf.sharedMesh.vertices; var t = mf.sharedMesh.triangles;
                var m = wheel.transform.worldToLocalMatrix * mf.transform.localToWorldMatrix;
                foreach (int i in t) tris.Add(m.MultiplyPoint3x4(v[i]) - pivot);
            }
            int n = tris.Count / 3;
            if (n == 0) throw new Exception("no visual mesh under " + wheel.name);
            var area = new float[n]; float total = 0;
            for (int i = 0; i < n; i++) { area[i] = 0.5f * Vector3.Cross(tris[3 * i + 1] - tris[3 * i], tris[3 * i + 2] - tris[3 * i]).magnitude; total += area[i]; }
            var rnd = new System.Random(1);
            var ys = new List<float>(samples); var rs = new List<float>(samples); var ths = new List<float>(samples);
            var u = new Unwrapped { yMin = float.MaxValue, yMax = float.MinValue };
            for (int i = 0; i < n; i++)
            {
                int k = Mathf.Max(1, Mathf.RoundToInt(area[i] / total * samples));
                Vector3 a = tris[3 * i], ab = tris[3 * i + 1] - a, ac = tris[3 * i + 2] - a;
                for (int j = 0; j < k; j++)
                {
                    float s = (float)rnd.NextDouble(), t = (float)rnd.NextDouble();
                    if (s + t > 1) { s = 1 - s; t = 1 - t; }
                    Vector3 p = a + ab * s + ac * t;
                    float y = Vector3.Dot(p, axis);
                    Vector3 rad = p - axis * y;
                    float r = rad.magnitude;
                    ys.Add(y); rs.Add(r); ths.Add(Mathf.Atan2(Vector3.Dot(rad, e2), Vector3.Dot(rad, e1)));
                    u.yMin = Mathf.Min(u.yMin, y); u.yMax = Mathf.Max(u.yMax, y); u.rMax = Mathf.Max(u.rMax, r);
                }
            }
            u.y = ys.ToArray(); u.r = rs.ToArray(); u.th = ths.ToArray();
            return u;
        }

        /// <summary>The tyre = the stretch along the axle around the largest radius where the part is nearly that big.</summary>
        static void TyreExtent(Unwrapped u, out float y0, out float y1)
        {
            const float bin = 0.001f;
            int nb = Mathf.Max(1, Mathf.CeilToInt((u.yMax - u.yMin) / bin));
            var mx = new float[nb];
            for (int i = 0; i < u.y.Length; i++)
            {
                int b = Mathf.Clamp((int)((u.y[i] - u.yMin) / bin), 0, nb - 1);
                if (u.r[i] > mx[b]) mx[b] = u.r[i];
            }
            int peak = Array.IndexOf(mx, mx.Max());
            int lo = peak, hi = peak;
            float thr = 0.8f * u.rMax;
            while (lo > 0 && mx[lo - 1] > thr) lo--;
            while (hi < nb - 1 && mx[hi + 1] > thr) hi++;
            y0 = u.yMin + lo * bin; y1 = u.yMin + (hi + 1) * bin;
            y1 = Mathf.Min(y1, u.yMax);
        }

        public static TreadData Bake(ArticulationBody wheel, out float tyreOffset)
        {
            var u = Unwrap(wheel, 3000000);
            TyreExtent(u, out float y0, out float y1);
            int nt = NTheta * Super, ny = NY * Super;
            var fine = new float[nt * ny];
            float w = y1 - y0;
            for (int i = 0; i < u.y.Length; i++)
            {
                if (u.y[i] < y0 || u.y[i] >= y1) continue;
                int t = Mathf.Clamp((int)((u.th[i] / (2 * Mathf.PI) + 1f) % 1f * nt), 0, nt - 1);
                int y = Mathf.Clamp((int)((u.y[i] - y0) / w * ny), 0, ny - 1);
                if (u.r[i] > fine[t * ny + y]) fine[t * ny + y] = u.r[i];
            }
            // holes (no sample): take the nearest filled neighbour around the wheel
            for (int y = 0; y < ny; y++)
            for (int t = 0; t < nt; t++)
                if (fine[t * ny + y] < 0.3f * u.rMax)
                    for (int k = 1; k < nt; k++)
                    {
                        float a = fine[((t + k) % nt) * ny + y];
                        if (a >= 0.3f * u.rMax) { fine[t * ny + y] = a; break; }
                    }
            var td = new TreadData { nTheta = NTheta, nY = NY, width = w, radius = new float[NTheta * NY] };
            for (int t = 0; t < nt; t++) for (int y = 0; y < ny; y++) td.radius[(t / Super) * NY + y / Super] += fine[t * ny + y] / (Super * Super);
            td.Analyse();
            td.tipRadius = u.rMax;
            tyreOffset = 0.5f * (y0 + y1);
            return td;
        }

        /// <summary>Where this wheel's tyre sits along its axle (m from the joint).</summary>
        public static float TyreOffset(ArticulationBody wheel)
        {
            var u = Unwrap(wheel, 200000);
            TyreExtent(u, out float y0, out float y1);
            return 0.5f * (y0 + y1);
        }

        /// <summary>Bake the tread of the first wheel into the spec and set every wheel's tyre position.</summary>
        public static void BakeInto(WheelSpec spec, IList<ArticulationBody> wheels)
        {
            if (spec == null || wheels.Count == 0) return;
            var td = Bake(wheels[0], out float refOffset);
            td.source = wheels[0].GetComponentsInChildren<MeshFilter>(true).Select(m => m.sharedMesh != null ? m.sharedMesh.name : "").FirstOrDefault() + " (" + wheels[0].name + ")";
            spec.tread = td;
            EditorUtility.SetDirty(spec);
            foreach (var w in wheels)
            {
                var dw = w.GetComponent<DriveWheel>();
                if (dw == null) continue;
                dw.tyreOffset = w == wheels[0] ? refOffset : TyreOffset(w);
                dw.treadMirror = Mathf.Abs(refOffset) > 1e-3f && Mathf.Sign(dw.tyreOffset) != Mathf.Sign(refOffset);
                EditorUtility.SetDirty(dw);
            }
            Debug.Log($"[Tread] {spec.name} from {td.source}: tip radius {td.tipRadius * 1000:F1} mm, width {td.width * 1000:F1} mm, " +
                      $"lugs ~{td.lugCount} x {td.lugHeight * 1000:F1} mm, land {td.landRatio * 100:F0} %, " +
                      $"lug face per tread area: {td.flankLong:F3} rolling / {td.flankLat:F3} sideways; tyre offsets " +
                      string.Join(", ", wheels.Select(w => { var d = w.GetComponent<DriveWheel>(); return d != null ? $"{d.tyreOffset * 1000:F0}{(d.treadMirror ? "m" : "")}" : "?"; })) + " mm");
        }

        /// <summary>The tread as a picture (angle left to right, width bottom to top; bright = high), for a quick look.</summary>
        public static void SavePreview(TreadData td, string path)
        {
            if (!td.Valid) return;
            float lo = td.radius.Min(), hi = td.radius.Max();
            var tex = new Texture2D(td.nTheta, td.nY, TextureFormat.RGB24, false);
            for (int t = 0; t < td.nTheta; t++) for (int y = 0; y < td.nY; y++)
            {
                float v = Mathf.InverseLerp(lo, hi, td.radius[t * td.nY + y]);
                tex.SetPixel(t, y, new Color(v, v, v));
            }
            System.IO.File.WriteAllBytes(path, tex.EncodeToPNG());
            UnityEngine.Object.DestroyImmediate(tex);
        }
    }
}
