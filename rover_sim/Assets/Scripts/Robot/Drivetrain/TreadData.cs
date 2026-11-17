using System;
using UnityEngine;

/// <summary>
/// A tyre's tread as a height map: radius (m) of the outer surface on a grid of angle around the axle x position across
/// the width. Baked from the wheel's mesh (STL) by <c>TreadBaker</c>, stored in the <see cref="WheelSpec"/>.
/// <see cref="WheelSoilModel"/> turns it into contact elements: lug tops and the voids between them, each with its own
/// radius, plus how much lug face there is to push soil forwards/sideways. So a new tread design = bake a new STL.
/// </summary>
[Serializable]
public class TreadData
{
    public int nTheta, nY;
    [Tooltip("m, tread width")]
    public float width;
    [Tooltip("m, max radius (lug tips)")]
    public float tipRadius;
    [Tooltip("m, typical lug height")]
    public float lugHeight;
    [Tooltip("0..1, part of the tread surface that is lug top")]
    public float landRatio;
    [Tooltip("Lugs around the circumference (estimate from the map)")]
    public int lugCount;
    [Tooltip("m² of lug face that pushes soil along the rolling direction, per m² of tread")]
    public float flankLong;
    [Tooltip("m² of lug face that pushes soil sideways, per m² of tread")]
    public float flankLat;
    public string source;
    [HideInInspector] public float[] radius;   // [theta * nY + y], row = one angle

    public bool Valid => radius != null && nTheta > 0 && nY > 0 && radius.Length == nTheta * nY;

    public static TreadData Smooth(float r, float w)
    {
        var t = new TreadData { nTheta = 60, nY = 6, width = w, tipRadius = r, source = "smooth cylinder" };
        t.radius = new float[t.nTheta * t.nY];
        for (int i = 0; i < t.radius.Length; i++) t.radius[i] = r;
        return t;
    }

    /// <summary>Carcass radius (surface between lugs) for each position across the width: a low percentile around the wheel.</summary>
    public float[] Carcass()
    {
        var c = new float[nY];
        var col = new float[nTheta];
        for (int y = 0; y < nY; y++)
        {
            for (int t = 0; t < nTheta; t++) col[t] = radius[t * nY + y];
            Array.Sort(col);
            c[y] = col[Mathf.Clamp(nTheta / 5, 0, nTheta - 1)];
        }
        return c;
    }

    /// <summary>Fill in the summary numbers from the map.</summary>
    public void Analyse()
    {
        if (!Valid) return;
        var c = Carcass();
        float tip = 0;
        foreach (float r in radius) tip = Mathf.Max(tip, r);
        tipRadius = tip;
        // lug height: high percentile of (r - carcass)
        var d = new float[radius.Length];
        for (int t = 0; t < nTheta; t++) for (int y = 0; y < nY; y++) d[t * nY + y] = radius[t * nY + y] - c[y];
        var s = (float[])d.Clone(); Array.Sort(s);
        lugHeight = Mathf.Max(0, s[(int)(s.Length * 0.97f)]);
        float thr = Mathf.Max(lugHeight * 0.5f, 2e-4f);
        int land = 0;
        foreach (float v in d) if (v > thr) land++;
        landRatio = lugHeight > 3e-4f ? (float)land / d.Length : 1f;
        float dTheta = 2 * Mathf.PI / nTheta, dy = width / nY, fl = 0, fa = 0, area = 0;
        int edges = 0;
        for (int t = 0; t < nTheta; t++)
        for (int y = 0; y < nY; y++)
        {
            float r = radius[t * nY + y];
            area += r * dTheta * dy;
            fl += Mathf.Abs(radius[((t + 1) % nTheta) * nY + y] - r) * dy;
            if (y + 1 < nY) fa += Mathf.Abs(radius[t * nY + y + 1] - r - (c[y + 1] - c[y])) * r * dTheta;
            if (y == nY / 4 && d[t * nY + y] <= thr && d[((t + 1) % nTheta) * nY + y] > thr) edges++;
        }
        // each lug has a leading and a trailing face; only one pushes at a time
        flankLong = lugHeight > 3e-4f ? 0.5f * fl / area : 0;
        flankLat = lugHeight > 3e-4f ? 0.5f * fa / area : 0;
        lugCount = edges;
    }
}
