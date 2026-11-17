using System;
using UnityEngine;

/// <summary>
/// Contact between one tyre and deformable ground: the terramechanics model.
///
/// The tread (from <see cref="TreadData"/>, i.e. the STL) is cut into elements that turn with the wheel. Each element
/// has a lug-top part and a void part (the surface between lugs) at their own radii. Every physics step, for each
/// element under the soil surface:
///   - normal pressure: the tyre rubber (a spring layer, stiffness from Shore hardness and thickness) pushes on the soil,
///     and the soil gives way following Bekker's pressure-sinkage law once the pressure passes what that spot has
///     carried before. So loose soil ruts, pressed soil is firm (a wheel in another wheel's rut sinks less), and on
///     hard ground only the lug tops touch and the rubber does the giving;
///   - shear: the element remembers how far it has slipped since it touched down (Janosi-Hanamoto shear
///     displacement). Strength is rubber friction on what the lug tops touch, limited by the soil's own strength
///     (Mohr-Coulomb), plus soil pushed by buried lug faces (passive earth pressure): that is where the tread pattern
///     and its orientation come in. Slip in any direction: one friction ellipse for traction + side slip;
///   - pressure acts along the wheel's radius, so the front of the contact pushes back: rolling resistance from
///     sinkage needs no coefficient, it comes out of the geometry.
/// The soil grid is then updated: pressed down under the tread, part of that volume piled beside the rut, soil dug
/// out and thrown back by slipping lugs (the wheel digs itself in), slopes slumped to the angle of repose.
///
/// Numerics: forces are explicit, and real rubber on rock is far too stiff for a 5 ms step. So the tread hangs on a
/// spring in series with the contact, as stiff as the time step allows (<see cref="Settings.stabilityFactor"/> x wheel
/// mass / dt²): the hub sits ~1-2 mm lower than it would (think of it as the flex of the wheel hub and suspension
/// arms), while pressures, patch size, sinkage and which parts of the tread touch stay physical.
/// Plain C#, no Unity physics: also used offline by the wheel test rig.
/// </summary>
public class WheelSoilModel
{
    [Serializable]
    public struct Settings
    {
        public int nTheta, nY;            // contact elements around / across the tyre
        public float stabilityFactor;     // contact stiffness limit = factor x mass / dt²
        public bool deform;               // ruts on/off
        public bool sideForces;           // soil pushed by the wheel's sides when it slides sideways in a rut
        public int relaxPasses;           // angle-of-repose passes per step
    }

    public struct Pose
    {
        public Vector3 hub, axis;         // world
        public Vector3 e1, e2;            // world directions of the wheel-fixed frame (turn with the wheel), e2 = axis x e1
        public Vector3 hubVelocity, angularVelocity;
        public float spin;                // rad/s about the axle relative to the carrier
        public float stabilityMass;       // kg
    }

    public struct Result
    {
        public bool handled;              // over soil: this model owns the contact (even when in the air)
        public bool grounded;
        public Vector3 force, torque;     // on the wheel, torque about the hub
        public Vector3 point, normal;
        public float load;                // N, normal
        public float sinkage;             // m, how far the contact sits below the undisturbed surface
        public float deflection;          // m, tyre rubber squash (physical, from the load)
        public float patchLength, patchWidth;   // m
        public float pressure;            // Pa, mean over the contact
        public float traction;            // N along the rolling direction (+ = pushing the rover forwards)
        public float tractionLimit;       // N the contact could give
        public float stiffnessScale;      // force correction this step (1 in steady contact)
        public float spring;              // m, deflection of the series spring (numerical give, see class comment)
        public float displaced;           // m³ of soil moved this step
        public int elements;              // in contact
        public GroundSpec ground;
    }

    public float axialCentre;             // m along the axle from the hub to the middle of the tread
    public bool mirror;
    /// <summary>Scales all shear strength (1 = normal). DriveWheel lowers it while the rover settles at start.</summary>
    public float gripFactor = 1f;

    WheelSpec spec;
    TreadData tread;
    Settings s;
    int nT, nY;
    float dTheta, dY, width, rMax, lugHeight;
    float[] rTop, rVoid, land, flankL, flankA, jx, jy, sT, sV, syT, syV, cosT, sinT;
    float rowMargin;
    int[] stamp, stT, stV;
    int step;
    float kTop, kVoid, lastKt, lastKn, lastCap, spring, lastScale = 1f, lugWidth;
    Vector3 lastNormal = Vector3.up;
    readonly SoilField.Patch patch = new SoilField.Patch();
    int[] cells = new int[256];
    int nCells;

    public float Radius => rMax;
    public float Width => width;
    public TreadData Tread => tread;

    /// <summary>Wheel-fixed reference directions for a given axle (same rule in the tread baker).</summary>
    public static void Frame(Vector3 axis, out Vector3 e1, out Vector3 e2)
    {
        e1 = Vector3.Cross(axis, Mathf.Abs(axis.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
        e2 = Vector3.Cross(axis, e1);
    }

    public void Configure(WheelSpec wheel, float fallbackRadius, float fallbackWidth, Settings settings)
    {
        spec = wheel; s = settings;
        tread = wheel != null && wheel.tread != null && wheel.tread.Valid ? wheel.tread : TreadData.Smooth(fallbackRadius, Mathf.Max(fallbackWidth, 0.01f));
        nT = Mathf.Clamp(s.nTheta, 12, tread.nTheta); nY = Mathf.Clamp(s.nY, 1, tread.nY);
        dTheta = 2 * Mathf.PI / nT; width = tread.width; dY = width / nY;
        int n = nT * nY;
        rTop = new float[n]; rVoid = new float[n]; land = new float[n]; flankL = new float[n]; flankA = new float[n];
        jx = new float[n]; jy = new float[n]; stamp = new int[n];
        sT = new float[n]; sV = new float[n]; syT = new float[n]; syV = new float[n]; stT = new int[n]; stV = new int[n];
        var nTop = new int[n]; var nVoid = new int[n];
        var carc = tread.Carcass();
        lugHeight = tread.lugHeight;
        bool lugs = lugHeight > 3e-4f;
        float thr = Mathf.Max(lugHeight * 0.5f, 2e-4f);
        float ndT = 2 * Mathf.PI / tread.nTheta, ndY = width / tread.nY;
        rMax = 0;
        for (int t = 0; t < tread.nTheta; t++)
        for (int y = 0; y < tread.nY; y++)
        {
            int e = (t * nT / tread.nTheta) * nY + (y * nY / tread.nY);
            float r = tread.radius[t * tread.nY + y];
            rMax = Mathf.Max(rMax, r);
            if (!lugs || r - carc[y] > thr) { rTop[e] += r; nTop[e]++; } else { rVoid[e] += r; nVoid[e]++; }
            if (lugs)
            {
                flankL[e] += 0.5f * Mathf.Abs(tread.radius[((t + 1) % tread.nTheta) * tread.nY + y] - r) * ndY;
                if (y + 1 < tread.nY) flankA[e] += 0.5f * Mathf.Abs(tread.radius[t * tread.nY + y + 1] - r - (carc[y + 1] - carc[y])) * r * ndT;
            }
        }
        for (int e = 0; e < n; e++)
        {
            if (nTop[e] > 0) rTop[e] /= nTop[e];
            if (nVoid[e] > 0) rVoid[e] /= nVoid[e];
            if (nTop[e] == 0) rTop[e] = rVoid[e];
            if (nVoid[e] == 0) rVoid[e] = rTop[e];
            land[e] = (float)nTop[e] / Mathf.Max(1, nTop[e] + nVoid[e]);
        }
        cosT = new float[nT]; sinT = new float[nT];
        for (int t = 0; t < nT; t++) { cosT[t] = MathF.Cos((t + 0.5f) * dTheta); sinT[t] = MathF.Sin((t + 0.5f) * dTheta); }
        float rLow = rMax;
        for (int e = 0; e < n; e++) rLow = Mathf.Min(rLow, Mathf.Min(rVoid[e], rTop[e]));
        rowMargin = rMax - rLow + 0.002f;
        float E = wheel != null ? wheel.YoungsModulus : 1.7e6f;
        float h = wheel != null ? Mathf.Max(wheel.rubberThickness, 1e-3f) : 0.008f;
        kVoid = 2f * E / h;
        kTop = 1f / (lugHeight / (2.5f * E) + h / (2f * E));
        // a lug is a small plate: narrow plates sink less in cohesive soil (the kc / b term)
        lugWidth = lugs && tread.lugCount > 0 ? Mathf.Clamp(tread.landRatio * 2 * Mathf.PI * rMax / tread.lugCount, 0.004f, width) : width;
        spring = 0; lastKn = 0; lastScale = 1;
    }

    /// <summary>Forget slip history and stiffness adaptation (teleport/reset).</summary>
    public void Reset()
    {
        Array.Clear(stamp, 0, stamp.Length); Array.Clear(stT, 0, stT.Length); Array.Clear(stV, 0, stV.Length);
        lastKt = lastKn = spring = 0; lastScale = 1; lastNormal = Vector3.up;
    }

    float Grip(float mu, float p) =>
        spec == null || spec.gripPressureExponent == 0 || p <= 0 ? mu : mu * Mathf.Clamp(MathF.Pow(p / 1e5f, spec.gripPressureExponent), 0.6f, 1.5f);

    // One part (lug top or void) of an element against the soil under it. Pure: used by the trial pass and the real one.
    // s = soil level under this part, sy = pressure that soil has carried, py = height of the tread surface.
    // Returns pressure; dz = how far the soil gives way this step; ktan = dp/d(overlap).
    static float Press(float s, float sy, float py, float kr, bool rigid, float ks, float en, float invN, out float dz, out float ktan)
    {
        dz = 0; ktan = 0;
        float dv = s - py;
        if (dv <= 0) return 0;
        float p = kr * dv;
        ktan = kr;
        if (rigid || p <= sy) return p;
        float zeq = sy > 0 ? MathF.Pow(sy / ks, invN) : 0;
        float zl = Mathf.Max(zeq + 0.5f * dv, 1e-5f);
        float ksl = ks * en * MathF.Pow(zl, en - 1f);
        dz = Mathf.Clamp((kr * dv - ks * MathF.Pow(zl, en) + ksl * 0.5f * dv) / (kr + ksl), 0, dv);
        for (int i = 0; i < 3; i++)
        {
            float z = Mathf.Max(zeq + dz, 1e-6f);
            float pz1 = MathF.Pow(z, en - 1f);
            float fn = ks * pz1 * z - kr * (dv - dz), df = ks * en * pz1 + kr;
            dz = Mathf.Clamp(dz - fn / df, 0, dv);
            ksl = ks * en * pz1;
        }
        ktan = kr * ksl / (kr + ksl);
        return kr * (dv - dz);
    }

    struct Ctx
    {
        public Vector3 hub, axis, e1, e2;
        public float ksT, ksV, en, invN, invC, ox, oz;
        public bool rigid;
        public int nx, nz;
    }

    // Can any element of this row (one angle around the tyre) be under the soil? Three cheap nearest-cell lookups.
    bool RowTouches(in Ctx k, float dx, float dyv, float dzv, float cell)
    {
        float[] ph = patch.h;
        float margin = rowMargin + 0.5f * cell;
        float sign = mirror ? -1f : 1f;
        for (int i = -1; i <= 1; i++)
        {
            float yoff = axialCentre + sign * i * 0.45f * width;
            float px = k.hub.x + k.axis.x * yoff + dx * rMax, py = k.hub.y + k.axis.y * yoff + dyv * rMax, pz = k.hub.z + k.axis.z * yoff + dzv * rMax;
            int ci = (int)(px * k.invC - k.ox), cj = (int)(pz * k.invC - k.oz);
            if (ci < 0 || cj < 0 || ci >= k.nx || cj >= k.nz) continue;
            if (py - margin < ph[cj * k.nx + ci]) return true;
        }
        return false;
    }

    // Normal load and stiffness for a given spring deflection, without changing anything (to find the deflection).
    void Trial(in Ctx k, out float N, out float Kn)
    {
        N = 0; Kn = 0;
        float[] ph = patch.h, psy = patch.sy;
        float sign = mirror ? -1f : 1f;
        for (int it = 0; it < nT; it++)
        {
            float cs = cosT[it], sn = sinT[it];
            float dx = k.e1.x * cs + k.e2.x * sn, dyv = k.e1.y * cs + k.e2.y * sn, dzv = k.e1.z * cs + k.e2.z * sn;
            float cb = -dyv;
            if (cb < 0.2f) continue;
            if (!RowTouches(k, dx, dyv, dzv, 1f / k.invC)) continue;
            for (int iy = 0; iy < nY; iy++)
            {
                int e = it * nY + iy;
                float yoff = axialCentre + sign * ((iy + 0.5f) / nY - 0.5f) * width;
                float bx = k.hub.x + k.axis.x * yoff, by = k.hub.y + k.axis.y * yoff, bz = k.hub.z + k.axis.z * yoff;
                float aE = rTop[e] * dTheta * dY;
                for (int sub = 0; sub < 2; sub++)
                {
                    float frac = sub == 0 ? land[e] : 1f - land[e];
                    if (frac < 0.02f) continue;
                    float r = sub == 0 ? rTop[e] : rVoid[e];
                    float py = by + dyv * r;
                    float s, sy;
                    if ((sub == 0 ? stT[e] : stV[e]) == step - 1 && !k.rigid) { s = sub == 0 ? sT[e] : sV[e]; sy = sub == 0 ? syT[e] : syV[e]; }
                    else
                    {
                        float fx = (bx + dx * r) * k.invC - 0.5f - k.ox, fz = (bz + dzv * r) * k.invC - 0.5f - k.oz;
                        int i0 = (int)fx, j0 = (int)fz;
                        if (fx < 0 || fz < 0 || i0 >= k.nx - 1 || j0 >= k.nz - 1) continue;
                        float tx = fx - i0, tz = fz - j0;
                        int k00 = j0 * k.nx + i0;
                        float h0 = ph[k00] + (ph[k00 + 1] - ph[k00]) * tx, h1 = ph[k00 + k.nx] + (ph[k00 + k.nx + 1] - ph[k00 + k.nx]) * tx;
                        s = h0 + (h1 - h0) * tz;
                        sy = psy[(tz > 0.5f ? k00 + k.nx : k00) + (tx > 0.5f ? 1 : 0)];
                    }
                    float p = Press(s, sy, py, (sub == 0 ? kTop : kVoid) / cb, k.rigid, sub == 0 ? k.ksT : k.ksV, k.en, k.invN, out _, out float kt);
                    if (p <= 0) continue;
                    N += p * aE * frac; Kn += kt * aE * frac * cb;
                }
            }
        }
    }

    // Three phases so several wheels can be computed at once: Begin and End touch the shared grid (main thread, one
    // wheel after the other), Compute only touches this wheel's own copy of the ground under it.
    Pose q;
    Result res;
    SoilField field;
    GroundSpec g;
    bool active;
    Vector3 cc;

    /// <summary>All three phases in one call (single wheel, e.g. the test rig).</summary>
    public Result Step(SoilField f, ISoilBase soil, in Pose pose, float dt)
    {
        if (Begin(f, soil, pose)) Compute(dt);
        return End();
    }

    /// <summary>Copy the ground under the wheel. False = nothing to compute (in the air, or not over soil).</summary>
    public bool Begin(SoilField f, ISoilBase soil, in Pose pose)
    {
        q = pose; field = f; active = false;
        res = new Result { stiffnessScale = 1 };
        step++;
        if (field == null || soil == null) return false;
        cc = q.hub + q.axis * axialCentre;
        float c = field.cell;
        float baseH = soil.BaseHeight(cc.x, cc.z);
        res.handled = true;
        if (cc.y - rMax > baseH + 0.25f) { spring = 0; lastKn = 0; return false; }           // well above the ground
        float ext = rMax * 0.95f + width * 0.5f + 4f * c;
        int ix0 = Mathf.FloorToInt((cc.x - ext) / c), iz0 = Mathf.FloorToInt((cc.z - ext) / c);
        int nx = Mathf.CeilToInt(2 * ext / c) + 1;
        if (!field.Fetch(patch, ix0, iz0, nx, nx)) { res.handled = false; return false; }
        if (cc.y - rMax - Mathf.Abs(q.axis.y) * width * 0.5f > patch.maxH) { spring = 0; lastKn = 0; return false; }
        g = soil.SpecAt(cc.x, cc.z);
        res.ground = g;
        if (g == null) { res.handled = false; return false; }
        active = true;
        return true;
    }

    /// <summary>Write the changed ground back and return the result.</summary>
    public Result End()
    {
        if (active) field.Store(patch);
        active = false;
        return res;
    }

    public void Compute(float dt)
    {
        // never let one bad step (a non-finite pose or state) poison the contact for good
        if (float.IsNaN(lastNormal.x) || float.IsNaN(spring) || float.IsInfinity(spring) || float.IsNaN(q.hubVelocity.x + q.hub.x + q.angularVelocity.x)
            || q.hubVelocity.sqrMagnitude > 1e4f)
        {
            Reset();
            if (float.IsNaN(q.hubVelocity.x + q.hub.x + q.angularVelocity.x) || q.hubVelocity.sqrMagnitude > 1e4f) return;
        }
        Vector3 axis = q.axis;
        float c = field.cell;
        int ix0 = patch.ix0, iz0 = patch.iz0, nx = patch.nx, nz = patch.nz;
        bool rigid = !g.deformable;
        bool deform = s.deform && !rigid;
        float en = Mathf.Max(g.n, 0.3f), invN = 1f / en;
        float coh = g.cohesion * 1000f, tanPhi = g.TanPhi, Kp = g.Kp, sqrtKp = MathF.Sqrt(Kp);
        // Shear give: the ground's, but never so stiff that the explicit shear spring rings at this time step
        // (matters for heavy wheels on rock: 60 N on 2 mm of tread shear is at the limit for a 0.9 kg wheel at 5 ms).
        float K = Mathf.Max(Mathf.Max(g.shearModulus, 2e-4f), lastCap * dt * dt / (0.36f * q.stabilityMass));
        float mu = g.rubberFriction * (spec != null ? spec.gripScale : 1f);
        float ox = ix0, oz = iz0, invC = 1f / c, cellArea = c * c;
        float[] ph = patch.h, psy = patch.sy, pmark = patch.mark, pb = patch.b;
        float tauD = lastKt > 0 ? Mathf.Clamp(0.6f * MathF.Sqrt(q.stabilityMass * 4f / lastKt), dt, 0.03f) : 2 * dt;   // bristle damping time
        float sign = mirror ? -1f : 1f;

        // Series spring (see class comment): the tread sits `spring` metres further from the ground than the hub says.
        float Kc = s.stabilityFactor * q.stabilityMass / (dt * dt);
        if (lastKn > 0) spring = Mathf.Max(0, spring - Vector3.Dot(q.hubVelocity, lastNormal) * dt * lastKn / (Kc + lastKn));
        else spring = 0;
        var ctx = new Ctx
        {
            hub = q.hub + lastNormal * spring, axis = axis, e1 = q.e1, e2 = q.e2, ksT = g.Stiffness(lugWidth), ksV = g.Stiffness(width),
            en = en, invN = invN, invC = invC, ox = ox, oz = oz, rigid = rigid, nx = nx, nz = nz,
        };
        if (lastKn <= 0 || lastScale < 0.75f || lastScale > 1.3f)
            for (int itn = 0; itn < 3; itn++)
            {
                Trial(ctx, out float n0, out float k0);
                if (n0 <= 0) break;
                float target = (n0 + k0 * spring) / (Kc + k0);
                bool done = Mathf.Abs(target - spring) < 0.1f * target;
                spring = target;
                ctx.hub = q.hub + lastNormal * spring;
                if (done) break;
            }
        Vector3 hub = ctx.hub;
        float ccy = cc.y + lastNormal.y * spring;

        Vector3 up = Vector3.up;
        Vector3 axisH = Vector3.ProjectOnPlane(axis, up).normalized;
        Vector3 fwdH = Vector3.Cross(axisH, up);
        float axialH = axialCentre * Vector3.Dot(axis, axisH), halfWidthH = 0.5f * width * Mathf.Abs(Vector3.Dot(axis, axisH)) + 0.25f * c;

        // slope of the undisturbed ground under the wheel (rise per metre along x and z)
        int last = (nz - 1) * nx;
        float gradX = (pb[nx - 1] + pb[last + nx - 1] - pb[0] - pb[last]) / (2f * (nx - 1) * c);
        float gradZ = (pb[last] + pb[last + nx - 1] - pb[0] - pb[nx - 1]) / (2f * (nz - 1) * c);

        // direction the wheel is travelling over the ground, and how much the "nothing solid behind" rule applies
        Vector3 travel = Vector3.ProjectOnPlane(q.hubVelocity, lastNormal);
        float speedH = travel.magnitude;
        travel = speedH > 1e-4f ? travel / speedH : Vector3.zero;
        float rearCap = rigid ? 0f : Mathf.Clamp01(speedH / 0.03f);
        float pPassive = 2f * coh * sqrtKp + g.density * 9.81f * 0.03f * Kp;

        Vector3 F = Vector3.zero, T = Vector3.zero, nSum = Vector3.zero, pSum = Vector3.zero, slipSum = Vector3.zero;
        float N = 0, Kn = 0, Kt = 0, area = 0, capSum = 0, vol = 0, sink = 0, voidLoad = 0;
        float thMin = float.MaxValue, thMax = float.MinValue; int yMin = int.MaxValue, yMax = -1, count = 0;
        float sideA0 = 0, sideA1 = 0, sideZ0 = 0, sideZ1 = 0;
        nCells = 0;
        // angle of the wheel-fixed frame's "down" so the contact length can be measured without wrap-around
        float thDown = Mathf.Atan2(-q.e2.y, -q.e1.y);

        for (int it = 0; it < nT; it++)
        {
            float th = (it + 0.5f) * dTheta;
            float cs = cosT[it], sn = sinT[it];
            Vector3 dir = new Vector3(q.e1.x * cs + q.e2.x * sn, q.e1.y * cs + q.e2.y * sn, q.e1.z * cs + q.e2.z * sn);
            float cb = -dir.y;
            if (cb < 0.2f) continue;
            if (!RowTouches(ctx, dir.x, dir.y, dir.z, c)) continue;
            Vector3 tvec = Vector3.Cross(axis, dir);
            float rel = Mathf.DeltaAngle(thDown * Mathf.Rad2Deg, th * Mathf.Rad2Deg) * Mathf.Deg2Rad;
            for (int iy = 0; iy < nY; iy++)
            {
                int e = it * nY + iy;
                float yoff = axialCentre + sign * ((iy + 0.5f) / nY - 0.5f) * width;
                Vector3 bp = new Vector3(hub.x + axis.x * yoff, hub.y + axis.y * yoff, hub.z + axis.z * yoff);
                float ld = land[e], rr = rTop[e];
                Vector3 arm = new Vector3(axis.x * yoff + dir.x * rr, axis.y * yoff + dir.y * rr, axis.z * yoff + dir.z * rr);
                Vector3 v = q.hubVelocity + Vector3.Cross(q.angularVelocity, arm);
                float vt = Vector3.Dot(v, tvec), va = Vector3.Dot(v, axis);
                float slid = MathF.Sqrt(vt * vt + va * va) * dt;

                float pT = 0, pV = 0, dzT = 0, aT = 0, aV = 0;
                float aE = rr * dTheta * dY;
                float levelT = 0, levelV = 0, hCell = 0; int cell = -1;
                for (int sub = 0; sub < 2; sub++)
                {
                    float frac = sub == 0 ? ld : 1f - ld;
                    if (frac < 0.02f) continue;
                    float r = sub == 0 ? rr : rVoid[e];
                    float px = bp.x + dir.x * r, py = bp.y + dir.y * r, pz = bp.z + dir.z * r;
                    float fx = px * invC - 0.5f - ox, fz = pz * invC - 0.5f - oz;
                    int i0 = (int)fx, j0 = (int)fz;
                    if (fx < 0 || fz < 0 || i0 >= nx - 1 || j0 >= nz - 1) continue;
                    float tx = fx - i0, tz = fz - j0;
                    int k00 = j0 * nx + i0;
                    float h0 = ph[k00] + (ph[k00 + 1] - ph[k00]) * tx, h1 = ph[k00 + nx] + (ph[k00 + nx + 1] - ph[k00 + nx]) * tx;
                    float hs = h0 + (h1 - h0) * tz;
                    int kc = (tz > 0.5f ? k00 + nx : k00) + (tx > 0.5f ? 1 : 0);
                    cell = kc; hCell = hs;
                    // The soil under a tread element that is in contact travels with it (no slip = same spot of ground):
                    // its level and the pressure it has carried are remembered per element until it lifts off.
                    bool was = (sub == 0 ? stT[e] : stV[e]) == step - 1 && !rigid;
                    float sl, sy;
                    if (was)
                    {
                        sl = sub == 0 ? sT[e] : sV[e]; sy = sub == 0 ? syT[e] : syV[e];
                        // a slipping element slides along the ground: on a slope the ground under it rises or falls
                        // with it (without this a spinning wheel on a slope keeps pressing on soil it has left behind)
                        sl += (v.x * gradX + v.z * gradZ) * dt;
                        if (hs > sl) sl += Mathf.Min(hs - sl, slid);        // sliding onto fresh soil
                    }
                    else { sl = hs; sy = psy[kc]; }
                    if (sub == 0) levelT = sl; else levelV = sl;
                    float krr = (sub == 0 ? kTop : kVoid) / cb;
                    float p = Press(sl, sy, py, krr, rigid, sub == 0 ? ctx.ksT : ctx.ksV, en, invN, out float dz, out float ktan);
                    if (p <= 0) continue;
                    // Behind a moving wheel there is only the rut it left, slumped soil and its own spoil: loose soil
                    // with a free face. It cannot push the wheel forwards harder than its passive strength (else a
                    // spinning wheel would climb any slope by leaning on its own spoil heap).
                    if (rearCap > 0 && p > pPassive)
                    {
                        // from 20 deg behind the bottom of the wheel (full effect at 36 deg): closer in, the soil is the confined rut floor
                        float back = Mathf.Clamp01((-(dir.x * travel.x + dir.y * travel.y + dir.z * travel.z) - 0.34f) * 4f) * rearCap;
                        if (back > 0)
                        {
                            float pc = p + (pPassive - p) * back;
                            dz = Mathf.Min(sl - py, dz + (p - pc) / krr);
                            p = pc; ktan *= 1f - back;
                        }
                    }
                    sl -= dz;
                    if (p > sy) sy = p;
                    float a = aE * frac;
                    Kn += ktan * a * cb;
                    if (sub == 0) { pT = p; dzT = dz; aT = a; sT[e] = sl; syT[e] = sy; stT[e] = step; levelT = sl; }
                    else { pV = p; aV = a; sV[e] = sl; syV[e] = sy; stV[e] = step; levelV = sl; }
                    float sk = pb[kc] - sl;
                    if (sk > sink) sink = sk;
                }
                float Ne = pT * aT + pV * aV;

                // what this element leaves in the grid: the mean of the soil under its lug tops and under its voids
                if (deform && Ne > 0 && cell >= 0)
                {
                    float mean = ld >= 0.98f ? levelT : ld <= 0.02f ? levelV : ld * (pT > 0 ? levelT : hCell) + (1f - ld) * (pV > 0 ? levelV : hCell);
                    // The cell takes the height the tread has at the cell's centre, not at this element: follow the
                    // wheel's round profile from here to there (else big cells end up at their lowest element).
                    // Cells whose centre is beside the tyre are left alone.
                    float ccx = (cell % nx + 0.5f + ox) * c - hub.x, ccz = (cell / nx + 0.5f + oz) * c - hub.z;
                    float dC = ccx * fwdH.x + ccz * fwdH.z, aC = ccx * axisH.x + ccz * axisH.z - axialH;
                    float dE = arm.x * fwdH.x + arm.z * fwdH.z;
                    if (mean > hCell - 2e-5f || Mathf.Abs(aC) > halfWidthH || Mathf.Abs(dC) >= rr) mean = float.MaxValue;   // nothing gave way here
                    else mean += MathF.Sqrt(Mathf.Max(rr * rr - dE * dE, 0)) - MathF.Sqrt(rr * rr - dC * dC);
                    if (mean < ph[cell] - 2e-5f)
                    {
                        vol += (ph[cell] - mean) * cellArea; ph[cell] = mean; patch.modified = true;
                        float mk = Mathf.Min(1f, 0.45f + (pb[cell] - mean) * 100f);
                        if (mk > pmark[cell]) pmark[cell] = mk;
                    }
                    else if (pmark[cell] < 0.2f) { pmark[cell] = 0.2f; patch.modified = true; }
                    float pm = Ne / aE;
                    if (pm > psy[cell]) psy[cell] = pm;
                }

                // soil against the tyre's sides (only matters once the wheel sits in a rut)
                if (s.sideForces && !rigid && (iy == 0 || iy == nY - 1) && Ne > 0)
                {
                    float so = sign * (iy == 0 ? -1f : 1f);
                    float r = rVoid[e];
                    float px = bp.x + dir.x * r + axis.x * so * (0.5f * dY + 1.5f * c), pz = bp.z + dir.z * r + axis.z * so * (0.5f * dY + 1.5f * c);
                    float fx = px * invC - 0.5f - ox, fz = pz * invC - 0.5f - oz;
                    int i0 = (int)fx, j0 = (int)fz;
                    if (fx >= 0 && fz >= 0 && i0 < nx && j0 < nz)
                    {
                        float dOut = ph[j0 * nx + i0] - (bp.y + dir.y * r);
                        if (dOut > 0)
                        {
                            if (so < 0) { sideA0 += dOut * r * dTheta * cb; sideZ0 = Mathf.Max(sideZ0, dOut); }
                            else { sideA1 += dOut * r * dTheta * cb; sideZ1 = Mathf.Max(sideZ1, dOut); }
                        }
                    }
                }
                if (Ne <= 0) continue;

                if (stamp[e] != step - 1) { jx[e] = 0; jy[e] = 0; }
                stamp[e] = step;
                float ejx = jx[e] + vt * dt, ejy = jy[e] + va * dt;

                // shear strength of this element, along the rolling direction and across it
                float capT = Grip(mu, pT) * pT * aT, capV = Grip(mu, pV) * pV * aV, cx, cy;
                if (rigid) cx = cy = capT + capV;
                else
                {
                    capT = Mathf.Min(capT, (coh + pT * tanPhi) * aT);
                    capV = Mathf.Min(capV, (coh + pV * tanPhi) * aV);
                    float buried = pV > 0 ? 1f : lugHeight > 0 ? Mathf.Clamp01((hCell - levelT) / lugHeight) : 0f;
                    float passive = (pV * Kp + 2f * coh * sqrtKp) * buried;
                    float limit = coh * (pV > 0 ? aT + aV : aT) + Ne * tanPhi;      // soil shearing on soil
                    cx = Mathf.Min(capT + capV + flankL[e] * passive, limit);
                    cy = Mathf.Min(capT + capV + flankA[e] * passive, limit);
                }
                cx *= gripFactor; cy *= gripFactor;
                float jm = MathF.Sqrt(ejx * ejx + ejy * ejy), ftx = 0, fty = 0;
                if (jm > 4f * K) { float sc = 4f * K / jm; ejx *= sc; ejy *= sc; jm = 4f * K; }
                if (jm > 1e-9f)
                {
                    float m = 1f - MathF.Exp(-jm / K);
                    ftx = -cx * m * ejx / jm; fty = -cy * m * ejy / jm;
                }
                jx[e] = ejx; jy[e] = ejy;
                ftx = Mathf.Clamp(ftx - cx / K * tauD * vt, -cx, cx);
                fty = Mathf.Clamp(fty - cy / K * tauD * va, -cy, cy);
                Kt += cx / K;

                Vector3 f = new Vector3(-dir.x * Ne + tvec.x * ftx + axis.x * fty, -dir.y * Ne + tvec.y * ftx + axis.y * fty, -dir.z * Ne + tvec.z * ftx + axis.z * fty);
                F += f; T += Vector3.Cross(arm, f);
                N += Ne; nSum -= dir * Ne; pSum += arm * Ne; slipSum += (tvec * vt + axis * va) * Ne;
                area += (pT > 0 ? aT : 0) + (pV > 0 ? aV : 0); capSum += cx; voidLoad += pV * aV;
                if (rel < thMin) thMin = rel;
                if (rel > thMax) thMax = rel;
                if (iy < yMin) yMin = iy;
                if (iy > yMax) yMax = iy;
                count++;
                if (cell >= 0 && nCells < cells.Length) cells[nCells++] = cell;
            }
        }
        lastKt = Kt; lastKn = Kn; lastCap = capSum;
        if (N <= 0) { spring = 0; lastScale = 1; return; }

        // The spring carries what the contact carries: correct its deflection for next step and scale this step's
        // forces to match (in steady contact the scale is 1).
        float want = (N + Kn * spring) / (Kc + Kn);
        float scale = Mathf.Clamp(Kc * want / N, 1e-4f, 2f);   // a hard landing: the spring, not the rubber, sets the force
        spring = want; lastScale = scale;
        F *= scale; T *= scale; N *= scale; capSum *= scale;
        res.stiffnessScale = scale;

        Vector3 nrm = nSum.normalized;
        lastNormal = nrm;
        Vector3 cp = hub + pSum * (scale / N);
        // damping along the contact normal (rubber + soil hysteresis); never pulls the wheel down
        float vn = Vector3.Dot(q.hubVelocity, nrm);
        float fd = Mathf.Max(-0.6f * MathF.Sqrt(Kc * Kn / (Kc + Kn) * q.stabilityMass) * vn, -N);
        F += nrm * fd;

        float patchLen = (thMax - thMin + dTheta) * rMax, patchWid = (yMax - yMin + 1) * dY;
        Vector3 slip = slipSum * (scale / N);

        if (!rigid && s.sideForces)
        {
            float gam = g.density * 9.81f;
            for (int sd = 0; sd < 2; sd++)
            {
                float a = sd == 0 ? sideA0 : sideA1, z = sd == 0 ? sideZ0 : sideZ1;
                if (a <= 0) continue;
                Vector3 o = axis * (sd == 0 ? -1f : 1f);
                float vo = Vector3.Dot(q.hubVelocity, o);
                if (vo <= 0) continue;
                float f = a * (0.5f * gam * z * Kp + 2f * coh * sqrtKp) * (float)Math.Tanh(vo / 0.02f);
                F -= o * f; T += Vector3.Cross(cp - q.hub, -o * f);
            }
        }

        if (deform)
        {
            // soil dug out by slipping lugs, thrown where the tread is going
            float vs = Vector3.ProjectOnPlane(slip, nrm).magnitude;
            float bite = Mathf.Max(lugHeight, 5e-4f) * (0.3f + 0.7f * Mathf.Clamp01(voidLoad / N * 2f));
            float dug = vs > 0.03f && nCells > 0 ? g.excavation * (vs - 0.03f) * bite * patchWid * dt : 0;
            if (dug > 0)
            {
                float dh = dug / (nCells * cellArea);
                for (int i = 0; i < nCells; i++) { ph[cells[i]] -= dh; pmark[cells[i]] = 1f; }
                Vector3 sdir = Vector3.ProjectOnPlane(slip, up).normalized;
                for (int k = -1; k <= 1; k++)
                    Deposit(cp + sdir * (rMax + 2.5f * c) + axisH * (k * patchWid / 3f), dug / 3f, c, ox, oz, nx, nz);   // thrown clear of the tyre
                patch.modified = true;
            }
            // soil pressed aside: berms along the rut
            float aside = vol * (1f - g.compaction);
            if (aside > 0)
            {
                Vector3 fwd = fwdH;
                Vector3 mid = q.hub + axis * axialCentre + (cp - q.hub - axis * Vector3.Dot(cp - q.hub, axis));
                for (int sd = -1; sd <= 1; sd += 2)
                for (int k = -1; k <= 1; k++)
                    Deposit(mid + axisH * (sd * (0.5f * width + 1.5f * c)) + fwd * (k * patchLen / 3f), aside / 6f, c, ox, oz, nx, nz);
            }
            if (patch.modified && s.relaxPasses > 0) Relax(Mathf.Tan(g.reposeAngle * Mathf.Deg2Rad) * c, s.relaxPasses);
        }

        // rubber hysteresis: rolling resistance on hard ground
        float kPhys = kTop * (tread.landRatio > 0 && sink < lugHeight ? tread.landRatio : 1f);
        float defl = MathF.Pow(N / (1.3333f * kPhys * Mathf.Max(patchWid, dY) * MathF.Sqrt(2f * rMax)), 0.6667f);
        float halfA = MathF.Sqrt(2f * rMax * defl);
        float crr = 0.3f * (spec != null ? spec.lossTangent : 0.1f) * halfA / (2f * rMax);
        T -= axis * (Mathf.Sign(q.spin) * Mathf.Clamp01(Mathf.Abs(q.spin) * rMax / 0.05f) * crr * N * rMax);

        res.grounded = true; res.force = F; res.torque = T + Vector3.Cross(hub - q.hub, F); res.point = cp; res.normal = nrm; res.load = N;
        res.sinkage = sink; res.deflection = defl; res.patchLength = patchLen; res.patchWidth = patchWid;
        res.pressure = area > 0 ? N / area : 0; res.tractionLimit = capSum; res.displaced = vol; res.elements = count;
        res.traction = Vector3.Dot(F, Vector3.Cross(axis, nrm));   // along the rolling direction (sign: axis x normal)
        if (float.IsNaN(F.x + F.y + F.z + T.x + T.y + T.z)) { Reset(); res.grounded = false; res.force = res.torque = Vector3.zero; }
        res.spring = spring;
    }

    void Deposit(Vector3 w, float volume, float c, float ox, float oz, int nx, int nz)
    {
        int i = Mathf.FloorToInt(w.x / c - ox), j = Mathf.FloorToInt(w.z / c - oz);
        if (i < 0 || j < 0 || i >= nx || j >= nz) return;
        int k = j * nx + i;
        patch.h[k] += volume / (c * c);
        patch.sy[k] = 0;            // loose spoil
        patch.mark[k] = 1f;
        patch.modified = true;
    }

    // Slopes steeper than the angle of repose slump, but only where the soil has been disturbed.
    void Relax(float maxDiff, int passes)
    {
        int nx = patch.nx, nz = patch.nz;
        float[] h = patch.h, mk = patch.mark;
        for (int p = 0; p < passes; p++)
        for (int j = 0; j < nz - 1; j++)
        for (int i = 0; i < nx - 1; i++)
        {
            int k = j * nx + i;
            for (int d = 0; d < 2; d++)
            {
                int o = k + (d == 0 ? 1 : nx);
                float diff = h[k] - h[o];
                if (diff > maxDiff && mk[k] > 0.4f) { float m = 0.25f * (diff - maxDiff); h[k] -= m; h[o] += m; if (mk[o] < 0.6f) mk[o] = 0.6f; }
                else if (-diff > maxDiff && mk[o] > 0.4f) { float m = 0.25f * (-diff - maxDiff); h[o] -= m; h[k] += m; if (mk[k] < 0.6f) mk[k] = 0.6f; }
            }
        }
    }
}
