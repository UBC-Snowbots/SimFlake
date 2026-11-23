using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Object = UnityEngine.Object;

/// <summary>
/// Headless arm tests (no ROS): a player run with <c>--arm-test</c> builds its own test area away from the scene and
/// checks the stepper drives, the gripper and breakage, on a fixed base and on chassis_2024, then quits. Results go to
/// the log as <c>[ArmTest]</c> lines. <c>--arm-test-only hold,grip,...</c> picks tests.
///   tools/arm_test.sh     (builds a player with ArmScene and runs this)
/// </summary>
public class ArmTestRunner : MonoBehaviour
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Create()
    {
        if (!Environment.GetCommandLineArgs().Contains("--arm-test")) return;
        new GameObject("Arm test").AddComponent<ArmTestRunner>();
    }

    static string Arg(string n) { var a = Environment.GetCommandLineArgs(); int i = Array.IndexOf(a, n); return i >= 0 && i + 1 < a.Length ? a[i + 1] : null; }

    static readonly Vector3 Origin = new Vector3(4000, 0, 4000);
    HashSet<string> only;
    GameObject armPrefab, roverPrefab;
    readonly List<string> summary = new List<string>();
    int failures;

    void Log(string s) => Debug.Log("[ArmTest] " + s);
    void Check(bool ok, string what)
    {
        string line = (ok ? "PASS " : "FAIL ") + what;
        summary.Add(line);
        if (!ok) failures++;
        Log(line);
    }
    bool Run(string name) => only == null || only.Contains(name);

    IEnumerator Start()
    {
        ArmBreakage.LogDetails = true;
        noBreak = Environment.GetCommandLineArgs().Contains("--arm-test-nobreak");
        if (float.TryParse(Arg("--arm-break-filter"), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var ft)) filterOverride = ft;
        var o = Arg("--arm-test-only");
        if (o != null) only = new HashSet<string>(o.Split(','));
        Application.targetFrameRate = -1;
        QualitySettings.vSyncCount = 0;
        Time.maximumDeltaTime = 1f;
        Time.timeScale = 20f;

        var cat = RoverCatalog.Load();
        foreach (var e in cat.rovers)
            if (e.prefab != null && e.prefab.TryGetComponent(out RoverPayload p)) { roverPrefab = e.prefab; armPrefab = p.armPrefab; }
        if (armPrefab == null) { Log("FAIL no chassis with a RoverPayload/arm prefab in the RoverCatalog"); Application.Quit(1); yield break; }

        var ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
        ground.name = "arm test ground";
        ground.transform.position = Origin + new Vector3(0, -0.5f, 0);
        ground.transform.localScale = new Vector3(200, 1, 200);

        if (only != null && only.Contains("scene")) { yield return SceneRover(); only.Remove("scene"); if (only.Count == 0) { Finish(); yield break; } }
        if (Run("hold")) yield return Hold();
        if (Run("move")) yield return Move();
        if (Run("limit")) yield return Limit();
        if (Run("geom")) yield return Geometry();
        if (Run("grip")) yield return Grip();
        if (Run("drop")) yield return Drop();
        if (Run("push")) yield return Push();
        if (Run("rover")) yield return Rover();
        if (Run("swapdrive")) yield return SwapWhileDriving();
        if (Run("hold")) yield return Hold();

        Finish();
    }

    void Finish()
    {
        Log("SUMMARY");
        foreach (var s in summary) Log("  " + s);
        Log($"done: {summary.Count - failures}/{summary.Count} passed");
        Application.Quit(failures == 0 ? 0 : 1);
    }

    // The scene's own rover (not the test area): attach the arm the way the Esc menu does, a few seconds in.
    IEnumerator SceneRover()
    {
        Log("== scene: attach the arm to the scene's rover");
        var payload = FindObjectOfType<RoverPayload>();
        if (payload == null) { Check(false, "scene: no RoverPayload in this scene"); yield break; }
        var rover = payload.gameObject;
        var body = RoverSwap.Body(rover);
        float oldScale = Time.timeScale; Time.timeScale = 1f;
        for (float t = 0; t < 3f; t += Time.deltaTime) yield return null;   // like a person opening the menu
        Log("before: " + Pose(rover, body) + $" pos {body.position.ToString("F2")}");
        Vector3 p0 = body.position;
        payload.Apply(RoverPayload.Configuration.Arm);
        float maxV = 0, maxW = 0;
        for (int i = 0; i < 400; i++)
        {
            yield return new WaitForFixedUpdate();
            var ab = body.GetComponent<ArticulationBody>();
            maxV = Mathf.Max(maxV, ab.velocity.magnitude); maxW = Mathf.Max(maxW, ab.angularVelocity.magnitude);
            if (i < 5 || i % 50 == 0) Log($"  step {i}: {Pose(rover, body)} v {ab.velocity.magnitude:0.00} w {ab.angularVelocity.magnitude:0.00}");
        }
        Time.timeScale = oldScale;
        var br = payload.Arm;
        Check(br != null && !br.AnyBroken && maxV < 0.5f && Vector3.Distance(p0, body.position) < 0.2f,
              $"scene: arm attaches quietly (max chassis speed {maxV:0.00} m/s, spin {maxW:0.00} rad/s, moved {Vector3.Distance(p0, body.position):0.00} m, broken: {(br == null ? "no arm" : string.Join(",", br.links.Where(l => l.broken).Select(l => l.name)))})");
    }

    // ---------------------------------------------------------------- helpers

    static IEnumerator Steps(float seconds)
    {
        int n = Mathf.CeilToInt(seconds / Time.fixedDeltaTime);
        for (int i = 0; i < n; i++) yield return new WaitForFixedUpdate();
    }

    static geometry_msgs.msg.Twist Twist(double x) { var t = new geometry_msgs.msg.Twist(); t.linear.x = x; return t; }

    static double[] Nan7() => Enumerable.Repeat(double.NaN, 7).ToArray();

    static void V(ArmController a, int axis, double degPerSec)
    {
        var v = Nan7(); v[axis] = degPerSec;
        a.ArmCommandCallback(new rover_msgs.msg.ArmCommand { cmd_type = (byte)'V', velocities = v });
    }

    static void P(ArmController a, int axis, double deg)
    {
        var p = Nan7(); p[axis] = deg / 360.0;
        a.ArmCommandCallback(new rover_msgs.msg.ArmCommand { cmd_type = (byte)'P', positions = p, velocities = Nan7() });
    }

    bool noBreak;
    float filterOverride = -1;

    void Configure(ArmBreakage b)
    {
        if (b == null) return;
        if (noBreak) b.breakable = false;
        if (filterOverride > 0) b.filterTime = filterOverride;
    }

    string Peaks(ArmBreakage b) =>
        string.Join(", ", b.links.Where(l => l.body != null).Select(l => $"{ArmBreakage.NiceName(l.name)} {l.peakForce:0}N/{l.peakMoment:0}Nm"));

    ArmController StandaloneArm(Vector3 at)
    {
        var arm = Instantiate(armPrefab);
        Configure(arm.GetComponent<ArmBreakage>());
        arm.name = "test arm";
        arm.transform.SetPositionAndRotation(at, Quaternion.identity);
        return arm.GetComponent<ArmController>();
    }

    static void Kill(Component c) { if (c != null) { c.gameObject.SetActive(false); Destroy(c.gameObject); } }

    string Axes(ArmController a)
    {
        var parts = new List<string>();
        for (int i = 0; i < ArmController.NUM_CMD; i++)
        {
            var d = a.Drive(i);
            if (d == null) continue;
            parts.Add($"{ArmController.AxisNames[i]} {a.Reading(i):0.00}° τ {d.torque:0.0}/{d.available:0.0} lost {d.lostSteps}");
        }
        return string.Join(" | ", parts);
    }

    string Loads(ArmBreakage b) =>
        string.Join(", ", b.links.Where(l => l.body != null).Select(l => $"{ArmBreakage.NiceName(l.name)} {l.force:0}N/{l.moment:0.0}Nm ({l.peak * 100:0}%)" + (l.broken ? " BROKEN" : "")));

    static float SubtreeMass(ArticulationBody b) => b.GetComponentsInChildren<ArticulationBody>().Sum(x => x.mass);

    // ---------------------------------------------------------------- tests

    IEnumerator Hold()
    {
        Log("== hold: arm on a fixed base, straight out, gravity on");
        var a = StandaloneArm(Origin + new Vector3(0, 0.6f, 0));
        yield return Steps(3f);
        var br = a.GetComponent<ArmBreakage>();
        Log(Axes(a));
        Log(Loads(br));
        foreach (var l in br.links.Take(3)) Log($"  {l.name}: {br.Describe(l)}");
        float maxDroop = 0; long lost = 0;
        for (int i = 0; i < ArmController.NUM_AXES; i++) { maxDroop = Mathf.Max(maxDroop, Mathf.Abs((float)a.Reading(i))); lost += a.Drive(i).lostSteps; }
        Check(maxDroop < 1.5f && lost == 0, $"hold: max droop {maxDroop:0.00}° (< 1.5), lost steps {lost}");
        // Static loads = weight of what each joint carries (no contacts): checks the Newton-Euler signs.
        foreach (var l in br.links.Where(x => x.body != null && x.body.jointType == ArticulationJointType.RevoluteJoint))
        {
            float w = SubtreeMass(l.body) * 9.81f;
            Log($"  {l.name}: force {l.force:0.0} N, weight carried {w:0.0} N");
            if (l.name == "link_2") Check(Mathf.Abs(l.force - w) < 0.15f * w + 1f, $"hold: shoulder load {l.force:0.0} N = weight beyond it {w:0.0} N");
        }
        Check(!br.AnyBroken, "hold: nothing broke");
        Kill(a);
    }

    IEnumerator Move()
    {
        Log("== move: shoulder 20 deg/s for 1.5 s, stop, then P back to 0");
        var a = StandaloneArm(Origin + new Vector3(3, 0.6f, 0));
        yield return Steps(1f);
        V(a, 1, 20);
        yield return Steps(1.5f);
        V(a, 1, 0);
        yield return Steps(0.5f);
        double moved = a.Reading(1);
        Log($"after V: {Axes(a)}");
        Check(Math.Abs(moved - 30) < 3 && a.Drive(1).lostSteps == 0, $"move: V 20 deg/s x 1.5 s -> {moved:0.0}° (want ~30, ramp), lost {a.Drive(1).lostSteps}");
        P(a, 1, 0);
        yield return Steps(3f);
        Check(Math.Abs(a.Reading(1)) < 0.5, $"move: P back to 0 -> {a.Reading(1):0.00}°");
        Kill(a);
    }

    IEnumerator Limit()
    {
        Log("== limit: elbow driven into its joint limit (stall, lost steps, no blow-up)");
        var a = StandaloneArm(Origin + new Vector3(6, 0.6f, 0));
        a.GetComponent<ArmBreakage>().breakable = false;
        yield return Steps(1f);
        V(a, 2, 40);
        yield return Steps(5f);
        var d = a.Drive(2);
        float vel = Mathf.Abs((float)a.Joint(2).jointVelocity[0]);
        Log($"at limit: {Axes(a)}, joint speed {vel * Mathf.Rad2Deg:0.0} deg/s");
        Check(d.lostSteps > 0 && vel < 0.2f, $"limit: elbow stalls at {a.Reading(2):0.0}° with {d.lostSteps} lost steps, speed {vel * Mathf.Rad2Deg:0.0} deg/s");
        V(a, 2, 0);
        P(a, 2, 0);
        yield return Steps(4f);
        Check(Math.Abs(a.Reading(2)) < 0.5, $"limit: back to 0 afterwards -> {a.Reading(2):0.00}°");
        Kill(a);
    }

    Bounds FingerBounds(ArticulationBody f)
    {
        Bounds b = default; bool first = true;
        foreach (var c in f.GetComponentsInChildren<Collider>())
        {
            if (c.attachedArticulationBody != f) continue;
            if (first) { b = c.bounds; first = false; } else b.Encapsulate(c.bounds);
        }
        return b;
    }

    // Inner gap between the fingers through point p along u (raycast onto each finger's own colliders).
    static float Gap(ArticulationBody l, ArticulationBody r, Vector3 p, Vector3 u)
    {
        float Dist(ArticulationBody f, Vector3 dir)
        {
            float best = float.PositiveInfinity;
            foreach (var h in Physics.RaycastAll(p, dir, 0.3f, ~0, QueryTriggerInteraction.Ignore))
                if (h.collider.attachedArticulationBody == f) best = Mathf.Min(best, h.distance);
            return best;
        }
        return Dist(r, u) + Dist(l, -u);
    }

    // Closest points between the two fingers' colliders (alternating projection, convex hulls).
    static float FingerDistance(ArticulationBody l, ArticulationBody r, out Vector3 pl, out Vector3 pr)
    {
        var cl = l.GetComponentsInChildren<Collider>().Where(c => c.attachedArticulationBody == l).ToArray();
        var cr = r.GetComponentsInChildren<Collider>().Where(c => c.attachedArticulationBody == r).ToArray();
        float best = float.PositiveInfinity; pl = pr = Vector3.zero;
        foreach (var a in cl)
            foreach (var b in cr)
            {
                Vector3 x = a.bounds.center, y = b.bounds.center;
                for (int i = 0; i < 8; i++) { y = Physics.ClosestPoint(x, b, b.transform.position, b.transform.rotation); x = Physics.ClosestPoint(y, a, a.transform.position, a.transform.rotation); }
                float d = Vector3.Distance(x, y);
                if (d < best) { best = d; pl = x; pr = y; }
            }
        return best;
    }

    IEnumerator Geometry()
    {
        Log("== geom: gripper finger distance vs EE position");
        var a = StandaloneArm(Origin + new Vector3(15, 0.6f, 0));
        yield return Steps(1f);
        var fl = a.Fingers[0]; var fr = a.Fingers[1];
        var w = a.Joint(5).transform;
        foreach (double t in new[] { -65.0, -40, -20, 0, 20, 40, 60 })
        {
            P(a, ArmController.EE, t);
            yield return Steps(1.2f);
            float d = FingerDistance(fl, fr, out var pl, out var pr);
            Log($"EE {a.Reading(6):0.0}° (q {fl.jointPosition[0] * 1000:0.0} / {fr.jointPosition[0] * 1000:0.0} mm): closest {d * 1000:0.0} mm at L {w.InverseTransformPoint(pl).ToString("F3")} R {w.InverseTransformPoint(pr).ToString("F3")} " +
                $"Lc {w.InverseTransformPoint(FingerBounds(fl).center).ToString("F3")} Rc {w.InverseTransformPoint(FingerBounds(fr).center).ToString("F3")} tt {w.InverseTransformPoint(a.transform.GetComponentsInChildren<Transform>().First(x => x.name == "link_tt").position).ToString("F3")}");
        }
        Kill(a);
    }

    IEnumerator Grip()
    {
        Log("== grip: close on a box, stop commanding, release the box, lift, open");
        var a = StandaloneArm(Origin + new Vector3(9, 0.6f, 0));
        yield return Steps(1f);
        // Point the gripper down: wrist pitch, so a box hangs in it.
        V(a, ArmController.EE, -150);
        yield return Steps(1f);
        V(a, ArmController.EE, 0);
        yield return Steps(0.3f);
        var fl = a.Fingers[0]; var fr = a.Fingers[1];
        var wrist = a.Joint(5).transform;
        float open = FingerDistance(fl, fr, out var pl, out var pr);
        Vector3 u = (pr - pl).normalized;
        // Box centre between the fingertips' closest points, 1.5 cm in from the tips (towards the wrist).
        Vector3 inward = (wrist.position - (pl + pr) / 2); inward -= Vector3.Dot(inward, u) * u; inward.Normalize();
        Vector3 mid = (pl + pr) / 2 + inward * 0.015f;
        float gap = FingerDistance(fl, fr, out _, out _);
        Log($"open: fingers {open * 1000:0} mm apart, EE {a.Reading(6):0}°");
        float size = 0.05f;
        var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
        box.name = "grip box";
        box.transform.localScale = Vector3.one * size;
        box.transform.SetPositionAndRotation(mid, Quaternion.LookRotation(u, inward));
        var rb = box.AddComponent<Rigidbody>();
        rb.mass = 0.2f;
        rb.isKinematic = true;
        long lost0 = a.Drive(6).lostSteps;
        V(a, ArmController.EE, 90);
        yield return Steps(2.5f);
        var d = a.Drive(6);
        Log($"closing: EE {a.Reading(6):0.0}°, finger forces {d.outputs[0].force:0.0} / {d.outputs[1].force:0.0} N, motor {d.torque:0.00}/{d.available:0.00} N·m, lost {d.lostSteps - lost0}");
        Check(d.lostSteps > lost0, $"grip: motor stalls on the box (lost {d.lostSteps - lost0} steps)");
        V(a, ArmController.EE, 0);   // stop: nothing commanded from here on
        yield return Steps(0.3f);
        rb.isKinematic = false;
        var ee = wrist;  // wrist pitch link: the box should move with it
        Vector3 rel0 = ee.InverseTransformPoint(box.transform.position);
        var forces = new List<float>();
        for (int i = 0; i < 10; i++)
        {
            yield return Steps(0.2f);
            forces.Add(Mathf.Min(Mathf.Abs(d.outputs[0].force), Mathf.Abs(d.outputs[1].force)));
        }
        float slip = (ee.InverseTransformPoint(box.transform.position) - rel0).magnitude;
        Log($"held 2 s with no command: squeeze {string.Join(" ", forces.Select(f => f.ToString("0.0")))} N, box moved {slip * 1000:0.0} mm in the gripper");
        Check(forces.Min() > 5f && forces.Max() - forces.Min() < 0.25f * forces.Max() + 0.5f, $"grip: squeeze holds without a command ({forces.Min():0.0}-{forces.Max():0.0} N, max {a.MaxGripForce:0} N)");
        Check(slip < 0.005f, $"grip: box stays in the gripper ({slip * 1000:0.0} mm)");

        Vector3 tip0 = box.transform.position;
        P(a, 1, 20);
        yield return Steps(1.5f);
        P(a, 3, 60);
        yield return Steps(2f);
        float slip2 = (ee.InverseTransformPoint(box.transform.position) - rel0).magnitude;
        Log($"moved the arm: box travelled {Vector3.Distance(tip0, box.transform.position) * 1000:0} mm, slipped {slip2 * 1000:0.0} mm. {Axes(a)}");
        // A cube pinched at two faces can pivot about the pinch line (so can a real one); it must not come out.
        Check(slip2 < 0.03f && Vector3.Distance(tip0, box.transform.position) > 0.05f, $"grip: box moves with the arm and stays in the gripper (shifted {slip2 * 1000:0.0} mm, pivoting allowed)");

        // Heavier: 1 kg.
        rb.mass = 1f;
        Vector3 rel1 = ee.InverseTransformPoint(box.transform.position);
        yield return Steps(1.5f);
        float slip3 = (ee.InverseTransformPoint(box.transform.position) - rel1).magnitude;
        Log($"1 kg box: slipped {slip3 * 1000:0.0} mm in 1.5 s");

        V(a, ArmController.EE, -150);
        yield return Steps(1f);
        float fell = (ee.InverseTransformPoint(box.transform.position) - rel1).magnitude;
        Check(fell > 0.03f, $"grip: opening drops the box (moved {fell * 1000:0} mm)");
        Destroy(box);
        Kill(a);
    }

    IEnumerator DropOn(ArmController a, float mass, float size, float height, Action<GameObject> done)
    {
        var target = a.Joint(3);   // forearm
        var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
        box.name = $"drop {mass} kg";
        box.transform.localScale = Vector3.one * size;
        box.transform.position = target.worldCenterOfMass + Vector3.up * (height + size / 2 + 0.08f);
        box.AddComponent<Rigidbody>().mass = mass;
        yield return Steps(2.5f);
        done(box);
    }

    IEnumerator Drop()
    {
        Log("== drop: boxes onto the forearm");
        var a = StandaloneArm(Origin + new Vector3(12, 0.6f, 0));
        var br = a.GetComponent<ArmBreakage>();
        yield return Steps(1.5f);
        GameObject box = null;
        yield return DropOn(a, 0.5f, 0.08f, 0.3f, b => box = b);
        Log("0.5 kg from 0.3 m: " + Loads(br));
        Log("  PEAKS drop 0.5 kg: " + Peaks(br));
        br.ResetPeaks();
        Check(!br.AnyBroken, "drop: 0.5 kg from 0.3 m breaks nothing");
        // Box resting on the forearm: elbow and shoulder carry its weight too (checks the contact force signs).
        var shoulder = br.links.First(l => l.name == "link_2");
        float w = (SubtreeMass(shoulder.body) + 0.5f) * 9.81f;
        Log($"  shoulder force with the box resting on the forearm {shoulder.force:0.0} N, weight {w:0.0} N");
        Check(Mathf.Abs(shoulder.force - w) < 0.2f * w, $"drop: contact forces counted with the right sign ({shoulder.force:0.0} vs {w:0.0} N)");
        Destroy(box);
        yield return Steps(0.5f);

        yield return DropOn(a, 40f, 0.34f, 1.0f, b => box = b);
        Log("40 kg from 1 m: " + Loads(br));
        Log("  PEAKS drop 40 kg: " + Peaks(br));
        var broke = br.links.Where(l => l.broken).Select(l => ArmBreakage.NiceName(l.name) + " (" + l.brokeBecause + ")").ToArray();
        Check(broke.Length > 0, "drop: 40 kg from 1 m breaks the arm: " + string.Join(", ", broke));
        Log("after break: " + Axes(a));
        Destroy(box);
        yield return Steps(0.5f);

        var fixedArm = br.Repair();
        yield return Steps(2f);
        var fa = fixedArm.GetComponent<ArmController>();
        bool intact = !fixedArm.AnyBroken && Enumerable.Range(0, 7).All(i => fa.Powered(i));
        Log("repaired: " + Axes(fa));
        Check(intact && Math.Abs(fa.Reading(1)) < 1.5, $"drop: FIX ARM gives an intact arm at home (shoulder {fa.Reading(1):0.00}°), debris gone: {br == null}");
        Kill(fa);
    }

    string Pose(GameObject rover, Transform body) =>
        $"chassis y {body.position.y - Origin.y:0.000} euler {body.eulerAngles.ToString("F1")} wheels y " +
        string.Join(" ", rover.GetComponentsInChildren<DriveWheel>().Select(w => (w.transform.position.y - Origin.y).ToString("0.000"))) +
        " bogies " + string.Join(" ", rover.GetComponentsInChildren<ArticulationBody>().Where(b => b.name.Contains("bogie")).Select(b => (b.jointPosition.dofCount > 0 ? b.jointPosition[0] * Mathf.Rad2Deg : 0).ToString("0.0")));

    IEnumerator Push()
    {
        Log("== push: shoulder drives the gripper down onto a block until the motor stalls");
        var a = StandaloneArm(Origin + new Vector3(18, 0.6f, 0));
        var br = a.GetComponent<ArmBreakage>();
        yield return Steps(1f);
        var tt = a.transform.GetComponentsInChildren<Transform>().First(x => x.name == "link_tt");
        var block = GameObject.CreatePrimitive(PrimitiveType.Cube);
        block.name = "push block";
        block.transform.localScale = new Vector3(0.3f, 0.6f, 0.3f);
        block.transform.position = tt.position - Vector3.up * 0.33f;
        br.ResetPeaks();
        for (int dir = -1; dir <= 1; dir += 2)
        {
            V(a, 1, 15 * dir);
            yield return Steps(4f);
            Log($"shoulder {(dir < 0 ? "-" : "+")}: {Axes(a)}");
        }
        V(a, 1, 0);
        Check(br.AnyBroken, "push: forcing the gripper into a block with the shoulder at stall breaks the arm" + (br.AnyBroken ? ": " + string.Join(", ", br.links.Where(l => l.broken).Select(l => ArmBreakage.NiceName(l.name))) : ""));
        Log("  PEAKS push: " + Peaks(br) + (br.AnyBroken ? "  BROKE: " + string.Join(", ", br.links.Where(l => l.broken).Select(l => l.name + " " + l.brokeBecause)) : ""));
        Destroy(block);
        Kill(a);
    }

    GameObject SpawnRover(Vector3 at, out Transform body)
    {
        var staging = new GameObject("staging"); staging.SetActive(false);
        var rover = Instantiate(roverPrefab, staging.transform);
        rover.name = "test rover";
        body = RoverSwap.Body(rover);
        rover.transform.rotation = Quaternion.Inverse(Quaternion.Euler(0, body.eulerAngles.y, 0)) * rover.transform.rotation;
        rover.transform.position += at - body.position;
        rover.transform.SetParent(null, true);
        rover.SetActive(true);
        Destroy(staging);
        body = RoverSwap.Body(rover);
        return rover;
    }

    IEnumerator SwapWhileDriving()
    {
        Log("== swapdrive: change configuration and FIX ARM while driving at 1 m/s");
        var rover = SpawnRover(Origin + new Vector3(20, 0.45f, -40), out var body);
        var payload = rover.GetComponent<RoverPayload>();
        var mc = rover.GetComponent<MotorController>();
        payload.Apply(RoverPayload.Configuration.Arm);
        yield return Steps(2.5f);
        Vector3 p0 = body.position;
        float minUp = 1;
        for (int k = 0; k < 4; k++)
        {
            for (float t = 0; t < 1.5f; t += Time.fixedDeltaTime)
            {
                mc.CmdVelCallback(Twist(1.0));
                yield return new WaitForFixedUpdate();
                minUp = Mathf.Min(minUp, body.up.y);
            }
            if (k == 0) payload.Apply(RoverPayload.Configuration.None);
            if (k == 1) payload.Apply(RoverPayload.Configuration.Arm);
            if (k == 2) { payload.Arm.Break(payload.Arm.links.First(l => l.name == "link_2")); payload.Arm.Repair(); }
            Log($"  after step {k}: {Pose(rover, body)} wheels " + string.Join(", ", rover.GetComponentsInChildren<DriveWheel>().Select(w => $"{w.TargetSpeed:0.0}/{w.SetpointSpeed:0.0}/{w.Speed:0.0}")) + $" t {Time.time:0.0}");
        }
        mc.CmdVelCallback(Twist(0));
        float d = Vector3.Distance(p0, body.position);
        Check(minUp > 0.97f && d > 4.5f && payload.Arm != null && !payload.Arm.AnyBroken,
              $"swapdrive: rover stays upright (min up.y {minUp:0.000}) and keeps going ({d:0.0} m in 6 s at 1 m/s) through NONE / ARM / FIX ARM");
        rover.SetActive(false);
        Destroy(rover);
    }

    IEnumerator Rover()
    {
        Log("== rover: chassis_2024 with the arm mounted");
        var staging = new GameObject("staging"); staging.SetActive(false);
        var rover = Instantiate(roverPrefab, staging.transform);
        rover.name = "test rover";
        var body = RoverSwap.Body(rover);
        rover.transform.rotation = Quaternion.identity * Quaternion.Inverse(Quaternion.Euler(0, body.eulerAngles.y, 0)) * rover.transform.rotation;
        rover.transform.position += Origin + new Vector3(0, 0.45f, -40) - body.position;
        rover.transform.SetParent(null, true);
        rover.SetActive(true);
        Destroy(staging);
        body = RoverSwap.Body(rover);
        var payload = rover.GetComponent<RoverPayload>();
        var mc = rover.GetComponent<MotorController>();
        yield return Steps(2f);
        Log("before mount: " + Pose(rover, body));

        var sw = System.Diagnostics.Stopwatch.StartNew();
        payload.Apply(RoverPayload.Configuration.Arm);
        Log($"mounting the arm took {sw.ElapsedMilliseconds} ms");
        for (int i = 0; i < 4; i++) { float t0 = Time.realtimeSinceStartup; yield return new WaitForFixedUpdate(); Log($"  step {i}: {(Time.realtimeSinceStartup - t0) * 1000:0} ms real"); }
        Log("after mount: " + Pose(rover, body));
        var br = payload.Arm;
        Configure(br);
        var a = br.GetComponent<ArmController>();
        yield return Steps(3f);
        float massTotal = rover.GetComponentsInChildren<ArticulationBody>().Sum(b => b.mass);
        Log($"mounted: rover + arm {massTotal:0.0} kg; {Axes(a)}");
        Log(Loads(br));
        Check(!br.AnyBroken && Math.Abs(a.Reading(1)) < 1.5, $"rover: arm holds on the rover (shoulder {a.Reading(1):0.00}°)");

        // Drive over 4 cm bumps at 1 m/s.
        bool ramps = Environment.GetCommandLineArgs().Contains("--arm-test-ramps");
        var bumps = new List<GameObject>();
        for (int i = 0; i < 4; i++)
        {
            var bump = GameObject.CreatePrimitive(PrimitiveType.Cube);
            bumps.Add(bump);
            bump.name = "bump";
            if (ramps)
            {
                // 4 cm high, 30 cm long hump: a cube rotated about the rover's lateral axis, mostly buried.
                bump.transform.localScale = new Vector3(3, 0.2f, 0.2f);
                bump.transform.rotation = Quaternion.LookRotation(body.forward) * Quaternion.Euler(45, 0, 0);
                bump.transform.position = body.position + body.forward * (2.5f + 1.7f * i) + Vector3.up * (0.04f - 0.1414f - (body.position.y - Origin.y));
            }
            else
            {
                bump.transform.localScale = new Vector3(3, 0.04f, 0.15f);
                bump.transform.position = body.position + body.forward * (2.5f + 1.7f * i) + Vector3.up * (0.02f - (body.position.y - Origin.y));
            }
        }
        Vector3 p0 = body.position;
        br.ResetPeaks();
        bool savedBreakable = br.breakable;
        br.breakable = Environment.GetCommandLineArgs().Contains("--arm-test-trace") ? false : savedBreakable;
        var fore = br.links.First(l => l.name == "link_4");
        var trace = new List<string>();
        Vector3 bp1 = body.position, bp2 = body.position; float bestLoad = 0; int bestIdx = 0;
        var chassisAb = body.GetComponent<ArticulationBody>();
        for (float t = 0; t < 8f; t += Time.fixedDeltaTime)
        {
            mc.CmdVelCallback(Twist(1.0));
            yield return new WaitForFixedUpdate();
            Vector3 bp = chassisAb.worldCenterOfMass;
            Vector3 acc = (bp - 2 * bp1 + bp2) / (Time.fixedDeltaTime * Time.fixedDeltaTime);
            bp2 = bp1; bp1 = bp;
            if (fore.load > bestLoad) { bestLoad = fore.load; bestIdx = trace.Count; }
            trace.Add($"t {t:0.000} chassis acc {acc.ToString("F0")} fore {fore.force:0}N {fore.moment:0.0}Nm | " +
                      string.Join(" ", Enumerable.Range(0, 6).Select(i => $"{a.Reading(i):0.00}/{a.Drive(i).torque:0.0}")));
        }
        br.breakable = savedBreakable;
        if (Environment.GetCommandLineArgs().Contains("--arm-test-trace"))
            for (int i = Mathf.Max(0, bestIdx - 60); i < Mathf.Min(trace.Count, bestIdx + 20); i++) Log("  trace " + trace[i]);
        mc.CmdVelCallback(Twist(0));
        yield return Steps(1f);
        Log($"drove {Vector3.Distance(p0, body.position):0.0} m over bumps: " + Loads(br) + $"; rover up.y {body.up.y:0.00}");
        Log("  " + Axes(a));
        Log("  PEAKS bumps: " + Peaks(br));
        Check(!br.AnyBroken, $"rover: 1 m/s over 4 cm bumps breaks nothing (worst {br.links.Max(l => l.peak) * 100:0}% of a limit)");
        br.ResetPeaks();
        foreach (var bmp in bumps) Destroy(bmp);   // so the rover isn't left sitting on one

        // Break it on purpose, FIX ARM, then none -> arm, then drive: the rover still works.
        br.Break(br.links.First(l => l.name == "link_3"));
        yield return Steps(1f);
        Check(br.AnyBroken && !a.Powered(4), "rover: a broken elbow leaves the forearm unpowered");
        br = br.Repair();
        yield return Steps(2f);
        Check(!br.AnyBroken && Math.Abs(br.GetComponent<ArmController>().Reading(1)) < 1.5, "rover: FIX ARM on the rover");
        payload.Apply(RoverPayload.Configuration.None);
        yield return Steps(0.5f);
        Log("after NONE: " + Pose(rover, body));
        Check(payload.Arm == null && rover.GetComponentsInChildren<ArmController>().Length == 0, "rover: configuration NONE removes the arm");
        payload.Apply(RoverPayload.Configuration.Arm);
        yield return new WaitForFixedUpdate();
        Log("right after ARM: " + Pose(rover, body));
        yield return Steps(2f);
        Log("after ARM: " + Pose(rover, body));
        p0 = body.position;
        for (float t = 0; t < 3f; t += Time.fixedDeltaTime)
        {
            mc.CmdVelCallback(Twist(-0.5));
            yield return new WaitForFixedUpdate();
        }
        mc.CmdVelCallback(Twist(0));
        float d = Vector3.Distance(p0, body.position);
        Log($"re-mounted drive: rover up.y {body.up.y:0.00}, wheels: " + string.Join(", ", rover.GetComponentsInChildren<DriveWheel>().Select(w => $"{w.name.Replace("_wheel_link", "")} {w.Speed:0.0} rad/s")));
        Check(d > 1.0f && payload.Arm != null && !payload.Arm.AnyBroken, $"rover: arm re-mounted, rover still drives ({d:0.00} m in 3 s at 0.5 m/s)");
        Check(!payload.Apply(RoverPayload.Configuration.RoverLab), "rover: ROVER LAB is refused (not modelled yet)");

        // Last: into a wall fast with the arm sticking out in front. Informational: steppers skip and the arm folds
        // back, so whether something breaks depends on how it hits.
        br = payload.Arm;
        float wallSpeed = float.TryParse(Arg("--arm-test-wall-speed"), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var ws) ? ws : 3f;
        br.ResetPeaks();
        var wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.name = "wall";
        wall.transform.localScale = new Vector3(6, 2, 0.5f);
        wall.transform.rotation = Quaternion.LookRotation(body.forward);
        wall.transform.position = body.position + body.forward * 6f + Vector3.up * (1 - (body.position.y - Origin.y));
        for (float t = 0; t < 4f; t += Time.fixedDeltaTime)
        {
            mc.CmdVelCallback(Twist(wallSpeed));
            yield return new WaitForFixedUpdate();
        }
        mc.CmdVelCallback(Twist(0));
        yield return Steps(1f);
        Log($"wall at {wallSpeed} m/s: " + (br.AnyBroken ? "BROKE " + string.Join(", ", br.links.Where(l => l.broken).Select(l => ArmBreakage.NiceName(l.name) + " (" + l.brokeBecause + ")")) : "nothing broke") + "; PEAKS " + Peaks(br));
        Destroy(wall);
        rover.SetActive(false);   // MotorController finds wheels by name: the next rover must not see these
        Destroy(rover);
    }
}
