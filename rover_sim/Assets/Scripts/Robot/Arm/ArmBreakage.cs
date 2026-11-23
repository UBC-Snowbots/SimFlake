using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

/// <summary>
/// Breakable arm: every joint (and the mount on the rover) has a strength. When the load through it exceeds that,
/// the link and everything beyond it come off as a loose piece (unpowered, it falls and tumbles).
/// <see cref="Repair"/> puts a fresh arm back (same settings), e.g. from the Esc menu's FIX ARM.
///
/// The load through a joint is worked out from the motion of everything beyond it (Newton-Euler, PhysX has no joint
/// reaction forces for articulations): force = sum of m (a - g) minus contact forces, moment likewise about the joint.
/// Accelerations come from position differences, contacts from collision impulses. For a revolute joint the moment
/// about its own axis is the motor's business (it stalls and skips instead, see StepperDrive), so only bending counts;
/// for a finger (prismatic) the squeeze along its travel is the motor's, so only the side force counts.
/// Loads are low-pass filtered (<see cref="filterTime"/>): rigid PhysX contacts give one-step spikes that a real
/// structure (and real tyres) would not see.
/// </summary>
[DefaultExecutionOrder(50)]
public class ArmBreakage : MonoBehaviour
{
    public enum Pivot { JointAnchor, CenterOfMass }

    [Serializable]
    public class Link
    {
        public string name;
        [Tooltip("Force through the joint that breaks it (N).")]
        public float maxForce = 1000f;
        [Tooltip("Bending moment that breaks it (N·m). Not the motor's own axis.")]
        public float maxMoment = 100f;
        [Tooltip("Where the moment is taken: the joint, or the link's centre of mass (fingers: their URDF frame is far from the part).")]
        public Pivot pivot = Pivot.JointAnchor;

        [NonSerialized] public ArticulationBody body;
        [NonSerialized] public ArticulationBody[] subtree;
        [NonSerialized] public bool broken, fixedToWorld;
        /// <summary>Filtered force (N) and moment (N·m) now, load = largest fraction of the limits, peak = highest load so far.</summary>
        [NonSerialized] public float force, moment, load, peak, peakForce, peakMoment;
        [NonSerialized] public string brokeBecause;
    }

    [Tooltip("Off = nothing breaks (loads are still measured).")]
    public bool breakable = true;
    [Tooltip("Multiplies every strength below.")]
    public float strengthScale = 1f;
    [Tooltip("Load filter time constant (s). Shorter = impacts break things more easily.")]
    public float filterTime = 0.1f;
    [Tooltip("No breaking during the first seconds (the arm settles, a rover lands).")]
    public float armingDelay = 1f;

    [Tooltip("Strengths are estimates for a printed / aluminium arm, not measured.")]
    public List<Link> links = DefaultLinks();

    public static List<Link> DefaultLinks() => new List<Link>
    {
        // Measured with the 0.1 s filter (tools/arm_test.sh -- --arm-test-nobreak): chassis_2024 over sharp 4 cm steps at
        // 1 m/s peaks at mount 200 / base 75 / shoulder 122 / elbow 53 / forearm 63 N·m; the shoulder pressing the
        // gripper into the ground at stall gives forearm 120, base 196; a 30 kg box dropped 0.6 m on the forearm gives
        // wrist roll 34. So: driving never breaks it, forcing it into the ground breaks the forearm, the drop the wrist.
        new Link { name = "link_0", maxForce = 1500, maxMoment = 300 },   // mount on the rover
        new Link { name = "link_1", maxForce = 1200, maxMoment = 250 },   // base turntable
        new Link { name = "link_2", maxForce = 1000, maxMoment = 180 },   // shoulder
        new Link { name = "link_3", maxForce = 800, maxMoment = 90 },     // elbow
        new Link { name = "link_4", maxForce = 600, maxMoment = 100 },    // forearm
        new Link { name = "link_5", maxForce = 400, maxMoment = 30 },     // wrist
        new Link { name = "link_6", maxForce = 400, maxMoment = 25 },     // wrist roll
        new Link { name = "ee_base_link", maxForce = 400, maxMoment = 25 },
        new Link { name = "finger_left", maxForce = 200, maxMoment = 6, pivot = Pivot.CenterOfMass },
        new Link { name = "finger_right", maxForce = 200, maxMoment = 6, pivot = Pivot.CenterOfMass },
    };

    public static readonly Dictionary<string, string> Nice = new Dictionary<string, string>
    {
        { "link_0", "Mount" }, { "link_1", "Base" }, { "link_2", "Shoulder" }, { "link_3", "Elbow" }, { "link_4", "Forearm" },
        { "link_5", "Wrist" }, { "link_6", "Wrist roll" }, { "ee_base_link", "Gripper mount" },
        { "finger_left", "Left finger" }, { "finger_right", "Right finger" },
    };
    public static string NiceName(string link) => Nice.TryGetValue(link, out var n) ? n : link;

    /// <summary>Raised when a link breaks off: (arm, link).</summary>
    public static event Action<ArmBreakage, Link> Broke;

    /// <summary>Log what each break was made of (per body forces and contacts).</summary>
    public static bool LogDetails;

    public bool AnyBroken => links.Any(l => l.broken);
    public IReadOnlyList<GameObject> Debris => debris;

    readonly List<GameObject> debris = new List<GameObject>();
    readonly Dictionary<ArticulationBody, Kin> kin = new Dictionary<ArticulationBody, Kin>();
    readonly Dictionary<ArticulationBody, Link> linkOf = new Dictionary<ArticulationBody, Link>();
    readonly Dictionary<ArticulationBody, List<Link>> lineage = new Dictionary<ArticulationBody, List<Link>>();
    float startTime;

    // Spare copy for Repair, taken before anything moves.
    GameObject spare, spareHolder;
    Transform homeParent;
    Vector3 homeLocalPos;
    Quaternion homeLocalRot;

    class Kin
    {
        public Vector3 com, vel, w;
        public int samples;
        public Vector3 force, moment;   // m (a - g), I alpha + w x I w  (this step)
        public ArmLinkContacts contacts;
        public int lastContactCount;
        public Vector3 lastContactSum;
    }

    void Awake()
    {
        spareHolder = new GameObject(name + " spare (for repair)");
        spareHolder.SetActive(false);
        if (gameObject.scene.IsValid() && spareHolder.scene != gameObject.scene) SceneManager.MoveGameObjectToScene(spareHolder, gameObject.scene);
        spare = Instantiate(gameObject, spareHolder.transform);
        spare.name = name;
    }

    void Start()
    {
        // Where it stands now (whoever instantiated it has placed it by now; Awake ran inside Instantiate).
        homeParent = transform.parent;
        homeLocalPos = transform.localPosition;
        homeLocalRot = transform.localRotation;
        startTime = Time.time;
        var bodies = GetComponentsInChildren<ArticulationBody>();
        foreach (var l in links)
        {
            l.body = bodies.FirstOrDefault(b => b.name == l.name);
            if (l.body == null) continue;
            linkOf[l.body] = l;
            // The arm's own root on a fixed base (ArmScene) can't come off anything.
            l.fixedToWorld = l.body.isRoot;
        }
        foreach (var b in bodies)
        {
            if (!b.TryGetComponent(out ArmLinkContacts c)) c = b.gameObject.AddComponent<ArmLinkContacts>();
            kin[b] = new Kin { contacts = c };
            var chain = new List<Link>();
            for (var t = b.transform; t != null && t != transform.parent; t = t.parent)
                if (t.TryGetComponent(out ArticulationBody ab) && linkOf.TryGetValue(ab, out var l)) chain.Add(l);
            lineage[b] = chain;
        }
        RefreshSubtrees();
    }

    void RefreshSubtrees()
    {
        foreach (var l in links)
            if (l.body != null) l.subtree = l.body.GetComponentsInChildren<ArticulationBody>();
    }

    /// <summary>True while neither this body's joint nor any joint between it and the rover has broken.</summary>
    public bool Intact(ArticulationBody b)
    {
        if (b == null) return false;
        if (!lineage.TryGetValue(b, out var chain)) return true;
        foreach (var l in chain) if (l.broken) return false;
        return true;
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        if (dt <= 0) return;
        Vector3 g = Physics.gravity;

        // Per body: m (a - g) and the rotational term, from the last physics step.
        foreach (var kv in kin)
        {
            var b = kv.Key; var k = kv.Value;
            if (b == null || !b.isActiveAndEnabled) continue;
            Vector3 com = b.worldCenterOfMass, w = b.angularVelocity;
            Vector3 vel = k.samples > 0 ? (com - k.com) / dt : Vector3.zero;
            if (k.samples >= 2)
            {
                Vector3 a = (vel - k.vel) / dt, alpha = (w - k.w) / dt;
                k.force = b.mass * (a - (b.useGravity ? g : Vector3.zero));
                Quaternion R = b.transform.rotation * b.inertiaTensorRotation;
                k.moment = R * Vector3.Scale(b.inertiaTensor, Quaternion.Inverse(R) * alpha)
                         + Vector3.Cross(w, R * Vector3.Scale(b.inertiaTensor, Quaternion.Inverse(R) * w));
            }
            k.com = com; k.vel = vel; k.w = w;
            k.samples = Math.Min(k.samples + 1, 3);
        }

        float blend = Mathf.Clamp01(dt / Mathf.Max(filterTime, dt));
        bool armed = Time.time - startTime > armingDelay;
        Link toBreak = null;
        foreach (var l in links)
        {
            if (l.body == null || l.broken || l.subtree == null) continue;
            var b = l.body;
            Vector3 p = l.pivot == Pivot.CenterOfMass ? b.worldCenterOfMass : b.transform.TransformPoint(b.anchorPosition);
            Vector3 F = Vector3.zero, M = Vector3.zero;
            bool valid = true;
            foreach (var s in l.subtree)
            {
                if (s == null || !kin.TryGetValue(s, out var k)) continue;
                if (k.samples < 3) { valid = false; break; }
                F += k.force;
                M += Vector3.Cross(k.com - p, k.force) + k.moment;
                var c = k.contacts;
                for (int i = 0; i < c.forces.Count; i++)
                {
                    F -= c.forces[i];
                    M -= Vector3.Cross(c.points[i] - p, c.forces[i]);
                }
            }
            if (!valid) continue;

            Vector3 axis = (b.transform.rotation * b.anchorRotation * Vector3.right).normalized;
            float f, m;
            switch (b.jointType)
            {
                case ArticulationJointType.RevoluteJoint:
                    f = F.magnitude; m = (M - Vector3.Dot(M, axis) * axis).magnitude; break;
                case ArticulationJointType.PrismaticJoint:
                    f = (F - Vector3.Dot(F, axis) * axis).magnitude; m = M.magnitude; break;
                default:
                    f = F.magnitude; m = M.magnitude; break;
            }
            l.force += (f - l.force) * blend;
            l.moment += (m - l.moment) * blend;
            float scale = Mathf.Max(strengthScale, 1e-3f);
            l.load = Mathf.Max(l.force / Mathf.Max(l.maxForce * scale, 1e-3f), l.moment / Mathf.Max(l.maxMoment * scale, 1e-3f));
            if (armed) { l.peak = Mathf.Max(l.peak, l.load); l.peakForce = Mathf.Max(l.peakForce, l.force); l.peakMoment = Mathf.Max(l.peakMoment, l.moment); }
            if (armed && breakable && !l.fixedToWorld && l.load > 1f && (toBreak == null || l.load > toBreak.load)) toBreak = l;
        }
        foreach (var k in kin.Values)
        {
            if (k.contacts == null) continue;
            k.lastContactCount = k.contacts.forces.Count;
            k.lastContactSum = Vector3.zero;
            foreach (var f in k.contacts.forces) k.lastContactSum += f;
            k.contacts.Clear();
        }
        if (toBreak != null) Break(toBreak);
    }

    /// <summary>Forget the peak loads (menu: RESET PEAKS).</summary>
    public void ResetPeaks() { foreach (var l in links) l.peak = l.peakForce = l.peakMoment = 0; }

    /// <summary>Debug: what the load through <paramref name="l"/> is made of (per body: m (a - g), contacts).</summary>
    public string Describe(Link l)
    {
        if (l?.subtree == null) return "";
        var sb = new System.Text.StringBuilder();
        foreach (var s in l.subtree)
        {
            if (s == null || !kin.TryGetValue(s, out var k)) continue;
            int n = k.contacts.forces.Count > 0 ? k.contacts.forces.Count : k.lastContactCount;
            Vector3 c = Vector3.zero;
            if (k.contacts.forces.Count > 0) foreach (var f in k.contacts.forces) c += f; else c = k.lastContactSum;
            sb.Append($"{s.name}: m(a-g) {k.force.ToString("F1")} contacts {n} sum {c.ToString("F1")}{(n > 0 ? " with " + k.contacts.lastOther : "")}; ");
        }
        return sb.ToString();
    }

    /// <summary>Break this link off now (also used by tests).</summary>
    public void Break(Link l)
    {
        if (l == null || l.broken || l.body == null || l.fixedToWorld) return;
        float scale = Mathf.Max(strengthScale, 1e-3f);
        l.brokeBecause = l.force / (l.maxForce * scale) >= l.moment / (l.maxMoment * scale)
            ? $"{l.force:0} N > {l.maxForce * scale:0} N"
            : $"{l.moment:0} N·m bending > {l.maxMoment * scale:0} N·m";
        l.broken = true;

        var b = l.body;
        var go = b.gameObject;
        Vector3 v = kin.TryGetValue(b, out var k) ? k.vel : Vector3.zero, w = b.angularVelocity;
        go.SetActive(false);                      // leaves the articulation (the rover's, if mounted)
        go.transform.SetParent(null, true);
        if (go.scene != gameObject.scene && gameObject.scene.IsValid()) SceneManager.MoveGameObjectToScene(go, gameObject.scene);
        b.immovable = false;
        b.useGravity = true;
        go.SetActive(true);                       // its own articulation now, nothing drives it
        b.velocity = v;
        b.angularVelocity = w;
        debris.Add(go);

        RefreshSubtrees();
        foreach (var kk in kin.Values) kk.samples = 0;   // the break itself is not an acceleration
        var ac = GetComponent<ArmController>();
        if (ac != null) ac.RefreshDrives();
        Debug.Log($"[ArmBreakage] {NiceName(l.name)} ({l.name}) broke: {l.brokeBecause}" + (LogDetails ? "\n  " + Describe(l) : ""));
        Broke?.Invoke(this, l);
    }

    /// <summary>Put a fresh arm back where this one was (home pose, nothing broken, same settings). Returns the new arm.</summary>
    public ArmBreakage Repair()
    {
        if (spare == null) { Debug.LogWarning("[ArmBreakage] no spare to repair from"); return this; }
        var fresh = spare;
        spare = null;
        // Carry over what may have been changed in the menu.
        var fb = fresh.GetComponent<ArmBreakage>();
        JsonUtility.FromJsonOverwrite(JsonUtility.ToJson(this), fb);
        var ac = GetComponent<ArmController>();
        if (ac != null && fresh.TryGetComponent(out ArmController fac)) ac.CopySettingsTo(fac);

        if (homeParent == null && transform.parent != null) homeParent = transform.parent;
        gameObject.SetActive(false);
        fresh.SetActive(false);   // so it wakes up only once it is in place
        fresh.transform.SetParent(homeParent, false);
        fresh.transform.localPosition = homeLocalPos;
        fresh.transform.localRotation = homeLocalRot;
        if (homeParent == null && gameObject.scene.IsValid()) SceneManager.MoveGameObjectToScene(fresh, gameObject.scene);
        fresh.SetActive(true);   // the spare was copied before anything moved: home pose, same mount settings
        // On a rover: leave out the rover parts it overlaps again (RoverPayload.Apply did that for the old colliders only;
        // without it the solver pushes e.g. a camera on the deck plate out of the arm and the fresh arm breaks at once).
        var payload = homeParent != null ? homeParent.GetComponentInParent<RoverPayload>() : null;
        if (payload != null) payload.Mounted(fresh);
        Destroy(gameObject);
        Debug.Log("[ArmBreakage] arm repaired");
        return fb;
    }

    void OnDestroy()
    {
        foreach (var d in debris) if (d != null) Destroy(d);
        debris.Clear();
        if (spareHolder != null) Destroy(spareHolder);
    }
}
