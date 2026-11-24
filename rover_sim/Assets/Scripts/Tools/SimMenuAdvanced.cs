using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Reflection;
using UnityEngine;
using Object = UnityEngine.Object;

/// <summary>
/// Esc menu > ADVANCED: every runtime knob of the drivetrain, tyres, rover body, soil, physics and the arm (SimMenuArm.cs), for quick tests.
/// Nothing is saved. Spec assets (motors, wheels, grounds, physic materials) and global physics settings are put back
/// when the sim quits, so in the editor a test never changes the project; components reset with Play mode anyway.
///
/// Knobs come from the specs' own fields by reflection (inspector order, [Header] as sub-headings, [Tooltip] shown
/// when hovered), so a field added to a spec shows up here without menu changes. Fields only used when a rover is
/// set up in the editor (wheel radius, collider shape...) are shown read-only.
/// </summary>
public partial class SimMenu
{
    bool advanced;
    int advTab;
    static readonly string[] AdvTabs = { "MOTORS", "WHEELS & TYRES", "ROVER BODY", "SOIL", "PHYSICS", "ARM" };

    class Knob
    {
        public string key, label, tip, note;
        public string[] options;                 // enum / choice: value = index
        public Func<float> get;
        public Action<float> set;
        public Action after;
        public bool isBool, isInt, readOnly, restoreOnQuit;
        public float min = float.NegativeInfinity, max = float.PositiveInfinity;
        public bool slider;
        public float original;
    }

    readonly Dictionary<string, Knob> knobs = new Dictionary<string, Knob>();
    string editKey, editText;
    GUIStyle sField, sChanged;
    int groundPick = -1;

    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    const BindingFlags InstanceFields = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
    // Spec fields that may be negative; every other spec number is clamped at 0.
    static readonly HashSet<string> MayBeNegative = new HashSet<string> { "gripPressureExponent" };
    // Spec fields that divide something: must stay above 0.
    static readonly HashSet<string> MustBePositive = new HashSet<string>
        { "gearRatio", "torqueConstant", "terminalResistance", "nominalVoltage", "noLoadSpeedRpm", "supplyVoltage", "gearEfficiency",
          "rubberThickness", "n", "shearModulus", "kphi" };

    // ---------------------------------------------------------------- knob store

    Knob K(string key, Func<Knob> make)
    {
        if (!knobs.TryGetValue(key, out var k))
        {
            k = make();
            k.key = key;
            k.original = k.get();
            knobs[key] = k;
        }
        return k;
    }

    static bool IsAsset(Object o) => o != null && !(o is Component) && !(o is GameObject);

    void SetKnob(Knob k, float v)
    {
        if (k.isInt || k.isBool || k.options != null) v = Mathf.Round(v);
        v = Mathf.Clamp(v, k.min, k.max);
        try { k.set(v); k.after?.Invoke(); }
        catch (Exception e) { Debug.LogWarning($"[SimMenu] {k.label}: {e.Message}"); }
    }

    /// <summary>Self-test (--sim-menu-shots): a spec change reaches the physics, RESET ALL undoes it. Run after the MOTORS page was drawn.</summary>
    System.Collections.IEnumerator AdvancedSelfTest()
    {
        var dw = rovers.Count > 0 ? rovers[roverIndex].root.GetComponentInChildren<DriveWheel>() : null;
        if (dw == null || dw.motor == null || !knobs.TryGetValue(dw.motor.GetInstanceID() + ":supplyVoltage", out var k)) { Debug.Log("[SimMenu] advanced self-test skipped"); yield break; }
        var body = rovers[roverIndex];
        float oldSpeed = keyboardSpeed; keyboardSpeed = 5f;
        float v0 = k.get(), d1 = 0, d2 = 0;
        for (int pass = 0; pass < 2; pass++)
        {
            if (pass == 1) SetKnob(k, v0 * 2);
            for (float t = 0; t < 1.5f; t += Time.deltaTime) { Drive(1, 0, false); yield return null; }   // get up to speed
            Vector3 p0 = body.position;
            for (float t = 0; t < 2f; t += Time.deltaTime) { Drive(1, 0, false); yield return null; }
            float d = Vector3.Distance(p0, body.position) / 2f;
            if (pass == 0) d1 = d; else d2 = d;
            Drive(0, 0, false);
            for (int i = 0; i < 60; i++) yield return null;
        }
        keyboardSpeed = oldSpeed;
        ResetAllKnobs();
        Debug.Log($"[SimMenu] advanced self-test: supply voltage {v0} V -> {v0 * 2} V: top speed {d1:0.00} -> {d2:0.00} m/s " +
                  $"(motor says {dw.motor.NoLoadWheelSpeed * dw.Radius:0.00} -> {2 * dw.motor.NoLoadWheelSpeed * dw.Radius:0.00}); after RESET ALL {k.get()} V");
    }

    void ResetAllKnobs()
    {
        foreach (var k in knobs.Values)
        {
            try { if (!Same(k.get(), k.original)) SetKnob(k, k.original); }
            catch (Exception) { } // its object is gone (rover swapped)
        }
        Toast("ALL ADVANCED SETTINGS RESET");
    }

    /// <summary>Spec assets and global settings back to how they were (editor: the project stays unchanged).</summary>
    void RestoreAssetKnobs()
    {
        foreach (var k in knobs.Values)
            if (k.restoreOnQuit)
                try { if (!Same(k.get(), k.original)) k.set(k.original); } catch (Exception) { }
    }

    // Component knobs belong to the old scene; asset knobs (and their originals) stay.
    void ForgetSceneKnobs()
    {
        foreach (var key in knobs.Where(kv => !kv.Value.restoreOnQuit).Select(kv => kv.Key).ToList()) knobs.Remove(key);
        editKey = null;
        groundPick = -1;
    }

    static bool Same(float a, float b) => Mathf.Abs(a - b) <= 1e-6f * Mathf.Max(1f, Mathf.Abs(a), Mathf.Abs(b));

    // ---------------------------------------------------------------- knobs from fields (reflection)

    static readonly Dictionary<string, string> Names = new Dictionary<string, string>
        { { "shoreA", "Shore A hardness" }, { "kc", "kc" }, { "kphi", "kphi" }, { "n", "n (sinkage exponent)" } };

    static string Nice(string name)
    {
        if (Names.TryGetValue(name, out var fixedName)) return fixedName;
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < name.Length; i++)
        {
            char c = name[i];
            if (c == '_') { sb.Append(' '); continue; }
            if (i > 0 && char.IsUpper(c) && name[i - 1] != '_' && !char.IsUpper(name[i - 1])) sb.Append(' ');
            sb.Append(i == 0 ? char.ToUpperInvariant(c) : char.ToLowerInvariant(c));
        }
        return sb.ToString();
    }

    static bool Editable(FieldInfo f)
    {
        if (f.IsNotSerialized || f.IsDefined(typeof(HideInInspector))) return false;
        if (!f.IsPublic && !f.IsDefined(typeof(SerializeField))) return false;
        var t = f.FieldType;
        return t == typeof(float) || t == typeof(int) || t == typeof(bool) || t == typeof(double) || t.IsEnum;
    }

    Knob FieldKnob(Object target, FieldInfo f, IEnumerable<Object> alsoSet)
    {
        var t = f.FieldType;
        var k = new Knob
        {
            label = Nice(f.Name),
            tip = f.GetCustomAttribute<TooltipAttribute>()?.tooltip,
            isBool = t == typeof(bool),
            isInt = t == typeof(int),
            restoreOnQuit = IsAsset(target),
        };
        var range = f.GetCustomAttribute<RangeAttribute>();
        if (range != null) { k.min = range.min; k.max = range.max; k.slider = true; }
        else if (target is ScriptableObject && !MayBeNegative.Contains(f.Name)) k.min = MustBePositive.Contains(f.Name) ? 1e-3f : 0f;
        var others = alsoSet?.Where(o => o != null && o != target).ToArray();
        if (t.IsEnum)
        {
            var values = Enum.GetValues(t);
            k.options = Enum.GetNames(t).Select(n => Nice(n).ToUpperInvariant()).ToArray();
            k.get = () => Array.IndexOf(values, f.GetValue(target));
            k.set = v => { var o = values.GetValue(Mathf.Clamp((int)v, 0, values.Length - 1)); f.SetValue(target, o); if (others != null) foreach (var a in others) f.SetValue(a, o); };
            return k;
        }
        k.get = () => { var o = f.GetValue(target); return o is bool b ? (b ? 1 : 0) : Convert.ToSingle(o, Inv); };
        k.set = v =>
        {
            object o = t == typeof(bool) ? v > 0.5f : t == typeof(int) ? Mathf.RoundToInt(v) : t == typeof(double) ? (double)v : (object)v;
            f.SetValue(target, o);
            if (others != null) foreach (var a in others) f.SetValue(a, o);
        };
        return k;
    }

    /// <summary>Knobs for the editable fields of <paramref name="target"/>, inspector order. <paramref name="alsoSet"/> get the
    /// same value (all six wheels). Fields in <paramref name="fixedFields"/> are shown read-only with <paramref name="fixedNote"/>.</summary>
    void Fields(Object target, string[] only = null, string[] skip = null, string[] fixedFields = null, string fixedNote = null,
                Action after = null, IEnumerable<Object> alsoSet = null, Dictionary<string, string> notes = null)
    {
        if (target == null) return;
        foreach (var f in target.GetType().GetFields(InstanceFields))
        {
            if (!Editable(f)) continue;
            if (only != null && Array.IndexOf(only, f.Name) < 0) continue;
            if (skip != null && Array.IndexOf(skip, f.Name) >= 0) continue;
            var header = f.GetCustomAttribute<HeaderAttribute>();
            if (header != null && only == null) SubHead(header.header);
            var k = K(target.GetInstanceID() + ":" + f.Name, () => FieldKnob(target, f, alsoSet));
            k.readOnly = fixedFields != null && Array.IndexOf(fixedFields, f.Name) >= 0;
            k.note = k.readOnly ? fixedNote : notes != null && notes.TryGetValue(f.Name, out var n) ? n : null;
            k.after = after;
            DrawKnob(k);
        }
    }

    Knob Custom(string key, string label, string tip, Func<float> get, Action<float> set, bool restore = false,
                float min = float.NegativeInfinity, float max = float.PositiveInfinity, bool isInt = false, bool slider = false, string[] options = null)
    {
        var k = K(key, () => new Knob { label = label, tip = tip, get = get, set = set, restoreOnQuit = restore, min = min, max = max, isInt = isInt, slider = slider, options = options });
        DrawKnob(k);
        return k;
    }

    // ---------------------------------------------------------------- drawing

    void SubHead(string title)
    {
        GUILayout.Space(4);
        GUILayout.Label(title.ToUpperInvariant(), sHead);
    }

    void Readout(string text) => GUILayout.Label(text, sLabel);

    static string Fmt(float v, bool isInt)
    {
        if (isInt) return Mathf.RoundToInt(v).ToString(Inv);
        float a = Mathf.Abs(v);
        return v.ToString(a >= 1000 ? "0.#" : a >= 1 ? "0.####" : "0.######", Inv);
    }

    void DrawKnob(Knob k)
    {
        float v;
        try { v = k.get(); } catch (Exception) { return; }
        bool changed = !Same(v, k.original);
        GUILayout.BeginHorizontal(GUILayout.MinHeight(28));
        GUILayout.Label(new GUIContent(k.label, k.tip), changed ? sChanged : sLabel, GUILayout.Width(280), GUILayout.Height(26));

        if (k.readOnly)
        {
            GUILayout.Label(k.options != null ? k.options[Mathf.Clamp((int)v, 0, k.options.Length - 1)] : k.isBool ? (v > 0.5f ? "ON" : "OFF") : Fmt(v, k.isInt), sHead, GUILayout.Width(120), GUILayout.Height(26));
            if (k.note != null) GUILayout.Label(k.note, sDim, GUILayout.Height(26));
        }
        else if (k.isBool || k.options != null)
        {
            int cur = Mathf.RoundToInt(v);
            int n = Seg(cur, k.options ?? new[] { "OFF", "ON" });
            if (n != cur) SetKnob(k, n);
        }
        else
        {
            float step = k.isInt ? 1f : v == 0 ? 0.01f : Mathf.Abs(v) * 0.1f;
            if (GUILayout.Button("−", sSegOff, GUILayout.Width(30), GUILayout.Height(26))) SetKnob(k, v - step);
            string name = "knob:" + k.key;
            bool focused = GUI.GetNameOfFocusedControl() == name;
            if (focused && editKey != k.key) { CommitEdit(); editKey = k.key; editText = Fmt(v, k.isInt); }
            GUI.SetNextControlName(name);
            string shown = focused && editKey == k.key ? editText : Fmt(v, k.isInt);
            string typed = GUILayout.TextField(shown, sField, GUILayout.Width(110), GUILayout.Height(26));
            if (focused && editKey == k.key) editText = typed;
            if (GUILayout.Button("+", sSegOff, GUILayout.Width(30), GUILayout.Height(26))) SetKnob(k, v + step);
            if (k.slider && !float.IsInfinity(k.min) && !float.IsInfinity(k.max))
            {
                GUILayout.Space(6);
                float s = GUILayout.HorizontalSlider(v, k.min, k.max, sSlider, sThumb, GUILayout.Width(150));
                if (!Same(s, v)) SetKnob(k, s);
            }
        }
        if (changed && !k.readOnly)
        {
            GUILayout.Space(8);
            GUILayout.Label("was " + (k.options != null ? k.options[Mathf.Clamp((int)k.original, 0, k.options.Length - 1)] : k.isBool ? (k.original > 0.5f ? "ON" : "OFF") : Fmt(k.original, k.isInt)), sDim, GUILayout.Width(100));
            if (GUILayout.Button("RESET", sSegOff, GUILayout.Height(26))) SetKnob(k, k.original);
        }
        else if (k.note != null && !k.readOnly) { GUILayout.Space(8); GUILayout.Label(k.note, sDim, GUILayout.Height(26)); }
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();
    }

    void CommitEdit()
    {
        if (editKey == null) return;
        string key = editKey, text = editText;
        editKey = null;
        if (!knobs.TryGetValue(key, out var k)) return;
        if (float.TryParse(text?.Replace(',', '.'), NumberStyles.Float, Inv, out float v))
        {
            float cur;
            try { cur = k.get(); } catch (Exception) { return; }
            if (!Same(v, cur)) SetKnob(k, v);
        }
    }

    void DrawAdvanced(Rect panel, float x0, float tabY)
    {
        if (sField == null)
        {
            sField = new GUIStyle(sSegOff) { alignment = TextAnchor.MiddleRight, clipping = TextClipping.Clip };
            sField.normal.textColor = sField.hover.textColor = sField.focused.textColor = Color.white;
            sField.focused.background = sSegOff.hover.background;
            sChanged = new GUIStyle(sLabel);
            sChanged.normal.textColor = Orange;
        }
        // Enter, or clicking elsewhere, applies a typed value.
        var e = Event.current;
        if (editKey != null && e.type == EventType.KeyDown && (e.keyCode == KeyCode.Return || e.keyCode == KeyCode.KeypadEnter))
        {
            CommitEdit(); GUI.FocusControl(null); e.Use();
        }
        if (editKey != null && GUI.GetNameOfFocusedControl() != "knob:" + editKey) CommitEdit();

        float tabW = 124;
        for (int i = 0; i < AdvTabs.Length; i++)
            if (GUI.Button(new Rect(x0 + i * (tabW + 6), tabY, tabW, 32), AdvTabs[i], i == advTab ? sTabOn : sTabOff)) { advTab = i; scroll = Vector2.zero; GUI.FocusControl(null); }
        if (GUI.Button(new Rect(panel.xMax - 208, tabY, 180, 32), "◂  MAIN MENU", sTabOff)) { advanced = false; scroll = Vector2.zero; GUI.FocusControl(null); }
        Fill(new Rect(panel.x + 12, tabY + 32, panel.width - 24, 2), Cyan);

        // Not-saved banner + which rover.
        float y = tabY + 42, w = panel.width - 56;
        Fill(new Rect(x0, y, w, 46), new Color(1f, 0.68f, 0.18f, 0.12f));
        Fill(new Rect(x0, y, 4, 46), Orange);
        GUI.Label(new Rect(x0 + 14, y + 3, w - 160, 22), "<b>NOTHING HERE IS SAVED.</b>  Quick tests only: every change is lost when Play stops or the sim closes.", sWarn);
        GUI.Label(new Rect(x0 + 14, y + 23, w - 160, 20), "To change a rover for good, edit its spec assets in the Unity editor (Assets/Robots/Drivetrain, Assets/Robots/Arm/Motors).", sDim);
        if (GUI.Button(new Rect(x0 + w - 140, y + 9, 130, 28), "RESET ALL", sSegOff)) ResetAllKnobs();

        var rover = rovers.Count > 0 && roverIndex < rovers.Count && rovers[roverIndex] != null ? rovers[roverIndex].root.gameObject : null;
        var content = new Rect(x0, y + 54, w, panel.yMax - (y + 54) - 80);
        GUILayout.BeginArea(content);
        scroll = GUILayout.BeginScrollView(scroll, false, false, GUIStyle.none, GUI.skin.verticalScrollbar);
        switch (advTab)
        {
            case 0: AdvMotors(rover); break;
            case 1: AdvWheels(rover); break;
            case 2: AdvBody(rover); break;
            case 3: AdvSoil(rover); break;
            case 4: AdvPhysics(); break;
            case 5: AdvArm(); break;
        }
        GUILayout.EndScrollView();
        GUILayout.EndArea();

        // Hovered knob's explanation.
        string tip = GUI.tooltip;
        Fill(new Rect(x0, panel.yMax - 76, w, 1), CyanDim);
        GUI.Label(new Rect(x0, panel.yMax - 74, w, 34), string.IsNullOrEmpty(tip) ? "Hover over a setting for what it does.  Type a value + Enter, or use − / +.  Changed values are orange." : tip, sDim);
    }

    // ---------------------------------------------------------------- pages

    static DriveWheel[] Wheels(GameObject rover) => rover != null ? rover.GetComponentsInChildren<DriveWheel>() : new DriveWheel[0];

    void RoverLine(GameObject rover)
    {
        if (rover == null) return;
        GUILayout.Label($"ROVER  <color=#ffad2e>{rover.name.ToUpperInvariant()}</color>" + (rovers.Count > 1 ? "   (pick another in CAMERA > ROVER)" : ""), sLabel);
    }

    void AdvMotors(GameObject rover)
    {
        var wheels = Wheels(rover);
        RoverLine(rover);
        if (wheels.Length == 0) { Hint("No rover with drivetrain-model wheels (DriveWheel) in this scene."); return; }
        foreach (var m in wheels.Select(dw => dw.motor).Where(m => m != null).Distinct())
        {
            Section("MOTOR  " + m.name.ToUpperInvariant());
            float r = wheels[0].Radius;
            Readout($"Top speed <b>{m.NoLoadWheelSpeed * r:0.00} m/s</b> (no-load {m.NoLoadWheelSpeed:0.0} rad/s at the wheel, radius {r * 1000:0} mm).  " +
                    $"Per wheel: peak <b>{m.PeakWheelTorque:0.00} N·m</b>, continuous {m.ContinuousWheelTorque:0.00} N·m.  " +
                    $"Six wheels push at most ~{6 * m.PeakWheelTorque / Mathf.Max(r, 1e-3f):0} N.");
            if (!string.IsNullOrEmpty(m.notes)) Hint(m.notes);
            Readout($"Speed ramp (maxWheelAcceleration {m.maxWheelAcceleration:0} rad/s² = {m.maxWheelAcceleration * r:0.0} m/s²): " +
                    $"<b>{m.NoLoadWheelSpeed / Mathf.Max(m.maxWheelAcceleration, 1e-3f):0.00} s</b> from 0 to top speed, and the same to stop. " +
                    "Raise it with the motor or the rover feels laggy at high speed.");
            Hint("The wheels follow the COMMANDED speed (/cmd_vel, or CONTROLS > KEYBOARD DRIVE speed x2 with Shift). A faster motor " +
                 "only shows when the command asks for more than the old top speed, or on climbs and in soil where torque runs out.");
            Fields(m, skip: null, fixedFields: new[] { "rotorInertia", "gearheadInertia" }, fixedNote: "added to the wheels' inertia at start");
        }
        Section("ALL SIX WHEELS");
        Fields(wheels[0], only: new[] { "ambientTemperature", "thermalModel", "rollingResistance" }, alsoSet: wheels);
        Hint("F8 in the sim shows each wheel's torque, current, winding temperature and slip live.");
    }

    void AdvWheels(GameObject rover)
    {
        var wheels = Wheels(rover);
        RoverLine(rover);
        if (wheels.Length == 0) { Hint("No rover with drivetrain-model wheels (DriveWheel) in this scene."); return; }
        Action resoil = () => { foreach (var w in FindObjectsOfType<DriveWheel>()) w.ReconfigureSoil(); };
        foreach (var ws in wheels.Select(dw => dw.wheel).Where(w => w != null).Distinct())
        {
            Section("WHEEL  " + ws.name.ToUpperInvariant());
            var t = ws.tread;
            Readout($"Radius {wheels[0].Radius * 1000:0} mm" + (t != null && t.Valid ? $", tread {t.width * 1000:0} mm wide, {t.lugCount} lugs {t.lugHeight * 1000:0.0} mm, {t.landRatio * 100:0} % land" : ", no tread baked (smooth)") +
                    $".  Rubber modulus {ws.YoungsModulus / 1e6f:0.0} MPa (from Shore A).");
            if (!string.IsNullOrEmpty(ws.notes)) Hint(ws.notes);
            Fields(ws, fixedFields: new[] { "radius", "width", "shape", "grouserCount", "grouserHeight", "mass" },
                   fixedNote: "built into the rover in the editor", after: resoil);
            if (ws.material != null) PhysicMaterialKnobs(ws.material, "TREAD MATERIAL  " + ws.material.name.ToUpperInvariant() + "  (PhysX contacts: rocks, ramps, ground without soil)");
        }
        Section("WHEEL MASS");
        var b0 = wheels[0].Body;
        Custom(rover.GetInstanceID() + ":wheelMass", "Mass per wheel (kg)", "Sets all six wheel bodies. The rover's total is on ROVER BODY.",
               () => b0.mass, v => { foreach (var w in wheels) if (w != null) w.Body.mass = v; }, min: 0.01f);
        Hint("Tyre settings apply at once (the tyre model is rebuilt; on soil the rover may twitch for a step).");
    }

    void PhysicMaterialKnobs(PhysicMaterial m, string title)
    {
        SubHead(title);
        string id = m.GetInstanceID() + ":";
        Custom(id + "dyn", "Dynamic friction", "Friction while sliding. Combined with the other surface's by the combine mode.", () => m.dynamicFriction, v => m.dynamicFriction = v, true, 0, 5);
        Custom(id + "stat", "Static friction", "Friction before sliding starts.", () => m.staticFriction, v => m.staticFriction = v, true, 0, 5);
        Custom(id + "comb", "Friction combine", "How this friction is combined with the other surface's (the more specific mode wins: Maximum > Multiply > Minimum > Average).",
               () => (int)m.frictionCombine, v => m.frictionCombine = (PhysicMaterialCombine)(int)v, true, options: new[] { "AVERAGE", "MINIMUM", "MULTIPLY", "MAXIMUM" });
    }

    void AdvBody(GameObject rover)
    {
        RoverLine(rover);
        if (rover == null) { Hint("No rover in this scene."); return; }
        var bodies = rover.GetComponentsInChildren<ArticulationBody>();
        var root = bodies.FirstOrDefault(b => b.isRoot);
        var wheelBodies = new HashSet<ArticulationBody>(Wheels(rover).Select(w => w.Body));

        Section("MASS");
        Readout($"Total <b>{bodies.Sum(b => b.mass):0.0} kg</b> ({bodies.Length} bodies; wheels {wheelBodies.Sum(b => b.mass):0.0} kg). Inertia tensors stay as they are.");
        foreach (var b in bodies)
            Custom(b.GetInstanceID() + ":mass", b.name + (b == root ? "  (chassis)" : wheelBodies.Contains(b) ? "  (wheel)" : ""), "kg. A payload on the rover = more mass on the chassis.",
                   () => b.mass, v => b.mass = v, min: 0.01f);

        var mc = rover.GetComponentInChildren<MotorController>();
        if (mc != null)
        {
            Section("DRIVE CONTROLLER (/cmd_vel → wheels, odometry)");
            Fields(mc, only: new[] { "turnSlipFactor", "wheel_radius_meters", "track_width_meters", "commandTimeout", "publishOdometry", "publishOdomTf", "odomRateHz" },
                   notes: new Dictionary<string, string> { { "wheel_radius_meters", "measured at start" }, { "track_width_meters", "measured at start" } });
        }

        var rd = rover.GetComponentInChildren<RockerDifferential>();
        var suspension = bodies.Where(b => b != root && !wheelBodies.Contains(b) && b.jointType == ArticulationJointType.RevoluteJoint).ToArray();
        if (rd != null || suspension.Length > 0) Section("SUSPENSION");
        if (rd != null)
        {
            SubHead("Rocker differential (right rocker = - left)");
            Fields(rd, only: new[] { "stiffness", "damping", "forceLimit", "differentialMultiplier" }, after: () => ApplyDifferential(rd));
        }
        if (suspension.Length > 0)
        {
            var s0 = suspension[0];
            Custom(rover.GetInstanceID() + ":jf", "Joint friction (all " + suspension.Length + " joints)", "Friction in the rocker and bogie pivots (PhysX joint friction).",
                   () => s0.jointFriction, v => { foreach (var s in suspension) s.jointFriction = v; }, min: 0);
            var free = suspension.Where(s => rd == null || (s != rd.leftRocker && s != rd.rightRocker && s != rd.differential)).ToArray();
            if (free.Length > 0)
            {
                var f0 = free[0];
                Custom(rover.GetInstanceID() + ":bd", "Pivot damping (" + string.Join(", ", free.Select(s => s.name.Replace("_link", "").Replace("_Link", ""))) + ")",
                       "N·m per rad/s (drive damping). Damps bogie swing; 0 = free.", () => f0.xDrive.damping,
                       v => { foreach (var s in free) { var d = s.xDrive; d.damping = v; s.xDrive = d; } }, min: 0);
            }
            Custom(rover.GetInstanceID() + ":ad", "Angular damping (all bodies)", "Air drag on rotation, per body.",
                   () => s0.angularDamping, v => { foreach (var b in bodies) if (!wheelBodies.Contains(b)) b.angularDamping = v; }, min: 0);
        }

        if (root != null)
        {
            Section("SOLVER");
            Custom(root.GetInstanceID() + ":si", "Position iterations", "PhysX solver iterations for this rover. More = stiffer joints and drives, slower. Wheel drives stall at the default 6.",
                   () => root.solverIterations, v => root.solverIterations = (int)v, min: 1, max: 255, isInt: true);
            Custom(root.GetInstanceID() + ":svi", "Velocity iterations", "PhysX velocity iterations for this rover.",
                   () => root.solverVelocityIterations, v => root.solverVelocityIterations = (int)v, min: 1, max: 255, isInt: true);
        }
    }

    static void ApplyDifferential(RockerDifferential rd)
    {
        foreach (var j in new[] { rd.leftRocker, rd.rightRocker, rd.differential })
        {
            if (j == null) continue;
            var d = j.xDrive;
            d.stiffness = rd.stiffness; d.damping = rd.damping; d.forceLimit = rd.forceLimit;
            j.xDrive = d;
        }
    }

    List<GroundSpec> Grounds(out HashSet<GroundSpec> onSoil)
    {
        onSoil = new HashSet<GroundSpec>();
        var list = new List<GroundSpec>();
        foreach (var st in FindObjectsOfType<SoilTerrain>())
        {
            if (st.layerSpecs != null) foreach (var g in st.layerSpecs) if (g != null) onSoil.Add(g);
            if (st.defaultSpec != null) onSoil.Add(st.defaultSpec);
        }
        list.AddRange(onSoil.OrderBy(g => g.name));
        var table = Resources.Load<GroundSpecTable>("GroundSpecs");
        if (table != null)
        {
            if (table.defaultGround != null && !list.Contains(table.defaultGround)) list.Add(table.defaultGround);
            foreach (var g in table.grounds) if (g != null && !list.Contains(g)) list.Add(g);
        }
        return list;
    }

    void AdvSoil(GameObject rover)
    {
        var sw = SoilWorld.Instance;
        Section("SOIL MODEL");
        if (sw == null) Hint("No deformable soil in this scene: wheels use PhysX contacts. The ground types below still set rolling resistance and friction.");
        else
        {
            Action resoil = () => { foreach (var w in FindObjectsOfType<DriveWheel>()) w.ReconfigureSoil(); };
            Readout($"Quality <b>{sw.quality.ToString().ToUpperInvariant()}</b> (SOIL tab or F9).  {(sw.Active ? $"{sw.Current.cell * 100:0.#} cm cells, {sw.Current.nTheta}x{sw.Current.nY} tyre elements." : "")}");
            Fields(sw, only: new[] { "ruts", "stabilityFactor", "parallel" }, after: resoil);
            Fields(sw, only: new[] { "relief" }, notes: new Dictionary<string, string> { { "relief", "applies on CLEAR RUTS" } });
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("CLEAR RUTS + APPLY GROUND CHANGES", sSegOff, GUILayout.Height(30), GUILayout.Width(330)))
            {
                foreach (var st in FindObjectsOfType<SoilTerrain>()) st.Reinit();
                sw.ClearRuts();
                Toast("SOIL REBUILT");
            }
            GUILayout.Space(10);
            GUILayout.Label("Needed for relief, crust and roughness; everything else applies at once.", sDim);
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
        }

        var grounds = Grounds(out var onSoil);
        if (grounds.Count == 0) return;
        Section("GROUND TYPES");
        var wheels = Wheels(rover);
        GroundSpec under = wheels.Length > 0 ? (wheels[0].Soil.grounded ? wheels[0].Soil.ground : wheels[0].Contact.groundSpec) : null;
        if (groundPick < 0 || groundPick >= grounds.Count) groundPick = under != null && grounds.Contains(under) ? grounds.IndexOf(under) : 0;
        const int perRow = 4;
        for (int i = 0; i < grounds.Count; i += perRow)
        {
            GUILayout.BeginHorizontal();
            for (int j = i; j < Mathf.Min(i + perRow, grounds.Count); j++)
            {
                var g = grounds[j];
                string label = g.name + (g == under ? "  ◆" : "") + (onSoil.Contains(g) ? "" : "  (PhysX)");
                if (GUILayout.Button(label, j == groundPick ? sSegOn : sSegOff, GUILayout.Height(28), GUILayout.Width(235))) groundPick = j;
            }
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
        }
        Hint("◆ = under the rover now.  Soil ground types are this scene's soil layers (wheel-soil model); (PhysX) ones apply on colliders by their physic material.");

        var gs = grounds[groundPick];
        SubHead(gs.name);
        if (!string.IsNullOrEmpty(gs.notes)) Hint(gs.notes);
        Fields(gs, notes: new Dictionary<string, string>
        {
            { "crustStrength", "new ground: CLEAR RUTS" }, { "roughnessFine", "applies on CLEAR RUTS" },
            { "roughnessCoarse", "applies on CLEAR RUTS" }, { "disturbedDarkening", "applies on CLEAR RUTS" },
        });
        if (gs.material != null) PhysicMaterialKnobs(gs.material, "PHYSIC MATERIAL  " + gs.material.name.ToUpperInvariant() + "  (PhysX contacts)");
    }

    static readonly string[] GravityNames = { "EARTH", "MARS", "MOON" };
    static readonly float[] GravityValues = { 9.81f, 3.71f, 1.62f };

    void AdvPhysics()
    {
        Section("TIME");
        Custom("time:scale", "Simulation speed", "1 = real time. Slow motion to watch a wheel, faster to get somewhere. ROS timestamps stay wall-clock (no /clock).",
               () => Time.timeScale, v => Time.timeScale = v, true, 0.05f, 4f, slider: true);
        Custom("time:dt", "Physics step (ms)", "Fixed timestep. 5 ms is what the drivetrain and soil were tuned at; soil ULTRA uses 2.5 ms. Bigger = faster but less stable.",
               () => Time.fixedDeltaTime * 1000f, v => Time.fixedDeltaTime = v / 1000f, true, 0.5f, 40f);
        Readout($"Running at {fps:0} FPS, {1f / Mathf.Max(Time.fixedDeltaTime, 1e-4f):0} physics steps per simulated second.");

        Section("WORLD");
        var gk = Custom("phys:g", "Gravity (m/s²)", "Straight down. Earth 9.81, Mars 3.71, Moon 1.62. Soil strength does not change with it.",
                        () => -Physics.gravity.y, v => Physics.gravity = new Vector3(0, -v, 0), true, 0f, 30f);
        Row("PRESET", 280);
        int cur = Array.FindIndex(GravityValues, g => Mathf.Abs(g + Physics.gravity.y) < 0.01f);
        int n = Seg(cur, GravityNames);
        if (n != cur && n >= 0) SetKnob(gk, GravityValues[n]);
        EndRow();
        Custom("phys:bounce", "Bounce threshold (m/s)", "Contacts slower than this don't bounce.", () => Physics.bounceThreshold, v => Physics.bounceThreshold = v, true, 0f, 10f);
        Custom("phys:sleep", "Sleep threshold", "Energy below which bodies go to sleep (stop being simulated).", () => Physics.sleepThreshold, v => Physics.sleepThreshold = v, true, 0f, 1f);
    }
}
