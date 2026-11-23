using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// Esc menu, arm parts: WORLD > ROVER > CONFIGURATION (what the rover carries: nothing / arm / rover lab), the arm's
/// state + FIX ARM, the ADVANCED > ARM page (stepper specs, gripper, breakage strengths, live loads), and a toast when
/// a link breaks.
/// </summary>
public partial class SimMenu
{
    int pendingConfig = -1;   // applied in Update, not in the middle of drawing

    void OnArmBroke(ArmBreakage arm, ArmBreakage.Link l)
    {
        Toast($"ARM BROKE: {ArmBreakage.NiceName(l.name).ToUpperInvariant()} ({l.brokeBecause})  ·  FIX: ESC > WORLD");
    }

    void ApplyPendingConfig()
    {
        if (pendingConfig < 0) return;
        var c = (RoverPayload.Configuration)pendingConfig;
        pendingConfig = -1;
        var p = CurrentPayload();
        if (p == null) return;
        if (p.Apply(c)) Toast("CONFIGURATION: " + RoverPayload.Labels[(int)c]);
        else Toast("THIS ROVER CAN'T CARRY THAT");
    }

    RoverPayload CurrentPayload() =>
        rovers.Count > 0 && roverIndex < rovers.Count && rovers[roverIndex] != null ? rovers[roverIndex].root.GetComponent<RoverPayload>() : null;

    /// <summary>The arm the menu talks about: the selected rover's, else any in the scene (ArmScene).</summary>
    ArmBreakage CurrentArm()
    {
        var rover = rovers.Count > 0 && roverIndex < rovers.Count && rovers[roverIndex] != null ? rovers[roverIndex].root : null;
        var a = rover != null ? rover.GetComponentInChildren<ArmBreakage>() : null;
        return a != null ? a : FindObjectOfType<ArmBreakage>();
    }

    /// <summary>Segmented choice with some options greyed out.</summary>
    int SegEnabled(int selected, string[] options, bool[] enabled)
    {
        int result = selected;
        for (int i = 0; i < options.Length; i++)
        {
            bool was = GUI.enabled;
            GUI.enabled = was && enabled[i];
            var old = GUI.color;
            if (!enabled[i]) GUI.color = new Color(1, 1, 1, 0.35f);
            if (GUILayout.Button(options[i], i == selected ? sSegOn : sSegOff, GUILayout.Height(28), GUILayout.MinWidth(56))) result = i;
            GUI.color = old;
            GUI.enabled = was;
        }
        return result;
    }

    // WORLD tab, under ROVER > MODEL.
    void DrawConfiguration()
    {
        if (rovers.Count == 0 || roverIndex >= rovers.Count || rovers[roverIndex] == null) return;
        if (rovers[roverIndex].root.GetComponentInChildren<MotorController>() == null) return;   // e.g. ArmScene's arm on a table
        var p = CurrentPayload();
        Row("CONFIGURATION");
        int cur = p != null ? (int)p.configuration : 0;
        var enabled = new bool[RoverPayload.Labels.Length];
        for (int i = 0; i < enabled.Length; i++)
        {
            var c = (RoverPayload.Configuration)i;
            enabled[i] = c == RoverPayload.Configuration.None || (p != null && RoverPayload.Implemented(c) && p.Supports(c));
        }
        int n = SegEnabled(cur, RoverPayload.Labels, enabled);
        EndRow();
        if (n != cur && p != null) pendingConfig = n;
        Hint(p == null
            ? "This rover carries nothing (the mini rover was never meant to carry the full arm)."
            : "ARM mounts the arm on the deck (same topics as the real one: /arm/command, /joint_states). ROVER LAB is not modelled yet.");
    }

    // WORLD tab: the arm's state and FIX ARM.
    void DrawArmStatus()
    {
        var arm = CurrentArm();
        if (arm == null) return;
        Section("ARM");
        var broken = arm.links.Where(l => l.broken).ToArray();
        Row("STATE");
        GUILayout.Label(broken.Length == 0 ? "<color=#5ddcf2>INTACT</color>"
                                           : "<color=#ff6050>BROKEN</color>  " + string.Join(", ", broken.Select(l => ArmBreakage.NiceName(l.name))), sLabel, GUILayout.Height(28));
        GUILayout.Space(12);
        if (GUILayout.Button("FIX ARM", sSegOff, GUILayout.Width(120), GUILayout.Height(28))) { arm.Repair(); Toast("ARM FIXED"); }
        EndRow();
        Row("BREAKAGE");
        bool on = Seg(arm.breakable ? 1 : 0, new[] { "OFF", "ON" }) == 1;
        if (on != arm.breakable) arm.breakable = on;
        EndRow();
        Hint("FIX ARM puts a fresh arm back in its home pose (all joints at zero) with the same settings. Strengths, motors and " +
             "the gripper: ADVANCED > ARM.");
    }

    // ---------------------------------------------------------------- ADVANCED > ARM

    void AdvArm()
    {
        var arm = CurrentArm();
        if (arm == null)
        {
            Hint("No arm in this scene. Mount one on chassis_2024: WORLD > ROVER > CONFIGURATION > ARM. ArmScene has one on a table.");
            return;
        }
        var ac = arm.GetComponent<ArmController>();
        GUILayout.Label($"ARM  <color=#ffad2e>{(arm.transform.root != arm.transform ? arm.transform.root.name.ToUpperInvariant() + " / " : "")}{arm.name.ToUpperInvariant()}</color>", sLabel);

        Section("STATUS");
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("FIX ARM", sSegOff, GUILayout.Width(120), GUILayout.Height(28))) { arm.Repair(); Toast("ARM FIXED"); }
        GUILayout.Space(8);
        if (GUILayout.Button("RESET PEAKS", sSegOff, GUILayout.Width(140), GUILayout.Height(28))) arm.ResetPeaks();
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();
        if (ac != null)
        {
            for (int i = 0; i < ArmController.NUM_CMD; i++)
            {
                var d = ac.Drive(i);
                if (d == null) continue;
                string state = !ac.Powered(i) ? "<color=#ff6050>NO POWER (broken off)</color>"
                             : d.Slipping ? "<color=#ffad2e>STALLED, skipping steps</color>" : "ok";
                string extra = i == ArmController.EE
                    ? $"   squeeze {Mathf.Abs(d.outputs[0].force):0} / {Mathf.Abs(d.outputs[1].force):0} N (max ~{ac.MaxGripForce:0} N)"
                    : "";
                Readout($"<b>{ArmController.AxisNames[i]}</b>  {ac.Reading(i),7:0.0}°   torque {d.torque,6:0.0} of {d.available:0.0} N·m   lost steps {d.lostSteps}   {state}{extra}");
            }
        }

        Section("BREAKAGE");
        Fields(arm, only: new[] { "breakable", "strengthScale", "filterTime" });
        Hint("Load through each joint = what it carries + contacts (bending only: the motor's own axis stalls instead). " +
             "Now / peak are % of the limit. Strengths are estimates; driving over sharp 4 cm steps peaks around 20-60 %.");
        foreach (var l in arm.links)
        {
            if (l.body == null) continue;
            SubHead($"{ArmBreakage.NiceName(l.name)} ({l.name})   now {l.load * 100:0} %   peak {l.peak * 100:0} %" +
                    (l.broken ? "   BROKEN: " + l.brokeBecause : l.fixedToWorld ? "   (fixed to the world here)" : ""));
            string id = arm.GetInstanceID() + ":" + l.name;
            var link = l;
            Custom(id + ":F", "Max force (N)", "Force through the joint that breaks it.", () => link.maxForce, v => link.maxForce = v, min: 1);
            Custom(id + ":M", "Max bending moment (N·m)", "Bending moment that breaks it (not about the motor's axis).", () => link.maxMoment, v => link.maxMoment = v, min: 0.1f);
        }

        if (ac == null) return;
        Section("GRIPPER");
        Readout($"Stall squeeze ~<b>{ac.MaxGripForce:0} N</b> per finger (gripper motor's output torque / finger travel). Closing on " +
                "something builds force through the finger flex until the motor stalls; it keeps squeezing after the command stops.");
        Fields(ac, only: new[] { "fingerStiffness", "eeMetersPerDegree" }, after: ac.RefreshDrives);
        var pad = ac.Fingers.Where(f => f != null).SelectMany(f => f.GetComponentsInChildren<Collider>()).Select(c => c.sharedMaterial).FirstOrDefault(m => m != null);
        if (pad != null) PhysicMaterialKnobs(pad, "FINGER PADS  " + pad.name.ToUpperInvariant());

        Section("COMMANDS");
        Fields(ac, only: new[] { "positionGain" });

        // One section per motor spec (several axes may share one).
        for (int i = 0; i < ArmController.NUM_CMD; i++)
        {
            var spec = ac.motors[i];
            if (spec == null || Array.IndexOf(ac.motors, spec) != i) continue;
            var axes = Enumerable.Range(0, ArmController.NUM_CMD).Where(k => ac.motors[k] == spec).Select(k => ArmController.AxisNames[k]);
            Section("MOTOR  " + spec.name.ToUpperInvariant() + "  (" + string.Join(", ", axes).ToUpperInvariant() + ")");
            Readout($"Output: holding <b>{spec.OutputHoldingTorque:0.0} N·m</b>, top speed {spec.MaxOutputDegPerSec:0} deg/s, " +
                    $"one step = {spec.OutputStepDeg * 1000:0.#} mdeg, stiffness {spec.outputStiffness:0} N·m/deg (gearbox + link).");
            if (!string.IsNullOrEmpty(spec.notes)) Hint(spec.notes);
            Fields(spec, after: () => { foreach (var c in FindObjectsOfType<ArmController>()) c.RefreshDrives(); });
        }
    }
}
