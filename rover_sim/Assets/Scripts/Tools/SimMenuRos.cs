using System;
using SimFlake.Ros;
using UnityEngine;

/// <summary>
/// Esc menu, ROS tab: where the wheels take their commands from (RoverFlake2's wheel speeds or /cmd_vel directly), the
/// RoverFlake2 workspace, and ROS nodes started with a click (<see cref="RosNodeRunner"/>: presets + any typed command) with
/// their last output lines. Nodes are stopped when Play stops / the player quits.
/// </summary>
public partial class SimMenu
{
    static readonly string[] InputNames = { "ROVERFLAKE2 (motor_control_node)", "/CMD_VEL DIRECT (bypass)" };
    const string WheelSpeedCmd = "ros2 run drive_control wheel_speed_node";
    const string CustomPref = "SimFlake.RosCustomCommand";

    GUIStyle sRosField, sLog;
    string customCmd, wsPathEdit;
    int logNode;

    // IMGUI draws each frame in several passes that must see the same controls, but nodes start/exit on other threads.
    // So: actions from buttons run in Update, and drawing uses the running states captured there.
    readonly System.Collections.Generic.List<Action> rosActions = new System.Collections.Generic.List<Action>();
    readonly System.Collections.Generic.List<RosNodeRunner.Node> rosNodes = new System.Collections.Generic.List<RosNodeRunner.Node>();
    readonly System.Collections.Generic.HashSet<RosNodeRunner.Node> rosRunning = new System.Collections.Generic.HashSet<RosNodeRunner.Node>();

    void UpdateRos()
    {
        foreach (var a in rosActions) a();
        rosActions.Clear();
        rosNodes.Clear(); rosNodes.AddRange(RosNodeRunner.Nodes);
        rosRunning.Clear();
        foreach (var n in rosNodes) if (n.Running) rosRunning.Add(n);
    }

    RosNodeRunner.Node FindNode(string command) => rosNodes.Find(n => n.command == command);

    void RosStyles()
    {
        if (sRosField != null) return;
        sRosField = new GUIStyle(sSegOff) { alignment = TextAnchor.MiddleLeft, clipping = TextClipping.Clip };
        sRosField.normal.textColor = sRosField.hover.textColor = sRosField.focused.textColor = Color.white;
        sRosField.focused.background = sSegOff.hover.background;
        sLog = new GUIStyle(sSub) { wordWrap = true, richText = false, fontSize = 12 };
        sLog.normal.textColor = Text;
    }

    void DrawRos()
    {
        RosStyles();
        float w = Mathf.Min(900, viewW - 120);

        Section("DRIVE INPUT");
        Row("WHEELS LISTEN TO");
        var mode = (MotorController.DriveInput)Seg((int)MotorController.InputMode, InputNames);
        if (mode != MotorController.InputMode) MotorController.InputMode = mode;
        EndRow();
        if (mode == MotorController.DriveInput.RoverFlake2)
        {
            Hint("The sim is RoverFlake2's motor_control_node: it takes /left_wheel_speeds and /right_wheel_speeds (m/s) and " +
                 "publishes /drive/feedback. Run the rest of the drive stack as on the rover: /cmd_vel -> wheel_speed_node -> wheel speeds.");
            var ws = FindNode(WheelSpeedCmd);
            if (ws == null || !rosRunning.Contains(ws))
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label("/cmd_vel does nothing until wheel_speed_node runs.", sWarn, GUILayout.Height(28));
                GUILayout.Space(10);
                if (GUILayout.Button("START wheel_speed_node", sSegOff, GUILayout.Width(230), GUILayout.Height(28)))
                    rosActions.Add(() => { RosNodeRunner.Run("wheel_speed_node", WheelSpeedCmd); logNode = RosNodeRunner.Nodes.Count - 1; });
                GUILayout.FlexibleSpace();
                GUILayout.EndHorizontal();
            }
        }
        else
            Hint("Skips RoverFlake2's drive control: /cmd_vel (geometry_msgs/Twist, m/s and rad/s) drives the wheels directly " +
                 "(skid steer with the calibrated slip factor). Wheels stop 0.5 s after the last command. Wheel-speed topics are ignored.");
        Hint("Arrow keys always drive directly, in either mode.");

        Section("ROVERFLAKE2 WORKSPACE");
        Row("PATH");
        if (wsPathEdit == null) wsPathEdit = RosProcess.RoverFlake2Path;
        string ed = GUILayout.TextField(wsPathEdit, 300, sRosField, GUILayout.Width(360), GUILayout.Height(28));
        if (ed != wsPathEdit) { wsPathEdit = ed; RosProcess.RoverFlake2Path = ed.Trim(); }
        GUILayout.Space(12);
        GUILayout.Label(RosProcess.RoverFlake2Found ? "<color=#5ddcf2>install/setup.bash found</color>"
                                                   : "<color=#ff6050>no install/setup.bash here (colcon build it)</color>", sLabel, GUILayout.Height(28));
        EndRow();
        Hint("Sourced for the nodes below (and rosbridge, for rover_msgs) unless your environment already has RoverFlake2. " +
             "rosbridge only picks up a change after Play is restarted.");

        Section("NODES");
        foreach (var p in RosNodeRunner.Presets) NodeRow(p.label, p.command, p.blurb, w);
        foreach (var n in rosNodes)
        {
            bool preset = false;
            foreach (var p in RosNodeRunner.Presets) preset |= p.command == n.command;
            if (!preset) NodeRow(n.label, n.command, null, w);
        }

        Section("RUN A COMMAND");
        if (customCmd == null) customCmd = PlayerPrefs.GetString(CustomPref, "ros2 run ");
        GUILayout.BeginHorizontal();
        customCmd = GUILayout.TextField(customCmd, 500, sRosField, GUILayout.Width(w - 140), GUILayout.Height(28));
        GUILayout.Space(10);
        bool enter = Event.current.type == EventType.KeyDown && (Event.current.keyCode == KeyCode.Return || Event.current.keyCode == KeyCode.KeypadEnter)
                     && GUIUtility.keyboardControl != 0;
        if ((GUILayout.Button("RUN", sSegOff, GUILayout.Width(110), GUILayout.Height(28)) || enter) && customCmd.Trim().Length > 0)
        {
            string c = customCmd.Trim();
            PlayerPrefs.SetString(CustomPref, c); PlayerPrefs.Save();
            rosActions.Add(() => { logNode = IndexOf(RosNodeRunner.Run(c.Length > 40 ? c.Substring(0, 40) + "…" : c, c)); });
            GUIUtility.keyboardControl = 0;
        }
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();
        Hint("Any bash command with the ROS + RoverFlake2 environment: ros2 run, ros2 launch, ros2 topic pub ... " +
             "Commands that need a terminal (teleop_twist_keyboard) won't work here: use the arrow keys or a terminal.");

        var nodes = rosNodes;
        if (nodes.Count == 0) return;
        Section("OUTPUT");
        logNode = Mathf.Clamp(logNode, 0, nodes.Count - 1);
        var names = new string[nodes.Count];
        for (int i = 0; i < names.Length; i++) names[i] = nodes[i].label;
        GUILayout.BeginHorizontal();
        logNode = Seg(logNode, names);
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();
        var tail = nodes[logNode].Tail(14);
        GUILayout.Label(tail.Length > 0 ? string.Join("\n", tail) : "(no output yet)", sLog, GUILayout.Width(w));
    }

    void NodeRow(string label, string command, string blurb, float w)
    {
        var n = FindNode(command);
        bool running = n != null && rosRunning.Contains(n);
        GUILayout.BeginHorizontal(GUILayout.MinHeight(32));
        if (GUILayout.Button(running ? "STOP" : "START", running ? sSegOn : sSegOff, GUILayout.Width(90), GUILayout.Height(28)))
        {
            if (running) rosActions.Add(() => RosNodeRunner.Stop(n));
            else rosActions.Add(() => logNode = IndexOf(RosNodeRunner.Run(label, command)));
        }
        GUILayout.Space(12);
        string state = running ? (n.stopping ? "<color=#ffa040>STOPPING</color>" : "<color=#5ddcf2>RUNNING</color>")
                     : n == null ? "" : n.exitCode == int.MinValue ? "" : $"<color=#ff6050>EXITED ({n.exitCode})</color>";
        GUILayout.Label($"<b>{label}</b>  {state}", sLabel, GUILayout.Width(330), GUILayout.Height(28));
        if (n != null && GUILayout.Button("LOG", sSegOff, GUILayout.Width(60), GUILayout.Height(28))) logNode = IndexOf(n);
        if (n != null && blurb == null && GUILayout.Button("REMOVE", sSegOff, GUILayout.Width(90), GUILayout.Height(28)))
            rosActions.Add(() => { RosNodeRunner.Stop(n); RosNodeRunner.Forget(n); });
        GUILayout.FlexibleSpace();
        GUILayout.EndHorizontal();
        GUILayout.Label($"<color=#7a9499>{command}</color>{(blurb != null ? "  ·  " + blurb : "")}", sDim, GUILayout.Width(w));
        GUILayout.Space(4);
    }

    static int IndexOf(RosNodeRunner.Node n)
    {
        var l = RosNodeRunner.Nodes;
        for (int i = 0; i < l.Count; i++) if (l[i] == n) return i;
        return 0;
    }
}
