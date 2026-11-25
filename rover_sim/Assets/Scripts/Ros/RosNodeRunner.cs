using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace SimFlake.Ros
{
    /// <summary>
    /// ROS nodes started from the Esc menu's ROS tab (RoverFlake2 nodes, RViz, any typed command). Each runs as its own
    /// process with the same environment as rosbridge (<see cref="RosProcess"/>), keeps its last output lines for the menu,
    /// and is stopped when Play stops / the player quits (<see cref="StopAll"/>, called from <see cref="RosConnection"/>).
    /// </summary>
    public static class RosNodeRunner
    {
        public class Preset
        {
            public string label, command, blurb;
            public string group;   // starting a preset stops the running ones of the same group
            public Preset(string label, string command, string blurb, string group = null)
            { this.label = label; this.command = command; this.blurb = blurb; this.group = group; }
        }

        /// <summary>The menu's list. Commands are bash; add entries here.</summary>
        public static readonly Preset[] Presets =
        {
            new Preset("wheel_speed_node", "ros2 run drive_control wheel_speed_node",
                "RoverFlake2 drive: /cmd_vel -> left/right_wheel_speeds. With DRIVE INPUT = ROVERFLAKE2 the rover only moves on /cmd_vel while this runs, as on the real rover."),
            // Rate-capped on purpose. joy_linux's defaults (coalesce_interval 1 ms) publish /joy on every stick event, up to
            // ~1 kHz while a stick moves; drive_control_node and wheel_speed_node pass that on (x3 topics), and rosbridge falls
            // seconds behind above ~100 Hz: the rover then replays old commands ("phantom" driving). 50 ms coalescing + 20 Hz
            // autorepeat = at most ~20 Hz, steady while a stick is held. See claude/rosbridge-camera-starvation.md.
            new Preset("joy_linux_node",
                "ros2 run joy_linux joy_linux_node --ros-args -r __node:=joy_linux_node -p coalesce_interval:=0.05 -p autorepeat_rate:=20.0 -p deadzone:=0.08",
                "Gamepad -> /joy at <= 20 Hz (RoverFlake2's default joystick driver, rate-capped so rosbridge keeps up). Plug the controller in first.",
                "joy"),
            new Preset("joy_node",
                "ros2 run joy joy_node --ros-args -p coalesce_interval_ms:=50 -p autorepeat_rate:=20.0 -p deadzone:=0.08",
                "Gamepad -> /joy via the joy package (Switch controllers need this one), also capped at ~20 Hz. Starting it stops joy_linux_node: " +
                "two drivers on /joy double the rate and mix two different axis layouts (joy and joy_linux number the sticks differently).",
                "joy"),
            new Preset("drive_control_node", "ros2 run drive_control drive_control_node",
                "RoverFlake2 gamepad drive: /joy -> /cmd_vel (left stick forward, right stick turn). Needs a joy node and wheel_speed_node."),
            new Preset("joy_arm_control", "ros2 run arm_control joy_arm_control",
                "RoverFlake2 gamepad arm: /joy -> /arm/command. Needs a joy node and the arm (ROVER AND WORLD > CONFIGURATION)."),
            new Preset("rviz2", "ros2 run rviz2 rviz2", "RViz."),
        };

        public class Node
        {
            public string label, command;
            public Process proc;
            public int exitCode = int.MinValue;   // MinValue = still running (or never started)
            public bool stopping;
            readonly Queue<string> log = new Queue<string>();
            public const int MaxLines = 200;

            public bool Running => RosProcess.Alive(proc);

            internal void Add(string line)
            {
                lock (log)
                {
                    log.Enqueue(line);
                    while (log.Count > MaxLines) log.Dequeue();
                }
            }

            public string[] Tail(int n)
            {
                lock (log)
                {
                    var all = log.ToArray();
                    int s = Math.Max(0, all.Length - n);
                    var r = new string[all.Length - s];
                    Array.Copy(all, s, r, 0, r.Length);
                    return r;
                }
            }
        }

        static readonly List<Node> nodes = new List<Node>();
        public static IReadOnlyList<Node> Nodes => nodes;

        public static Node Find(string command) => nodes.Find(n => n.command == command);
        public static bool IsRunning(string command) { var n = Find(command); return n != null && n.Running; }

        /// <summary>Starts <paramref name="command"/> (restarts it if it already ran and exited).</summary>
        public static Node Run(string label, string command)
        {
            var n = Find(command);
            if (n != null && n.Running) return n;
            // Exclusive presets (two joystick drivers on /joy): stop the others in the group first.
            var preset = Array.Find(Presets, p => p.command == command);
            if (preset?.group != null)
                foreach (var p in Presets)
                    if (p != preset && p.group == preset.group) { var o = Find(p.command); if (o != null && o.Running) { o.Add($"[stopped: {label} started]"); Stop(o); } }
            if (n == null) { n = new Node { label = label, command = command }; nodes.Add(n); }
            n.exitCode = int.MinValue; n.stopping = false;
            n.Add($"$ {command}");
            try
            {
                var node = n;
                bool logAll = UnityEngine.Application.isBatchMode;   // headless: no menu to read the output in
                n.proc = RosProcess.Start(command, (line, err) =>
                {
                    node.Add(line);
                    if (line.Contains("ERROR") || line.Contains("Traceback") || line.Contains("not found"))
                        UnityEngine.Debug.LogWarning($"[{node.label}] {line}");
                    else if (logAll) UnityEngine.Debug.Log($"[{node.label}] {line}");
                }, code =>
                {
                    node.exitCode = code;
                    node.Add(node.stopping ? "[stopped]" : $"[exited with code {code}]");
                });
                UnityEngine.Debug.Log($"[ROS] started '{command}' (pid {n.proc.Id})");
            }
            catch (Exception e)
            {
                n.Add("could not start: " + e.Message);
                n.exitCode = -1;
            }
            return n;
        }

        /// <summary>Stops a node without blocking the caller (SIGINT, then SIGKILL after 3 s).</summary>
        public static void Stop(Node n)
        {
            if (n == null || !n.Running) return;
            n.stopping = true;
            var p = n.proc;
            System.Threading.Tasks.Task.Run(() => RosProcess.Stop(p));
        }

        /// <summary>Players (and the editor) take <c>--ros-run "&lt;command&gt;"</c>, repeatable: started once, like the menu's START.</summary>
        static bool argsDone;
        [UnityEngine.RuntimeInitializeOnLoadMethod(UnityEngine.RuntimeInitializeLoadType.AfterSceneLoad)]
        static void FromCommandLine()
        {
            if (argsDone) return;
            argsDone = true;
            var a = Environment.GetCommandLineArgs();
            for (int i = 0; i + 1 < a.Length; i++)
                if (a[i] == "--ros-run") Run(a[i + 1].Length > 40 ? a[i + 1].Substring(0, 40) + "…" : a[i + 1], a[i + 1]);
        }

        public static void Forget(Node n) { if (n != null) nodes.Remove(n); }

        /// <summary>Stops every node (blocking, short grace period): Play stopped / player quitting.</summary>
        public static void StopAll()
        {
            var tasks = new List<System.Threading.Tasks.Task>();
            foreach (var n in nodes)
            {
                if (!n.Running) continue;
                n.stopping = true;
                var p = n.proc;
                tasks.Add(System.Threading.Tasks.Task.Run(() => RosProcess.Stop(p, 1500)));
            }
            System.Threading.Tasks.Task.WaitAll(tasks.ToArray(), 5000);
        }
    }
}
