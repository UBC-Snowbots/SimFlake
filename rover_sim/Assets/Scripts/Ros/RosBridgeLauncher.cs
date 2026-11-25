using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

namespace SimFlake.Ros
{
    /// <summary>
    /// Starts rosbridge_websocket as a child process when nothing is already listening (environment: see <see cref="RosProcess"/>).
    /// One process per port: the main one (control topics) and one for the camera streams, see
    /// <see cref="RosConnection"/> (rosbridge is a single Python process; a raw camera stream keeps it at 100 % CPU).
    /// </summary>
    public static class RosBridgeLauncher
    {
        static readonly System.Collections.Generic.Dictionary<int, Process> procs = new System.Collections.Generic.Dictionary<int, Process>();

        public static bool Running { get { foreach (var p in procs.Values) if (RosProcess.Alive(p)) return true; return false; } }
        public static bool RunningOn(int port) => procs.TryGetValue(port, out var p) && RosProcess.Alive(p);

        public static void Launch(int port, string nodeName = "rosbridge_websocket")
        {
            if (RunningOn(port)) return;
            string tag = nodeName == "rosbridge_websocket" ? "[rosbridge] " : "[" + nodeName + "] ";
            string check =
                "if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then " +
                "  echo 'ERROR: rosbridge_server not found. Run SimFlake/setup.sh (or: sudo apt install ros-jazzy-rosbridge-suite)'; exit 3; fi; " +
                "echo \"starting rosbridge (ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-default})\"; ";
            try
            {
                var proc = RosProcess.Start(
                    $"ros2 run rosbridge_server rosbridge_websocket --ros-args -r __node:={nodeName} -p port:={port} -p address:=127.0.0.1",
                    (line, stderr) => Forward(line, stderr, tag),
                    code =>
                    {
                        if (code != 0 && code != 130 && code != 143 && code != -2 && code != -15)
                            Debug.LogWarning($"{tag}exited with code {code}");
                    },
                    check);
                procs[port] = proc;
                Debug.Log("[ROS] Nothing listening on port " + port + ", launched " + nodeName + " (pid " + proc.Id + ")");
            }
            catch (Exception e)
            {
                Debug.LogError("[ROS] Could not launch rosbridge: " + e.Message +
                               "\nStart it yourself: ros2 run rosbridge_server rosbridge_websocket");
            }
        }

        static void Forward(string line, bool stderr, string tag)
        {
            if (line.Contains("ERROR") || line.Contains("Traceback")) Debug.LogError(tag + line);
            else if (stderr && line.Contains("WARN")) Debug.LogWarning(tag + line);
            else Debug.Log(tag + line);
        }

        public static void Stop()
        {
            foreach (var p in procs.Values) RosProcess.Stop(p);
            procs.Clear();
        }
    }
}
