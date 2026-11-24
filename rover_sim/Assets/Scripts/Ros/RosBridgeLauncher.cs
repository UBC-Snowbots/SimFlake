using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

namespace SimFlake.Ros
{
    /// <summary>
    /// Starts rosbridge_websocket as a child process when nothing is already listening.
    /// If Unity was started from a terminal with ROS sourced, rosbridge inherits that environment.
    /// Otherwise it runs through an interactive bash so it picks up the same ROS setup as your
    /// terminals (~/.bashrc: ROS_DOMAIN_ID, RMW_IMPLEMENTATION, RoverFlake2 overlay for rover_msgs, ...).
    /// One process per port: the main one (control topics) and one for the camera streams, see
    /// <see cref="RosConnection"/> (rosbridge is a single Python process; a raw camera stream keeps it at 100 % CPU).
    /// </summary>
    public static class RosBridgeLauncher
    {
        static readonly System.Collections.Generic.Dictionary<int, Process> procs = new System.Collections.Generic.Dictionary<int, Process>();

        static bool Alive(Process p) { try { return p != null && !p.HasExited; } catch { return false; } }

        public static bool Running { get { foreach (var p in procs.Values) if (Alive(p)) return true; return false; } }
        public static bool RunningOn(int port) => procs.TryGetValue(port, out var p) && Alive(p);

        public static void Launch(int port, string nodeName = "rosbridge_websocket")
        {
            if (RunningOn(port)) return;
            string script =
                // No job control: keeps rosbridge in this session/process group so Stop() always reaches it.
                "set +m; " +
                "command -v ros2 >/dev/null 2>&1 || source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; " +
                "if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then " +
                "  echo 'ERROR: rosbridge_server not found. Run SimFlake/setup.sh (or: sudo apt install ros-jazzy-rosbridge-suite)'; exit 3; fi; " +
                "echo \"starting rosbridge (ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-default})\"; " +
                // rosbridge runs in the background while this shell watches Unity's PID, so it also goes away
                // if Unity crashes or is killed (OnApplicationQuit doesn't run then).
                $"ros2 run rosbridge_server rosbridge_websocket --ros-args -r __node:={nodeName} -p port:={port} -p address:=127.0.0.1 & RB=$!; " +
                "trap 'kill -INT $RB 2>/dev/null; wait $RB; exit 0' INT TERM; " +
                $"while kill -0 {Process.GetCurrentProcess().Id} 2>/dev/null && kill -0 $RB 2>/dev/null; do sleep 1; done; " +
                "kill -INT $RB 2>/dev/null; sleep 3; kill -KILL -- -$$ 2>/dev/null";

            // Launched from a terminal with ROS sourced: keep that exact environment.
            // Launched from Unity Hub / desktop: use an interactive bash so ~/.bashrc sets ROS up.
            bool inheritEnv = !string.IsNullOrEmpty(Environment.GetEnvironmentVariable("ROS_DISTRO"));
            var psi = new ProcessStartInfo
            {
                // setsid: own process group, so Stop() can take down ros2 run and its child together.
                FileName = "/usr/bin/setsid",
                Arguments = (inheritEnv ? "/bin/bash -c " : "/bin/bash -ic ") + ShellQuote(script),
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                RedirectStandardInput = true,
                CreateNoWindow = true,
            };
            Process proc = null;
            string tag = nodeName == "rosbridge_websocket" ? "[rosbridge] " : "[" + nodeName + "] ";
            try
            {
                proc = new Process { StartInfo = psi, EnableRaisingEvents = true };
                proc.OutputDataReceived += (_, e) => Forward(e.Data, false, tag);
                proc.ErrorDataReceived += (_, e) => Forward(e.Data, true, tag);
                proc.Exited += (_, __) =>
                {
                    int code = -1;
                    try { code = proc.ExitCode; } catch { }
                    if (code != 0 && code != 130 && code != 143 && code != -2 && code != -15)
                        Debug.LogWarning($"{tag}exited with code {code}");
                };
                proc.Start();
                proc.BeginOutputReadLine();
                proc.BeginErrorReadLine();
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
            if (string.IsNullOrWhiteSpace(line)) return;
            // Harmless noise from running bash -i without a terminal.
            if (line.Contains("cannot set terminal process group") || line.Contains("no job control in this shell")) return;
            if (line.Contains("ERROR") || line.Contains("Traceback")) Debug.LogError(tag + line);
            else if (stderr && line.Contains("WARN")) Debug.LogWarning(tag + line);
            else Debug.Log(tag + line);
        }

        public static void Stop()
        {
            foreach (var p in procs.Values) Stop(p);
            procs.Clear();
        }

        static void Stop(Process proc)
        {
            if (!Alive(proc)) return;
            int sid = proc.Id; // setsid made the launch shell a session leader, so its pid is the session id
            try
            {
                // SIGINT to everything in the session (like Ctrl+C), then SIGKILL whatever is left.
                // Session rather than process group: an interactive bash may move jobs into their own group.
                // Always finish with SIGKILL: if Play is stopped while ~/.bashrc is still loading, SIGINT
                // can interrupt it halfway and the shell would otherwise go on to start rosbridge with a
                // half-loaded environment (e.g. no ROS_DOMAIN_ID).
                Process.Start("/usr/bin/pkill", $"-INT -s {sid}")?.WaitForExit(1000);
                proc.WaitForExit(3000);
                Process.Start("/usr/bin/pkill", $"-KILL -s {sid}")?.WaitForExit(1000);
            }
            catch (Exception e) { Debug.LogWarning("[ROS] stopping rosbridge: " + e.Message); }
        }

        static string ShellQuote(string s) => "'" + s.Replace("'", "'\\''") + "'";
    }
}
