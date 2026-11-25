using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

namespace SimFlake.Ros
{
    /// <summary>
    /// Runs a shell command with the ROS environment as a child process: rosbridge (<see cref="RosBridgeLauncher"/>) and the
    /// nodes started from the Esc menu's ROS tab (<see cref="RosNodeRunner"/>).
    ///
    /// Environment: if Unity was started from a terminal with ROS sourced, that environment is kept. Otherwise the command
    /// runs through an interactive bash so ~/.bashrc applies (ROS_DOMAIN_ID, RMW_IMPLEMENTATION). Either way /opt/ros is
    /// sourced if ros2 is missing, and the RoverFlake2 workspace (<see cref="RoverFlake2Path"/>) is sourced if its packages
    /// (rover_msgs) are not found yet, so rosbridge knows rover_msgs and `ros2 run drive_control ...` works.
    ///
    /// Each process gets its own session (setsid). <see cref="Stop"/> sends SIGINT to the whole session, then SIGKILL.
    /// The command also dies when Unity does (the shell watches Unity's PID), because OnApplicationQuit doesn't run on a crash.
    /// </summary>
    public static class RosProcess
    {
        const string PathPref = "SimFlake.RoverFlake2Path";

        /// <summary>Folder of the RoverFlake2 colcon workspace (has install/setup.bash). Saved between runs.</summary>
        public static string RoverFlake2Path
        {
            get
            {
                string p = UnityEngine.PlayerPrefs.GetString(PathPref, "");
                if (!string.IsNullOrEmpty(p)) return p;
                p = Environment.GetEnvironmentVariable("ROVERFLAKE2_WS");
                if (!string.IsNullOrEmpty(p)) return p;
                return System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), "RoverFlake2");
            }
            set { UnityEngine.PlayerPrefs.SetString(PathPref, value ?? ""); UnityEngine.PlayerPrefs.Save(); }
        }

        public static string RoverFlake2Setup => System.IO.Path.Combine(ExpandHome(RoverFlake2Path), "install", "setup.bash");
        public static bool RoverFlake2Found => System.IO.File.Exists(RoverFlake2Setup);

        static string ExpandHome(string p) =>
            p.StartsWith("~") ? Environment.GetFolderPath(Environment.SpecialFolder.UserProfile) + p.Substring(1) : p;

        static string Prelude()
        {
            string ws = RoverFlake2Setup;
            return
                // No job control: everything stays in this session so Stop() reaches it.
                "set +m; " +
                "command -v ros2 >/dev/null 2>&1 || source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash; " +
                $"if ! ros2 pkg prefix rover_msgs >/dev/null 2>&1 && [ -f {ShellQuote(ws)} ]; then source {ShellQuote(ws)}; fi; ";
        }

        /// <summary>
        /// Starts <paramref name="command"/> (bash; may contain several commands). Output lines go to <paramref name="onLine"/>
        /// (line, isStderr) on a worker thread; <paramref name="onExit"/> gets the exit code.
        /// </summary>
        public static Process Start(string command, Action<string, bool> onLine, Action<int> onExit = null, string preamble = "")
        {
            string script = Prelude() + preamble +
                // The command runs in the background while this shell watches Unity's PID.
                $"( {command} ) & CHILD=$!; " +
                "trap 'kill -INT $CHILD 2>/dev/null; wait $CHILD; exit 0' INT TERM; " +
                $"while kill -0 {Process.GetCurrentProcess().Id} 2>/dev/null && kill -0 $CHILD 2>/dev/null; do sleep 1; done; " +
                "if kill -0 $CHILD 2>/dev/null; then kill -INT $CHILD 2>/dev/null; sleep 3; kill -KILL -- -$$ 2>/dev/null; fi; " +
                "wait $CHILD";

            bool inheritEnv = !string.IsNullOrEmpty(Environment.GetEnvironmentVariable("ROS_DISTRO"));
            var psi = new ProcessStartInfo
            {
                FileName = "/usr/bin/setsid",
                Arguments = (inheritEnv ? "/bin/bash -c " : "/bin/bash -ic ") + ShellQuote(script),
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                RedirectStandardInput = true,
                CreateNoWindow = true,
            };
            var proc = new Process { StartInfo = psi, EnableRaisingEvents = true };
            proc.OutputDataReceived += (_, e) => { if (Keep(e.Data)) onLine?.Invoke(e.Data, false); };
            proc.ErrorDataReceived += (_, e) => { if (Keep(e.Data)) onLine?.Invoke(e.Data, true); };
            proc.Exited += (_, __) =>
            {
                int code = -1;
                try { code = proc.ExitCode; } catch { }
                onExit?.Invoke(code);
            };
            proc.Start();
            proc.BeginOutputReadLine();
            proc.BeginErrorReadLine();
            return proc;
        }

        // Harmless noise from running bash -i without a terminal.
        static bool Keep(string line) =>
            !string.IsNullOrWhiteSpace(line) &&
            !line.Contains("cannot set terminal process group") && !line.Contains("no job control in this shell");

        public static bool Alive(Process p) { try { return p != null && !p.HasExited; } catch { return false; } }

        /// <summary>SIGINT to the whole session (like Ctrl+C), wait up to <paramref name="graceMs"/>, then SIGKILL what is left.</summary>
        public static void Stop(Process proc, int graceMs = 3000)
        {
            if (!Alive(proc)) return;
            int sid = proc.Id; // setsid made the shell a session leader, so its pid is the session id
            try
            {
                // Session rather than process group: an interactive bash may move jobs into their own group.
                // Always finish with SIGKILL: if this runs while ~/.bashrc is still loading, SIGINT can interrupt it
                // halfway and the shell would go on with a half-loaded environment (e.g. no ROS_DOMAIN_ID).
                Process.Start("/usr/bin/pkill", $"-INT -s {sid}")?.WaitForExit(1000);
                proc.WaitForExit(graceMs);
                Process.Start("/usr/bin/pkill", $"-KILL -s {sid}")?.WaitForExit(1000);
            }
            catch (Exception e) { Debug.LogWarning("[ROS] stopping a ROS process: " + e.Message); }
        }

        public static string ShellQuote(string s) => "'" + s.Replace("'", "'\\''") + "'";
    }
}
