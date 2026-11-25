# Turning over /cmd_vel did nothing (rosbridge starved by the camera stream)

Fixed 2026-11-23 in commit `749782b`.

## Symptom
`tools/sim_check.py --drive` drove forward but the "turn left" step showed zero yaw on odometry and the IMU (chassis_24 in
WaypointTask, chassis_2026 too). The forward step also overshot (2.4 m instead of 2.0): the stop came late. Keyboard drive
turned fine, and so did every headless (`-nographics`) test. Nothing to do with RoverFlake2: reproduced by publishing only
`/cmd_vel`.

## Cause
rosbridge is one Python process. Each raw camera frame reaches it as base64 JSON (640x480 rgb8 = ~1.2 MB, ~37 MB/s at 30 Hz),
which keeps it at 85-100 % of a core. While it is that busy, a fast publisher on a topic the sim subscribes to backs it up,
and commands reach the sim seconds late or not at all. `sim_check` published `/cmd_vel` once per incoming callback, ~230 Hz.

The sim's own code (`MotorController.CmdVelCallback`, Unity-side message dispatch) was never the problem: Unity drains every
queued message each frame, and a separate test client subscribed to `/cmd_vel` through the same rosbridge was delayed just
as much.

## Measurements
One shared rosbridge, WaypointTask player with graphics (camera streaming 30 Hz), a test client subscribed to `/cmd_vel`
through rosbridge timing 10 Hz stamped messages while another node floods `/cmd_vel`:

| flood rate | delivered | worst latency |
|---|---|---|
| none / 60 Hz | 100 % | < 0.04 s |
| 100 Hz | 100 % | 0.4 s |
| 150 Hz | 100 % | 2 s |
| 230 Hz | 0 % | - |
| 230 Hz, camera off (`-nographics`) | 100 % | 0.008 s |
| 230 Hz, camera on its own rosbridge (the fix) | 100 % | 0.008 s |

Not the cause (each tested alone, all fine): rosbridge CPU halved (pinned to a core shared with a busy loop: the camera
publisher drops frames when rosbridge falls behind, so slowness alone builds no backlog), a ROS subscriber on the raw image,
a 60 Hz publisher, battery vs AC.

## Fix
- `RosConnection` / `RosBridgeLauncher`: when the sim launches rosbridge itself (nothing on the port after 1.5 s), it also
  launches `rosbridge_streams` on port + 1 (9091 by default) and sends the dedicated sockets (cameras) there. Camera sockets
  wait until that decision is made. Both processes stop with the sim.
- A rosbridge that is already running (yours, a launch file, a test's on its own domain) is shared by the streams, as before:
  a second one launched from `~/.bashrc` could land on a different ROS domain. Override with `--rosbridge-stream-url <url>`
  or `SIMFLAKE_ROSBRIDGE_STREAM_URL`. In that shared setup the limits in the table still apply (fine below ~100 Hz).
- `tools/sim_check.py` publishes `/cmd_vel` at a fixed 20 Hz, like a real teleop/nav node, so it passes on either setup.

Tried and dropped: subscribing with `queue_length` 1 and a 10 ms throttle (rosbridge forwards only the newest command). No
effect on the shared setup, so the bottleneck is before rosbridge's per-client write queue.

## Still open
The very first failing run on 2026-11-23 also lost commands at 10 Hz with no flood running (a probe got 7 of 100, up to 10 s
late). Not reproduced in ~15 later runs, with old or new code. With the cameras on their own process, the control path should
no longer be exposed to it.

## How to reproduce / test
Isolated graph (`ROS_DOMAIN_ID=199 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`), player run with graphics:
`<player> -batchmode --rosbridge-url ws://127.0.0.1:9393` (no `-nographics`, or the camera sends nothing).
- Shared setup: start `ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9393` first.
- Fixed setup: start nothing (with ROS sourced in that shell, so the launched rosbridges inherit the isolated domain);
  the log shows `camera streams go through ws://127.0.0.1:9394/`.
Then flood `/cmd_vel` at 230 Hz from one node and check delivery from another, or run the old `sim_check`
(`git show 749782b~1:tools/sim_check.py`).

## Joysticks: "phantom" driving (2026-11-25)
Reported with joy_linux driving through RoverFlake2's chain (joy -> drive_control_node -> /cmd_vel -> wheel_speed_node ->
/left_wheel_speeds + /right_wheel_speeds -> sim): the rover kept doing things after the stick was let go. joy_linux's defaults
(`coalesce_interval` 0.001 s, publish on every event) give up to ~1 kHz on /joy while a stick moves; drive_control_node republishes
each one, wheel_speed_node turns each into two messages. Measured (WaypointTask player with graphics, cameras on their own rosbridge,
real wheel_speed_node, synthetic /cmd_vel, time from the first stop command until odom < 0.05 m/s; 0.15 s of that is the motor ramp):

| /cmd_vel rate | stopped after |
|---|---|
| 1000 Hz | 0.68 s |
| 20 Hz | 0.15 s |

Worse with a shared rosbridge or a busier machine (table above). Fixes: the Esc > ROS joy presets cap /joy at ~20 Hz
(`coalesce_interval` 50 ms, `autorepeat_rate` 20 so a held stick still refreshes); `MotorController` subscribes only to the current
DRIVE INPUT's topics, and `RosConnection.Unsubscribe` now sends rosbridge an `unsubscribe` when a topic's last local subscriber goes
(before, rosbridge kept forwarding /cmd_vel to the sim in ROVERFLAKE2 mode). Not measured: how much the dropped /cmd_vel subscription
alone helps. On the real rover the same chain has no rosbridge, but RoverFlake2's game_controller.launch (100 Hz autorepeat) is worth
a look if its drive lags too.
