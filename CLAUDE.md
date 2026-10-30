# SimFlake: notes for Claude (and humans)

Unity simulator for the UBC Snowbots rover. It talks to the ROS 2 **Jazzy** stack in
[RoverFlake2](https://github.com/UBC-Snowbots/RoverFlake2) on **Ubuntu 24.04**.
Claude Code reads this file automatically, so keep it current when the architecture or plan changes.

## Requirements

Hard requirements (don't break these):
- Runs on Ubuntu 24.04 with ROS 2 Jazzy and works with RoverFlake2 (same topics as the real rover where possible).
- Keeps every current feature: GNSS, RGB cameras, IMU, drive control (`/cmd_vel`), URDF prefabs of the arm and chassis,
  task worlds, ArUco markers, lidar, wheel/joint odometry.
- URDFs can still be imported as prefabs with as little work as possible (Unity URDF-Importer, see below).
- Unity assets (Asset Store etc.) can be added and reach everyone else through git. Commit them under `rover_sim/Assets`.

Soft goals:
- Performance and ease of use. A new member should only need to: install Unity Hub, clone SimFlake, run `./setup.sh`,
  add `rover_sim` in Unity Hub and install the Unity version it asks for.
- As few dependencies as possible. Don't add packages or ROS nodes we don't need.

Roadmap (wanted, not started; don't do these unless asked):
- RGBD camera that mimics the RealSense cameras (color + aligned depth + camera_info, RealSense topic names).
- Standalone Linux executable members can open without Unity: title screen to pick a world and configure things, then
  they just run whatever RoverFlake2 nodes they want. Groundwork exists: `SimFlake > Build Linux Player`, `--rosbridge-url`,
  and rosbridge auto-launch also works in a player build.

## How it fits together

```
 RoverFlake2 nodes  <-- DDS (any RMW / domain) -->  rosbridge_websocket  <-- WebSocket JSON :9090 -->  Unity (rover_sim)
```

- **ROS connection = rosbridge.** `ros-jazzy-rosbridge-suite` (apt) is the only ROS-side dependency, and nothing gets built.
  The Unity side is our own small client in `rover_sim/Assets/Scripts/Ros/` with no packages or native plugins.
  Unity starts rosbridge automatically when you press Play (if nothing is already on port 9090) and stops it afterwards.
  If Unity was started from a terminal with ROS sourced, rosbridge gets that terminal's environment.
  Otherwise it goes through `bash -i`, i.e. `~/.bashrc`, so `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` and the RoverFlake2
  overlay match your terminals.
- **Why not Ros2ForUnity / ROS-TCP-Connector?** Ros2ForUnity (used before) ships native binaries built per distro.
  It only supports Humble and refuses to run on Jazzy; that was the reason the sim broke. ROS-TCP-Endpoint isn't
  packaged for Jazzy and needs building. rosbridge is maintained, packaged for Jazzy, and doesn't care which RMW or
  domain you use.
- **Bandwidth limits of rosbridge** (measured on a dev laptop, raw `sensor_msgs/Image`): 640x480 @ 30 Hz fine,
  1280x720 about 15 Hz, 1920x1080 about 6 Hz. So raw cameras default to 640x480, and high-res feeds use
  `CompressedCameraCapture` (JPEG, encoded off the main thread). Each camera stream gets its own WebSocket so images
  never delay small messages. If we ever need much more bandwidth (e.g. several RGBD streams), that is when to look
  at a native transport again. Keep the `Ros.Advertise/Subscribe` API so scripts don't care.
- **Unity 2022.3.62f3 LTS**, built-in render pipeline (no URP/HDRP). Unity 6 would also work; we just haven't needed it.
  If you upgrade, change `ProjectSettings/ProjectVersion.txt` through the editor and re-run the checks below.

## Writing sim code

```csharp
using SimFlake.Ros;
var pub = Ros.Advertise<sensor_msgs.msg.Imu>("/imu/data");               // in Start()
pub.Publish(msg);                                                         // any thread; drops if not connected/behind
Ros.Subscribe<geometry_msgs.msg.Twist>("/cmd_vel", msg => { ... });       // callback runs on the main thread
```
- Messages live in `Scripts/Ros/RosMessages.cs` as `[Serializable]` classes whose **field names match the ROS .msg
  exactly** (`header.frame_id`, `linear.x`, ...). Add new types there. Fixed-size arrays must be created at the right
  length. Big `uint8[]` fields implement `IBinaryMessage` (see `Image`). Custom `rover_msgs/...` types work if
  RoverFlake2 is sourced for rosbridge.
- Timestamps: `RosTime.Now()` / `RosTime.Header(frame)` use wall-clock time (no `/clock`, no use_sim_time yet).
- Coordinates (`RosFrames`): Unity world x = east, y = up, z = north. ROS world = ENU. Robot body = FLU.
  Use `RosFrames.UnityToEnu`, `UnityToFlu`, `UnityRotationToEnu`. Unity angular velocities are left-handed: negate
  after converting (see `ImuSensor`).
- Unity rule: a MonoBehaviour's class name must equal its file name, or it can't be attached.
- Scenes reference scripts by the GUID in the `.meta` file. When rewriting a script, keep the file and its `.meta`
  (and serialized field names where possible) so scenes keep their component settings. Never delete `.meta` files.

## Topics (what RoverFlake2 sees)

| Topic | Type | Script |
|---|---|---|
| `/cmd_vel` (sub) | geometry_msgs/Twist | `MotorController`: skid-steer, real m/s and rad/s |
| `/motor_commands` (sub) | std_msgs/Float64MultiArray | `MotorController`: 6 raw wheel targets, deg/s |
| `/odom` | nav_msgs/Odometry | `MotorController`: wheel odometry, `odom` -> `base_link` |
| `/wheel_joint_states` | sensor_msgs/JointState | `MotorController` |
| `/imu/data` | sensor_msgs/Imu | `ImuSensor` (on `chassis_link`) |
| `/gps/fix` | sensor_msgs/NavSatFix | `LocationStuff`: origin 49, -123; 1 Unity unit = 1 m; +z north |
| `/scan` | sensor_msgs/LaserScan | `HorizontalLidarScanner`: 10 Hz, CCW, +inf = no hit |
| `/camera/image_raw`, `/camera/camera_info` | Image, CameraInfo | `RGBCameraCapture` |
| `/camera/image/compressed` | CompressedImage | `CompressedCameraCapture` |
| `/arm/command` (sub) | rover_msgs/ArmCommand | `ArmController`: same topic/contract as the real arm driver (`arm_commands.h`): 'V' jog deg/s, 'P' position in output revs, 'S', 'Z', 'H', 'G'. NaN = leave that axis alone, index 6 = end effector. No sim_helper needed |
| `/arm/sim_command`, `/arm/ee_command/sim` (sub) | Float64MultiArray, Float64 | `ArmController`: legacy sim_helper topics, ignored while `/arm/command` is active |
| `/joint_states`, `/arm/sim_feedback` | JointState, Float64MultiArray | `ArmController` |
| `/chatter` / `/listener` | std_msgs/String | `testnode` (connection test, SampleScene) |

## Scenes
`Assets/Scenes/ChassisPlayground` (chassis), `ArmScene` (arm on a table), `TaskWorlds/WaypointTask` (chassis + GPS +
lidar + cameras), `SampleScene` (older all-in-one playground). The rover is the `chassis_24` URDF prefab; the moving body
is `chassis_24/base_link/chassis_link` (the root ArticulationBody). `base_link` itself does not move.

## Importing URDFs
Package `com.unity.robotics.urdf-importer` v0.5.2 (git URL in `Packages/manifest.json`, downloads automatically).
Copy the ROS description package into `Assets/`, right-click the `.urdf` and choose **Import Robot from Selected URDF
file**. Use the VHACD collider option for exact meshes (chassis and arm). Then add `MotorController` / `ArmController`
and sensors.

## Checking your work (Claude can run all of this headless)
Other `-executeMethod` entry points in `SimFlakeTools`: `UpgradeScenes [-dryrun]` (scene migration), and
`TestUrdfImport -urdf Assets/<copy>/x.urdf -prefab Assets/<copy>/x.prefab` (run it on a throwaway copy of a description
package without .meta files, then delete the copy).
```bash
U=~/Unity/Hub/Editor/2022.3.62f3/Editor/Unity     # the editor GUI must be closed (project lock)
$U -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.SimFlakeTools.SelfTest -logFile -   # compile + JSON/frames self-test
$U -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.SimFlakeTools.BuildLinux \
   -scenes Assets/Scenes/TaskWorlds/WaypointTask.unity -out /tmp/simbuild/rover_sim.x86_64 -logFile build.log
/tmp/simbuild/rover_sim.x86_64 -batchmode -nographics --rosbridge-url ws://127.0.0.1:9090 -logFile player.log &   # physics/sensors without a GPU window
python3 tools/sim_check.py --drive          # topic rates + drives the rover and checks odom/GPS/IMU agree
```
For automated tests, use an isolated ROS graph so you don't publish onto the team's domain:
`ROS_DOMAIN_ID=199 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, start rosbridge yourself on a spare port, and pass `--rosbridge-url`.

## Gotchas found during the Jazzy migration
- rosbridge's JSON (ujson) escapes `/` as `\/`. Envelope strings go through `RosJson.Unquote`; JsonUtility handles it in bodies.
- The imported wheel joints had velocity-drive damping 0, which produces no torque. `MotorController` sets
  `wheelDriveDamping` when it's 0.
- Skid steer slips: `turnSlipFactor` (2.3) scales the effective track width for both commands and odometry.
  It was tuned so a commanded 0.5 rad/s turn roughly matches the IMU. Retune against the real rover if needed.
- Saving a scene writes every serialized field. Changing a default in code does NOT change existing scenes; use
  `SimFlakeTools.UpgradeScenes` (or the inspector).
- GPU readback row order: on OpenGL (Unity's default on Linux) camera RenderTexture readback is top-row-first.
  `ImageConversion.EncodeArrayToJPG` wants bottom-row-first. See `SensorCamera.ReadbackIsTopDown`. Verified with real
  frames on OpenGL only. If someone forces Vulkan and images are upside down, that's the place to fix.
- `Shader.Find` returns null in player builds for shaders nothing references. ArUco uses `Resources/ArucoUnlit.mat`.
- Ubuntu 24.04 has no `libdl.so` symlink; the URDF importer's Assimp (COLLADA `.dae` only) needs it. `setup.sh` offers
  to create it. STL meshes and VHACD colliders work without it (tested: dev_arm imports to a prefab headless).
- Don't `pkill -f <pattern>` from a shell whose own command line contains the pattern. It kills itself.
- Stopping Play while the launch shell is still reading `~/.bashrc` used to interrupt RoverFlake2's `setup.bash`
  halfway (so no `ROS_DOMAIN_ID`/RMW) and leave an orphaned rosbridge on domain 0 that every later Play connected to.
  `RosBridgeLauncher` now runs with `set +m` and SIGKILLs the whole session on stop. If topics vanish, check the
  `[rosbridge] starting rosbridge (... ROS_DOMAIN_ID=...)` line in the Console.
- The arm listens on `/arm/command` directly (needs `rover_msgs`, i.e. RoverFlake2 sourced for rosbridge). The sim has no
  limit switches or differential wrist: 'H' drives axes back to the URDF zero, motor-space commands are treated as axis space.
  The HMI's status panels read `/arm/moteus_feedback`, which the sim does not publish (yet).
- The scenes' `CompressedCameraCapture` components are disabled (team choice); enable them for a JPEG feed.

## Housekeeping
- `Library/`, `Temp/`, `Logs/`, `UserSettings/`, `Builds/` are local only (gitignored).
- `Assets/TerrainSampleAssets` is ~1.9 GB of the ~2.3 GB repo. Trimming unused parts would make cloning much faster
  (not done yet: check which scenes use it first).
