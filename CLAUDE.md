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
- Unity assets (Asset Store etc.) can be added and reach everyone else through git. Commit them under `rover_sim/Assets/ThirdParty` (Unity imports packs at the `Assets/` root; move the folder in the editor).

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
  It also starts `rosbridge_streams` on 9091 for the cameras (else fast `/cmd_vel` starves): see `claude/rosbridge-camera-starvation.md`.
  If Unity was started from a terminal with ROS sourced, rosbridge gets that terminal's environment.
  Otherwise it goes through `bash -i`, i.e. `~/.bashrc`, so `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` match your terminals.
  Either way, if `rover_msgs` isn't on the path yet, the RoverFlake2 workspace's `install/setup.bash` is sourced too (path: Esc >
  ROS > ROVERFLAKE2 WORKSPACE, default `~/RoverFlake2` or `$ROVERFLAKE2_WS`). All of that is `Ros/RosProcess.cs`, shared by
  rosbridge and the nodes started from the menu (`Ros/RosNodeRunner.cs`, stopped when Play stops; players: `--ros-run "<cmd>"`).
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
| `/left_wheel_speeds`, `/right_wheel_speeds` (sub) | std_msgs/Float64MultiArray | `MotorController`, DRIVE INPUT = ROVERFLAKE2 (default): stands in for RoverFlake2's `motor_control_node`. data[0] = m/s at the wheel in motor direction (left motors mirrored: `wheel_speed_node` sends -linear + angular), < 0.05 = stop, clamp 3.5, ramp 50 rad/s². Keeps the last speed like the real node (no timeout). `/cmd_vel` then needs `wheel_speed_node` (Esc > ROS) |
| `/drive/feedback` | rover_msgs/DriveFeedback | `MotorController` (ROVERFLAKE2 mode), 1 Hz, motors 0-2 right, 3-5 left, motor direction, m/s and rad |
| `/cmd_vel` (sub) | geometry_msgs/Twist | `MotorController`, DRIVE INPUT = /CMD_VEL DIRECT (bypass, `--drive-input cmdvel`): skid-steer, real m/s and rad/s with `turnSlipFactor`. Wheels stop 0.5 s after the last command, so publish continuously. Arrow keys and benches call this path directly in either mode |
| `/odom` | nav_msgs/Odometry | `MotorController`: wheel odometry, `odom` -> `base_link` |
| `/wheel_joint_states` | sensor_msgs/JointState | `MotorController`: position, velocity, effort = wheel torque (N·m) |
| `/imu/data` | sensor_msgs/Imu | `ImuSensor` (on `chassis_link`) |
| `/gps/fix` | sensor_msgs/NavSatFix | `LocationStuff` (`GPS` child of `chassis_link`): origin 49, -123 (per-scene override); 1 Unity unit = 1 m; +z north |
| `/scan` | sensor_msgs/LaserScan | `HorizontalLidarScanner`: 10 Hz, CCW, +inf = no hit |
| `/camera/image_raw`, `/camera/camera_info` | Image, CameraInfo | `RGBCameraCapture` |
| `/camera/image/compressed` | CompressedImage | `CompressedCameraCapture` |
| `/arm/command` (sub) | rover_msgs/ArmCommand | `ArmController`: same topic/contract as the real arm driver (`arm_commands.h`): 'V' jog deg/s, 'P' position in output revs, 'S', 'Z', 'H', 'G'. NaN = leave that axis alone, index 6 = end effector. No sim_helper needed. Axes are steppers (see Arm) |
| `/arm/sim_command`, `/arm/ee_command/sim` (sub) | Float64MultiArray, Float64 | `ArmController`: legacy sim_helper topics, ignored while `/arm/command` is active |
| `/joint_states`, `/arm/sim_feedback` | JointState, Float64MultiArray | `ArmController` |
| `/chatter` / `/listener` | std_msgs/String | `testnode` (connection test, SampleScene) |

## Folder layout (`rover_sim/Assets`)
```
Robots/        Chassis/ (one prefab per chassis: chassis_2024.prefab, mini_rover.prefab, chassis_2026.prefab), Arm/ (Arm.prefab, Motors/ stepper specs,
               GripperPad material), Descriptions/ (URDF packages),
               Drivetrain/ (Motors/, Wheels/, Grounds/ spec assets, see Drive physics)
Prefabs/Props/ props we place in worlds (ArucoMarker_75cm)
Scenes/        scenes (TaskWorlds/ for competition-style worlds, Tests/DriveTest = drivetrain test bench, SoilYard = soil lanes)
Worlds/        per-world data: Badlands/, UrcSite/Generated/ and ChassisPlayground/Generated/ are builder output (wiped on
               rebuild), UrcSite/Source/ is the processed real-world data the URC site is built from (keep),
               <Scene>/Terrain.asset are hand-made terrains, Skies/ the sky materials
Scripts/       Ros/ (bridge client), Robot/ (motor/arm control; Arm/ = steppers, breakage), Soil/ (deformable ground + wheel-soil model), Sensors/,
               Environment/ (sky, ArUco), Tools/ (cameras, FPS, test node, bench), Editor/ (builders, SimFlakeTools)
Materials/, Shaders/, Resources/   ours
ThirdParty/    Asset Store / downloaded packs, unchanged (HQ Rocks, klen, TerrainSampleAssets, ...)
TextMesh Pro/  stays at the root (TMP expects it there)
```
Builders and `SimFlakeTools.PathLoadedAssets()` load some assets by path; grep for `"Assets/` before moving things.
Move assets with their `.meta` (in the editor, or `AssetDatabase.MoveAsset` headless), then run `CheckMissingRefs`.

## Scenes
`Assets/Scenes/SoilYard` (mini rover on lanes of Utah soils: ruts, sinkage, tyre tests; see Soil model),
`Assets/Scenes/ChassisPlayground` (chassis), `ArmScene` (arm on a table), `TaskWorlds/WaypointTask` (chassis + GPS +
lidar + cameras), `SampleScene` (older all-in-one playground). The moving body is `chassis_24/base_link/chassis_link`
(the root ArticulationBody). `base_link` itself does not move.

Robots are prefabs: `Assets/Robots/Chassis/chassis_2024.prefab` (rover with motor control, IMU, lidar, GPS, cameras) and
`Assets/Robots/Arm/Arm.prefab` (`dev_arm` + `ArmController`). Every rover scene uses an instance, so change the rover in
the prefab (or Open Prefab), not in a scene. Per-scene differences are overrides: ChassisPlayground's camera mount,
the GPS origin/debug text, CameraSwapper's cameras. Physics (masses, joint limits) is never overridden. To add the rover
to a new world, drag the prefab in. In SampleScene the old lidar/camera objects outside the rover are disabled
(the prefab's own sensors publish those topics). Rename or move prefabs inside Unity (F2) so the `.meta` follows;
scenes find them by GUID, and the scene instances keep their own name (`chassis_24`).

`Assets/Robots/Chassis/mini_rover.prefab` is the mini rover, generated from RoverFlake2's `rover_description/urdf/mini_rover.urdf`
(copy in `Robots/Descriptions/mini_rover/`, layout `<urdf dir>/rover_description/meshes/...` so `package://` paths resolve
headless) by `Editor/MiniRoverBuilder.cs` (menu `SimFlake > Build Mini Rover Prefab`, or `-executeMethod
SimFlake.EditorTools.MiniRoverBuilder.BuildFromCommandLine`). A rebuild overwrites the prefab, including sensor moves saved in it.
It uses chassis_2024's topics and frame ids, so don't put both rovers in one scene. Things the builder does on purpose:
- Sensors are `SensorBlock`s under `base_link/Sensors`: a coloured box with a collider (the lidar sees it) and a floating label
  (drawn on top, hidden from sensor cameras). The sensor script sits on the block's object, so dragging the block moves the sensor.
  LiDAR, IMU and GPS (the antenna, `GPS2_Link`; the receiver board is dropped) come from the URDF; the camera is added.
- The URDF zero pose is not a standing pose (front wheels ~15 cm above the bogie wheels). The builder rotates the rockers
  (~17 deg) and bogies until all six wheels are level; that becomes the joint zero.
- `RockerDifferential` couples the rockers (right = -left) with stiff drives; PhysX has no mimic joint.
- `RobotPhysicsSetup` raises the root articulation's solver iterations at runtime (not serializable; with the default 6/1
  the wheel drives stalled against ground friction) and ignores self-collisions.
- Wheels and motors come from the drivetrain model (see Drive physics): `MiniRoverWheel` spec (sphere tread colliders;
  VHACD hulls snagged) and the `ES_36PG_3429_100` motor. `MiniRoverWheel.physicMaterial`
  (0.35, Minimum combine) caps grip: the bogie pivot is higher (14 cm) than half the bogie wheel spacing (12.7 cm), so at
  friction ~0.5 skid turns flip the bogies onto their limits and roll the rover. `turnSlipFactor` 2.2 matches IMU yaw on CustomDirt.
  (That cap was found before improved patch friction was on; with it on, higher grip may now be fine. Untested.)

`Assets/Robots/Chassis/chassis_2026.prefab` is the 2026 big rover, built the same way from RoverFlake2's `rover_description/urdf/chassis.urdf`
(copy: `Robots/Descriptions/chassis_2026/chassis_2026.urdf` + only the meshes it uses) by the same builder: each rover is a
`MiniRoverBuilder.Def` (`MiniRover`, `Chassis2026`), menu `SimFlake > Build Chassis 2026 Prefab`, or `-executeMethod
SimFlake.EditorTools.MiniRoverBuilder.BuildChassis2026FromCommandLine` (~2 min). In the Esc menu's rover list as CHASSIS 2026 (the builder adds it to
`Resources/RoverCatalog.asset`). Same topics and frame ids as the other two: one of them per scene. Its URDF is hand-built and partly placeholder
(see RoverFlake2 `rover_description/CLAUDE.md`, "Big rover"); differences from the mini rover:
- base_link's origin is the front-bottom corner of the frame (not mid-body), so wheels are sorted front/mid/back per side by position.
  The zero pose already stands level (no rocker/bogie correction). `RockerDifferential` multiplier -0.3 (the URDF mimic).
- **Body pitch = `RockerDifferential.stiffness`** (N·m per *radian*, per rocker; sag = m g d / (2 k) for a centre of mass d behind the
  rocker pivot). At the default 50 chassis_2026 sagged 11.6 deg nose-up (centre of mass ~12 cm behind the pivot); the real differential
  is a rigid linkage, so it has 20000 (0.03 deg). The drive targets are predicted one step ahead so a stiff drive doesn't drag on the
  terrain-following motion. The mini rover keeps 50 (sags 1.6 deg). The bench logs `settled, body pitch` and `rest_pitch_deg`.
- **Looks (`Editor/Chassis2026Body.cs`, from photos of the real rover):** blue powder-coated frame, white panels on all six faces of
  the frame box (x -0.6..0, y +-0.305, z 0..0.21, 25.4 mm tube; panels 3 mm inside the tubes' outer faces so the blue frame shows around
  each), three 120 mm fans per side (procedural meshes in `Robots/Chassis/Parts`, 7 mm deep: the rockers run ~2 cm outside the panels;
  placed clear of the rocker pivot boss), e-stop at the back-left of the top, silver rockers/bogies/differential, carbon mast. Materials in
  `Robots/Chassis/Materials`, updated in place on a rebuild (edit the code, not the .mat). No internals: the shell is closed.
- **Arm:** `RoverPayload` + `ArmMount` like chassis_2024 (WORLD menu CONFIGURATION, `--rover-config arm`), on the front-left corner of
  the top panel as on the real rover: motor hole 10 cm behind the front, 17 cm left of centre (19 would put the base plate 1.7 cm into the
  left GPS antenna's block; the prefab was edited in place, a rebuild gives the same), base plate on the panel (link_0 10 cm
  below it), motor down into the box (overlaps the shell collider: excluded by `IgnoreOverlaps`). Tested: attaches quietly, FIX ARM,
  drives with it. The position is from a photo, not measured: `MiniRoverBuilder.Chassis2026.armMount`.
- Sensors: LiDAR = the same Scanse Sweep (scan plane 41.5 mm above `LiDAR_Link`), IMU and GPS on their 19 x 25 x 19 mm boxes (`/gps/fix`
  from the left antenna; the right one is not simulated). Camera on the front of the PTZ head at the top of the 1 m mast (`PTZ_Link` is kept
  as the mast's visual + collider), fixed forward: pan/tilt is not simulated. The LiDAR sees the mast behind it (nearest return ~0.67 m).
- Collision: the importer's VHACD made 527 hulls (PhysX at half chassis_2024's speed; `RobotPhysicsSetup`'s self-collision pairs grow with the
  square). base_link is one box over the closed shell; the builder re-decomposes every other link above `maxHullsPerLink` (16) from its
  visual mesh (`Descriptions/chassis_2026/Hulls/`, 85 hulls in all, PhysX ~15x real time like the mini) and deletes the importer's hull assets. It keeps the STL prefabs and `<stl>_0.asset` meshes: the next import needs them (deleting them
  made the importer crash with a NullReference in `VHACD.GenerateConvexMeshes`; the fix is to delete the STL prefabs so they are regenerated).
- **Estimates (no team numbers yet):** total 35 kg (like chassis_2024; the URDF masses add up to ~11 kg and are placeholders, scaled up
  keeping their proportions), the drive motors are chassis_2024's maxon (`Maxon_ECXFL42M_GPX42_26`), wheel = `Chassis2026Wheel` (the URDF's
  placeholder: mini rover tread scaled to r = 0.1 m, 69 mm wide, 0.9 kg). Constants in `DrivetrainSetup` (`Chassis2026MassKg`, `ApplyToChassis2026`).
- Bench (firm ground, 35 kg): top 3.13 m/s, odom < 1.2 %, `turnSlipFactor` 2.0 (point turn matches), climbs 20 deg everywhere and 30 deg on
  asphalt but not on CustomDirt (slides back at 73 % slip; chassis_2024 climbs it). On soil it climbs 20 deg on sand, hardpack and gravel, 10 on
  bentonite, 30 on slickrock. Soil costs ~1 ms per step (its baked tread has lugs; chassis_2024's tyre is smooth: 0.4).
- Size: the visual meshes are ~80 MB of text mesh assets (CAD detail: base_link alone is 212k triangles; vertex welding would save only 37 %).

Adding a robot (new chassis, arm revision...): import the URDF (see Importing URDFs), add the controller and sensors,
and save it as a new prefab next to the others (`Robots/Chassis/chassis_<year>.prefab`, `Robots/Arm/...`). Copy sensor
setups from the existing prefab (Lidar, GPS, Movable Camera children of `chassis_link`; `ImuSensor` on `chassis_link`).
Topics come from the components, so two robots in one scene need different topic names.
`Editor/RobotPrefabs.cs` was the one-off migration that turned the per-scene rover copies into the prefab. Don't run
it for new robots (it refuses if the scene rover is already a prefab). Its `ReplaceWithInstance` is the reusable part:
it swaps an unpacked copy in a scene for a prefab instance, keeps differing settings as overrides (except physics),
relinks scene references and disables duplicate sensors. Use it as a starting point if a robot ever gets copied into
scenes again.

ChassisPlayground's test features (sand rock pit, 0.3/0.6/1.0 m drops, 10/20/30 deg slopes, under `Generated Features`)
and its sunset sky are generated by `Editor/WorldBuilder.cs` + `Editor/SkyBuilder.cs` (menu `SimFlake > World Generation >
Build Playground Features`, or `-executeMethod SimFlake.EditorTools.WorldBuilder.BuildFromCommandLine [-screenshots <dir>]`). Edit the code
and re-run rather than hand-editing those objects; a re-run deletes and rebuilds them. The sky is a procedural shader
(`Shaders/MountainSkybox.shader`, material `Worlds/Skies/SunsetMountains.mat`), so it has no textures. The sun follows the directional light.
Time of day (Sunrise/Midday/Sunset/Night) and weather (Clear/Cloudy) are independent settings on the scene's
`Sky Environment` object (`Scripts/Environment/SkyEnvironment.cs`). Change them in the inspector (also during Play), from
`SimFlake > Environment`, or in a player with `--time-of-day night --weather cloudy`. It drives a runtime copy of the sky
material, the sun/moon light, ambient, fog and reflections. Scenes are saved with the plain material (`SkyBuilder` hooks
scene saving), so don't be surprised the `.mat` doesn't change. Per-time looks are the `Get(TimeOfDay)` table; per-scene
looks (mountain colours) live in the material. `SkyBuilder.SetUpScenesFromCommandLine [-screenshots <dir>]` adds it
to ChassisPlayground and BadlandsTraversal and renders every combination.
The `SimFlake` menu has submenus: `World Generation` (world/feature builders), `Environment` (sky, time of day, weather),
then build/test tools at the top level. Put new `[MenuItem]`s in the matching submenu (priorities 100s / 200s / 300s).
New files in `Assets/` only show up in an open editor after it refreshes (focus the window, or Ctrl+R).

`TaskWorlds/BadlandsTraversal` is a URC/CIRC-style traversal world with a mini_rover instance and a `FreeFlyCamera` for looking
around (right-drag + WASD/QE). The map is 3x3 terrain tiles of 500 m (1.5 km). Only the centre tile (-250..250) is
detailed; the outer tiles are coarse and rise into mountains at the edge. Contents: control station + rover start box at
(-20, -191); a 12 m escarpment with three ways up (west ramp ~15 deg, slot canyon ~4 m wide with rubble, the builder logs the narrowest gap, east talus ~22 deg + boulders);
a dry wash with 2 m banks and two fords; hoodoo field, boulder field, three buttes, a 3.5 m ledge band, a crater, dunes,
a dirt track (the long safe route) and ArUco posts. It's all generated by `Editor/BadlandsBuilder.cs` from one height
function and the layout constants at the top (menu `SimFlake > World Generation > Build Badlands Traversal World`, or
`-executeMethod SimFlake.EditorTools.BadlandsBuilder.BuildFromCommandLine [-screenshots <dir>]`, ~15 s, needs a GPU
for screenshots; it logs slope/height checks). Re-running rebuilds `Generated World` and `Assets/Worlds/Badlands` but
keeps anything else in the scene. Waypoint GNSS coordinates are written to `Worlds/Badlands/waypoints.csv`.
Tinted ground textures come from `tools/make_badlands_textures.py` (committed output in `Worlds/BadlandsTextures`).
Cliffs are terrain steps covered by generated strata wall meshes (the heightmap alone smears texture down a cliff).
Ramps (west ramp, east talus, the ledge band's ramp) are notches: sharp cut sides where they sink into the plateau, a wide
debris fan where they stand out below the cliff (`WithRamp`). Each cliff face is one mesh (`CliffPath`) that wraps into
every notch and the slot canyon and runs to the map edge; any terrain step without a face shows smeared texture.
Waypoint ArUco posts are linked instances of `Prefabs/Props/ArucoMarker_75cm.prefab` (`TripleAruco.id` set per
waypoint; marker textures only appear in Play mode). The builder places them by the bottom of the prefab's `Post` child.
Shrubs are terrain trees with capsule colliders (they block the rover and show on lidar); grass has no collision.
A rebuild rewrites ~40 MB of generated assets with new GUIDs, so commit rebuilds sparingly.
The centre terrain is soil (ruts, sinkage: see Soil model). A rebuild drops that: run `SimFlake > Soil > Add Soil To Badlands`
(`SoilYardBuilder.AddSoilToBadlands`) afterwards.

`TaskWorlds/UrcAutonomySite` is the real place: the Utah state-owned square mile north of MDRS that the URC 2027 rules
(1.e.xiii) name for the Autonomy Mission's route-finding sub-mission (38.411N -110.786E to 38.425N -110.768E), built from
public data by `Editor/UrcSiteBuilder.cs` (menu `SimFlake > World Generation > Build URC Autonomy Site`, or `-executeMethod
SimFlake.EditorTools.UrcSiteBuilder.BuildFromCommandLine [-screenshots <dir>]`, ~6 s + import; it logs its checks).
- **Frame:** x = east, z = north, metres from the centre of the square mile (38.418, -110.777 = UTM 12N 519467 E 4252218 N;
  grid north, 0.14 deg off true north). y = elevation - 1375 m. `UrcSiteBuilder.LatLon(x, z)` gives the real coordinates.
- **Real:** ground shape = USGS 3DEP 1 m lidar, unchanged (2048 m square, 2 x 2 terrains of 1025 samples; checked against the
  file to 1.5 mm). Ground colour = NAIP aerial photo with sun shading, shadows and shrubs removed, drawn by
  `Shaders/SiteTerrain` (terrain layers are grey detail textures, `albedo = layers * 2 * ground colour`; in Play
  `SoilRenderer` passes the same map to the soil shaders). ~98,000 shrubs and ~5,000 boulders stand where the photo shows
  them (shrubs wider than ~1 m have colliders). Backdrop = 16 km of 16 m DEM + Sentinel-2 colour, no collider.
- **Our own assets (no ThirdParty packs in this world):** `python3 -s tools/urc_site/make_assets.py [shrubs]` draws the
  textures into `Source/Layers` and `Source/Props` (~1.5 min; `shrubs` alone 4 s): a seamless detail + normal texture per soil
  (crusted silt, rippled sand, popcorn bentonite, lag gravel, angular talus, bedded sandstone), the boulder rock texture and
  the shrub card atlas. `Editor/UrcSiteProps.cs` makes the meshes at build time: seven shrub kinds (shadscale x 3, mat
  saltbush, Mormon tea, bunch grass, dead shrub; ~25 triangles of cut-out cards each, shader `SimFlake/DesertShrub`) and 14
  sandstone blocks (bedding-parallel slabs with fracture faces, convex colliders). Real in them: the colours of ground and
  rock (measured in the photo, `site.json` / `Props/palette.csv`; each boulder takes the measured colour nearest to the ground
  under it) and where each shrub and boulder stands. Not real: the close-up patterns (a 0.6 m photo can't show them; they are
  modelled on how these surfaces look) and which species a shrub is (chosen by soil and size: mat saltbush on clay, grass and
  small plants where the photo's blob is small). Pictures: `tools/urc_site/previews/assets_*.jpg`.
- **Interpreted, not measured:** the soil map (`Source/soil_classes.png`, one of six per 1 m cell; rules in
  `tools/urc_site/process_data.py`: sandstone = slope > 38 deg, talus = 24-38 deg or below a ledge, wash sand = drainage lines
  from flow accumulation, bentonite = shrub-free coloured or sloping ground, gravel = convex crests and gully beds, hardpack =
  the rest). Layer names pick the `GroundSpec` (`SoilSetup.ForLayer`). Picture: `tools/urc_site/previews/preview_soil.jpg`.
- **Ground under 1 m (judgment, the lidar can't show it):** table `UrcSiteBuilder.GroundTable`, one row per soil class.
  - *Relief:* the site has its own `GroundSpec`s (`Grounds/Mdrs_*.asset`, copies of the Utah ones; an existing asset is kept,
    so tune in the inspector or delete it to get the table's values back) with `roughnessCoarse` 40-160 mm peak = 1-4 cm RMS
    (the relief map's RMS is a quarter of its peak; the old 10-15 mm did nothing measurable). Anchor: profile roughness of
    desert surfaces (Greeley et al., Death Valley: 0.5-1.5 cm RMS at particle scale, 4-6 cm on bouldery fans).
  - *Stones:* `Scripts/Environment/StoneField.cs` (object `Stones` under `Generated World`). Stones exist only within 9 m
    (5-15 cm) / 25 m (bigger) of the rovers and the main camera, are worked out from position + seed (same every run, nothing
    in the scene file), and pushed loose stones stay where they ended up. Numbers and sizes follow the Mars rock
    size-frequency law (Golombek & Rapp 1997: area covered by rocks >= D is `k exp(-q D)`, `q = 1.79 + 0.152 / k`) with the
    rock abundance `k` and the largest stone per soil: hardpack 2 % / 12 cm (1.4 stones per m2), wash 3 % / 20 cm, clay 1 % /
    12 cm, lag gravel 10 % / 15 cm, talus 20 % / 50 cm (7 per m2 + 1.6 bigger), sandstone 8 % / 40 cm. 30-60 % lie loose
    (`LooseStone`: kinematic until touched, then a Rigidbody, 2300 kg/m3); the rest are embedded (fixed, 30-65 % buried).
    No stones in the start lane and at task points (`clearZones`). Player flags: `--no-stones`, `--stone-abundance 2`.
  - *Measured effect* (chassis_24, 1 m/s, `PlayShots`, smooth = `--no-stones --soil-relief 0`): flats 0.97 -> 0.95 m/s made
    good, vertical acceleration 0.7 -> 1.8 m/s2 RMS, body rocking 4.5 -> 13 deg/s RMS. Straight at the T1 ridge from the
    south-west it stalls on the talus toe after ~19 m either way (0.48 -> 0.41 m/s, climbs 1.5 -> 0.7 m).
  - *Limits:* wheels meet stones with the rigid PhysX sphere, not the tyre model, so each hit is a one-step jolt (peaks of
    40-200 m/s2 on the body: the IMU sees spikes a real tyre would soften; trust the RMS, not the peaks). Rills and vehicle
    ruts are not modelled as channels (they'd need a direction field in the relief shader): clay slopes only get high
    roughness. Shrubs and boulders stand on the plain terrain, so they can sit a few cm off the displaced surface.
    Next step if the jolts matter: the plan below.
- **Planned, not started: embedded stones in the soil height grid** (so the tyre model, with its rubber spring, meets them
  instead of the rigid PhysX sphere). Estimate: 1-2 h of Claude working time if the wheel model stays stable on stone edges
  (about 150 lines; most of the time is build + drive cycles of 3-5 min), up to half a day if it hops and needs model work.
  Step 3 (the rig, 20 s per run) tells which case it is before anything else is built. Steps:
  1. *Split `StoneField` into "which stones" and "make objects".* `Build` does both now. Move the generation into a pure
     function `Generate(big, cx, cz)` returning structs (x, z, diameter, height, sink, rotation, loose, soil index), no Unity
     objects, cached per patch in a `ConcurrentDictionary`: soil tiles are created inside `WheelSoilModel.Compute`, which
     runs on worker threads, so it may only touch `SoilTerrain.BaseHeight / SpecAt / Roughness` (plain arrays, safe).
     Keep the draw order of random numbers: the stones must stay where they are today.
  2. *Add the stones to the soil base.* In `SoilField.GetOrCreate` the undisturbed height is `BaseHeight + SoilNoise.Relief`.
     Add an optional hook (`ISoilBase.Stones(x0, z0, x1, z1)` or a delegate on `SoilField`) that returns the embedded stones
     overlapping the tile (+ their radius), and for each cell under a stone raise `t.b` to the stone's exposed top: a cap
     `exposed * sqrt(1 - (r / R)^2)` flattened for slabs, with the rim spread over at least 2 cells (a vertical step is what
     made wheels hop before, see "Mistakes already made"). Set those cells' yield pressure `t.sy` very high (e.g. 5 MPa) so a
     stone neither sinks nor ruts, and skip the "disturbed" mark there. Loose stones are NOT stamped: they move, they stay
     PhysX bodies.
  3. *Check the numerics on the wheel rig first.* Give `WheelRig` a `-stone <diameter>` option (one stone on the flat bed in
     the wheel's path, `FlatSoil` needs the same hook) and compare hub height and force over the stone with what geometry
     says, for `MiniRoverWheel` and `Chassis2024Wheel`, at High and Ultra, 0.3 and 1 m/s. Pass = no hop larger than the
     stone, peak force a few times the wheel load, not tens. If it fails: widen the rim, or raise `stabilityFactor`.
  4. *Stop the wheels hitting the stamped stones twice.* When soil is active, embedded stones keep their mesh and collider
     (lidar, cameras and the chassis belly still need them) but each `DriveWheel` collider must ignore them:
     `Physics.IgnoreCollision(wheelCollider, stoneCollider)` when `StoneField` takes a stone from its pool (wheel colliders:
     `DriveWheel.myColliders`, expose them through a static list). Soil quality Off, or a terrain without `SoilTerrain`:
     leave the PhysX contact as it is now.
  5. *Presets.* Low has 2.5 cm cells: a 5 cm stone is two cells. Either stamp only stones over ~8 cm there or keep PhysX
     stones at Low. Changing quality clears the grid; the stones come back identical because they are computed, not stored.
  6. *Nothing to do in the shader:* the stone mesh covers its own bump (check that the tessellated relief does not poke
     through small stones; if it does, write the bump into the soil window texture's deformation channel as well).
  7. *Measure* with `PlayShots` on the same two drives as the table above (flats heading 135; `--play-shots-at -301,258
     --play-shots-heading 45`), smooth / PhysX stones / stamped stones. Expected: speed and RMS about the same, vertical
     acceleration peaks down from 40-200 m/s2 to a few tens. Also `tools/drive_bench.sh` on soil (no stones there: results
     must not change) and soil ms per step (tile creation gets slower, steady state should not).
  8. Update this section and the Soil model's "Known rough edges" (rocks would then be half done: stamped stones use the
     tyre model, loose stones and boulders still the PhysX sphere).
- **Made up:** the task layout (URC publishes targets at the competition), following rules 1.e.iv-xvi: C2 + start gate on the
  real URC staging area (vehicle tracks in the photo), mission start on flat ground, astronaut figure, rock pick hammer
  (Rigidbody), drone pad, T1 on a clay ridge 420 m away (+16 m; 26-30 deg from most sides, 22 deg straight in from the
  south-west, 17 deg by the easiest winding route), T2 behind it without radio line of sight to the C2 mast. Also rocks
  under ~1 m (see above). Change the constants at the top of the builder and rebuild.
  Points are in `Worlds/UrcSite/waypoints.csv` (sim GNSS and true coordinates).
- **Rovers:** `chassis_24` (active) and a mini_rover (inactive: same topics) start behind the gate. Their `LocationStuff`
  origin is the real position of that spot, so `/gps/fix` reports real URC coordinates (flat-earth from the start: < 1.5 m off
  the true grid inside the task area). The rovers, camera and anything else outside `Generated World` survive a rebuild.
- **Data pipeline** (only to change the area or the rules; outputs are committed): `python3 -s tools/urc_site/fetch_data.py
  <raw dir>` downloads the DEM (USGS ImageServer) and imagery (Planetary Computer; the USGS NAIP server timed out), then
  `python3 -s tools/urc_site/process_data.py <raw dir>` (~30 s) writes `Worlds/UrcSite/Source`. `-s` because a pip numpy 2 in
  `~/.local` breaks the system OpenCV. Then rebuild the world.
- **Sizes:** generated assets 31 MB, scene 17 MB (5,000 boulder instances), source 24 MB. Splat maps are 2 m per texel because
  terrain assets are stored as text. A rebuild changes GUIDs: commit rebuilds sparingly.
- **Not done:** the astronaut does not walk (follow / come need a moving figure); no LED indicator on the rover; cliffs are as smooth as a 1 m DEM makes them (no overhangs); soil has not been checked
  against the ground. The other missions (delivery, equipment servicing, astrobiology) would be separate scenes; the URC
  North site of the same lidar tile is the obvious place for a delivery course.

## In-game menu and cameras
`Scripts/Tools/SimMenu.cs`: **Esc** in Play / a player opens a menu with five tabs plus ADVANCED. CONTROLS lists every hotkey except the menu's own
(Esc, Q/E) and greys out the ones the scene lacks; add new hotkeys to `SimMenu.Hotkeys()`. CAMERA has free/follow camera, Unity quality level, AA,
shadows, terrain detail, frame cap and the rover cameras' ROS resolution/rate/JPEG. SOIL has quality, ruts and soil ms/step. ROVER AND WORLD swaps the rover (ROVER > MODEL), shows the arm, has an ENVIRONMENT section
(the scene's `SkyEnvironment` time of day and weather, no hotkeys on purpose) and loads another scene of the build. ROS (`Tools/SimMenuRos.cs`)
has DRIVE INPUT (RoverFlake2 wheel speeds vs /cmd_vel direct, see Topics), the RoverFlake2 workspace path, START/STOP for preset nodes
(`RosNodeRunner.Presets`: wheel_speed_node, joy_linux_node / joy_node (exclusive: `Preset.group`; two drivers on /joy double the rate and mix two axis layouts), drive_control_node, joy_arm_control, rviz2), a field to run any command,
and each node's last output lines. Commands that need a terminal (teleop_twist_keyboard) don't work there. The joy presets are rate-capped
(~20 Hz: `coalesce_interval` 50 ms + `autorepeat_rate` 20): joy_linux's default publishes on every stick event (~1 kHz), the chain
triples it, rosbridge falls behind and the rover replays old commands ("phantom" driving; claude/rosbridge-camera-starvation.md, "Joysticks").
`MotorController` only subscribes to the current DRIVE INPUT's topics, and `Ros` unsubscribes on rosbridge when a topic's last local subscriber goes.
  Before loading a scene it calls `RosConnection.ResetForSceneChange()`: subscriptions and camera sockets belong to the old scene's
  objects, but the connection is `DontDestroyOnLoad`. rosbridge stays up. Scenes load by build index, so list them in the build.
- Rover swap (`Scripts/Robot/RoverCatalog.cs`): the list is `Resources/RoverCatalog.asset` (label + prefab; being in Resources puts
  both prefabs in every build). A scene rover is matched to an entry by its `DriveWheel.wheel` spec. The new rover goes where the old
  one's moving body was (same heading, 5 cm above the ground under it); the old one is destroyed. It uses the scene's own copy of
  that rover if there is one (copied at scene load, before anything moves, so per-scene overrides survive), else the prefab.
  `/gps/fix` continues in the old rover's frame (`LocationStuff.ContinueFrameOf`), `/odom` restarts at zero. The old rover's
  subscriptions are dropped and its camera sockets closed (`RosPublisher.Close`). Scene objects pointing into the old rover
  (`CameraOrbit`, `RGBCameraObject`, `CameraSwapper`, `SoilRenderer`, `StoneField`) are pointed at the same-named part of the new one.
  `--sim-menu-shots` also swaps to every rover and back (`rover_*.png`).
- It creates itself after the scene loads (`RuntimeInitializeOnLoadMethod`), so scenes need nothing, and it doesn't run in
  `-batchmode` (benches, PlayShots and the soil demo are unaffected). Settings are runtime only. In the editor,
  QualitySettings changes are undone when Play stops, because Unity would otherwise keep them in ProjectSettings.
- **ADVANCED** (button right of the tabs, `Tools/SimMenuAdvanced.cs`, a `partial` of SimMenu): MOTORS (`DriveMotorSpec` + derived top
  speed/torque, per-wheel thermal/rolling resistance), WHEELS & TYRES (`WheelSpec`, tread PhysicMaterial, wheel mass), ROVER BODY (every
  body's mass, `MotorController` turnSlipFactor/geometry/timeout, rocker differential, suspension friction/damping, solver iterations),
  SOIL (`SoilWorld` knobs, every `GroundSpec` + its PhysicMaterial, "clear ruts + apply"), PHYSICS (time scale, physics step, gravity).
  Nothing is saved: spec assets, PhysicMaterials and global physics settings are put back in `OnApplicationQuit` (so in the editor
  a test never changes the project), components reset with Play mode. Spec knobs come from the fields by reflection ([Header] =
  sub-heading, [Tooltip] shown on hover, [Range] = slider), so new spec fields appear without menu changes; fields only used at
  setup (wheel radius, collider shape, motor inertia) are read-only. Tyre changes rebuild the tyre model (`DriveWheel.ReconfigureSoil`).
  `--sim-menu-shots` saves each page and checks that doubling the motor supply voltage doubles top speed and RESET ALL undoes it.
- **Arrow keys drive the rover** (on by default; toggle, speed 0.5 m/s and turn rate 0.8 rad/s in CONTROLS > KEYBOARD DRIVE, Shift = x2).
  For quick tests without ROS. `SimMenu.KeyboardDrive` calls `MotorController.CmdVelCallback` only while a key is held (+ one stop
  on release), so ROS `/cmd_vel` still works. It drives the followed / selected rover. `CameraOrbit` (SampleScene) ignores the arrows while it is on.
  `--sim-menu-shots` drives 3 s forward + 2 s left through the same path and logs the distance and turn.
- **V** toggles `FollowCamera` (third person, on the MainCamera: RMB orbit, scroll zoom, R behind the rover, T next rover).
  Off = `FreeFlyCamera`, which gets added if the scene had none. Camera scripts ignore input while `SimMenu.IsOpen`.
- Look: original "field PDA" style, drawn with IMGUI. Textures are generated in code, so there are no image assets.
- Pictures: `<player> -screen-fullscreen 0 -screen-width 1600 -screen-height 900 --sim-menu-shots <dir>` saves the follow view
  plus one PNG per tab, then loads the next world of the build (follow camera kept) and saves that, then quits. It needs a **windowed** run: OnGUI never runs in `-batchmode`, and a batchmode
  backbuffer reads back blank.
- Linux players: fullscreen below the desktop resolution ("downscaled viewport" in the log) offsets the mouse, so IMGUI clicks
  miss. `SimMenu` switches such a player to the desktop resolution at start. Any `-screen-width/-height` run is saved to
  `~/.config/unity3d/DefaultCompany/rover_sim/prefs` and reused by the next normal launch, so restore that file after test runs.

## Drive physics (drivetrain model)
Code in `Scripts/Robot/Drivetrain/`, data in `Assets/Robots/Drivetrain/`, setup in `Editor/DrivetrainSetup.cs`.
- **`DriveWheel`** (on each wheel ArticulationBody) = motor + gearhead + speed controller. Every physics step it sets the
  joint's velocity drive (type **Force**, damping = P gain) and its force limit to what the motor can give right now:
  DC curve (stall torque falling to 0 at no-load speed), controller current limit (derated by a two-node winding
  temperature model), gearhead efficiency and rating. The setpoint ramps at the controller's acceleration limit.
  It also reads the wheel's collisions: normal load, contact point, slip ratio/angle, ground `GroundSpec`, and applies
  rolling resistance (coefficient x load x radius, against the spin). `MotorController` keeps its topics and just sets
  `DriveWheel.TargetSpeed`; wheels without a DriveWheel use the old ideal velocity drive.
- **Specs (ScriptableObjects, edit in the inspector):** `DriveMotorSpec` (`Motors/Maxon_ECXFL42M_GPX42_26` for chassis_2024
  from the maxon datasheet; `Motors/ES_36PG_3429_100` for mini_rover from the retailer spec, motor side derived; the
  notes field says which values are estimates), `WheelSpec` (collider shape/size, tread PhysicMaterial, rolling-resistance
  scale), `GroundSpec` per ground PhysicMaterial (rolling resistance now; Bekker/Wong soil values for later), all listed in
  `Resources/GroundSpecs.asset`. Unknown materials use its default.
- **Applying:** `SimFlake > Drivetrain > ...` or `-executeMethod SimFlake.EditorTools.DrivetrainSetup.SetUpFromCommandLine`
  (creates missing spec assets, sets the physics settings below, puts colliders/DriveWheels/HUD on both rover prefabs
  in place, sets their calibrated `turnSlipFactor`, rebuilds the test scene). `MiniRoverBuilder` does the same on a rebuild.
  To try a different wheel or motor: duplicate a spec, point the rover at it (`ApplyToPrefab`), run the bench.
- **Physics settings:** fixed timestep 5 ms (200 Hz; a rover-only scene still runs ~18x faster than real time), TGS
  solver, improved patch friction (the default patch friction exceeded Coulomb: the mini climbed 30 deg on 0.35 friction).
  `CustomDirt` is 0.6/0.5 (it was 0.03 static / 0.4), `Asphalt` 0.8/0.7. Tread material `RoverTread` = 1.0, Multiply.
- **HUD:** F8 in Play shows per-wheel setpoint, speed, torque, current, winding temperature, load, slip, ground.
- **Test bench:** `tools/drive_bench.sh [--setup] [-- <flags>]` builds `Scenes/Tests/DriveTest` and runs it headless (no ROS):
  per rover x ground, accel/cruise, top speed, point turn, arc, command timeout, 10/20/30 deg climbs + hold, perf. Table
  + JSON. Flags for experiments (no rebuild): `--drive-test-trace` (per-wheel state), `--fixed-dt`, `--turnslip`,
  `--speed-gain`, `--integral-time`, `--legacy-drive`, `--solver-iterations p,v`, `--contact-offset`, `--bogie-damping`,
  `--ground-shape box|terrain`, `--drive-test-rovers/-grounds/-only`. Open the scene and press Play to watch it.
- **Masses:** chassis_2024 is 35 kg without the arm (team numbers: wheels 0.9 kg each, < 50 kg with the ~15 kg arm; the
  URDF said 86 kg). `DrivetrainSetup.Chassis2024MassKg` scales the chassis + bogies (URDF proportions, not measured) to
  fit; wheel mass comes from the `Chassis2024Wheel` spec. The arm is a bench payload (`chassis_2024_arm`, +15 kg on the root).
- **Results now (CustomDirt, 35 kg / 50 kg with arm):** 0.125 s to 63 % of 1 m/s, top 3.8 m/s (no-load 3.87), climbs
  30 deg at 0.72 / 0.30 m/s (with the arm the windings reach ~107 °C on the way up) and holds, body vibration
  0.17 m/s² RMS, odom error < 0.5 %. `turnSlipFactor` 2.1 (2.05-2.15 match a point turn for 35-50 kg).
  mini_rover top 0.65 m/s, climbs 20 deg, stalls on 30 deg (0.35 friction cap). On asphalt with the arm chassis_2024
  binds in gentle arcs (high-grip skid steer, outer front wheel lifts): real skid steers do that too, but check against the rover.
- **Why it is built this way (measured, don't undo):**
  - Unity's `Velocity`/`Target` drive types compute their own gains and ignore damping (chassis_2024's imported wheels were
    `Velocity`: bang-bang ±max torque). Revolute drive damping is N·m per **rad/s** (target velocity is in deg/s).
  - Convex mesh tread cylinders jitter on the ground at small timesteps (chassis_2024: wheels unloaded 25-30 % of steps,
    0.5 g vertical vibration; worse at smaller dt; 32 vs 120 sides makes no difference). Sphere colliders roll cleanly.
    The cost: a sphere is 2 x radius wide, so side contact with rocks/walls starts ~75 mm early on chassis_2024.
  - Speed loop is P only by default: with rigid (no tyre compliance) contacts, six wheel integrators fight each other
    (3-4 N·m per wheel of internal torque on flat ground). P only still holds 30 deg slopes (< 1 cm creep).
  - `ArticulationBody.velocity` reads several % high with TGS (1.05 m/s reported at 0.99 true). Angular velocity is fine.
    Use position differences for linear speed (DriveWheel, HUD, bench do; the IMU already did).
- **Start-up:** for up to 1.5 s after start (until all six wheels carry load) the motors are off and, on soil, the tyres
  have 5 % grip, so the suspension settles. `DrivetrainSetup.LevelBogies` also turns each bogie in the prefab until its two
  wheels are level (chassis_2024's URDF zero pose had every bogie ~12 deg off, three wheels 10 cm in the air). Reason: with
  the speed loop holding every wheel and real grip, a rocker-bogie is locked solid and stays however it was dropped.

## Soil model (ruts, sinkage, tyres)
Code in `Scripts/Soil/`, shaders `Shaders/SoilTerrain*`, setup in `Editor/SoilSetup.cs`, `TreadBaker.cs`, `SoilYardBuilder.cs`.
On a terrain with a `SoilTerrain` component, in a scene with a `SoilWorld` (+ `SoilRenderer`) object, wheels with a
`DriveWheel` ignore the terrain's collider and `WheelSoilModel` computes the contact instead. Everything else (chassis,
rocks, props) still collides with the plain terrain. No SoilWorld, or quality Off = the PhysX contacts described above.
Scenes with soil: `Scenes/SoilYard` and BadlandsTraversal's centre terrain.
- **`WheelSoilModel`** (plain C#, per wheel, per physics step). The tread is cut into elements that turn with the wheel;
  each has a lug-top part and a void part at their own radii (from the baked tread). For each element under the soil:
  - normal pressure: tyre rubber as a spring layer (stiffness from Shore A and thickness) in series with the soil, which
    gives way by Bekker's law (`p = (kc/b + kphi) z^n`) once pressure exceeds what that spot has carried (crust, or an
    earlier wheel pass: wheels following in a rut sink less). On hard ground only lug tops touch;
  - shear: Janosi-Hanamoto on the slip accumulated since the element touched down; strength = rubber friction, limited
    by the soil's own strength (cohesion + pressure x tan phi), plus soil pushed by buried lug faces (passive earth
    pressure, separately for rolling and sideways directions: this is where tread pattern matters). One ellipse for
    traction + side slip, so skid-steer scrub comes out of it;
  - pressure acts along the wheel radius, so rolling resistance from sinkage needs no coefficient;
  - soil buried against the tyre's sides resists sideways sliding in a rut; rubber hysteresis (`lossTangent`) gives the
    small rolling resistance on rock.
  Then the grid is updated: pressed down to the tread, part of the volume (1 - `compaction`) piled beside the rut, soil dug
  out by slipping lugs (`excavation`) thrown behind (the wheel digs in), slopes slumped to `reposeAngle`.
- **`SoilField`**: sparse 64x64 tiles of cells (0.5-2.5 cm by preset), only where a wheel has been: deformation, highest
  pressure carried, a "disturbed" mark. Base = terrain height (same triangle interpolation as Unity, checked to 0.02 mm)
  + small-scale relief (`SoilNoise`, two wavelengths, amplitudes per `GroundSpec`), identical in physics and shader.
- **`GroundSpec`** per terrain layer (`SoilTerrain.layerSpecs`, assigned by layer name in `SoilSetup.ForLayer`):
  `Grounds/Utah_{LooseSand,Hardpack,BentoniteCrust,Gravel,Slickrock}`. Literature values for similar soils (Wong), adjusted
  for a dry desert by judgment. **Nobody has measured the URC site or our wheels: treat every number as a starting point.**
- **`WheelSpec` tyre fields:** `shoreA` (Mold Max 40 = 40), `rubberThickness`, `lossTangent`, `gripScale`,
  `gripPressureExponent`, and `tread` (`TreadData`: radius over 180 angles x 16 positions across, baked from the wheel's
  visual mesh by `TreadBaker`, which also finds where the tyre sits along the axle per wheel: `DriveWheel.tyreOffset`).
  mini_rover's STL gives: tip radius 65.5 mm, tread 46 mm wide, 24 staggered lugs per side x 3.1 mm, 36 % land.
  chassis_2024's STL has no tread (smooth, 101 mm wide). New tread = new STL, then `SimFlake > Drivetrain > Set Up Everything`.
- **Quality** (`SoilWorld.quality`; F9 in Play; `--soil-quality off|low|medium|high|ultra`; changing it clears the ruts).
  Mini rover, headless, soil cost per physics step for 6 wheels (computed on several cores, `--soil-serial` to compare):
  | preset | cells | tyre elements | physics | ruts drawn as | soil ms/step | faster than real time |
  |---|---|---|---|---|---|---|
  | Low | 2.5 cm | 60x3 | 200 Hz | shading only (no tessellation, for iGPUs) | 0.15-0.18 | ~10-12x |
  | Medium | 1.5 cm | 90x4 | 200 Hz | tessellated, coarse | 0.18-0.25 | ~9-11x |
  | High (default) | 1 cm | 120x8 | 200 Hz | tessellated | 0.23-0.28 | ~9-10x |
  | Ultra | 0.5 cm | 180x16 (lugs resolved, lug prints in the ruts) | 400 Hz | tessellated, fine | 0.45-0.6 | ~3x |
  Results agree across presets within ~10 % (sand cruise 0.23-0.25 m/s, hardpack identical). Low/Medium shake more
  (vertical accel 0.5-1.3 m/s² RMS on sand vs 0.2-0.3) because coarse ruts are bumpy for the following wheels.
  Frame rate with graphics has not been measured; the demo screenshots were rendered on the Radeon iGPU.
- **Rendering:** `SoilRenderer` keeps a window of the grid around the rover in a wrap-around texture (uploads only changed
  tiles) and swaps soil terrains to `SimFlake/SoilTerrain(Tess)` at runtime (the terrain's own material is restored on
  exit; Draw Instanced is turned off on them). Same look as Unity's terrain shader + displacement by tessellation +
  normals from the soil height + darker disturbed soil. Materials in `Resources/` only exist so player builds include
  the shaders. Tessellation needs the one-argument `vertex:` overload (see `SoilTerrain.cginc`).
- **Numerics (don't undo):** forces are explicit. Real rubber on rock is far too stiff for 5 ms steps, so the tread hangs on a
  series spring of `stabilityFactor` x wheel mass / dt² (hub sits 1-3 mm low); pressures and which tread parts touch stay
  physical. Shear give is floored the same way. Element soil state travels with the element (lug prints can't be stored in
  1 cm cells) and follows the slope when the element slips (without that a spinning wheel climbed any slope). Soil more than
  20 deg behind a moving wheel only gives passive pressure (else it leans on its own spoil). Hub velocity from positions.
  A hub jump > 0.5 m in a step is treated as a teleport.
- **Tools:**
  - Wheel test rig (no PhysX, seconds): `-executeMethod SimFlake.EditorTools.WheelRig.RunFromCommandLine [-wheel MiniRoverWheel]
    [-ground Utah_LooseSand,...] [-load 13] [-slope 20] [-summary 1] [-variants "smooth:lugs=0,tall:lugs=2,soft:shore=20,wide:width=1.5"]
    [-out x.csv]`: drawbar pull, torque, sinkage, efficiency vs slip for a wheel and changed copies of it.
  - Bench: `tools/drive_bench.sh -- --drive-test-grounds none --soils LooseSand,Hardpack [--soil-quality high]
    [--soil-relief 0] [--soil-stability 1] [--no-ruts]` runs the same tests on a soil terrain (slopes in the heightmap).
  - `Scenes/SoilYard` (`SimFlake > World Generation > Build Soil Yard`): five 12 m lanes (sand, hardpack, bentonite, gravel,
    slickrock), bumps, 15/25 deg ramps, a mini_rover. `--soil-demo <dir>` in a player (batchmode, with graphics) drives each
    lane and saves pictures (`Tools/SoilDemo.cs`; it respawns the rover, see gotchas).
  - HUD (F8) shows sinkage, contact patch, pressure and grip in use per wheel, and the soil preset.
- **Results (High, 5 ms):** mini_rover (7.7 kg, ~13 N per wheel):
  | soil | cruise at 0.3 cmd | odom over-reads | sinkage | 0.5 rad/s point turn gives | climbs |
  |---|---|---|---|---|---|
  | loose sand | 0.24 m/s | 22-30 % | 17-21 mm | 0.47-0.50 | 10 deg barely, not 20 |
  | hardpack | 0.29 | 4-5 % | 2 mm (lug prints) | 0.63 | 20 deg, not 30 |
  | bentonite crust | 0.27 | 12 % | 13 mm | 0.44 | 10 deg, not 20 |
  | gravel | 0.26 | 16 % | 5 mm | 0.58 | 10 deg, not 20 |
  | slickrock | 0.30 | 0.3 % | 0 | 0.85 | 30 deg |
  Rig, 13 N: sand sinkage 18 mm (Bekker's closed form for a rigid wheel: 14 mm), peak pull 0.23 x load at 60 % slip,
  rolling resistance 0.23 x load. chassis_2024 with arm: sand 15 mm, climbs 20 deg sand, not 30; hardpack and slickrock 30 deg.
  `turnSlipFactor` now depends on the ground (mini: 2.2-2.3 sand, 1.75 hardpack, 1.3 slickrock; chassis_2024 1.4-1.7):
  left at the old values, which match sand. In loose sand the mini rover sinks to within ~1 cm of its bogies' clearance.
- **Known rough edges:** chassis_2024 on soil is less tested than the mini (on slickrock some wheels read unloaded ~13 % of
  steps at 1 m/s; its STL has no tread). Rocks and other colliders still use the PhysX sphere, so no tyre squash or tread
  there. The belly and rocks collide with the undeformed terrain. Only wheels deform soil. Wet soil is not modelled. Ruts
  are not saved between runs, and only the window around the rover is drawn (the rest is kept in memory, up to 6000 tiles).
- **Ideas not done:** the same contact model against rocks (rays instead of the height grid); PI speed loops (the shear
  give should now allow it: test `--integral-time`); `ArticulationBody.velocity` bias; saving ruts; a tyre mesh that
  visibly squashes; calibrating soils and `turnSlipFactor` against the real rover (drawbar pull with a luggage scale,
  sinkage with a ruler, odometry vs GPS over 10 m are enough to pin the main numbers).

### Working on the soil model (how it was built, start here)
Read in this order: `WheelSoilModel.cs` (class comment, then `Compute`), `SoilField.cs`, `DriveWheel.SoilBegin/SoilEnd`,
`SoilWorld.FixedUpdate` (steps all wheels: Begin on the main thread, Compute in parallel, End on the main thread).
- **Iteration loop, fastest first.** (1) Wheel rig, ~20 s, no player build: change the model, run `WheelRig.RunFromCommandLine
  -summary 1 -load 13 [-slope 25]`, compare flat vs slope and against Bekker's closed form
  `z = (3W / (b (3-n) k sqrt(D)))^(2/(2n+1))`. (2) Bench, ~1-3 min: `tools/drive_bench.sh [--no-build] -- --drive-test-rovers
  mini_rover --drive-test-grounds none --soils LooseSand --drive-test-only accel,slope [--drive-test-trace]`. The trace prints
  per wheel: load, slip, height above ground, sinkage, spring, traction/limit, patch, contact normal angle, force. (3) Pictures:
  build a player of `Scenes/SoilYard`, run it `-batchmode` (not `-nographics`) with `--soil-demo <dir> --soil-demo-lanes 0,1
  --time-of-day sunset`, then look at the PNGs. Always check `build.log` for `error CS`: a failed build silently reruns the old player.
- **How bugs were found (the pattern that worked):** when the rover and the rig disagree, reproduce it on the rig and switch
  features off one at a time (`-relax 0`, `-nodig`, `-noaside`). When a rover result looks too good, add up the forces: total
  traction vs weight along the slope showed the contact normal was tilted uphill. When a wheel hangs in the air, print hub
  height and the bogie joint angle, and compare with `--soil-quality off` (PhysX).
- **Mistakes already made, don't repeat:**
  - Softening the rubber for stability let lug voids touch rock. Fix: physical rubber + one series spring per wheel.
  - Stamping each cell to its lowest element made coarse presets sink more. Fix: stamp the wheel's round profile at the cell centre.
  - Storing an element's soil level as a fixed world height: on slopes a slipping wheel pushed on soil it had left. Fix: advect by the ground slope.
  - Spoil thrown right behind the contact, and uncapped pressure behind the wheel: the wheel leaned on its own heap.
  - Sharp "grit" in the relief noise made heavy wheels hop at 1 m/s. Relief is now smooth (15-20 cm and 0.5-1 m).
  - A lower clamp on the landing force scale gave 10x forces on hard landings.
  - Changing a field default in code did nothing: the prefab kept the old value (see gotchas).
- **Tuning knobs and what they move:** `rubberFriction` and `frictionAngle` set peak pull (max slope ~ atan of rig "DP max");
  `kphi`, `n`, `crustStrength` set sinkage and so rolling resistance; `shearModulus` sets how much slip is needed;
  `excavation` sets how fast a spinning wheel digs in; `SoilWorld.stabilityFactor` trades hub sag against buzz on rock.
- **Not understood yet:** why bogies settle slowly (not at all with motors on) from a tilted pose under this model when
  PhysX contacts settle in 0.5 s (worked around by `LevelBogies` + start-up coast); the `TeleportRoot` tumble; why a smooth
  tyre out-pulls the lugged one above ~60 % slip on the rig (lug digging vs the soil-strength cap, may be wrong).
- **Good next steps:** measure frame rate per preset with graphics (iGPU and RTX 3060); give chassis_2024 a treaded STL and
  retest on slickrock; rays against rocks so tyres squash on them; calibrate against the real mini rover.

## Arm (steppers, gripper, breakage, mounting)
Code: `Scripts/Robot/ArmController.cs` (topics, commands), `Robot/Arm/` (`StepperMotorSpec`, `StepperDrive`, `ArmBreakage`,
`ArmLinkContacts`), `Robot/RoverPayload.cs` (mounting), setup `Editor/ArmSetup.cs` (menu `SimFlake > Arm > Set Up Arm`, or
`-executeMethod SimFlake.EditorTools.ArmSetup.SetUpFromCommandLine`: creates missing specs, keeps existing ones, rewrites Arm.prefab's
physics + breakage strengths and the chassis_2024 mount). Tests: `tools/arm_test.sh [--no-build] [-- --arm-test-only hold,move,limit,geom,grip,drop,push,rover,swapdrive]`
(builds ArmScene, runs `Tools/ArmTestRunner.cs` headless in its own test area, ~4 min, PASS/FAIL list; `--arm-test-nobreak` +
`PEAKS` lines = peak loads per joint, `--arm-test-trace`, `--arm-break-filter`, `--arm-test-ramps`).
- **Steppers** (one `StepperDrive` per axis, gripper = one motor driving both fingers). Spec per axis in `Robots/Arm/Motors/`
  (`StepperMotorSpec`: holding torque, step angle, pull-out curve, run/hold current, gear ratio/efficiency, output stiffness of
  gearbox + link, damping). **All values are estimates** (NEMA 17/23 + planetary sized to carry the URDF masses: base 50, shoulder
  150, elbow 50, forearm roll 10, wrist 18, wrist roll 5 N·m, gripper ~40 N squeeze per finger, ~45 deg/s): put in the real arm's.
  Model: torque = `T sin(Nr (field - rotor))` with a massless overdamped rotor in series with the flex spring; past half an electrical
  period the rotor drops a pole (lost steps, counted), so overload = stall + buzz at the limit, not more force. The PhysX drive gets
  the flex force with target = predicted position + force / stiffness (implicit stiffness only resists change within a step, the
  same trick as the soil model: stable for any gearbox stiffness at 5 ms). Velocity commands are open loop (lost steps stay lost);
  'P' moves close the loop on the joint position like before. Feedback topics publish the true joint angles.
- **Gripper**: closing on something builds force through the finger flex (`ArmController.fingerStiffness`, N/mm) until the motor
  stalls; when the command stops the field stays put, so it keeps squeezing (test: 37-38 N held 2 s with no command, a 1 kg box
  holds). Finger pads `GripperPad` (1.0/0.9, Maximum combine). This URDF's fingers close from 183 mm to ~57 mm at EE +122 deg (the
  joint named finger_left_joint moves finger_right). A cube pinched on two faces can pivot about the pinch line.
- **Fixed in the prefab** (ArmSetup): the import had gravity off on every link and `Velocity` drives (ignore damping);
  `ee_base_link` / `link_tt` had no inertial (Unity gave 1 kg, 1 kg·m²: the wrist was ~5000x too heavy to turn). Now gravity on,
  Force drives, 0.05 / 0.005 kg.
- **Collisions**: every arm object is on layer 8 `RoverArm`, which doesn't collide with itself (layer matrix in
  DynamicsManager). Don't go back to `Physics.IgnoreCollision` pairs: link_6 has 466 VHACD hulls and each call re-filters the whole
  body (~10 s for the arm). Changing a collider's layer at runtime costs the same, so the layer is set in the prefab. The arm does
  collide with the rover (link_0 not with chassis_link: parent/child in one articulation) and the world.
- **Breakage** (`ArmBreakage` on the arm root): load through each joint by Newton-Euler over everything beyond it (sum of
  m (a - g) from position differences, I alpha + w x I w, minus contact forces from collision impulses; PhysX 4 has no joint reaction
  forces). Checked: static loads = carried weight to 0.1 N, a box resting on the forearm adds its weight. Revolute joints count
  bending only (their own axis stalls instead), fingers the side force; fingers take moments about their COM (their URDF frames are
  far from the parts). Low-pass `filterTime` 0.1 s, nothing breaks in the first second. A broken link + everything beyond it is
  reparented to the scene root (own articulation, unpowered, drives = detent brake); axes beyond it report no power.
  `Repair()` swaps in a spare copy taken in Awake (home pose), carries over the menu settings, destroys debris.
  Strengths (`ArmBreakage.DefaultLinks`, estimates, set from measured peaks): mount 300, base 250, shoulder 180, elbow 90, forearm 100,
  wrist 30, wrist roll 25, gripper mount 25, fingers 6 N·m. Measured (0.1 s filter): sharp 4 cm steps at 1 m/s peak at 200 / 75 / 122
  / 53 / 63 N·m (rigid wheel contacts give single-step 200-450 m/s² jolts, see URC limits); the shoulder pushing the gripper into the
  ground at stall breaks the forearm (120); a 40 kg box dropped 1 m on the forearm breaks base + forearm; a 3 m/s wall hit usually
  breaks nothing (steppers skip and the arm folds back).
- **Mounting** (`RoverPayload` on chassis_2024's root, `ArmMount` child of chassis_link at (0.0025, -0.089, 0.2025)): the arm's base
  plate sits flat on the chassis mounting plate with the motor down through the front hole and the wire holes lined up (both 13.25 cm
  apart; found from the meshes, see `ArmSetup.MountPosition`). ArmSetup only moves an existing mount if it is still at the first guess,
  so a hand-tuned position survives. Configuration None / Arm / RoverLab (not modelled, refused). On mounting, rover colliders the
  arm already overlaps (the chassis hulls around the sunk motor; ChassisPlayground's camera override sits on the deck plate) are
  excluded from colliding with the arm, else the solver pushes them apart at full force and the arm breaks at once
  (`RoverPayload.IgnoreOverlaps`, logged; FIX ARM redoes it for the fresh copy via `RoverPayload.Mounted`). `tools/arm_test.sh` only uses the prefab; `--arm-test-only scene` in a player of a real
  scene attaches the arm to that scene's rover. Default None, so scenes and benches are unchanged. The arm
  joins the rover's articulation (fixed joint): adding/removing it rebuilds the articulation, tested while driving (no effect beyond
  one step). Rover + arm 39.9 kg (URDF arm ~5 kg; the team's ~15 kg arm is the bench's `chassis_2024_arm` payload). The mini rover has
  no RoverPayload. Players: `--rover-config arm`.
- **Arm camera**: `Arm Camera` on link_6 (turns with the wrist): `RGBCameraCapture` on `/arm/camera/image_raw` + `camera_info`,
  frame `arm_camera_link`, 640x480 at 15 Hz (two raw 30 Hz streams would be near rosbridge's limit). It is part of Arm.prefab, so a
  mounted arm, FIX ARM and ArmScene all have it where the prefab puts it: position it in ArmScene, then apply the overrides to the
  prefab. ArmSetup only creates it if missing.
- **ArmScene** has a Main Camera with `FreeFlyCamera` (it had no camera at all); V follows the arm (SimMenu's fallback when there
  is no rover: root articulation bodies). It also has a `SkyEnvironment` (midday): without one a player build has no ambient
  light or reflections there (the editor shows them anyway) and the arm's 75 % metallic URDF materials render almost black.
- **Menu**: ROVER AND WORLD > ROVER > CONFIGURATION, ROVER AND WORLD > ARM (state, FIX ARM, breakage on/off), ADVANCED > ARM (per-axis torque / lost
  steps / stall, squeeze, strengths with live and peak load, gripper, pads, motor specs). A toast says what broke.
- **Not done**: rover lab; rotor inertia and backlash; the wrist differential (motor-space commands are still treated as axis space);
  `/arm/moteus_feedback`; arm vs its own rover's body is not a break cause tested yet. `MotorController` finds wheels with a global
  `GameObject.Find` (two chassis_2024 in one frame pick up each other's wheels; RoverSwap deactivates the old one first).

## Importing URDFs
Package `com.unity.robotics.urdf-importer` v0.5.2 (git URL in `Packages/manifest.json`, downloads automatically).
Copy the ROS description package into `Assets/Robots/Descriptions/`, right-click the `.urdf` and choose **Import Robot from Selected URDF
file**. Use the VHACD collider option for exact meshes (chassis and arm). Then add `MotorController` / `ArmController`
and sensors, and save it as (or into) the prefab under `Assets/Robots/`.

## Checking your work (Claude can run all of this headless)
Other `-executeMethod` entry points in `SimFlakeTools`: `CheckMissingRefs` (opens every scene, logs missing scripts/prefabs
and broken references, exit 1 if any; run it after moving or deleting assets), `UpgradeScenes [-dryrun]` (scene migration), `ReportUnusedAssets` (see Housekeeping), and
`TestUrdfImport -urdf Assets/<copy>/x.urdf -prefab Assets/<copy>/x.prefab` (run it on a throwaway copy of a description
package without .meta files, then delete the copy).
```bash
U=~/Unity/Hub/Editor/2022.3.62f3/Editor/Unity     # the editor GUI must be closed (project lock)
$U -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.SimFlakeTools.SelfTest -logFile -   # compile + JSON/frames self-test
$U -batchmode -nographics -projectPath rover_sim -executeMethod SimFlake.EditorTools.SimFlakeTools.BuildLinux \
   -scenes Assets/Scenes/TaskWorlds/WaypointTask.unity -out /tmp/simbuild/rover_sim.x86_64 -logFile build.log
/tmp/simbuild/rover_sim.x86_64 -batchmode -nographics --rosbridge-url ws://127.0.0.1:9090 -logFile player.log &   # physics/sensors without a GPU window
python3 tools/sim_check.py --drive          # topic rates + drives the rover (cmd_vel at 20 Hz) and checks odom/GPS/IMU agree
                                             # (player needs --drive-input cmdvel, or --ros-run "ros2 run drive_control wheel_speed_node")
tools/drive_bench.sh                         # drivetrain test bench (no ROS), see Drive physics and Soil model
/tmp/simbuild/rover_sim.x86_64 -batchmode --play-shots <dir> [--play-shots-drive 1.0] [--play-shots-seconds 12] \
   [--play-shots-at x,z] [--play-shots-heading 135]   # any scene in Play (needs graphics): the rover drives straight, the log
                                             # gets speed made good + ride roughness, <dir> gets pictures (Tools/PlayShots.cs):
                                             # 0a-0c = close-ups of the rover before it drives, then the drive and terrain views
```
For automated tests, use an isolated ROS graph so you don't publish onto the team's domain:
`ROS_DOMAIN_ID=199 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, start rosbridge yourself on a spare port, and pass `--rosbridge-url`.

## Issue notes
Detailed write-ups of investigated issues live in `claude/` (index: `claude/README.md`); add new ones there with a one-line pointer here.

## Gotchas found during the Jazzy migration
- rosbridge's JSON (ujson) escapes `/` as `\/`. Envelope strings go through `RosJson.Unquote`; JsonUtility handles it in bodies.
- The imported wheel joints had velocity-drive damping 0, which produces no torque. `MotorController` sets
  `wheelDriveDamping` when it's 0 (legacy path, wheels without a `DriveWheel`).
- Skid steer slips: `turnSlipFactor` scales the effective track width for both commands and odometry. Calibrated on
  CustomDirt with the drive bench so a commanded 0.5 rad/s point turn matches: chassis_2024 2.1 (was 2.3 before the
  drivetrain model and the 86 -> 35 kg mass fix), mini_rover 2.2. Retune against the real rover if needed.
- Saving a scene writes every serialized field. Changing a default in code does NOT change existing scenes; use
  `SimFlakeTools.UpgradeScenes` (or the inspector). Same for prefabs: a field added to a component later keeps the
  default it had when the prefab was first imported, so setup code sets such values explicitly (`startupCoast`).
- `TerrainData.SetAlphamaps` before `AssetDatabase.CreateAsset(terrainData)` is lost (splat maps come back as layer 0
  everywhere). Create the asset first (`SoilYardBuilder`).
- `ArticulationBody.TeleportRoot` on a rover that has been driving can leave it tumbling (seen in the soil demo, cause
  not found). Destroy and re-instantiate instead (`DriveTestRunner.Spawn`, `SoilDemo.Teleport`).
- GPU readback row order: on OpenGL (Unity's default on Linux) camera RenderTexture readback is top-row-first.
  `ImageConversion.EncodeArrayToJPG` wants bottom-row-first. See `SensorCamera.ReadbackIsTopDown`. Verified with real
  frames on OpenGL only. If someone forces Vulkan and images are upside down, that's the place to fix.
- `Shader.Find` returns null in player builds for shaders nothing references. ArUco uses `Resources/ArucoUnlit.mat`.
- The URDF importer's `UrdfAssetPathHandler.GetRelativeAssetPath` cuts `len(Application.dataPath)` characters off any path longer than
  that, even a relative `Assets/...` one, so a relative URDF path only works when it happens to be short enough (the mini rover's was
  exactly 37 characters). Pass an absolute path to `UrdfRobotExtensions.Create` (the builder does); the symptom is "Creating asset at path
  sis_2026/Materials/Default.mat failed".
- Ubuntu 24.04 has no `libdl.so` symlink; the URDF importer's Assimp (COLLADA `.dae` only) needs it. `setup.sh` offers
  to create it. STL meshes and VHACD colliders work without it (tested: dev_arm imports to a prefab headless).
- Don't `pkill -f <pattern>` from a shell whose own command line contains the pattern. It kills itself.
- Stopping Play while the launch shell is still reading `~/.bashrc` used to interrupt RoverFlake2's `setup.bash`
  halfway (so no `ROS_DOMAIN_ID`/RMW) and leave an orphaned rosbridge on domain 0 that every later Play connected to.
  `RosBridgeLauncher` now runs with `set +m` and SIGKILLs the whole session on stop. If topics vanish, check the
  `[rosbridge] starting rosbridge (... ROS_DOMAIN_ID=...)` line in the Console.
- The arm listens on `/arm/command` directly (needs `rover_msgs`, i.e. RoverFlake2 sourced for rosbridge). The sim has no
  limit switches or differential wrist: 'H' drives axes back to the URDF zero (driving into a joint limit stalls the stepper and
  loses steps, like the real one), motor-space commands are treated as axis space.
  The HMI's status panels read `/arm/moteus_feedback`, which the sim does not publish (yet).
- Terrain shaders hit the 16 sampler limit in some light variants (the error only shows in a player build log: "maximum
  ps_4_0 sampler register index (16) exceeded"). `SoilTerrain.cginc` therefore declares layer textures with
  `UNITY_DECLARE_TEX2D` and lets the normal maps and the ground-colour map borrow their samplers. Don't add `sampler2D`s there.
- The scenes' `CompressedCameraCapture` components are disabled (team choice); enable them for a JPEG feed.

## Housekeeping
- `Library/`, `Temp/`, `Logs/`, `UserSettings/`, `Builds/` are local only (gitignored).
- `Assets/ThirdParty/TerrainSampleAssets` (Unity's free Terrain Sample Asset Pack, ~1.9 GB) is kept whole on purpose: the brushes,
  heightmaps and extra plants are for future worlds. Deleted assets can be restored from history with
  `git checkout <commit> -- <path>`; slimming clones would need a history rewrite (team decision, not done).
- Before deleting assets, run `SimFlakeTools.ReportUnusedAssets -check "Assets/<folder>,..." -out unused.txt`. It follows
  dependencies from everything SimFlake uses (scenes, Robots, Worlds, Scripts, Resources...) plus the files the builders
  load by path (`PathLoadedAssets()`; add to it when a builder or `tools/` script loads something new by path).
  Delete each file together with its `.meta`, then run `CheckMissingRefs` and the builders.
