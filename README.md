# SimFlake
Unity simulator for the UBC Snowbots rover. It works with the ROS 2 Jazzy stack in RoverFlake2 on Ubuntu 24.04.

Workflow document (internal for rover members): https://docs.google.com/document/d/1W4p3Dt0q54WnVc1O408SYnV7q7jjMJXXKghRvAwRw5M/edit?tab=t.0

## Setup (once)
You need Ubuntu 24.04 with ROS 2 Jazzy (RoverFlake2's setup installs it).
```bash
git clone https://github.com/UBC-Snowbots/SimFlake.git
cd SimFlake && ./setup.sh
```
`setup.sh` installs `ros-jazzy-rosbridge-suite` (the only ROS package the sim needs). It also offers to install
Unity Hub and the exact Unity version the project uses. Then, in Unity Hub: **Projects > Add > Add project from disk >
`SimFlake/rover_sim`**, and open it.

## Using the sim
1. Open a scene in `Assets/Scenes`. `TaskWorlds/WaypointTask` has everything; `ArmScene` is the arm on a table.
2. Press **Play**. The sim starts rosbridge for you and the top-right of the Game view shows **ROS: connected**.
   It uses the ROS setup from your `~/.bashrc`, or from the terminal you launched Unity from.
   You don't need to source anything first.
3. Use it like the real rover:
   ```bash
   python3 tools/sim_check.py --drive        # is everything publishing? drives the rover a bit
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ros2 run rover_hmi main_hmi_node          # cmd_vel + arm panel (the sim arm listens on /arm/command, no sim_helper needed)
   rviz2                                     # /camera/image_raw, /scan, /odom ...
   ```

Topics, conventions and how the ROS link works are documented in [CLAUDE.md](CLAUDE.md).

## Adding things
- **Unity assets** (Asset Store or otherwise): import them into `rover_sim/Assets` and commit. Everyone gets them on pull.
- **URDF robots**: copy the ROS description package into `Assets/`, right-click the `.urdf` and choose
  **Import Robot from Selected URDF file** (VHACD for exact colliders). Then add `MotorController` / `ArmController` and sensors.
- **New sensors / topics**: see "Writing sim code" in [CLAUDE.md](CLAUDE.md). It's a few lines with `Ros.Advertise` / `Ros.Subscribe`.

## Troubleshooting
- *"ROS: waiting for rosbridge"*: check the Console for `[rosbridge]` lines. Run `./setup.sh` if rosbridge is missing,
  or start it by hand: `ros2 run rosbridge_server rosbridge_websocket`.
- *Topics don't show up in `ros2 topic list`*: your terminal and the sim are on different `ROS_DOMAIN_ID`s. The sim uses
  the one from `~/.bashrc` (or from the terminal that launched Unity).
- *Unity says the project is locked*: another Unity instance has it open.
