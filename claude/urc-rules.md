# URC rules: where to find them, what the sim models, and the astronaut

Notes on the **University Rover Challenge 2027** rules as they affect SimFlake. The task worlds follow these rules, and so will the
playable astronaut. Rule numbers below are the PDF's (e.g. 1.e.vi).

## Where the rules are

- Page: <https://urc.marssociety.org/home/requirements-guidelines> ("University Rover Challenge Rules 2027.pdf" is embedded there;
  the page itself doesn't contain the rule text).
- PDF direct download (Google Drive id taken from that page, 2026-11-17):
  `https://drive.google.com/uc?id=1rsKkkrWkLyB8JmJG5VgD8T7EQ4WewH5A&export=download`. It has 8 pages, sha256
  `356549544e86041938e0d5b633cf7d0bb3875e2777f3be0d8bc676dd0734678d` (the copy read on 2026-11-27). `pdftotext -layout` converts it
  cleanly. If the hash changes, the rules were revised: re-read section 1.e and update this file.
- Q&A: the URC site's Q&A section. Its answers count as rules (PDF preamble). It hasn't been checked for astronaut rulings yet.
- Logistics / site map: <https://urc.marssociety.org/home/about-urc/urc-logistics> (Google My Maps KML, id `1AaiO9T-Z5DiZWh9EVBt0zuJavZs`).
- Dates: competition June 2-5, 2027 at MDRS (Hanksville, Utah). PDR due Dec 2, 2026; SAR due Mar 5, 2027.

## Summary of the rules that matter for the sim

**General (1.a, 2.a, 3.d):**
- Four missions of 100 points each, plus SAR: Astrobiology, Delivery, Equipment Servicing and Autonomy.
- Penalties add up (10 % + 20 % = 70 % of the points). An intervention, i.e. anyone touching the rover, costs 20 % of the mission each time (3.e).
- Rover limits: 50 kg deployed, 70 kg for all modules together, fits a 1.2 m cube at weigh-in, red push-button e-stop.
- C2 antenna at most 3 m high, within 5 m of the C2 station (2.c.iv). Operators can't see the course. Everything runs in daylight.
  GNSS uses WGS 84 with lat/lon given in decimal degrees.

**Delivery (1.c):** a staged course up to 1 km from the gate on sand, gravel, boulders, drops and steep slopes. Later stages are out of
radio line of sight. Objects weigh up to 5 kg, fit in 40 cm and have a graspable feature up to 7.5 cm across. Tasks include reading
signs carried by astronauts, opening toolboxes and searching large areas. A drone may help; a mini-rover may not.

**Equipment Servicing (1.d):** a lander up to 100 m away across flat ground, with everything between the ground and 1.5 m high. Tasks:
sample tube into a cache, cache into a drawer, a latch, autonomous typing of a 3-6 letter key (2 cm 4x4_50 ArUco tags at the
keyboard corners), moving a key between tubular locks (1 cm ArUco), a 1.5" cam-lock hose, a 1/4-turn valve, buttons, switches and knobs.

**Autonomy (1.e), 40 min on course, two sub-missions in any order:**
- 1.e.ii: LED on the back of the rover: **red** = autonomous, **blue** = teleoperated, **flashing green** = arrived at a target.
- 1.e.iii: while the rover is stopped, teams may reprogram anything (GNSS points, stay-out zones, algorithms) but may not drive.
- Astronaut Assistance (50 points): see the next section.
- Route finding (50 points), 1.e.xii-xviii:
  - Teams teleoperate to a mission start on flat ground, then drive autonomously to two targets.
  - Each target is an AR post: 3 faces of 20 x 20 cm, 0.5-1.5 m off the ground, ArUco 4x4_50 with a 1-cell white border (2.5 cm
    cells), the same tag on all three faces.
  - All points lie in the Utah state-owned square mile (38.411N -110.786E) to (38.425N -110.768E).
  - T1 is on a hill that can't be approached from every direction. T2 is behind the hill, out of radio line of sight of the C2 station.
  - Each target is worth 25 points for stopping within **1 m** of it and signalling arrival.
  - Abort and return: 20 % penalty. Teleoperating back: 50 % penalty, and only into areas the rover has already explored
    autonomously. Abort penalties are capped at 50 %.
  - A drone may scout while the rover helps the astronaut, but it must be back on the landing pad before the rover leaves the
    mission start.

## The astronaut (Astronaut Assistance sub-mission, 1.e.iv-xi, 50 points)

Mostly close to the rule text, because the details decide the points.

| Rule | Task | Points | What success means |
|---|---|---|---|
| 1.e.iv | **EVA suit** worn by a team member acting as the astronaut. It needn't be realistic (no pressure, backpack or oxygen needed). It **must have a camera and a microphone the C2 operators can monitor.** If there is a helmet, it must come off easily (heat). | 5 | The suit's camera and mic work at C2 |
| 1.e.v | **Go to astronaut:** drive autonomously to a given GNSS point where the astronaut waits | 5 | Stop within **3 m** of the GNSS point |
| 1.e.vi | **Follow!** The astronaut tells the rover to follow, then walks to a place announced during set-up. The rover follows autonomously. | up to 15 | Stop within **3 m** of the astronaut when they stop |
| 1.e.vii | **Stay!** The astronaut tells the rover to stay, then walks to a new spot **more than 20 m** away | 5 | The rover doesn't move until the next command |
| 1.e.viii | **Fetch!** The astronaut tells the rover to fetch a tool. The rover picks up a **rock pick hammer** autonomously. | 10 | Picked up autonomously (teleoperated = 0 points, but allowed) |
| 1.e.ix | **Come!** The rover drives to the astronaut autonomously | 5 | Stop within **3 m** of the astronaut |
| 1.e.x | **Give!** On command, the rover puts the hammer on the ground or drops it | 5 | Done autonomously |

**The four ways to give a command (1.e.vi).** The points depend on how the astronaut gives the Follow command: 5/15 = 1/3, 10/15 = 2/3,
15/15 = full points:
1. **A device the astronaut carries** (remote, phone, button): **5 points** (1/3).
2. **A sign the astronaut carries, recognized visually:** any image the team picks (an AR tag, written words or a picture): **5 points**.
3. **A spoken word or phrase, recognized by audio**, e.g. "follow": **10 points** (2/3).
4. **A beckoning gesture, given silently, recognized visually:** **15 points** (full).

The rules give these methods only for Follow. Stay, Fetch, Come and Give just say "will give a command"; nothing in the PDF names a
method or points scale for them. So presumably any method works, and the team chooses. Check the Q&A before relying on that.

**Aborts (1.e.xi):**
- The rover may stop on its own, or return to the astronaut on its own, at any time with no penalty. The command can then be given again.
- If C2 sends "stop" or "return to astronaut", that task (any of 1.e.v-x) loses **20 %**. If C2 teleoperates back to a visited place,
  that task loses **50 %**. The cap is 50 % per task; beyond it, aborts cost only time.
- Physical interventions (3.e: −20 % of the mission each) are **not** capped.

**Other rules that touch the astronaut:**
- The astronaut is a team member on the course, so the C2 radio rules apply to the suit's camera and mic link: 900 MHz sub-band limits
  cover "C2 station, rover, drone, and astronaut" (2.c.v).
- The EVA suit counts against the $26,000 budget (3.c.iii).
- In the Delivery Mission (1.c.iii-iv), astronauts carry **signs** the rover must read, e.g. which object to deliver. That's a
  separate mission and scene, but the same astronaut character could be reused there.

## What the sim has now (UrcAutonomySite)

Built by `rover_sim/Assets/Scripts/Editor/UrcSiteBuilder.cs`, `BuildSite()`; positions are constants at the top (`Astronaut`,
`FollowTo`, `Hammer`, `C2`, `Gate`, `DronePad`, `MissionStart`). Each point is snapped to the flattest ground within 10 m and kept
clear of stones and shrubs (`keepClear`, 4 m). Coordinates are listed in `rover_sim/Assets/Worlds/UrcSite/waypoints.csv`.

| Item | Where (world x, z) | Rule | What it is |
|---|---|---|---|
| Astronaut | (-537, 545) | 1.e.v | **A static figure, no script.** Children of `Generated World/Site (C2, start gate, astronaut)/Astronaut (GNSS point, 1.e.v)`: capsule torso, legs and arms, sphere helmet, gold visor, life-support box. White suit, about 1.86 m tall, facing the mission start. Primitive colliders (all but the visor), so lidar and cameras see it. |
| "Follow me" destination | (-500, 505), about 54 m from the astronaut | 1.e.vi | Orange flag. The csv notes that the figure doesn't walk there. |
| Rock pick hammer | (-514, 554) | 1.e.viii | `Rigidbody` 0.65 kg, about 33 cm: blue handle cylinder and steel head. The real one is an Amazon-linked rock pick (the PDF only gives the link). |
| C2 station + 3 m mast | (-608, 588) | 2.c.iv, 1.e.xvi | Canopy, tables, laptops. The mast is what the T2 line-of-sight check uses. |
| Start gate, drone pad, mission start | see csv | 1.e.xii, xiv | Posts, pad, blue flag |
| T1 / T2 AR posts | (-259, 300) / (-191, 234) | 1.e.xv-xvi | `ArucoMarker_75cm` prefab, ids 1 and 2. The builder checks that T2 has no line of sight to the mast. |
| Square-mile corners | ±785, ±777 | 1.e.xiii | Orange posts |

Everything in the table above was made up following the rules. URC publishes the real targets at the competition.

**Not done (rules-related):**
- The astronaut doesn't move, so Follow, Stay, Come and the walk away for Stay can't be tested.
- No command input: no device, sign, voice or gesture.
- No suit camera or microphone.
- No rover LED (1.e.ii); the sim has no topic for it yet.
- Fetch and Give need the arm on the rover (`--rover-config arm`). Picking up the hammer has never been tried in this scene.
- The delivery, equipment-servicing and astrobiology missions have no scenes. The URC North site in the same lidar tile is the obvious
  place for a delivery course.

## Notes for the playable astronaut (planning input, nothing built yet)

What the rules imply the character needs, roughly in order of points:
- **Walking** on the terrain: a CharacterController or a kinematic capsule that follows `Terrain.SampleHeight`. The site is real
  lidar with slopes up to 38°+, plus shrubs and boulders with colliders. Lidar and cameras need a collider that moves with the figure.
  `StoneField` only spawns stones near rovers and the main camera, so a walking astronaut far from the rover may need to be added as
  a "viewer" there.
- **A beckoning gesture** (15 points, the method worth most): an animated arm wave that the rover's cameras can see. To train or test
  a gesture detector, the animation should be realistic and vary between runs. Keep a "sign" mode too: a carried board showing an
  ArUco 4x4_50 tag, which can reuse the `TripleAruco` texture code.
- **Commands to the rover's software:**
  - Device (5 points): the obvious sim version is a ROS topic, e.g. a `std_msgs/String` such as `"follow"`, `"stay"`, `"come"`,
    `"fetch"` or `"give"`, published from Unity when the player presses a key.
  - Voice (10 points): the sim has no audio stream to ROS. Options are publishing a "heard word" topic as a stand-in, or real audio
    (a microphone stream is new ground for rosbridge; check bandwidth).
  - The topic name and message type should match whatever RoverFlake2 ends up using; nothing exists there yet (see below).
- **Suit camera** (5 points): an `RGBCameraCapture` on the helmet on its own topic (e.g. `/astronaut/camera/image_raw`).
  Mind the rosbridge camera budget in `claude/rosbridge-camera-starvation.md`.
- **The 20 m walk-away for Stay, and stopping for Follow/Come:** the player controls these, but a scripted mode (walk to `FollowTo`,
  stop, wait) would make it repeatable for headless tests, like `PlayShots`. A scoring helper could log distance rover ↔ astronaut
  when the astronaut stops (pass = within 3 m) and whether the rover moved during Stay.
- **Input conflicts:** the arrow keys drive the rover (`SimMenu.KeyboardDrive`), V toggles `FollowCamera`, T cycles rovers, and
  `FreeFlyCamera` uses right-drag + WASD/QE. A playable astronaut needs its own mode or keys, added to `SimMenu.Hotkeys()`.
- **The figure is generated** by the builder. If the playable astronaut becomes a prefab, have `BuildSite()` instantiate the prefab
  (as it does for the ArUco posts) instead of building primitives, so a world rebuild keeps it.

**RoverFlake2 side (checked 2026-11-27, commit 717fe34):** nothing about the astronaut, following or gestures exists yet. Related
pieces:
- `aruco_detector` (`DetectMarker`, publishes `identified` String plus `bounding_boxes`).
- `object_detection/yolo_detector_node.py` (YOLO on `/camera/color/image_raw`, the RealSense topic name; SimFlake publishes
  `/camera/image_raw`).
- `rover_vision/waypoint_node.py` (GNSS go-to: `/gnss_fix`, `/gnss_coord`, `/heading`, `/rover_state` → `/cmd_vel`,
  `/target_reached`). Its topic names differ from the sim's `/gps/fix`.
- `rover_sounds`.
