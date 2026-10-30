#!/usr/bin/env python3
"""
SimFlake health check. Run while the sim is playing:

    python3 tools/sim_check.py            # which sim topics are alive, and at what rate
    python3 tools/sim_check.py --drive    # also drive forward then turn left, and check odom/GPS/IMU respond
    python3 tools/sim_check.py --arm      # also wiggle arm joint 1 and check /joint_states responds

Needs only a sourced ROS 2 Jazzy environment (no build).
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, JointState, LaserScan, NavSatFix
from std_msgs.msg import Float64MultiArray, String

TOPICS = [
    ("/gps/fix", NavSatFix),
    ("/imu/data", Imu),
    ("/scan", LaserScan),
    ("/odom", Odometry),
    ("/wheel_joint_states", JointState),
    ("/camera/image_raw", Image),
    ("/camera/camera_info", CameraInfo),
    ("/camera/image/compressed", CompressedImage),
    ("/joint_states", JointState),
    ("/arm/sim_feedback", Float64MultiArray),
    ("/chatter", String),
]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Checker(Node):
    def __init__(self):
        super().__init__("simflake_check")
        self.stamps = {t: [] for t, _ in TOPICS}
        self.last = {}
        for topic, typ in TOPICS:
            self.create_subscription(typ, topic, self._cb(topic), qos_profile_sensor_data)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_cmd = self.create_publisher(Float64MultiArray, "/arm/sim_command", 10)

    def _cb(self, topic):
        def cb(msg):
            self.stamps[topic].append(time.monotonic())
            self.last[topic] = msg
        return cb

    def spin_for(self, seconds, every=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if every:
                every()


def rate(stamps, window):
    recent = [s for s in stamps if s > time.monotonic() - window]
    if len(recent) < 2:
        return 0.0
    return (len(recent) - 1) / (recent[-1] - recent[0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--drive", action="store_true")
    ap.add_argument("--arm", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    n = Checker()
    print(f"Listening for {args.seconds:.0f}s ...")
    n.spin_for(args.seconds)

    print(f"\n{'topic':32} {'rate':>8}  notes")
    alive = 0
    for topic, _ in TOPICS:
        r = rate(n.stamps[topic], args.seconds)
        note = ""
        m = n.last.get(topic)
        if m is not None:
            alive += 1
            if isinstance(m, NavSatFix):
                note = f"lat {m.latitude:.7f} lon {m.longitude:.7f}"
            elif isinstance(m, Imu):
                a = m.linear_acceleration
                note = f"acc z {a.z:+.2f} m/s2, yaw {math.degrees(yaw_of(m.orientation)):+.1f} deg (ENU)"
            elif isinstance(m, LaserScan):
                hits = [x for x in m.ranges if math.isfinite(x)]
                note = f"{len(m.ranges)} rays, {len(hits)} hits" + (f", nearest {min(hits):.2f} m" if hits else "")
            elif isinstance(m, (Image, CameraInfo)):
                note = f"{m.width}x{m.height}" + (f" {m.encoding}" if isinstance(m, Image) else "")
            elif isinstance(m, CompressedImage):
                note = f"{m.format}, {len(m.data) / 1024:.0f} KiB"
            elif isinstance(m, Odometry):
                p = m.pose.pose.position
                note = f"x {p.x:+.2f} y {p.y:+.2f} yaw {math.degrees(yaw_of(m.pose.pose.orientation)):+.1f} deg"
            elif isinstance(m, JointState):
                note = ", ".join(m.name[:3]) + (" ..." if len(m.name) > 3 else "")
            elif isinstance(m, String):
                note = m.data
        print(f"{topic:32} {r:7.1f}Hz  {note if m is not None else '-- nothing received'}")

    ok = alive > 0
    if not ok:
        print("\nNothing received. Is the sim playing, and does the corner of the Game view say 'ROS: connected'?")
        print("Also check this terminal uses the same ROS_DOMAIN_ID as the sim (~/.bashrc).")

    if args.drive:
        ok = drive_test(n) and ok
    if args.arm:
        ok = arm_test(n) and ok

    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


def drive_test(n):
    print("\n--- drive test ---")
    if "/odom" not in n.last:
        print("no /odom, skipping")
        return False

    def snapshot():
        o, g, i = n.last.get("/odom"), n.last.get("/gps/fix"), n.last.get("/imu/data")
        return (o.pose.pose.position.x, o.pose.pose.position.y, yaw_of(o.pose.pose.orientation),
                g.latitude if g else None, g.longitude if g else None,
                yaw_of(i.orientation) if i else None)

    def drive(v, w, seconds):
        t = Twist()
        t.linear.x, t.angular.z = v, w
        n.spin_for(seconds, every=lambda: n.cmd_vel.publish(t))

    s0 = snapshot()
    drive(0.5, 0.0, 4.0)
    drive(0.0, 0.0, 1.0)
    s1 = snapshot()
    dist = math.hypot(s1[0] - s0[0], s1[1] - s0[1])
    print(f"forward 0.5 m/s for 4 s: odom moved {dist:.2f} m (expect ~2), dx {s1[0] - s0[0]:+.2f} dy {s1[1] - s0[1]:+.2f}")
    if s0[3] is not None:
        dn = (s1[3] - s0[3]) * math.pi / 180 * 6378137.0
        de = (s1[4] - s0[4]) * math.pi / 180 * 6378137.0 * math.cos(math.radians(s0[3]))
        print(f"  GPS moved {math.hypot(dn, de):.2f} m (north {dn:+.2f}, east {de:+.2f})")
        if s0[5] is not None:
            heading = math.atan2(dn, de)
            print(f"  GPS track heading {math.degrees(heading):+.0f} deg vs IMU yaw {math.degrees(s1[5]):+.0f} deg (ENU, should match)")

    drive(0.0, 0.5, 4.0)
    drive(0.0, 0.0, 1.0)
    s2 = snapshot()
    dyaw = math.degrees(math.atan2(math.sin(s2[2] - s1[2]), math.cos(s2[2] - s1[2])))
    imu_dyaw = None
    if s2[5] is not None:
        imu_dyaw = math.degrees(math.atan2(math.sin(s2[5] - s1[5]), math.cos(s2[5] - s1[5])))
    print(f"turn left 0.5 rad/s for 4 s: odom yaw {dyaw:+.0f} deg, IMU yaw {imu_dyaw if imu_dyaw is None else round(imu_dyaw)} deg (both should be positive, ~+115)")
    good = dist > 0.5 and dyaw > 10
    print("drive test", "PASSED" if good else "FAILED")
    return good


def arm_test(n):
    print("\n--- arm test ---")
    js = n.last.get("/joint_states")
    if js is None:
        print("no /joint_states, skipping")
        return False
    p0 = list(js.position)
    cmd = Float64MultiArray()
    cmd.data = [10.0, 0, 0, 0, 0, 0, 0]
    n.spin_for(2.0, every=lambda: n.arm_cmd.publish(cmd))
    cmd.data = [0.0] * 7
    n.arm_cmd.publish(cmd)
    n.spin_for(0.5)
    p1 = list(n.last["/joint_states"].position)
    moved = abs(p1[0] - p0[0])
    print(f"joint_1 moved {math.degrees(moved):.1f} deg after 2 s at 10 deg/s (expect ~20)")
    return moved > math.radians(2)


if __name__ == "__main__":
    main()
