using System;
using SimFlake.Ros;
using UnityEngine;

/// <summary>
/// Drives the 6 rover wheels and publishes wheel odometry. Where the wheel commands come from is <see cref="InputMode"/>
/// (Esc menu > ROS > DRIVE INPUT, or <c>--drive-input roverflake2|cmdvel</c>; one setting for every rover):
///
/// ROVERFLAKE2 (default): stands in for RoverFlake2's motor_control_node (drive_control/src/motor_control.cpp), so the rest of
/// RoverFlake2's drive stack runs unchanged (/cmd_vel -> wheel_speed_node -> wheel speeds -> here):
///   /left_wheel_speeds, /right_wheel_speeds (sub)  std_msgs/Float64MultiArray, data[0] = that side's speed in m/s at the wheel,
///                                                  in motor direction (the left motors are mirrored: wheel_speed_node sends
///                                                  -linear + angular for them). Below 0.05 m/s = stop, clamped to 3.5 m/s,
///                                                  ramped at 50 rad/s². Like the real node, the last speed is kept until a new
///                                                  one arrives (its Phidget failsafe is fed by its own 20 Hz loop).
///   /drive/feedback (pub, 1 Hz)                    rover_msgs/DriveFeedback, motors 0-2 = right, 3-5 = left, motor direction.
/// CMDVEL (bypass RoverFlake2's drive control): skid-steer straight from
///   /cmd_vel (sub)  geometry_msgs/Twist  linear.x [m/s], angular.z [rad/s], with <see cref="turnSlipFactor"/>. Wheels stop if no
///                   command arrives for <see cref="commandTimeout"/> s (publish continuously; teleop and nav2 do).
/// Always:
///   /odom                nav_msgs/Odometry      dead-reckoned from wheel joint speeds (frame odom -> base_link)
///   /wheel_joint_states  sensor_msgs/JointState wheel positions [rad], velocities [rad/s], effort = wheel torque [N·m]
///
/// The in-sim tools (keyboard drive, benches, PlayShots) call <see cref="CmdVelCallback"/> directly in either mode: the latest
/// command wins. Wheels with a <see cref="DriveWheel"/> get their speed target through it (motor + gearhead + controller model);
/// wheels without one fall back to the old ideal velocity drive.
/// </summary>
public class MotorController : MonoBehaviour
{
    const int motorCount = 6;

    public enum DriveInput { RoverFlake2, CmdVel }
    static DriveInput inputMode = InputFromCommandLine();

    /// <summary>Where every rover's wheel commands come from. Changing it stops the wheels.</summary>
    public static DriveInput InputMode
    {
        get => inputMode;
        set
        {
            if (value == inputMode) return;
            inputMode = value;
            foreach (var mc in FindObjectsOfType<MotorController>()) { mc.StopWheels(); mc.SubscribeCommands(); }
            Debug.Log($"[MotorController] drive input: {value}");
        }
    }

    static DriveInput InputFromCommandLine()
    {
        var a = Environment.GetCommandLineArgs();
        int i = Array.IndexOf(a, "--drive-input");
        if (i >= 0 && i + 1 < a.Length && a[i + 1].Replace("_", "").ToLowerInvariant() == "cmdvel") return DriveInput.CmdVel;
        return DriveInput.RoverFlake2;
    }

    [Header("Wheels")]
    [Tooltip("Extra per-wheel sign flip, applied on top of the left/right convention. Leave at 1 unless one wheel spins the wrong way.")]
    [SerializeField] private int[] motor_dirs = { 1, 1, 1, 1, 1, 1 };

    [SerializeField]
    private string[] wheelJointNames = {
        "back_left_wheel_link",
        "back_right_wheel_link",
        "left_front_wheel_link",
        "left_mid_wheel_link",
        "right_front_wheel_link",
        "right_mid_wheel_link"
    };

    [Tooltip("Found automatically by name if left empty.")]
    [SerializeField] private ArticulationBody[] wheelJoints = new ArticulationBody[motorCount];

    [Tooltip("True for wheels on the left side of the rover (positive joint speed = forward on the left, backward on the right).")]
    [SerializeField] private bool[] isLeftWheel = { true, false, true, true, false, false };

    [Tooltip("Velocity-drive damping applied when the imported joint has none (a velocity drive with 0 damping produces no torque).")]
    [SerializeField] private float wheelDriveDamping = 1000f;

    [Header("Topics")]
    [SerializeField] private string cmdVelTopicName = "/cmd_vel";
    [SerializeField] private string leftWheelTopic = "/left_wheel_speeds";
    [SerializeField] private string rightWheelTopic = "/right_wheel_speeds";
    [SerializeField] private string driveFeedbackTopic = "/drive/feedback";

    // motor_control.h (RoverFlake2)
    const double MinVelocityMs = 0.05, MaxVelocityMs = 3.5, MaxAccelRads = 50.0, FeedbackPeriod = 1.0;

    [Header("Geometry")]
    [Tooltip("Measured from the wheel colliders at start when 'autoMeasureGeometry' is on.")]
    [SerializeField] private double wheel_radius_meters = 0.3;
    [Tooltip("Distance between left and right wheel centres. Measured at start when 'autoMeasureGeometry' is on.")]
    [SerializeField] private double track_width_meters = 0.8;
    [SerializeField] private bool autoMeasureGeometry = true;
    [Tooltip("Skid-steer turning needs more wheel speed difference than the geometric track suggests. 1 = ideal.")]
    [SerializeField] private double turnSlipFactor = 2.3;

    [Header("Safety")]
    [Tooltip("DRIVE INPUT = CMDVEL: stop the wheels if no /cmd_vel arrives for this long (s). 0 = never.")]
    [SerializeField] private float commandTimeout = 0.5f;

    [Header("Odometry")]
    [SerializeField] private bool publishOdometry = true;
    [SerializeField] private string odomTopic = "/odom";
    [SerializeField] private string odomFrame = "odom";
    [SerializeField] private string baseFrame = "base_link";
    [SerializeField] private bool publishOdomTf = false;
    [SerializeField] private string wheelJointStatesTopic = "/wheel_joint_states";
    [SerializeField] private float odomRateHz = 50f;

    RosPublisher<nav_msgs.msg.Odometry> odomPub;
    RosPublisher<sensor_msgs.msg.JointState> wheelStatePub;
    RosPublisher<tf2_msgs.msg.TFMessage> tfPub;

    double odomX, odomY, odomYaw;
    double lastV, lastW;
    float odomTimer;
    readonly DriveWheel[] driveWheels = new DriveWheel[motorCount];
    float lastCommandTime = float.NegativeInfinity;
    bool stoppedByTimeout = true;

    // RoverFlake2 path: per wheel, forward rad/s.
    bool rf2Active;                                     // the newest command came from the wheel-speed topics
    readonly double[] rf2Target = new double[motorCount];
    readonly double[] wheelCmd = new double[motorCount];   // forward rad/s last sent to each wheel (any path)
    readonly double[] motorTargetPos = new double[motorCount];
    RosPublisher<rover_msgs.msg.DriveFeedback> feedbackPub;
    double feedbackTimer;

    /// <summary>Wheel-odometry pose in the odom frame (x forward at start, y left, yaw CCW), m / rad.</summary>
    public double OdomX => odomX;
    public double OdomY => odomY;
    public double OdomYaw => odomYaw;
    public double WheelRadius => wheel_radius_meters;
    public double TrackWidth => track_width_meters;
    public double TurnSlipFactor { get => turnSlipFactor; set => turnSlipFactor = value; }
    public DriveWheel[] DriveWheels => driveWheels;

    readonly System.Collections.Generic.List<RosSubscription> subs = new System.Collections.Generic.List<RosSubscription>();

    // Swapped out at runtime (Esc menu): stop receiving commands for a rover that no longer exists.
    void OnDestroy() { foreach (var s in subs) s.Unsubscribe(); subs.Clear(); }

    void Start()
    {
        for (int i = 0; i < motorCount; i++)
        {
            if (wheelJoints[i] == null)
            {
                var go = GameObject.Find(wheelJointNames[i]);
                if (go != null) wheelJoints[i] = go.GetComponent<ArticulationBody>();
            }
            if (wheelJoints[i] == null)
            {
                Debug.LogError($"[MotorController] Wheel '{wheelJointNames[i]}' not found (needs an ArticulationBody)");
                continue;
            }
            driveWheels[i] = wheelJoints[i].GetComponent<DriveWheel>();
            if (driveWheels[i] != null) continue;   // the motor model owns the drive
            // Legacy ideal velocity drive.
            wheelJoints[i].jointFriction = 1;
            wheelJoints[i].angularDamping = 1;
            var drive = wheelJoints[i].xDrive;
            if (drive.damping <= 0) { drive.damping = wheelDriveDamping; wheelJoints[i].xDrive = drive; }
        }

        if (autoMeasureGeometry) MeasureGeometry();
        if (driveWheels[0] != null && driveWheels[0].motor != null)
            Debug.Log($"[MotorController] motor model {driveWheels[0].motor.name}: {driveWheels[0].motor.Summary}");
        else if (wheelJoints[0] != null)
        {
            var d = wheelJoints[0].xDrive;
            Debug.Log($"[MotorController] wheel drive: type {wheelJoints[0].jointType}, stiffness {d.stiffness}, damping {d.damping}, forceLimit {d.forceLimit}");
        }

        // queue_length 1: only the newest command matters, a backlog in rosbridge would only add lag.
        SubscribeCommands();

        if (publishOdometry)
        {
            odomPub = Ros.Advertise<nav_msgs.msg.Odometry>(odomTopic, 2);
            wheelStatePub = Ros.Advertise<sensor_msgs.msg.JointState>(wheelJointStatesTopic, 2);
            if (publishOdomTf) tfPub = Ros.Advertise<tf2_msgs.msg.TFMessage>("/tf", 2);
        }
    }

    void MeasureGeometry()
    {
        // Wheel radius: half the vertical size of the wheel's colliders (rover is upright at start).
        double radiusSum = 0; int radiusCount = 0;
        Vector3 leftSum = Vector3.zero, rightSum = Vector3.zero; int nl = 0, nr = 0;
        Transform rover = transform;
        for (int i = 0; i < motorCount; i++)
        {
            var w = wheelJoints[i];
            if (w == null) continue;
            var cols = w.GetComponentsInChildren<Collider>();
            if (cols.Length > 0)
            {
                Bounds b = cols[0].bounds;
                foreach (var c in cols) b.Encapsulate(c.bounds);
                radiusSum += b.extents.y; radiusCount++;
                Vector3 local = rover.InverseTransformPoint(b.center);
                if (isLeftWheel[i]) { leftSum += local; nl++; } else { rightSum += local; nr++; }
            }
        }
        if (radiusCount > 0) wheel_radius_meters = radiusSum / radiusCount;
        if (nl > 0 && nr > 0)
        {
            Vector3 d = rover.TransformVector(rightSum / nr - leftSum / nl);
            track_width_meters = d.magnitude;
        }
        Debug.Log($"[MotorController] wheel radius {wheel_radius_meters:F3} m, track width {track_width_meters:F3} m");
    }

    int SideSign(int i) => (isLeftWheel[i] ? 1 : -1) * (motor_dirs.Length > i && motor_dirs[i] != 0 ? motor_dirs[i] : 1);

    // Only the current mode's topics: rosbridge then doesn't carry the other ones at all (a joystick chain publishes
    // /cmd_vel and both wheel-speed topics; everything rosbridge forwards adds to its backlog).
    void SubscribeCommands()
    {
        foreach (var s in subs) s.Unsubscribe();
        subs.Clear();
        if (inputMode == DriveInput.CmdVel)
            subs.Add(Ros.Subscribe<geometry_msgs.msg.Twist>(cmdVelTopicName, OnCmdVelTopic, queueLength: 1));
        else
        {
            subs.Add(Ros.Subscribe<std_msgs.msg.Float64MultiArray>(leftWheelTopic, m => OnWheelSpeeds(m, true), queueLength: 1));
            subs.Add(Ros.Subscribe<std_msgs.msg.Float64MultiArray>(rightWheelTopic, m => OnWheelSpeeds(m, false), queueLength: 1));
        }
    }

    // ---------------------------------------------------------------- RoverFlake2 motor_control_node

    void OnWheelSpeeds(std_msgs.msg.Float64MultiArray msg, bool left)
    {
        if (inputMode != DriveInput.RoverFlake2) return;
        double v = msg.data == null || msg.data.Length == 0 ? 0 : msg.data[0];   // like the node: only data[0]
        double rad = Math.Abs(v) < MinVelocityMs ? 0 : Math.Max(-MaxVelocityMs, Math.Min(MaxVelocityMs, v)) / wheel_radius_meters;
        for (int i = 0; i < motorCount; i++)
            if (isLeftWheel[i] == left) rf2Target[i] = rad * MotorSign(i);
        if (!rf2Active) { rf2Active = true; feedbackTimer = 0; }
    }

    // The real rover's left motors are mounted mirrored: positive motor speed = forward on the right, backward on the left.
    int MotorSign(int i) => isLeftWheel[i] ? -1 : 1;

    // Motor index on the real rover (0-2 right, 3-5 left) -> wheel index here (sides in wheel order).
    int[] motorToWheel;
    int[] MotorToWheel()
    {
        if (motorToWheel != null) return motorToWheel;
        var l = new System.Collections.Generic.List<int>();
        for (int i = 0; i < motorCount; i++) if (!isLeftWheel[i]) l.Add(i);
        for (int i = 0; i < motorCount; i++) if (isLeftWheel[i]) l.Add(i);
        return motorToWheel = l.ToArray();
    }

    void StepRoverFlake2(double dt)
    {
        double dv = MaxAccelRads * dt;
        for (int i = 0; i < motorCount; i++)
        {
            double c = wheelCmd[i] + Math.Max(-dv, Math.Min(dv, rf2Target[i] - wheelCmd[i]));
            SetWheel(i, c);
            motorTargetPos[i] += c * MotorSign(i) * dt;
        }
        feedbackTimer -= dt;
        if (feedbackTimer > 0) return;
        feedbackTimer += FeedbackPeriod;
        if (feedbackPub == null) feedbackPub = Ros.Advertise<rover_msgs.msg.DriveFeedback>(driveFeedbackTopic, 2);
        var m = new rover_msgs.msg.DriveFeedback
        {
            valid_data = new bool[motorCount], target_velocities = new double[motorCount], actual_velocities = new double[motorCount],
            target_positions = new double[motorCount], actual_positions = new double[motorCount],
        };
        var map = MotorToWheel();
        for (int k = 0; k < map.Length; k++)
        {
            int i = map[k];
            var j = wheelJoints[i];
            m.target_velocities[k] = rf2Target[i] * MotorSign(i) * wheel_radius_meters;
            m.target_positions[k] = motorTargetPos[i];
            if (j == null || j.jointPosition.dofCount == 0) continue;
            m.valid_data[k] = true;
            double fwd = j.jointVelocity[0] * SideSign(i);   // rad/s forward
            m.actual_velocities[k] = fwd * MotorSign(i) * wheel_radius_meters;
            m.actual_positions[k] = j.jointPosition[0] * SideSign(i) * MotorSign(i);
        }
        feedbackPub.Publish(m);
    }

    // ---------------------------------------------------------------- /cmd_vel (bypass) and in-sim tools

    void OnCmdVelTopic(geometry_msgs.msg.Twist msg)
    {
        if (inputMode == DriveInput.CmdVel) CmdVelCallback(msg);
    }

    void StopWheels()
    {
        rf2Active = false;
        Array.Clear(rf2Target, 0, motorCount);
        for (int i = 0; i < motorCount; i++) SetWheel(i, 0);
    }

    /// <summary>Skid-steer straight from a twist (the /cmd_vel bypass; keyboard drive and benches call it in either mode).</summary>
    public void CmdVelCallback(geometry_msgs.msg.Twist msg)
    {
        double v = msg.linear.x;   // m/s forward
        double w = msg.angular.z;  // rad/s, positive = turn left (counter-clockwise from above)
        lastCommandTime = Time.time; stoppedByTimeout = false;
        rf2Active = false;
        double half = track_width_meters * turnSlipFactor / 2.0;
        double fwd = v / wheel_radius_meters, diff = w * half / wheel_radius_meters;   // wheel rad/s
        // Above the motors' top speed both sides would clip to the same speed and the turn would vanish. Keep the
        // turn and give up forward speed instead (like a differential-drive mixer that prioritises steering).
        double lim = MaxWheelSpeed();
        if (lim > 0 && Math.Abs(fwd) + Math.Abs(diff) > lim)
        {
            if (Math.Abs(diff) > lim) diff = Math.Sign(diff) * lim;
            fwd = Math.Sign(fwd) * Math.Max(0, lim - Math.Abs(diff));
        }
        double left = fwd - diff;
        double right = fwd + diff;
        for (int i = 0; i < motorCount; i++) SetWheel(i, isLeftWheel[i] ? left : right);
    }

    // rad/s the weakest drive-model wheel can reach (0 = unknown / legacy drive: no limit).
    double MaxWheelSpeed()
    {
        double lim = double.MaxValue;
        foreach (var dw in driveWheels)
            if (dw != null && dw.motor != null) lim = Math.Min(lim, dw.motor.NoLoadWheelSpeed * 0.95);
        return lim == double.MaxValue ? 0 : lim;
    }

    void SetWheel(int i, double forwardRadS)
    {
        wheelCmd[i] = forwardRadS;
        SetJointTarget(i, (float)(forwardRadS * Mathf.Rad2Deg * SideSign(i)));
    }

    void SetJointTarget(int i, float degPerSec)
    {
        if (driveWheels[i] != null) { driveWheels[i].TargetSpeed = degPerSec * Mathf.Deg2Rad; return; }
        var joint = wheelJoints[i];
        if (joint == null) return;
        var drive = joint.xDrive;
        drive.targetVelocity = degPerSec;
        joint.xDrive = drive;
    }

    void FixedUpdate()
    {
        if (rf2Active) StepRoverFlake2(Time.fixedDeltaTime);
        else if (!stoppedByTimeout && commandTimeout > 0 && Time.time - lastCommandTime > commandTimeout)
        {
            stoppedByTimeout = true;
            for (int i = 0; i < motorCount; i++) SetWheel(i, 0);
        }
        if (!publishOdometry) return;

        // Average forward wheel speed per side (rad/s).
        double l = 0, r = 0; int nl = 0, nr = 0;
        for (int i = 0; i < motorCount; i++)
        {
            var j = wheelJoints[i];
            if (j == null || j.jointVelocity.dofCount == 0) continue;
            double ws = j.jointVelocity[0] * SideSign(i);
            if (isLeftWheel[i]) { l += ws; nl++; } else { r += ws; nr++; }
        }
        if (nl == 0 || nr == 0) return;
        l /= nl; r /= nr;

        double v = wheel_radius_meters * (l + r) / 2.0;
        double w = wheel_radius_meters * (r - l) / (track_width_meters * turnSlipFactor);
        double dt = Time.fixedDeltaTime;
        double midYaw = odomYaw + w * dt / 2.0;
        odomX += v * Math.Cos(midYaw) * dt;
        odomY += v * Math.Sin(midYaw) * dt;
        odomYaw = Math.IEEERemainder(odomYaw + w * dt, 2 * Math.PI);
        lastV = v; lastW = w;

        odomTimer += Time.fixedDeltaTime;
        if (odomTimer + 1e-4f >= 1f / odomRateHz)
        {
            odomTimer = 0;
            PublishOdometry();
        }
    }

    void PublishOdometry()
    {
        var stamp = RosTime.Now();
        var q = RosFrames.Yaw(odomYaw);
        var odom = new nav_msgs.msg.Odometry
        {
            header = new std_msgs.msg.Header { stamp = stamp, frame_id = odomFrame },
            child_frame_id = baseFrame,
        };
        odom.pose.pose.position = new geometry_msgs.msg.Point(odomX, odomY, 0);
        odom.pose.pose.orientation = q;
        odom.twist.twist.linear.x = lastV;
        odom.twist.twist.angular.z = lastW;
        // Rough diagonal covariances so EKFs accept the message.
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.01;
        odom.pose.covariance[14] = odom.pose.covariance[21] = odom.pose.covariance[28] = 1e6;
        odom.pose.covariance[35] = 0.03;
        odom.twist.covariance[0] = odom.twist.covariance[7] = 0.01;
        odom.twist.covariance[14] = odom.twist.covariance[21] = odom.twist.covariance[28] = 1e6;
        odom.twist.covariance[35] = 0.03;
        odomPub.Publish(odom);

        if (tfPub != null)
        {
            var t = new geometry_msgs.msg.TransformStamped
            {
                header = new std_msgs.msg.Header { stamp = stamp, frame_id = odomFrame },
                child_frame_id = baseFrame,
            };
            t.transform.translation = new geometry_msgs.msg.Vector3(odomX, odomY, 0);
            t.transform.rotation = q;
            tfPub.Publish(new tf2_msgs.msg.TFMessage { transforms = new[] { t } });
        }

        var js = new sensor_msgs.msg.JointState
        {
            header = new std_msgs.msg.Header { stamp = stamp, frame_id = baseFrame },
            name = new string[motorCount],
            position = new double[motorCount],
            velocity = new double[motorCount],
            effort = new double[motorCount],
        };
        for (int i = 0; i < motorCount; i++)
        {
            js.name[i] = wheelJointNames[i].Replace("_link", "_joint");
            var j = wheelJoints[i];
            if (j == null || j.jointPosition.dofCount == 0) continue;
            js.position[i] = j.jointPosition[0] * SideSign(i);
            js.velocity[i] = j.jointVelocity[0] * SideSign(i);
            if (driveWheels[i] != null) js.effort[i] = driveWheels[i].OutputTorque * SideSign(i);
        }
        wheelStatePub.Publish(js);
    }
}
