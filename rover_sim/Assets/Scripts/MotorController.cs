using System;
using SimFlake.Ros;
using UnityEngine;

/// <summary>
/// Drives the 6 rover wheels and publishes wheel odometry.
///
/// Subscribes:
///   /cmd_vel         geometry_msgs/Twist          linear.x [m/s], angular.z [rad/s] (skid-steer)
///   /motor_commands  std_msgs/Float64MultiArray   6 raw wheel joint targets [deg/s], order = wheelJointNames
/// Publishes:
///   /odom                nav_msgs/Odometry      dead-reckoned from wheel joint speeds (frame odom -> base_link)
///   /wheel_joint_states  sensor_msgs/JointState wheel positions [rad] and velocities [rad/s]
/// </summary>
public class MotorController : MonoBehaviour
{
    const int motorCount = 6;

    [Header("Wheels (order matters: matches /motor_commands)")]
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
    [SerializeField] private string motorTopicName = "/motor_commands";
    [SerializeField] private string cmdVelTopicName = "/cmd_vel";

    [Header("Geometry")]
    [Tooltip("Measured from the wheel colliders at start when 'autoMeasureGeometry' is on.")]
    [SerializeField] private double wheel_radius_meters = 0.3;
    [Tooltip("Distance between left and right wheel centres. Measured at start when 'autoMeasureGeometry' is on.")]
    [SerializeField] private double track_width_meters = 0.8;
    [SerializeField] private bool autoMeasureGeometry = true;
    [Tooltip("Skid-steer turning needs more wheel speed difference than the geometric track suggests. 1 = ideal.")]
    [SerializeField] private double turnSlipFactor = 2.3;

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
            wheelJoints[i].jointFriction = 1;
            wheelJoints[i].angularDamping = 1;
            var drive = wheelJoints[i].xDrive;
            if (drive.damping <= 0) { drive.damping = wheelDriveDamping; wheelJoints[i].xDrive = drive; }
        }

        if (autoMeasureGeometry) MeasureGeometry();
        if (wheelJoints[0] != null)
        {
            var d = wheelJoints[0].xDrive;
            Debug.Log($"[MotorController] wheel drive: type {wheelJoints[0].jointType}, stiffness {d.stiffness}, damping {d.damping}, forceLimit {d.forceLimit}");
        }

        Ros.Subscribe<geometry_msgs.msg.Twist>(cmdVelTopicName, CmdVelCallback);
        Ros.Subscribe<std_msgs.msg.Float64MultiArray>(motorTopicName, MotorCommandCallback);

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

    // Raw per-wheel joint targets in deg/s (same behaviour as before the Jazzy migration).
    public void MotorCommandCallback(std_msgs.msg.Float64MultiArray msg)
    {
        if (msg.data == null || msg.data.Length != motorCount)
        {
            Debug.LogWarning($"[MotorController] Expected {motorCount} motor commands, got {msg.data?.Length ?? 0}.");
            return;
        }
        for (int i = 0; i < motorCount; i++) SetJointTarget(i, (float)msg.data[i]);
    }

    public void CmdVelCallback(geometry_msgs.msg.Twist msg)
    {
        double v = msg.linear.x;   // m/s forward
        double w = msg.angular.z;  // rad/s, positive = turn left (counter-clockwise from above)
        double half = track_width_meters * turnSlipFactor / 2.0;
        double left = (v - w * half) / wheel_radius_meters;   // wheel rad/s
        double right = (v + w * half) / wheel_radius_meters;
        for (int i = 0; i < motorCount; i++)
        {
            double ws = isLeftWheel[i] ? left : right;
            SetJointTarget(i, (float)(ws * Mathf.Rad2Deg * SideSign(i)));
        }
    }

    void SetJointTarget(int i, float degPerSec)
    {
        var joint = wheelJoints[i];
        if (joint == null) return;
        var drive = joint.xDrive;
        drive.targetVelocity = degPerSec;
        joint.xDrive = drive;
    }

    void FixedUpdate()
    {
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
        };
        for (int i = 0; i < motorCount; i++)
        {
            js.name[i] = wheelJointNames[i].Replace("_link", "_joint");
            var j = wheelJoints[i];
            if (j == null || j.jointPosition.dofCount == 0) continue;
            js.position[i] = j.jointPosition[0] * SideSign(i);
            js.velocity[i] = j.jointVelocity[0] * SideSign(i);
        }
        wheelStatePub.Publish(js);
    }
}
