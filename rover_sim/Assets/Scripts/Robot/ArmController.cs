using System;
using System.Collections.Generic;
using SimFlake.Ros;
using UnityEngine;

/// <summary>
/// Simulated arm (6 axes + 2-finger end effector). Listens to the same topic as the real arm driver,
/// so the RoverFlake2 HMI / joystick / MoveIt bridge drive it directly. sim_helper is not needed.
///
/// Subscribes:
///   /arm/command          rover_msgs/ArmCommand       same contract as moteus_driver_node (arm_commands.h):
///                           'V' velocities[7] deg/s (NaN = leave that axis alone, index 6 = end effector)
///                           'P' positions[7] output revs, velocities[7] deg/s travel speed (NaN = axis max)
///                           'S' stop (positions[] empty = all, else non-NaN entries)
///                           'Z' zero the selected axes where they are, 'H' "home" (moves to the sim's zero)
///                           'G' legacy gripper command (end_effector)
///   /arm/sim_command      std_msgs/Float64MultiArray  legacy sim_helper output, ignored while /arm/command is active
///   /arm/ee_command/sim   std_msgs/Float64            legacy gripper command, same rule
/// Publishes:
///   /arm/sim_feedback     std_msgs/Float64MultiArray  6 joint positions [rad]
///   /joint_states         sensor_msgs/JointState      joint_1..joint_6 positions [rad] / velocities [rad/s]
/// </summary>
public class ArmController : MonoBehaviour
{
    private const int NUM_AXES = 6;   // arm joints
    private const int EE = 6;         // end effector index in ArmCommand arrays
    private const int NUM_CMD = 7;    // axes + end effector

    [SerializeField] private int[] motor_dirs = { 1, -1, 1, 1, -1, 1 };

    [SerializeField]
    private string[] armLinkNames = { "link_1", "link_2", "link_3", "link_4", "link_5", "link_6" };

    [SerializeField] float[] torque = { 200, 300, 300, 400, 400, 400 };
    [SerializeField] float ee_torque = 0.5F;

    [Tooltip("Assigned in the inspector; found by armLinkNames if left empty.")]
    [SerializeField] private ArticulationBody[] armJoints = new ArticulationBody[NUM_AXES];
    [SerializeField] private ArticulationBody endEffectorLeftJoint;
    [SerializeField] private ArticulationBody endEffectorRightJoint;

    [Header("Topics")]
    [SerializeField] private string armCommandTopicName = "/arm/command";
    [SerializeField] private string motorTopicName = "/arm/sim_command";
    [SerializeField] private string jointStatesTopicName = "/joint_states";
    [SerializeField] private string armPositionFeedbackTopicName = "/arm/sim_feedback";
    [SerializeField] private string endEffectorCommandTopicName = "/arm/ee_command/sim";
    [SerializeField] private float feedbackRateHz = 50f;

    // Joint names on the ROS side (MoveIt), not the Unity link names.
    [SerializeField] private string[] moveit_joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

    [Header("Command handling")]
    [Tooltip("Finger travel (m) per degree of end-effector motor rotation. 0.0005 = full stroke in ~1 s at the real EE max speed (0.5 rev/s).")]
    [SerializeField] private float eeMetersPerDegree = 0.0005f;
    [Tooltip("Speed cap (deg/s) for 'P' moves that don't give one. Real arm: 0.1 rev/s for axes, 0.5 rev/s for the EE.")]
    [SerializeField] private float[] maxSpeedDegPerSec = { 36, 36, 36, 36, 36, 36, 180 };
    [Tooltip("Position-move gain (1/s): commanded speed = gain * error, capped by the move speed.")]
    [SerializeField] private float positionGain = 4f;
    [Tooltip("Velocity drives with damping 0 produce no torque; this damping is used for those.")]
    [SerializeField] private float jointDriveDamping = 1000f;

    enum Mode { Velocity, Position }
    readonly Mode[] mode = new Mode[NUM_CMD];
    readonly double[] velCmd = new double[NUM_CMD];    // deg/s, axis frame
    readonly double[] posTarget = new double[NUM_CMD]; // deg, axis frame
    readonly double[] posSpeed = new double[NUM_CMD];  // deg/s
    readonly double[] zeroDeg = new double[NUM_CMD];   // reading offset set by 'Z'
    float lastArmCommandTime = -100f;
    bool warnedMotorSpace;
    readonly HashSet<byte> warnedTypes = new HashSet<byte>();

    RosPublisher<std_msgs.msg.Float64MultiArray> positionPub;
    RosPublisher<sensor_msgs.msg.JointState> jointStatePub;
    float feedbackTimer;

    void Start()
    {
        for (int i = 0; i < NUM_AXES; i++)
        {
            if (armJoints[i] == null)
            {
                var go = GameObject.Find(armLinkNames[i]);
                if (go != null) armJoints[i] = go.GetComponent<ArticulationBody>();
            }
            if (armJoints[i] == null)
            {
                Debug.LogError($"[ArmController] Joint '{armLinkNames[i]}' not found");
                continue;
            }
            SetupDrive(armJoints[i], torque[i]);
        }
        SetupDrive(endEffectorLeftJoint, ee_torque);
        SetupDrive(endEffectorRightJoint, ee_torque);

        Ros.Subscribe<rover_msgs.msg.ArmCommand>(armCommandTopicName, ArmCommandCallback);
        Ros.Subscribe<std_msgs.msg.Float64MultiArray>(motorTopicName, MotorCommandCallback);
        Ros.Subscribe<std_msgs.msg.Float64>(endEffectorCommandTopicName, endEffectorCallback);
        positionPub = Ros.Advertise<std_msgs.msg.Float64MultiArray>(armPositionFeedbackTopicName, 2);
        jointStatePub = Ros.Advertise<sensor_msgs.msg.JointState>(jointStatesTopicName, 2);
    }

    void SetupDrive(ArticulationBody j, float forceLimit)
    {
        if (j == null) return;
        j.jointFriction = 1;
        j.angularDamping = 1;
        var d = j.xDrive;
        d.forceLimit = forceLimit;
        if (d.damping <= 0) d.damping = jointDriveDamping;
        j.xDrive = d;
    }

    // ---- joint access in the "axis frame" (degrees, same sign/zero as the commands) ----

    bool Has(int i) => i == EE ? endEffectorLeftJoint != null && endEffectorRightJoint != null : armJoints[i] != null;

    double Reading(int i)
    {
        if (i == EE)
        {
            var r = endEffectorRightJoint;
            return (r.jointPosition.dofCount > 0 ? r.jointPosition[0] : 0) / eeMetersPerDegree - zeroDeg[i];
        }
        var j = armJoints[i];
        double deg = j.jointPosition.dofCount > 0 ? j.jointPosition[0] * Mathf.Rad2Deg : 0;
        return deg * motor_dirs[i] - zeroDeg[i];
    }

    void ApplyVelocity(int i, double degPerSec)
    {
        if (i == EE)
        {
            float v = (float)(degPerSec * eeMetersPerDegree);
            var l = endEffectorLeftJoint.xDrive; l.targetVelocity = -v; endEffectorLeftJoint.xDrive = l;
            var r = endEffectorRightJoint.xDrive; r.targetVelocity = v; endEffectorRightJoint.xDrive = r;
            return;
        }
        var d = armJoints[i].xDrive;
        d.targetVelocity = (float)degPerSec * motor_dirs[i];
        armJoints[i].xDrive = d;
    }

    void FixedUpdate()
    {
        for (int i = 0; i < NUM_CMD; i++)
        {
            if (!Has(i)) continue;
            double v = velCmd[i];
            if (mode[i] == Mode.Position)
                v = Math.Max(-posSpeed[i], Math.Min(posSpeed[i], (posTarget[i] - Reading(i)) * positionGain));
            ApplyVelocity(i, v);
        }
    }

    void SetVelocity(int i, double v) { mode[i] = Mode.Velocity; velCmd[i] = v; }

    void MoveTo(int i, double deg, double speed)
    {
        mode[i] = Mode.Position;
        posTarget[i] = deg;
        posSpeed[i] = double.IsFinite(speed) && Math.Abs(speed) > 1e-6 ? Math.Abs(speed) : maxSpeedDegPerSec[i];
    }

    static bool Selected(double[] a, int i) => a != null && i < a.Length && !double.IsNaN(a[i]);

    // ---- /arm/command (rover_msgs/ArmCommand) ----

    public void ArmCommandCallback(rover_msgs.msg.ArmCommand m)
    {
        lastArmCommandTime = Time.time;
        char type = (char)m.cmd_type;
        if (m.cmd_value == 1 && (type == 'P' || type == 'V' || type == 'S') && !warnedMotorSpace)
        {
            warnedMotorSpace = true;
            Debug.LogWarning("[ArmController] Motor-space command (cmd_value=1): the sim has no differential wrist, treating it as axis space.");
        }

        switch (type)
        {
            case 'V':
                for (int i = 0; i < NUM_CMD; i++)
                    if (Selected(m.velocities, i)) SetVelocity(i, m.velocities[i]);
                break;

            case 'P':
                for (int i = 0; i < NUM_CMD; i++)
                    if (Selected(m.positions, i))
                        MoveTo(i, m.positions[i] * 360.0, Selected(m.velocities, i) ? m.velocities[i] : double.NaN);
                break;

            case 'S':
                bool all = m.positions == null || m.positions.Length == 0;
                for (int i = 0; i < NUM_CMD; i++)
                    if (all || Selected(m.positions, i)) SetVelocity(i, 0);
                break;

            case 'Z':
                for (int i = 0; i < NUM_CMD; i++)
                {
                    if (!Selected(m.positions, i) || !Has(i)) continue;
                    double r = Reading(i);
                    zeroDeg[i] += r;
                    if (mode[i] == Mode.Position) posTarget[i] -= r; // keep the same physical target
                }
                break;

            case 'H':
                // No limit switches in the sim: homing drives the selected axes back to the URDF zero.
                var axes = new List<int>();
                if (m.cmd_value == 0xAA) for (int i = 0; i < NUM_AXES; i++) axes.Add(i);
                else if (m.cmd_value == 0xAE) for (int i = 0; i < NUM_CMD; i++) axes.Add(i);
                else if (m.positions != null) foreach (var p in m.positions) if (!double.IsNaN(p)) axes.Add((int)p);
                foreach (int i in axes)
                {
                    if (i < 0 || i >= NUM_CMD) continue;
                    zeroDeg[i] = 0;
                    MoveTo(i, 0, double.NaN);
                }
                break;

            case 'G':
                SetGripper(m.end_effector);
                break;

            default:
                if (warnedTypes.Add(m.cmd_type))
                    Debug.Log($"[ArmController] Ignoring arm command type '{type}' ({m.cmd_type}); not simulated.");
                break;
        }
    }

    // ---- legacy topics from `ros2 run arm_control sim_helper` ----

    bool ArmCommandActive => Time.time - lastArmCommandTime < 2f;

    public void MotorCommandCallback(std_msgs.msg.Float64MultiArray msg)
    {
        if (ArmCommandActive) return; // we already got the same command on /arm/command
        double[] cmd = msg.data;
        if (cmd == null || cmd.Length < NUM_AXES)
        {
            Debug.LogWarning($"[ArmController] Expected {NUM_AXES} (or {NUM_AXES + 1} with gripper) values, got {cmd?.Length ?? 0}.");
            return;
        }
        for (int i = 0; i < NUM_AXES; i++)
            if (!double.IsNaN(cmd[i])) SetVelocity(i, cmd[i]);
        if (cmd.Length > NUM_AXES) SetGripper(cmd[NUM_AXES]);
    }

    public void endEffectorCallback(std_msgs.msg.Float64 msg)
    {
        if (!ArmCommandActive) SetGripper(msg.data);
    }

    /// <summary>Legacy gripper command: finger velocity in m/s (positive closes).</summary>
    void SetGripper(double cmd)
    {
        if (double.IsNaN(cmd)) return;
        SetVelocity(EE, cmd / eeMetersPerDegree);
    }

    // ---- feedback ----

    void Update()
    {
        feedbackTimer += Time.deltaTime;
        if (feedbackTimer < 1f / feedbackRateHz) return;
        feedbackTimer = 0;

        var positions = new double[NUM_AXES];
        var velocities = new double[NUM_AXES];
        for (int i = 0; i < NUM_AXES; i++)
        {
            var j = armJoints[i];
            if (j == null || j.jointPosition.dofCount == 0) continue;
            positions[i] = j.jointPosition[0] * motor_dirs[i];
            velocities[i] = j.jointVelocity[0] * motor_dirs[i];
        }

        positionPub.Publish(new std_msgs.msg.Float64MultiArray { data = positions });
        jointStatePub.Publish(new sensor_msgs.msg.JointState
        {
            header = RosTime.Header(""),
            name = moveit_joint_names,
            position = positions,
            velocity = velocities,
        });
    }
}
