using System;
using System.Collections.Generic;
using System.Linq;
using SimFlake.Ros;
using UnityEngine;

/// <summary>
/// Simulated arm (6 axes + 2-finger end effector). Listens to the same topic as the real arm driver,
/// so the RoverFlake2 HMI / joystick / MoveIt bridge drive it directly. sim_helper is not needed.
///
/// Every axis and the gripper is a stepper motor through a gearbox (<see cref="StepperDrive"/>, specs in
/// Assets/Robots/Arm/Motors): open loop, it holds with its holding torque, slows down at high step rates, and when
/// overloaded it stalls and loses steps instead of pushing harder. The gripper is one motor driving both fingers through
/// flexible (printed) fingers: closed on something, it squeezes until the motor stalls, and keeps squeezing after the
/// command stops. Links can break off under load (<see cref="ArmBreakage"/>).
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
    public const int NUM_AXES = 6;   // arm joints
    public const int EE = 6;         // end effector index in ArmCommand arrays
    public const int NUM_CMD = 7;    // axes + end effector

    public static readonly string[] AxisNames = { "Base yaw", "Shoulder", "Elbow", "Forearm roll", "Wrist pitch", "Wrist roll", "Gripper" };

    [SerializeField] private int[] motor_dirs = { 1, -1, 1, 1, -1, 1 };

    [SerializeField]
    private string[] armLinkNames = { "link_1", "link_2", "link_3", "link_4", "link_5", "link_6" };

    [Tooltip("Assigned in the inspector; found by armLinkNames (inside this arm) if left empty.")]
    [SerializeField] private ArticulationBody[] armJoints = new ArticulationBody[NUM_AXES];
    [SerializeField] private ArticulationBody endEffectorLeftJoint;
    [SerializeField] private ArticulationBody endEffectorRightJoint;

    [Header("Motors (index 6 = gripper)")]
    [Tooltip("Stepper + gearbox per axis. Several axes may share one spec.")]
    public StepperMotorSpec[] motors = new StepperMotorSpec[NUM_CMD];

    [Header("Gripper")]
    [Tooltip("Finger travel (m) per degree of end-effector motor rotation. 0.0005 = full stroke in ~1 s at the real EE max speed (0.5 rev/s).")]
    public float eeMetersPerDegree = 0.0005f;
    [Tooltip("Flex of each finger + its drive train (N/mm at the finger). Printed fingers bend: lower = softer squeeze, more travel after contact.")]
    public float fingerStiffness = 15f;

    [Header("Physics")]
    [Tooltip("The URDF import had gravity off on the arm links; the motors now carry the arm's weight.")]
    public bool armGravity = true;
    [Tooltip("Solver iterations when the arm is its own articulation (not on a rover).")]
    public int solverIterations = 30;
    public int solverVelocityIterations = 20;

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
    [Tooltip("Speed cap (deg/s) for 'P' moves that don't give one. Real arm: 0.1 rev/s for axes, 0.5 rev/s for the EE.")]
    public float[] maxSpeedDegPerSec = { 36, 36, 36, 36, 36, 36, 180 };
    [Tooltip("Position-move gain (1/s): commanded speed = gain * error, capped by the move speed.")]
    public float positionGain = 4f;

    enum Mode { Velocity, Position }
    readonly Mode[] mode = new Mode[NUM_CMD];
    readonly double[] velCmd = new double[NUM_CMD];    // deg/s, axis frame
    readonly double[] posTarget = new double[NUM_CMD]; // deg, axis frame
    readonly double[] posSpeed = new double[NUM_CMD];  // deg/s
    readonly double[] zeroDeg = new double[NUM_CMD];   // reading offset set by 'Z'
    float lastArmCommandTime = -100f;
    bool warnedMotorSpace;
    readonly HashSet<byte> warnedTypes = new HashSet<byte>();

    readonly StepperDrive[] drives = new StepperDrive[NUM_CMD];
    ArmBreakage breakage;

    RosPublisher<std_msgs.msg.Float64MultiArray> positionPub;
    RosPublisher<sensor_msgs.msg.JointState> jointStatePub;
    float feedbackTimer;

    readonly List<RosSubscription> subs = new List<RosSubscription>();

    /// <summary>The stepper of axis <paramref name="i"/> (6 = gripper), or null.</summary>
    public StepperDrive Drive(int i) => i >= 0 && i < NUM_CMD ? drives[i] : null;
    public ArticulationBody Joint(int i) => i == EE ? endEffectorRightJoint : i >= 0 && i < NUM_AXES ? armJoints[i] : null;
    public ArticulationBody[] Fingers => new[] { endEffectorLeftJoint, endEffectorRightJoint };
    /// <summary>Finger travel per radian of the gripper motor's output (m/rad).</summary>
    public float FingerMetersPerRad => eeMetersPerDegree * Mathf.Rad2Deg;
    /// <summary>Squeeze per finger when both touch and the gripper motor is at its limit (N).</summary>
    public float MaxGripForce => motors[EE] != null ? motors[EE].OutputHoldingTorque / (2 * FingerMetersPerRad) : 0;

    void OnDestroy() { foreach (var s in subs) s.Unsubscribe(); subs.Clear(); }

    void Awake()
    {
        for (int i = 0; i < NUM_AXES; i++)
            if (armJoints[i] == null) armJoints[i] = FindLink(armLinkNames[i]);
        if (endEffectorLeftJoint == null) endEffectorLeftJoint = FindLink("finger_left");
        if (endEffectorRightJoint == null) endEffectorRightJoint = FindLink("finger_right");
        UseArmLayer();
        foreach (var b in GetComponentsInChildren<ArticulationBody>(true))
        {
            b.useGravity = armGravity;
            b.angularDamping = 0.05f;
            b.linearDamping = 0.05f;
            b.jointFriction = 0.05f;
        }
    }

    ArticulationBody FindLink(string n)
    {
        foreach (var b in GetComponentsInChildren<ArticulationBody>(true)) if (b.name == n) return b;
        return null;
    }

    void Start()
    {
        breakage = GetComponent<ArmBreakage>();
        var root = GetComponentsInChildren<ArticulationBody>().FirstOrDefault(b => b.isRoot);
        if (root != null) { root.solverIterations = solverIterations; root.solverVelocityIterations = solverVelocityIterations; }
        BuildDrives();

        subs.Add(Ros.Subscribe<rover_msgs.msg.ArmCommand>(armCommandTopicName, ArmCommandCallback));
        subs.Add(Ros.Subscribe<std_msgs.msg.Float64MultiArray>(motorTopicName, MotorCommandCallback));
        subs.Add(Ros.Subscribe<std_msgs.msg.Float64>(endEffectorCommandTopicName, endEffectorCallback));
        positionPub = Ros.Advertise<std_msgs.msg.Float64MultiArray>(armPositionFeedbackTopicName, 2);
        jointStatePub = Ros.Advertise<sensor_msgs.msg.JointState>(jointStatesTopicName, 2);
    }

    void BuildDrives()
    {
        for (int i = 0; i < NUM_CMD; i++)
        {
            if (motors[i] == null)
            {
                Debug.LogWarning($"[ArmController] No motor spec for {AxisNames[i]}; using a default stepper (run SimFlake > Arm > Set Up Arm).");
                motors[i] = ScriptableObject.CreateInstance<StepperMotorSpec>();
                motors[i].name = "default stepper";
            }
            if (i < NUM_AXES)
            {
                if (armJoints[i] == null) { Debug.LogError($"[ArmController] Joint '{armLinkNames[i]}' not found"); continue; }
                drives[i] = new StepperDrive(AxisNames[i], motors[i]);
                drives[i].Add(armJoints[i], motor_dirs[i], 1f, motors[i].outputStiffness * Mathf.Rad2Deg);
            }
            else if (endEffectorLeftJoint != null && endEffectorRightJoint != null)
            {
                drives[i] = new StepperDrive(AxisNames[i], motors[i]);
                drives[i].Add(endEffectorLeftJoint, -1, FingerMetersPerRad, fingerStiffness * 1000f);
                drives[i].Add(endEffectorRightJoint, 1, FingerMetersPerRad, fingerStiffness * 1000f);
            }
        }
    }

    /// <summary>Physics layer of every arm part ("RoverArm", 8). Arm parts never collide with each other (convex hulls
    /// are fatter than the parts and overlap at the joints and the fingers): one layer-matrix entry instead of pairs of
    /// colliders (link_6 alone has 466 hulls; ignoring pairs took ~10 s). They do collide with the rover they sit on
    /// (except link_0 with the chassis: parent and child in one articulation never collide) and with everything else.</summary>
    public static int ArmLayer
    {
        get
        {
            int l = LayerMask.NameToLayer("RoverArm");
            return l >= 0 ? l : 8;
        }
    }

    void UseArmLayer()
    {
        int layer = ArmLayer;
        if (!Physics.GetIgnoreLayerCollision(layer, layer)) Physics.IgnoreLayerCollision(layer, layer, true);
        // Set in the prefab already (ArmSetup); changing a collider's layer at runtime re-filters its whole body (466 hulls).
        foreach (var t in GetComponentsInChildren<Transform>(true)) if (t.gameObject.layer != layer) t.gameObject.layer = layer;
    }

    /// <summary>Re-apply the spec values that are copied at start (flex stiffness), e.g. after a menu change.</summary>
    public void RefreshDrives()
    {
        for (int i = 0; i < NUM_CMD; i++)
        {
            var d = drives[i];
            if (d == null) continue;
            d.spec = motors[i];
            foreach (var o in d.outputs)
            {
                if (i == EE) { o.ratio = FingerMetersPerRad; o.stiffness = fingerStiffness * 1000f; }
                else o.stiffness = motors[i].outputStiffness * Mathf.Rad2Deg;
            }
            d.RefreshSubtree();
        }
    }

    // ---- joint access in the "axis frame" (degrees, same sign/zero as the commands) ----

    bool Has(int i) => drives[i] != null;

    /// <summary>Axis position (deg, command frame: same sign and zero as /arm/command).</summary>
    public double Reading(int i) => drives[i] != null ? drives[i].Position * Mathf.Rad2Deg - zeroDeg[i] : 0;

    /// <summary>False once the axis' motor or anything between it and the rover broke off.</summary>
    public bool Powered(int i)
    {
        if (breakage == null || drives[i] == null) return drives[i] != null;
        foreach (var o in drives[i].outputs) if (!breakage.Intact(o.body)) return false;
        return true;
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        for (int i = 0; i < NUM_CMD; i++)
        {
            if (!Has(i)) continue;
            double v = velCmd[i];
            if (mode[i] == Mode.Position)
                v = Math.Max(-posSpeed[i], Math.Min(posSpeed[i], (posTarget[i] - Reading(i)) * positionGain));
            drives[i].powered = Powered(i);
            drives[i].Step(v * Mathf.Deg2Rad, dt);
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

    /// <summary>Copy the tunable settings to another arm (after a repair the fresh arm keeps menu changes).</summary>
    public void CopySettingsTo(ArmController other)
    {
        other.motors = (StepperMotorSpec[])motors.Clone();
        other.eeMetersPerDegree = eeMetersPerDegree;
        other.fingerStiffness = fingerStiffness;
        other.armGravity = armGravity;
        other.positionGain = positionGain;
        other.maxSpeedDegPerSec = (float[])maxSpeedDegPerSec.Clone();
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

        positionPub?.Publish(new std_msgs.msg.Float64MultiArray { data = positions });
        jointStatePub?.Publish(new sensor_msgs.msg.JointState
        {
            header = RosTime.Header(""),
            name = moveit_joint_names,
            position = positions,
            velocity = velocities,
        });
    }
}
