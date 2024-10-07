using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using System;

// Import ROS2 message types with aliases to avoid conflicts
// using std_msgs = std_msgs.msg;
// using geometry_msgs = geometry_msgs.msg;

public class MotorController : MonoBehaviour
{
    // ROS2 Node and Subscribers
    private ROS2Node rosNode;
    private ISubscription<std_msgs.msg.Float64MultiArray> motorSub;
    private ISubscription<geometry_msgs.msg.Twist> cmdVelSub;

    // Motor command array size
    private const int motorCount = 6;

    // Wheel joint names in the same order as motor commands
    private string[] wheelJointNames = new string[motorCount] {
        "back_left_wheel_link",
        "back_right_wheel_link",
        "left_front_wheel_link",
        "left_mid_wheel_link",
        "right_front_wheel_link",
        "right_mid_wheel_link"
    };

    // Wheel joints
    private ArticulationBody[] wheelJoints = new ArticulationBody[motorCount];

    // ROS2 Topic Names
    [SerializeField]
    private string motorTopicName = "/motor_commands";
    [SerializeField]
    private string cmdVelTopicName = "/cmd_vel";

    void Start()
    {
        // Initialize ROS2
        ROS2UnityCore ros2Unity = new ROS2UnityCore();
        if (ros2Unity.Ok())
        {
            rosNode = ros2Unity.CreateNode("unity_motor_controller");

            // Subscribe to motor commands topic
            motorSub = rosNode.CreateSubscription<std_msgs.msg.Float64MultiArray>(
                motorTopicName,
                msg => MotorCommandCallback(msg)
            );

            // Subscribe to cmd_vel topic
            cmdVelSub = rosNode.CreateSubscription<geometry_msgs.msg.Twist>(
                cmdVelTopicName,
                msg => CmdVelCallback(msg)
            );
        }
        else
        {
            Debug.LogError("ROS2UnityCore is not OK");
        }

        // Find and store the wheel joints
        for (int i = 0; i < motorCount; i++)
        {
            string jointName = wheelJointNames[i];
            // Use UnityEngine.Transform to avoid ambiguity
            UnityEngine.Transform jointTransform = transform.Find(jointName);
            if (jointTransform != null)
            {
                wheelJoints[i] = jointTransform.GetComponent<ArticulationBody>();
                if (wheelJoints[i] == null)
                {
                    Debug.LogError($"ArticulationBody not found on joint {jointName}");
                }
            }
            else
            {
                Debug.LogError($"Joint {jointName} not found in the scene");
            }
        }
    }

    // Update method may not be necessary if ROS2UnityCore handles spinning
    void Update()
    {
        // If needed, you can call ros2Unity.SpinOnce() here
    }

    // Callback for motor commands
    private void MotorCommandCallback(std_msgs.msg.Float64MultiArray msg)
    {
        double[] motorCommands = msg.Data;

        if (motorCommands.Length != motorCount)
        {
            Debug.LogWarning($"Expected {motorCount} motor commands, but received {motorCommands.Length}.");
            return;
        }

        for (int i = 0; i < motorCount; i++)
        {
            double command = motorCommands[i];
            var joint = wheelJoints[i];
            if (joint != null)
            {
                // Set the target velocity of the joint
                var drive = joint.xDrive;
                drive.targetVelocity = (float)command;
                joint.xDrive = drive;
            }
            else
            {
                Debug.LogError($"Wheel joint at index {i} is null");
            }
        }
    }

    // Callback for cmd_vel messages
    private void CmdVelCallback(geometry_msgs.msg.Twist msg)
    {
        // Vector3 linear = new Vector3(
        //     (float)msg.Linear.X,
        //     (float)msg.Linear.Y,
        //     (float)msg.Linear.Z
        // );

        // Vector3 angular = new Vector3(
        //     (float)msg.Angular.X,
        //     (float)msg.Angular.Y,
        //     (float)msg.Angular.Z
        // );

        // Debug.Log($"Received cmd_vel - Linear: {linear}, Angular: {angular}");
        // // TODO: Implement robot movement based on cmd_vel
        // // For example, convert linear and angular velocities to wheel velocities
        // and set the wheel joints' target velocities accordingly
    }
}
