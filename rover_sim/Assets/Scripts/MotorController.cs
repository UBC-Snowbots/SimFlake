using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using System;
// using GameObject;
//TODO Figure out
    //? Unity uses degrees or rads?
    //? 

// Import ROS2 message types with aliases to avoid conflicts
// using std_msgs = std_msgs.msg;
// using geometry_msgs = geometry_msgs.msg;


public class MotorController : MonoBehaviour
{
    // ROS2 Node and Subscribers
    // private ROS2UnityComponent ros2Unity
    private ROS2Node rosNode;
    private ISubscription<std_msgs.msg.Float64MultiArray> motorSub;
    private ISubscription<geometry_msgs.msg.Twist> cmdVelSub;

    // Motor command array size
    private const int motorCount = 6;
        ROS2UnityCore ros2Unity = new ROS2UnityCore();

    // Wheel joint names in the same order as motor commands
    [SerializeField]
    private int[] motor_dirs = new int[motorCount]{
        1, //back_left_wheel dir
        1,  
        1,
        1,
        1,
        1

    };

    [SerializeField]
    private string[] wheelJointNames = new string[motorCount] {
        "back_left_wheel_link",
        "back_right_wheel_link",
        "left_front_wheel_link",
        "left_mid_wheel_link",
        "right_front_wheel_link",
        "right_mid_wheel_link"
    };
        //  "back_left_wheel_link", //Original Layout
        // "back_right_wheel_link",
        // "left_front_wheel_link",
        // "left_mid_wheel_link",
        // "right_front_wheel_link",
        // "right_mid_wheel_link"

    // Wheel joints
    [SerializeField]
    private ArticulationBody[] wheelJoints = new ArticulationBody[motorCount];

    // ROS2 Topic Names
    [SerializeField]
    private string motorTopicName = "/motor_commands";
    [SerializeField]
    private string cmdVelTopicName = "/cmd_vel";


private Queue<std_msgs.msg.Float64MultiArray> motorCommandQueue = new Queue<std_msgs.msg.Float64MultiArray>();
private readonly object queueLock = new object(); // Lock for thread safety

    void Start()
    {

        // Initialize ROS2
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
            
            GameObject jointObject = GameObject.Find(jointName);

            if (jointObject != null)
            {
                wheelJoints[i] = jointObject.GetComponent<ArticulationBody>();
                // wheelJoints[i].gameObject.AddComponent<JointControl>();
                wheelJoints[i].jointFriction = 1;
                wheelJoints[i].angularDamping = 1;
                // JointControl currentDrive = wheelJoints.xDrive;
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
        lock (queueLock)
    {
        while (motorCommandQueue.Count > 0)
        {
            var msg = motorCommandQueue.Dequeue();
            DriveMotors(msg); //Cannot be called directly from callback, so needs callback to just trigger a queue
        }
    }
    //Test code
        // std_msgs.msg.Float64MultiArray test_msg;
        // std_msgs.msg.Float64MultiArray test_msg = new std_msgs.msg.Float64MultiArray();
        // test_msg.Data = new double[6] {1, 1, 1, 1, 1, 1};
        // MotorCommandCallback(test_msg);
        // rosNode.SpinSome();
    }

     private void DriveMotors(std_msgs.msg.Float64MultiArray msg){
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
                print("Meow");
            }
            else
            {
                Debug.LogError($"Wheel joint at index {i} is null");
            }
        }
     }

    // Callback for motor commands
    public void MotorCommandCallback(std_msgs.msg.Float64MultiArray msg)
    {
          
lock (queueLock){
  print("MeowCallbackLocked");
          motorCommandQueue.Enqueue(msg);
}

 
    }

    // Callback for cmd_vel messages
    public void CmdVelCallback(geometry_msgs.msg.Twist msg)
    {
 
        double linear = msg.Linear.X;       // Forwards/Backwards velocity. Turns into 
        double angular = msg.Angular.Z;


        //RIGHT is the inverse motor control of regular b/c motors dk what side they're on
        // counterclockwise = positive velocity
        double right = linear*-1 + angular;
        double left = linear - angular;
   // std_msgs.msg.Float64MultiArray test_msg;
        std_msgs.msg.Float64MultiArray test_msg = new std_msgs.msg.Float64MultiArray();

        //this is correct distribution of motor commands following wheel_joint_names
        //TODO: could change this to be more dynamic in the future
        test_msg.Data = new double[6] {left, right, left, left, right, right};
        MotorCommandCallback(test_msg);
        Debug.Log($"Received cmd_vel - Linear: {linear}, Angular: {angular}");
        // // TODO: Implement robot movement based on cmd_vel
        // // For example, convert linear and angular velocities to wheel velocities
        // and set the wheel joints' target velocities accordingly
    }
}
