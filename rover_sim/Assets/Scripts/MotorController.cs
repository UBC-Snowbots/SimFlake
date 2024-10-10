using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using System;
// using GameObject;

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
    private string[] wheelJointNames = new string[motorCount] {
        "back_left_wheel_link",
        "back_right_wheel_link",
        "left_front_wheel_link",
        "left_mid_wheel_link",
        "right_front_wheel_link",
        "right_mid_wheel_link"
    };
    // public List<string> childActuatorNames;    
    // private List<ActuatorScript> actuators = new List<ActuatorScript>();
// private Dictionary<string, ActuatorScript> actuators = new Dictionary<string, ActuatorScript>();
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
                        print("MeowStart");

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
            // Use UnityEngine.Transform to avoid ambiguity
            // UnityEngine.Transform jointTransform = transform.Find(jointName);
            GameObject jointObject = GameObject.Find(jointName);

            if (jointObject != null)
            {
                wheelJoints[i] = jointObject.GetComponent<ArticulationBody>();
                wheelJoints[i].gameObject.AddComponent<JointControl>();
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
            DriveMotors(msg);
        }
    }
        // If needed, you can call ros2Unity.SpinOnce() here
        // ros2Unity.SpinOnce();
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
        double linear = msg.Linear.X;
        double angular = msg.Angular.Z;
        double right = linear + angular;
        double left = linear - angular;
   // std_msgs.msg.Float64MultiArray test_msg;
        std_msgs.msg.Float64MultiArray test_msg = new std_msgs.msg.Float64MultiArray();
        test_msg.Data = new double[6] {left, left, left, right, right, right};
        MotorCommandCallback(test_msg);
        Debug.Log($"Received cmd_vel - Linear: {linear}, Angular: {angular}");
        // // TODO: Implement robot movement based on cmd_vel
        // // For example, convert linear and angular velocities to wheel velocities
        // and set the wheel joints' target velocities accordingly
    }
}
