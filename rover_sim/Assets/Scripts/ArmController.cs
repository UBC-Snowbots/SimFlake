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


public class ArmController : MonoBehaviour
{
    // ROS2 Node and Subscribers
    // private ROS2UnityComponent ros2Unity
    private ROS2Node rosNode;
    private ISubscription<std_msgs.msg.Float64MultiArray> armSub;
    private ISubscription<geometry_msgs.msg.Twist> cmdVelSub;

        //For arm feedback
    private IPublisher<std_msgs.msg.Float64MultiArray> position_pubber;
    private IPublisher<sensor_msgs.msg.JointState> joint_state_pubber;



    // Motor command array size
    private const int NUM_AXES = 6;
        ROS2UnityCore ros2Unity = new ROS2UnityCore();

    // Wheel joint names in the same order as motor commands
    [SerializeField]
    private int[] motor_dirs = new int[NUM_AXES]{
        1, //back_left_wheel dir
        1,  
        1,
        1,
        1,
        1

    };

    [SerializeField]
    private string[] armLinkNames = new string[NUM_AXES] {
        "link_1",
        "link_2",
        "link_3",
        "link_4",
        "link_5",
        "link_6"
    };
        //  "back_left_wheel_link", //Original Layout
        // "back_right_wheel_link",
        // "left_front_wheel_link",
        // "left_mid_wheel_link",
        // "right_front_wheel_link",
        // "right_mid_wheel_link"

    [SerializeField]
    float[] torque = new float[]{200, 300, 300, 400, 400, 400};

    // Wheel joints
    [SerializeField]
    private ArticulationBody[] armJoints = new ArticulationBody[NUM_AXES];

    // ROS2 Topic Names
    [SerializeField]
    private string motorTopicName = "/arm/sim_command";
    [SerializeField]
    private string jointStatesTopicName = "/joint_states";

    [SerializeField]
    private string armPositionFeedbackTopicName = "/arm/sim_feedback";


    private double[] current_positions = new double[]{0, 0, 0, 0, 0, 0};
    private double[] current_velocities = new double[]{0, 0, 0, 0, 0, 0};


    
    private string[] moveit_joint_names = new string[]{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"}; //? newest (Sep 2024) arm urdf - For ros2 side, not unity joint names


    private double time_elapsed;
private Queue<std_msgs.msg.Float64MultiArray> motorCommandQueue = new Queue<std_msgs.msg.Float64MultiArray>();
private readonly object queueLock = new object(); // Lock for thread safety

    void Start()
    {
                        print("Arm Start");

        // Initialize ROS2
        if (ros2Unity.Ok())
        {
            rosNode = ros2Unity.CreateNode("unity_arm_controller");
            // Subscribe to motor commands topic
            armSub = rosNode.CreateSubscription<std_msgs.msg.Float64MultiArray>(
                motorTopicName,
                msg => MotorCommandCallback(msg)
                );

            //! DONT Subscribe to cmd_vel topic
            // cmdVelSub = rosNode.CreateSubscription<geometry_msgs.msg.Twist>(
            //     cmdVelTopicName,
            //     msg => CmdVelCallback(msg)
            // );

            position_pubber = rosNode.CreatePublisher<std_msgs.msg.Float64MultiArray>(armPositionFeedbackTopicName);

            joint_state_pubber = rosNode.CreatePublisher<sensor_msgs.msg.JointState>(jointStatesTopicName);
        }
        else
        {
            Debug.LogError("ROS2UnityCore is not OK");
        }

        // Find and store the wheel joints
        for (int i = 0; i < NUM_AXES; i++)
        {
            string jointName = armLinkNames[i];
            
            GameObject jointObject = GameObject.Find(jointName);

            if (jointObject != null)
            {
                // armJoints[i] = jointObject.GetComponent<ArticulationBody>(); //! Articulation body is set manually in unity, after script has been placed on arm
                armJoints[i].gameObject.AddComponent<JointControl>();
                armJoints[i].jointFriction = 1;
                armJoints[i].angularDamping = 1;
                ArticulationDrive drive = armJoints[i].xDrive;
                drive.forceLimit = torque[i];
                armJoints[i].xDrive = drive;
                // JointControl currentDrive = wheelJoints.xDrive;
                if (armJoints[i] == null)
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

    for(int i = 0; i < NUM_AXES; i++){
    ArticulationReducedSpace joint_position = armJoints[i].jointPosition;
    ArticulationReducedSpace joint_velocity = armJoints[i].jointVelocity;;
    current_positions[i] = joint_position[0];
    current_velocities[i] = joint_velocity[0];

    }
    // time_elapsed += Time.deltaTime; //* can add a timer, but would be trying to update positions at FPS rate
    std_msgs.msg.Float64MultiArray feedback_msg = new std_msgs.msg.Float64MultiArray();
    feedback_msg.Data = new double[NUM_AXES];
    feedback_msg.Data = current_positions;
    position_pubber.Publish(feedback_msg);
// Define the Unix epoch
DateTime unixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

// DateTime unixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
DateTime currentTime = DateTime.UtcNow;
double timeSinceEpoch = (currentTime - unixEpoch).TotalSeconds;
int sec = (int)currentTime.Second;
uint nanosec = (uint)(((currentTime.Second - (int)currentTime.Second) * 1e9));
    sensor_msgs.msg.JointState joint_state_msg = new sensor_msgs.msg.JointState();
    joint_state_msg.Header.Stamp.Sec = sec;
    joint_state_msg.Header.Stamp.Nanosec = nanosec;

    joint_state_msg.Name = new string[NUM_AXES];
    joint_state_msg.Position = new double[NUM_AXES];
    joint_state_msg.Velocity = new double[NUM_AXES];
    joint_state_msg.Name = moveit_joint_names;
    joint_state_msg.Position = current_positions;
    joint_state_msg.Velocity = current_velocities;
    joint_state_pubber.Publish(joint_state_msg);

    // if(time_elapsed > )
    //Test code
        // std_msgs.msg.Float64MultiArray test_msg;
        // std_msgs.msg.Float64MultiArray test_msg = new std_msgs.msg.Float64MultiArray();
        // test_msg.Data = new double[6] {1, 1, 1, 1, 1, 1};
        // MotorCommandCallback(test_msg);
        // rosNode.SpinSome();
    }

     private void DriveMotors(std_msgs.msg.Float64MultiArray msg){
        double[] armCommands = msg.Data;
       if (armCommands.Length != NUM_AXES)
        {
            Debug.LogWarning($"Expected {NUM_AXES} motor commands, but received {armCommands.Length}.");
            return;
        }

        for (int i = 0; i < NUM_AXES; i++)
        {
            double command = armCommands[i];
            var joint = armJoints[i];
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
                Debug.LogError($"Arm joint at index {i} is null");
            }
        }
     }

    // Callback for motor commands
    public void MotorCommandCallback(std_msgs.msg.Float64MultiArray msg)
    {
          //Using a lock as ROS subscribers are on a seperate  thread
lock (queueLock){
  print("MeowCallbackLocked");
          motorCommandQueue.Enqueue(msg);
}

 
    }

}


















//* Graveyard/Archive - Code that wasn't used



    // Callback for cmd_vel messages
//     public void CmdVelCallback(geometry_msgs.msg.Twist msg)
//     {
 
//         double linear = msg.Linear.X;       // Forwards/Backwards velocity. Turns into 
//         double angular = msg.Angular.Z;


//         //RIGHT is the inverse motor control of regular b/c motors dk what side they're on
//         // counterclockwise = positive velocity
//         // double right = linear*-1 + angular;
//         // double left = linear - angular;
//         double right = angular;
//         double left = angular;
//    // std_msgs.msg.Float64MultiArray test_msg;
//         std_msgs.msg.Float64MultiArray test_msg = new std_msgs.msg.Float64MultiArray();

//         //this is correct distribution of motor commands following wheel_joint_names
//         //TODO: could change this to be more dynamic in the future
//         test_msg.Data = new double[6] {left, right, left, left, right, right};
        
//         MotorCommandCallback(test_msg);
//         Debug.Log($"Received cmd_vel - Linear: {linear}, Angular: {angular}");
//         // // TODO: Implement robot movement based on cmd_vel
//         // // For example, convert linear and angular velocities to wheel velocities
//         // and set the wheel joints' target velocities accordingly
//     }