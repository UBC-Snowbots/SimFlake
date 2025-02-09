using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Threading.Tasks;
using ROS2;
using System.Runtime.CompilerServices;
using sensor_msgs.msg;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using System.Numerics;

public class IMUPublisher : MonoBehaviour
{
    private ROS2Node rosNode;
    private IPublisher<sensor_msgs.msg.Imu> imuPublisher;
    private ROS2UnityCore ros2Unity = new ROS2UnityCore();
    private float publishInterval = 0.1f; // Adjust the publish interval as needed


    private UnityEngine.Vector3 past;

    private void Start()
    {
        if (ros2Unity.Ok())
        {
            rosNode = ros2Unity.CreateNode("unity_imu_publisher");
            imuPublisher = rosNode.CreatePublisher<sensor_msgs.msg.Imu>("/imu/data");
        }
        else
        {
            Debug.LogError("ROS2UnityCore is not OK");
        }

        past = new System.Numerics.Vector3(0, 0, 0);
        StartCoroutine(PublishIMUData());
    }

    private IEnumerator PublishIMUData()
    {
        while (true)
        {
            PublishIMU();
            yield return new WaitForSeconds(publishInterval);
        }
    }

    private void PublishIMU()
    {

        Debug.Log(GetComponent<Rigidbody>().angularVelocity.x);
        if (imuPublisher == null)
        {
            Debug.LogError("imuPublisher is not initialized.");
            return;
        }



        var imuMsg = new sensor_msgs.msg.Imu
        {
            Header = new std_msgs.msg.Header  
            {
                Frame_id = "Unity",

            },
            Orientation = new geometry_msgs.msg.Quaternion
            {
                X = transform.localRotation.x,
                Y = transform.localRotation.y,
                Z = transform.localRotation.z,
                W = transform.localRotation.w
            },
            Angular_velocity = new geometry_msgs.msg.Vector3
            {
                X = GetComponent<Rigidbody>().angularVelocity.x,
                Y = GetComponent<Rigidbody>().angularVelocity.y,
                Z = GetComponent<Rigidbody>().angularVelocity.z
            },
            Linear_acceleration = new geometry_msgs.msg.Vector3
            {
                X = (GetComponent<Rigidbody>().angularVelocity.x - past.x) /.1,
                Y = (GetComponent<Rigidbody>().angularVelocity.y - past.y) /.1,
                Z = (GetComponent<Rigidbody>().angularVelocity.z - past.z) /.1
            }
        };

        past = GetComponent<Rigidbody>().angularVelocity;

        imuPublisher.Publish(imuMsg);
    }
}