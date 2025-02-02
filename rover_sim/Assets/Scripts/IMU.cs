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

public class IMUPublisher : MonoBehaviour
{
    private ROS2Node rosNode;
    private IPublisher<sensor_msgs.msg.Imu> imuPublisher;
    private ROS2UnityCore ros2Unity = new ROS2UnityCore();
    private float publishInterval = 0.1f; // Adjust the publish interval as needed

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
                X = transform.rotation.x,
                Y = transform.rotation.y,
                Z = transform.rotation.z,
                W = transform.rotation.w
            },
            Angular_velocity = new geometry_msgs.msg.Vector3
            {
                X = Input.gyro.rotationRateUnbiased.x,
                Y = Input.gyro.rotationRateUnbiased.y,
                Z = Input.gyro.rotationRateUnbiased.z
            },
            Linear_acceleration = new geometry_msgs.msg.Vector3
            {
                X = Input.acceleration.x,
                Y = Input.acceleration.y,
                Z = Input.acceleration.z
            }
        };

        imuPublisher.Publish(imuMsg);
    }
}