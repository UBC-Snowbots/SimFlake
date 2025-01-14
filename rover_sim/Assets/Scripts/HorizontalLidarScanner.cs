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

public class HorizontalLidarScanner : MonoBehaviour
{


    private IPublisher<sensor_msgs.msg.LaserScan> laserScanPublisher;
    public float scanRadius = 10f;
    public int numberOfRays = 360;
    public float scanInterval = 1f;
    public GameObject hitMarkerPrefab; // Reference to the green square prefab
    public float hitMarkerLifetime = 2f; // Lifetime of the hit marker
    List<float> ranges = new List<float>();

    public int max = 10;
    public int min = 0;

    private ROS2Node rosNode;
    private LaserScanPublisher lPublisher;

    ROS2UnityCore ros2Unity = new ROS2UnityCore();


    private void Start()
    {

        if (ros2Unity.Ok())
        {
            rosNode = ros2Unity.CreateNode("unity_lidar_capture");
            
        laserScanPublisher = rosNode.CreatePublisher<sensor_msgs.msg.LaserScan>("/scan");

        }
        else
        {
            Debug.LogError("ROS2UnityCore is not OK");
        }
        StartCoroutine(Scan());
    }

    private IEnumerator Scan()
    {
        while (true)
        {
            for (int i = numberOfRays; i > 0; i--)
            {
                float angle = i * (360f / numberOfRays);
                Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
                RaycastHit hit;
                if (Physics.Raycast(transform.position, direction, out hit, scanRadius))
                {
                    Debug.Log($"Hit detected at {hit.point}");
                    ranges.Add(hit.distance);
                    GameObject hitMarker = Instantiate(hitMarkerPrefab, hit.point, Quaternion.identity); // Instantiate the square
                    Destroy(hitMarker, hitMarkerLifetime); // Destroy the hit marker after a certain duration
                }
            }

           PublishImage();

            yield return new WaitForSeconds(scanInterval);
        }
    }


    private void PublishImage() {

        var msg = new sensor_msgs.msg.LaserScan
        {
            Header = new std_msgs.msg.Header
            {
                Frame_id = "Unity"
            },

            Range_min = min,
            Range_max = max,
            Angle_min = 0,
            Angle_max = 360,
            Angle_increment = (360 - 0) / numberOfRays,
            Time_increment = 1,
            Scan_time = (float)1,
            Intensities = new float[ranges.Count],
            Ranges = ranges.ToArray(),
        };
        Debug.Log("Publishing");
       laserScanPublisher.Publish(msg);



    }
}