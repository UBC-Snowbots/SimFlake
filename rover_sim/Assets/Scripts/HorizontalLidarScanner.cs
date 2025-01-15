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
                float angle = i * ((360f) / numberOfRays);
                Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
                // RaycastHit hit;
                if (Physics.Raycast(transform.position, direction, out var hit, scanRadius))
                    {
                    Debug.Log($"Hit detected at {hit.point}");
                    ranges.Add(hit.distance);
                    Debug.Log(hit.distance);

                    GameObject hitMarker = Instantiate(hitMarkerPrefab, hit.point, Quaternion.identity); // Instantiate the square
                    Destroy(hitMarker, hitMarkerLifetime); // Destroy the hit marker after a certain duration
                } else {
                ranges.Add(float.MaxValue);
                }

            }


            // var yawSensorDegrees = Mathf.Lerp(m_CurrentScanAngleStart, m_CurrentScanAngleEnd, t);
            // var yawDegrees = yawBaseDegrees + yawSensorDegrees;
            // var directionVector = Quaternion.Euler(0f, yawDegrees, 0f) * Vector3.forward;
            // var measurementStart = RangeMetersMin * directionVector + transform.position;
            // var measurementRay = new Ray(measurementStart, directionVector);
            // var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, RangeMetersMax);
            // // Only record measurement if it's within the sensor's operating range
            // if (foundValidMeasurement)
            // {
            //     ranges.Add(hit.distance);
            // }
            // else
            // {
            //     ranges.Add(float.MaxValue);
            // }
            // }

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
            Angle_min = 0 * Mathf.Deg2Rad,
            Angle_max = 360 * Mathf.Deg2Rad,
            Angle_increment = (2 * Mathf.PI)  / numberOfRays,
            Time_increment = 1.0f / numberOfRays,
            Scan_time = (float)1,
            Intensities = new float[ranges.Count],
            Ranges = ranges.ToArray(),
        };
        Debug.Log("Publishing");
       laserScanPublisher.Publish(msg);



    }
}