using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using System;

//ROS2 stuff
using ROS2;
using sensor_msgs.msg;

public class LocationStuff : MonoBehaviour
{
    [SerializeField]
    private char unit = 'K';

    public TMP_Text debugTxt;
    public bool gps_ok = false;
    float PI = Mathf.PI;

    GPSLoc startLoc = new GPSLoc();
    GPSLoc currLoc = new GPSLoc();

    bool measureDistance = false;

    Vector3 startPosition;  // Store the rover's initial position in world space
    public Transform object_to_track;

    [Header("ROS2 Publishing")]
    [Tooltip("Publish rate in Hz")]
    [SerializeField] private float publishRateHz = 5f;
    ROS2UnityCore ros2Unity = new ROS2UnityCore();
    private float publishTimer = 0f;
    private ROS2Node rosNode;
    private IPublisher<NavSatFix> gpsPublisher;
    private static readonly DateTime epoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
    // Start is called before the first frame update
    IEnumerator Start()
    {
        // Simulate starting position as 0, 0 in local GPS coordinates
        startLoc = new GPSLoc(0, 0);
        startPosition = object_to_track.position;

        debugTxt.text = "Initializing GPS...";
        yield return new WaitForSeconds(2);  // Simulate a brief delay for GPS startup
        gps_ok = true;
        debugTxt.text = "GPS initialized. Starting at (0, 0).";
       // Initialize ROS2
        if (ros2Unity.Ok())
        {
            rosNode = ros2Unity.CreateNode("gps_sensor_node");
            gpsPublisher = rosNode.CreatePublisher<NavSatFix>("/gps/fix");
        }
        else
        {
            Debug.LogError("ROS2UnityCore is not OK");
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (gps_ok)
        {
            // Calculate new local GPS coordinates relative to starting position
            currLoc.lat = (object_to_track.position.x - startPosition.x) * 0.0001f;  // Scale factor for lat
            currLoc.lon = (object_to_track.position.z - startPosition.z) * 0.0001f;  // Scale factor for lon

            debugTxt.text = "Current Location: \nLat: " + currLoc.lat.ToString("F6") 
                          + "\nLon: " + currLoc.lon.ToString("F6");

            if (measureDistance)
            {
                double distanceBetween = distance(currLoc.lat, currLoc.lon, startLoc.lat, startLoc.lon, 'K');
                debugTxt.text += "\nDistance from start: " + distanceBetween.ToString("F2") + " km";
            }
                publishTimer += Time.deltaTime;
            if (publishTimer >= (1f / publishRateHz))
            {
                publishTimer = 0f;
                PublishGPS();
            }
        }
  
    }
    private void PublishGPS()
    {
        if (gpsPublisher == null)
            return;

        // Create a new NavSatFix message
        NavSatFix msg = new NavSatFix();

        // // 1. Fill the header with a ROS timestamp
        // double secs = (DateTime.UtcNow - epoch).TotalSeconds; //? Little odd, but unity sets start point as Jan 01, 0001 (start of time). All other computers think time started at Jan 1, 1970, so we subtract the amount of seconds from 0001 to 1970.
        // // uint secsInt = (uint)secs;
        // uint nsecs = (uint)((secs - secsInt) * 1e9);

        // msg.Header = new std_msgs.msg.Header
        //     {
        //         Frame_id = "camera_frame",
        //         Stamp = new builtin_interfaces.msg.Time
        //         {
        //             Sec = (int)(secs),//(Time.timeSinceLevelLoad),
        //             Nanosec = (uint)(nsecs)//((Time.timeSinceLevelLoad - Mathf.Floor(Time.timeSinceLevelLoad)) * 1e9f)
        //         }
        //     };
        msg.Header = RoverUtils.CreateHeader("gnss"); //! FRAME ID

        // 2. Fill latitude/longitude (and altitude if available)
        msg.Latitude = currLoc.lat;   // NavSatFix expects double
        msg.Longitude = currLoc.lon;  // so cast if needed
        msg.Altitude = 100.0;

        // 3. Publish
        gpsPublisher.Publish(msg);
    }
    public void StopGPS()
    {
        gps_ok = false;
        debugTxt.text = "GPS stopped.";

    }

    public void StoreCurrentGPS()
    {
        startLoc = new GPSLoc(currLoc.lon, currLoc.lat);
        measureDistance = true;
        debugTxt.text += "\nStored current location as new starting point.";
    }

    //https://www.geodatasource.com/resources/tutorials/how-to-calculate-the-distance-between-2-locations-using-c/
    private double distance(double lat1, double lon1, double lat2, double lon2, char unit)
    {
        if ((lat1 == lat2) && (lon1 == lon2))
        {
            return 0;
        }
        else
        {
            double theta = lon1 - lon2;
            double dist = Math.Sin(deg2rad(lat1)) * Math.Sin(deg2rad(lat2)) 
                        + Math.Cos(deg2rad(lat1)) * Math.Cos(deg2rad(lat2)) * Math.Cos(deg2rad(theta));
            dist = Math.Acos(dist);
            dist = rad2deg(dist);
            dist = dist * 60 * 1.1515;
            if (unit == 'K')
            {
                dist = dist * 1.609344;
            }
            else if (unit == 'N')
            {
                dist = dist * 0.8684;
            }
            return (dist);
        }
    }

    //:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    //::  This function converts decimal degrees to radians             :::
    //:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    private double deg2rad(double deg)
    {
        return (deg * Math.PI / 180.0);
    }

    //:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    //::  This function converts radians to decimal degrees             :::
    //:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    private double rad2deg(double rad)
    {
        return (rad / Math.PI * 180.0);
    }

}


public class GPSLoc
{
    public float lon;
    public float lat;

    public GPSLoc()
    {
        lon = 0;
        lat = 0;
    }
    public GPSLoc(float lon, float lat)
    {
        this.lon = lon;
        this.lat = lat;
    }

    public string getLocData()
    {
        return "Lat: " + lat + " \nLon: " + lon;
    }
}