using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using System;

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
        }
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