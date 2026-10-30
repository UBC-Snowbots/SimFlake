using System;
using SimFlake.Ros;
using TMPro;
using UnityEngine;

/// <summary>
/// Simulated GNSS receiver. Publishes sensor_msgs/NavSatFix on /gps/fix.
///
/// The Unity world is treated as a flat local tangent plane centred on (startLatitude, startLongitude)
/// at the tracked object's start position: Unity +z = north, +x = east, +y = up.
/// 1 Unity unit = 1 metre.
/// </summary>
public class LocationStuff : MonoBehaviour
{
    [SerializeField] private char unit = 'K';

    public TMP_Text debugTxt;
    public bool gps_ok = false;

    [Tooltip("The object whose position is reported (the rover's base link). Defaults to this object.")]
    public Transform object_to_track;

    [Header("Origin (lat/lon at the rover's start position)")]
    public double startLatitude = 49.0;
    public double startLongitude = -123.0;
    public double startAltitude = 100.0;

    [Header("ROS")]
    public string topic = "/gps/fix";
    public string frameId = "gnss";
    [Tooltip("Publish rate in Hz")]
    [SerializeField] private float publishRateHz = 5f;
    [Tooltip("Seconds before the first fix (simulates GNSS start-up).")]
    public float startupDelay = 2f;

    [Header("Noise (metres, 1 sigma). 0 = perfect GPS")]
    public float horizontalNoise = 0f;
    public float verticalNoise = 0f;

    const double EarthRadius = 6378137.0; // WGS-84 equatorial radius

    RosPublisher<sensor_msgs.msg.NavSatFix> pub;
    Vector3 startPosition;
    double lat, lon, alt;
    double storedLat, storedLon;
    bool measureDistance;
    float timer;
    System.Random rng = new System.Random();

    void Start()
    {
        if (object_to_track == null) object_to_track = transform;
        startPosition = object_to_track.position;
        pub = Ros.Advertise<sensor_msgs.msg.NavSatFix>(topic, 2);
        SetText("Initializing GPS...");
        Invoke(nameof(StartGps), startupDelay);
    }

    void StartGps()
    {
        gps_ok = true;
        SetText("GPS initialized.");
    }

    void Update()
    {
        if (!gps_ok) return;

        Vector3 d = object_to_track.position - startPosition;
        double east = d.x + Gaussian() * horizontalNoise;
        double north = d.z + Gaussian() * horizontalNoise;
        double up = d.y + Gaussian() * verticalNoise;
        lat = startLatitude + Rad2Deg(north / EarthRadius);
        lon = startLongitude + Rad2Deg(east / (EarthRadius * Math.Cos(Deg2Rad(startLatitude))));
        alt = startAltitude + up;

        if (debugTxt != null)
        {
            string text = $"Current Location:\nLat: {lat:F7}\nLon: {lon:F7}";
            if (measureDistance)
                text += $"\nDistance from start: {distance(lat, lon, storedLat, storedLon, unit):F3} km";
            debugTxt.text = text;
        }

        timer += Time.deltaTime;
        if (timer >= 1f / publishRateHz)
        {
            timer = 0;
            PublishGPS();
        }
    }

    void PublishGPS()
    {
        var msg = new sensor_msgs.msg.NavSatFix
        {
            header = RosTime.Header(frameId),
            latitude = lat,
            longitude = lon,
            altitude = alt,
            position_covariance_type = sensor_msgs.msg.NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
        };
        msg.status.status = sensor_msgs.msg.NavSatStatus.STATUS_FIX;
        double h2 = Math.Max(horizontalNoise * horizontalNoise, 1e-4);
        double v2 = Math.Max(verticalNoise * verticalNoise, 1e-4);
        msg.position_covariance[0] = h2;
        msg.position_covariance[4] = h2;
        msg.position_covariance[8] = v2;
        pub.Publish(msg);
    }

    public void StopGPS()
    {
        gps_ok = false;
        SetText("GPS stopped.");
    }

    public void StoreCurrentGPS()
    {
        storedLat = lat;
        storedLon = lon;
        measureDistance = true;
    }

    void SetText(string s) { if (debugTxt != null) debugTxt.text = s; }

    double Gaussian()
    {
        // Box-Muller
        double u1 = 1.0 - rng.NextDouble(), u2 = rng.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
    }

    // Great-circle distance. unit: 'K' km, 'N' nautical miles, otherwise statute miles.
    private double distance(double lat1, double lon1, double lat2, double lon2, char unit)
    {
        if (lat1 == lat2 && lon1 == lon2) return 0;
        double theta = lon1 - lon2;
        double dist = Math.Sin(Deg2Rad(lat1)) * Math.Sin(Deg2Rad(lat2))
                    + Math.Cos(Deg2Rad(lat1)) * Math.Cos(Deg2Rad(lat2)) * Math.Cos(Deg2Rad(theta));
        dist = Rad2Deg(Math.Acos(Math.Min(1.0, dist))) * 60 * 1.1515;
        if (unit == 'K') dist *= 1.609344;
        else if (unit == 'N') dist *= 0.8684;
        return dist;
    }

    static double Deg2Rad(double deg) => deg * Math.PI / 180.0;
    static double Rad2Deg(double rad) => rad / Math.PI * 180.0;
}
