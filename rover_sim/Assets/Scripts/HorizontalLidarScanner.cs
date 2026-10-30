using SimFlake.Ros;
using Unity.Collections;
using UnityEngine;

/// <summary>
/// 2D (planar) lidar. Casts <see cref="numberOfRays"/> rays in a full circle around this
/// object's up axis and publishes sensor_msgs/LaserScan.
/// Angles follow ROS: 0 = this object's forward (+z in Unity), counter-clockwise seen from above.
/// Misses are reported as +inf (REP-117).
/// </summary>
public class HorizontalLidarScanner : MonoBehaviour
{
    public string topic = "/scan";
    public float scanRadius = 10f;
    public int numberOfRays = 360;
    [Tooltip("Seconds between scans (0.1 = 10 Hz, like most real lidars).")]
    public float scanInterval = 0.1f;
    public string frameid = "lidar";
    [Tooltip("Reported range_max. Hits beyond scanRadius are never seen.")]
    public int max = 10;
    [Tooltip("Reported range_min. Hits closer than this are dropped (report +inf).")]
    public int min = 0;
    [Tooltip("Layers the lidar can see. Default: everything except 'Ignore Raycast'.")]
    public LayerMask layers = Physics.DefaultRaycastLayers;
    [Tooltip("Draw the rays in the Scene view.")]
    public bool debugDraw = false;

    RosPublisher<sensor_msgs.msg.LaserScan> pub;
    float timer;

    void Start()
    {
        pub = Ros.Advertise<sensor_msgs.msg.LaserScan>(topic, 2);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < scanInterval) return;
        timer = 0;
        Scan();
    }

    void Scan()
    {
        int n = Mathf.Max(1, numberOfRays);
        float inc = 2 * Mathf.PI / n;
        Vector3 origin = transform.position;
        Vector3 up = transform.up, fwd = transform.forward;

        // Batched raycasts (runs on worker threads).
        var cmds = new NativeArray<RaycastCommand>(n, Allocator.TempJob);
        var hits = new NativeArray<RaycastHit>(n, Allocator.TempJob);
        for (int i = 0; i < n; i++)
        {
            // Unity rotates clockwise for positive angles about +y, ROS is counter-clockwise: negate.
            Vector3 dir = Quaternion.AngleAxis(-i * inc * Mathf.Rad2Deg, up) * fwd;
            cmds[i] = new RaycastCommand(origin, dir, new QueryParameters(layers), scanRadius);
        }
        RaycastCommand.ScheduleBatch(cmds, hits, 32).Complete();

        var ranges = new float[n];
        for (int i = 0; i < n; i++)
        {
            var h = hits[i];
            bool hit = h.collider != null && h.distance >= min;
            ranges[i] = hit ? h.distance : float.PositiveInfinity;
            if (debugDraw)
                Debug.DrawRay(origin, cmds[i].direction * (hit ? h.distance : scanRadius), hit ? Color.green : Color.gray, scanInterval);
        }
        cmds.Dispose();
        hits.Dispose();

        pub.Publish(new sensor_msgs.msg.LaserScan
        {
            header = RosTime.Header(frameid),
            angle_min = 0,
            angle_max = inc * (n - 1),
            angle_increment = inc,
            time_increment = 0,
            scan_time = scanInterval,
            range_min = min,
            range_max = max,
            ranges = ranges,
        });
    }
}
