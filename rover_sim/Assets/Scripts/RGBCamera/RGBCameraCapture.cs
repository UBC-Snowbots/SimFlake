using System.Buffers;
using SimFlake.Ros;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Raw RGB camera: publishes sensor_msgs/Image (rgb8) and sensor_msgs/CameraInfo.
///
/// Raw images are big (width*height*3 bytes per frame). Through rosbridge, 640x480 @ 30 Hz is fine;
/// 1280x720 tops out around 15 Hz. For high resolutions use CompressedCameraCapture instead.
/// </summary>
public class RGBCameraCapture : MonoBehaviour
{
    public Camera rgbCamera;
    public int width = 640;
    public int height = 480;
    public int fps = 30;
    public string frameid = "camera_link";

    public string imageTopic = "/camera/image_raw";
    public string cameraInfoTopic = "/camera/camera_info";
    [Tooltip("Stop the camera drawing to the screen; it is still rendered off-screen for ROS.")]
    public bool hideFromScreen = true;

    RosPublisher<sensor_msgs.msg.Image> imagePub;
    RosPublisher<sensor_msgs.msg.CameraInfo> infoPub;
    float timer;
    [Tooltip("Frames that may be in the readback/encode/send pipeline at once.")]
    public int maxInFlight = 2;
    int inFlight;

    void Start()
    {
        if (rgbCamera == null) rgbCamera = GetComponent<Camera>();
        if (rgbCamera == null) { Debug.LogError($"[{name}] RGBCameraCapture has no camera"); enabled = false; return; }
        if (hideFromScreen) rgbCamera.enabled = false;
        if ((long)width * height * 3 * fps > 30_000_000)
            Debug.LogWarning($"[{name}] Raw camera {width}x{height}@{fps} is ~{width * height * 3L * fps / 1_000_000} MB/s; rosbridge will drop frames. Lower the resolution/fps or use CompressedCameraCapture.");

        // Dedicated socket: big frames never delay small messages like /cmd_vel feedback or IMU.
        imagePub = Ros.Advertise<sensor_msgs.msg.Image>(imageTopic, 1, dedicatedSocket: true);
        infoPub = Ros.Advertise<sensor_msgs.msg.CameraInfo>(cameraInfoTopic, 2);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 1f / fps) return;
        timer = 0;
        if (System.Threading.Volatile.Read(ref inFlight) >= maxInFlight || !Ros.Connected) return; // previous frame still being sent, or nobody to send to

        var rt = SensorCamera.Render(rgbCamera, width, height);
        var header = RosTime.Header(frameid);
        System.Threading.Interlocked.Increment(ref inFlight);
        AsyncGPUReadback.Request(rt, 0, TextureFormat.RGB24, req => OnReadback(req, header));
    }

    void OnReadback(AsyncGPUReadbackRequest req, std_msgs.msg.Header header)
    {
        if (req.hasError || this == null) { System.Threading.Interlocked.Decrement(ref inFlight); return; }
        byte[] pixels = SensorCamera.CopyRows(req.GetData<byte>(), width, height, 3, topDown: true);
        int w = width, h = height;
        infoPub.Publish(SensorCamera.Info(rgbCamera, w, h, header));

        SensorCamera.Background(() =>
        {
            try
            {
                imagePub.Publish(new sensor_msgs.msg.Image
                {
                    header = header,
                    width = (uint)w,
                    height = (uint)h,
                    encoding = "rgb8",
                    step = (uint)(w * 3),
                    data = pixels,
                    dataLength = w * h * 3,
                });
            }
            finally
            {
                ArrayPool<byte>.Shared.Return(pixels);
                System.Threading.Interlocked.Decrement(ref inFlight);
            }
        });
    }
}
