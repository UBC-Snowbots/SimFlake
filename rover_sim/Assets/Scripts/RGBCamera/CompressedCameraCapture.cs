using System.Buffers;
using SimFlake.Ros;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;

/// <summary>
/// JPEG camera: publishes sensor_msgs/CompressedImage. JPEG encoding runs on a worker thread,
/// so high resolutions (1080p @ 30) are fine for the sim's frame rate.
/// </summary>
public class CompressedCameraCapture : MonoBehaviour
{
    public Camera rgbCamera;
    public int width = 1280;
    public int height = 720;
    public int fps = 30;
    [Range(1, 100)] public int jpegQuality = 75;
    public string frameid = "camera_link";
    public string topic = "/camera/image/compressed";
    [Tooltip("Stop the camera drawing to the screen; it is still rendered off-screen for ROS.")]
    public bool hideFromScreen = true;

    RosPublisher<sensor_msgs.msg.CompressedImage> pub;
    float timer;
    [Tooltip("Frames that may be in the readback/encode/send pipeline at once.")]
    public int maxInFlight = 2;
    int inFlight;

    void Start()
    {
        if (rgbCamera == null) rgbCamera = GetComponent<Camera>();
        if (rgbCamera == null) { Debug.LogError($"[{name}] CompressedCameraCapture has no camera"); enabled = false; return; }
        if (hideFromScreen) rgbCamera.enabled = false;
        pub = Ros.Advertise<sensor_msgs.msg.CompressedImage>(topic, 1, dedicatedSocket: true);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 1f / fps) return;
        timer = 0;
        if (System.Threading.Volatile.Read(ref inFlight) >= maxInFlight || !Ros.Connected) return;

        var rt = SensorCamera.Render(rgbCamera, width, height);
        var header = RosTime.Header(frameid);
        System.Threading.Interlocked.Increment(ref inFlight);
        AsyncGPUReadback.Request(rt, 0, TextureFormat.RGB24, req => OnReadback(req, header));
    }

    void OnReadback(AsyncGPUReadbackRequest req, std_msgs.msg.Header header)
    {
        if (req.hasError || this == null) { System.Threading.Interlocked.Decrement(ref inFlight); return; }
        byte[] pixels = SensorCamera.CopyRows(req.GetData<byte>(), width, height, 3, topDown: false);
        int w = width, h = height, q = jpegQuality;

        SensorCamera.Background(() =>
        {
            try
            {
                byte[] jpg = ImageConversion.EncodeArrayToJPG(pixels, GraphicsFormat.R8G8B8_SRGB, (uint)w, (uint)h, (uint)(w * 3), q);
                pub.Publish(new sensor_msgs.msg.CompressedImage { header = header, format = "jpeg", data = jpg });
            }
            finally
            {
                ArrayPool<byte>.Shared.Return(pixels);
                System.Threading.Interlocked.Decrement(ref inFlight);
            }
        });
    }
}
