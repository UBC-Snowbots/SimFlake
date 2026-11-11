using SimFlake.Ros;
using UnityEngine;

/// <summary>
/// EXPERIMENTAL / not used in any scene. Straight port of the old depth publisher so it still compiles.
/// A proper RealSense-style RGBD camera is on the roadmap (see CLAUDE.md).
/// </summary>
public class DepthCameraCapture : MonoBehaviour
{
    public Camera depthCamera;
    public int width = 640;
    public int height = 480;
    public int fps = 15;
    public string frameid = "camera_depth_frame";
    public string topic = "/camera/depth/image_raw";

    RenderTexture renderTexture;
    Texture2D depthTexture;
    RosPublisher<sensor_msgs.msg.Image> pub;
    float timer;

    void Start()
    {
        renderTexture = new RenderTexture(width, height, 24, RenderTextureFormat.RFloat);
        depthTexture = new Texture2D(width, height, TextureFormat.RFloat, false);
        depthCamera.targetTexture = renderTexture;
        pub = Ros.Advertise<sensor_msgs.msg.Image>(topic, 1, dedicatedSocket: true);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 1f / fps) return;
        timer = 0;

        depthCamera.Render();
        RenderTexture.active = renderTexture;
        depthTexture.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        depthTexture.Apply();
        RenderTexture.active = null;

        pub.Publish(new sensor_msgs.msg.Image
        {
            header = RosTime.Header(frameid),
            width = (uint)width,
            height = (uint)height,
            encoding = "32FC1",
            step = (uint)(width * sizeof(float)),
            data = depthTexture.GetRawTextureData(),
        });
    }
}
