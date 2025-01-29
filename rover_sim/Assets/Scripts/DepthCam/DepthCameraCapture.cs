using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Threading.Tasks;
using ROS2;
using System.Runtime.CompilerServices;
using sensor_msgs.msg;
using std_msgs.msg;

public class DepthCameraCapture : MonoBehaviour
{
    public Camera depthCamera; // The depth camera
    public int width = 1920; // Width of the image
    public int height = 1080; // Height of the image
    public int fps = 30; // Frames per second

    public string frameid = "cam_unused";

    private RenderTexture renderTexture;
    private Texture2D depthTexture;

    // ROS2 Node and Publisher
    private ROS2Node rosNode;
    private IPublisher<sensor_msgs.msg.Image> depthImagePublisher;

    ROS2UnityCore ros2Unity = new ROS2UnityCore();

    void Start()
    {
        // Initialize the RenderTexture and Texture2D
        renderTexture = new RenderTexture(width, height, 24, RenderTextureFormat.Depth);
        depthTexture = new Texture2D(width, height, TextureFormat.RFloat, false);

        // Set the camera's target texture to the RenderTexture
        depthCamera.targetTexture = renderTexture;

        // Initialize ROS2
        if (ros2Unity.Ok())
        {
            rosNode = ros2Unity.CreateNode("unity_depth_capture");
            depthImagePublisher = rosNode.CreatePublisher<sensor_msgs.msg.Image>("/camera/depth/image_raw");
        }
        else
        {
            Debug.LogError("ROS2UnityCore is not OK");
        }

        StartCoroutine(CaptureAndPublishDepthImages());
    }

    IEnumerator CaptureAndPublishDepthImages()
    {
        while (true)
        {
            yield return new WaitForSeconds(1.0f / fps);
            CaptureDepthImage();
        }
    }

    void CaptureDepthImage()
    {
        // Render the camera's view to the RenderTexture
        depthCamera.Render();

        // Read the RenderTexture into the Texture2D
        RenderTexture.active = renderTexture;
        depthTexture.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        depthTexture.Apply();
        RenderTexture.active = null;

        // Convert the Texture2D to a byte array
        byte[] depthBytes = TextureToByteArray(depthTexture);

        // Publish the depth data
        PublishDepthImage(depthBytes);
    }

    private byte[] TextureToByteArray(Texture2D texture)
    {
        // Extract raw pixel data (RFloat = 32-bit float values)
        float[] depthData = new float[texture.width * texture.height];
        Color[] pixels = texture.GetPixels();
        for (int i = 0; i < pixels.Length; i++)
        {
            depthData[i] = pixels[i].r; // Assuming depth is stored in the red channel
        }

        // Convert float array to byte array
        byte[] byteArray = new byte[depthData.Length * sizeof(float)];
        System.Buffer.BlockCopy(depthData, 0, byteArray, 0, byteArray.Length);

        return byteArray;
    }

    private void PublishDepthImage(byte[] depthBytes)
    {
        // Create the ROS 2 Image message for depth
        var depthMessage = new Image
        {
            Header = RoverUtils.CreateHeader(frameid),
            Height = (uint)height,
            Width = (uint)width,
            Encoding = "32FC1", // 32-bit floating point, single channel
            Step = (uint)(width * sizeof(float)),
            Data = depthBytes
        };

        // Publish the depth image to ROS 2
        depthImagePublisher.Publish(depthMessage);
    }
}
