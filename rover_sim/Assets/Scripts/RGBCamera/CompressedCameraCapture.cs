using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Threading.Tasks;
using ROS2;
using sensor_msgs.msg;
using std_msgs.msg;

public class CompressedCameraCapture : MonoBehaviour
{
    public Camera rgbCamera; // The RGB camera
    public int width = 1920; // Width of the image
    public int height = 1080; // Height of the image
    public int fps = 30; // Frames per second

    private RenderTexture renderTexture;
    private Texture2D texture2D;

    // ROS2 Node and Publisher
    private ROS2Node rosNode;
    private IPublisher<CompressedImage> compressedImagePublisher;

    ROS2UnityCore ros2Unity = new ROS2UnityCore();

    void Start()
    {
        // Initialize the RenderTexture and Texture2D
        renderTexture = new RenderTexture(width, height, 24);
        texture2D = new Texture2D(width, height, TextureFormat.RGB24, false);

        // Set the camera's target texture to the RenderTexture
        rgbCamera.targetTexture = renderTexture;

        // Initialize ROS2
        if (ros2Unity.Ok())
        {
            rosNode = ros2Unity.CreateNode("unity_compressed_image_capture");
            compressedImagePublisher = rosNode.CreatePublisher<CompressedImage>("/camera/image/compressed");
        }
        else
        {
            Debug.LogError("ROS2UnityCore is not OK");
        }

        StartCoroutine(CaptureAndPublishImages());
    }

    IEnumerator CaptureAndPublishImages()
    {
   float interval = 1.0f / fps;
        float nextExecutionTime = Time.time;

    while (true)
    {
        // Ensure the next execution happens at the fixed interval
        nextExecutionTime += interval;


        CaptureAndPublishCompressedImage();

        // Calculate the time to wait until the next execution
        float sleepTime = nextExecutionTime - Time.time;
        if (sleepTime > 0)
        {
            yield return new WaitForSeconds(sleepTime);
        }
        else
        {
            // If we're behind schedule, skip waiting to catch up
            Debug.LogWarning("Frame skipped to maintain timing! This means i couldn't keep up with the FPS");
        }
    }
    }

    async void CaptureAndPublishCompressedImage()
    {
        // Render the camera's view to the RenderTexture
        rgbCamera.Render();

        // Read the RenderTexture into the Texture2D
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        // Compress the image to JPEG
        // byte[] compressedImageBytes = texture2D.EncodeToJPG(50); // Quality: 75 (adjustable)
                // Capture the image
        // Task<byte[]> compressTask = Task.Run(() =>
        // {
            Debug.LogWarning("Frame skipped to maintain timing! This means i couldn't keep up with the FPS");
        byte[] compressedImageBytes = texture2D.EncodeToJPG(50);
        // return compressedImageBytes;

        // });
        // Create the ROS2 CompressedImage message
        // byte[] compressedImageBytes = await compressTask;
        var compressedImageMessage = new CompressedImage
        {
            Header = new std_msgs.msg.Header
            {
                Frame_id = "camera_frame",
                Stamp = new builtin_interfaces.msg.Time
                {
                    Sec = (int)(Time.timeSinceLevelLoad),
                    Nanosec = (uint)((Time.timeSinceLevelLoad - Mathf.Floor(Time.timeSinceLevelLoad)) * 1e9f)
                }
            },
            Format = "jpeg",

            Data = compressedImageBytes
        };

        // Publish the compressed image message
        compressedImagePublisher.Publish(compressedImageMessage);
    }
}
