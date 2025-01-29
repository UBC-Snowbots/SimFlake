using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Threading.Tasks;
using ROS2;
using System.Runtime.CompilerServices;
using sensor_msgs.msg;
using std_msgs.msg;

public class RGBCameraCapture : MonoBehaviour
{
    public Camera rgbCamera; // The RGB camera
    public int width = 1920; // Width of the image
    public int height = 1080; // Height of the image
    public int fps = 30; // Frames per second

    public string frameid = "cam_unused";
    private RenderTexture renderTexture;
    private Texture2D texture2D;


    // ROS2 Node and Publisher
    //private ROS2UnityComponent ros2Unity;
    private ROS2Node rosNode;
    private IPublisher<sensor_msgs.msg.Image> imagePublisher;

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
            rosNode = ros2Unity.CreateNode("unity_image_capture");
            // Subscribe to motor commands topic
            
            imagePublisher = rosNode.CreatePublisher<sensor_msgs.msg.Image>("/camera/image_raw");

        }
        else
        {
            Debug.LogError("ROS2UnityCore is not OK");
        }


        StartCoroutine(CaptureAndPublishImages());



    }





    //TODO: Huge issue, encoding six million uncompressed bytes per second is insane
    // don't know how to compress images... need to figure out... still technically works
    // 
    IEnumerator CaptureAndPublishImages()
    {
        float interval = 1.0f / fps;
        float nextExecutionTime = Time.time;

    while (true)
    {
        // Ensure the next execution happens at the fixed interval
        nextExecutionTime += interval;

        // Capture the image
        CaptureImage();

        // Calculate the time to wait until the next execution
        float sleepTime = nextExecutionTime - Time.time;
        if (sleepTime > 0)
        {
            yield return new WaitForSeconds(sleepTime);
        }
        else
        {
            // If we're behind schedule, skip waiting to catch up
            Debug.LogWarning("Frame skipped to maintain timing!");
        }
    }
    }

   //encodes rendered view into byte[] to send thru ROS network
    void CaptureImage()
    {
        // Render the camera's view to the RenderTexture
        rgbCamera.Render();

        // Read the RenderTexture into the Texture2D
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        texture2D.Apply();
        RenderTexture.active = null;

        // Convert the Texture2D to a byte array
        //texture2D.Compress(false);
        byte[] imageBytes = texture2D.GetRawTextureData();


        
        
        //return byte array
        PublishImage(imageBytes);
        
    }



    //publishing single image to ROS..
     private void PublishImage(byte[] imageBytes)
    {

        // Create the ROS 2 Image message
        var imageMessage = new Image
        {
            Header  = RoverUtils.CreateHeader(frameid),
            Height = (uint)height,      //requires unsigned... doesn't matter for our usecase, can explicit cast
            Width = (uint)width,        //ditto 
            Encoding = "rgb8",            // Image encoding 
            Step = (uint)width * 3,             // 3 bytes per pixel for RGB encoding
            Data = imageBytes             // The image data as byte array
        };

        // Publish the image message to ROS 2
        imagePublisher.Publish(imageMessage);

      //  Debug.Log("Image message published to ROS 2 topic!");
    }


    


 
}