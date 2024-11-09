using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class RGBCameraCapture : MonoBehaviour
{
    public Camera rgbCamera; // The RGB camera
    public int width = 1920; // Width of the captured image
    public int height = 1080; // Height of the captured image
    public int fps = 30; // Frames per second

    private RenderTexture renderTexture;
    private Texture2D texture2D;

    void Start()
    {
        // Initialize the RenderTexture and Texture2D
        renderTexture = new RenderTexture(width, height, 24);
        texture2D = new Texture2D(width, height, TextureFormat.RGB24, false);

        // Set the camera's target texture to the RenderTexture
        rgbCamera.targetTexture = renderTexture;

        // Start the coroutine to capture images at 30 fps
        StartCoroutine(CaptureImages());
    }

    IEnumerator CaptureImages()
    {
        while (true)
        {
            yield return new WaitForSeconds(1.0f / fps);

            // Capture the image
            CaptureImage();
        }
    }

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
        byte[] imageBytes = texture2D.EncodeToPNG();


        //TODO: send image over ROS network... 
        // or save the image to a file, then send over asynchronusly (no clue if this is necessary)
                                                                     //need to learn more about unity


    }
}