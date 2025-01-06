using UnityEngine;

public class DepthToROS : MonoBehaviour
{
    public Camera depthCamera;
    public RenderTexture depthTexture;

    void Update()
    {
        RenderTexture.active = depthTexture;
        Texture2D depthImage = new Texture2D(depthTexture.width, depthTexture.height, TextureFormat.RFloat, false);
        depthImage.ReadPixels(new Rect(0, 0, depthTexture.width, depthTexture.height), 0, 0);
        depthImage.Apply();
        RenderTexture.active = null;

        // Convert depthImage to a ROS 2 message format here
        // For example, use RobotecAI's ROS 2 Unity integration to publish.
    }
}
