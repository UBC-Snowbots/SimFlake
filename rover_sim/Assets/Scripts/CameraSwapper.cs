using UnityEngine;

/// <summary>Press C to toggle between two cameras.</summary>
public class CameraSwapper : MonoBehaviour
{
    public Camera cam1;
    public Camera cam2;

    void Start()
    {
        if (cam1 != null) cam1.enabled = true;
        if (cam2 != null) cam2.enabled = false;
    }

    void Update()
    {
        if (cam1 == null || cam2 == null) return;
        if (Input.GetKeyDown(KeyCode.C))
        {
            cam1.enabled = !cam1.enabled;
            cam2.enabled = !cam2.enabled;
        }
    }
}
