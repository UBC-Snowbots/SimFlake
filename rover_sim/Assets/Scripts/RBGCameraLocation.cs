using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RGBCameraLocation : MonoBehaviour
{
    public Transform target; // The target object to follow
    public Vector3 offset; // Offset from the target object

    void LateUpdate()
    {
        if (target != null)
        {
            // Update the camera's position to follow the target with the specified offset
            transform.position = target.position + offset;

            // Update the camera's rotation to match the target's rotation
            transform.rotation = target.rotation;
        }
    }
}
