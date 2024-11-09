using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RGBCameraObject : MonoBehaviour
{
    public Transform target; // The target object to follow

    public Vector3 initialOffset; // Initial offset from the target object



    void Start()
    {
        // Save the initial set of the camera from the rover
        // this allows the user to move the camera in the scene view and have it updated in the game view
        initialOffset = transform.position - target.position;
    }

    void LateUpdate()
    {
        if (target != null)
        {
            // Update the rover's position to follow the target with the specified offset
            transform.position = target.position + initialOffset;

            // Update the rover's rotation to match the target's rotation
            transform.rotation = target.rotation;
        }
    }
}