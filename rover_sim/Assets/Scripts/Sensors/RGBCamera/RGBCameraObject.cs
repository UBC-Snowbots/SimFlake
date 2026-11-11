using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RGBCameraObject : MonoBehaviour
{
    public Transform target; // The target object to follow
    private Vector3 initialOffset; // Initial offset from the target object

    void Start()
    {
        // Save the initial offset in the target's local space
        initialOffset = target.InverseTransformPoint(transform.position);
    }

    void LateUpdate()
    {
        if (target != null)
        {
            // Calculate the new position based on the target's rotation and initial offset
            Vector3 desiredPosition = target.TransformPoint(initialOffset);
            transform.position = desiredPosition;

            // Directly set the rotation to match the target's rotation
            transform.rotation = target.rotation;
        }
    }
}