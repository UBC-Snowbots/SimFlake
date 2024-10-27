using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraOrbit : MonoBehaviour
{
    public Transform target; // The target object to orbit around
    public float distance = 10.0f; // Distance from the target
    public float xSpeed = 120.0f; // Speed of rotation around the x-axis
    public float ySpeed = 120.0f; // Speed of rotation around the y-axis
    public float yMinLimit = -20f; // Minimum y-axis angle
    public float yMaxLimit = 80f; // Maximum y-axis angle
    public float zoomSpeed = 2.0f; // Speed of zooming
    public float minDistance = 2.0f; // Minimum zoom distance
    public float maxDistance = 20.0f; // Maximum zoom distance

    private float x = 0.0f;
    private float y = 0.0f;

    void Start()
    {
        Vector3 angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;

        // Ensure the camera starts at the correct distance from the target
        if (target != null)
        {
            transform.position = target.position - transform.forward * distance;
        }
    }

    void LateUpdate()
    {
        if (target != null)
        {
            // Rotate with arrow keys
            if (Input.GetKey(KeyCode.LeftArrow))
            {
                x -= xSpeed * Time.deltaTime;
            }
            if (Input.GetKey(KeyCode.RightArrow))
            {
                x += xSpeed * Time.deltaTime;
            }
            if (Input.GetKey(KeyCode.UpArrow))
            {
                y -= ySpeed * Time.deltaTime;
            }
            if (Input.GetKey(KeyCode.DownArrow))
            {
                y += ySpeed * Time.deltaTime;
            }

            // Zoom with Z and X keys
            if (Input.GetKey(KeyCode.Z))
            {
                distance = Mathf.Clamp(distance - zoomSpeed * Time.deltaTime, minDistance, maxDistance);
            }
            if (Input.GetKey(KeyCode.X))
            {
                distance = Mathf.Clamp(distance + zoomSpeed * Time.deltaTime, minDistance, maxDistance);
            }

            y = ClampAngle(y, yMinLimit, yMaxLimit);

            Quaternion rotation = Quaternion.Euler(y, x, 0);
            Vector3 position = rotation * new Vector3(0.0f, 0.0f, -distance) + target.position;

            transform.rotation = rotation;
            transform.position = position;
        }
    }

    static float ClampAngle(float angle, float min, float max)
    {
        if (angle < -360F)
            angle += 360F;
        if (angle > 360F)
            angle -= 360F;
        return Mathf.Clamp(angle, min, max);
    }
}
