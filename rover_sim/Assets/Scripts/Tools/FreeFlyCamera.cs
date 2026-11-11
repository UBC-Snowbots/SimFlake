using UnityEngine;

/// <summary>
/// Spectator camera for looking around a world in Play mode (no robot needed).
/// Hold the right mouse button to look, WASD to move, Q/E down/up, Shift = fast, scroll = change speed.
/// </summary>
public class FreeFlyCamera : MonoBehaviour
{
    public float speed = 10f;
    public float fastMultiplier = 5f;
    public float lookSensitivity = 3f;

    float yaw, pitch;

    void Start()
    {
        var e = transform.eulerAngles;
        yaw = e.y;
        pitch = e.x > 180 ? e.x - 360 : e.x;
    }

    void Update()
    {
        if (Input.GetMouseButton(1))
        {
            yaw += Input.GetAxis("Mouse X") * lookSensitivity;
            pitch = Mathf.Clamp(pitch - Input.GetAxis("Mouse Y") * lookSensitivity, -89f, 89f);
            transform.rotation = Quaternion.Euler(pitch, yaw, 0);
        }
        speed = Mathf.Clamp(speed * (1 + Input.mouseScrollDelta.y * 0.1f), 0.5f, 200f);

        var move = new Vector3(
            (Input.GetKey(KeyCode.D) ? 1 : 0) - (Input.GetKey(KeyCode.A) ? 1 : 0),
            (Input.GetKey(KeyCode.E) ? 1 : 0) - (Input.GetKey(KeyCode.Q) ? 1 : 0),
            (Input.GetKey(KeyCode.W) ? 1 : 0) - (Input.GetKey(KeyCode.S) ? 1 : 0));
        float s = speed * (Input.GetKey(KeyCode.LeftShift) ? fastMultiplier : 1f);
        transform.position += transform.rotation * move * (s * Time.deltaTime);
    }
}
