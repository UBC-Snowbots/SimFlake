using UnityEngine;

/// <summary>
/// Third-person camera that follows a rover, like a third-person game. <see cref="SimMenu"/> puts it on the view camera
/// and switches between it and <see cref="FreeFlyCamera"/> (V, or the CAMERA tab of the Esc menu).
/// Hold the right mouse button to orbit, scroll to zoom, R = back behind the rover. After a moment without mouse input the
/// camera swings back behind the rover while it drives. It pulls in when something (terrain, rocks) is between it and the rover.
/// </summary>
public class FollowCamera : MonoBehaviour
{
    public Transform target;
    public float distance = 5f;
    public float minDistance = 0.8f, maxDistance = 40f;
    [Range(-15, 85)] public float pitch = 18f;
    public float lookSensitivity = 3f;
    [Tooltip("Swing back behind the rover while it drives, after recenterDelay seconds without mouse input")]
    public bool autoRecenter = true;
    public float recenterDelay = 2f;
    [Tooltip("How quickly the camera catches up with the rover (1/s)")]
    public float followSharpness = 10f;
    public KeyCode recenterKey = KeyCode.R;

    Transform roverRoot;
    float yaw, pivotHeight = 0.4f, currentDistance, speed, lastInput = -999f;
    Vector3 pivot, lastTargetPos;
    bool snap = true;
    readonly RaycastHit[] hits = new RaycastHit[32];

    /// <summary>Follow <paramref name="body"/> (the rover's moving root, e.g. chassis_link). Picks a distance from the rover's size.</summary>
    public void SetTarget(Transform body)
    {
        target = body;
        snap = true;
        if (body == null) return;

        // Highest ancestor that is still "the rover" (does not also contain a terrain): its colliders never block the view.
        roverRoot = body;
        while (roverRoot.parent != null && roverRoot.parent.GetComponentInChildren<Terrain>() == null) roverRoot = roverRoot.parent;

        var bounds = new Bounds(body.position, Vector3.zero);
        foreach (var r in body.GetComponentsInChildren<MeshRenderer>()) bounds.Encapsulate(r.bounds);
        float size = Mathf.Max(0.2f, bounds.extents.magnitude);
        distance = Mathf.Clamp(size * 3.5f, 1.5f, 15f);
        pivotHeight = size * 0.35f;
    }

    public void Recenter() => snap = true;

    void OnEnable() { snap = true; }

    void LateUpdate()
    {
        if (target == null) return;
        float dt = Mathf.Max(Time.unscaledDeltaTime, 1e-4f);

        if (!SimMenu.IsOpen)
        {
            if (Input.GetMouseButton(1))
            {
                yaw += Input.GetAxis("Mouse X") * lookSensitivity;
                pitch = Mathf.Clamp(pitch - Input.GetAxis("Mouse Y") * lookSensitivity, -15f, 85f);
                lastInput = Time.unscaledTime;
            }
            float scroll = Input.mouseScrollDelta.y;
            if (scroll != 0) distance = Mathf.Clamp(distance * (1f - scroll * 0.1f), minDistance, maxDistance);
            if (Input.GetKeyDown(recenterKey)) { snap = true; lastInput = -999f; }
        }

        Vector3 tp = target.position;
        Vector3 fwd = Vector3.ProjectOnPlane(target.forward, Vector3.up);
        float heading = fwd.sqrMagnitude > 1e-6f ? Mathf.Atan2(fwd.x, fwd.z) * Mathf.Rad2Deg : yaw;
        // Rendered frames and physics steps don't line up, so smooth the speed estimate.
        speed = Mathf.Lerp(speed, (tp - lastTargetPos).magnitude / dt, 1f - Mathf.Exp(-5f * dt));
        lastTargetPos = tp;

        Vector3 p = tp + Vector3.up * pivotHeight;
        if (snap)
        {
            yaw = heading;
            pivot = p;
            currentDistance = distance;
            speed = 0;
        }
        else
        {
            if (autoRecenter && Time.unscaledTime - lastInput > recenterDelay && speed > 0.1f)
                yaw = Mathf.LerpAngle(yaw, heading, 1f - Mathf.Exp(-1.5f * dt));
            pivot = Vector3.Lerp(pivot, p, 1f - Mathf.Exp(-followSharpness * dt));
        }

        Quaternion rot = Quaternion.Euler(pitch, yaw, 0);
        Vector3 back = rot * Vector3.back;

        // Pull in in front of anything between rover and camera (not the rover itself); ease back out.
        float d = distance;
        int n = Physics.SphereCastNonAlloc(pivot, 0.2f, back, hits, distance, ~0, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < n; i++)
        {
            var h = hits[i];
            if (h.distance <= 0f || (roverRoot != null && h.collider.transform.IsChildOf(roverRoot))) continue;
            d = Mathf.Min(d, h.distance);
        }
        d = Mathf.Max(d, Mathf.Min(minDistance, distance) * 0.5f);
        currentDistance = d < currentDistance || snap ? d : Mathf.Lerp(currentDistance, d, 1f - Mathf.Exp(-3f * dt));

        transform.SetPositionAndRotation(pivot + back * currentDistance, rot);
        snap = false;
    }
}
