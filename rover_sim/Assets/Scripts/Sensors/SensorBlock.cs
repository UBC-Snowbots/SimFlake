using TMPro;
using UnityEngine;

/// <summary>
/// Makes a sensor visible and movable: a coloured box (with a collider, so other sensors like the lidar see it)
/// and a floating label. Put it on the sensor's own GameObject (the one with the sensor script, at the sensor's
/// frame). Move/rotate that object to try a different mount; the sensor goes with it.
///
/// Children it manages (created by the builder):
///   Block  cube mesh + BoxCollider, sized/offset by <see cref="size"/> / <see cref="center"/>
///   Label  TextMeshPro, faces whichever camera renders it, drawn on top of geometry, hidden from sensor cameras
/// </summary>
[ExecuteAlways]
public class SensorBlock : MonoBehaviour
{
    public string label = "Sensor";
    public Color color = Color.white;
    [Tooltip("Block size in metres.")]
    public Vector3 size = new Vector3(0.04f, 0.04f, 0.04f);
    [Tooltip("Block centre relative to the sensor frame (this object), in metres.")]
    public Vector3 center = Vector3.zero;

    [Header("Label")]
    public bool showLabel = true;
    [Tooltip("Height of the label text in metres.")]
    public float labelSize = 0.025f;
    [Tooltip("Gap between the top of the block and the label, in metres.")]
    public float labelGap = 0.03f;

    public Transform block;
    public TMP_Text labelText;

    static readonly int ColorId = Shader.PropertyToID("_Color");
    MaterialPropertyBlock props;

    void OnEnable()
    {
        Camera.onPreCull += FaceCamera;
        Apply();
    }

    void OnDisable() => Camera.onPreCull -= FaceCamera;

    void OnValidate() => Apply();

    /// <summary>Pushes size, colour and label text onto the children.</summary>
    public void Apply()
    {
        if (block != null)
        {
            block.localPosition = center;
            block.localRotation = Quaternion.identity;
            block.localScale = size;
            var r = block.GetComponent<Renderer>();
            if (r != null)
            {
                props ??= new MaterialPropertyBlock();
                r.GetPropertyBlock(props);
                props.SetColor(ColorId, color);
                r.SetPropertyBlock(props);
            }
        }
        if (labelText != null)
        {
            labelText.text = label;
            labelText.color = color;
            labelText.fontSize = labelSize * 10f; // TMP world text: fontSize 1 is 0.1 m tall
            labelText.transform.localScale = Vector3.one;
        }
    }

    // Runs for every camera (Game, Scene view, sensor cameras) just before it culls.
    void FaceCamera(Camera cam)
    {
        if (labelText == null) return;
        var r = labelText.GetComponent<Renderer>();
        // Sensor cameras render into a RenderTexture (SensorCamera.Render): keep labels out of the published images.
        bool show = showLabel && isActiveAndEnabled
                    && (cam.cameraType == CameraType.SceneView || (cam.cameraType == CameraType.Game && cam.targetTexture == null));
        if (r != null) r.enabled = show;
        if (!show) return;

        Transform t = labelText.transform;
        Vector3 top = transform.TransformPoint(center) + Vector3.up * (size.y * 0.5f * Mathf.Abs(transform.lossyScale.y) + labelGap);
        t.position = top;
        t.rotation = Quaternion.LookRotation(t.position - cam.transform.position, cam.transform.up);
    }

    void OnDrawGizmosSelected()
    {
        // Sensor frame axes: red = right, green = up, blue = forward (Unity); ROS FLU forward is the blue axis.
        Gizmos.matrix = transform.localToWorldMatrix;
        float l = Mathf.Max(size.x, size.y, size.z) * 1.2f;
        Gizmos.color = Color.red; Gizmos.DrawLine(Vector3.zero, Vector3.right * l);
        Gizmos.color = Color.green; Gizmos.DrawLine(Vector3.zero, Vector3.up * l);
        Gizmos.color = Color.blue; Gizmos.DrawLine(Vector3.zero, Vector3.forward * l);
    }
}
