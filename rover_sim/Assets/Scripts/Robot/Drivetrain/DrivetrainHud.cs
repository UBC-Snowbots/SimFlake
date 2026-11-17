using UnityEngine;

/// <summary>
/// On-screen drivetrain readout (Game view): per wheel speed, torque, current, winding temperature, load, slip and
/// ground; on soil also sinkage, contact patch, contact pressure and how much of the available grip is in use.
/// Press F8 to toggle. Put it on the rover root (the builders do).
/// </summary>
public class DrivetrainHud : MonoBehaviour
{
    public bool visible = false;
    public KeyCode toggleKey = KeyCode.F8;

    DriveWheel[] wheels;
    ArticulationBody root;
    GUIStyle style;
    Vector3 lastPos, velocity; float lastT = -1;

    void Start()
    {
        wheels = GetComponentsInChildren<DriveWheel>();
        foreach (var b in GetComponentsInChildren<ArticulationBody>()) if (b.isRoot) root = b;
    }

    void Update()
    {
        if (Input.GetKeyDown(toggleKey)) visible = !visible;
    }

    // Speed from positions: ArticulationBody.velocity reads several % high with the TGS solver.
    void FixedUpdate()
    {
        if (root == null) return;
        Vector3 p = root.transform.position;
        if (lastT >= 0 && Time.fixedTime > lastT) velocity = (p - lastPos) / (Time.fixedTime - lastT);
        lastPos = p; lastT = Time.fixedTime;
    }

    void OnGUI()
    {
        if (!visible || wheels == null || wheels.Length == 0) return;
        if (style == null)
            style = new GUIStyle(GUI.skin.label) { font = Font.CreateDynamicFontFromOSFont("Monospace", 12), fontSize = 12, richText = true };

        var sb = new System.Text.StringBuilder();
        if (root != null)
        {
            Vector3 v = velocity;
            float fwd = Vector3.Dot(v, root.transform.forward);
            float pitch = Vector3.SignedAngle(Vector3.ProjectOnPlane(root.transform.forward, Vector3.up), root.transform.forward, root.transform.right);
            sb.AppendLine($"<b>{name}</b>  speed {fwd:+0.00;-0.00} m/s  yaw rate {-root.angularVelocity.y:+0.00;-0.00} rad/s  pitch {-pitch:+0;-0}°");
        }
        var soil = SoilWorld.Instance;
        if (soil != null)
            sb.AppendLine(soil.Active ? $"soil model: {soil.quality} ({soil.Current.cell * 100:0.#} cm cells, {soil.Current.nTheta}x{soil.Current.nY} tyre elements, physics {1f / Time.fixedDeltaTime:0} Hz)  [{soil.cycleKey}: change]"
                                      : $"soil model: Off (plain PhysX contacts)  [{soil.cycleKey}: change]");
        sb.AppendLine("wheel            set    act rad/s  torque  avail     I     T°C   load N  slip  angle  sink mm  patch mm  kPa  grip used  ground");
        foreach (var w in wheels)
        {
            var c = w.Contact;
            string sat = w.Saturated ? "<color=orange>*</color>" : " ";
            string hot = w.WindingTemperature > (w.motor != null ? w.motor.maxWindingTemperature - 20 : 1e9f) ? "orange" : "white";
            string ground = c.grounded ? (c.groundSpec != null ? c.groundSpec.name : c.ground != null ? c.ground.name : "?") : "<color=grey>air</color>";
            sb.AppendLine($"{Short(w.name),-14} {w.SetpointSpeed,6:F2} {w.Speed,6:F2}  {w.OutputTorque,7:F2}{sat} {w.TorqueAvailable,6:F2} {w.MotorCurrent,6:F1} " +
                          $"<color={hot}>{w.WindingTemperature,6:F0}</color>{c.normalForce,8:F0} {c.slipRatio,5:F2} {c.slipAngle,6:F1} " +
                          (w.Soil.grounded ? $"{w.Soil.sinkage * 1000,8:F1} {w.Soil.patchLength * 1000,5:F0}x{w.Soil.patchWidth * 1000,-3:F0} {w.Soil.pressure / 1000,4:F0} {(w.Soil.tractionLimit > 0 ? Mathf.Abs(w.Soil.traction) / w.Soil.tractionLimit * 100 : 0),8:F0} %" : "       -        -     -          -") +
                          $"  {ground}");
        }
        var content = new GUIContent(sb.ToString());
        Vector2 size = style.CalcSize(content);
        var rect = new Rect(10, Screen.height - size.y - 20, size.x + 20, size.y + 10);
        GUI.Box(rect, GUIContent.none);
        GUI.Label(new Rect(rect.x + 10, rect.y + 5, size.x, size.y), content, style);
    }

    static string Short(string n) => n.Replace("_wheel_link", "").Replace("_Link", "");
}
