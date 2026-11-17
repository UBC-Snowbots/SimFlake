using UnityEngine;

/// <summary>
/// One wheel design, as data. <c>SimFlake &gt; Drivetrain &gt; Apply Wheel Specs</c> (or a rover builder) turns it into the
/// wheel's collider and mass; <see cref="DriveWheel"/> uses the rest at runtime.
///
/// To try a different wheel: duplicate an asset, change it, point the rover's DriveWheels at it, re-apply.
/// Radius 0 = keep the size measured from the imported mesh (the tread of the visual mesh).
/// Create new ones with Assets > Create > SimFlake > Wheel.
/// </summary>
[CreateAssetMenu(menuName = "SimFlake/Wheel", fileName = "Wheel")]
public class WheelSpec : ScriptableObject
{
    [TextArea(2, 6)] public string notes;

    [Header("Geometry (0 = measure from the imported mesh)")]
    [Tooltip("m, to the tread surface")]
    public float radius = 0f;
    [Tooltip("m, tread width")]
    public float width = 0f;

    [Header("Collider")]
    [Tooltip("Sphere: smoothest rolling (recommended for heavy rovers). SmoothCylinder: exact width, some contact jitter. " +
             "Grousers adds paddles: more bite on loose ground, bumpier on hard ground.")]
    public WheelColliderShape shape = WheelColliderShape.SmoothCylinder;
    [Tooltip("Number of grousers (Grousers shape only)")]
    public int grouserCount = 12;
    [Tooltip("m, how far grousers stand out from the tread (Grousers shape only)")]
    public float grouserHeight = 0.01f;
    [Tooltip("Friction of the tread. With 'Multiply' combine, 1.0 means the ground's own friction applies unchanged.")]
    public PhysicMaterial material;

    [Header("Mass (0 = keep the URDF value)")]
    [Tooltip("kg")]
    public float mass = 0f;

    [Header("Rolling resistance")]
    [Tooltip("Scales the ground's rolling-resistance coefficient (GroundSpec). Wide/soft tyres < 1, narrow rigid wheels > 1.")]
    public float rollingResistanceScale = 1f;

    [Header("Tyre (wheel-soil model, used on soil terrains)")]
    [Tooltip("Shore A hardness of the tyre rubber. Mold Max 40 silicone = 40. Softer = bigger contact patch, more grip on rock.")]
    [Range(5, 95)] public float shoreA = 40f;
    [Tooltip("m, rubber between the tread surface (lug base) and the rigid hub")]
    public float rubberThickness = 0.008f;
    [Tooltip("Loss factor (tan delta) of the rubber: the part of the squash energy lost as heat each turn. Rolling resistance " +
             "on hard ground comes from it. Silicone 0.05-0.15, natural rubber 0.1-0.3.")]
    public float lossTangent = 0.1f;
    [Tooltip("Scales the ground's rubber friction for this compound (1 = the GroundSpec value). Silicone on dusty rock grips " +
             "less than natural rubber or polyurethane: try 0.8-1.2.")]
    public float gripScale = 1f;
    [Tooltip("Rubber grips a little less at high contact pressure: friction x (pressure / 100 kPa)^exponent. 0 = Coulomb, -0.1 typical.")]
    public float gripPressureExponent = -0.1f;
    [Tooltip("The tread, measured from the wheel's STL: SimFlake > Drivetrain > Bake Tread From Mesh. Empty = smooth cylinder of radius x width.")]
    public TreadData tread = new TreadData();

    /// <summary>Young's modulus of the rubber (Pa) from Shore A (Gent's relation).</summary>
    public float YoungsModulus => 0.0981f * (56f + 7.62336f * shoreA) / (0.137505f * (254f - 2.54f * shoreA)) * 1e6f;
}

/// <summary>
/// SmoothCylinder: convex mesh cylinder fitted to the tread. Exact width, but PhysX convex contacts jitter while
///   rolling at small timesteps (measured on chassis_2024 at 5 ms: wheels off the ground 25 % of steps, 0.5 g body vibration).
/// Sphere: analytic sphere of the tread radius. Rolls perfectly smoothly; bulges sideways (2 x radius wide), so side
///   contact with rocks/walls happens a bit early.
/// Grousers: cylinder core + box paddles. KeepImported: leave the URDF/VHACD colliders.
/// </summary>
public enum WheelColliderShape { SmoothCylinder, Grousers, KeepImported, Sphere }
