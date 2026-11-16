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
}

/// <summary>
/// SmoothCylinder: convex mesh cylinder fitted to the tread. Exact width, but PhysX convex contacts jitter while
///   rolling at small timesteps (measured on chassis_2024 at 5 ms: wheels off the ground 25 % of steps, 0.5 g body vibration).
/// Sphere: analytic sphere of the tread radius. Rolls perfectly smoothly; bulges sideways (2 x radius wide), so side
///   contact with rocks/walls happens a bit early.
/// Grousers: cylinder core + box paddles. KeepImported: leave the URDF/VHACD colliders.
/// </summary>
public enum WheelColliderShape { SmoothCylinder, Grousers, KeepImported, Sphere }
