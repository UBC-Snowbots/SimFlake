using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// What the drivetrain knows about a kind of ground, keyed by the PhysicMaterial on the ground's collider.
/// Friction itself stays in the PhysicMaterial (PhysX applies it). This adds what PhysX doesn't model.
///
/// Today: rolling resistance. Terramechanics parameters (Bekker/Wong) are listed for the next tier (a custom
/// wheel-soil force model, sinkage, ruts). Nothing reads them yet.
/// All specs live in Resources/GroundSpecs.asset (<see cref="GroundSpecTable"/>). Unknown materials use its default.
/// </summary>
[CreateAssetMenu(menuName = "SimFlake/Ground", fileName = "Ground")]
public class GroundSpec : ScriptableObject
{
    [TextArea(2, 6)] public string notes;
    [Tooltip("Collider material this spec applies to")]
    public PhysicMaterial material;

    [Tooltip("Rolling-resistance coefficient for a rigid rover wheel: resisting force = coefficient x wheel load. " +
             "Asphalt ~0.015, packed dirt ~0.05, loose dirt ~0.1, sand 0.15-0.3.")]
    public float rollingResistance = 0.05f;

    [Header("Terramechanics (next tier; not used yet)")]
    [Tooltip("kN/m^(n+1), cohesive modulus of deformation")]
    public float kc = 0.9f;
    [Tooltip("kN/m^(n+2), frictional modulus of deformation")]
    public float kphi = 1523.4f;
    [Tooltip("sinkage exponent")]
    public float n = 1.1f;
    [Tooltip("kPa, cohesion")]
    public float cohesion = 1.04f;
    [Tooltip("deg, internal friction angle")]
    public float frictionAngle = 28f;
    [Tooltip("m, shear deformation modulus")]
    public float shearModulus = 0.0254f;
    [Tooltip("Can wheels leave ruts here")]
    public bool deformable = false;
}
