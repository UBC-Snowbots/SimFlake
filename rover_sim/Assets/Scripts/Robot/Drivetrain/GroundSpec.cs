using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// What the drivetrain knows about a kind of ground. Two users:
///
/// 1. PhysX contacts (rocks, ramps, any collider that is not a soil terrain): keyed by the collider's PhysicMaterial.
///    Friction comes from the PhysicMaterial; this adds <see cref="rollingResistance"/>.
/// 2. The wheel-soil model (<see cref="WheelSoilModel"/>, on terrains with a <see cref="SoilTerrain"/>): keyed by terrain
///    layer. Everything under "Terramechanics" and below is used: Bekker pressure-sinkage, Mohr-Coulomb shear with
///    Janosi-Hanamoto shear displacement, crust, how the soil moves (ruts, berms, digging in) and the small-scale relief.
///
/// All specs live in Resources/GroundSpecs.asset (<see cref="GroundSpecTable"/>). Unknown materials use its default.
/// </summary>
[CreateAssetMenu(menuName = "SimFlake/Ground", fileName = "Ground")]
public class GroundSpec : ScriptableObject
{
    [TextArea(2, 6)] public string notes;
    [Tooltip("Collider material this spec applies to (PhysX contacts)")]
    public PhysicMaterial material;

    [Tooltip("PhysX contacts only. Rolling-resistance coefficient for a rigid rover wheel: resisting force = coefficient x wheel load. " +
             "Asphalt ~0.015, packed dirt ~0.05, loose dirt ~0.1, sand 0.15-0.3. On soil terrains rolling resistance comes out of the soil model.")]
    public float rollingResistance = 0.05f;

    [Header("Terramechanics: pressure-sinkage (Bekker: p = (kc/b + kphi) z^n)")]
    [Tooltip("kN/m^(n+1), cohesive modulus of deformation")]
    public float kc = 0.9f;
    [Tooltip("kN/m^(n+2), frictional modulus of deformation")]
    public float kphi = 1523.4f;
    [Tooltip("sinkage exponent")]
    public float n = 1.1f;
    [Tooltip("kPa. Undisturbed ground carries this much pressure before it starts to sink (dry crust, desert pavement). " +
             "A light rover leaves only scuffs; a heavy one breaks through.")]
    public float crustStrength = 0f;

    [Header("Terramechanics: shear (Mohr-Coulomb + Janosi-Hanamoto)")]
    [Tooltip("kPa, cohesion")]
    public float cohesion = 1.04f;
    [Tooltip("deg, internal friction angle (soil shearing on soil: between tread lugs)")]
    public float frictionAngle = 28f;
    [Tooltip("m, shear deformation modulus: how far the tread must slip to build up traction. Loose sand ~0.025, firm soil ~0.01.")]
    public float shearModulus = 0.0254f;
    [Tooltip("Friction coefficient of tyre rubber sliding on this ground (soil grains or rock). Soil can't give more than its " +
             "own internal strength, so on loose soil the lugs matter more than this. Dry sandstone ~0.8, dusty ~0.6.")]
    public float rubberFriction = 0.6f;

    [Header("How the soil moves")]
    [Tooltip("Can wheels leave ruts here. Off = rigid ground (rock): only friction is used.")]
    public bool deformable = false;
    [Tooltip("kg/m³, bulk density (weight of the soil a wheel pushes aside)")]
    public float density = 1500f;
    [Tooltip("deg. Rut walls and berms slump to this slope. Dry sand ~32, cohesive soil 50-70 (crisp ruts and lug prints).")]
    public float reposeAngle = 34f;
    [Tooltip("0..1. Part of the soil pressed down by a wheel that disappears into a denser floor; the rest is pushed up beside the rut.")]
    [Range(0, 1)] public float compaction = 0.6f;
    [Tooltip("0..1. How readily spinning lugs dig soil out from under the wheel and throw it backwards (slip sinkage). Dry sand ~0.5, firm soil ~0.1.")]
    [Range(0, 1)] public float excavation = 0.3f;

    [Header("Small-scale relief (wheels feel it, the terrain shader shows it)")]
    [Tooltip("m, height of small bumps: clods, stones, ripples (15-20 cm apart)")]
    public float roughnessFine = 0.003f;
    [Tooltip("m, height of gentle undulations (0.3-1 m apart)")]
    public float roughnessCoarse = 0.01f;
    [Tooltip("0..1. How much darker freshly disturbed soil looks (broken crust, turned-over soil).")]
    [Range(0, 1)] public float disturbedDarkening = 0.25f;

    // ---- derived, SI ----
    /// <summary>Pa per m^n for a contact of width b (m).</summary>
    public float Stiffness(float b) => (kc / Mathf.Max(b, 0.01f) + kphi) * 1000f;
    public float TanPhi => Mathf.Tan(frictionAngle * Mathf.Deg2Rad);
    /// <summary>Passive earth pressure coefficient (soil pushed by a lug face or a wheel's side).</summary>
    public float Kp { get { float t = Mathf.Tan((45f + frictionAngle * 0.5f) * Mathf.Deg2Rad); return t * t; } }
}
