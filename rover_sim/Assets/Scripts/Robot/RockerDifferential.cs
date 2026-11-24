using UnityEngine;

/// <summary>
/// Rocker-bogie differential: couples the two rocker joints so right = -left (what the differential bar does on
/// the real rover). Without it both rockers swing freely and the body tips over its rocker axle.
///
/// PhysX articulations have no gear/mimic constraint, so this uses stiff position drives: every physics step each
/// rocker is driven toward the antisymmetric pose that keeps their difference (predicted one step ahead, so a stiff drive
/// doesn't drag on the terrain-following motion). That resists only the symmetric motion (body pitch relative to both
/// rockers) and leaves the terrain-following motion free. The stiffness is the whole pitch stiffness of the body: a body
/// whose centre of mass is d behind the rocker pivot sags m g d / (2 x stiffness) radians (chassis_2026 at 50: 11.6 deg).
/// The differential bar link (cosmetic) is driven to <see cref="differentialMultiplier"/> x the left rocker angle,
/// matching the URDF's &lt;mimic&gt;.
/// </summary>
public class RockerDifferential : MonoBehaviour
{
    public ArticulationBody leftRocker;
    public ArticulationBody rightRocker;
    [Tooltip("Optional: the differential bar, driven to follow the rockers.")]
    public ArticulationBody differential;
    [Tooltip("Differential angle = multiplier x left rocker angle (URDF mimic multiplier).")]
    public float differentialMultiplier = 0.25f;

    [Tooltip("Drive stiffness per rocker (N·m per radian: Unity passes it to PhysX as is) holding right = -left. " +
             "Sag = m g d / (2 x stiffness) rad for a body whose centre of mass is d behind the rocker pivot.")]
    public float stiffness = 50f;
    [Tooltip("Drive damping per rocker (N·m per rad/s).")]
    public float damping = 2f;
    public float forceLimit = 1000f;

    void Start()
    {
        foreach (var j in new[] { leftRocker, rightRocker, differential })
        {
            if (j == null) continue;
            var d = j.xDrive;
            d.stiffness = stiffness;
            d.damping = damping;
            d.forceLimit = forceLimit;
            d.target = 0;
            j.xDrive = d;
        }
        if (leftRocker == null || rightRocker == null)
            Debug.LogError($"[RockerDifferential] {name}: both rockers must be assigned");
    }

    void FixedUpdate()
    {
        if (leftRocker == null || rightRocker == null) return;
        float dt = Time.fixedDeltaTime;
        float l = Pos(leftRocker) + Vel(leftRocker) * dt, r = Pos(rightRocker) + Vel(rightRocker) * dt;
        float half = (l - r) * 0.5f; // the free (antisymmetric) mode, where it will be at the end of this step
        SetTarget(leftRocker, half);
        SetTarget(rightRocker, -half);
        if (differential != null) SetTarget(differential, differentialMultiplier * half);
    }

    static float Pos(ArticulationBody j) => j.jointPosition.dofCount > 0 ? j.jointPosition[0] * Mathf.Rad2Deg : 0f;
    static float Vel(ArticulationBody j) => j.jointVelocity.dofCount > 0 ? j.jointVelocity[0] * Mathf.Rad2Deg : 0f;

    static void SetTarget(ArticulationBody j, float deg)
    {
        var d = j.xDrive;
        d.target = deg;
        j.xDrive = d;
    }
}
