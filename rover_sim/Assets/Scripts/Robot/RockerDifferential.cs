using UnityEngine;

/// <summary>
/// Rocker-bogie differential: couples the two rocker joints so right = -left (what the differential bar does on
/// the real rover). Without it both rockers swing freely and the body tips over its rocker axle.
///
/// PhysX articulations have no gear/mimic constraint, so this uses stiff position drives: every physics step each
/// rocker is driven toward the antisymmetric pose that keeps their current difference. That resists only the
/// symmetric motion (body pitch relative to both rockers) and leaves the terrain-following motion free.
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

    [Tooltip("Drive stiffness (N·m per degree) holding right = -left.")]
    public float stiffness = 50f;
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
        float l = Pos(leftRocker), r = Pos(rightRocker);
        float half = (l - r) * 0.5f; // the free (antisymmetric) mode
        SetTarget(leftRocker, half);
        SetTarget(rightRocker, -half);
        if (differential != null) SetTarget(differential, differentialMultiplier * half);
    }

    static float Pos(ArticulationBody j) => j.jointPosition.dofCount > 0 ? j.jointPosition[0] * Mathf.Rad2Deg : 0f;

    static void SetTarget(ArticulationBody j, float deg)
    {
        var d = j.xDrive;
        d.target = deg;
        j.xDrive = d;
    }
}
