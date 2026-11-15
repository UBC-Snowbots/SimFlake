using System.Linq;
using UnityEngine;

/// <summary>
/// Physics settings for an imported robot that can't be saved in a prefab. Put it on the robot's root object.
///  - Solver iterations of the root ArticulationBody (runtime-only property). Light links (the mini rover's 70 g
///    bogies carrying 565 g wheels) need more than the project default (6 / 1): with it, skid turns stalled
///    some wheel drives against ground friction.
///  - Ignores collisions between the robot's own links. Articulations only skip parent/child pairs, and the convex
///    (VHACD) hulls of imported meshes are fatter than the real parts (e.g. a wheel rubbing on the chassis).
///    Raycasts (lidar) are not affected: the rover still sees its own parts.
/// </summary>
public class RobotPhysicsSetup : MonoBehaviour
{
    public int solverIterations = 30;
    public int solverVelocityIterations = 20;
    public bool ignoreSelfCollisions = true;

    void Awake()
    {
        var root = GetComponentsInChildren<ArticulationBody>().FirstOrDefault(b => b.isRoot);
        if (root != null)
        {
            root.solverIterations = solverIterations;
            root.solverVelocityIterations = solverVelocityIterations;
        }
    }

    void Start()
    {
        if (!ignoreSelfCollisions) return;
        // Colliders on one body never collide with each other; pair up colliders of different bodies.
        var groups = GetComponentsInChildren<Collider>(true)
                     .GroupBy(c => (Component)c.attachedArticulationBody ?? c.attachedRigidbody ?? (Component)c)
                     .Select(g => g.ToArray()).ToArray();
        for (int i = 0; i < groups.Length; i++)
            for (int j = i + 1; j < groups.Length; j++)
                foreach (var a in groups[i])
                    foreach (var b in groups[j]) Physics.IgnoreCollision(a, b);
    }
}
