using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Collects the contact forces on one arm link during the last physics step, for <see cref="ArmBreakage"/>.
/// Added at runtime to every link (collision messages go to the ArticulationBody's object).
/// </summary>
[AddComponentMenu("")]
public class ArmLinkContacts : MonoBehaviour
{
    public readonly List<Vector3> points = new List<Vector3>();
    public readonly List<Vector3> forces = new List<Vector3>();
    /// <summary>What it touched last (debugging).</summary>
    public string lastOther;

    void OnCollisionEnter(Collision c) => Add(c);
    void OnCollisionStay(Collision c) => Add(c);

    void Add(Collision c)
    {
        int n = c.contactCount;
        if (n == 0) return;
        Vector3 p = Vector3.zero, normal = Vector3.zero;
        for (int i = 0; i < n; i++)
        {
            var cp = c.GetContact(i);
            p += cp.point;
            normal += cp.normal;
        }
        p /= n;
        Vector3 f = c.impulse / Time.fixedDeltaTime;
        // Unity does not say which body the impulse is for. A contact pushes this body away from the other one, and
        // the contact normals point from the other body towards this one.
        if (Vector3.Dot(f, normal) < 0) f = -f;
        lastOther = c.collider != null ? c.collider.name + (c.articulationBody != null ? " (" + c.articulationBody.name + ")" : "") : null;
        points.Add(p);
        forces.Add(f);
    }

    /// <summary>Called by ArmBreakage after reading (callbacks of one step arrive before the next FixedUpdate).</summary>
    public void Clear() { points.Clear(); forces.Clear(); }
}
