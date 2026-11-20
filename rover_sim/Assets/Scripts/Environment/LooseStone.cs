using UnityEngine;

/// <summary>
/// A stone lying loose on the ground (made by <see cref="StoneField"/>). It rests as a kinematic body exactly on the
/// ground you see, costing nothing, and becomes a normal Rigidbody the moment a wheel, the chassis or another moving
/// stone touches it.
/// </summary>
public class LooseStone : MonoBehaviour
{
    Rigidbody rb;

    void Awake() { rb = GetComponent<Rigidbody>(); }

    void OnCollisionEnter(Collision c)
    {
        if (rb == null || !rb.isKinematic || c.collider is TerrainCollider) return;
        rb.isKinematic = false;
        rb.WakeUp();
    }
}
