using System.Collections.Generic;
using UnityEngine;

/// <summary>Lookup table: Resources/GroundSpecs.asset.</summary>
public class GroundSpecTable : ScriptableObject
{
    public GroundSpec defaultGround;
    public List<GroundSpec> grounds = new List<GroundSpec>();

    static GroundSpecTable table;
    static bool loaded;
    static readonly Dictionary<PhysicMaterial, GroundSpec> cache = new Dictionary<PhysicMaterial, GroundSpec>();

    public static GroundSpec Lookup(PhysicMaterial m)
    {
        if (!loaded) { loaded = true; table = Resources.Load<GroundSpecTable>("GroundSpecs"); }
        if (table == null) return null;
        if (m == null) return table.defaultGround;
        if (cache.TryGetValue(m, out var g)) return g;
        g = table.grounds.Find(s => s != null && s.material == m);
        // Runtime material instances are named "<name> (Instance)".
        if (g == null) g = table.grounds.Find(s => s != null && s.material != null && m.name.StartsWith(s.material.name));
        if (g == null) g = table.defaultGround;
        cache[m] = g;
        return g;
    }
}
