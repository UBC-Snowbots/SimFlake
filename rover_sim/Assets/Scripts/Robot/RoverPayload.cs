using UnityEngine;

/// <summary>
/// What the rover carries on its deck: nothing, the arm, or (later) the rover lab. On the rover's root object, with a
/// mount point (<see cref="armMount"/>, a child of the chassis body; move it in the prefab to move the arm).
///
/// The arm is instantiated under the mount and joins the rover's articulation (fixed joint at the mount), so its
/// weight and reaction forces act on the chassis. Change it at runtime with <see cref="Apply"/> (Esc menu > WORLD >
/// ROVER > CONFIGURATION), or in a player with <c>--rover-config arm|none</c>.
/// </summary>
public class RoverPayload : MonoBehaviour
{
    public enum Configuration { None, Arm, RoverLab }

    public static readonly string[] Labels = { "NONE", "ARM", "ROVER LAB" };

    [Tooltip("What is mounted when the scene starts.")]
    public Configuration configuration = Configuration.None;
    [Tooltip("Child of the chassis body where the arm's base (link_0) goes.")]
    public Transform armMount;
    public GameObject armPrefab;

    /// <summary>Configurations this build can simulate at all (the rover lab isn't modelled yet).</summary>
    public static bool Implemented(Configuration c) => c != Configuration.RoverLab;

    public bool Supports(Configuration c) =>
        c == Configuration.None || (c == Configuration.Arm && armPrefab != null && armMount != null);

    public ArmBreakage Arm => armMount != null ? armMount.GetComponentInChildren<ArmBreakage>() : null;

    void Awake()
    {
        var arg = Arg("--rover-config");
        if (arg != null && System.Enum.TryParse(arg.Replace("-", "").Replace("_", ""), true, out Configuration c)) configuration = c;
        // A copy of a rover that already carries its payload (RoverSwap's template) keeps it.
        bool has = armMount != null && armMount.GetComponentInChildren<ArmController>(true) != null;
        if (configuration == Configuration.Arm && has) return;
        Apply(configuration);
    }

    static string Arg(string n)
    {
        var a = System.Environment.GetCommandLineArgs();
        int i = System.Array.IndexOf(a, n);
        return i >= 0 && i + 1 < a.Length ? a[i + 1] : null;
    }

    /// <summary>Mount <paramref name="c"/> (removing what was there). Returns false if this rover can't carry it.</summary>
    public bool Apply(Configuration c)
    {
        if (!Implemented(c) || !Supports(c))
        {
            if (c != Configuration.None) Debug.LogWarning($"[RoverPayload] {name} can't carry {c}");
            return false;
        }
        Clear();
        configuration = c;
        if (c != Configuration.Arm) return true;

        // Build it inactive, so it joins the rover's articulation only once it is in place.
        var staging = new GameObject("payload staging");
        staging.SetActive(false);
        var arm = Instantiate(armPrefab, staging.transform);
        arm.name = armPrefab.name;
        arm.SetActive(false);
        foreach (var b in arm.GetComponentsInChildren<ArticulationBody>(true)) b.immovable = false;
        arm.transform.SetParent(armMount, false);
        arm.transform.localPosition = Vector3.zero;
        arm.transform.localRotation = Quaternion.identity;
        arm.SetActive(true);
        Destroy(staging);
        Debug.Log($"[RoverPayload] {name}: arm mounted");
        return true;
    }

    void Clear()
    {
        if (armMount == null) return;
        for (int i = armMount.childCount - 1; i >= 0; i--)
        {
            var child = armMount.GetChild(i).gameObject;
            child.SetActive(false);   // out of the articulation now, not at the end of the frame
            Destroy(child);
        }
    }
}
