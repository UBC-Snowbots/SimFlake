using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

/// <summary>
/// The rovers the Esc menu can swap between: <c>Resources/RoverCatalog.asset</c> (referencing the prefabs from
/// Resources is what puts them into every player build). A scene rover is matched to an entry by its wheel spec.
/// Add a rover by adding an entry in the inspector.
/// </summary>
public class RoverCatalog : ScriptableObject
{
    [Serializable]
    public class Entry
    {
        public string label;
        public GameObject prefab;
    }

    public List<Entry> rovers = new List<Entry>();

    static RoverCatalog cached;
    public static RoverCatalog Load() => cached != null ? cached : (cached = Resources.Load<RoverCatalog>("RoverCatalog"));

    /// <summary>Catalog index of a rover (its root object), or -1. Matched by the wheels' <see cref="WheelSpec"/>.</summary>
    public int IndexOf(GameObject rover)
    {
        var dw = rover != null ? rover.GetComponentInChildren<DriveWheel>(true) : null;
        if (dw == null || dw.wheel == null) return -1;
        for (int i = 0; i < rovers.Count; i++)
        {
            var p = rovers[i].prefab != null ? rovers[i].prefab.GetComponentInChildren<DriveWheel>(true) : null;
            if (p != null && p.wheel == dw.wheel) return i;
        }
        return -1;
    }
}

/// <summary>
/// Swaps the rover in the running scene for another kind, at the same place and heading. Uses the scene's own copy
/// of that rover when the scene has one (keeps its per-scene settings), else the catalog prefab. /gps/fix continues in
/// the old rover's frame; odometry starts again at zero.
/// </summary>
public static class RoverSwap
{
    static readonly Dictionary<int, GameObject> templates = new Dictionary<int, GameObject>();
    static GameObject holder;
    static Scene templateScene;

    /// <summary>Copy every rover of the scene before it moves (called when a scene loads, before physics runs).</summary>
    public static void Snapshot()
    {
        var scene = SceneManager.GetActiveScene();
        if (holder != null && templateScene == scene) return;
        templates.Clear();
        templateScene = scene;
        var cat = RoverCatalog.Load();
        if (cat == null) return;
        holder = new GameObject("Rover templates (SimMenu)");
        holder.SetActive(false);
        foreach (var mc in Resources.FindObjectsOfTypeAll<MotorController>())
        {
            if (!mc.gameObject.scene.isLoaded || mc.transform.IsChildOf(holder.transform)) continue;
            var root = mc.transform.root.gameObject;
            int k = cat.IndexOf(root);
            if (k < 0) continue;
            // An active rover beats an inactive spare of the same kind (it is the one the scene was set up around).
            if (templates.TryGetValue(k, out var have) && (have == null || !root.activeInHierarchy)) continue;
            if (have != null) Object.Destroy(have);
            var copy = Object.Instantiate(root, holder.transform);
            copy.name = root.name;
            templates[k] = copy;
        }
    }

    /// <summary>The rover's moving body (the root ArticulationBody; the prefab root itself does not move).</summary>
    public static Transform Body(GameObject rover)
    {
        foreach (var b in rover.GetComponentsInChildren<ArticulationBody>(true))
        {
            var p = b.transform.parent;
            if (p == null || p.GetComponentInParent<ArticulationBody>(true) == null) return b.transform;
        }
        return rover.transform;
    }

    /// <summary>Replace <paramref name="current"/> (a rover root) with catalog entry <paramref name="index"/>. Returns the new root.</summary>
    public static GameObject Swap(GameObject current, int index)
    {
        var cat = RoverCatalog.Load();
        if (cat == null || index < 0 || index >= cat.rovers.Count) return null;
        Snapshot();
        GameObject src = templates.TryGetValue(index, out var t) && t != null ? t : cat.rovers[index].prefab;
        if (src == null) return null;

        var oldBody = Body(current);
        Vector3 p = oldBody.position;
        float yaw = Yaw(oldBody.forward);

        // Build it under an inactive parent: nothing runs (Awake, articulation) until it is in place.
        var staging = new GameObject("rover swap");
        staging.SetActive(false);
        var rover = Object.Instantiate(src, staging.transform);
        rover.name = src.name;
        var body = Body(rover);
        rover.transform.rotation = Quaternion.Euler(0, yaw - Yaw(body.forward), 0) * rover.transform.rotation;
        rover.transform.position += new Vector3(p.x - body.position.x, 0, p.z - body.position.z);
        float ground = GroundBelow(p, current, out bool hit);
        if (!hit) ground = Lowest(current);
        rover.transform.position += Vector3.up * (ground + 0.05f - Lowest(rover));

        var oldGps = current.GetComponentsInChildren<LocationStuff>(true);
        foreach (var g in rover.GetComponentsInChildren<LocationStuff>(true))
            foreach (var o in oldGps)
                if (o.topic == g.topic) g.ContinueFrameOf(o);

        // The old rover goes first, so nothing (GameObject.Find, FindObjectsOfType) picks up its parts.
        current.SetActive(false);
        rover.transform.SetParent(null, true);
        if (current.scene.isLoaded) SceneManager.MoveGameObjectToScene(rover, current.scene);
        rover.SetActive(true);
        Object.Destroy(staging);

        var newBody = Body(rover);
        Retarget(current, rover, newBody);
        Object.Destroy(current);

        foreach (var sf in Object.FindObjectsOfType<StoneField>()) sf.FindRovers();
        foreach (var sr in Object.FindObjectsOfType<SoilRenderer>()) sr.target = newBody;
        Debug.Log($"[RoverSwap] {current.name} -> {rover.name} at ({p.x:0.0}, {p.z:0.0}), heading {yaw:0}°");
        return rover;
    }

    // Scene objects outside the rover that pointed into the old one: point them at the same-named part of the new one.
    static void Retarget(GameObject old, GameObject rover, Transform body)
    {
        Transform Map(Transform tr) => tr != null && tr.IsChildOf(old.transform) ? (Find(rover.transform, tr.name) ?? body) : tr;
        Camera MapCam(Camera c)
        {
            if (c == null || !c.transform.IsChildOf(old.transform)) return c;
            var f = Find(rover.transform, c.name);
            return f != null ? f.GetComponent<Camera>() : null;
        }
        foreach (var o in Object.FindObjectsOfType<CameraOrbit>()) o.target = Map(o.target);
        foreach (var o in Object.FindObjectsOfType<RGBCameraObject>()) o.target = Map(o.target);
        foreach (var o in Object.FindObjectsOfType<CameraSwapper>()) { o.cam1 = MapCam(o.cam1); o.cam2 = MapCam(o.cam2); }
    }

    static Transform Find(Transform root, string name)
    {
        foreach (var t in root.GetComponentsInChildren<Transform>(true)) if (t.name == name) return t;
        return null;
    }

    static float Yaw(Vector3 forward) => Mathf.Atan2(forward.x, forward.z) * Mathf.Rad2Deg;

    static float GroundBelow(Vector3 p, GameObject except, out bool hit)
    {
        hit = false;
        float best = float.NegativeInfinity;
        foreach (var h in Physics.RaycastAll(p + Vector3.up * 3f, Vector3.down, 20f, ~0, QueryTriggerInteraction.Ignore))
            if (!h.transform.IsChildOf(except.transform) && h.point.y > best) { best = h.point.y; hit = true; }
        return best;
    }

    // Lowest point of the wheels' meshes (works on inactive objects, unlike Renderer.bounds); all meshes if no wheels.
    static float Lowest(GameObject rover)
    {
        float min = float.PositiveInfinity;
        foreach (var w in rover.GetComponentsInChildren<DriveWheel>(true))
            foreach (var mf in w.GetComponentsInChildren<MeshFilter>(true)) min = Mathf.Min(min, Lowest(mf));
        if (float.IsInfinity(min))
            foreach (var mf in rover.GetComponentsInChildren<MeshFilter>(true)) min = Mathf.Min(min, Lowest(mf));
        return float.IsInfinity(min) ? Body(rover).position.y : min;
    }

    static float Lowest(MeshFilter mf)
    {
        if (mf.sharedMesh == null) return float.PositiveInfinity;
        var b = mf.sharedMesh.bounds;
        var m = mf.transform.localToWorldMatrix;
        float min = float.PositiveInfinity;
        for (int i = 0; i < 8; i++)
        {
            var c = new Vector3((i & 1) == 0 ? b.min.x : b.max.x, (i & 2) == 0 ? b.min.y : b.max.y, (i & 4) == 0 ? b.min.z : b.max.z);
            min = Mathf.Min(min, m.MultiplyPoint3x4(c).y);
        }
        return min;
    }
}
