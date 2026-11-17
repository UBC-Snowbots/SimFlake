using System;
using System.Collections;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEngine;

/// <summary>
/// Scripted soil demo for the Soil Yard: drives the rover down each lane, turns on the spot, tries the steep ramp, and
/// saves pictures of the tracks. Only runs when the player/editor is started with <c>--soil-demo &lt;folder&gt;</c>
/// (optional <c>--soil-demo-lanes 0,3</c>, <c>--soil-demo-size 1600x900</c>); otherwise it does nothing.
/// Works in a -batchmode player as long as there is a graphics device (no -nographics).
/// </summary>
public class SoilDemo : MonoBehaviour
{
    public float laneWidth = 12f, startZ = -14f, rampZ = 8f;
    public int lanes = 5;

    string dir;
    MotorController mc;
    GameObject template;
    float templateHeight;
    ArticulationBody root;
    Camera cam;
    RenderTexture rt;
    Texture2D tex;

    static string Arg(string n) { var a = Environment.GetCommandLineArgs(); int i = Array.IndexOf(a, n); return i >= 0 && i + 1 < a.Length ? a[i + 1] : null; }

    IEnumerator Start()
    {
        dir = Arg("--soil-demo");
        if (dir == null) yield break;
        Directory.CreateDirectory(dir);
        mc = FindObjectOfType<MotorController>();
        if (mc == null) { Debug.LogError("[SoilDemo] no rover in the scene"); Application.Quit(1); yield break; }
        root = mc.GetComponentsInChildren<ArticulationBody>().First(b => b.isRoot);
        // keep an untouched copy of the rover to spawn from
        var scene = mc.transform.root.gameObject;
        templateHeight = scene.transform.position.y - (Terrain.activeTerrain != null ? Terrain.activeTerrain.SampleHeight(scene.transform.position) + Terrain.activeTerrain.transform.position.y : 0);
        template = Instantiate(scene);
        template.SetActive(false);
        int w = 1600, h = 900;
        var size = Arg("--soil-demo-size")?.Split('x');
        if (size != null && size.Length == 2) { int.TryParse(size[0], out w); int.TryParse(size[1], out h); }
        cam = new GameObject("Soil Demo Camera").AddComponent<Camera>();
        cam.enabled = false; cam.nearClipPlane = 0.02f; cam.farClipPlane = 1500; cam.fieldOfView = 50;
        rt = new RenderTexture(w, h, 24) { antiAliasing = 4 };
        tex = new Texture2D(w, h, TextureFormat.RGB24, false);
        Time.timeScale = 3f;
        var which = Arg("--soil-demo-lanes")?.Split(',').Select(int.Parse).ToArray() ?? Enumerable.Range(0, lanes).ToArray();
        string[] names = { "sand", "hardpack", "bentonite", "gravel", "slickrock" };
        yield return new WaitForSeconds(1f);

        foreach (int i in which)
        {
            string n = i < names.Length ? names[i] : "lane" + i;
            float x = (i - (lanes - 1) * 0.5f) * laneWidth;
            yield return Teleport(new Vector3(x - 3f, 0, startZ));
            yield return Drive(0.4f, 0, 9f);
            Log(n + " straight");
            yield return Shot(n + "_1_tracks", new Vector3(-0.55f, 0.32f, -1.5f), new Vector3(0, 0.05f, 0.3f));
            yield return Shot(n + "_2_wheel", new Vector3(-0.42f, 0.10f, 0.12f), new Vector3(-0.15f, 0.04f, 0.0f));
            yield return Drive(0, 0.8f, 5f);
            Log(n + " turn");
            yield return Shot(n + "_3_turn", new Vector3(-0.2f, 1.3f, -1.0f), new Vector3(0, 0, 0));
            yield return Drive(0.4f, 0, 5f);
            yield return Shot(n + "_4_after_turn", new Vector3(0.7f, 0.45f, -1.6f), new Vector3(0, 0.05f, 0));
            // the 25 deg ramp (east half of the lane)
            yield return Teleport(new Vector3(x + 3f, 0, rampZ - 2.5f));
            yield return Drive(0.4f, 0, 16f);
            Log(n + " ramp25");
            yield return Shot(n + "_5_ramp25", new Vector3(-1.1f, 0.5f, -1.3f), new Vector3(0, 0.05f, 0.2f));
            yield return Shot(n + "_6_ramp25_wheel", new Vector3(-0.45f, 0.14f, -0.25f), new Vector3(-0.15f, 0.03f, -0.1f));
        }
        Debug.Log("[SoilDemo] done");
        Application.Quit(0);
    }

    // A fresh copy of the rover at p, facing +z (teleporting the driven one is not reliable: it can land tumbling).
    IEnumerator Teleport(Vector3 p)
    {
        float y = Terrain.activeTerrain != null ? Terrain.activeTerrain.SampleHeight(p) + Terrain.activeTerrain.transform.position.y : 0;
        if (mc != null) { mc.transform.root.gameObject.SetActive(false); Destroy(mc.transform.root.gameObject); }
        var go = Instantiate(template, new Vector3(p.x, y + templateHeight, p.z), Quaternion.identity);
        go.name = template.name.Replace("(Clone)", "");
        go.SetActive(true);
        mc = go.GetComponentInChildren<MotorController>();
        root = go.GetComponentsInChildren<ArticulationBody>().First(b => b.isRoot);
        yield return new WaitForSeconds(2f);
    }

    IEnumerator Drive(float v, float yaw, float seconds)
    {
        var tw = new geometry_msgs.msg.Twist();
        tw.linear.x = v; tw.angular.z = yaw;
        float t0 = Time.fixedTime;
        while (Time.fixedTime - t0 < seconds) { mc.CmdVelCallback(tw); yield return new WaitForFixedUpdate(); }
        tw.linear.x = 0; tw.angular.z = 0;
        t0 = Time.fixedTime;
        while (Time.fixedTime - t0 < 0.8f) { mc.CmdVelCallback(tw); yield return new WaitForFixedUpdate(); }
    }

    void Log(string what)
    {
        var ws = mc.GetComponentsInChildren<DriveWheel>();
        Debug.Log(string.Format(CultureInfo.InvariantCulture, "[SoilDemo] {0}: pos {1:F2} up {5:F2}, sink mm {2}, ground {3}, tiles {4}", what, root.transform.position,
            string.Join(" ", ws.Select(d => (d.Soil.sinkage * 1000).ToString("F0"))), ws[0].Soil.ground != null ? ws[0].Soil.ground.name : "-",
            SoilWorld.Instance != null && SoilWorld.Instance.Field != null ? SoilWorld.Instance.Field.TileCount : 0, root.transform.up.y));
    }

    // offset/look are in the rover's frame (x right, y up, z forward), relative to the rover root
    IEnumerator Shot(string name, Vector3 offset, Vector3 look)
    {
        yield return new WaitForEndOfFrame();
        var t = root.transform;
        Quaternion yawOnly = Quaternion.Euler(0, t.eulerAngles.y, 0);
        // relative to the ground under the rover, not the chassis height
        Vector3 basePos = t.position;
        float gy = SoilWorld.Instance != null ? SoilWorld.Instance.SurfaceHeight(basePos) : float.NaN;
        if (!float.IsNaN(gy)) basePos.y = gy;
        cam.transform.position = basePos + yawOnly * offset;
        cam.transform.LookAt(basePos + yawOnly * look);
        cam.targetTexture = rt;
        cam.Render();
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
        File.WriteAllBytes(Path.Combine(dir, name + ".png"), tex.EncodeToPNG());
    }
}
