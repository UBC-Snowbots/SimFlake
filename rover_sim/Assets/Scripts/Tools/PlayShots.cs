using System;
using System.Collections;
using System.IO;
using System.Linq;
using UnityEngine;

/// <summary>
/// Pictures from a running player, for checking how a world looks in Play (soil shader, ruts, sky) without opening the
/// editor. Works in any scene: nothing to add, it starts itself when the player is run with
///   rover_sim.x86_64 -batchmode --play-shots &lt;dir&gt; [--play-shots-drive 1.0] [--play-shots-seconds 12] [--play-shots-heading 135] [--play-shots-at x,z]
/// (-batchmode with graphics, not -nographics). It lets the rover settle, drives it straight at the given speed (m/s,
/// default 0.8) for the given time, logs how rough the ride was, saves a few views of the rover and its tracks as JPG, then quits.
/// </summary>
public class PlayShots : MonoBehaviour
{
    static string Arg(string n) { var a = Environment.GetCommandLineArgs(); int i = Array.IndexOf(a, n); return i >= 0 && i + 1 < a.Length ? a[i + 1] : null; }
    static float F(string n, float def) => float.TryParse(Arg(n), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var v) ? v : def;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Boot()
    {
        if (Arg("--play-shots") != null) new GameObject("Play Shots").AddComponent<PlayShots>();
    }

    Camera cam; RenderTexture rt; Texture2D tex; string dir;

    IEnumerator Start()
    {
        dir = Arg("--play-shots");
        Directory.CreateDirectory(dir);
        var mc = FindObjectOfType<MotorController>();
        if (mc == null) { Debug.LogError("[PlayShots] no rover in the scene"); Application.Quit(1); yield break; }
        var body = mc.GetComponentsInChildren<ArticulationBody>().First(b => b.isRoot).transform;
        // The moving body's own axes are the URDF's, not Unity's. The rover's root object was placed facing its
        // direction of travel and never moves, so the body's rotation relative to it at the start tells which way is forward.
        var bodyToHeading = Quaternion.Inverse(Quaternion.Inverse(mc.transform.root.rotation) * body.rotation);
        float Heading() => (body.rotation * bodyToHeading).eulerAngles.y;
        cam = new GameObject("Play Shots Camera").AddComponent<Camera>();
        cam.enabled = false; cam.nearClipPlane = 0.03f; cam.farClipPlane = 14000; cam.fieldOfView = 55;
        rt = new RenderTexture(1600, 900, 24) { antiAliasing = 4 };
        tex = new Texture2D(1600, 900, TextureFormat.RGB24, false);

        // optional: start somewhere else (world x,z). Moved before the rover has driven, which is when TeleportRoot is safe.
        if (Arg("--play-shots-at") != null)
        {
            var xz = Arg("--play-shots-at").Split(',');
            float x = float.Parse(xz[0], System.Globalization.CultureInfo.InvariantCulture), z = float.Parse(xz[1], System.Globalization.CultureInfo.InvariantCulture);
            float GroundY(Vector3 p) { foreach (var t in Terrain.activeTerrains) { var o = t.transform.position; var sz = t.terrainData.size; if (p.x >= o.x && p.z >= o.z && p.x <= o.x + sz.x && p.z <= o.z + sz.z) return t.SampleHeight(p) + o.y; } return 0; }
            var rootBody = body.GetComponent<ArticulationBody>();
            float clearance = body.position.y - GroundY(body.position);
            var to = new Vector3(x, 0, z); to.y = GroundY(to) + clearance + 0.15f;
            float yaw = Arg("--play-shots-heading") != null ? F("--play-shots-heading", 0) : Heading();
            rootBody.TeleportRoot(to, Quaternion.Euler(0, yaw, 0) * Quaternion.Inverse(bodyToHeading));
            rootBody.velocity = Vector3.zero; rootBody.angularVelocity = Vector3.zero;
        }
        yield return new WaitForSeconds(3f);
        yield return CloseUps(mc.transform.root, body);
        yield return Shot("1_start", body, new Vector3(-2.2f, 1.2f, -3.2f), new Vector3(0, 0.3f, 1f));
        float speed = F("--play-shots-drive", 0.8f), seconds = F("--play-shots-seconds", 12f);
        var cmd = new geometry_msgs.msg.Twist();
        cmd.linear.x = speed;
        // optional: turn on the spot to a compass heading first (0 = north, 90 = east), to leave a prepared start lane
        if (Arg("--play-shots-heading") != null)
        {
            float want = F("--play-shots-heading", 0);
            var turn = new geometry_msgs.msg.Twist();
            float turnFrom = Heading(), turnTime = 0;
            // the rover coasts on after the command stops: turn, stop, look again, finish slowly
            for (int pass = 0; pass < 5 && Mathf.Abs(Mathf.DeltaAngle(Heading(), want)) >= 2f; pass++)
            {
                for (float t = 0; t < 25f; t += Time.deltaTime)
                {
                    float err = Mathf.DeltaAngle(Heading(), want);
                    if (Mathf.Abs(err) < (pass == 0 ? 12f : 2f)) break;
                    turn.angular.z = (err > 0 ? -1 : 1) * (pass == 0 ? 0.6 : 0.25);      // ROS: positive = left = compass heading falling
                    mc.CmdVelCallback(turn);
                    turnTime += Time.deltaTime;
                    yield return null;
                }
                turn.angular.z = 0;
                for (float t = 0; t < 1.2f; t += Time.deltaTime) { mc.CmdVelCallback(turn); yield return null; }
            }
            Debug.Log($"[PlayShots] point turn from heading {turnFrom:F0} to {Heading():F0} (wanted {want:F0}) took {turnTime:F1} s");
            yield return new WaitForSeconds(1f);
        }
        // How hard the ground is to drive, measured on the body every physics step: speed made good, vertical
        // acceleration, and how fast the body is rocked (pitch + roll + yaw rate).
        float h0 = Heading();
        Vector3 p0 = body.position, last = p0, lastUp = body.up, lastFwd = body.forward;
        float vy = 0, sumA = 0, peakA = 0, sumW = 0, peakW = 0, path = 0; int n = 0;
        var wait = new WaitForFixedUpdate();
        for (float t = 0; t < seconds; t += Time.fixedDeltaTime)
        {
            mc.CmdVelCallback(cmd);
            yield return wait;
            float dt = Time.fixedDeltaTime;
            float v = (body.position.y - last.y) / dt, a = (v - vy) / dt;
            // rotation rate from how far the body's axes moved (Quaternion.Angle rounds small steps to zero)
            float w = Mathf.Sqrt(0.5f * ((body.up - lastUp).sqrMagnitude + (body.forward - lastFwd).sqrMagnitude + (Vector3.Cross(body.up, body.forward) - Vector3.Cross(lastUp, lastFwd)).sqrMagnitude)) / dt * Mathf.Rad2Deg;
            if (n > 0) { sumA += a * a; peakA = Mathf.Max(peakA, Mathf.Abs(a)); sumW += w * w; peakW = Mathf.Max(peakW, w); }
            path += Vector3.Distance(body.position, last);
            vy = v; last = body.position; lastUp = body.up; lastFwd = body.forward; n++;
        }
        var stones = FindObjectOfType<StoneField>();
        float made = Vector3.Distance(new Vector3(p0.x, 0, p0.z), new Vector3(body.position.x, 0, body.position.z));
        Debug.Log(string.Format(System.Globalization.CultureInfo.InvariantCulture,
            "[PlayShots] from ({11:F0}, {12:F0}) heading {13:F0} to ({14:F0}, {15:F0}) heading {16:F0}. Commanded {0:F2} m/s for {1:F0} s: made {2:F2} m/s good ({3:F1} m, path {4:F1} m), vertical accel RMS {5:F2} peak {6:F1} m/s2, body rocking RMS {7:F1} peak {8:F0} deg/s, climbed {9:+0.0;-0.0} m{10}",
            speed, seconds, made / seconds, made, path, Mathf.Sqrt(sumA / Mathf.Max(n - 1, 1)), peakA, Mathf.Sqrt(sumW / Mathf.Max(n - 1, 1)), peakW,
            body.position.y - p0.y, stones != null && stones.enabled ? $", {stones.Alive} stones alive" : ", no stones",
            p0.x, p0.z, h0, body.position.x, body.position.z, Heading()));
        yield return Shot("2_driving_tracks", body, new Vector3(-1.6f, 0.9f, -3.4f), new Vector3(0, 0.1f, -0.5f));
        yield return Shot("3_wheel", body, new Vector3(-1.1f, 0.25f, 0.3f), new Vector3(-0.3f, 0.1f, 0f));
        yield return Shot("3b_ground_ahead", body, new Vector3(0.9f, 0.45f, 0.6f), new Vector3(0.2f, 0f, 3.5f));
        yield return Shot("4_ahead", body, new Vector3(0f, 1.0f, 0.2f), new Vector3(0, 0.6f, 30f));
        yield return Shot("5_above", body, new Vector3(-6f, 9f, -9f), new Vector3(0, 0, 2f));
        Debug.Log("[PlayShots] done: " + dir);
        Application.Quit(0);
    }

    /// <summary>Close-ups of the rover itself (front-left, right side, above the back), framed on its renderers' bounds, so they
    /// work for any rover size and wherever its body's origin is.</summary>
    IEnumerator CloseUps(Transform rover, Transform body)
    {
        var rs = rover.GetComponentsInChildren<Renderer>().Where(r => r.enabled && r.GetComponent<TMPro.TextMeshPro>() == null).ToArray();
        if (rs.Length == 0) yield break;
        var b = rs[0].bounds; foreach (var r in rs) b.Encapsulate(r.bounds);
        var yaw = Quaternion.Euler(0, body.eulerAngles.y, 0);
        Vector3 c = Quaternion.Inverse(yaw) * (b.center - body.position);   // centre in the heading frame
        float k = Mathf.Max(b.extents.x, b.extents.z) * 2.2f;
        yield return Shot("0a_front_left", body, c + new Vector3(-0.8f, 0.55f, 0.9f) * k, c);
        yield return Shot("0b_right", body, c + new Vector3(1.25f, 0.2f, -0.1f) * k, c);
        yield return Shot("0c_above_back", body, c + new Vector3(-0.35f, 1.0f, -0.8f) * k, c);
    }

    /// <summary>View from a point given in the rover's heading frame (x right, y up, z forward), looking at another.</summary>
    IEnumerator Shot(string name, Transform body, Vector3 from, Vector3 look)
    {
        yield return new WaitForEndOfFrame();
        var yaw = Quaternion.Euler(0, body.eulerAngles.y, 0);
        cam.transform.position = body.position + yaw * from;
        cam.transform.LookAt(body.position + yaw * look);
        cam.targetTexture = rt;
        cam.Render();
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();
        RenderTexture.active = null;
        File.WriteAllBytes(Path.Combine(dir, name + ".jpg"), tex.EncodeToJPG(88));
    }
}
