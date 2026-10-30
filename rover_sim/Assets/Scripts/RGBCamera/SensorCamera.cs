using System;
using System.Buffers;
using System.Collections.Generic;
using System.Threading.Tasks;
using SimFlake.Ros;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Shared plumbing for camera sensors: off-screen rendering (one render per camera/resolution per frame,
/// even when several publishers share a camera), async GPU readback, row flipping and CameraInfo.
/// </summary>
public static class SensorCamera
{
    class Target { public RenderTexture rt; public int frame = -1; }
    static readonly Dictionary<(int, int, int), Target> targets = new Dictionary<(int, int, int), Target>();

    /// <summary>Renders <paramref name="cam"/> into a cached RenderTexture of the given size (once per frame).</summary>
    public static RenderTexture Render(Camera cam, int width, int height)
    {
        var key = (cam.GetInstanceID(), width, height);
        if (!targets.TryGetValue(key, out var t) || t.rt == null)
        {
            t = new Target { rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32) { name = cam.name + "_sensor" } };
            t.rt.Create();
            targets[key] = t;
        }
        if (t.frame != Time.frameCount)
        {
            var prev = cam.targetTexture;
            cam.targetTexture = t.rt;
            cam.Render();
            cam.targetTexture = prev;
            t.frame = Time.frameCount;
        }
        return t.rt;
    }

    /// <summary>
    /// True when AsyncGPUReadback of a camera RenderTexture comes back top row first (ROS order).
    /// Verified on OpenGL Core, which Unity uses on Linux: readback is top-down there.
    /// APIs whose UV origin is at the top (Vulkan/D3D/Metal) are assumed to be bottom-up. If images come out
    /// upside down on such an API, flip this.
    /// </summary>
    public static bool ReadbackIsTopDown => !SystemInfo.graphicsUVStartsAtTop;

    /// <summary>
    /// Copies a GPU readback into a pooled buffer, top row first (what ROS expects) when <paramref name="topDown"/>,
    /// bottom row first (what Unity's ImageConversion expects) otherwise.
    /// Return the buffer with ArrayPool&lt;byte&gt;.Shared.Return when done.
    /// </summary>
    public static byte[] CopyRows(NativeArray<byte> src, int width, int height, int bytesPerPixel, bool topDown)
    {
        int row = width * bytesPerPixel;
        var dst = ArrayPool<byte>.Shared.Rent(row * height);
        if (topDown == ReadbackIsTopDown)
        {
            NativeArray<byte>.Copy(src, 0, dst, 0, row * height);
        }
        else
        {
            for (int y = 0; y < height; y++)
                NativeArray<byte>.Copy(src, (height - 1 - y) * row, dst, y * row, row);
        }
        return dst;
    }

    /// <summary>Pinhole intrinsics matching the Unity camera's field of view.</summary>
    public static sensor_msgs.msg.CameraInfo Info(Camera cam, int width, int height, std_msgs.msg.Header header)
    {
        double fy = height / 2.0 / Math.Tan(cam.fieldOfView * Mathf.Deg2Rad / 2.0);
        double fx = fy; // square pixels
        double cx = width / 2.0, cy = height / 2.0;
        var info = new sensor_msgs.msg.CameraInfo { header = header, width = (uint)width, height = (uint)height };
        info.k[0] = fx; info.k[2] = cx; info.k[4] = fy; info.k[5] = cy; info.k[8] = 1;
        info.r[0] = info.r[4] = info.r[8] = 1;
        info.p[0] = fx; info.p[2] = cx; info.p[5] = fy; info.p[6] = cy; info.p[10] = 1;
        return info;
    }

    /// <summary>Run on the thread pool, logging (not swallowing) exceptions.</summary>
    public static void Background(Action work)
    {
        Task.Run(() =>
        {
            try { work(); }
            catch (Exception e) { Debug.LogException(e); }
        });
    }
}
