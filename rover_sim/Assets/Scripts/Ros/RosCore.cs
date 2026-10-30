using System;
using System.Collections.Generic;
using System.Text;

namespace SimFlake.Ros
{
    /// <summary>Base class for every ROS message. Subclasses carry a [RosMsg("pkg/msg/Type")] attribute.</summary>
    [Serializable]
    public abstract class RosMessage { }

    [AttributeUsage(AttributeTargets.Class, Inherited = false)]
    public sealed class RosMsgAttribute : Attribute
    {
        public readonly string Type;
        public RosMsgAttribute(string type) { Type = type; }
    }

    /// <summary>
    /// A message with one top-level uint8[] field that is too big to go through JsonUtility.
    /// The bytes get base64-encoded straight into the outgoing buffer.
    /// </summary>
    public interface IBinaryMessage
    {
        string BinaryFieldName { get; }
        ArraySegment<byte> BinaryData { get; }
    }

    public static class RosType<T> where T : RosMessage
    {
        public static readonly string Name = Lookup();

        static string Lookup()
        {
            var attrs = typeof(T).GetCustomAttributes(typeof(RosMsgAttribute), false);
            if (attrs.Length == 0)
                throw new InvalidOperationException($"{typeof(T).FullName} is missing a [RosMsg(\"pkg/msg/Type\")] attribute");
            return ((RosMsgAttribute)attrs[0]).Type;
        }
    }

    /// <summary>ROS timestamps. Uses wall-clock (system) time so stamps line up with other nodes on the machine.</summary>
    public static class RosTime
    {
        static readonly DateTime Epoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

        public static builtin_interfaces.msg.Time Now()
        {
            long ticks = (DateTime.UtcNow - Epoch).Ticks; // 100 ns ticks
            return new builtin_interfaces.msg.Time
            {
                sec = (int)(ticks / TimeSpan.TicksPerSecond),
                nanosec = (uint)(ticks % TimeSpan.TicksPerSecond * 100),
            };
        }

        public static std_msgs.msg.Header Header(string frameId) =>
            new std_msgs.msg.Header { stamp = Now(), frame_id = frameId ?? "" };
    }

    /// <summary>Tiny JSON helpers for the rosbridge envelope. Message bodies go through UnityEngine.JsonUtility.</summary>
    public static class RosJson
    {
        public static string Quote(string s)
        {
            if (s == null) return "\"\"";
            var sb = new StringBuilder(s.Length + 2);
            sb.Append('"');
            foreach (char c in s)
            {
                switch (c)
                {
                    case '"': sb.Append("\\\""); break;
                    case '\\': sb.Append("\\\\"); break;
                    case '\n': sb.Append("\\n"); break;
                    case '\r': sb.Append("\\r"); break;
                    case '\t': sb.Append("\\t"); break;
                    default:
                        if (c < 0x20) sb.Append("\\u").Append(((int)c).ToString("x4"));
                        else sb.Append(c);
                        break;
                }
            }
            sb.Append('"');
            return sb.ToString();
        }

        /// <summary>Decodes a quoted JSON string literal (handles \/ which rosbridge's ujson emits, \uXXXX, etc.).</summary>
        public static string Unquote(string raw)
        {
            if (raw == null || raw.Length < 2 || raw[0] != '"') return null;
            if (raw.IndexOf('\\') < 0) return raw.Substring(1, raw.Length - 2);
            var sb = new StringBuilder(raw.Length);
            for (int i = 1; i < raw.Length - 1; i++)
            {
                char c = raw[i];
                if (c != '\\' || i + 1 >= raw.Length - 1) { sb.Append(c); continue; }
                char e = raw[++i];
                switch (e)
                {
                    case 'n': sb.Append('\n'); break;
                    case 'r': sb.Append('\r'); break;
                    case 't': sb.Append('\t'); break;
                    case 'b': sb.Append('\b'); break;
                    case 'f': sb.Append('\f'); break;
                    case 'u':
                        if (i + 4 < raw.Length) { sb.Append((char)Convert.ToInt32(raw.Substring(i + 1, 4), 16)); i += 4; }
                        break;
                    default: sb.Append(e); break; // \" \\ \/
                }
            }
            return sb.ToString();
        }

        /// <summary>
        /// Returns the raw JSON text of a top-level object/array field (e.g. "msg"), or null.
        /// Scans only the outer object and respects strings, so it works whatever the key order is.
        /// </summary>
        public static string ExtractField(string json, string key)
        {
            int depth = 0;
            int i = 0, n = json.Length;
            while (i < n)
            {
                char c = json[i];
                if (c == '"')
                {
                    int start = i;
                    i = SkipString(json, i);
                    if (depth == 1)
                    {
                        // Is this string a key equal to `key`?
                        int j = i;
                        while (j < n && char.IsWhiteSpace(json[j])) j++;
                        if (j < n && json[j] == ':' && i - start - 2 == key.Length &&
                            string.CompareOrdinal(json, start + 1, key, 0, key.Length) == 0)
                        {
                            j++;
                            while (j < n && char.IsWhiteSpace(json[j])) j++;
                            int end = SkipValue(json, j);
                            return json.Substring(j, end - j);
                        }
                    }
                    continue;
                }
                if (c == '{' || c == '[') depth++;
                else if (c == '}' || c == ']') depth--;
                i++;
            }
            return null;
        }

        // i points at the opening quote; returns index just past the closing quote.
        static int SkipString(string s, int i)
        {
            i++;
            while (i < s.Length)
            {
                if (s[i] == '\\') { i += 2; continue; }
                if (s[i] == '"') return i + 1;
                i++;
            }
            return i;
        }

        static int SkipValue(string s, int i)
        {
            if (i >= s.Length) return i;
            char c = s[i];
            if (c == '"') return SkipString(s, i);
            if (c == '{' || c == '[')
            {
                int depth = 0;
                while (i < s.Length)
                {
                    char ch = s[i];
                    if (ch == '"') { i = SkipString(s, i); continue; }
                    if (ch == '{' || ch == '[') depth++;
                    else if (ch == '}' || ch == ']') { depth--; if (depth == 0) return i + 1; }
                    i++;
                }
                return i;
            }
            // number / true / false / null
            while (i < s.Length && s[i] != ',' && s[i] != '}' && s[i] != ']') i++;
            return i;
        }
    }

    /// <summary>
    /// Coordinate conventions.
    /// Unity world: x = east, y = up, z = north (left-handed).
    /// ROS world (REP-103): ENU, x = east, y = north, z = up.
    /// Unity body: x = right, y = up, z = forward. ROS body (REP-103): FLU, x = forward, y = left, z = up.
    /// </summary>
    public static class RosFrames
    {
        /// <summary>Unity world vector/point -> ROS ENU.</summary>
        public static UnityEngine.Vector3 UnityToEnu(UnityEngine.Vector3 v) => new UnityEngine.Vector3(v.x, v.z, v.y);

        /// <summary>Unity body-local vector -> ROS FLU body vector.</summary>
        public static UnityEngine.Vector3 UnityToFlu(UnityEngine.Vector3 v) => new UnityEngine.Vector3(v.z, -v.x, v.y);

        /// <summary>
        /// Orientation of a Unity transform (rotation from body to world) expressed as a ROS
        /// quaternion rotating the FLU body frame into the ENU world frame.
        /// </summary>
        public static geometry_msgs.msg.Quaternion UnityRotationToEnu(UnityEngine.Quaternion q)
        {
            // Columns of the ROS rotation matrix are the FLU body axes expressed in ENU.
            var fwd = UnityToEnu(q * UnityEngine.Vector3.forward); // body +x (forward)
            var left = UnityToEnu(q * UnityEngine.Vector3.left);   // body +y (left)
            var up = UnityToEnu(q * UnityEngine.Vector3.up);       // body +z (up)
            return FromMatrix(fwd.x, left.x, up.x,
                              fwd.y, left.y, up.y,
                              fwd.z, left.z, up.z);
        }

        /// <summary>Yaw-only quaternion around +z (ROS convention).</summary>
        public static geometry_msgs.msg.Quaternion Yaw(double yaw) =>
            new geometry_msgs.msg.Quaternion(0, 0, Math.Sin(yaw / 2), Math.Cos(yaw / 2));

        static geometry_msgs.msg.Quaternion FromMatrix(double m00, double m01, double m02,
                                                       double m10, double m11, double m12,
                                                       double m20, double m21, double m22)
        {
            double tr = m00 + m11 + m22;
            double x, y, z, w;
            if (tr > 0)
            {
                double s = Math.Sqrt(tr + 1.0) * 2;
                w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s;
            }
            else if (m00 > m11 && m00 > m22)
            {
                double s = Math.Sqrt(1.0 + m00 - m11 - m22) * 2;
                w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s;
            }
            else if (m11 > m22)
            {
                double s = Math.Sqrt(1.0 + m11 - m00 - m22) * 2;
                w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s;
            }
            else
            {
                double s = Math.Sqrt(1.0 + m22 - m00 - m11) * 2;
                w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s;
            }
            double norm = Math.Sqrt(x * x + y * y + z * z + w * w);
            return new geometry_msgs.msg.Quaternion(x / norm, y / norm, z / norm, w / norm);
        }

        public static geometry_msgs.msg.Vector3 ToMsg(UnityEngine.Vector3 v) => new geometry_msgs.msg.Vector3(v.x, v.y, v.z);
        public static geometry_msgs.msg.Point ToPoint(UnityEngine.Vector3 v) => new geometry_msgs.msg.Point(v.x, v.y, v.z);
    }
}
