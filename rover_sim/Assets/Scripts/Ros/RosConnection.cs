using System;
using System.Buffers;
using System.Buffers.Text;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using UnityEngine;

namespace SimFlake.Ros
{
    /// <summary>
    /// Shortcut API. Typical use inside a MonoBehaviour:
    /// <code>
    ///   var pub = Ros.Advertise&lt;sensor_msgs.msg.Imu&gt;("/imu/data");
    ///   pub.Publish(msg);                         // any thread
    ///   Ros.Subscribe&lt;geometry_msgs.msg.Twist&gt;("/cmd_vel", OnCmdVel);   // callback runs on the main thread
    /// </code>
    /// </summary>
    public static class Ros
    {
        public static RosPublisher<T> Advertise<T>(string topic, int queueSize = 10, bool dedicatedSocket = false) where T : RosMessage
            => RosConnection.Instance.Advertise<T>(topic, queueSize, dedicatedSocket);

        public static RosSubscription Subscribe<T>(string topic, Action<T> callback, int queueLength = 10, int throttleMs = 0) where T : RosMessage
            => RosConnection.Instance.Subscribe(topic, callback, queueLength, throttleMs);

        public static bool Connected => RosConnection.HasInstance && RosConnection.Instance.IsConnected;
    }

    public sealed class RosPublisher<T> where T : RosMessage
    {
        public readonly string Topic;
        public readonly int QueueSize;
        internal readonly RosBridgeSocket Socket;
        internal readonly RosBridgeSocket.PendingCounter Pending = new RosBridgeSocket.PendingCounter();
        readonly byte[] prefix;

        internal RosPublisher(string topic, int queueSize, RosBridgeSocket socket)
        {
            Topic = topic;
            QueueSize = Math.Max(1, queueSize);
            Socket = socket;
            prefix = Encoding.UTF8.GetBytes("{\"op\":\"publish\",\"topic\":" + RosJson.Quote(topic) + ",\"msg\":");
        }

        internal string AdvertiseOp() =>
            "{\"op\":\"advertise\",\"topic\":" + RosJson.Quote(Topic) + ",\"type\":" + RosJson.Quote(RosType<T>.Name) +
            ",\"queue_size\":" + QueueSize + "}";

        /// <summary>True when the frame was queued, false when dropped (not connected, or the publisher is behind).</summary>
        public bool Publish(T msg)
        {
            if (!Socket.Connected) return false;
            // Drop instead of building up latency when the link can't keep up.
            if (Volatile.Read(ref Pending.Value) >= QueueSize) return false;

            string body = JsonUtility.ToJson(msg);
            byte[] buf;
            int len;
            if (msg is IBinaryMessage bin)
            {
                // body is "{...}" without the binary field: splice `"data":"<base64>"` in before the closing brace.
                var data = bin.BinaryData;
                int bodyLen = body.Length - 1; // drop trailing '}'
                bool hasFields = body.Length > 2;
                string fieldHead = (hasFields ? "," : "") + "\"" + bin.BinaryFieldName + "\":\"";
                int b64Len = Base64.GetMaxEncodedToUtf8Length(data.Count);
                int max = prefix.Length + Encoding.UTF8.GetMaxByteCount(bodyLen) + fieldHead.Length + b64Len + 3;
                buf = ArrayPool<byte>.Shared.Rent(max);
                len = 0;
                Buffer.BlockCopy(prefix, 0, buf, 0, prefix.Length); len += prefix.Length;
                len += Encoding.UTF8.GetBytes(body, 0, bodyLen, buf, len);
                len += Encoding.ASCII.GetBytes(fieldHead, 0, fieldHead.Length, buf, len);
                Base64.EncodeToUtf8(new ReadOnlySpan<byte>(data.Array, data.Offset, data.Count),
                                    new Span<byte>(buf, len, b64Len), out _, out int written);
                len += written;
                buf[len++] = (byte)'"';
                buf[len++] = (byte)'}';
                buf[len++] = (byte)'}';
            }
            else
            {
                int max = prefix.Length + Encoding.UTF8.GetMaxByteCount(body.Length) + 1;
                buf = ArrayPool<byte>.Shared.Rent(max);
                Buffer.BlockCopy(prefix, 0, buf, 0, prefix.Length);
                len = prefix.Length + Encoding.UTF8.GetBytes(body, 0, body.Length, buf, prefix.Length);
                buf[len++] = (byte)'}';
            }
            return Socket.Enqueue(new RosBridgeSocket.Frame { Buffer = buf, Length = len, Pooled = true, Counter = Pending });
        }
    }

    public abstract class RosSubscription
    {
        public readonly string Topic;
        internal RosSubscription(string topic) { Topic = topic; }
        internal abstract string SubscribeOp();
        internal abstract void HandleBackground(string fullJson, ConcurrentQueue<Action> mainThread);
        public bool Active { get; internal set; } = true;
        public void Unsubscribe()
        {
            Active = false;
            if (RosConnection.HasInstance) RosConnection.Instance.Unsubscribe(this);
        }
    }

    internal sealed class RosSubscription<T> : RosSubscription where T : RosMessage
    {
        readonly Action<T> callback;
        readonly int queueLength, throttleMs;
        bool gotFirst;

        public RosSubscription(string topic, Action<T> cb, int queueLength, int throttleMs) : base(topic)
        {
            callback = cb;
            this.queueLength = queueLength;
            this.throttleMs = throttleMs;
        }

        internal override string SubscribeOp() =>
            "{\"op\":\"subscribe\",\"topic\":" + RosJson.Quote(Topic) + ",\"type\":" + RosJson.Quote(RosType<T>.Name) +
            ",\"queue_length\":" + queueLength + ",\"throttle_rate\":" + throttleMs + "}";

        internal override void HandleBackground(string fullJson, ConcurrentQueue<Action> mainThread)
        {
            string body = RosJson.ExtractField(fullJson, "msg");
            if (body == null) return;
            T msg = JsonUtility.FromJson<T>(body); // JsonUtility is safe to use off the main thread
            if (!gotFirst) { gotFirst = true; Debug.Log($"[ROS] first message on {Topic}"); }
            mainThread.Enqueue(() => { if (Active) callback(msg); });
        }
    }

    /// <summary>
    /// Owns the rosbridge connection(s). One is created automatically the first time a script
    /// uses <see cref="Ros"/>; drop this component into a scene only if you want to change the settings.
    /// </summary>
    [DefaultExecutionOrder(-1000)]
    public sealed class RosConnection : MonoBehaviour
    {
        [Tooltip("rosbridge WebSocket URL. Override with env SIMFLAKE_ROSBRIDGE_URL or the --rosbridge-url <url> command-line flag.")]
        public string rosbridgeUrl = "ws://127.0.0.1:9090";

        [Tooltip("If nothing is listening at the URL above (and it is localhost), start rosbridge automatically using your ~/.bashrc ROS environment. It is stopped again when the sim stops.")]
        public bool autoLaunchRosbridge = true;

        [Tooltip("Show a small connection indicator in the corner of the Game view.")]
        public bool showStatusOverlay = true;

        static RosConnection instance;
        static bool quitting;
        public static bool HasInstance => instance != null;

        public static RosConnection Instance
        {
            get
            {
                if (instance != null) return instance;
                if (quitting) throw new InvalidOperationException("ROS connection is shutting down");
                instance = FindObjectOfType<RosConnection>();
                if (instance == null)
                    instance = new GameObject("ROS Connection (auto)").AddComponent<RosConnection>();
                instance.Init();
                return instance;
            }
        }

        readonly object registryLock = new object();
        readonly ConcurrentQueue<Action> mainThread = new ConcurrentQueue<Action>();
        readonly Dictionary<string, List<RosSubscription>> subsByTopic = new Dictionary<string, List<RosSubscription>>();
        readonly List<Func<string>> mainAdverts = new List<Func<string>>();
        readonly List<RosBridgeSocket> sockets = new List<RosBridgeSocket>();
        RosBridgeSocket main;
        Uri uri;
        bool initialized;
        float launchAfter = -1;

        public bool IsConnected => main != null && main.Connected;
        public string Url => uri?.ToString();

        void Awake()
        {
            if (instance != null && instance != this) { Destroy(gameObject); return; }
            instance = this;
            Init();
        }

        void Init()
        {
            if (initialized) return;
            initialized = true;
            quitting = false;
            if (transform.parent == null) DontDestroyOnLoad(gameObject);

            string url = Environment.GetEnvironmentVariable("SIMFLAKE_ROSBRIDGE_URL");
            var args = Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
                if (args[i] == "--rosbridge-url") url = args[i + 1];
            if (!string.IsNullOrEmpty(url)) rosbridgeUrl = url;
            uri = new Uri(rosbridgeUrl);

            main = NewSocket("rosbridge", MainHandshake);
            main.OnText = HandleIncoming;
            // Give an already-running rosbridge a moment to answer before launching our own.
            launchAfter = autoLaunchRosbridge && uri.IsLoopback ? Time.realtimeSinceStartup + 1.5f : -1;
        }

        RosBridgeSocket NewSocket(string name, Func<List<string>> handshake, bool start = true)
        {
            var s = new RosBridgeSocket(uri, name) { Log = m => Debug.Log("[ROS] " + m), Handshake = handshake };
            sockets.Add(s);
            if (start) s.Start();
            return s;
        }

        List<string> MainHandshake()
        {
            var ops = new List<string>();
            lock (registryLock)
            {
                foreach (var a in mainAdverts) ops.Add(a());
                foreach (var list in subsByTopic.Values)
                    foreach (var s in list) ops.Add(s.SubscribeOp());
            }
            return ops;
        }

        public RosPublisher<T> Advertise<T>(string topic, int queueSize, bool dedicatedSocket) where T : RosMessage
        {
            if (!dedicatedSocket)
            {
                var pub = new RosPublisher<T>(topic, queueSize, main);
                lock (main.HandshakeLock)
                {
                    lock (registryLock) mainAdverts.Add(pub.AdvertiseOp);
                    if (main.Connected) main.EnqueueText(pub.AdvertiseOp());
                }
                return pub;
            }
            // Big streams (camera images) get their own socket so they never delay small messages.
            var sock = NewSocket("rosbridge:" + topic, null, start: false);
            var dpub = new RosPublisher<T>(topic, queueSize, sock);
            sock.Handshake = () => new List<string> { dpub.AdvertiseOp() };
            sock.Start();
            return dpub;
        }

        public RosSubscription Subscribe<T>(string topic, Action<T> callback, int queueLength, int throttleMs) where T : RosMessage
        {
            var sub = new RosSubscription<T>(topic, callback, queueLength, throttleMs);
            lock (main.HandshakeLock)
            {
                lock (registryLock)
                {
                    if (!subsByTopic.TryGetValue(topic, out var list)) subsByTopic[topic] = list = new List<RosSubscription>();
                    list.Add(sub);
                }
                if (main.Connected) main.EnqueueText(sub.SubscribeOp());
            }
            return sub;
        }

        internal void Unsubscribe(RosSubscription sub)
        {
            sub.Active = false;
            lock (registryLock)
            {
                if (subsByTopic.TryGetValue(sub.Topic, out var list))
                {
                    list.Remove(sub);
                    if (list.Count == 0) subsByTopic.Remove(sub.Topic);
                }
            }
            // Leave the rosbridge-side subscription; it is cleaned up on disconnect and costs nothing.
        }

        [Serializable] class Envelope { public string op; public string topic; public string level; public string msg; }

        // Background thread.
        void HandleIncoming(string json)
        {
            string op = ReadStringField(json, "op");
            if (op == "publish")
            {
                string topic = ReadStringField(json, "topic");
                RosSubscription[] targets;
                lock (registryLock)
                {
                    if (topic == null || !subsByTopic.TryGetValue(topic, out var list)) return;
                    targets = list.ToArray();
                }
                foreach (var s in targets) s.HandleBackground(json, mainThread);
            }
            else if (op == "status")
            {
                var env = JsonUtility.FromJson<Envelope>(json);
                if (env.level == "error") Debug.LogError("[rosbridge] " + env.msg);
                else if (env.level == "warning") Debug.LogWarning("[rosbridge] " + env.msg);
            }
        }

        static string ReadStringField(string json, string key)
        {
            return RosJson.Unquote(RosJson.ExtractField(json, key));
        }

        void Update()
        {
            while (mainThread.TryDequeue(out var a))
            {
                try { a(); }
                catch (Exception e) { Debug.LogException(e); }
            }

            if (launchAfter > 0 && Time.realtimeSinceStartup > launchAfter)
            {
                launchAfter = -1;
                if (!main.Connected) RosBridgeLauncher.Launch(uri.Port);
            }
        }

        void OnGUI()
        {
            if (!showStatusOverlay) return;
            string text;
            Color c;
            if (IsConnected) { text = "ROS: connected"; c = new Color(0.2f, 0.8f, 0.3f); }
            else if (RosBridgeLauncher.Running) { text = "ROS: starting rosbridge..."; c = new Color(0.95f, 0.75f, 0.2f); }
            else { text = $"ROS: waiting for rosbridge ({uri?.Host}:{uri?.Port})"; c = new Color(0.9f, 0.3f, 0.3f); }
            var prev = GUI.color;
            GUI.color = c;
            GUI.Label(new Rect(Screen.width - 330, 6, 324, 22), text, new GUIStyle(GUI.skin.label) { alignment = TextAnchor.UpperRight, fontStyle = FontStyle.Bold });
            GUI.color = prev;
        }

        void OnApplicationQuit() => Shutdown();
        void OnDestroy() { if (instance == this) Shutdown(); }

        void Shutdown()
        {
            if (!initialized) return;
            initialized = false;
            quitting = true;
            foreach (var s in sockets) s.Dispose();
            sockets.Clear();
            RosBridgeLauncher.Stop();
            if (instance == this) instance = null;
        }
    }
}
