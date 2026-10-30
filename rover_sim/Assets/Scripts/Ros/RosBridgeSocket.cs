using System;
using System.Buffers;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SimFlake.Ros
{
    /// <summary>
    /// One WebSocket connection to rosbridge. Runs entirely on background tasks:
    /// connects (and reconnects) on its own, sends queued frames, and hands received
    /// text frames to <see cref="OnText"/> (called on a background thread).
    /// </summary>
    internal sealed class RosBridgeSocket : IDisposable
    {
        internal struct Frame
        {
            public byte[] Buffer;   // rented from ArrayPool when Pooled
            public int Length;
            public bool Pooled;
            public PendingCounter Counter; // may be null
        }

        /// <summary>Counts frames queued for one publisher so it can drop when it falls behind.</summary>
        internal sealed class PendingCounter { public int Value; }

        readonly Uri uri;
        readonly string name;
        readonly ConcurrentQueue<Frame> queue = new ConcurrentQueue<Frame>();
        readonly SemaphoreSlim signal = new SemaphoreSlim(0);
        readonly CancellationTokenSource cts = new CancellationTokenSource();

        /// <summary>Returns the control ops (advertise/subscribe) to send right after connecting.</summary>
        public Func<List<string>> Handshake;
        public Action<string> OnText;
        public Action<string> Log;

        volatile bool connected;
        public bool Connected => connected;
        public int ConnectAttempts { get; private set; }
        /// <summary>Lock shared with the owner so registration and handshake can't interleave.</summary>
        public readonly object HandshakeLock = new object();

        public RosBridgeSocket(Uri uri, string name)
        {
            this.uri = uri;
            this.name = name;
        }

        public void Start() => Task.Run(RunAsync);

        public void Dispose()
        {
            cts.Cancel();
            signal.Release();
        }

        /// <summary>Queue a text frame. Returns false (and drops it) when not connected.</summary>
        public bool Enqueue(Frame f)
        {
            if (!connected) { Release(f); return false; }
            if (f.Counter != null) Interlocked.Increment(ref f.Counter.Value);
            queue.Enqueue(f);
            signal.Release();
            return true;
        }

        public bool EnqueueText(string text)
        {
            var bytes = Encoding.UTF8.GetBytes(text);
            return Enqueue(new Frame { Buffer = bytes, Length = bytes.Length });
        }

        static void Release(Frame f)
        {
            if (f.Counter != null) Interlocked.Decrement(ref f.Counter.Value);
            if (f.Pooled && f.Buffer != null) ArrayPool<byte>.Shared.Return(f.Buffer);
        }

        void DrainQueue()
        {
            while (queue.TryDequeue(out var f))
            {
                if (f.Counter != null) Interlocked.Decrement(ref f.Counter.Value);
                if (f.Pooled && f.Buffer != null) ArrayPool<byte>.Shared.Return(f.Buffer);
            }
        }

        async Task RunAsync()
        {
            var token = cts.Token;
            bool loggedFailure = false;
            while (!token.IsCancellationRequested)
            {
                using (var ws = new ClientWebSocket())
                {
                    ws.Options.KeepAliveInterval = TimeSpan.FromSeconds(10);
                    try
                    {
                        ConnectAttempts++;
                        using (var timeout = CancellationTokenSource.CreateLinkedTokenSource(token))
                        {
                            timeout.CancelAfter(TimeSpan.FromSeconds(3));
                            await ws.ConnectAsync(uri, timeout.Token).ConfigureAwait(false);
                        }
                    }
                    catch (Exception e)
                    {
                        if (!loggedFailure && !token.IsCancellationRequested)
                        {
                            Log?.Invoke($"[{name}] waiting for rosbridge at {uri} ({e.GetBaseException().Message}). Will keep retrying.");
                            loggedFailure = true;
                        }
                        try { await Task.Delay(1000, token).ConfigureAwait(false); } catch { }
                        continue;
                    }

                    loggedFailure = false;
                    List<string> hello;
                    lock (HandshakeLock)
                    {
                        DrainQueue();
                        hello = Handshake?.Invoke() ?? new List<string>();
                        connected = true;
                    }
                    Log?.Invoke($"[{name}] connected to rosbridge at {uri}");

                    try
                    {
                        foreach (var op in hello)
                        {
                            var b = Encoding.UTF8.GetBytes(op);
                            await ws.SendAsync(new ArraySegment<byte>(b), WebSocketMessageType.Text, true, token).ConfigureAwait(false);
                        }
                        var recv = ReceiveLoop(ws, token);
                        var send = SendLoop(ws, token);
                        await Task.WhenAny(recv, send).ConfigureAwait(false);
                    }
                    catch (Exception e)
                    {
                        if (!token.IsCancellationRequested)
                            Log?.Invoke($"[{name}] connection error: {e.GetBaseException().Message}");
                    }
                    finally
                    {
                        connected = false;
                        DrainQueue();
                        try { ws.Abort(); } catch { }
                    }
                    if (!token.IsCancellationRequested)
                        Log?.Invoke($"[{name}] disconnected from rosbridge, reconnecting...");
                }
                try { await Task.Delay(500, token).ConfigureAwait(false); } catch { }
            }
            DrainQueue();
        }

        async Task SendLoop(ClientWebSocket ws, CancellationToken token)
        {
            while (!token.IsCancellationRequested && ws.State == WebSocketState.Open)
            {
                await signal.WaitAsync(token).ConfigureAwait(false);
                while (queue.TryDequeue(out var f))
                {
                    try
                    {
                        await ws.SendAsync(new ArraySegment<byte>(f.Buffer, 0, f.Length),
                                           WebSocketMessageType.Text, true, token).ConfigureAwait(false);
                    }
                    finally { Release(f); }
                }
            }
        }

        async Task ReceiveLoop(ClientWebSocket ws, CancellationToken token)
        {
            var buf = new byte[64 * 1024];
            int len = 0;
            while (!token.IsCancellationRequested && ws.State == WebSocketState.Open)
            {
                if (len == buf.Length) Array.Resize(ref buf, buf.Length * 2);
                var r = await ws.ReceiveAsync(new ArraySegment<byte>(buf, len, buf.Length - len), token).ConfigureAwait(false);
                if (r.MessageType == WebSocketMessageType.Close) return;
                len += r.Count;
                if (!r.EndOfMessage) continue;
                if (r.MessageType == WebSocketMessageType.Text)
                {
                    string text = Encoding.UTF8.GetString(buf, 0, len);
                    try { OnText?.Invoke(text); }
                    catch (Exception e) { Log?.Invoke($"[{name}] error handling message: {e}"); }
                }
                len = 0;
            }
        }
    }
}
