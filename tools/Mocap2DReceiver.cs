// Mocap2DReceiver.cs — Unity 端接收上位机 UDP 推送的 2D 质心（M2D0 协议）。
// 用法：挂到任意 GameObject 上，运行后监听 9010 端口，收到的每台相机的点
// 存在 latestByCam 里。当前是骨架：上位机发的是 2D 像素点；后续上位机接入
// 三角化 + IK 后会改发 3D 位姿，届时替换 Parse 里的解析即可，网络层不变。
//
// 包格式（小端）：
//   "M2D0"(4) | camId u32 | ts_ns i64 | count u16 | count*(x f32, y f32)

using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class Mocap2DReceiver : MonoBehaviour
{
    public int port = 9010;

    public struct Frame { public long tsNs; public Vector2[] points; }
    // camId -> 最近一帧（多线程写，主线程读）
    public readonly Dictionary<uint, Frame> latestByCam = new Dictionary<uint, Frame>();

    private UdpClient _client;
    private Thread _thread;
    private volatile bool _running;
    private readonly object _lock = new object();

    void Start()
    {
        _client = new UdpClient(port);
        _running = true;
        _thread = new Thread(Loop) { IsBackground = true };
        _thread.Start();
        Debug.Log($"[Mocap2D] listening on :{port}");
    }

    void Loop()
    {
        var ep = new IPEndPoint(IPAddress.Any, port);
        while (_running)
        {
            try
            {
                byte[] data = _client.Receive(ref ep);
                if (data.Length < 18) continue;
                if (data[0] != (byte)'M' || data[1] != (byte)'2' ||
                    data[2] != (byte)'D' || data[3] != (byte)'0') continue;

                uint camId = BitConverter.ToUInt32(data, 4);
                long tsNs  = BitConverter.ToInt64(data, 8);
                ushort count = BitConverter.ToUInt16(data, 16);

                var pts = new Vector2[count];
                int off = 18;
                for (int i = 0; i < count && off + 8 <= data.Length; i++, off += 8)
                {
                    float x = BitConverter.ToSingle(data, off);
                    float y = BitConverter.ToSingle(data, off + 4);
                    pts[i] = new Vector2(x, y);
                }
                lock (_lock) { latestByCam[camId] = new Frame { tsNs = tsNs, points = pts }; }
            }
            catch (SocketException) { }
            catch (Exception e) { Debug.LogWarning("[Mocap2D] " + e.Message); }
        }
    }

    // 示例：主线程里读取（可在你自己的脚本里 foreach latestByCam）
    void Update()
    {
        // lock (_lock) { foreach (var kv in latestByCam) { ... 用 kv.Value.points ... } }
    }

    void OnDestroy()
    {
        _running = false;
        try { _client?.Close(); } catch { }
        try { _thread?.Join(200); } catch { }
    }
}
