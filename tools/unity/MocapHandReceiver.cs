// =============================================================================
// MocapHandReceiver.cs —— Unity 端接收器（关节角 M3DS + 分段四元数 M3DQ）
// =============================================================================
// 用法：挂在场景里任意 GameObject 上，把手骨骼拖进 Inspector 的数组即可。
// 两种驱动方式都实现了，用 driveMode 切换，可以直接对比哪个顺手。
//
// 【必读：两种模式的取舍】
//   JointAngles(M3DS)  16 个关节角。骨长恒定、关节合法、不会出现反屈或
//                      凭空的手指自转。缺点是要跟骨骼绑定姿势对齐一次。
//                      驱动机械臂只能用这个（伺服只吃角度）。
//   Quaternion(M3DQ)   16 个分段四元数，直接赋 localRotation。最省事，
//                      最忠实还原测量值 —— 包括那些不合解剖的姿态。
//
// 【关于 roll，用四元数模式前一定要知道】
// 每节指骨只贴了一颗反光球，绕骨轴的自转在数学上【不可观测】。四元数里
// 那一维是发送端用手背法向填出来的，不是测量值。手摊开时看不出来，手指
// 弯曲时可能看到轻微的"拧"。关节角模式天然没有这个问题（铰链只有一个
// 自由度，roll 由骨骼层级推出来）。
//
// 【坐标系换算】发送端是右手系、毫米、+Z 朝手背；Unity 是左手系、米、+Y 朝上。
// 换算故意放在这里做 —— 发送端不知道你的骨骼绑定姿势，这边知道。
// 下面 ConvertQuat/ConvertPos 里的实现是最常见的一种约定，如果你的模型
// 朝向不对，改这两个函数即可，不用动 C++ 侧。
// =============================================================================
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class MocapHandReceiver : MonoBehaviour
{
    public enum DriveMode { JointAngles, Quaternion }

    [Header("网络")]
    public int port = 9010;

    [Header("驱动方式")]
    public DriveMode driveMode = DriveMode.JointAngles;

    [Header("骨骼")]
    [Tooltip("手腕根骨骼")]
    public Transform wrist;
    [Tooltip("16 段骨骼，顺序：0=手腕, 然后每指 3 节(近/中/远)，" +
             "指序 拇/食/中/无名/小。没有的留空，会被跳过。")]
    public Transform[] segments = new Transform[16];

    [Header("选项")]
    [Tooltip("跟随手腕世界位置。做遥操作时通常关掉，只要姿态。")]
    public bool applyWristPosition = false;
    [Tooltip("毫米 -> 米")]
    public float positionScale = 0.001f;
    [Tooltip("跳过 segSource 为 Predicted/None 的分段，保持上一帧姿势。" +
             "开着更稳，关掉更跟手但会看到猜出来的姿态。")]
    public bool skipUnmeasuredSegments = true;

    // ---- 接收线程与最新一帧 ----
    UdpClient _sock;
    Thread _thread;
    volatile bool _run;
    readonly object _lock = new object();

    // M3DS
    Vector3 _wristPos;
    float[] _rot9 = new float[9];
    float[] _joints = new float[16];
    bool _hasPose;

    // M3DQ
    Quaternion[] _qWorld = new Quaternion[16];
    Quaternion[] _qLocal = new Quaternion[16];
    byte[] _segSrc = new byte[16];
    uint _flags;
    bool _hasQuat;

    // 诊断，直接看 Inspector
    [Header("诊断（只读）")]
    public int packetsReceived;
    public bool wristPoseValid;
    public bool mcpValid;
    public int validFingers;

    void OnEnable()
    {
        _sock = new UdpClient(port);
        _sock.Client.ReceiveTimeout = 500;
        _run = true;
        _thread = new Thread(RecvLoop) { IsBackground = true };
        _thread.Start();
    }

    void OnDisable()
    {
        _run = false;
        try { _sock?.Close(); } catch { }
        _thread?.Join(800);
        _sock = null; _thread = null;
    }

    void RecvLoop()
    {
        var ep = new IPEndPoint(IPAddress.Any, port);
        while (_run)
        {
            byte[] data;
            try { data = _sock.Receive(ref ep); }
            catch (SocketException) { continue; }   // 超时，正常
            catch (ObjectDisposedException) { break; }
            if (data == null || data.Length < 12) continue;

            // magic 前 3 字节都是 "M3D"，第 4 字节区分包类型
            if (data[0] != 'M' || data[1] != '3' || data[2] != 'D') continue;
            if (data[3] == 'S') ParseM3DS(data);
            else if (data[3] == 'Q') ParseM3DQ(data);
        }
    }

    // magic(4) ts(8) wristPos(3f) rot9(9f) joints(16f)
    void ParseM3DS(byte[] d)
    {
        const int need = 4 + 8 + (3 + 9 + 16) * 4;
        if (d.Length < need) return;
        int o = 12;
        lock (_lock)
        {
            _wristPos = new Vector3(BitConverter.ToSingle(d, o), BitConverter.ToSingle(d, o + 4),
                                    BitConverter.ToSingle(d, o + 8));
            o += 12;
            for (int i = 0; i < 9; ++i, o += 4) _rot9[i] = BitConverter.ToSingle(d, o);
            for (int i = 0; i < 16; ++i, o += 4) _joints[i] = BitConverter.ToSingle(d, o);
            _hasPose = true;
            packetsReceived++;
        }
    }

    // magic(4) ts(8) flags(4) wristPos(3f) qWorld(64f) qLocal(64f) segSource(16B)
    void ParseM3DQ(byte[] d)
    {
        const int need = 4 + 8 + 4 + 3 * 4 + 64 * 4 + 64 * 4 + 16;
        if (d.Length < need) return;
        int o = 12;
        lock (_lock)
        {
            _flags = BitConverter.ToUInt32(d, o); o += 4;
            _wristPos = new Vector3(BitConverter.ToSingle(d, o), BitConverter.ToSingle(d, o + 4),
                                    BitConverter.ToSingle(d, o + 8));
            o += 12;
            // 发送端顺序是 (w,x,y,z)，Unity 的构造函数是 (x,y,z,w)
            for (int s = 0; s < 16; ++s, o += 16)
                _qWorld[s] = new Quaternion(BitConverter.ToSingle(d, o + 4),
                                            BitConverter.ToSingle(d, o + 8),
                                            BitConverter.ToSingle(d, o + 12),
                                            BitConverter.ToSingle(d, o));
            for (int s = 0; s < 16; ++s, o += 16)
                _qLocal[s] = new Quaternion(BitConverter.ToSingle(d, o + 4),
                                            BitConverter.ToSingle(d, o + 8),
                                            BitConverter.ToSingle(d, o + 12),
                                            BitConverter.ToSingle(d, o));
            for (int s = 0; s < 16; ++s) _segSrc[s] = d[o + s];
            _hasQuat = true;
            packetsReceived++;
        }
    }

    void Update()
    {
        lock (_lock)
        {
            wristPoseValid = (_flags & 1u) != 0;
            mcpValid = (_flags & 2u) != 0;
            validFingers = 0;
            for (int f = 0; f < 5; ++f) if ((_flags & (1u << (8 + f))) != 0) validFingers++;

            if (applyWristPosition && wrist != null && (_hasPose || _hasQuat))
                wrist.position = ConvertPos(_wristPos);

            if (driveMode == DriveMode.Quaternion && _hasQuat) ApplyQuaternions();
            else if (driveMode == DriveMode.JointAngles && _hasPose) ApplyJointAngles();
        }
    }

    void ApplyQuaternions()
    {
        // 段0 = 手腕，用世界系；其余用 local（相对父节点），直接赋 localRotation
        if (wrist != null) wrist.rotation = ConvertQuat(_qWorld[0]);
        for (int s = 1; s < 16; ++s)
        {
            var t = segments[s];
            if (t == null) continue;
            // segSource: 0=None 1=Predicted 2=Geometry 3=IK
            // 【别把猜出来的当测量值】Predicted 表示该段两端点都是网络补的。
            if (skipUnmeasuredSegments && _segSrc[s] < 2) continue;
            t.localRotation = ConvertQuat(_qLocal[s]);
        }
    }

    void ApplyJointAngles()
    {
        // 16 维布局：q[0..3] 拇指 CMC屈/CMC展/MCP/IP
        //            q[4..6] 食指 MCP屈/MCP展/PIP，之后中/无名/小同构
        // 四指的 DIP 不在包里 —— 生理上它跟 PIP 强相关，这里按经验系数推。
        const float dipCoupling = 0.7f;
        ApplyFinger(0, _joints[0], _joints[1], _joints[2], _joints[3]);
        for (int f = 1; f < 5; ++f)
        {
            int b = 4 + (f - 1) * 3;
            ApplyFinger(f, _joints[b], _joints[b + 1], _joints[b + 2],
                        _joints[b + 2] * dipCoupling);
        }
    }

    // flex/abd 作用在近节，pip 在中节，dip 在远节。
    // 【旋转轴按你的骨骼绑定姿势改】下面这组（屈曲绕 X、外展绕 Y）是最常见的
    // 一种，如果手指弯错方向，通常只需要改这里的轴或符号。
    void ApplyFinger(int finger, float flex, float abd, float pip, float dip)
    {
        int s0 = 1 + finger * 3;
        SetLocal(s0 + 0, Quaternion.Euler(flex * Mathf.Rad2Deg, abd * Mathf.Rad2Deg, 0f));
        SetLocal(s0 + 1, Quaternion.Euler(pip * Mathf.Rad2Deg, 0f, 0f));
        SetLocal(s0 + 2, Quaternion.Euler(dip * Mathf.Rad2Deg, 0f, 0f));
    }

    void SetLocal(int idx, Quaternion q)
    {
        if (idx >= 0 && idx < segments.Length && segments[idx] != null)
            segments[idx].localRotation = q;
    }

    // ---- 坐标系换算：右手系(+Z朝手背, mm) -> Unity 左手系(+Y朝上, m) ----
    // 手性翻转对四元数的作用是把某一个轴的分量连同 w 一起变号。
    // 【模型朝向不对就改这里】不用动 C++ 侧。
    static Quaternion ConvertQuat(Quaternion q) => new Quaternion(-q.x, -q.y, q.z, q.w);
    Vector3 ConvertPos(Vector3 p) => new Vector3(p.x, p.y, -p.z) * positionScale;
}
