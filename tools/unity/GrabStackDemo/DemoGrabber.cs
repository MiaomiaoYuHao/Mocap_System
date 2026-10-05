using UnityEngine;

/// <summary>
/// Demo 抓取核心：用“拇指-食指-中指捏合”判定抓取，把物体挂到手掌锚点；
/// 松开时若在堆叠区上方则放上堆叠区，否则自然掉落。
/// 用法：挂在场景里，把 MocapHandReceiver 和 3 个指尖/手掌锚点拖进来。
/// </summary>
public class DemoGrabber : MonoBehaviour
{
    [Header("依赖")]
    public MocapHandReceiver receiver;
    public StackZone stackZone;

    [Header("指尖（不填会自动从 receiver.segments 取）")]
    public Transform thumbTip;   // 拇指远节 segments[3]
    public Transform indexTip;   // 食指远节 segments[6]
    public Transform middleTip;  // 中指远节 segments[9]

    [Header("手掌锚点（物体抓取后挂这里）")]
    public Transform palmAnchor;

    [Header("捏合判定（米）")]
    [Tooltip("指尖两两距离低于该值判定为捏住")]
    public float pinchGrabDist = 0.020f;
    [Tooltip("指尖距离高于该值判定为松开（要比抓取阈值大，避免抖动）")]
    public float pinchReleaseDist = 0.050f;
    [Tooltip("能抓到多远的物体")]
    public float grabRadius = 0.12f;

    GrabbableObject _held;
    bool _grasping;

    void Reset()
    {
        if (receiver == null) receiver = FindObjectOfType<MocapHandReceiver>();
    }

    [ContextMenu("自动填充指尖")]
    void AutoFillTips()
    {
        if (receiver == null) receiver = FindObjectOfType<MocapHandReceiver>();
        if (receiver == null || receiver.segments == null) return;
        if (thumbTip  == null && receiver.segments.Length > 3)  thumbTip  = receiver.segments[3];
        if (indexTip  == null && receiver.segments.Length > 6)  indexTip  = receiver.segments[6];
        if (middleTip == null && receiver.segments.Length > 9)  middleTip = receiver.segments[9];
    }

    void Update()
    {
        if (receiver == null) return;

        bool pinched = IsPinched();

        if (!_grasping && pinched) TryGrab();
        else if (_grasping && !pinched) TryRelease();
        else if (_grasping) HoldObject();
    }

    bool IsPinched()
    {
        if (thumbTip == null || indexTip == null || middleTip == null) return false;
        float d1 = Vector3.Distance(thumbTip.position, indexTip.position);
        float d2 = Vector3.Distance(thumbTip.position, middleTip.position);
        return d1 < pinchGrabDist && d2 < pinchGrabDist;
    }

    void TryGrab()
    {
        GrabbableObject best = null;
        float bestDist = grabRadius;
        Vector3 center = GripCenter();

        foreach (var o in GrabbableObject.all)
        {
            if (o.isHeld) continue;
            float d = Vector3.Distance(center, o.transform.position);
            if (d < bestDist) { bestDist = d; best = o; }
        }

        if (best != null && palmAnchor != null)
        {
            _held = best;
            best.Grab(palmAnchor);
            _grasping = true;
        }
    }

    void HoldObject()
    {
        // 已父级到 palmAnchor，本地零位即可；这里只做兜底
        if (_held == null) { _grasping = false; return; }
    }

    void TryRelease()
    {
        if (_held == null) { _grasping = false; return; }

        if (stackZone != null && stackZone.InRange(_held.transform.position))
            stackZone.Place(_held);
        else
            _held.DropWithPhysics();

        _held = null;
        _grasping = false;
    }

    Vector3 GripCenter()
    {
        Vector3 c = Vector3.zero; int n = 0;
        if (thumbTip  != null) { c += thumbTip.position;  n++; }
        if (indexTip  != null) { c += indexTip.position;  n++; }
        if (middleTip != null) { c += middleTip.position; n++; }
        return n == 0 ? transform.position : c / n;
    }
}