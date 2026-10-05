using System.Collections.Generic;
using UnityEngine;

/// <summary>可被抓取物体：挂到方块/球等物体上（需要 Rigidbody + Collider）。</summary>
public class GrabbableObject : MonoBehaviour
{
    public static readonly List<GrabbableObject> all = new List<GrabbableObject>();

    public bool isHeld { get; private set; }

    Rigidbody _rb;
    bool _wasKinematic;
    bool _hadGravity;

    void Awake()
    {
        _rb = GetComponent<Rigidbody>();
        if (_rb != null) { _wasKinematic = _rb.isKinematic; _hadGravity = _rb.useGravity; }
    }

    void OnEnable()  { all.Add(this); }
    void OnDisable() { all.Remove(this); }

    /// <summary>抓取：挂到锚点，冻结物理。</summary>
    public void Grab(Transform anchor)
    {
        isHeld = true;
        transform.SetParent(anchor);
        transform.localPosition = Vector3.zero;
        transform.localRotation = Quaternion.identity;
        if (_rb != null) _rb.isKinematic = true;
    }

    /// <summary>自然掉落：恢复物理。</summary>
    public void DropWithPhysics()
    {
        isHeld = false;
        transform.SetParent(null);
        if (_rb != null)
        {
            _rb.isKinematic = _wasKinematic;
            _rb.useGravity = _hadGravity;
        }
    }

    /// <summary>冻结到堆叠槽位（保持整齐，不掉落）。</summary>
    public void FreezeAt(Vector3 pos, Quaternion rot)
    {
        isHeld = false;
        transform.SetParent(null);
        transform.position = pos;
        transform.rotation = rot;
        if (_rb != null) _rb.isKinematic = true;
    }
}