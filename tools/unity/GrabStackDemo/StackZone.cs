using System.Collections.Generic;
using UnityEngine;

/// <summary>堆叠区：松开时若物体在范围内，就吸到整齐的网格槽位并冻结。</summary>
public class StackZone : MonoBehaviour
{
    [Header("槽位尺寸")]
    public Vector3 cellSize = new Vector3(0.055f, 0.055f, 0.055f);

    [Header("判定范围（物体到中心多近才吸进去）")]
    public float snapRadius = 0.10f;

    [Header("每列叠几个（满了换下一列）")]
    public int maxPerColumn = 4;

    readonly List<GrabbableObject> _stack = new List<GrabbableObject>();

    public bool InRange(Vector3 p)
    {
        return Vector3.Distance(transform.position, p) < snapRadius;
    }

    public void Place(GrabbableObject o)
    {
        int i = _stack.Count;
        int col = i / maxPerColumn;
        int row = i % maxPerColumn;
        Vector3 pos = transform.position +
                      new Vector3(col * cellSize.x, row * cellSize.y, 0f);
        o.FreezeAt(pos, transform.rotation);
        _stack.Add(o);
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawWireCube(transform.position, new Vector3(cellSize.x, cellSize.y * 2, cellSize.z));
        Gizmos.color = new Color(0, 1, 1, 0.15f);
        Gizmos.DrawWireSphere(transform.position, snapRadius);
    }
}