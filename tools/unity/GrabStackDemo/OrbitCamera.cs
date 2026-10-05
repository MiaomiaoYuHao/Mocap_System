using UnityEngine;

/// <summary>拍摄用环绕镜头：绕着目标缓慢旋转，可抬头/俯视。</summary>
public class OrbitCamera : MonoBehaviour
{
    public Transform target;
    public float radius = 1.2f;
    public float height = 0.6f;
    public float speedDegPerSec = 12f;

    float _angle;

    void LateUpdate()
    {
        if (target == null) return;
        _angle += speedDegPerSec * Time.deltaTime;
        var rad = _angle * Mathf.Deg2Rad;
        Vector3 pos = target.position +
                      new Vector3(Mathf.Cos(rad) * radius, height, Mathf.Sin(rad) * radius);
        transform.position = pos;
        transform.LookAt(target.position + Vector3.up * 0.1f);
    }
}