using UnityEngine;

/// <summary>可选：在指定位置生成一排可抓取方块。</summary>
public class ObjectSpawner : MonoBehaviour
{
    public GameObject prefab;
    public int count = 6;
    public Vector3 spacing = new Vector3(0.07f, 0f, 0f);

    [ContextMenu("生成")]
    public void Spawn()
    {
        for (int i = 0; i < count; ++i)
        {
            var go = Instantiate(prefab, transform.position + spacing * i, transform.rotation);
            go.name = "Cube_" + i;
        }
    }
}