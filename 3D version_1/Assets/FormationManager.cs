using UnityEngine;

public class FormationManager : MonoBehaviour
{
    [Header("智能体预制体 (Prefabs)")]
    public GameObject uavPrefab;
    public GameObject ugvPrefab;

    [Header("编队规模配置")]
    public int numUAVs = 3;
    public int numUGVs = 3;

    [HideInInspector]
    public Transform[] uavTransforms;
    [HideInInspector]
    public Transform[] ugvTransforms;

    void Awake()
    {
        uavTransforms = new Transform[numUAVs];
        for (int i = 0; i < numUAVs; i++)
        {
            // 默认全在原点生成，之后会被 UDP 传来的真实坐标瞬间接管
            GameObject obj = Instantiate(uavPrefab, Vector3.zero, Quaternion.identity, this.transform);
            obj.name = "UAV_" + i;
            uavTransforms[i] = obj.transform;
        }

        ugvTransforms = new Transform[numUGVs];
        for (int i = 0; i < numUGVs; i++)
        {
            GameObject obj = Instantiate(ugvPrefab, Vector3.zero, Quaternion.identity, this.transform);
            obj.name = "UGV_" + i;
            ugvTransforms[i] = obj.transform;
        }
    }
}