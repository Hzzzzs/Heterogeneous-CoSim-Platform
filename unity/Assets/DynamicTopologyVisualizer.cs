using UnityEngine;
using System.Collections.Generic;

public class DynamicTopologyVisualizer : MonoBehaviour
{
    public enum AgentType { UAV, UGV }

    [System.Serializable]
    public struct TopologyEdge
    {
        [Tooltip("节点A的类型")]
        public AgentType typeA;
        [Tooltip("节点A的ID (从0开始)")]
        public int idA;

        [Tooltip("节点B的类型")]
        public AgentType typeB;
        [Tooltip("节点B的ID (从0开始)")]
        public int idB;
    }

    [Header("依赖组件")]
    [Tooltip("场景中的 FormationManager 实例")]
    public FormationManager manager;

    [Header("外观设置")]
    public Color lineColor = Color.black;
    public float lineWidth = 0.015f;

    [Header("拓扑连接配置 (配置ID对即可)")]
    public List<TopologyEdge> topologyEdges = new List<TopologyEdge>();

    private List<LineRenderer> lines = new List<LineRenderer>();
    private bool isInitialized = false;

    void Start()
    {
        // 如果没有手动拖入，尝试在场景中自动寻找管理器
        if (manager == null)
        {
            manager = FindObjectOfType<FormationManager>();
        }
    }

    void LateUpdate()
    {
        // 1. 【核心防御】：如果还没初始化，高频检查 FormationManager 是否已经把物体生成完毕
        if (!isInitialized)
        {
            if (CheckManagerReady())
            {
                InitializeLines();
            }
            return; // 物体没生成好之前，这一帧什么都不做，防止连线报错或闪烁
        }

        // 2. 【实时追踪】：物体生成好后，每帧平滑更新线条位置
        for (int i = 0; i < topologyEdges.Count; i++)
        {
            Transform tA = GetAgentTransform(topologyEdges[i].typeA, topologyEdges[i].idA);
            Transform tB = GetAgentTransform(topologyEdges[i].typeB, topologyEdges[i].idB);

            if (tA != null && tB != null)
            {
                lines[i].SetPosition(0, tA.position);
                lines[i].SetPosition(1, tB.position);
            }
        }
    }

    // 检查 FormationManager 是否已经完成了实例化并给数组赋了值
    bool CheckManagerReady()
    {
        if (manager == null) return false;
        if (manager.uavTransforms == null || manager.ugvTransforms == null) return false;

        // 确保数组长度已经达到预期
        if (manager.uavTransforms.Length < manager.numUAVs || manager.ugvTransforms.Length < manager.numUGVs) return false;

        // 确保生成的每一个 Transform 实体都不为 null
        for (int i = 0; i < manager.numUAVs; i++) if (manager.uavTransforms[i] == null) return false;
        for (int i = 0; i < manager.numUGVs; i++) if (manager.ugvTransforms[i] == null) return false;

        return true;
    }

    // 动态生成物理画笔
    void InitializeLines()
    {
        foreach (var edge in topologyEdges)
        {
            GameObject lineObj = new GameObject("DynamicTopologyLine_Auto");
            lineObj.transform.SetParent(this.transform);

            LineRenderer lr = lineObj.AddComponent<LineRenderer>();
            lr.material = new Material(Shader.Find("Sprites/Default"));
            lr.material.color = lineColor;
            lr.startColor = lineColor;
            lr.endColor = lineColor;
            lr.startWidth = lineWidth;
            lr.endWidth = lineWidth;
            lr.positionCount = 2;

            lines.Add(lr);
        }
        isInitialized = true;
        Debug.Log("✅ 动态拓扑连线系统成功捕获动态物体，初始化完成！");
    }

    // 根据类型和ID去 FormationManager 的开放数组里动态抓取 Transform
    Transform GetAgentTransform(AgentType type, int id)
    {
        if (type == AgentType.UAV)
        {
            if (id >= 0 && id < manager.uavTransforms.Length) return manager.uavTransforms[id];
        }
        else if (type == AgentType.UGV)
        {
            if (id >= 0 && id < manager.ugvTransforms.Length) return manager.ugvTransforms[id];
        }
        return null;
    }
}