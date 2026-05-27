using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Globalization;

[RequireComponent(typeof(FormationManager))]
public class UDPReceiver : MonoBehaviour
{
    [Header("网络设置")]
    public int port = 5006;
    public bool showDebugLog = false;

    [Header("视觉偏航角补偿")]
    public Vector3 ugvRotationOffset = new Vector3(0, 90, 0);

    private FormationManager manager;
    private UdpClient client;
    private Thread receiveThread;
    private bool isRunning = true;

    private Vector3[] uavTargetPos; private Vector3[] uavTargetEuler; private bool[] uavHasData;
    private Vector3[] ugvTargetPos; private Vector3[] ugvTargetEuler; private bool[] ugvHasData;

    private bool[] uavIsFirstFrame;
    private bool[] ugvIsFirstFrame;

    void Start()
    {
        Debug.Log($"✅ UDPReceiver 已启动，正在监听 {port} 端口...");
        manager = GetComponent<FormationManager>();

        uavTargetPos = new Vector3[manager.numUAVs];
        uavTargetEuler = new Vector3[manager.numUAVs];
        uavHasData = new bool[manager.numUAVs];

        ugvTargetPos = new Vector3[manager.numUGVs];
        ugvTargetEuler = new Vector3[manager.numUGVs];
        ugvHasData = new bool[manager.numUGVs];

        uavIsFirstFrame = new bool[manager.numUAVs];
        for (int i = 0; i < uavIsFirstFrame.Length; i++) uavIsFirstFrame[i] = true;

        ugvIsFirstFrame = new bool[manager.numUGVs];
        for (int i = 0; i < ugvIsFirstFrame.Length; i++) ugvIsFirstFrame[i] = true;

        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void Update()
    {
        // ================= UAV 同步 =================
        for (int i = 0; i < manager.numUAVs; i++)
        {
            if (uavHasData[i] && manager.uavTransforms[i] != null)
            {
                Transform t = manager.uavTransforms[i];

                if (uavIsFirstFrame[i])
                {
                    // 1. 首帧瞬移
                    Vector3 startPos = new Vector3(uavTargetPos[i].x, uavTargetPos[i].z, uavTargetPos[i].y);
                    t.position = startPos;
                    float r = uavTargetEuler[i].x * Mathf.Rad2Deg;
                    float p = uavTargetEuler[i].y * Mathf.Rad2Deg;
                    float y = uavTargetEuler[i].z * Mathf.Rad2Deg;
                    t.rotation = Quaternion.Euler(-p, -y, r);

                    // 2. 纯净版轨迹处理（完全尊重 Prefab 设置）
                    TrailRenderer trail = t.GetComponent<TrailRenderer>();
                    if (trail != null)
                    {
                        trail.enabled = true;     // 唤醒组件
                        trail.emitting = false;   // 暂停发射墨水
                        trail.Clear();            // 抹除从原点瞬移过来的拉线残影
                        trail.emitting = true;    // 在正确坐标上重新开始发射
                    }

                    uavIsFirstFrame[i] = false;
                    uavHasData[i] = false;
                    continue;
                }

                ApplyTransform(t, uavTargetPos[i], uavTargetEuler[i], Quaternion.identity);
                uavHasData[i] = false;
            }
        }

        // ================= UGV 同步 =================
        for (int i = 0; i < manager.numUGVs; i++)
        {
            if (ugvHasData[i] && manager.ugvTransforms[i] != null)
            {
                Transform t = manager.ugvTransforms[i];
                Quaternion ugvOffsetQuat = Quaternion.Euler(ugvRotationOffset);

                if (ugvIsFirstFrame[i])
                {
                    // 1. 首帧瞬移
                    Vector3 startPos = new Vector3(ugvTargetPos[i].x, ugvTargetPos[i].z, ugvTargetPos[i].y);
                    t.position = startPos;
                    float r = ugvTargetEuler[i].x * Mathf.Rad2Deg;
                    float p = ugvTargetEuler[i].y * Mathf.Rad2Deg;
                    float y = ugvTargetEuler[i].z * Mathf.Rad2Deg;
                    t.rotation = Quaternion.Euler(-p, -y, r) * ugvOffsetQuat;

                    // 2. 纯净版轨迹处理（完全尊重 Prefab 设置）
                    TrailRenderer trail = t.GetComponent<TrailRenderer>();
                    if (trail != null)
                    {
                        trail.enabled = true;     // 唤醒组件
                        trail.emitting = false;   // 暂停发射墨水
                        trail.Clear();            // 抹除从原点瞬移过来的拉线残影
                        trail.emitting = true;    // 在正确坐标上重新开始发射
                    }

                    ugvIsFirstFrame[i] = false;
                    ugvHasData[i] = false;
                    continue;
                }

                ApplyTransform(t, ugvTargetPos[i], ugvTargetEuler[i], ugvOffsetQuat);
                ugvHasData[i] = false;
            }
        }
    }

    void ApplyTransform(Transform t, Vector3 targetPos, Vector3 targetEuler, Quaternion offset)
    {
        Vector3 unityPos = new Vector3(targetPos.x, targetPos.z, targetPos.y);
        t.position = Vector3.Lerp(t.position, unityPos, Time.deltaTime * 20f);

        float r = targetEuler.x * Mathf.Rad2Deg;
        float p = targetEuler.y * Mathf.Rad2Deg;
        float y = targetEuler.z * Mathf.Rad2Deg;
        Quaternion targetRot = Quaternion.Euler(-p, -y, r) * offset;

        t.rotation = Quaternion.Slerp(t.rotation, targetRot, Time.deltaTime * 20f);
    }

    private void ReceiveData()
    {
        try { client = new UdpClient(port); } catch (System.Exception e) { Debug.LogError("端口占用: " + e.Message); return; }
        IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);

        while (isRunning)
        {
            try
            {
                byte[] data = client.Receive(ref anyIP);
                string text = Encoding.UTF8.GetString(data);

                string[] p = text.Split(',');
                if (p.Length >= 8)
                {
                    string type = p[0];
                    int id = int.Parse(p[1]);
                    float x = float.Parse(p[2], CultureInfo.InvariantCulture);
                    float y = float.Parse(p[3], CultureInfo.InvariantCulture);
                    float z = float.Parse(p[4], CultureInfo.InvariantCulture);
                    float r = float.Parse(p[5], CultureInfo.InvariantCulture);
                    float pt = float.Parse(p[6], CultureInfo.InvariantCulture);
                    float yw = float.Parse(p[7], CultureInfo.InvariantCulture);

                    if (type == "UAV" && id >= 0 && id < manager.numUAVs)
                    {
                        uavTargetPos[id] = new Vector3(x, y, z);
                        uavTargetEuler[id] = new Vector3(r, pt, yw);
                        uavHasData[id] = true;
                    }
                    else if (type == "UGV" && id >= 0 && id < manager.numUGVs)
                    {
                        ugvTargetPos[id] = new Vector3(x, y, z);
                        ugvTargetEuler[id] = new Vector3(r, pt, yw);
                        ugvHasData[id] = true;
                    }
                }
            }
            catch (System.Exception e) { if (isRunning) Debug.LogWarning("UDP Error: " + e.Message); }
        }
    }

    void OnDestroy()
    {
        isRunning = false;
        if (client != null) client.Close();
        if (receiveThread != null && receiveThread.IsAlive) receiveThread.Abort();
    }
}