using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Globalization;

[RequireComponent(typeof(FormationManager))] // 强制依赖管理器
public class UDPReceiver : MonoBehaviour
{
    [Header("网络设置")]
    public int port = 5006;
    public bool showDebugLog = false;

    private FormationManager manager;
    private UdpClient client;
    private Thread receiveThread;
    private bool isRunning = true;

    // 数据缓存区：分开存储 UAV 和 UGV
    private Vector3[] uavTargetPos; private Vector3[] uavTargetEuler; private bool[] uavHasData;
    private Vector3[] ugvTargetPos; private Vector3[] ugvTargetEuler; private bool[] ugvHasData;

    void Start()
    {
        manager = GetComponent<FormationManager>();

        // 根据 Manager 的数量初始化缓存
        uavTargetPos = new Vector3[manager.numUAVs];
        uavTargetEuler = new Vector3[manager.numUAVs];
        uavHasData = new bool[manager.numUAVs];

        ugvTargetPos = new Vector3[manager.numUGVs];
        ugvTargetEuler = new Vector3[manager.numUGVs];
        ugvHasData = new bool[manager.numUGVs];

        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void Update()
    {
        // 同步 UAV 物理渲染
        for (int i = 0; i < manager.numUAVs; i++)
        {
            if (uavHasData[i] && manager.uavTransforms[i] != null)
            {
                ApplyTransform(manager.uavTransforms[i], uavTargetPos[i], uavTargetEuler[i]);
                uavHasData[i] = false;
            }
        }

        // 同步 UGV 物理渲染
        for (int i = 0; i < manager.numUGVs; i++)
        {
            if (ugvHasData[i] && manager.ugvTransforms[i] != null)
            {
                ApplyTransform(manager.ugvTransforms[i], ugvTargetPos[i], ugvTargetEuler[i]);
                ugvHasData[i] = false;
            }
        }
    }

    void ApplyTransform(Transform t, Vector3 targetPos, Vector3 targetEuler)
    {
        // 坐标系转换：PyBullet (x, y, z) -> Unity (x, z, y)
        Vector3 unityPos = new Vector3(targetPos.x, targetPos.z, targetPos.y);
        t.position = Vector3.Lerp(t.position, unityPos, Time.deltaTime * 20f);

        float r = targetEuler.x * Mathf.Rad2Deg;
        float p = targetEuler.y * Mathf.Rad2Deg;
        float y = targetEuler.z * Mathf.Rad2Deg;
        Quaternion targetRot = Quaternion.Euler(-p, -y, r);
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
                if (showDebugLog) Debug.Log($"收到: {text}");

                string[] p = text.Split(',');

                // 协议格式: "Type, ID, x, y, z, r, p, yw" (例如 "UAV,0,1.2,3.4,...")
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