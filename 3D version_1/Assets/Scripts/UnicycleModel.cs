using UnityEngine;

// 对应 PPT 中的物理层 (Physics Layer)：处理动力学模型与环境约束
public class UnicycleModel : MonoBehaviour
{
    [Header("状态变量")]
    public float linearVelocity;   // 线速度 v
    public float angularVelocity;  // 角速度 w (度/秒)

    [Header("异构属性设置")]
    public bool isUAV = false;     // 是否为无人机
    public float flightHeight = 2.0f; // 目标高度

    [Header("动力学限制")]
    public float maxLinearSpeed = 3.0f;
    public float maxAngularSpeed = 180.0f;

    void FixedUpdate()
    {
        // 1. 线速度：控制物体沿其正前方移动
        linearVelocity = Mathf.Clamp(linearVelocity, -maxLinearSpeed, maxLinearSpeed);
        transform.position += transform.forward * linearVelocity * Time.fixedDeltaTime;

        // 2. 角速度：控制物体绕 Y 轴转向
        float rotationAmount = Mathf.Clamp(angularVelocity, -maxAngularSpeed, maxAngularSpeed);
        transform.Rotate(0, rotationAmount * Time.fixedDeltaTime, 0);

        // 3. 异构约束处理 [cite: 119, 120]
        if (isUAV)
        {
            // 无人机：简单的比例控制追踪巡航高度
            float heightError = flightHeight - transform.position.y;
            transform.Translate(Vector3.up * heightError * 2.0f * Time.fixedDeltaTime, Space.World);
        }
        else
        {
            // 无人车：强制贴地，防止物理引擎误差导致离地
            Vector3 pos = transform.position;
            pos.y = 0.1f;
            transform.position = pos;
        }
    }

    // 提供给上层控制器的接口 [cite: 122]
    public void SetInputs(float v, float w)
    {
        linearVelocity = v;
        angularVelocity = w;
    }
}