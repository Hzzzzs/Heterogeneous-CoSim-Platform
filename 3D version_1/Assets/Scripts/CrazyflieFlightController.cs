using UnityEngine;

public class CrazyflieFlightController : MonoBehaviour
{
    [Header("1. 飞行指令 (Z-UP 坐标系)")]
    [Tooltip("目标位置：X=东, Y=北, Z=高 (米)")]
    public Vector3 targetPosition_Zup = new Vector3(0, 0, 1.0f); // 默认飞1米高
    [Tooltip("飞行速度限制 (m/s)")]
    public float maxSpeed = 1.0f;

    [Header("2. 物理规格")]
    public float motorMaxThrust = 0.16f; // 单个电机最大推力 (N)
    public float droneMass = 0.027f;     // 无人机质量 (kg)

    [Header("3. PID 参数 (需要微调)")]
    // 高度环 PID (控制油门)
    public float p_gain_height = 10.0f;
    public float d_gain_height = 5.0f;
    // 水平位置 PID (控制 Pitch/Roll)
    public float p_gain_pos = 2.0f;
    public float d_gain_pos = 1.0f;
    // 姿态环 PID (控制角速度)
    public float p_gain_att = 0.1f;

    [Header("4. 电机连接")]
    public Transform motor1_FR; // M1: 右前
    public Transform motor2_RR; // M2: 右后
    public Transform motor3_RL; // M3: 左后
    public Transform motor4_FL; // M4: 左前

    [Header("5. 实时监控 (只读)")]
    public Vector3 currentPos_Zup; // 当前飞机的 Z-up 坐标
    public float[] motorPWM = new float[4]; // 0~1 之间的油门值

    private Rigidbody rb;
    private float lastErrorHeight = 0f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        // 确保使用物理模拟，而不是运动学
        rb.isKinematic = false;
        // 锁定刚体的 X/Z 轴旋转，防止翻车 (为了简化初级验证，先只控位置)
        // 进阶时可以解锁，依靠更复杂的姿态 PID 来平衡
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
    }

    void FixedUpdate()
    {
        // --- A. 坐标变换 (Unity -> Z-up) ---
        // 获取当前 Unity 坐标 (Y-up) 并转为 Z-up 方便计算
        currentPos_Zup = UnityToZUp(transform.position);

        // --- B. 导航计算 (计算下一步该去哪) ---
        // 限制速度：不要试图一步瞬移到目标点
        Vector3 posError = targetPosition_Zup - currentPos_Zup;
        Vector3 desiredVelocity = Vector3.ClampMagnitude(posError, 1.0f) * maxSpeed;

        // --- C. 高度控制 (Throttle PID) ---
        // 目标：控制 Z 轴 (Unity 的 Y 轴)
        // 基础油门 = 抵消重力
        float hoverThrust = droneMass * 9.81f;

        // PID 计算
        float heightError = targetPosition_Zup.z - currentPos_Zup.z;
        float heightDerivative = (heightError - lastErrorHeight) / Time.fixedDeltaTime;
        float pidOutput = (heightError * p_gain_height) + (heightDerivative * d_gain_height);
        lastErrorHeight = heightError;

        // 总推力 (力 = 质量 * 加速度 + 重力补偿)
        float totalThrustForce = hoverThrust + pidOutput;

        // --- D. 水平移动控制 (简化版) ---
        // 在完整飞控中，水平位移->目标倾角(Roll/Pitch)->电机差速
        // 这里为了验证“电机升力”，我们把水平力也分配给电机
        float moveForceX = (targetPosition_Zup.x - currentPos_Zup.x) * p_gain_pos;
        float moveForceY = (targetPosition_Zup.y - currentPos_Zup.y) * p_gain_pos;

        // --- E. 混控 (Mixer) ---
        // 将计算出的“总力”分配给 4 个电机
        // 这里做一个简化的力分配模型：
        // 1. 基础升力均分
        float baseForcePerMotor = totalThrustForce / 4.0f;

        // 2. 根据水平移动需求微调 (这里简化了力矩计算，直接给力)
        // 想向东飞(X+) -> 左边电机加力，右边减力 (Roll)
        // 想向北飞(Y+) -> 后边电机加力，前边减力 (Pitch)

        float rollInput = moveForceX * 0.05f;  // 系数需根据手感调整
        float pitchInput = moveForceY * 0.05f;

        // X字型混控逻辑 (标准 Crazyflie)
        // M1(右前): -Pitch, -Roll
        // M2(右后): +Pitch, -Roll
        // M3(左后): +Pitch, +Roll
        // M4(左前): -Pitch, +Roll

        float f1 = baseForcePerMotor - pitchInput - rollInput;
        float f2 = baseForcePerMotor + pitchInput - rollInput;
        float f3 = baseForcePerMotor + pitchInput + rollInput;
        float f4 = baseForcePerMotor - pitchInput + rollInput;

        // --- F. 输出给 Unity 物理引擎 ---
        ApplyMotorForce(motor1_FR, f1, 0);
        ApplyMotorForce(motor2_RR, f2, 1);
        ApplyMotorForce(motor3_RL, f3, 2);
        ApplyMotorForce(motor4_FL, f4, 3);
    }

    // 将计算出的牛顿(N)转换为 Unity 的力，并计算 PWM 供可视化
    void ApplyMotorForce(Transform motor, float forceN, int motorIndex)
    {
        // 限制力的大小 (不能小于0，不能超过最大推力)
        forceN = Mathf.Clamp(forceN, 0, motorMaxThrust);

        // 计算 PWM (0~1)，仅用于显示或传给可视化脚本
        motorPWM[motorIndex] = forceN / motorMaxThrust;

        // 施加物理力 (这是让飞机飞起来的关键)
        // 注意：电机永远沿着自己的 Y 轴 (绿色) 向上推
        rb.AddForceAtPosition(motor.up * forceN, motor.position);

        // 可视化红线
        Debug.DrawRay(motor.position, motor.up * forceN * 5.0f, Color.red);
    }

    // --- 坐标转换工具 ---
    // Unity (Y-up) -> 飞控 (Z-up)
    Vector3 UnityToZUp(Vector3 unityPos)
    {
        return new Vector3(unityPos.x, unityPos.z, unityPos.y);
    }
}