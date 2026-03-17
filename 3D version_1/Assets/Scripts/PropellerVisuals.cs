using UnityEngine;

public class PropellerVisuals : MonoBehaviour
{
    // 转速乘数，视觉效果参数
    public float rotationSpeedMultiplier = 2000f;

    // 旋转方向：1 为顺时针，-1 为逆时针
    public float direction = 1f;

    // 接收来自飞控的油门信号 (0.0 ~ 1.0)
    public float currentThrottle = 0f;

    void Update()
    {
        // 只有当有油门信号时才旋转
        if (currentThrottle > 0.01f)
        {
            // 注意：因为你的模型坐标轴修正过，我们要绕 Z轴 (Forward) 旋转
            // 如果发现转的方向不对，可以把 direction 改成 -1
            transform.Rotate(Vector3.forward, currentThrottle * rotationSpeedMultiplier * direction * Time.deltaTime);
        }
    }
}