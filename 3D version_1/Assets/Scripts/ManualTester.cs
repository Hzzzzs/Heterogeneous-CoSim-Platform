using UnityEngine;

public class ManualTester : MonoBehaviour
{
    private UnicycleModel model;

    void Start()
    {
        model = GetComponent<UnicycleModel>();
    }

    void Update()
    {
        // 获取键盘轴向输入 (W/S 控制前进，A/D 控制转向)
        float vInput = Input.GetAxis("Vertical") * 5f;
        float wInput = Input.GetAxis("Horizontal") * 60f;

        // 将指令传给模型
        model.SetInputs(vInput, wInput);
    }
}