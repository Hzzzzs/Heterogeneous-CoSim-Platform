using UnityEngine;

public class PropellerRotator : MonoBehaviour
{
    public float speed = 3000f;
    // 轴向：根据你的模型，可能是 Vector3.up, forward 或 right
    public Vector3 axis = Vector3.up;

    void Update()
    {
        transform.Rotate(axis * speed * Time.deltaTime);
    }
}