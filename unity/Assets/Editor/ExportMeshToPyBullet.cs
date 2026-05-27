using UnityEngine;
using UnityEditor;
using System.IO;
using System.Text;

public class MeshExporter : EditorWindow
{
    [MenuItem("Tools/Batch Export Selected to PyBullet")]
    public static void BatchExport()
    {
        // 1. 获取所有选中的物体
        GameObject[] selections = Selection.gameObjects;
        if (selections.Length == 0)
        {
            EditorUtility.DisplayDialog("提示", "请先在层级面板选中想要导出的物体（按住Ctrl多选）", "OK");
            return;
        }

        // 2. 选择保存文件夹
        string folderPath = EditorUtility.OpenFolderPanel("选择保存 .obj 文件的文件夹", "", "");
        if (string.IsNullOrEmpty(folderPath)) return;

        int successCount = 0;

        foreach (GameObject obj in selections)
        {
            MeshFilter mf = obj.GetComponent<MeshFilter>();
            if (mf == null) continue; // 跳过没有网格的物体

            Mesh mesh = mf.sharedMesh;
            StringBuilder sb = new StringBuilder();
            Transform t = obj.transform;

            sb.AppendLine("# Unity to PyBullet Batch Export");

            // 顶点导出（包含 Y/Z 翻转）
            foreach (Vector3 v in mesh.vertices)
            {
                Vector3 worldV = t.TransformPoint(v);
                sb.AppendLine($"v {worldV.x} {worldV.z} {worldV.y}");
            }

            // 面片导出（反转绕序）
            for (int i = 0; i < mesh.triangles.Length; i += 3)
            {
                sb.AppendLine($"f {mesh.triangles[i + 2] + 1} {mesh.triangles[i + 1] + 1} {mesh.triangles[i] + 1}");
            }

            // 以物体名命名文件
            string fileName = obj.name.Replace(" ", "_") + ".obj";
            string fullPath = Path.Combine(folderPath, fileName);

            File.WriteAllText(fullPath, sb.ToString());
            successCount++;
        }

        EditorUtility.DisplayDialog("导出完成", $"成功导出 {successCount} 个物体到文件夹：\n{folderPath}", "OK");
        Debug.Log($"✅ 批量导出成功，共 {successCount} 个文件。");
    }
}