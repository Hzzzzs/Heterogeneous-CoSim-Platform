using System.Collections.Generic;
using UnityEngine;

public class SimulationManager : MonoBehaviour
{
    public GameObject ugvPrefab;
    public GameObject uavPrefab;

    public int ugvCount = 3;
    public int uavCount = 3;

    public static List<GameObject> UGVs = new List<GameObject>();
    public static List<GameObject> UAVs = new List<GameObject>();

    void Start()
    {
        SpawnAgents();
    }

    void SpawnAgents()
    {
        UGVs.Clear();
        UAVs.Clear();

        for (int i = 0; i < ugvCount; i++)
        {
            Vector3 pos = new Vector3(
                Random.Range(-3f, 3f),
                0f,
                Random.Range(-3f, 3f)
            );

            GameObject ugv = Instantiate(ugvPrefab, pos, Quaternion.identity);
            UGVs.Add(ugv);
        }

        for (int i = 0; i < uavCount; i++)
        {
            Vector3 pos = new Vector3(
                Random.Range(-3f, 3f),
                3f,
                Random.Range(-3f, 3f)
            );

            GameObject uav = Instantiate(uavPrefab, pos, Quaternion.identity);
            UAVs.Add(uav);
        }
    }
}
