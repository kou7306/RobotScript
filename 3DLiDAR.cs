using UnityEngine;
using System.Collections.Generic;

public class LiDAR3DSimulator : MonoBehaviour
{
    public int horizontalRays = 120; // 水平方向のビーム数（最大120）
    public int verticalRays = 32;    // 垂直方向のビーム数（最大32）
    public float maxDistance = 100f; // 最大測定距離
    public float horizontalFOV = 360f; // 水平方向の視野角（360度）
    public float verticalFOV = 60f;   // 垂直方向の視野角（最大90度）

    private List<Vector3> pointCloud = new List<Vector3>();

    public List<Vector3> GetPointCloud()
    {
        return pointCloud;
    }

    void Update()
    {
        Simulate3DLiDAR();
    }

    void Simulate3DLiDAR()
    {
        pointCloud.Clear(); // 前フレームの点群データをクリア

        float horizontalAngleIncrement = horizontalFOV / horizontalRays;
        float verticalAngleIncrement = verticalFOV / verticalRays;

        for (int h = 0; h < horizontalRays; h++)
        {
            float horizontalAngle = -horizontalFOV / 2 + h * horizontalAngleIncrement;

            for (int v = 0; v < verticalRays; v++)
            {
                float verticalAngle = -verticalFOV / 2 + v * verticalAngleIncrement;
                Vector3 direction = Quaternion.Euler(verticalAngle, horizontalAngle, 0) * transform.forward;

                RaycastHit hit;
                if (Physics.Raycast(transform.position, direction, out hit, maxDistance))
                {
                    pointCloud.Add(hit.point);
                    Debug.DrawLine(transform.position, hit.point, Color.red);
                }
            }
        }

        ProcessPointCloud(pointCloud);
    }

    void ProcessPointCloud(List<Vector3> points)
    {
        Debug.Log($"Captured {points.Count} points");
    }
}