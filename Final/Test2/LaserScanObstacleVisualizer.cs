using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class LaserScanObstacleVisualizer : MonoBehaviour
{
    public string topicName = "/scan";
    public GameObject obstaclePrefab;
    public Transform lidarOrigin; // usually the robot base_link or scanner link
    public float obstacleScale = 0.1f;

    List<GameObject> currentObstacles = new List<GameObject>();
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<LaserScanMsg>(topicName, ScanCallback);
    }

    void ClearObstacles()
    {
        foreach (var obj in currentObstacles)
            Destroy(obj);
        currentObstacles.Clear();
    }

    void ScanCallback(LaserScanMsg scan)
    {
        ClearObstacles();

        // Uncomment this block to log latency:
        /*
        double rosTime = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9;
        double unityTime = Time.realtimeSinceStartupAsDouble;
        double latency = unityTime - rosTime;
        Debug.Log($"[LIDAR] Latency: {latency * 1000:F2} ms");
        */

        float angle = scan.angle_min;

        for (int i = 0; i < scan.ranges.Length; i++)
        {
            float range = scan.ranges[i];
            if (range < scan.range_min || range > scan.range_max)
            {
                angle += scan.angle_increment;
                continue;
            }

            // Compute local hit point
            Vector3 direction = Quaternion.Euler(0, -angle * Mathf.Rad2Deg, 0) * Vector3.forward;
            Vector3 localPos = direction * range;

            // Convert to world space
            Vector3 worldPos = lidarOrigin.TransformPoint(localPos);

            // Instantiate obstacle cube
            GameObject obj = Instantiate(obstaclePrefab, worldPos, Quaternion.identity);
            //obj.transform.localScale = Vector3.one * new Vector3(obstacleScale, scaleY, obstacleScale);
            currentObstacles.Add(obj);

            angle += scan.angle_increment;
        }
    }
}
