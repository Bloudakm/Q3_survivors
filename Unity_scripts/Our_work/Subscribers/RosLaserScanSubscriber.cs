using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // For LaserScanMsg

public class RosLaserScanSubscriber : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/scan";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<LaserScanMsg>(topicName, ReceiveLaserScan);
    }

    void ReceiveLaserScan(LaserScanMsg laserScan)
    {
        // Process laserScan.ranges, laserScan.angle_min, etc.
        Debug.Log("Received LaserScan with " + laserScan.ranges.Length + " points.");

        for (int i = 0; i < laserScan.ranges.Length; i++)
        {
            float distance = laserScanMsg.ranges.ElementAt(i);
            string mes = $"Distance at deg({i}): {distance}";
            Debug.Log(mes);   
        }
    }
}
