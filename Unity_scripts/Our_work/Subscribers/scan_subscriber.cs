// Lidar info: https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/

using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;
using System.Linq;

public class ScanSub
{
    const string scanTopic = "scan";
    ROSConnection ros;
    public void Start()
    {
        // Establish connection
        ros = ROSConnection.GetOrCreateInstance();
        // Start publisher
        ros.Subscribe<LaserScanMsg>(scanTopic, LogScan);
    }

    public void LogScan(LaserScanMsg laserScanMsg)
    {
        for (int i = 0; i < 4; i++)
        {
            int deg = i * 90;
            float distance = laserScanMsg.ranges.ElementAt(deg);
            string mes = $"Distance at deg({deg}): {distance}";
            Console.WriteLine(mes);   
        }
    }
}