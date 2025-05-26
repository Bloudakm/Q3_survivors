using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;

public class OdomSubscriber : MonoBehaviour
{
    public GameObject turtlebot3Model; // Assign digital twin in Unity Inspector
    private ROSConnection ros;
    public string topicName = "/odom";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(topicName, UpdateRobotPose);
    }

    void UpdateRobotPose(OdometryMsg odom)
    {
        // Extract position (odom.pose.pose.position)
        Vector3 position = new Vector3(
            (float)odom.pose.pose.position.x,
            0,
            (float)odom.pose.pose.position.y
        );

        // Extract orientation (odom.pose.pose.orientation - converted from quaternion)
        Quaternion orientation = new Quaternion(
            (float)odom.pose.pose.orientation.x,
            (float)odom.pose.pose.orientation.y,
            (float)odom.pose.pose.orientation.z,
            (float)odom.pose.pose.orientation.w
        );

        // Apply to the Unity model
        turtlebot3Model.transform.position = position;
        turtlebot3Model.transform.rotation = orientation;
    }
}
