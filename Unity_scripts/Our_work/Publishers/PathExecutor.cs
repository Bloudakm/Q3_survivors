using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

public class PathExecutor : MonoBehaviour
{
    private ROSConnection ros;
    public Vector3[] waypoints; // Assign in Unity Inspector
    public float arrivalThreshold = 0.1f; // How close to waypoint before moving to next

    private const string moveTopic = "/cmd_vel";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(moveTopic);
        StartCoroutine(FollowPath());
    }

    IEnumerator FollowPath()
    {
        foreach (Vector3 waypoint in waypoints)
        {
            while (Vector3.Distance(transform.position, waypoint) > arrivalThreshold)
            {
                Vector3 direction = (waypoint - transform.position).normalized;
                float linearX = direction.z * 0.5f; // Adjust speed as needed
                float angularZ = -Mathf.Atan2(direction.x, direction.z);

                TwistMsg cmdVel = new TwistMsg();
                cmdVel.linear.x = linearX;
                cmdVel.angular.z = angularZ;
                ros.Publish(moveTopic, cmdVel);

                yield return null;
            }
        }
        // Stop at the end
        ros.Publish(moveTopic, new TwistMsg());
    }
}
