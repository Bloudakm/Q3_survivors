using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;

public class CmdVelPublisher : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "/cmd_vel";
    public float publishMessageFrequency = 0.1f; // 10 Hz
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)
        {
            // Example: Move forward at 0.2 m/s, no rotation
            TwistMsg twist = new TwistMsg();
            twist.linear = new Vector3Msg(0.2f, 0, 0);
            twist.angular = new Vector3Msg(0, 0, 0);

            ros.Publish(topicName, twist);
            timeElapsed = 0;
        }
    }

    // Call this function from your navigation algorithm to set velocities dynamically
    public void SendVelocity(float linear, float angular)
    {
        TwistMsg twist = new TwistMsg();
        twist.linear = new Vector3Msg(linear, 0, 0);
        twist.angular = new Vector3Msg(0, 0, angular);
        ros.Publish(topicName, twist);
    }
}
