/*
Trying to move the turtlebot from a Unity script.
Something that might help: https://wiki.ros.org/ROS/Tutorials/UnderstandingTopics
*/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

public class MovePublisher : MonoBehaviour
{
    const string topicName = "cmd_vel";
    ROSConnection ros;
    public float publishMessagePeriod = 0.5f;
    private float timeElapsed = 0;
    private float totalTime = 0;

    public void Start()
    {
        // Establish connection
        ros = ROSConnection.GetOrCreateInstance();
        // Start publisher
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    private void PublishMes(int direction)
    {
        TwistMsg cmdVel = new TwistMsg
        {
            linear: 
            {
                x: 0,
                y: 0,
                z: 0
            },
            angular:
            {
                x: 0,
                y: 0,
                z: 0
            }
    };
        /* ALso might be like this?
        TwistMsg cmdVel = new TwistMsg {
            {
                0.0,
                0.0,
                0.0
            },
            {
                0.0,
                0.0,
                0.0
            }
        }
        */


        if (direction == 0)
        {
            cmdVel.linear.x = 1.0;
        }
        else if (direction == 1) {
            cmdVel.linear.x = 0.0;
            cmdVel.angular.x = 1.0;
        } else {
            cmdVel.linear.x = 0.0;
            cmdVel.angular.x = 0.0;
        }

        ros.Publish(topicName, cmdVel);
    }

    private void Update()
    {
        // Add the time elapsed
        timeElapsed += Time.deltaTime;
        totalTime += Time.deltaTime;

        // Only publish the message at some frequency && only for the first 4 seconds
        if (timeElapsed > publishMessagePeriod)
        {
            // drive in a direction for first two seconds and then turn.
            if (totalTime < 2)
            {
                PublishMes(0);
            }
            else if (totatlTime < 4)
            {
                PublishMes(1);
            }
            else
            {
                PublishMes(3);
            }

            timeElapsed = 0;
        }
    }
}