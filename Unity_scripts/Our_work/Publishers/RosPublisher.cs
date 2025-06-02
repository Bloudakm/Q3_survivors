using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;

public class RosPublisher : MonoBehaviour

{
ROSConnection ros;
    public string topicName = "pos_rot";

    // The game object
    public GameObject turtlebot_burger;
    // Publish the robots's position and rotation every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;
    
    
    // Start is called before the first frame update
    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(topicName);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            turtlebot_burger.transform.rotation = Random.rotation;

            PosRotMsg turtlebot_burgerPos = new PosRotMsg(
                turtlebot_burger.transform.position.x,
                turtlebot_burger.transform.position.y,
                turtlebot_burger.transform.position.z,
                turtlebot_burger.transform.rotation.x,
                turtlebot_burger.transform.rotation.y,
                turtlebot_burger.transform.rotation.z,
                turtlebot_burger.transform.rotation.w
            );

            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, turtlebot_burgerPos);

            timeElapsed = 0;
        }
    }
}
