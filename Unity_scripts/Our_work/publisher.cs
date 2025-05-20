/*
Publisher for a very simple text sending.
*/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class Q3SurvivorsPublisher : MonoBehaviour // Parent is MonoBehaviour (what is that?)
{
    const string topicName = "Q3Survivors";
    ROSConnection ros;
    public float publishMessagePeriod = 0.5f;
    private float timeElapsed;

    // Function for running everything.
    public void Start()
    {

        // Establish connection
        ros = ROSConnection.GetOrCreateInstance();
        // Start publisher
        ros.RegisterPublisher<std_msgs>(topicName); // std_msgs.mes.String??
    }

    private void PublishMes()
    {
        // Create a random number from 0 to 100
        Random rnd = new Random();
        int randomNum = rnd.Next(100);

        // Create message
        string mes = $"Q3 Survivors will get: {randomNum}%";
        // Publish it
        ros.Publish(topicName, mes);
    }

    private void Update()
    {
        // Add the time elapsed
        timeElapsed += Time.deltaTime;

        // Only publish the message at some frequency
        if (timeElapsed > publishMessagePeriod)
        {
            PublishMes();

            timeElapsed = 0;
        }
    }
}