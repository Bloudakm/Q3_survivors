/*
Attempting to create a script to move the robot until it reaches an
obstacle and then it turns 90°
*/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class MoveScan
{
    enum RobotState { MovingForward, Rotating }
    RobotState currentState = RobotState.MovingForward;

    const string moveTopic = "cmd_vel";
    const string scanTopic = "scan";
    ROSConnection ros;
    public float publishMessagePeriod = 0.5f;
    private float timeElapsed;

    public float forwardSpeed = 0.2f;
    public float rotationSpeed = 0.5f; // rad/s
    public float obstacleThreshold = 0.5f; // meters
    public float avoidanceAngleDegrees = 50f;

    private float rotationStartTime;
    private float rotationDuration; // time needed to rotate 90°
    private  bool obstacleTooClose = false;

    public void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<LaserScanMsg>(scanTopic, OnReceivedScan);
        ros.RegisterPublisher<TwistMsg>(moveTopic);
    }

    private void OnReceivedScan(LaserScanMsg scan)
    {
        int totalPoints = scan.ranges.Length;
        float angleMin = scan.angle_min;
        float angleIncrement = scan.angle_increment;

        // Calculate start and end indices for ±25°
        float halfConeRad = (avoidanceAngleDegrees / 2f) * Mathf.Deg2Rad;
        int startIdx = Mathf.Max(0, Mathf.FloorToInt((0 - halfConeRad - angleMin) / angleIncrement));
        int endIdx = Mathf.Min(totalPoints - 1, Mathf.CeilToInt((0 + halfConeRad - angleMin) / angleIncrement));

        obstacleDetected = false;
        for (int i = startIdx; i <= endIdx; i++)
        {
            float range = scan.ranges[i];
            if (range > scan.range_min && range < obstacleThreshold)
            {
                obstacleDetected = true;
                break;
            }
        }

        if (obstacleDetected && currentState == RobotState.MovingForward)
        {
            currentState = RobotState.Rotating;
            rotationStartTime = Time.time;
            rotationDuration = Mathf.PI / 2f / rotationSpeed; // 90° / rotationSpeed
        }
    }

    void Update()
    {
        TwistMsg twist = new TwistMsg();

        switch (currentState)
        {
            case RobotState.MovingForward:
                twist.linear = new Vector3Msg(forwardSpeed, 0, 0);
                twist.angular = new Vector3Msg(0, 0, 0);
                break;

            case RobotState.Rotating:
                twist.linear = new Vector3Msg(0, 0, 0);
                twist.angular = new Vector3Msg(0, 0, rotationSpeed);

                if (Time.time - rotationStartTime >= rotationDuration)
                {
                    currentState = RobotState.MovingForward;
                }
                break;
        }

        ros.Publish(cmdVelTopic, twist);
    }
}