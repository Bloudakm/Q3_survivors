using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class AutoNavigationManager : MonoBehaviour
{
    private Transform[] waypoints = {}; 
    int currentIdx = 0;
    float distanceThreshold = 0.25f;
    double startTime = 0;
    ROSConnection ros;
    public GameObject robot;
    public float k_Nav2InitializeTime = 5.0f;
    public float destinationTimeout = 5.0f;
    const string k_GoalPoseFrameId = "map";
    public string goalTopic = "/goal_pose";
    public bool infinite = false;
    private bool firstTime = true;

    void Start()
    {

    }
    
    System.Collections.IEnumerator InitAndSendFirstGoal()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<RosMessageTypes.Geometry.PoseStampedMsg>(goalTopic);
	firstTime = false;
        Debug.Log($"Waiting {k_Nav2InitializeTime} seconds for Nav2 to initialize...");
        yield return new WaitForSeconds(k_Nav2InitializeTime);
        SendNextGoal();
    }

    void Update()
    {
        if (currentIdx >= waypoints.Length) 
        {
        	if(currentIdx == 0) return;
        	if(infinite)
        	{
        		ResetWaypoints();
        	}
        	Debug.Log("Finished all waypoints, what now.");
        	return;
        }

        if (robot == null) return;

        float distance = Vector3.Distance(robot.transform.position, waypoints[currentIdx].position);
	
	double currentTime = Clock.time;
	
        if (distance < distanceThreshold)
        {
            Debug.Log($"Waypoint {currentIdx} reached!");
            var reachedObj = GameObject.Find("NavWaypoint_"+currentIdx);
            reachedObj.transform.localScale = new Vector3(0, 0, 0);
            
            currentIdx++;
            if (currentIdx < waypoints.Length)
            {
                SendNextGoal();
            }
        } else if((currentTime-startTime) > destinationTimeout) 
        {
            Debug.Log("Waiting too long!");
            SendNextGoal();
        }
        
    }

    void SendNextGoal()
    {
        if (currentIdx >= waypoints.Length) return;

        var target = waypoints[currentIdx];

        var pose = new RosMessageTypes.Geometry.PoseMsg
        {
            position = target.position.To<FLU>(),
            orientation = target.rotation.To<FLU>()
        };

        var msg = new RosMessageTypes.Geometry.PoseStampedMsg
	    {
		header =
		{
		    stamp = new TimeStamp(Clock.time),
		    frame_id = k_GoalPoseFrameId
		},
		pose = pose
	    };

        Debug.Log($"Sending goal to waypoint {currentIdx}: {target.position}");
        ros.Publish(goalTopic, msg);
        startTime = Clock.time;
    }

    public void SetWaypoints(Vector3[] newWaypoints)
    {
        waypoints = new Transform[newWaypoints.Length];
        for (int i = 0; i < newWaypoints.Length; i++)
        {
            GameObject wpObj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
		wpObj.name = $"NavWaypoint_{i}";
		wpObj.transform.position = newWaypoints[i];
		wpObj.transform.localScale = Vector3.one * 0.02f; // Small sphere

		// Make the sphere red
		var renderer = wpObj.GetComponent<Renderer>();
		if (renderer != null)
		{
		    renderer.material.color = Color.red;
		}

		// Optional: disable collider if you don’t want it to block things
		var collider = wpObj.GetComponent<Collider>();
		if (collider != null)
		{
		    collider.enabled = false;
		}

		waypoints[i] = wpObj.transform;
        }
        currentIdx = 0;
        
        if (firstTime) 
        {
        	StartCoroutine(InitAndSendFirstGoal());
        } else {
        	SendNextGoal();
        }
    }
    
    public int getIndex() 
    {
    	return currentIdx;
    }
    
    public void destroyWaypoints()
    {
    	for(int i = currentIdx; i < waypoints.Length; i++)
    	{
    		var reachedObj = GameObject.Find("NavWaypoint_"+i);
            	reachedObj.transform.localScale = new Vector3(0, 0, 0);
    	}
    }
    
    public void ResetWaypoints()
    {
    	for(int i = 0; i < waypoints.Length; i++)
    	{
    		
    		var reachedObj = GameObject.Find("NavWaypoint_"+i);
    		Debug.Log(reachedObj);
            	reachedObj.transform.localScale = Vector3.one * 0.02f;
    	}
    	
    	currentIdx = 0;
    	SendNextGoal();
    	Debug.Log("points reset");
    }
}

