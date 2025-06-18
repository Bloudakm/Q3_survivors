using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class SimulatedObstacleTester : MonoBehaviour
{
    [Header("Obstacle Settings")]
    public GameObject obstaclePrefab;
    public Vector3 dropPosition = new Vector3(0.3f, 0f, 0.8f);
    public Vector3 obstacleScale = new Vector3(0.05f, 0.3f, 0.05f);
    public string obstacleLayerName = "Default";

    private float lastDropTime;
    private bool waitingForReplan;

    void Start()
    {
        // Subscribe to path updates
        ROSConnection.GetOrCreateInstance().Subscribe<PathMsg>("/plan", OnPathUpdate);
    }
    
    void Update()
	{
	    if (Input.GetKeyDown("space")) {// Press ' ' to drop
	    	Debug.Log("Dropping obstacle");
		DropObstacleAndStartTest();
	    }
	}

    public void DropObstacleAndStartTest()
    {
        // Instantiate the obstacle
        GameObject obstacle = Instantiate(obstaclePrefab, dropPosition, Quaternion.identity);
        obstacle.transform.localScale = obstacleScale;
        obstacle.layer = LayerMask.NameToLayer(obstacleLayerName);
        obstacle.AddComponent<BoxCollider>();

        // Start timing
        lastDropTime = Time.realtimeSinceStartup;
        waitingForReplan = true;
        Debug.Log($"[LatencyTest] Obstacle dropped at {lastDropTime:F3}s");
    }

    void OnPathUpdate(PathMsg msg)
    {
        if (!waitingForReplan) return;

        float now = Time.realtimeSinceStartup;
        float latency = (now - lastDropTime) * 1000f;
        Debug.Log($"[LatencyTest] New path received after {latency:F1} ms");

        waitingForReplan = false;
    }
}

