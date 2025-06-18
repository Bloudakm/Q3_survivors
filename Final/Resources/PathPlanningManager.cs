using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class PathPlanningManager : MonoBehaviour
{
    public GameObject robot;
    public List<Passenger> passengers = new List<Passenger>
        {
            new Passenger(2, 2, 10, 5), 
            new Passenger(4, 10, 2, 3),
            //new Passenger(3, 3, 2, 2)
        };
    public AutoNavigationManager navManager;
    public int mapWidth = 16, mapHeight = 16;
    public bool[,] walkableMap;
    private bool even = true;

    void Start()
    {
        // Define full walkability or integrate obstacle map from SLAM if needed
        walkableMap = new bool[mapWidth, mapHeight];
        for (int x = 0; x < mapWidth; x++)
            for (int y = 0; y < mapHeight; y++)
                walkableMap[x, y] = true;

        Invoke("PlanRoute", 2f); // Wait for SLAM pose to initialize
    }
    
    void Update() 
    {
    	if(Input.GetKeyDown("space")) 
    	{	
    		Debug.Log("Adding passenger");
    		navManager.destroyWaypoints();
    		var newPass = even ? new Passenger(3, 3, 2, 2) : new Passenger(4, 10, 6, 5);
    		passengers.Add(newPass);
    		float startTime = Time.realtimeSinceStartup;
    		PlanRoute();
    		float endTime = Time.realtimeSinceStartup;
    		Debug.Log("Time taken to recalculate)"+(endTime - startTime));
    		even = !even;
    	}
    }

    public void PlanRoute()
    {
        var robotPos = GridUtils.WorldToGrid(robot.transform.position);

        var pathfinder = new AStarPathfinder(mapWidth, mapHeight, walkableMap);
        var planner = new RoutePlanner(pathfinder);

        var result = planner.FindBestPath(robotPos, passengers);

        if (result.HasValue)
        {
            var (nodePath, order) = result.Value;
            Vector3[] unityPath = order.Select(n => GridUtils.GridToWorld(n.pos.x, n.pos.y)).ToArray();
            navManager.SetWaypoints(unityPath);
        }
        else
        {
            Debug.LogWarning("No valid path found.");
        }
    }
}

public static class GridUtils
{
    public static float cellSize = 0.1f; // Size of one grid cell in Unity units

    public static Vector3 GridToWorld(int x, int y)
    {
        return new Vector3(x * cellSize, 0, y * cellSize);
    }

    public static (int x, int y) WorldToGrid(Vector3 pos)
    {
        return (Mathf.RoundToInt(pos.x / cellSize), Mathf.RoundToInt(pos.z / cellSize));
    }
}
