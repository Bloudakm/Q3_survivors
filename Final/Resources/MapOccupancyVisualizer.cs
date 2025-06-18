using UnityEngine;
using System.Collections.Generic;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class MapOccupancyVisualizer : MonoBehaviour
{
    public string mapTopic = "/map";
    public GameObject occupiedCellPrefab;
    public float cellHeight = 0.05f;

    ROSConnection ros;
    List<GameObject> spawnedCells = new List<GameObject>();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OccupancyGridMsg>(mapTopic, MapCallback);
    }

    void MapCallback(OccupancyGridMsg map)
    {
        ClearMap();

        int width = (int)map.info.width;
        int height = (int)map.info.height;
        float resolution = map.info.resolution;

        Vector3 origin = new Vector3(
        	-0.4f,//(float)map.info.origin.position.x, 
        	0.0f,//(float)map.info.origin.position.y, 	
        	-0.4f//(float)map.info.origin.position.z
        ).To<FLU>().toUnity;

	//Debug.Log("Origin: "+origin);

        Quaternion rotation = new Quaternion(
        	(float)map.info.origin.orientation.x, 
        	(float)map.info.origin.orientation.y, 	
        	(float)map.info.origin.orientation.z,
        	(float)map.info.origin.orientation.w
        ).To<FLU>().toUnity;
        //map.info.origin.orientation.To<FLU>();

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int idx = y * width + x;
                int occVal = map.data[idx];

                if (occVal < 50) continue; // Only show occupied cells

                Vector3 localPos = new Vector3(x * resolution, 0, y * resolution);
                Vector3 worldPos = origin + rotation * localPos;
                

                GameObject cell = Instantiate(occupiedCellPrefab, worldPos + Vector3.up * cellHeight / 2, Quaternion.identity);
                cell.transform.localScale = new Vector3(resolution, cellHeight, resolution);
                spawnedCells.Add(cell);
            }
        }
    }

    void ClearMap()
    {
        foreach (var obj in spawnedCells)
            Destroy(obj);
        spawnedCells.Clear();
    }
}

