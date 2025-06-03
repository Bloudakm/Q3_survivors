using UnityEngine;
using System.Collections.Generic;
using System.Collections;

public class RandomSpawn : MonoBehaviour
{
    public GameObject cubePrefab;
    public Transform grid;
    private int maxX;
    private int maxZ;
    private const int yLevel = 3;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        maxX = grid.size.x;
        maxZ = grid.size.z
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.GetKeyDown(KeyCode.Space)) {
            Vector3 randomPos = new Vector3(Random.Range(-maxX, maxX), yLevel , Random.Range(-maxZ, maxZ));
            Instantiate(cubePrefab, randomPos, Quaternion.identity);
        }
    }
}
