using UnityEngine;
using System.Collections.Generic;
using System.Collections;

public class RandomSpawn : MonoBehaviour
{
    public GameObject cubePrefab;
    private int maxX = 10;
    private int maxZ = 10;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    /*void Start()
    {
        
    }*/

    // Update is called once per frame
    void Update()
    {
        if(Input.GetKeyDown(KeyCode.Space)) {
            Vector3 randomPos = new Vector3(Random.Range(-maxX, maxX), 3 , Random.Range(-maxZ, maxZ));
            Instantiate(cubePrefab, randomPos, Quaternion.identity);
        }
    }
}
