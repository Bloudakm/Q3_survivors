using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotController : MonoBehaviour
{
public Transform target;
    public float speed = 2.0f;
    public float rotationSpeed = 5.0f;
    private List<Node> path;
    private int currentWaypointIndex = 0;

    void Update()
    {
        if (path != null && currentWaypointIndex < path.Count)
        {
            Vector3 targetPosition = path[currentWaypointIndex].worldPosition;
            Vector3 direction = (targetPosition - transform.position).normalized;

            // Move forward
            transform.Translate(direction * speed * Time.deltaTime, Space.World);

            // Rotate toward target
            Quaternion lookRotation = Quaternion.LookRotation(direction);
            transform.rotation = Quaternion.Slerp(transform.rotation, lookRotation, rotationSpeed * Time.deltaTime);

            // Check if reached waypoint
            if (Vector3.Distance(transform.position, targetPosition) < 0.1f)
            {
                currentWaypointIndex++;
            }
        }
    }

    public void SetPath(List<Node> newPath)
    {
        path = newPath;
        currentWaypointIndex = 0;
    }
}
