# Test 3: 
The digital environment will receive a simulated obstacle. We expect the software to recompute the route to the destination, with the latency between the digital environment registering the obstacle and the new route being recomputed being within 500 ms.

## Prerequizites:
- Laser Scan Obstacle Visualizer *script*
- LaserCube *prefab*
- DigitalObject *prefab*

### Turtlebot
```bash
ssh ubuntu@192.168.8.40
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

### Laptop (ros2_ws)
```bash
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.8.225
```