# Test 2: 
The TurtleBot will be tasked to detect an obstacle and send this data to the digital environment. We expect the latency between the sensor and the data getting to the digital environment to be within 200 ms.

## Prerequizites:
- Laser Scan Obstacle Visualizer *script*
- Default Visualizer *prefab* (sensor_msg/Laser...)
- LaserCube *prefab*

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