# Test 4: 
The digital environment will receive a simulated passenger. We expect the software to recompute the route to all passenger locations and destinations, with the latency between the digital environment registering the new passenger and the new route being recomputed being within 1000 ms.

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