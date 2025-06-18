# Test 6: 
The TurtleBot will be sent a new route during operation. We expect it to follow the new route, with the digital environment showing the twin following the new route as well.

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