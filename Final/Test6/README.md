# Test 6: 
The TurtleBot will be sent a new route during operation. We expect it to follow the new route, with the digital environment showing the twin following the new route as well.

## Prerequizites
- Default Visualizer *prefab* (nav_msgs/OccupancyGrid... + path)
- PathPlanningManager *script*
- AutoNavigationManager *script*

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
```bash
ros2 launch unity_slam_example unity_slam_example.py params_file:='/home/ubuntuhost/Downloads/2IRR10UnityProject/My project/Assets/turtlebot3-foxy-devel/turtlebot3_navigation2/param/burger.yaml'
```
or
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```