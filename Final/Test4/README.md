# Test 4: 
The digital environment will receive a simulated passenger. We expect the software to recompute the route to all passenger locations and destinations, with the latency between the digital environment registering the new passenger and the new route being recomputed being within 1000 ms.

## Prerequizites:
- Default Visualizer *prefab* (nav_msgs/OccupancyGrid... + path)
- PathPlanningManager *script*
- AutoNavigationManager *script* 

*(For rerouting without actually acting on the new route comment line **59** in PathManager) Also might be possible to run just bring up?*

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