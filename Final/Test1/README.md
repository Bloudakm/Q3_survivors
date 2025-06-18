# Test 1: 
The TurtleBot will be tasked to scan the environment using its LiDAR sensor. We expect it to output the layout of the test map.

## Prerequizites:
- Default Visualizer *prefab* (nav_msgs/OccupancyGrid...)
- Map.yaml *weird ahhh map*
- burger.yaml

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
Then we can move around with teleop:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```


#### In case of issues
https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node