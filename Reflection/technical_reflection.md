# Technical reflection
*Team 25 Q3-Survivors*

## Bidirectional communication
Bidirectional communication was one of our most significant bottlenecks during the whole process. At the start we managed to get some of the most basic examples like the *teleop keyboard* to work, however we were soon halted in our progress by the subscription to the **/scan** topic. While we were able to comunicate well towards the turtlebot from Unity, using the **/cmd_vel** topic, for example, we were stuck on making the subscriber work. At first we thought that it was an issue with our scripts and the syntax, later at the end of one lab session we were told that it could be the fact that the previous team did not shutdown the robot correctly. This unfortunately turned out to also not be the case, as the following lab session was dedicated almost solely to the instructors trying to fix our issue, but failing to do so. We finally managed to fix it by adjusting the **QoS** settings for the subscriber in the *TCPConnector*. While we fixed it, the time poured into it, especially time with the bot was definetely too much and it affected our ability to do what we wanted greatly. 

Once the QoS issues were resolved, we achieved true bidirectional communication: Unity was able to receive LiDAR data and reflect it in the virtual twin; Unity could continue sending movement commands to the robot; the flow of data was now real-time in both directions.

We did not achieve internal hardware state communication (e.g., battery levels, hardware health or actuator faults), which is part of the extended expectation for digital twins, as well as failure states or system health telemetry between twin and robot.

Future work: With an additional 5-8 weeks, we could extend ROS2 bridge to include diagnostics and robot_state_publisher data; implement mirroring of sensor or hardware failures in Unity and add middleware for internal state alerts to simulate real-world system health degradation.



## Real time synchronization
While we encountered quite some issues regarding the real-time sync, they were in a lot of cases not as severe, apart from one. The most common one that was present almost during our entire project was the precision of how the digital twin copied the real turtlebot's movement. Since it is very hard to get the material friction and other physics of the situation exhaustively captured in the Unity simulation this issue was bound to happen. We tried adjusting some of these paramenters between the labs, however the results were not really persuasive, so we decided that we did not have the resources and time to fully get rid of these uncertainties and errors and we would have to accept them. 

Apart from this very common issue, we also encoured a rather strange one which was the clock synchronization between the laptop and the Turtlebot. At the begining it seemed that it was once again an issue in our Unity scripts, but that was not the case becasue even the tutorials with Gazebo and similar tools were failing and giving a message along the line of *"data too old"*. This put us in a somewhat tricky situation, as it occured late into the development stage, since the time we lost on it was more valuable than the time at the very begining. 

Summary of achievements: basic real-time mirroring of position and orientation from TurtleBot to Unity; commands from Unity reflected immediately in physical robot motion.

Not achieved: perfect match in movement physics between real and digital twin.

Future work: With additional 5-8 weeks, we could introduce timestamp filtering or use message_filters to better handle latency; fine-tune Unity physics with measured real-world motion profiles, and use Unity's Rigidbody constraints to stabilize simulation alignment with real-world values.


## Environmental & Object Interaction
As mentioned in the bidirectional communication section the issues with the **/scan** topic obviously propagated even into our Evironmental and Object Interaction, since without eyes (the lidar) one can't really see nor avoid obstacles. We also encountered some issues when using the Nav2 nodes, mostly due to the costmap creation. At first we obviously ran into the problem that our environment was too small for the default *inflation-radius*, meaning the bot would get stuck in places, since it had no route to get where it needed to get, or the route was too thin, so it could not work for the bot-radius. After we adjusted the parameters in the *burger.yaml* file most of these issues went away, but there were still some present, such as that sometimes when the robot was relatively close to a wall/obstacle it would start the *"recovering"* sequence. We did not get to the exact root of this issue, but we suspect that it was mostly due to the fact that we simply placed the bot too close to the obstacle to begin with. Another issue we came across during the purely digital testing was that the premade simulated laser-scaner was weirdly interfacing with the whole mapping system. It did work and made a pretty solid map, but the issue was that it's size did no match the unity environment, because it was somehow scaled down. As we encoutered this issue only digitally we fixed it by adjusting the colliders of the simulated walls, which worked quite well.

Achieved: real-time obstacle detection and rerouting using LiDAR and Nav2 (after resolving /scan subscription); adjusted parameters to enable navigation in small spaces; reconciled simulated laser scan range with Unity collder scale.

Not achieved: environmental interaction in a fully integrated system.


## Future plans
With 5 to 8 more weeks for futher development, we would definitelly implement some additional features. To start with, we would like to interface more with all the other informational topics about stuff like the battery state. This would allow us to implement a more robust solution to the problem, as it would be able to withstand the edge cases of for example a drained battery, which would mean that the bus (turtlebot) has to finish all of its duties as soon as possible and return to a place designeted for charging. 

Additionally there is a lot we could add to our algorithm implementation. We got it to work quite well and recalculate the routes in way that worked but there could be a lot more optimization. Firstly our implementation did not fully account for multiple objects being added, which would complicate the situation too much. This was mostly due to the fact that our custom algorithm handled only preparing the order in which we visit the locations (you can't drop off a passanger you did not pick up yet etc.) and than the interaction with the obstacles and environment was left to the in-built nav2 nodes. Moreover we could start implementing the network model, because in our grand technical solution one of the key aspects is that the buses operate as a fleet together (which we can't showcase since we only have one turtlebot), so the algorithm for that could also be implemented and perhaps tested somewhat digitally. 

Lastly it would also be very helpful to add more tests to fully examine if our algorithm is actually as optimal as we think it is, because for example in the current set up we did not really test how optimal the algorithm becomes if multiple obstacles are added into the path.

#### Laptop credentials:
- user: team25
- password: $Q3Survivors25*

#### Our Github repository 
https://github.com/Bloudakm/Q3_survivors
