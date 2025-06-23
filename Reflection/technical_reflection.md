# Technical reflection
*Team 25 Q3-Survivors*

## Bidirectional communication
Bidirectional communication was one of our largest bottle necks during the whole process. At the start we managed to get some of the most basic examples like the *teleop keyboard* to work, however we were soon halted in our progress by the subscription to the **/scan** topic. While we were able to comunicate well towards the turtlebot from Unity, using the **/cmd_vel** topic for example we were stuck on making the subscriber work. At first we thought that it was an issue with our scripts and the syntax, later at the end of one lab session we were told that it could be the fact that the previous team did not shutdown the robot correctly. This unfortunately turned out to also not be the case, as the following lab session was dedicated almost solely to the instructors trying to fix our issue, but failing to do so. We finally managed to fix it by adjusting the **QoS** settings for the subscriber in the *TCPConnector*. While we fixed it, the time poured into it, especially time with the bot was definetely too much and it affected our ability to do what we wanted greatly. Apart from the lidar subscriber not working the rest of bidirectional communication went relatively well for us. 

## Real time synchronization
While we encountered quite some issues regarding the real-time sync, they were in most cases not as severe. The most common one that was present almost during our entire project was the precision of how the digital twin copied the real turtlebot's movement. Since it is very hard to get the material friction and other physics of the situation exhaustively captured in the Unity simulation this issue was bound to happen. We tried adjusting some of these paramenters between the labs, however the results were not really persuasive, so we decided that we did not have the resources and time to fully get rid of these uncertainties and errors and we would have to accept them. Apart from this very common issue, we also encoured a rather strange one which was the clock synchronization between the laptop and the Turtlebot. At the begining it seemed that it was once again an issue in our Unity scripts, but that was not the case becasue even the tutorials with Gazebo and similar tools were failing and giving a message along the line of *"data too old"*. This put us in a somewhat tricky situation, as it occured late into the development stage, since the time we lost on it was more valuable than the time at the very begining. 

## Environmental & Object Interaction


## Future plans


#### Laptop credentials:
- user: team25
- password: $Q3Survivors25*