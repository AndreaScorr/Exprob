EXPERIMENTAL ROBOTICS LABORATORY 
=================================
Andrea Scorrano
id: 6463777

Has been developed the assignment in two different packages, for the physical implementation (robot model, configuration files, urdfs file and launch files) the package is robot_urdf.
For nodes implementation like ( moving_robot, aruco nodes etc...) the package is ros2_aruco
 

Running
----------------------

In order to execute the first part of the assignment moving only the robot to see the marker write the following instructions

```bash
root@ab7d7c31fa8a:~/experimental_robotics# ros2 launch robot_urdf gazebo_moving_robot.launch.py
```

In order to execute the second part of the assignment moving the camera write the following instructions. One for the launch file and one for the aruco node (it's better to execute in this way to avoid glitch and missed markers)


```bash
root@ab7d7c31fa8a:~/experimental_robotics# ros2 launch robot_urdf gazebo.py
root@ab7d7c31fa8a:~/experimental_robotics# ros2 run ros2_aruco aruco_node_moving_camera 

```

## Flow Charts ##
In both case the logic is the same. The robot will move arount itself, detect de marker and update the dictionary with the id and the position that he get from the odom topic or from the position that we send to the controller. To get more details i suggest to look the comments of the moving_robot.py, aruco_node_moving_robot.py ,aruco_node_moving_camera.py in the aruco_node folder. 

### First Part Moving only the robot ###
![alt text](https://github.com/AndreaScorr/Exprob/blob/main/Movin_Robot.drawio.png?raw=true)




### Second Part Moving only the camera ###

The camera has been controlled by position.

![alt text](https://github.com/AndreaScorr/Exprob/blob/main/Moving_Camera.drawio.png?raw=true)

