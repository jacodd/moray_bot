# Case Moray Bot #

Welcome to the Moray Bot Case. The challenge here is to make a robot navigate with a set of coordinates using ROS2 and the Gazebo simulation environment.
In this package, you will be given a differential robot that has already been configured with some sensors (LiDAR, GPS, and IMU). Odometry has been published, and so the GPS coordinates to the gazebo world.
The cmd_vel topic can be used to control de robot.

![Scheme](https://bitbucket.org/morayai/moray_bot/raw/c4d845f24f686d950bb4d297303790cd0889da30/media/moraybot_case.png)


## The Task ##
Your task is to make Moray Bot go through the numbered squares in the ground from 1 to 4 in ascending order, the Moray Bot spawns at the square number 1. You should record the GPS position of 
each square the robot needs to pass and save it in the file://config/gps_waypoints.yaml. After that, you should make the robot navigate autonomously through the saved points while using 
the LiDAR to avoid any obstacles in the way. Feel free to come up with your own solution, but using ROS2 packages are welcome as well.

Here is a video of the task you should accomplish:

![Alt Text](https://bitbucket.org/morayai/moray_bot/raw/c4d845f24f686d950bb4d297303790cd0889da30/media/moraybot_case_gif.gif)

### How do I get set up? ###

1. Use a Ubuntu 22.04 setup. You are set if you already have a PC running Ubuntu 22.04! Otherwise, here are some alternatives:
	
	* You can use Windows WSL2 as shown in this [tutorial](https://youtu.be/F3n0SMAFheM). Bear in mind that WSL2 doesn't output graphical interfaces, but as they are necessary to this case, then this [tutorial](https://jackkawell.wordpress.com/2020/06/12/ros-wsl2/) (just need to read the "Setting up GUI forwarding" part) on X11 forwarding will come handy.
	* You can also set up a virtual machine to run Ubuntu 22.04 
	
2. Install ROS2 Humble on your Ubuntu. This [tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) is a good starting point. 
3. Install Turtlebot simulation package ```sudo apt install ros-humble-turtlebot3-gazebo```
4. Add the Turtlebot models to the gazebo models path ```echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc```
3. Finally, you can create your ros2 workspace and place the provided files in the src folder.
4. After compiling and sourcing the workspace, you can run the following launch file to launch the moraybot in the gazebo world.
```bash
ros2 launch moray_bot gazebo_gps_world.launch.py
```
5. You can drive the Moray bot using the following command on another terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Delivery guidelines ###

1. After making the robot navigate through the square from 1 to 4, you should record a bag of your run using ```ros2 bag record -a ```
2. Also, you need to record the screen of the robot passing through the squares and send it to us. Here are two free software to record your screen [OBS Studio](https://obsproject.com/pt-br/download) and [Screen to GIF](https://www.screentogif.com/)
3. Send us the folder with your workspace containing all the files used to make the case. 

### Bonus Points (This is not mandatory) ###

* If you feel you can take an extra mile, then the challenge is to use docker to create a container with the case. 
* The container should have all the files and configurations to run the case without additional installations.

### Who do I talk to? ###

* If you have any technical questions or face any problems, please feel free to send me an email at rlc@moray.ai including bs@moray.ai as a copy.