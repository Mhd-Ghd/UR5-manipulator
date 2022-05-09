==============================[prerequisites]===================================
[-] System: 		Ubuntu 16.04
[-] Ros Distrbution: 	kinetic

==========================[runnig the requirments]==============================
To launch the gazebo simulator: 
	roslaunch ur5_gazebo ur5_cubes.launch

To run the Ros-Manipulator Node:
	rosrun ur5_gazebo inv-pose.py 


The inv-pose.py script availble in 
	UR5-manipulator/ur5_gazebo/scripts

==============================[How to use?]=====================================

1- Launch the ur5 gazebo world with cubes, using:
	 roslaunch ur5_gazebo ur5_cubes.launch

2-run the UR5-manipulator ROS node, as follows:
	rosrun ur5_gazebo inv-pose.py

3- to append a waypoint enter (p) then press enter.

4- enter the waypoint in the form x y z seperated by spaces, i.e.:
	0.2 0.2 0.3

5- toggle the mode to see the robot moving by entering (t) then enter.

6- The robot starts from home position pointing up.

7- enter (n) to move to next waypoint or (p) to previous waypoint.

8- the arm shall be moving, press ctrl+c to terminate.

