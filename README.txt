Chris Wheatley
University of Maryland (College Park)
ENPM661 Spring 2020
Dr. Monfaredi
Project #3 - Phase #3

Project Description:
These programs leverage the A* seraching algorithm, as well as nonholonomic differential-drive constraints, to explore an action space and naviagte obstacles to generate an optimal path, for a rigid robot.

The project contains the following files: 
	-Astar_rigid_diff_drive.m
	-obstacleCheckRigid.m
	-changeAngle.m
	-simulation.mp4
	-README.txt
	

User Instructions:
1) Ensure that MATLAB is installed on your machine (2017b or later).

2) Download all project contents and unzip the "proj3p3_group2_matlab.zip" contents into a newly created local directory.

3) In MATLAB, change the working directory to the working directory created in step #2.

4) Run"Astar_rigid_diff_drive.m"
	-A prompt will appear in the MATLAB command window asking you to enter location (x and y in meters) and orientation (degrees) of starting node. Enter it between brackets and with column separation (Ex: "[5,5,60]")
	-Another prompt will appear asking to enter location (x and y in meters) of goal node. Enter it between brackets and with column separation (Ex: "[150,150]")
	-Another prompt will appear asking you to enter the robot's obstacle clearance (meters). Enter any number.
	-Another prompt will appear asking you to enter the robot's left/right wheel RPMs (revolutions per minute). Enter it between brackets and with column separation (Ex: "[14,5]")
	-If there is conflict with these points, the program will prompt you again, so please enter new values.
	-If the RPMs are either less than zero, or too high thus resicting exploration ability, a warning message will appear and thee program will stop. 

5) A MATLAB figure (.fig) should appear and be updating in real time, drawing blue lines/curves from node to node in a branching/blooming motion.  Obstacles are outlined in black.
6) Once the program finishes running, node exploration should stop, and the optimal path should appear colored in green.
7) The pink circle is the start node and the pink triangle is the goal node.  If you would like to zoom in, select the + magnifying glass the top ribbon of figure and use your mouse to zoom.


For the scenaio shown in the attached video "simulation.mp4" :
	- Start Node: [-4.5,-4.5,0] 
	- Goal Node: [4.5,4.5]
	- Clearance: 0.1
	- Wheel RPMs: [40, 20]
	- Hard-coded assumptions:
		o Simulation stops when robot comes within a distance of 0.5 meters or less to the goal.