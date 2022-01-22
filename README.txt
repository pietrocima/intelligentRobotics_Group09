All the necessary files for the correct execution of the tasks are contained in the tiago_iaslab_simulation package.

Instructions HOMEWORK 2:
1-Clone the package in the src folder of the workspace
2-Build the workspace
3-Launch: roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full
4-Launch: roslaunch tiago_iaslab_simulation navigation.launch
5-Launch: roslaunch tiago_iaslab_simulation apriltag.launch
6-Run: rosrun tiago_iaslab_simulation human_node 
7-Run: rosrun tiago_iaslab_simulation moveTiago_server
8-Run: rosrun tiago_iaslab_simulation nodeB_Detection 
9-Run: rosrun tiago_iaslab_simulation nodeC_moveArm
10-Run: rosrun tiago_iaslab_simulation nodeA 

Alternatively, it is possible to run the whole procedure from a single terminal using the following command, but all the information outputted by the nodes will not be visible in the terminal. 
	roslaunch tiago_iaslab_simulation hw2.launch




Instructions HOMEWORK 1:
1-Clone the package in the src folder of the workspace
2-Build the workspace
3-Run roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full
4-Run roslaunch tiago_iaslab_simulation navigation.launch
5-Launch the server: rosrun tiago_iaslab_simulation moveTiago_server
6-Launch the client: rosrun tiago_iaslab_simulation moveTiago_client x y theta

In the last command three parameters (floats) have to be given for the code to work as intended.
The first two parameters are the coordinates, expressed in meters with respect to the map frame, of the point that the robot has to reach.
The third parameter is  the angle of rotation around the z-axis, expressed in radians, where the 0 corresponds to the direction of the x-axis in the map frame.

To make the laser_geometry package work, we had to modify the file opt/ros/melodic/include/laser_geometry/laser_geometry.h by substituting "#include <Eigen/Core>" with "#include <eigen3/Eigen/Core>"
