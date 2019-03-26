Add the video file into Drovi/mini-project2/src/video_stabilizer_node/data
	Currently the programs opens and streams files called: 2017_06_23_dyrskuepladsen.mp4
	If needed you can change the file name in /Drovi/mini-project2/src/video_stabilizer_node/launch/test.launch

To build the program open the terminal in dir /Drovi/mini-project2/
	build the catkin workspace: catkin build
	source the ROS setup files: source devel/setup.bash

OBS! You need to source the ROS framework whenever you open a new terminal 

to run the videostream and stabilizer: 
	roslaunch video_stabilizer_node test.launch 
	rosrun miniproject2 "insert python file here"

To create a new ros node, simply create a python file and rebuild the catkin workspace



