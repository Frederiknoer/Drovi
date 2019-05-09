
echo "Source'ing devel/setup.bash"
source ./devel/setup.bash

echo "Copied rovi.world"
cp ./setup_files/rovi.world ./src/rotors_simulator/rotors_gazebo/worlds

echo "Copied rovi.launch"
cp ./setup_files/rovi.launch ./src/rotors_simulator/rotors_gazebo/launch

echo "Prepared a hummingbird, mounted camera and IMU"
cp ./setup_files/hummingbird_base.xacro ./src/rotors_simulator/rotors_description/urdf

echo "Prepared marker for detection"
cp -r ./rmurv2_marker ./src/rotors_simulator/rotors_gazebo/models
catkin build

echo "To run do the following: "
echo "1. source ./devel/setup.bash "
echo "2. roslaunch rotors_gazebo rovi.launch "
