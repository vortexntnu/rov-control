Requires xboxdrv

To add package simply clone into src directory.
Build the package with 'catkin_make' in ~/catkin_ws
You might have to run 'source ~/catkin_ws/devel/setup.bash'
In order for the package to be visible for ROS you might also have to run 'rospack profile' 
Start the ros node with 'rosrun controller ctrl_node.py'

You probably also want to rename the folder to joystick rather than controller...
