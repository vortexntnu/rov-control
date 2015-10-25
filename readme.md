Requires xboxdrv

To use this ROS package you have to prepare your ros workspace for it.

Run 'catkin_create_pkg controller' in ~/catkin_ws/src to set up the package.
Then remove package.xml and CMakeLists.txt from the package.
Now clone this repo and copy all the files into the controller ros package (NB remember the .git and .gitignore files).
Build the package with 'catkin_make' in ~/catkin_ws
You might have to run 'source ~/catkin_ws/devel/setup.bash'
Start the ros node with 'rosrun controller ctrl_node.py'

You just have to create the package manually once. Now you can just git push and pull from there.

