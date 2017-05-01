# RBE3002_Mag_Men
Repo for the Magnificent Men group of RBE3002 (Richard Cole, Jonathan Tai)

To run the final project:
1) Download the msg folder, Node.py, driveTurt_4.py, lab3_config.rviz and lab3_1_4.py and move them into a package in your catkin workspace
2) Ensure that all of the aforementioned files are executable
3) Run catkin_make on your catkin workspace
4) Change line 12 in lab3_1_4.py from "from rpcole_lab2.msg import Waypoint" to "from < name of the package that the files are in>.msg import Waypoint"
5) Change line 10 in driveTurt_4.py from "from rpcole_lab2.msg import Waypoint" to "from < name of the package that the files are in>.msg import Waypoint"
6) Connect to the Turtlebot and run "roslaunch turtlebot_bringup minimal.launch" and "roslaunch turtlebot_navigation gmapping_demo.launch"
7) On your computer run "rosrun rviz rviz" and open the lab3_config.rviz config file
8) On your computer run "rosrun < name of the package that the files are in> driveTurt_4.py" and wait until you see "has position" appear
9) On your computer run "rosrun < name of the package that the files are in> lab3_1_4.py"
