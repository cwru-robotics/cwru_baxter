example program to command a position to a Baxter joint via C++

Example program:  wsn_test_baxter_cmd.cpp
defines joint names for left and right arms;

put joint-angle commands in: right_cmd.command[i], etc, to command joint-angle motion
(numbering is from first shoulder jnt = 0 to last wrist jnt = 6)

Example program sends sinusoids to all joints, both arms.

TO COMPILE:
navigate to the ros_ws directory and enter:
catkin_make

TO RUN THIS PROGRAM:

(might need to first run, from ros_ws: ./baxter.sh sim)

start up Baxter simulator:  roslaunch baxter_gazebo baxter_world.launch

(if fails, do: ps -aux | grep gzserver, find the process ID of the gzserver, then
  kill -9 PID, where "PID" is the process number of the gzserver; then try again to start gazebo)

still need to learn how to enable Baxter from within C++.  Present work-around is:

rosrun baxter_examples joint_position_keyboard.py

better: `rosrun baxter_tools enable_robot.py -e`

This code enables the joints for position control, then waits for keyboard input.  Simply shrink this window 
(keeping it alive), then run the demo C++ node:

rosrun wsn_test_baxter_cmd wsn_test_baxter_cmd_node

Baxter will waive his arms wildly. 

DEMO ON BAXTER:  this simple program sends Baxter to a pre-defined pose; worked on physical arms
 from ros_ws: `./baxter.sh`
`rosrun baxter_tools enable_robot.py -e`
`rosrun wsn_test_baxter_cmd wsn_test_baxter_cmd2`




