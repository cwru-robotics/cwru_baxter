# cwru_baxter
This repository is for CWRU-developed Baxter code.
 

#### Baxter start-up notes (per Luc Bettaieb)
We got baxter working! Woohoo! Now let's set him up for development.

1) Make sure that Baxter's power, e-stop, and network cable are all plugged in and ready to go.

2) Turn baxter on by pressing the power button on his side.

3) Wait for a while until the LED halo around his head is GREEN.

4) Now you're ready to enter the development environment.

5) On the development workstation, open up a new terminal window

6) Navigate to ~/ros_ws

    `cd ~/ros_ws`

7) Initiate the baxter environment

    `./baxter.sh`

8) Now you can see all baxter's topics by doing a rostopic list

9) You can check environment variables by doing

    `env | grep ROS`

10) Fun example command: `rosrun baxter_examples joint_velocity_wobbler.py`

#### Example C++ code (per Wyatt Newman)
In the sub-package wsn_test_baxter_cmd, see the sample C++ code and the README for how to execute 
C++ based arm motion control.

#### Baxter simulation w/ CWRU nodes:
start-up notes:

Start the Gazebo simulator of Baxter: (use optirun w/ bumblebee, if needed)
(optirun) roslaunch baxter_gazebo baxter_world.launch

Start the ROS visualization interface:
rosrun rviz rviz

Start up custom nodes...
REACHABILITY:
rosrun reachability reachability_from_above_v2

(manual test... rosservice call compute_reachability_svc 0.0
(shows reachability as marker topic /reachability_from_above_marker for specifyed z-height)

TRAJ INTERPOLATOR:
rosrun baxter_traj_streamer traj_interpolator_as 
(test/prepare baxter w/: rosrun baxter_traj_streamer traj_action_client_pre_pose)

CARTESIAN MOVE INTERFACE:
rosrun cartesian_moves arm_motion_interface

ARM MOTION INTERFACE:
rosrun cartesian_moves arm_motion_interface
 presents service: cartMoveSvc
 see: cartesian_move_svc_client_test for example of how to interface to arm_motion_interface

INTERACTIVE MARKER:
rosrun interactive_marker_node interactive_marker_node
 test w/: 

3D PERCEPTION NODE:
rosrun pcl_perception_node pcl_perception_node2

COORDINATOR:
rosrun coordinator coordinator

SIMPLE GUI:
~/ros_ws/src/cwru_baxter/simple_GUI$ ./service_call_gui
