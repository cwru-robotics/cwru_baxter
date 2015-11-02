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

In first window, run:
(optirun) `roslaunch cwru_baxter_sim baxter_world.launch`

wait for "gravity compensation was turned off"

In second terminal, run:
`roslaunch cwru_baxter_sim cwru_baxter_sim.launch` 

In third terminal,
`roscd simple_gui`
`./service_call_gui`
The simple GUI will offer a button to press to enable the robot.

### Baxter (physical) start-up (wsn, 11/2015)
As above,
1) Make sure that Baxter's power, e-stop, and network cable are all plugged in and ready to go.

2) Turn baxter on by pressing the power button on his side.

3) Wait for a while until the LED halo around his head is GREEN.

4)  from the development workstation, open a terminal and run:
`export ROS_MASTER_URI="http://011503P0031:11311"`
This will set up Baxter as the ROS "master", and your workstation will use the roscore that is running on Baxter.

5) enable the Baxter robot to respond to motion commands with:
    `rosrun baxter_tools enable_robot.py -e`
(or launch the simple GUI, as above, and use the "enable" button)

6) Start the joint-interpolation action server:
 `rosrun baxter_traj_streamer traj_interpolator_as`

Baxter is now ready to receive goal requests from a client (see, e.g. baxter_traj_streamer/traj_action_client_pre_pose)

 

