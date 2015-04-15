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


