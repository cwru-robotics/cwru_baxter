This folder contains modified Baxter world, modified Baxter URDF (w/ Kinect),
and two launch files to facilitate Baxter demo.

In first window, run:
(optirun) `roslaunch cwru_baxter_sim baxter_world.launch`

wait for "gravity compensation was turned off"

In second terminal, run:
`roslaunch cwru_baxter_sim cwru_baxter_sim.launch` 

In third terminal,
`roscd simple_gui`
`./service_call_gui`

To use Yale hand, (in 4th terminal):
`roslaunch motor_controller yale_hand.launch`

If race condition, then do the above via separate launches:
`roslaunch motor_controller controller_manager.launch`
(wait) and
`roslaunch motor_controller tilt_controller.launch`
(runs to completion)
