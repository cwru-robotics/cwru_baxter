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
