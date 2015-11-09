# baxter_test_IK

This package illustrates using the baxter_kinematics library and the baxter_traj_streamer library.
Starting from an arbitrary pose of the right arm, the hand frame is found with respect to the torso frame, using a tf_listener.  This value is used to double-check baxter forward kinematics, as computed via a Baxter_fwd_solver object. 

Given this hand pose, a vector of arm-pose solution alternatives is computed that preserves this hand pose.  These solutions are computed using a Baxter_IK_solver object.  

A  Baxter_traj_streamer object is used to convert paths to feasible trajectories.

A simple action client sends trajectory goals to the trajectory interpolation action server. 

## Example usage
`roslaunch cwru_baxter_sim baxter_world.launch`
`rosrun baxter_tools enable_robot.py -e` (runs to completion)
`rosrun baxter_traj_streamer traj_interpolator_as`(action server stays alive)
`rosrun baxter_test_ik baxter_test_ik`  (runs to completion after many motion commands)

    
