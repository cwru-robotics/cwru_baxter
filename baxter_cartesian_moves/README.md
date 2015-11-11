# cartesian_moves
This package provides a Cartesian path-planning library (which uses cwru_joint_space_planner)
 and an action server for performing Cartesian-space moves with Baxter.  At present, only
right-arm commands are functional, and gripper actions are not yet implemented.  

Movement options are defined as command codes within the action message:
cwru_action/cwru_baxter_cart_move.action. 

An example client is given in: example_baxter_cart_move_action_client.
This illustrates sending commands for:
* plan a joint-space path to a hard-coded pre-pose and execute it
* query the server for joint states (joint angles)
* query the server for the current tool-frame pose, w/rt torso
* plan and execute a joint-space trajectory from current pose to specified joint-state pose
* plan and execute a cartesian path from current pose to some specified tool-frame pose
* plan and execute a cartesian relative move from current pose along a specified vector, at fixed gripper orientation

## Example usage
`roslaunch cwru_baxter_sim baxter_world.launch` (or start real robot)
`rosrun baxter_traj_streamer  traj_interpolator_as`
`rosrun baxter_tools enable_robot.py -e`
`roslaunch cwru_baxter_launch yale_gripper_xform.launch` (to see gripper frame in rviz)
`rosrun baxter_cartesian_moves baxter_cart_move_as`
`rosrun baxter_cartesian_moves example_baxter_cart_move_action_client`

    
