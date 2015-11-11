# example_sensor_guided_motion
This package illustrates how to combine sensor information and Cartesian motion control.  It combines elements of baxter_cartesian_moves/example_baxter_cart_move_action_client (for Cartesian motion) and cwru_pcl_utils/cwru_pcl_utils_example_main, which uses the cwru_pcl_utils library for pointcloud processing.

By selecting points in Rviz, via the "publish selected points" tool, a coordinate frame is constructed, with origin at the centroid of the selected points, a z-axis normal to the selected patch, an x-axis along the major axis of the selected points, and a y-axis constructed as a consistent right-hand frame.  This frame is converted to a geometry_msgs::PoseStamped and sent to the baxter_cart_move_as action server as a goal pose for a Cartesian move.  This invokes a Cartesian motion that ends up aligning the gripper frame origin with the patch centroid, orients the gripper z-axis anti-parallel to the patch, and orients the gripper frame x and y axis in the plane of the patch, with the x-axis oriented along the major axis of the selected points.


## Example usage
* start up and enable the robot (or simulator)

`roslaunch cwru_baxter_sim baxter_world.launch` (or start real robot)

* wait for the robot (or Gazebo) to finish coming up; then enable the robot with:

`rosrun baxter_tools enable_robot.py -e` 

* start up the action servers and transform publishers with the following 6 commands in separate terminals

`rosrun baxter_traj_streamer  traj_interpolator_as`

`rosrun baxter_cartesian_moves baxter_cart_move_as`

`roslaunch cwru_baxter_launch yale_gripper_xform.launch` (to see gripper frame in rviz)

Watch out for the following; kinect transform is different for Gazebo vs real Baxter
`roslaunch cwru_baxter_sim kinect_xform.launch`

* start up rviz and the example sensor-guided motion node
`rosrun rviz rviz` (and set display to see kinect/depth/points and gripper frame)

`rosrun example_sensor_guided_motion example_sensor_guided_motion_client`

*OR: after starting and enabling the robot, run the following launch file:

`roslaunch example_sensor_guided_motion  baxter_sensor_guided_motion.launch`

## Running tests/demos
    
