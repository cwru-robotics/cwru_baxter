This folder intended to work either with real or simulated baxter.
For real baxter, need to set ROS_MASTER_URI.
Can do this with command line: `export ROS_MASTER_URI=http://baxter01:11311`

OR, do (from this directory): `source set_remote_master.sh`

Should then be able to do: rostopic list (from the terminal with ROS_MASTER_URI set).

For all of the items below, make sure the terminals have defined ROS_MASTER_URI

Start the Kinect sensor w/:
`roslaunch freenect_launch freenect.launch rgb_frame_id:=kinect_pc_frame depth_frame_id:=camera_depth_optical_frame`

Start cwru nodes (including rviz) with:
`roslaunch cwru_baxter_nodes.launch`

Start the camera transform with:
`roslaunch cwru_baxter_launch kinect_transform.launch`

In rviz, set: fixed frame = base,
PointCloud2 topic to: camera/depth_registered/points

Edit kinect_transform.launch to calibrate transform






