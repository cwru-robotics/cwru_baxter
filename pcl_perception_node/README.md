# pcl_perception_node

A start towards some work on perceptual processing of 3-D point clouds.
This is a work in progress, and it needs completion, testing and lots of refactoring.

Intro programs to illustrate pointcloud processing are pcd_save and pcd_transform_and_save.cpp.  The first acquires Kinect data and saves a snapshot to disk as a PCD file, snapshot.pcd.  The second acquires Kinect data, transforms it to the torso frame of Baxter and saves it to disk as snapshot_wrt_torso.pcd.

## Example usage
The save_pcd node only requires that a Kinect stream is being published on topic "/kinect/depth/points".  This could be a physical Kinect or simulated. To get simulated data, run:
`roslaunch cwru_baxter_sim baxter_world.launch`

 Navigate in a terminal to a directory to store a snapshot, and:
  `rosrun pcl_perception_node pcd_save`

pcd_transform_and_save additionally requires transform data relating the Kinect sensor frame to the torso.  This can be done with the baxter simulator by running:
`roslaunch cwru_baxter_sim baxter_world.launch`

Additionally, a transform publisher is needed to relate the Kinect sensor frame to the Kinect link frame.  This can be done with:
`roslaunch cwru_baxter_sim kinect_xform.launch`

With these running, the example program can be executed.  Navigate to the directory in which to store the saved pcd file and run:
`rosrun pcl_perception_node pcd_transform_and_save`

This will produce the file "snapshot_wrt_torso.pcd"

    
