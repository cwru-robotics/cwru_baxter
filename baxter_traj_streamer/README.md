wsn pgm to receive Baxter trajectories and interpolate them smoothly as commands to Baxter;

right arm only, at present; June 1, 2015

rosrun baxter_traj_streamer traj_interpolator_as

source in: /src/traj_interpolator_as.cpp

action message goal includes trajectory_msgs/JointTrajectory
action server breaks this up into incremental commands, spaced at dt_traj
populates and publishes a messages of type baxter_core_msgs::JointCommand every dt_traj
objects/exits if have fewer than 2 trajectory points

values for dt_traj and max velocities are set in header, baxter_traj_streamer.h

See traj_action_client_pre_pose.cpp for example of how to use this action server


