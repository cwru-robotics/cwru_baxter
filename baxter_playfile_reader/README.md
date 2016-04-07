# baxter_playfile_reader
Can record joint-space trajectories with:
`rosrun baxter_playfile_reader baxter_record_trajectory`
This node listens on the joint-space topic, samples the right-arm joint angles,
and saves them to disk in the file "merry_r_arm_traj.jsp" (joint-space playfile).

Trajectories recorded in this fashion can be played back with the playfile reader,
`baxter_playfile_jointspace`
## Example usage
With Baxter (sim or real) running, cd to a trajectory appropriate to save recordings.
Be careful to rename "merry_r_arm_traj.jsp" to avoid overwriting previous recording.
When ready to record, start:
`rosrun baxter_playfile_reader baxter_record_trajectory`
enter "1", then move the arm in the desired trajectory (path and speed).
When done with recording, ctl-C.  The result will be in "merry_r_arm_traj.jsp".

To play back a joint-space trajectory file, start up the robot.  Start up the trajectory
interpolation action server:
`rosrun baxter_traj_streamer traj_interpolator_as`
In another terminal, cd to the directory containing the desired filename.
Start the playback using the desired filename (e.g., "merry_r_arm_traj.jsp").
`rosrun baxter_playfile_reader baxter_playfile_jointspace merry_r_arm_traj.jsp`

## Running tests/demos
    
