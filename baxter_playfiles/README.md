# baxter_playfiles

This is a ros package that just holds baxter playfiles.  These should be runnable by the baxter playfile reader.

## Example usage
Start up the trajectory
interpolation action server:
`rosrun baxter_traj_streamer traj_interpolator_as`

From THIS directory, run a known playfile name with,  (e.g., "merry_r_arm_traj.jsp").
`rosrun baxter_playfile_reader baxter_playfile_jointspace merry_r_arm_traj.jsp`

For Alexa interface, run:
`rosrun baxter_playfile_reader baxter_multitraj_player`

Then can invoke mappings to files either with Alexa, or manually with:
`rostopic pub Alexa_codes std_msgs/UInt32 1`

(which will invoke playfile on case 1, which maps to a file in this directory) 



    
