This folder intended to work either with real or simulated baxter.
For real baxter, need to set ROS_MASTER_URI.
Can do this with command line: `export ROS_MASTER_URI=http://baxter01:11311`

OR, do (from this directory): `source set_remote_master.sh`

Should then be able to do: rostopic list (from the terminal with ROS_MASTER_URI set).





