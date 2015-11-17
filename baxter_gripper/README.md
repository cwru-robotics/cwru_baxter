# baxter_gripper
This package combines some Dynamixel communications C-code with a ROS node.  A Dynamixle MX-64 is assumed, with an input command range of 0-4095 for one revolution.

Make sure the chosen USB port is read/writable (e.g. /dev/ttyUSB0).

The node: `dynamixel_motor_node` can be run with:

`rosrun baxter_gripper dynamixel_motor_node`

This will attempt to communicate to a Dynamixel motor with default values of:
* motor_id = 1
* baudnum code = 1 (1,000,000 BPS)
* ttynum=0 (/dev/ttyUSB0)

The node will subscribe to topic "dynamixel_motor1_cmd" for input commands in range 0-4095, and it will publish to topic "dynamixel_motor1_ang" with the current reported motor angle.  However, there are frequent communications faults.  If a read fault is detected, the published angle will be > 4096.  This should be inspected, to determine if the angle may be trusted.

The node may be run with options, e.g.:
`rosrun baxter_gripper dynamixel_motor_node -m 2 -baud 1 -tty 0`

Which will assign motor_id=1, baudrate code = 1 and comm port /dev/ttyUSB0.  In this case, the node name will be `dynamixel_motor2`, which will listen for commands on topic `dynamixel_motor2_cmd` and will publish feedback on topic `dynamixel_motor2_ang`.  

The command and feedback values may be plotted via rqt_plot to observer performance.

A complementary test function is: 
`rosrun baxter_gripper dynamixel_sin_test`

This will prompt for a motor_id, then send sinusoidal commands to the corresponding command topic.

## Example usage
`rosrun baxter_gripper dynamixel_motor_node` (default for Baxter's right-hand Yale gripper)
`rosrun baxter_gripper dynamixel_sin_test` (for slow, 1-rev peak-to-peak sinusoidal oscillations)

Alternatively, try a manual test, e.g. to command motor 1 to angle 1000:
`rostopic pub dynamixel_motor1_cmd std_msgs/Int16 1000`





    
