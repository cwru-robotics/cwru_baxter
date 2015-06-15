dynamixel_motor
===============
wsn June 15, 2015

ROS stack for interfacing with Robotis Dynamixel line of servo motors.

first, enable the USB port for user I/O with:

 `sudo chmod ugo+rwx /dev/ttyUSB0`

start the controller interface node:
`roslaunch dynamixel_controllers controller_manager.launch`

Default mount point for usb2dynamixel is at /dev/ttyUSB0. You can change this in controller_manager.launch (path: dynamixel_controllers/launch/controller_manager.launch)

Motor ID is constrained within 1 to 10. This search range is set in the above launch file.

Enable the controller with this command:
`roslaunch motor_controller tilt_controller.launch`
Process should run to completion with happy talk.

Will see ros topics:
/motor_states/dxl_tty1
/motor_states/pan_tilt_port

can manually move the gripper with:
Spread hand:
	rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- -3.5
Grab object:
	rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- -4.0

BE CAREFUL OF VALUES USED; CAN RESULT IN BREAKING  THE GRIPPER
