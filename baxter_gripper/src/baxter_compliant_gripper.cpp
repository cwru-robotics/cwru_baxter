// node intended to send desired position/torque value for opening/closing and then

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <math.h>

int input_command = 0; //initialize gripper to open command
void manualCmdCB(const std_msgs::Bool& input_cmd) 
{ 
  if (input_cmd.data == 0)
  {
     ROS_INFO("received open command");
  }
  else if (input_cmd.data == 1)
  {
     ROS_INFO("received close command");
  }

  input_command = input_cmd.data;
} 


int main(int argc, char **argv) {
    ros::init(argc, argv, "hand_open_close_test"); 
    ros::NodeHandle n; 

    //std::cout<<"enter motor_id to test: ";
    int motor_id=1; // hard coded for bare motor
    //std::cin>>motor_id;
    char cmd_topic_name[50];
    sprintf(cmd_topic_name,"dynamixel_motor%d_cmd",motor_id);
    ROS_INFO("using command topic: %s",cmd_topic_name);

    char cmd_topic_toggle[50];
    sprintf(cmd_topic_toggle,"dynamixel_motor%d_mode",motor_id);

    ros::Publisher dyn_pub = n.advertise<std_msgs::Int16>(cmd_topic_name, 1);
    ros::Publisher torque_toggle = n.advertise<std_msgs::Bool>(cmd_topic_toggle, 1);


    ros::Subscriber manual_cmd_subscriber = n.subscribe("gripper_cmd",1,manualCmdCB);     


    std_msgs::Int16 cmd_msg;
    std_msgs::Bool toggle_msg;
 
   double dt = 0.02; // repeat at freq 1/dt
   ros::Rate naptime(1/dt); //create a ros object from the ros “Rate” class; 

// initialize motor to position mode and open hand

  toggle_msg.data = 0;
  cmd_msg.data = 3000; 
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {

	toggle_msg.data = input_command;

  	if (toggle_msg.data == 0)
  	{
   	  ROS_INFO("sending open command");
	  cmd_msg.data = 3000; //setting open value (position)
  	}

  	if (toggle_msg.data == 1)
  	{
   	  ROS_INFO("sending close command");
	  cmd_msg.data = 80; //setting close value (torque). This should firmly grip the larger blocks and loosely grip the smaller blocks
  	}

	torque_toggle.publish(toggle_msg); // publish the operation mode (position/torque) to the motor node
        dyn_pub.publish(cmd_msg); // publish the command to the motor node
	ros::spinOnce();	
	naptime.sleep();


    }
}

