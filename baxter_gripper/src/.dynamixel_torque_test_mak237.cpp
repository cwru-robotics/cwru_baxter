#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_sin_test"); 
    ros::NodeHandle n; 

    //std::cout<<"enter motor_id to test: ";
    int motor_id=2; // hard coded for Yale hand
    //std::cin>>motor_id;
    char cmd_topic_name[50];
    sprintf(cmd_topic_name,"dynamixel_motor%d_torque_cmd",motor_id);
    ROS_INFO("using command topic: %s",cmd_topic_name);

    ros::Publisher torque_pub = n.advertise<std_msgs::Int16>(cmd_topic_name, 1);
    
    std_msgs::Int16 int_torque; 
   double dt = 0.02; // repeat at freq 1/dt
   ros::Rate naptime(1/dt); //create a ros object from the ros “Rate” class; 

    int_torque.data = 0.0;

    //double mid_angle = 3500;
    //double amp = 500;
    //double omega = 0.1*2*M_PI; // in Hz 
    //double phase = 0;
    //double dbl_ang=0.0;
    short int int_ang=3500; // mak237, initialize hand at mid position for safety
    short int int_trq=0; // initialize torque to 0 to keep still
    int cycle_counter = 0;
    
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        //phase += omega*dt;
        //dbl_ang = amp*sin(phase)+mid_angle;
	//int_ang = (short int) dbl_ang;
// mak237, modified code to move between two positions at set intervals
	
	if (cycle_counter <= (2 * (1/dt))) // 2 is a magic number indicating holding the position for 2 seconds
	{
	    int_trq = 0;
	}
	else if (cycle_counter > (2 * (1/dt))) // 2 is a magic number indicating changing the set torque
	{
	    int_trq = 64;
	}

        int_torque.data = int_trq;
        ROS_INFO("sending: %d",int_trq);
        torque_pub.publish(int_torque); // publish the value--of type Float64-- 
	naptime.sleep();
	cycle_counter++; // increment counter to indicate a cycle has passed
	// If more than 4 seconds have elapsed reset the counter to the initial state
	if (cycle_counter >= (4 * (1/dt)))  
	{
	    cycle_counter = 0;
	}
    }
}

