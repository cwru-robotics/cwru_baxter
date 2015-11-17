//wsn, 11/15; compile low-level C-code for Dynamixel communication w/ C++ ROS node
// this node subscribes to topic dynamixel_motor2_ang

#include<ros/ros.h> 
#include<std_msgs/Int16.h> 
#include <linux/serial.h>
#include <termios.h>

// Default settings: EDIT THESE FOR YOUR MOTOR
#define DEFAULT_BAUDNUM		1 // code "1" --> 1Mbps
#define DEFAULT_ID		2 //this is the motor ID
#define TTY_NUM			0 // typically, 0 for /dev/ttyUSB0

extern "C" { 
  int send_dynamixel_goal(short int motor_id, short int goalval); 
  int open_dxl(int deviceIndex, int baudnum);
  //make these global, so connection info is not lost after fnc call
  char	gDeviceName[20];
  struct termios newtio;
  struct serial_struct serinfo;
  char dev_name[100] = {0, };
  void dxl_terminate(void);
  short int read_position(short int motor_id);
}

//globals:
  short int motor_id = DEFAULT_ID;
  short int baudnum = DEFAULT_BAUDNUM;

void baxterGripperCB(const std_msgs::Int16& goal_angle_msg) 
{ 
  short int goal_angle = goal_angle_msg.data;

     send_dynamixel_goal(motor_id,goal_angle);
} 


int main(int argc, char **argv) 
{ 
  ros::init(argc,argv,"baxter_gripper"); //name this node 

  ros::NodeHandle n; // need this to establish communications with our new node 
  ros::Publisher pub_jnt = n.advertise<std_msgs::Int16>("dynamixel_motor2_ang", 1);
  
  double dt= 0.01; // 100Hz
  int ttynum = TTY_NUM;
  ROS_INFO("attempting to open /dev/ttyUSB%d",ttynum);
  bool open_success = open_dxl(ttynum,baudnum);

  if (!open_success) {
    ROS_WARN("could not open /dev/ttyUSB%d; check permissions?",ttynum);
    return 0;
  }

  ROS_INFO("attempting communication with motor_id %d at baudrate code %d",motor_id,baudnum);

  ros::Subscriber subscriber = n.subscribe("baxter_gripper",1,baxterGripperCB); 
  std_msgs::Int16 motor_ang_msg;
  short int sensed_motor_ang=0;

  while(ros::ok()) {
   sensed_motor_ang = read_position(motor_id);
   if (sensed_motor_ang>4096) {
      ROS_WARN("read error from Dynamixel: value %d",sensed_motor_ang);
    }
    motor_ang_msg.data = sensed_motor_ang;
   pub_jnt.publish(motor_ang_msg);
   ros::Duration(dt).sleep();
   ros::spinOnce();
   }
  dxl_terminate();
  ROS_INFO("goodbye");
  return 0; // should never get here, unless roscore dies 
} 
