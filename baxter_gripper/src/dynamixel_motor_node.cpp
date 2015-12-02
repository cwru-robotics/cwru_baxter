//wsn, 11/15; compile low-level C-code for Dynamixel communication w/ C++ ROS node
// this node subscribes to topic "dynamixel_motor1_cmd" for position commands in the range 0-4095
// It also publishes motor angles on topic "dynamixel_motor1_ang"

// for baxter gripper, want: motor_id=1; baudnum=1; ttynum=0;
// these are the defaults;
// alternatively, run w/: rosrun baxter_gripper dynamixel_motor_node -m 2 -tty 0 -baudnum 1
// to specify motor 2; or accept tty and baudnum defaults w/:
// rosrun baxter_gripper dynamixel_motor_node -m 2

// this node's name and its topic names are mangled to match motor_id, e.g.
//  dynamixel_motor1_cmd for motor 1, and dynamixel_motor1_ang for motor1 feedback topic
//  NOTE: there are fairly frequent read errors from the motor;  
//  read errors are published as ang+4096;  inspect the angle value, and if >4096, DO NOT BELIEVE IT

// can test this node with "rosrun baxter_gripper baxter_gripper_test", which will prompt for the motor_id,
// then command slow sinusoides on the chosen motor topic


#include<ros/ros.h> 
#include<std_msgs/Int16.h> 
#include<std_msgs/Bool.h> 
#include <linux/serial.h>
#include <termios.h>

// Default settings: EDIT THESE FOR YOUR MOTOR
#define DEFAULT_BAUDNUM		1 // code "1" --> 1Mbps
#define DEFAULT_ID		1 //this is the motor ID
#define DEFAULT_TTY_NUM			0 // typically, 0 for /dev/ttyUSB0

// Dynamixel motor position/torque limit settings: 
#define DEFAULT_CW_LIMIT	3000 // Yale hand full open
#define DEFAULT_CCW_LIMIT	3800 // Yale hand full close (fingertips together)
#define DEFAULT_TORQUE_LIMIT	128

// add motor communication commands defined in dxl_c_files/ReadWrite.c
extern "C" { 
  int send_dynamixel_goal(short int motor_id, short int goalval); 
  int open_dxl(int deviceIndex, int baudnum);
  int set_dynamixel_CW_limit(short int motor_id, int CW_limit);
  int set_dynamixel_CCW_limit(short int motor_id, int CCW_limit);
  int torque_control_toggle(short int motor_id, int torque_mode);
  int set_torque_max(short int motor_id, int torque_max);
  int send_dynamixel_torque_goal(short int motor_id, int torquegoalval);

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
  int ttynum = DEFAULT_TTY_NUM;
  short int g_goal_cmd=0;

  int CW_limit = DEFAULT_CW_LIMIT;
  int CCW_limit = DEFAULT_CCW_LIMIT;
  int torque_max = DEFAULT_TORQUE_LIMIT;

  bool torque_mode = 0;
  bool torque_mode_old = 0; 

// Callback to toggle between position and torque modes
void toggleCB(const std_msgs::Bool& torque_toggle_msg)
{
  torque_mode = torque_toggle_msg.data; // pull incoming torque mode
  if (torque_mode != torque_mode_old) // if torque mode has changed send a command to change toggle status to motor
  {
     torque_control_toggle(motor_id, torque_mode);
  }
  torque_mode_old = torque_mode; // remember most recent torque mode

}

// Callback to receive goal commands and send them to the motor
void dynamixelCB(const std_msgs::Int16& goal_cmd_msg) 
{ 
  short int goal_cmd = goal_cmd_msg.data;

  if (torque_mode == torque_mode_old) //if not in the middle of a togle state send current goal message
  {
     g_goal_cmd = goal_cmd; // for use by main()

     if (torque_mode == 0)
     {
	send_dynamixel_goal(motor_id,goal_cmd);
     }
     else if (torque_mode == 1)
     {
	send_dynamixel_torque_goal(motor_id, goal_cmd);
     }
  }

} 



int main(int argc, char **argv) 
{ 
  std::string dash_m = "-m";
  std::string dash_tty = "-tty";
  std::string dash_baud = "-baud";
  
  if (argc<2) {
   ROS_INFO("using default motor_id %d, baud code %d, via /dev/ttyUSB%d",motor_id,baudnum,ttynum);
   ROS_INFO("may run with command args, e.g.: -m 2 -tty 1 for motor_id=2 on /dev/ttyUSB1");
  }
  else {
   std::vector <std::string> sources;
    for (int i = 1; i < argc; ++i) { // argv[0] is the path to the program, we want from argv[1] onwards
            sources.push_back(argv[i]); 
    }
    for (int i=0;i<argc-2;i++) {  // if have a -m or -tty, MUST have at least 2 args
       std::cout<<sources[i]<<std::endl;
       if (sources[i]==dash_m) {
        std::cout<<"found dash_m"<<std::endl;
        motor_id = atoi(sources[i+1].c_str()); 
 
        }
       if (sources[i]==dash_tty) {
        std::cout<<"found dash_tty"<<std::endl;
        ttynum = atoi(sources[i+1].c_str()); 
        }
       if (sources[i]==dash_baud) {
        std::cout<<"found dash_baud"<<std::endl;
        baudnum = atoi(sources[i+1].c_str()); 
        }
    }
    ROS_INFO("using motor_id %d at baud code %d via /dev/ttyUSB%d",motor_id,baudnum,ttynum);
  }
        

  char node_name[50];
  char in_topic_name[50];
  char out_topic_name[50];
  sprintf(node_name,"dynamixel_motor%d",motor_id);
  ROS_INFO("node name: %s",node_name);
  sprintf(in_topic_name,"dynamixel_motor%d_cmd",motor_id);
  ROS_INFO("input command topic: %s",in_topic_name);
  sprintf(out_topic_name,"dynamixel_motor%d_ang",motor_id);
  ROS_INFO("output topic: %s",out_topic_name);

  char in_topic_toggle[50];
  sprintf(in_topic_toggle,"dynamixel_motor%d_mode",motor_id);


  ros::init(argc,argv,node_name); //name this node 

  ros::NodeHandle n; // need this to establish communications with our new node 
  ros::Publisher pub_jnt = n.advertise<std_msgs::Int16>(out_topic_name, 1);
  
  double dt= 0.01; // 100Hz

  ROS_INFO("attempting to open /dev/ttyUSB%d",ttynum);
  bool open_success = open_dxl(ttynum,baudnum);

  if (!open_success) {
    ROS_WARN("could not open /dev/ttyUSB%d; check permissions?",ttynum);
    return 0;
  }

  ROS_INFO("attempting communication with motor_id %d at baudrate code %d",motor_id,baudnum);

  ros::Subscriber subscriber = n.subscribe(in_topic_name,1,dynamixelCB); 
  ros::Subscriber toggle_subscriber = n.subscribe(in_topic_toggle,1,toggleCB); 
  std_msgs::Int16 motor_ang_msg;
  short int sensed_motor_ang=0;

  torque_control_toggle(motor_id,0); // set motor to torque mode during initialization

  // set position/torque limits during initialization
  set_torque_max(motor_id,torque_max);
  set_dynamixel_CW_limit(motor_id,CW_limit);
  set_dynamixel_CCW_limit(motor_id,CCW_limit);


  while(ros::ok()) {
   sensed_motor_ang = read_position(motor_id);
   if (sensed_motor_ang>4096) {
      ROS_WARN("read error from Dynamixel: ang value %d at cmd %d",sensed_motor_ang-4096,g_goal_cmd);
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
