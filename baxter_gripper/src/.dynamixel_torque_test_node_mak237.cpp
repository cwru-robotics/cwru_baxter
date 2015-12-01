//wsn, 11/15; compile low-level C-code for Dynamixel communication w/ C++ ROS node
// this node subscribes to topic "dynamixel_motor2_cmd" for position commands in the range 0-4095
// It also publishes motor angles on topic "dynamixel_motor2_ang"
// for baxter gripper, want: motor_id=1; baudnum=1; ttynum=0;
// these are the defaults;
// alternatively, run w/: rosrun baxter_gripper dynamixel_motor_node -m 2 -tty 0 -baudnum 1
// to specify motor 2; or accept tty and baudnum defaults w/:
// rosrun baxter_gripper dynamixel_motor_node -m 2

// this node's name and its topic names are mangled to match motor_id, e.g.
//  dynamixel_motor2_cmd for motor 2, and dynamixel_motor2_ang for motor2 feedback topic
//  NOTE: there are fairly frequent read errors from the motor;  
//  read errors are published as ang+4096;  inspect the angle value, and if >4096, DO NOT BELIEVE IT

// can test this node with "rosrun baxter_gripper dynamixel_sin_test", which will prompt for the motor_id,
// then command slow sinusoides on the chosen motor topic


#include<ros/ros.h> 
#include<std_msgs/Int16.h> 
#include <linux/serial.h>
#include <termios.h>

// Default settings: EDIT THESE FOR YOUR MOTOR
#define DEFAULT_BAUDNUM		1 // code "1" --> 1Mbps
#define DEFAULT_ID		2 //this is the motor ID
#define DEFAULT_TTY_NUM			0 // typically, 0 for /dev/ttyUSB0
// mak237, added predefined motor positions for yale hand min/max
#define DEFAULT_CW_LIMIT	3000
#define DEFAULT_CCW_LIMIT	3800
// mak237, added torque max value definition
#define DEFAULT_TORQUE_LIMIT	128


extern "C" { 
  int send_dynamixel_goal(short int motor_id, short int goalval); 
  int open_dxl(int deviceIndex, int baudnum);
// mak237, added CW and CCW position limit functions
  int set_dynamixel_CW_limit(short int motor_id, int CW_limit);
  int set_dynamixel_CCW_limit(short int motor_id, int CCW_limit);

// mak237, added torque limit function
  int set_torque_max(short int motor_id, int torque_max);

// mak237, added torque mode toggle function
  int torque_control_toggle(short int motor_id, int torque_mode);

// mak237, added torque command function

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
  short int g_goal_torque=0;
// mak237, added variables for cw and ccw limits
  int CW_limit = DEFAULT_CW_LIMIT;
  int CCW_limit = DEFAULT_CCW_LIMIT;

// mak237, added variable for torque limit
  int torque_max = DEFAULT_TORQUE_LIMIT;


void dynamixelCB(const std_msgs::Int16& goal_torque_msg) 
{ 
  short int goal_torque = goal_torque_msg.data;
  g_goal_torque = goal_torque; // for use by main()
     send_dynamixel_torque_goal(motor_id,goal_torque);
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
  sprintf(in_topic_name,"dynamixel_motor%d_torque_cmd",motor_id);
  ROS_INFO("input command topic: %s",in_topic_name);
  sprintf(out_topic_name,"dynamixel_motor%d_torque",motor_id);
  ROS_INFO("output topic: %s",out_topic_name);

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
  std_msgs::Int16 motor_ang_msg;
  short int sensed_motor_ang=0;

// mak237, added torque mode nablee for initialization of torque mode
  torque_control_toggle(motor_id,1);

// mak237, added motor torque limit initialization
  set_torque_max(motor_id,torque_max);



  while(ros::ok()) {
   sensed_motor_ang = read_position(motor_id);
   if (sensed_motor_ang>4096) {
//      ROS_WARN("read error from Dynamixel: ang value %d at cmd %d",sensed_motor_ang-4096,g_goal_angle);
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
