// wsn test pgm to command joint values to Baxter
// try out some predefined poses:


#include <ros/init.h>
#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <baxter_core_msgs/JointCommand.h>
#include <math.h>
#include <string.h>
using namespace std;

ros::Publisher pub_right, pub_left; //Create a 'publisher' object

// pre-define a pose with arms symmetric: rqrs alternate jnt angles to be negated
double mantis_right[] = {-0.5, -1.20,  0.5, 0.7,  0.92, 1.23, -0.5};
double mantis_left[] =  { 0.5, -1.20, -0.5, 0.7, -0.92, 1.23,  0.5};

int main(int argc, char** argv){
  double dt = 0.01; // time step

  ros::init(argc, argv, "example_baxter_commander"); //standard ros init with node name string
  ros::NodeHandle nh; //standard ros node handle
  baxter_core_msgs::JointCommand right_cmd,left_cmd;  // define instances of these message types, to control arms
  left_cmd.mode = 1; // set the command modes to "position"
  right_cmd.mode = 1;

// define the joint angles 0-6 to be right arm, from shoulder out to wrist;
  right_cmd.names.push_back("right_s0");
  right_cmd.names.push_back("right_s1");
  right_cmd.names.push_back("right_e0");
  right_cmd.names.push_back("right_e1");
  right_cmd.names.push_back("right_w0");
  right_cmd.names.push_back("right_w1");
  right_cmd.names.push_back("right_w2");
// same order for left arm
  left_cmd.names.push_back("left_s0");
  left_cmd.names.push_back("left_s1");
  left_cmd.names.push_back("left_e0");
  left_cmd.names.push_back("left_e1");
  left_cmd.names.push_back("left_w0");
  left_cmd.names.push_back("left_w1");
  left_cmd.names.push_back("left_w2");

  // do push-backs to establish desired vector size with valid joint angles
  for (int i=0;i<7;i++) {
     right_cmd.command.push_back(0.0); // start commanding 0 angle for right-arm 7 joints
     left_cmd.command.push_back(0.0); // start commanding 0 angle for left-arm 7 joints
   }

//set up the publisher to publish to /robot/limb/right/joint_command with a queue size of 1
  pub_right = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1); 
//do same for left limb:
  pub_left = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1); 

// done with all the initializations;  can now fill in desired joint angles and publish them as commands
// set the "mantis" position, as hard-code defined above (globally)
  for (int i=0;i<7;i++) {
     right_cmd.command[i] = mantis_right[i]; // right arm, pre-defined "mantis" pose
     left_cmd.command[i] = mantis_left[i]; // left arm, pre-defined "mantis" pose
   }

// loop sending out these commands; control-C to halt this node
  while(ros::ok()){ //while ROS is okay...

    // boring--always sending out same commands;  could make this more interesting
     pub_right.publish(right_cmd); //publish jnt cmds to right arm
     pub_left.publish(left_cmd); //publish jnt cmds to left arm
     ros::Duration(dt).sleep(); //Wait dt

  }
}
