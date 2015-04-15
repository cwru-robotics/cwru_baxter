// wsn test pgm to command joint values to Baxter
// emulating keyboard jog, use, e.g., for right elbow command:
//mode: 1
//command: [0.2439671942241816]
//names: ['right_e1']
// for message type:  baxter_core_msgs/JointCommand
// on topic: /robot/limb/right/joint_command

#include <ros/init.h>
#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <baxter_core_msgs/JointCommand.h>
#include <math.h>
#include <string.h>
using namespace std;

ros::Publisher pub_right, pub_left; //Create a 'publisher' object
//string j_name = "right_e1";

int main(int argc, char** argv){
  double amp = 1.0; // amplitude for sin fnc
  double freq = 0.5; // Hz
  double time = 0.0;
  double dt = 0.01; // time step
  double phase = 0.0; // phase angle for sinusoid
  double dtheta = dt*freq*2.0*M_PI;

  ros::init(argc, argv, "example_baxter_commander"); //standard ros init with node name string
  ros::NodeHandle nh; //standard ros node handle
  baxter_core_msgs::JointCommand right_cmd,left_cmd;
  left_cmd.mode = 1;
  right_cmd.mode = 1;
// define the joint angles 0-6 to be right arm, from shouler out to wrist;
// joint angles 7-13 are corresponding left-arm joints
  right_cmd.names.push_back("right_s0");
  right_cmd.names.push_back("right_s1");
  right_cmd.names.push_back("right_e0");
  right_cmd.names.push_back("right_e1");
  right_cmd.names.push_back("right_w0");
  right_cmd.names.push_back("right_w1");
  right_cmd.names.push_back("right_w2");
  left_cmd.names.push_back("left_s0");
  left_cmd.names.push_back("left_s1");
  left_cmd.names.push_back("left_e0");
  left_cmd.names.push_back("left_e1");
  left_cmd.names.push_back("left_w0");
  left_cmd.names.push_back("left_w1");
  left_cmd.names.push_back("left_w2");

  for (int i=0;i<7;i++) {
     right_cmd.command.push_back(0.0); // start commanding 0 angle for right-arm 7 joints
     left_cmd.command.push_back(0.0); // start commanding 0 angle for left-arm 7 joints
   }

  pub_right = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1); //setting up the publisher to   
   //publish to /robot/limb/right/joint_command with a queue size of 1
  pub_left = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1); //setting up the publisher to 
    // left arm  

  double jnt_angle=0.0;
  while(ros::ok()){ //while ROS is okay...
     phase+= dtheta;
     if (phase>2.0*M_PI) phase -= 2.0*M_PI;
     jnt_angle = amp*sin(phase);
  for (int i=0;i<7;i++) {
     right_cmd.command[i] = jnt_angle; // all 7 jnts get same command
   }
   //make left arm move out of phase:
  for (int i=0;i<7;i++) {
     left_cmd.command[i] = -jnt_angle; // all 7 jnts get same command
   }
     pub_right.publish(right_cmd); //publish jnt cmds to right arm
     pub_left.publish(left_cmd); //publish jnt cmds to left arm
     ros::Duration(dt).sleep(); //Wait dt

  }
}
