// baxter_kinematics_test_main3.cpp
// wsn, May 2015
// test function for baxter_kinematics library
// this version to test higher-level fnc: ik_solve_approx()


#include <baxter_kinematics/baxter_kinematics.h>
#include <tf/transform_listener.h>
// this to subscribe to joint states:
#include <sensor_msgs/JointState.h>
//stuff to command joint values:
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <baxter_core_msgs/JointCommand.h>

using namespace std;

 Vectorq7x1 q_vec_right_arm,q_in,q_soln,q_snapshot;
void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    // copy right-arm angles to global vec
    for (int i=0;i<7;i++)
    {
        // should do this better; manually remap from joint_states indices to right-arm joint angles
        q_vec_right_arm[6] = js_msg.position[14]; //w2
        q_vec_right_arm[5] = js_msg.position[13]; //w1
        q_vec_right_arm[4] = js_msg.position[12]; //w0
        q_vec_right_arm[3] = js_msg.position[9]; //e1
        q_vec_right_arm[2] = js_msg.position[8]; //e0  
        q_vec_right_arm[1] = js_msg.position[11]; //s1
        q_vec_right_arm[0] = js_msg.position[10]; //s0           
    }
}    
   
int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_kinematics_test_main");
    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub = nh.subscribe("robot/joint_states", 1, jointStatesCb);
    q_vec_right_arm<< 0,0,0,0,0,0,0;
    q_in << 0,0,0,0,0,0,0;
    //q_in << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    tf::TransformListener tfListener;
    tf::StampedTransform tfResult;
    
// set up and initialize joint publishers:
  ros::Publisher pub_right, pub_left; //Create a 'publisher' object
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
    
   bool tferr=true;
    ROS_INFO("waiting for tf between base and right_hand...");
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                tfListener.lookupTransform("right_arm_mount","right_hand", ros::Time(0), tfResult);
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good");
    //right_upper_shoulder is located at bottom of first shoulder jnt
    // from now on, tfListener will keep track of transforms    
    tfListener.lookupTransform("right_arm_mount", "right_lower_shoulder", ros::Time(0), tfResult);
    tf::Vector3 pos = tfResult.getOrigin();
    ROS_INFO("shoulder x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);

    tfListener.lookupTransform("right_arm_mount","right_lower_elbow", ros::Time(0), tfResult);
    pos = tfResult.getOrigin();
    ROS_INFO("elbow x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);
    tfListener.lookupTransform("right_arm_mount","right_lower_forearm",  ros::Time(0), tfResult);
    pos = tfResult.getOrigin();
    ROS_INFO("wrist x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);     
    tfListener.lookupTransform("right_arm_mount","right_hand",  ros::Time(0), tfResult);
    pos = tfResult.getOrigin();
    ROS_INFO("hand x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);   
    
    Baxter_fwd_solver baxter_fwd_solver; //instantiate a forward-kinematics solver
    Baxter_IK_solver baxter_IK_solver;  // instantiate an IK solver
    
    
    //Baxter_IK_solver ik_solver;
    Eigen::Affine3d A_fwd_DH;
    Eigen::Matrix4d A_wrist,A_elbow,A_shoulder,A_flange;    
    Eigen::Matrix3d R_hand;
    std::cout << "==== Test for Baxter kinematics solver ====" << std::endl;
    Eigen::Matrix3d R_flange_wrt_right_arm_mount;
    Eigen::Vector3d wrist_pt_wrt_right_arm_frame1;
    Eigen::Vector3d w_err;
    int ans;
    int nsolns;
    Vectorq7x1 q_vec_display;
    std::vector<Vectorq7x1> q_solns;    
    while (ros::ok()) 
    {
        cout<<"enter 1 to initiate: "; //pause here to allow user to move joint angles to desired test pose
        cin>>ans;
        for (int i=0;i<10;i++) { // spin to get current joint angles
            ros::spinOnce();
            ros::Duration(0.1).sleep(); //sleep for 2 seconds
        }
        q_snapshot = q_vec_right_arm;
        cout<<"qvec right arm: "<<endl;
        cout<< q_snapshot.transpose() <<endl;
        // from this test pose, establish a viable hand frame:
        A_fwd_DH = baxter_fwd_solver.fwd_kin_solve(q_snapshot); //fwd_kin_solve

        cout<<"TRYING NEW FNC ik_solve_approx"<<endl;
        nsolns = baxter_IK_solver.ik_solve_approx(A_fwd_DH,q_solns);
        cout<<"num solns: "<<nsolns<<endl;
        cout<<"commanding these solutions..."<<endl;

 
       for (int isoln=0; isoln<nsolns; isoln++) {
               q_vec_display= q_solns[isoln];
               for (int j=0;j<7;j++) {
                    right_cmd.command[j] = q_vec_display[j];    
                }

            pub_right.publish(right_cmd); //publish jnt cmds to right arm
            cout<<"soln: "<<q_vec_display.transpose()<<endl;
            ros::Duration(2).sleep(); //sleep for 2 seconds
            ros::spinOnce();            
        }
    
    ros::Duration(2).sleep(); //sleep for 2 seconds
    ros::spinOnce();

    }
    return 0;
}
