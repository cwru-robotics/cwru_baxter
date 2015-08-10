// baxter_kinematics_test_main.cpp
// wsn, May 2015
// test function for baxter_kinematics library
// this version subscribes to actual Baxter joint angles (joint_states) and
// uses home-brew fwd_kin to compute shoulder, elbow, wrist and tool-flange origins
//  these are compared to tf results 
//  all expressed w/rt right_arm_mount frame (which is slightly offset from S0 axis)


#include <baxter_kinematics/baxter_kinematics.h>
#include <tf/transform_listener.h>
// this to subscribe to joint states:
#include <sensor_msgs/JointState.h>
//stuff to command joint values:
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <baxter_core_msgs/JointCommand.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Geometry> 

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
    q_snapshot = q_vec_right_arm;
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
    
    tfListener.lookupTransform("torso", "right_hand", ros::Time(0), tfResult);
    pos = tfResult.getOrigin();
    ROS_INFO("vector to right hand from torso:  x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);  
    //baseToHand.getRotation()
    tf::Quaternion quat = tfResult.getRotation();
  
    Eigen::Quaterniond e_quat;
    e_quat.x() = quat.x();
    e_quat.y() = quat.y();
    e_quat.z() = quat.z();
    e_quat.w() = quat.w();   
    Eigen::Matrix3d Rflange =  e_quat.toRotationMatrix();
    cout<<"Rflange = "<<endl;
    cout<<Rflange<<endl;
    
    //compare to fwd kin soln:
    Eigen::Affine3d Affine_flange_wrt_torso = baxter_fwd_solver.fwd_kin_flange_wrt_torso_solve(q_snapshot);
    cout<<"O of hand wrt torso from fwd kin: "<<Affine_flange_wrt_torso.translation().transpose()<<endl;
    cout<<"R of hand wrt torso from fwd kin:"<<endl;
    cout<<Affine_flange_wrt_torso.linear()<<endl;
    
    //test conversion fnc:
    tfListener.lookupTransform("right_arm_mount", "right_hand", ros::Time(0), tfResult);
    pos = tfResult.getOrigin();
    ROS_INFO("vector to right hand from arm_mount:  x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);  
    //baseToHand.getRotation()
    quat = tfResult.getRotation();
    e_quat.x() = quat.x();
    e_quat.y() = quat.y();
    e_quat.z() = quat.z();
    e_quat.w() = quat.w();   
    Rflange =  e_quat.toRotationMatrix();
    cout<<"Rflange wrt arm-mount frame per tf = "<<endl;
    cout<<Rflange<<endl;
    
    //compare w/ conversion fnc:
    Eigen::Affine3d affine_hand_wrt_arm_mount = baxter_fwd_solver.transform_affine_from_torso_frame_to_arm_mount_frame(Affine_flange_wrt_torso);
    cout<<"O of hand wrt arm mount per xform: "<<affine_hand_wrt_arm_mount.translation().transpose()<<endl;
    cout<<"R of hand wrt arm mount per xform:"<<endl;
    cout<<affine_hand_wrt_arm_mount.linear()<<endl;  
    
    
    
    int ans1;
    cout<<"enter 1 to continue: ";
    cin>>ans1;
    
    //Baxter_IK_solver ik_solver;
    Eigen::Affine3d A_fwd_DH;
    Eigen::Matrix4d A_wrist,A_elbow,A_shoulder,A_flange;    
    Eigen::Matrix3d R_hand;
    std::cout << "==== Test for Baxter kinematics solver ====" << std::endl;
    Eigen::Matrix3d R_flange_wrt_right_arm_mount;
    Eigen::Vector3d wrist_pt_wrt_right_arm_frame1;
    Eigen::Vector3d w_err;
    while (ros::ok()) 
    {
        q_snapshot = q_vec_right_arm;
        cout<<"qvec right arm: "<<endl;
        cout<< q_snapshot.transpose() <<endl;
            A_fwd_DH = baxter_fwd_solver.fwd_kin_flange_wrt_r_arm_mount_solve(q_snapshot); //fwd_kin_solve

            //get and print solutions--origins off selected frames
            A_shoulder = baxter_fwd_solver.get_shoulder_frame();
            std::cout << "fwd kin shoulder point: " << A_shoulder(0, 3) << ", " << A_shoulder(1, 3) << ", " << A_shoulder(2, 3) << std::endl;

            A_elbow = baxter_fwd_solver.get_elbow_frame();
            std::cout << "fwd kin elbow point: " << A_elbow(0, 3) << ", " << A_elbow(1, 3) << ", " << A_elbow(2, 3) << std::endl;
 
            A_wrist = baxter_fwd_solver.get_wrist_frame();
            std::cout << "fwd kin wrist point: " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) << std::endl;
            
            A_flange = baxter_fwd_solver.get_flange_frame();
            std::cout << "fwd kin flange origin: " << A_flange(0, 3) << ", " << A_flange(1, 3) << ", " << A_flange(2, 3) << std::endl;
            // generate the tool-flange pose as an affine matrix from fwd kin (for testing)
            Eigen::Affine3d affine_flange(A_flange);   

        //PROVIDE q_s0 to compute wrist pt w/rt frame1
        wrist_pt_wrt_right_arm_frame1 = baxter_IK_solver.wrist_frame1_from_flange_wrt_rarm_mount(affine_flange,q_snapshot);
        cout<< "w w/rt frame1 from fwd kin code: "<<wrist_pt_wrt_right_arm_frame1.transpose()<<endl;
        
    tferr=true;
    ROS_INFO("waiting for tf between base and right_hand...");
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform from target frame "odom" to source frame "map"
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                tfListener.lookupTransform("right_hand", "torso", ros::Time(0), tfResult);
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good; frame origins w/rt right_arm_mount: ");  
        tfListener.lookupTransform("right_arm_mount", "right_lower_shoulder", ros::Time(0), tfResult);
    tf::Vector3 pos = tfResult.getOrigin();
    ROS_INFO("tf: shoulder x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);

    tfListener.lookupTransform("right_arm_mount","right_lower_elbow", ros::Time(0), tfResult);
    pos = tfResult.getOrigin();
    ROS_INFO("tf: elbow x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);
    tfListener.lookupTransform("right_arm_mount","right_lower_forearm",  ros::Time(0), tfResult);
    pos = tfResult.getOrigin();
    ROS_INFO("tf: wrist x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);     
    tfListener.lookupTransform("right_arm_mount","right_hand",  ros::Time(0), tfResult);
    pos = tfResult.getOrigin();
    ROS_INFO("tf: hand x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);  
    tfListener.lookupTransform("right_lower_shoulder","right_hand",  ros::Time(0), tfResult);
    pos = tfResult.getOrigin();
    ROS_INFO("tf: flange w/rt frame1: x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);   
    
    //this test is not quite right...right_lower_shoulder has desired origin, but frame
    // rotates w/ q(1); but test of w_wrt_frame1 looks successful, when q(1)=0.0
    //tfListener.lookupTransform("right_lower_shoulder","right_lower_forearm",  ros::Time(0), tfResult);
    //pos = tfResult.getOrigin();
    //ROS_INFO("tf: wrist w/rt frame1: x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);  
    //for (int i=0;i<3;i++) w_err(i) = pos[i]-wrist_pt_wrt_right_arm_frame1(i);
    //cout<<"wrist err: "<<w_err.transpose()<<endl;
    double q_elbow;
    double q_humerus[2];
    double q_s1[2];
    double q_s1_temp;

    bool reachable=false;
    reachable= baxter_IK_solver.solve_for_elbow_ang(wrist_pt_wrt_right_arm_frame1, q_elbow);
    //cout<<"q_elbow from IK: "<<q_elbow<<endl;
    if (reachable) {
       reachable = baxter_IK_solver.solve_for_humerus_ang(wrist_pt_wrt_right_arm_frame1,q_elbow, q_humerus); //expect 2 q_humerus values
       if (reachable) {           
        cout<<"q_humerus[0]: "<<q_humerus[0]<<endl;
        //solve for q_shoulder_elevation
        reachable = baxter_IK_solver.solve_for_s1_ang(wrist_pt_wrt_right_arm_frame1,q_elbow,  q_humerus[0], q_s1_temp);
        if (reachable) { q_s1[0] = q_s1_temp; }
        
        cout<<"q_humerus[1]: "<<q_humerus[1]<<endl;
        //solve for q_shoulder_elevation
        reachable = baxter_IK_solver.solve_for_s1_ang(wrist_pt_wrt_right_arm_frame1,q_elbow,  q_humerus[1], q_s1_temp);
        if (reachable) { q_s1[1] = q_s1_temp; }        
       }
    }
    ROS_INFO("actual q1,q2,q3: %f, %f, %f",q_snapshot(1),q_snapshot(2),q_snapshot(3));
    ROS_INFO("IK soln1: %f , %f, %f",q_s1[0], q_humerus[0], q_elbow);
    ROS_INFO("IK soln2: %f , %f, %f",q_s1[1], q_humerus[1], q_elbow);  
    
    // try out the new fnc:
    double q_s0 = q_snapshot(0);
    std::vector<Vectorq7x1> q_solns;
    cout<<"TRYING NEW FNC q123"<<endl;
    //must provide q_s0 to get wrist pt w/rt frame1
    baxter_IK_solver.compute_q123_solns(affine_flange, q_s0, q_solns);
    cout<<"num solns: "<<q_solns.size()<<endl;

    
    // Eigen::Affine3d A_fwd_DH_approx = baxter_fwd_solver.fwd_kin_solve_approx(q_vec_right_arm); 
    //Eigen::Matrix4d A_wrist_approx;
    //A_wrist_approx = baxter_fwd_solver.get_wrist_frame_approx();
    //cout<<"main: A_wrist_approx:"<<endl;
    //cout<<A_wrist_approx<<endl;
    //std::cout << "main: approx FK wrist point: " << A_wrist_approx(0, 3) << ", " << A_wrist_approx(1, 3) << ", " << A_wrist_approx(2, 3) << std::endl;    
 
    
    /*
    tfListener.lookupTransform("right_arm_mount","right_upper_shoulder",  ros::Time(0), tfResult);
    pos = tfResult.getOrigin();
    ROS_INFO("tf: right_upper_shoulder w/rt right_arm_mount: x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]); 
    
    tfListener.lookupTransform("torso","right_arm_mount",  ros::Time(0), tfResult);
    pos = tfResult.getOrigin();
    ROS_INFO("tf: right_arm_mount w/rt torso: x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]); 
    */
    
    // test wrist soln:
    Eigen::Matrix3d Rdes = A_fwd_DH.linear();
    cout<<"Rdes: "<<endl;
    cout<<Rdes<<endl;
    baxter_IK_solver.solve_spherical_wrist(q_snapshot,Rdes, q_solns);
    int nsolns = q_solns.size();    
    cout<<"num solns (not tested for joint limits)= "<<nsolns<<endl;
    for (int i=0;i<nsolns;i++) {
        cout<<"q_soln"<<i<<": "<<q_solns[i].transpose()<<endl;
    }
    cout<<"actual: "<<q_snapshot.transpose()<<endl;
    double q_s0_ctr; //,q_s0_max;
    q_s0_ctr = baxter_IK_solver.compute_qs0_ctr(A_fwd_DH);  
    cout<<"solving IK at q_s0_ctr: "<<endl;
    reachable = true;
    double dqs0 = 0.001;
    q_s0 = q_s0_ctr;
    Eigen::Vector3d w_des_wrt_0 = baxter_IK_solver.wrist_frame0_from_flange_wrt_rarm_mount(affine_flange);
    while (reachable) { 
        cout<<"try q_s0 = "<<q_s0<<endl;
        reachable = baxter_IK_solver.compute_q123_solns(affine_flange, q_s0, q_solns); 
        if (reachable ) {
            //test these solns:
            cout<<"test: des w_wrt_0 = "<<w_des_wrt_0.transpose()<<endl;
            for (int i=0;i<q_solns.size();i++) {
                A_fwd_DH = baxter_fwd_solver.fwd_kin_flange_wrt_r_arm_mount_solve(q_solns[i]);
                A_wrist = baxter_fwd_solver.get_wrist_frame();
                std::cout << "sln"<<i<<":  w_wrt_0 " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) << std::endl;
            }
            q_s0+= dqs0;
            //visualization test: send soln as command to Baxter:
            q_soln = q_solns[0];
            for (int i=0;i<7;i++) {
               right_cmd.command[i] = q_soln(i);
            }
            baxter_IK_solver.solve_spherical_wrist(q_soln,Rdes, q_solns);
            q_soln = q_solns[0];
            if (q_solns.size()>0) {
              for (int i=0;i<7;i++) {
               right_cmd.command[i] = q_soln(i);    
                }
            }
            else {
                  for (int i=4;i<7;i++) {
               right_cmd.command[i] = 0.0;    //if no wrist solns, go to zero--just for visualization of solns
                }
            }
                  
            pub_right.publish(right_cmd); //publish jnt cmds to right arm
            ros::Duration(0.01).sleep(); //sleep for 2 seconds
            ros::spinOnce();           

        }
    }
    cout<<"found q_s0 max = "<<q_s0<<endl;
    reachable = true;
    q_s0-= dqs0;
    while (reachable) { 
        cout<<"try q_s0 = "<<q_s0<<endl;
        reachable = baxter_IK_solver.compute_q123_solns(affine_flange, q_s0, q_solns); 
        if (reachable ) {
            //test these solns:
            cout<<"test: des w_wrt_0 = "<<w_des_wrt_0.transpose()<<endl;
            for (int i=0;i<q_solns.size();i++) {
                A_fwd_DH = baxter_fwd_solver.fwd_kin_flange_wrt_r_arm_mount_solve(q_solns[i]);
                A_wrist = baxter_fwd_solver.get_wrist_frame();
                std::cout << "sln"<<i<<":  w_wrt_0 " << A_wrist(0, 3) << ", " << A_wrist(1, 3) << ", " << A_wrist(2, 3) << std::endl;
            }
            q_s0-= dqs0;
            //visualization test: send soln as command to Baxter:
            q_soln = q_solns[0];
            for (int i=0;i<7;i++) {
               right_cmd.command[i] = q_soln(i);
            }
            baxter_IK_solver.solve_spherical_wrist(q_soln,Rdes, q_solns);
            q_soln = q_solns[0];
            if (q_solns.size()>0) {
              for (int i=0;i<7;i++) {
               right_cmd.command[i] = q_soln(i);    
                }
            }
            else {
                  for (int i=4;i<7;i++) {
               right_cmd.command[i] = 0.0;    //if no wrist solns, go to zero--just for visualization of solns
                }
            }
            pub_right.publish(right_cmd); //publish jnt cmds to right arm
            ros::Duration(0.01).sleep(); //sleep for 2 seconds
            ros::spinOnce();           

        }
    }    
    
    
    ros::Duration(2).sleep(); //sleep for 2 seconds
    ros::spinOnce();

    }
    return 0;
}
