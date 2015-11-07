// baxter_test_IK.cpp
// wsn, Nov 2015
// test function for baxter_kinematics library
// this version uses trajectory interpolation action server from baxter_traj_streamer package
// uses higher-level fnc: ik_solve_approx_wrt_torso()
//int Baxter_IK_solver::ik_solve_approx_wrt_torso(Eigen::Affine3d const& desired_hand_pose_wrt_torso,std::vector<Vectorq7x1> &q_solns)

#include <baxter_kinematics/baxter_kinematics.h>
#include <tf/transform_listener.h>
// this to subscribe to joint states:
#include <sensor_msgs/JointState.h>
//stuff to command joint values:
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <baxter_core_msgs/JointCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <cwru_action/trajAction.h>

using namespace std;

Eigen::Affine3d transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3d e;
    // treat the Eigen::Affine as a 4x4 matrix:
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i]; //copy the origin from tf to Eigen
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j]; //and copy 3x3 rotation matrix
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}



 Vectorq7x1 q_vec_right_arm,q_soln,q_snapshot;
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

    tf::TransformListener tfListener;
    tf::StampedTransform tfResult;
    Eigen::VectorXd q_in_vecxd;    
    trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 
    std::vector<Eigen::VectorXd> des_path;
    Baxter_traj_streamer baxter_traj_streamer_obj(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    for (int i=0;i<100;i++) { //warm up callbacks
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    //create an action client      
    actionlib::SimpleActionClient<cwru_action::trajAction> action_client("trajActionServer", true);

    bool server_exists = false;
    while (!server_exists) {
        ROS_INFO("waiting for server: ");
        server_exists=action_client.waitForServer(ros::Duration(0.5));
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("connected to traj action server");
    cwru_action::trajGoal goal;    
       
   bool tferr=true;
    ROS_INFO("waiting for tf between base and right_hand...");
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform between target frame and source frame 
            //The direction of the transform returned will be from the target_frame to the source_frame. 
             //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
                tfListener.lookupTransform("torso","right_hand", ros::Time(0), tfResult);
            } catch(tf::TransformException &exception) {
                ROS_ERROR("%s", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }    //right_upper_shoulder is located at bottom of first shoulder jnt
    ROS_INFO("tf is good");
    // from now on, tfListener will keep track of transforms         
    tfListener.lookupTransform("torso","right_hand",  ros::Time(0), tfResult);
    tf::Vector3 pos = tfResult.getOrigin();
    ROS_INFO("from tf, hand x,y, z = %f, %f, %f",pos[0],pos[1],pos[2]);  
    Eigen::Affine3d affine_init_hand_pose = transformTFToEigen(tfResult);
    ROS_INFO_STREAM("affine from tf: origin = "<<affine_init_hand_pose.translation().transpose());
    ROS_INFO("orientation: ");
    cout<<affine_init_hand_pose.linear()<<endl;
    //wait for joint states: init an impossible angle, and wait for callback to fix it
    q_vec_right_arm[0] = 1000.0;
    while (q_vec_right_arm[0]>500) {
     // spin to get current joint angles
            ros::spinOnce();
            ros::Duration(0.1).sleep(); 
    } 
    q_snapshot = q_vec_right_arm; // this is robot's current pose;
    cout<<"got joint angles: "<<q_snapshot.transpose()<<endl;
    
    Baxter_fwd_solver baxter_fwd_solver; //instantiate a forward-kinematics solver
    Baxter_IK_solver baxter_IK_solver;  // instantiate an IK solver
    
    Eigen::Affine3d Affine_flange_wrt_utorso = baxter_fwd_solver.fwd_kin_flange_wrt_torso_solve(q_snapshot);
    ROS_INFO_STREAM("from fwd kin: hand origin = "<<Affine_flange_wrt_utorso.translation().transpose());
    ROS_INFO("orientation: ");
    cout<<Affine_flange_wrt_utorso.linear()<<endl;
    
    //compute inverse kinematics options:
    std::vector<Vectorq7x1> q_solns;    
    int nsolns = baxter_IK_solver.ik_solve_approx_wrt_torso(Affine_flange_wrt_utorso,q_solns);  
    cout<<"num solns: "<<nsolns<<endl;  
    
        Vectorq7x1 q_vec_display;

        cout<<"commanding the solutions..."<<endl;

       for (int isoln=0; isoln<nsolns; isoln++) {
           
         des_path.clear();
         q_in_vecxd = q_vec_right_arm; // start from current pose
         des_path.push_back(q_in_vecxd); 
         q_in_vecxd= q_solns[isoln]; //next goal
         des_path.push_back(q_in_vecxd); 
         q_vec_display= q_solns[isoln]; //next goal
        Affine_flange_wrt_utorso = baxter_fwd_solver.fwd_kin_flange_wrt_torso_solve(q_vec_display);
        ROS_INFO("soln %d",isoln);
        cout<<"qvec: "<<q_in_vecxd.transpose()<<endl;
        ROS_INFO_STREAM("from fwd kin: hand origin = "<<Affine_flange_wrt_utorso.translation().transpose());
        ROS_INFO("orientation: ");
        cout<<Affine_flange_wrt_utorso.linear()<<endl;            
        //convert path to a trajectory by adding timing:
        baxter_traj_streamer_obj.stuff_trajectory(des_path, des_trajectory); 
 
         goal.trajectory = des_trajectory; //fill an action message
         action_client.sendGoal(goal); //no callback fnc specified
         action_client.waitForResult(); //wait indefinitely for action server to finish
            //ros::Duration(2).sleep(); //sleep for 2 seconds
            ros::spinOnce();            

        }
    return 0;
}
