// test_cart_path_planner_lib: 
// wsn, June, 2015
// start of higher-level node to accept commands and perform planning and motion requests
// requires trajActionServer is running (rosrun baxter_traj_streamer traj_interpolator_as)
// at present, talk to this node via service: cartMoveSvc

// uses library of arm-motion planning functions
#include <cartesian_moves/cart_path_planner_lib.h>
#include <cwru_srv/arm_nav_service_message.h>
#include <cwru_srv/simple_int_service_message.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include<baxter_traj_streamer/trajAction.h>
#include<std_msgs::float32>

//if these continue to be used, put them in a header file
// COMMAND MODES:
const int UNDEFINED_MODE=0;
const int GO_TO_PREDFINED_PRE_POSE=1;
const int DESCEND_20CM=2;
const int DEPART_20CM=3;

const int IS_SERVER_BUSY_QUERY = 1;
const int PLAN_PATH_QSTART_TO_ADES = 4;
const int PLAN_PATH_QSTART_TO_QGOAL = 5;
const int PLAN_PATH_ASTART_TO_QGOAL = 6;



//responses...
const int RECEIVED_AND_INITIATED_RQST=1;
const int REQUEST_REJECTED_ALREADY_BUSY=2;
const int SERVER_NOT_BUSY=3;
const int SERVER_IS_BUSY=4;
 
// create a class to use:
class armMotionInterface {
private:

    //ros::NodeHandle nh_; // if need a node handle, get one upon instantiation
    

    Eigen::Vector3d n_des_, t_des_, b_des_;
    
    Eigen::Affine3d a_tool_start_,a_tool_end_;
    Eigen::Matrix3d R_gripper_down_;
    std::vector<Eigen::VectorXd> optimal_path_;
    
    Baxter_IK_solver baxter_IK_solver_; // instantiate an IK solver
    Baxter_fwd_solver baxter_fwd_solver_; //instantiate a forward-kinematics solver   
    
    // use this classes baxter fk solver to compute and return tool-flange pose w/rt torso, given right-arm joint angles  
    Eigen::Affine3d get_fk_Affine_from_qvec(Vectorq7x1 q_vec);

public:
    CartTrajPlanner(); //define the body of the constructor outside of class definition

    ~CartTrajPlanner(void) {
    }
    //specify start and end poses w/rt torso.  Only orientation of end pose will be considered; orientation of start pose is ignored
    bool cartesian_path_planner(Eigen::Affine3d a_tool_start,Eigen::Affine3d a_tool_end, std::vector<Eigen::VectorXd> &optimal_path);
    // alt version: specify start as a q_vec, and goal as a Cartesian pose (w/rt torso)    
    bool cartesian_path_planner(Vectorq7x1 q_start,Eigen::Affine3d a_tool_end, std::vector<Eigen::VectorXd> &optimal_path);
    bool cartesian_path_planner_wrist(Vectorq7x1 q_start,Eigen::Affine3d a_tool_end, std::vector<Eigen::VectorXd> &optimal_path);
   
    Eigen::Matrix3d get_R_gripper_down(void) { return R_gripper_down_;}
    

};


//globals for main/callback sync:
bool g_received_new_request=false;
bool g_busy_working_on_a_request=false;

//globals for CB to populate:
geometry_msgs::PoseStamped g_poseStamped_start_rqst;
geometry_msgs::PoseStamped g_poseStamped_goal_rqst;

int g_command_mode=UNDEFINED_MODE;
int g_plan_id_rqst = 0;
int g_plan_id_resp = 0;
bool g_bool_resp = false;
int  g_rtn_code = 0;
std_msgs::float32 g_q_vec_start_msg[],g_q_vec_end_msg[];



Eigen::VectorXd g_q_vec_start_rqst;
Eigen::VectorXd g_q_vec_end_rqst;
Eigen::VectorXd g_q_vec_start_resp;
Eigen::VectorXd g_q_vec_end_resp;
//Eigen::Affine3d A_start;
//Eigen::Affine3d A_end;
Eigen::Affine3d g_a_tool_start,g_a_tool_end;  

Vectorq7x1 g_q_vec_right_arm; //use this for current joint-space pose of robot

// these can morph into member data

std::vector<Eigen::VectorXd> g_optimal_path;
trajectory_msgs::JointTrajectory g_des_trajectory; // an empty trajectory
        
//some handy constants...
Eigen::Matrix3d g_R_gripper_down;
Vectorq7x1 g_q_pre_pose;
    
bool cartMoveSvcCB(cwru_srv::arm_nav_service_messageRequest& request, cwru_srv::arm_nav_service_message_service_messageResponse& response) {
    //if busy, refuse new requests;
    if(g_busy_working_on_a_request||g_received_new_request) {
        response.bool_resp = false; // dummy; //working_on_trajectory; // return status of "working on trajectory"
        response.rtn_code = REQUEST_REJECTED_ALREADY_BUSY;
        return false;  //redundant way to say request was rejected      
    }
    
    // for a simple status query, handle it now;
    if (request.cmd_mode == IS_SERVER_BUSY_QUERY) {
        if ((g_busy_working_on_a_request) {
            response.rtn_code = SERVER_IS_BUSY;
        }
        else {
            response.rtn_code = SERVER_NOT_BUSY;
        }
        return true;  
    }
    
    //if here, ready to accept a new command
    g_command_mode = request.cmd_mode;
    g_received_new_request=true; // alert "main" that a new request came in
 
    // copy the message data to globals:
    g_poseStamped_start_rqst =    request.poseStamped_start;
    g_poseStamped_goal_rqst = request.poseStamped_goal;
    g_q_vec_start_msg = request.q_vec_start;
    g_q_vec_end_msg= request.q_vec_end;   
    g_plan_id_rqst = request.plan_id;

    response.bool_resp = true; // dummy; //working_on_trajectory; // return status of "working on trajectory"
    response.rtn_code = RECEIVED_RQST;
    return true;
}

void go_to_predefined_pre_pose(void) {
    //start from current jspace pose:
    g_q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm(); 
    
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_cart_path_planner_lib");
    ros::NodeHandle nh; //standard ros node handle    
    Eigen::VectorXd q_in_vecxd;
 
    g_q_pre_pose<< -0.907528, -0.111813,   2.06622,    1.8737,    -1.295,   2.00164,  -2.87179; 
        
    ROS_INFO("instantiating a cartesian planner object: ");   
    CartTrajPlanner cartTrajPlanner;
    g_R_gripper_down = cartTrajPlanner.get_R_gripper_down();
    
    ROS_INFO("Initializing service: cartMoveSvc");
    ros::ServiceServer cartMoveSvc = nh.advertiseService("cartMoveSvc",cartMoveSvcCB); 
    
    
    //can do this, if needed: A = cartTrajPlanner.get_fk_Affine_from_qvec(Vectorq7x1 q_vec)    
    
    cout<<"instantiating a traj streamer"<<endl; // enter 1:";
        //cin>>ans;
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    cout<<"warming up callbacks..."<<endl;
    for (int i=0;i<100;i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }
    cout<<"getting current right-arm pose:"<<endl;
    g_q_vec_right_arm[0] = 1000;
    while (fabs(g_q_vec_right_arm[0])>3) { // keep trying until see viable value populated by fnc
     g_q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm();       
    }
    cout<<"r_arm state:"<<g_q_vec_right_arm.transpose()<<endl; 
    
    ROS_INFO("instantiating an action client of trajActionServer");
    actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> action_client("trajActionServer", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server trajActionServer: ");
        bool server_exists = false;
        while ((!server_exists)&&(ros::ok())) {
            server_exists= action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            ROS_INFO("retrying...");
        }
            
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
       // if (!server_exists) {
       //     ROS_WARN("could not connect to server; will wait forever");
        //    return 0; // bail out; optionally, could print a warning message and retry
        //}
        //server_exists = action_client.waitForServer(); //wait forever 
               
        ROS_INFO("connected to action server");  // if here, then we connected to the server;    
        
        // start servicing requests:
        while (ros::ok())  {
            if (g_received_new_request&&!g_busy_working_on_a_request) {
                g_received_new_request=false; // reset trigger to receive a new request
                g_busy_working_on_a_request= true; // begin processing new request
            }
            if (g_busy_working_on_a_request) {
                switch (g_command_mode) {

                    case GO_TO_PREDFINED_PRE_POSE : 
                        go_to_predefined_pre_pose();
                        g_busy_working_on_a_request=false;
                        break;
                    //case DESCEND_20CM:
                    //case DEPART_20CM:
                    //case PLAN_PATH_QSTART_TO_ADES:
                    //case PLAN_PATH_QSTART_TO_QGOAL:
                    //case PLAN_PATH_ASTART_TO_QGOAL:
                    case UNDEFINED_MODE: 
                    default : 
                        ROS_WARN("this command mode is not defined: %d",g_command_mode);
                        //clean up/terminate:
                        g_busy_working_on_a_request=false;
                }
            }
            ros::spinOnce();
            ros::Duration(0.1).sleep(); //don't consume much cpu time if not actively working on a command
        }
    

    //a_tool_end.translation() = p_des;
    //cout<<"p_des: "<<p_des.transpose()<<endl;
    //cout<<"R_des: "<<endl;
    //cout<<R_gripper_down<<endl;
    
/*

    bool valid_path;
    
    valid_path = cartTrajPlanner.cartesian_path_planner_wrist(g_q_pre_pose,g_a_tool_end,g_optimal_path);
    int npts = g_optimal_path.size();
    cout<<"path npts = "<<npts<<endl;
    for (int i=0;i<npts;i++) {
        cout<<g_optimal_path[i].transpose()<<endl;
    }
    xxx

    q_in_vecxd = q_vec_right_arm; // start from here;
    des_path.push_back(q_in_vecxd); //put all zeros here
    q_in_vecxd = q_pre_pose; // conversion; not sure why I needed to do this...but des_path.push_back(q_in_vecxd) likes it
    des_path.push_back(q_in_vecxd); //twice, to define a trajectory  
    

    cout << "stuffing traj: " << endl;
    baxter_traj_streamer.stuff_trajectory(des_path, des_trajectory); //convert from vector of 7dof poses to trajectory message        
        // here is a "goal" object compatible with the server, as defined in example_action_server/action
        baxter_traj_streamer::trajGoal goal; 
        // does this work?  copy traj to goal:
        goal.trajectory = des_trajectory;
        //cout<<"ready to connect to action server; enter 1: ";
        //cin>>ans;
        // use the name of our server, which is: trajActionServer (named in traj_interpolator_as.cpp)
        actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> action_client("trajActionServer", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running


        if (!server_exists) {
            ROS_WARN("could not connect to server; will wait forever");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        server_exists = action_client.waitForServer(); //wait forever 
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        //while(true) {
        // stuff a goal message:
        g_count++;
        goal.traj_id = g_count; // this merely sequentially numbers the goals sent
        ROS_INFO("sending traj_id %d",g_count);
        //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
        
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for goal number %d",g_count);
            return 0;
        }
        else {
            ROS_INFO("finished before timeout");
        }   
*/
    return 0;
}
