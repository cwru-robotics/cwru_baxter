// arm_motion_interface: 
// wsn, June, 2015
// start of higher-level node to accept commands and perform planning and motion requests
// requires trajActionServer is running (rosrun baxter_traj_streamer traj_interpolator_as)
// at present, talk to this node via service: cartMoveSvc
// likely, later change to an action server

// uses library of arm-motion planning functions
#include <cartesian_moves/cart_path_planner_lib.h>
#include <cwru_srv/arm_nav_service_message.h>
#include <cwru_srv/simple_int_service_message.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include<baxter_traj_streamer/trajAction.h>
#include<std_msgs/Float32.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>

#include <cartesian_moves/arm_motion_interface_defs.h>
//if these continue to be used, put them in a header file
// COMMAND MODES:
/*
const int TEST_MODE =0;
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
 */

// define a class to use:
class ArmMotionInterface {
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    //ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::ServiceServer arm_motion_interface_service_;
    //ros::Publisher  minimal_publisher_;
    
//globals for main/callback sync:
    bool g_received_new_request=false;
    bool g_busy_working_on_a_request=false;

    //globals for CB to populate:
    geometry_msgs::PoseStamped g_poseStamped_start_rqst;
    geometry_msgs::PoseStamped g_poseStamped_goal_rqst;

    int g_command_mode=TEST_MODE;
    int g_plan_id_rqst = 0;
    int g_plan_id_resp = 0;
    bool g_bool_resp = false;
    int  g_rtn_code = 0;
    
    cwru_srv::arm_nav_service_messageRequest  g_request;
    
    std_msgs::Float32 g_q_vec_start_msg[],g_q_vec_end_msg[];



    Eigen::VectorXd g_q_vec_start_rqst;
    Eigen::VectorXd g_q_vec_end_rqst;
    Eigen::VectorXd g_q_vec_start_resp;
    Eigen::VectorXd g_q_vec_end_resp;
//Eigen::Affine3d A_start;
//Eigen::Affine3d A_end;
    Eigen::Affine3d g_a_tool_start,g_a_tool_end;  

    Vectorq7x1 g_q_vec_right_arm; //use this for current joint-space pose of robot
    Eigen::VectorXd g_q_in_vecxd;
// these can morph into member data

    std::vector<Eigen::VectorXd> g_optimal_path;
    trajectory_msgs::JointTrajectory g_des_trajectory; // an empty trajectory
        
    //some handy constants...
    Eigen::Matrix3d g_R_gripper_down;
    Vectorq7x1 g_q_pre_pose;    
    
    //Baxter_IK_solver baxter_IK_solver_; // instantiate an IK solver
    Baxter_fwd_solver baxter_fwd_solver_; //instantiate a forward-kinematics solver 
    CartTrajPlanner cartTrajPlanner_; // from cartesian trajectory planner library
    
    // member methods as well:
    //void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    //void initializePublishers();
    void initializeServices();
    
    //void subscriberCallback(const std_msgs::Float32& message_holder); //prototype for callback of example subscriber
    //prototype for callback for service
    bool cartMoveSvcCB(cwru_srv::arm_nav_service_messageRequest& request, cwru_srv::arm_nav_service_messageResponse& response);

public:
    ArmMotionInterface(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition
    ~ArmMotionInterface(void) {
    }
    void go_to_predefined_pre_pose(void);
    bool isBusy(void) { return g_busy_working_on_a_request; }
    bool newRqst(void) { return g_received_new_request; }
    void setNewRqst(bool rcvd_new_rqst) {  g_received_new_request=rcvd_new_rqst;}  // 
    void setIsBusy(bool isBusy) {g_busy_working_on_a_request= isBusy; } // 
    int get_cmd_mode(void) { return g_command_mode; }
    Eigen::VectorXd  get_start_qvec(void) { return g_q_vec_start_rqst; }
    bool unpack_qstart(void);
};

//CONSTRUCTOR: pass in a node handle;
ArmMotionInterface::ArmMotionInterface(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of ArmMotionInterface");
    //initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    //initializePublishers();
    initializeServices();
    
    //initialize variables here, as needed
    g_q_pre_pose<< -0.907528, -0.111813,   2.06622,    1.8737,    -1.295,   2.00164,  -2.87179; 
    g_q_vec_start_rqst<< 0,0,0,0,0,0,0; // make this a 7-d vector
    g_q_vec_end_rqst<< 0,0,0,0,0,0,0;
    g_q_vec_start_resp<< 0,0,0,0,0,0,0;
    g_q_vec_end_resp<< 0,0,0,0,0,0,0;
    g_R_gripper_down = cartTrajPlanner_.get_R_gripper_down();
    
    // can also do tests/waits to make sure all required services, topics, etc are alive
}


void ArmMotionInterface::initializeServices()
{
    ROS_INFO("Initializing cartMoveSvc");
    arm_motion_interface_service_ = nh_.advertiseService("cartMoveSvc",
                                                   &ArmMotionInterface::cartMoveSvcCB,
                                                   this);  
    // add more services here, as needed
}
    
bool ArmMotionInterface::cartMoveSvcCB(cwru_srv::arm_nav_service_messageRequest& request, cwru_srv::arm_nav_service_messageResponse& response) {
    //if busy, refuse new requests;
    if(g_busy_working_on_a_request||g_received_new_request) {
        response.bool_resp = false; // dummy; //working_on_trajectory; // return status of "working on trajectory"
        response.rtn_code = REQUEST_REJECTED_ALREADY_BUSY;
        return false;  //redundant way to say request was rejected      
    }
    
    // for a simple status query, handle it now;
    if (request.cmd_mode == IS_SERVER_BUSY_QUERY) {
        if (g_busy_working_on_a_request) {
            response.rtn_code = SERVER_IS_BUSY;
        }
        else {
            response.rtn_code = SERVER_NOT_BUSY;
        }
        return true;  
    }
    
    //if here, ready to accept a new command
    g_request = request;
    

    g_command_mode = request.cmd_mode;
    g_received_new_request=true; // alert "main" that a new request came in
 
    // copy the message data to globals:
    /*
    g_poseStamped_start_rqst =    request.poseStamped_start;
    g_poseStamped_goal_rqst = request.poseStamped_goal;
    g_q_vec_start_msg = request.q_vec_start;
    g_q_vec_end_msg= request.q_vec_end;   
    g_plan_id_rqst = request.plan_id;
    */
    response.bool_resp = true; // dummy; //working_on_trajectory; // return status of "working on trajectory"
    response.rtn_code = RECEIVED_AND_INITIATED_RQST;
    return true;
}

void ArmMotionInterface::go_to_predefined_pre_pose(void) {
    //start from current jspace pose:
    cout<<"called go_to_predefined_pre_pose"<<endl;
    //g_q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm(); 
     
}

//convert float32[] to Eigen type for joint-space start vector;
bool ArmMotionInterface::unpack_qstart(void) {
    //Eigen::VectorXd g_q_vec_start_rqst;
    //g_request
    int njoints = g_request.q_vec_start.size();
    cout<<"size of request q_start: "<< njoints<<endl;
    if (njoints != 7) {
        return false;
    }
    for (int i=0;i<7;i++) {
        g_q_vec_start_rqst[i] = g_request.q_vec_start[i];
    }
    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "test_cart_path_planner_lib");
    ros::NodeHandle nh; //standard ros node handle   
    
    ROS_INFO("instantiating an ArmMotionInterface: ");
    ArmMotionInterface armMotionInterface(&nh);
      
    
    //can do this, if needed: A = cartTrajPlanner.get_fk_Affine_from_qvec(Vectorq7x1 q_vec)    
    
    /*
    cout<<"instantiating a traj streamer"<<endl; // enter 1:";
        //cin>>ans;
    // let the streamer be owned by "main"
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
    */
    //let's leave this in main:
    /*
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
      */
    
    bool unpackok;
        // start servicing requests:
        while (ros::ok())  {
            //decide if should start processing a new request:
            if (armMotionInterface.newRqst()&&!armMotionInterface.isBusy()) {
                armMotionInterface.setNewRqst(false); //g_received_new_request=false; // reset trigger to receive a new request
                armMotionInterface.setIsBusy(true); //g_busy_working_on_a_request= true; // begin processing new request
            }
            if (armMotionInterface.isBusy()) {
                switch (armMotionInterface.get_cmd_mode()) {
                    case TEST_MODE: 
                        cout<<"testing request data: "<<endl;
                        unpackok=false;
                        //unpackok = armMotionInterface.unpack_qstart();
                        if (unpackok) {
                            cout<<"start qvec: "<<armMotionInterface.get_start_qvec().transpose()<<endl;
                        }
                        armMotionInterface.setIsBusy(false);
                        break;
                        
                    case GO_TO_PREDFINED_PRE_POSE : 
                        armMotionInterface.go_to_predefined_pre_pose();
                        armMotionInterface.setIsBusy(false);
                        break;
                    //case DESCEND_20CM:
                    //case DEPART_20CM:
                    //case PLAN_PATH_QSTART_TO_ADES:
                    //case PLAN_PATH_QSTART_TO_QGOAL:
                    //case PLAN_PATH_ASTART_TO_QGOAL:

                    default : 
                        ROS_WARN("this command mode is not defined: %d",armMotionInterface.get_cmd_mode());
                        //clean up/terminate:
                         armMotionInterface.setIsBusy(false);
                }
            }
            ros::spinOnce();
            cout<<"main loop..."<<endl;
            ros::Duration(0.5).sleep(); //don't consume much cpu time if not actively working on a command
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
