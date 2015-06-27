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

// COMMAND MODES: check the above header file for latest
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

//global vec for current joint angles of right arm
Vectorq7x1 g_q_vec_right_arm; //use this for current joint-space pose of robot    

//global vec to contain optimal path from planners
//treating this vector as global, so can be accessed by "main"
std::vector<Eigen::VectorXd> g_optimal_path;
trajectory_msgs::JointTrajectory g_des_trajectory; // global trajectory object

// define a class to use:

class ArmMotionInterface {
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber, service, and publisher
    //ros::Subscriber minimal_subscriber_; //these will be set up within the class constructor, hiding these ugly details
    ros::ServiceServer arm_motion_interface_service_;
    //ros::Publisher  minimal_publisher_;

    //globals for main/callback sync:
    bool g_received_new_request = false;
    bool g_busy_working_on_a_request = false;

    //globals for CB to populate:
    geometry_msgs::PoseStamped g_poseStamped_start_rqst;
    geometry_msgs::PoseStamped g_poseStamped_goal_rqst;

    int g_command_mode = ARM_TEST_MODE;
    int g_plan_id_rqst = 0;
    int g_plan_id_resp = 0;
    bool g_bool_resp = false;
    int g_rtn_code = 0;

    cwru_srv::arm_nav_service_messageRequest g_request;

    std_msgs::Float32 g_q_vec_start_msg[], g_q_vec_end_msg[];
    //these may be specified in request, if desire offset cartesian move
    Eigen::Vector3d delta_p_;


    Eigen::VectorXd g_q_vec_start_rqst;
    Eigen::VectorXd g_q_vec_end_rqst;
    Eigen::VectorXd g_q_vec_start_resp;
    Eigen::VectorXd g_q_vec_end_resp;
    //Eigen::Affine3d A_start;
    //Eigen::Affine3d A_end;
    Eigen::Affine3d g_a_tool_start, g_a_tool_end;

    Eigen::VectorXd g_q_in_vecxd;

    bool path_is_valid_;
    int path_id_;

    //treating this vector as global, so can be accessed by "main"
    //std::vector<Eigen::VectorXd> g_optimal_path;
    //trajectory_msgs::JointTrajectory g_des_trajectory; // an empty trajectory

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
    void pack_qstart(cwru_srv::arm_nav_service_messageResponse& response);
    void pack_qend(cwru_srv::arm_nav_service_messageResponse& response);
public:
    ArmMotionInterface(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition

    ~ArmMotionInterface(void) {
    }
    void go_to_predefined_pre_pose(void);
    bool plan_path_to_predefined_pre_pose(void);

    bool plan_cartesian_delta_p(Vectorq7x1 q_start, Eigen::Vector3d delta_p);

    bool plan_path_qstart_to_Agoal(Vectorq7x1 q_start); // provide q_start; assumes use of g_a_tool_end and fills in g_optimal_path
    
    bool plan_jspace_path_qstart_to_qend(Vectorq7x1 q_start,Vectorq7x1 q_goal);

    bool isBusy(void) {
        return g_busy_working_on_a_request;
    }

    bool newRqst(void) {
        return g_received_new_request;
    }

    void setNewRqst(bool rcvd_new_rqst) {
        g_received_new_request = rcvd_new_rqst;
    } // 

    void setIsBusy(bool isBusy) {
        g_busy_working_on_a_request = isBusy;
    } // 

    int get_cmd_mode(void) {
        return g_command_mode;
    }

    Eigen::VectorXd get_start_qvec(void) {
        return g_q_vec_start_rqst;
    }

    Eigen::VectorXd get_end_qvec(void) {
        return g_q_vec_end_rqst;
    }

    void setPathValid(bool is_valid) {
        path_is_valid_ = is_valid;
    }

    bool getPathValid(void) {
        return path_is_valid_;
    }

    void incrementPathID(void) {
        path_id_++;
    }
    bool unpack_goal_pose(void);
    bool unpack_qstart(void);
    bool unpack_qend(void);
    bool unpack_delta_p(void);

};

//CONSTRUCTOR: pass in a node handle;

ArmMotionInterface::ArmMotionInterface(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { // constructor
    ROS_INFO("in class constructor of ArmMotionInterface");
    //initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    //initializePublishers();
    initializeServices();
    //cout<<"done initializing service; initializing member vars: ";

    //initialize variables here, as needed
    g_q_pre_pose << -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, -2.87179;
    g_q_vec_start_rqst = g_q_pre_pose; // 0,0,0,0,0,0,0; // make this a 7-d vector
    g_q_vec_end_rqst = g_q_pre_pose; //<< 0,0,0,0,0,0,0;
    g_q_vec_start_resp = g_q_pre_pose; //<< 0,0,0,0,0,0,0;
    g_q_vec_end_resp = g_q_pre_pose; //<< 0,0,0,0,0,0,0;
    g_R_gripper_down = cartTrajPlanner_.get_R_gripper_down();

    path_is_valid_ = false;
    path_id_ = 0;
    // can also do tests/waits to make sure all required services, topics, etc are alive
}

void ArmMotionInterface::initializeServices() {
    ROS_INFO("Initializing cartMoveSvc");
    arm_motion_interface_service_ = nh_.advertiseService("cartMoveSvc",
            &ArmMotionInterface::cartMoveSvcCB,
            this);
    // add more services here, as needed
}

bool ArmMotionInterface::cartMoveSvcCB(cwru_srv::arm_nav_service_messageRequest& request, cwru_srv::arm_nav_service_messageResponse& response) {
    //if busy, refuse new requests;
    if (g_busy_working_on_a_request || g_received_new_request) {
        response.bool_resp = false; // dummy; //working_on_trajectory; // return status of "working on trajectory"
        response.rtn_code = ARM_REQUEST_REJECTED_ALREADY_BUSY;
        return false; //redundant way to say request was rejected      
    }

    // for a simple status query, handle it now;
    if (request.cmd_mode == ARM_IS_SERVER_BUSY_QUERY) {
        ROS_INFO("rcvd request for query--IS_SERVER_BUSY_QUERY");
        if (g_busy_working_on_a_request) {
            response.rtn_code = ARM_SERVER_IS_BUSY;
        } else {
            response.rtn_code = ARM_SERVER_NOT_BUSY;
        }
        return true;
    }

    if (request.cmd_mode == ARM_QUERY_IS_PATH_VALID) {
        ROS_INFO("rcvd request for query--ARM_QUERY_IS_PATH_VALID");
        if (path_is_valid_) {
            response.rtn_code = ARM_PATH_IS_VALID;
            return true;
        } else {
            response.rtn_code = ARM_PATH_NOT_VALID;
            return false; //hmm--don't want to confuse valid transaction w/ status?
        }
    }

    //quick command requesting the internal value of q_start:
    if (request.cmd_mode == ARM_GET_Q_DATA) {
        ROS_INFO("rcvd request for query--GET_Q_DATA");
        pack_qstart(response);
        pack_qend(response);
        response.rtn_code = ARM_RECEIVED_AND_COMPLETED_RQST;
        return true;
    }

    //if here, ready to accept a new command
    g_request = request;


    g_command_mode = request.cmd_mode;
    g_received_new_request = true; // alert "main" that a new request came in

    // copy the message data to globals:
    /*
    g_poseStamped_start_rqst =    request.poseStamped_start;
    g_poseStamped_goal_rqst = request.poseStamped_goal;
    g_q_vec_start_msg = request.q_vec_start;
    g_q_vec_end_msg= request.q_vec_end;   
    g_plan_id_rqst = request.plan_id;
     */
    response.bool_resp = true; // dummy; //working_on_trajectory; // return status of "working on trajectory"
    response.rtn_code = ARM_RECEIVED_AND_INITIATED_RQST;
    return true;
}

void ArmMotionInterface::go_to_predefined_pre_pose(void) {
    //start from current jspace pose:
    cout << "called go_to_predefined_pre_pose" << endl;
    cout << "setting q_start and q_end: " << endl;
    g_q_vec_end_resp = g_q_pre_pose;
    g_q_vec_end_resp = g_q_vec_right_arm;
    //compute a path here:

    //g_q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm(); 

}

//given q_start, compute the tool-flange pose, and compute a path to move delta_p with R fixed
// return the optimized joint-space path in optimal_path
// this fnc can be used fairly generally--e.g., special cases such as 20cm descent from current arm pose 

bool ArmMotionInterface::plan_cartesian_delta_p(Vectorq7x1 q_start, Eigen::Vector3d delta_p) {
    //ROS_INFO("attempting Cartesian path plan for delta-p = %f, %f, %f",delta_p_(0),delta_p_(1),delta_p_(2));
    cout << delta_p.transpose() << endl;
    // this fnc will put the optimal_path result in a global vector, accessible by main
    path_is_valid_ = cartTrajPlanner_.cartesian_path_planner_delta_p(q_start, delta_p, g_optimal_path);
    if (path_is_valid_) {
        ROS_INFO("plan_cartesian_delta_p: computed valid delta-p path");
        g_q_vec_start_resp = g_optimal_path.front();
        g_q_vec_end_resp = g_optimal_path.back();
    } else {
        ROS_WARN("plan_cartesian_delta_p: path plan attempt not successful");
    }

    return path_is_valid_;
}

//take in q_start and q_end and build trivial path in g_optimal_path for pure joint-space move
bool ArmMotionInterface::plan_jspace_path_qstart_to_qend(Vectorq7x1 q_start,Vectorq7x1 q_goal) {
    ROS_INFO("setting up a joint-space path");
    path_is_valid_ = cartTrajPlanner_.jspace_trivial_path_planner(q_start, q_goal, g_optimal_path);
    return path_is_valid_;
}

bool ArmMotionInterface::plan_path_to_predefined_pre_pose(void) {
    cout << "called plan_path_to_predefined_pre_pose" << endl;
    cout << "setting q_start and q_end: " << endl;
    g_q_vec_end_resp = g_q_pre_pose;
    cout << "q pre-pose goal: " << g_q_vec_end_resp.transpose() << endl;
    g_q_vec_start_resp = g_q_vec_right_arm;
    cout << "q start = current arm state: " << g_q_vec_start_resp.transpose() << endl;
    //plan a path: from q-start to q-goal...what to do with wrist??
    cout<<"NOT DONE YET"<<endl;
    path_is_valid_ =false;
    return path_is_valid_;

}

bool ArmMotionInterface::plan_path_qstart_to_Agoal(Vectorq7x1 q_start) { // provide q_start; assumes use of g_a_tool_end and fills in g_optimal_path 
    path_is_valid_ = cartTrajPlanner_.cartesian_path_planner(q_start, g_a_tool_end, g_optimal_path);
    return path_is_valid_;
}

//convert float32[] from request into Eigen type for joint-space start vector;

bool ArmMotionInterface::unpack_qstart(void) {
    //Eigen::VectorXd g_q_vec_start_rqst;
    //g_request
    int njoints = g_request.q_vec_start.size();
    cout << "size of request q_start: " << njoints << endl;
    if (njoints != 7) {
        return false;
    }
    for (int i = 0; i < 7; i++) {
        g_q_vec_start_rqst[i] = g_request.q_vec_start[i];
    }
    return true;
}

//convert float32[] from request into Eigen type for joint-space end vector;

bool ArmMotionInterface::unpack_qend(void) {
    int njoints = g_request.q_vec_end.size();
    //cout<<"size of request q_end: "<< njoints<<endl;
    if (njoints != 7) {
        ROS_WARN("received bad joint-space goal: njoints = %d",njoints);
        return false;
    }
    for (int i = 0; i < 7; i++) {
        g_q_vec_end_rqst[i] = g_request.q_vec_end[i];
    }
    cout<<"unpacked q_vec_goal from request: "<<g_q_vec_end_rqst.transpose()<<endl;
    return true;
}

//convert goal pose from request into Eigen::Affine type for end pose;
// Eigen::Affine3d g_a_tool_start,g_a_tool_end;
// populate g_a_tool_end from g_request.poseStamped_goal

bool ArmMotionInterface::unpack_goal_pose(void) {
    geometry_msgs::PoseStamped poseStamped_goal = g_request.poseStamped_goal;
    Eigen::Vector3d goal_origin;
    goal_origin[0] = poseStamped_goal.pose.position.x;
    goal_origin[1] = poseStamped_goal.pose.position.y;
    goal_origin[2] = poseStamped_goal.pose.position.z;
    g_a_tool_end.translation() = goal_origin;

    Eigen::Quaterniond quat;
    quat.x() = poseStamped_goal.pose.orientation.x;
    quat.y() = poseStamped_goal.pose.orientation.y;
    quat.z() = poseStamped_goal.pose.orientation.z;
    quat.w() = poseStamped_goal.pose.orientation.w;
    Eigen::Matrix3d R_goal(quat);
    g_a_tool_end.linear() = R_goal;


    return true;
}

// need this fnc if client requests a specific delta-p move

bool ArmMotionInterface::unpack_delta_p(void) {
    int npts = g_request.delta_p.size();
    if (npts != 3) {
        ROS_WARN("plan_cartesian_delta_p: request delta_p is wrong size");
        return false;
    }
    //copy message data to member var:
    for (int i = 0; i < 3; i++) delta_p_(i) = g_request.delta_p[i];
    cout << "requested delta_p: " << delta_p_.transpose() << endl;
}

// fill qstart in response message

void ArmMotionInterface::pack_qstart(cwru_srv::arm_nav_service_messageResponse& response) {
    response.q_vec_start.clear();
    for (int i = 0; i < 7; i++) {
        response.q_vec_start.push_back(g_q_vec_start_resp[i]);
    }
}


// fill qend in response message

void ArmMotionInterface::pack_qend(cwru_srv::arm_nav_service_messageResponse& response) {
    response.q_vec_end.clear();
    for (int i = 0; i < 7; i++) {
        response.q_vec_end.push_back(g_q_vec_end_resp[i]);
    }
}

// This function will be called once when the goal sent to the associated action server is complete
// this is optional, but it is a convenient way to get access to the "result" message sent by the server

void doneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d", result->return_val, result->traj_id);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_cart_path_planner_lib");
    ros::NodeHandle nh; //standard ros node handle   

    ROS_INFO("instantiating an ArmMotionInterface: ");
    ArmMotionInterface armMotionInterface(&nh);


    //can do this, if needed: A = cartTrajPlanner.get_fk_Affine_from_qvec(Vectorq7x1 q_vec)    

    cout << "instantiating a traj streamer" << endl; // enter 1:";
    //cin>>ans;
    // let the streamer be owned by "main"
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }
    cout << "getting current right-arm pose:" << endl;
    g_q_vec_right_arm[0] = 1000;
    while (fabs(g_q_vec_right_arm[0]) > 3) { // keep trying until see viable value populated by fnc
        g_q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm();
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    cout << "r_arm state:" << g_q_vec_right_arm.transpose() << endl;

    //let's leave this in main:

    ROS_INFO("instantiating an action client of trajActionServer");
    actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> traj_streamer_action_client("trajActionServer", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for server trajActionServer: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = traj_streamer_action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
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

    ROS_INFO("connected to action server"); // if here, then we connected to the server;    


    bool unpackok;
    Eigen::Vector3d delta_p;
    bool is_valid;
    bool finished_before_timeout;
    baxter_traj_streamer::trajGoal goal;
    Vectorq7x1 q_vec_goal_vq7; //goal pose in joint space, expressed as a 7x1 vector
    
    // start servicing requests:
    while (ros::ok()) {
        g_q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm(); // update the current sensed arm angles
        //decide if should start processing a new request:
        if (armMotionInterface.newRqst()&&!armMotionInterface.isBusy()) {
            armMotionInterface.setNewRqst(false); //g_received_new_request=false; // reset trigger to receive a new request
            armMotionInterface.setIsBusy(true); //g_busy_working_on_a_request= true; // begin processing new request
        }
        if (armMotionInterface.isBusy()) {
            switch (armMotionInterface.get_cmd_mode()) {
                case ARM_TEST_MODE:
                    ROS_INFO("responding to request TEST_MODE: ");
                    cout << "unpacking q_vec_start from request message..." << endl;
                    unpackok = armMotionInterface.unpack_qstart();
                    if (unpackok) {
                        cout << "start qvec rqst: " << armMotionInterface.get_start_qvec().transpose() << endl;
                    }
                    unpackok = armMotionInterface.unpack_qend();
                    if (unpackok) {
                        cout << "end qvec rqst: " << armMotionInterface.get_end_qvec().transpose() << endl;
                    }
                    //reply w/ populating q_vec_start in response:

                    armMotionInterface.setIsBusy(false);
                    break;
                    
                case ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL: 
                    ROS_INFO("case ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL");
                    //get the start q_vec = current arm pose:
                    cout << "getting current right-arm pose:" << endl;
                    g_q_vec_right_arm[0] = 1000;  //Vectorq7x1 g_q_vec_right_arm;
                    while (fabs(g_q_vec_right_arm[0]) > 3) { // keep trying until see viable value populated by fnc
                        g_q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm();
                        ros::spinOnce();
                        ros::Duration(0.01).sleep();
                    }
                    //get the goal q_vec, per request message:
                    armMotionInterface.unpack_qend();
                    q_vec_goal_vq7= armMotionInterface.get_end_qvec(); //need to convert from VectorXd to Vectorq7x1
                    is_valid = armMotionInterface.plan_jspace_path_qstart_to_qend(g_q_vec_right_arm,q_vec_goal_vq7);
                    if (is_valid) ROS_INFO("computed valid path");
                    else ROS_INFO("no valid path found");
                    armMotionInterface.setIsBusy(false);
                    break;                    
                

                case ARM_PLAN_PATH_CURRENT_TO_PRE_POSE:
                    ROS_INFO("responding to request PLAN_PATH_CURRENT_TO_PRE_POSE");
                    armMotionInterface.plan_path_to_predefined_pre_pose();
                    armMotionInterface.setIsBusy(false);
                    break;

                case ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE:
                    cout << "getting current right-arm pose:" << endl;
                    g_q_vec_right_arm[0] = 1000;
                    while (fabs(g_q_vec_right_arm[0]) > 3) { // keep trying until see viable value populated by fnc
                        g_q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm();
                        ros::spinOnce();
                        ros::Duration(0.01).sleep();
                    }
                    //armMotionInterface.unpack_qstart(); // fills in g_q_vec_start_rqst--no...use current arm pose
                    armMotionInterface.unpack_goal_pose(); //fills in g_a_tool_end;  should check return status   
                    //    bool plan_path_qstart_to_Agoal(Vectorq7x1 q_start); // provide q_start; assumes use of g_a_tool_end and fills in g_optimal_path 
                    is_valid = armMotionInterface.plan_path_qstart_to_Agoal(g_q_vec_right_arm);
                    if (is_valid) ROS_INFO("computed valid path");
                    else ROS_INFO("no valid path found");
                    armMotionInterface.setIsBusy(false);
                    break;

                case ARM_DESCEND_20CM:
                    ROS_INFO("responding to request DESCEND_20CM");
                    delta_p << 0, 0, -0.2; //specify delta-p for a 20cm descent
                    // find a joint-space path to do this, holding R fixed

                    cout << "getting current right-arm pose:" << endl;
                    g_q_vec_right_arm[0] = 1000;
                    while (fabs(g_q_vec_right_arm[0]) > 3) { // keep trying until see viable value populated by fnc
                        g_q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm();
                        ros::spinOnce();
                        ros::Duration(0.01).sleep();
                    }

                    is_valid = armMotionInterface.plan_cartesian_delta_p(g_q_vec_right_arm, delta_p);
                    armMotionInterface.setPathValid(is_valid);
                    armMotionInterface.incrementPathID();
                    armMotionInterface.setIsBusy(false);
                    break;
                case ARM_DEPART_20CM:
                    ROS_INFO("responding to request ARM_DEPART_20CM");
                    delta_p << 0, 0, 0.2; //specify delta-p for a 20cm descent
                    // find a joint-space path to do this, holding R fixed

                    cout << "getting current right-arm pose:" << endl;
                    g_q_vec_right_arm[0] = 1000;
                    while (fabs(g_q_vec_right_arm[0]) > 3) { // keep trying until see viable value populated by fnc
                        g_q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm();
                        ros::spinOnce();
                        ros::Duration(0.01).sleep();
                    }

                    is_valid = armMotionInterface.plan_cartesian_delta_p(g_q_vec_right_arm, delta_p);
                    armMotionInterface.setPathValid(is_valid);
                    armMotionInterface.incrementPathID();
                    armMotionInterface.setIsBusy(false);
                    break;

                    //case DEPART_20CM:
                    //case CART_MOVE_DELTA_P: unpack_delta_p()...
                    //case PLAN_PATH_QSTART_TO_ADES:
                    //case PLAN_PATH_QSTART_TO_QGOAL:
                    //case PLAN_PATH_ASTART_TO_QGOAL:

                case ARM_EXECUTE_PLANNED_PATH: //assumes there is a valid planned path in g_optimal_path
                    ROS_INFO("responding to request EXECUTE_PLANNED_PATH");
                    if (armMotionInterface.getPathValid()) {
                        // convert path to a trajectory:
                        baxter_traj_streamer.stuff_trajectory(g_optimal_path, g_des_trajectory); //convert from vector of 7dof poses to trajectory message   
                        goal.trajectory = g_des_trajectory;
                        ROS_INFO("sending action request to traj streamer node");
                        traj_streamer_action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
                        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

                        finished_before_timeout = traj_streamer_action_client.waitForResult(ros::Duration(5.0));

                        if (!finished_before_timeout) {
                            ROS_WARN("EXECUTE_PLANNED_PATH: giving up waiting on result");
                            // should set status in service request here...
                        } else {
                            ROS_INFO("finished before timeout");
                            // should set status in service request here...
                        }
                    } else {
                        ROS_WARN("requested move without valid path! doing nothing");
                    }
                    armMotionInterface.setIsBusy(false);
                    break;
                default:
                    ROS_WARN("this command mode is not defined: %d", armMotionInterface.get_cmd_mode());
                    //clean up/terminate:
                    armMotionInterface.setIsBusy(false);
            }
        }
        ros::spinOnce();
        //cout<<"main loop..."<<endl;
        ros::Duration(0.1).sleep(); //don't consume much cpu time if not actively working on a command
    }

    return 0;
}
