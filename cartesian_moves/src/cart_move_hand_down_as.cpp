// cart_move_hand_down_as: 
// wsn, June, 2015
// cartesian-move action server, specialized for gripper-down orientation
// this action server accepts a goal consisting of a Cartesian destination, x,y,z of the tool flange,
// and imposes gripper orientation down.

//has an action client of interpolator action server;

//goal input is simply 3 coords (origin of tool flange) and scalar speed (m/s)
//this action server assumes current pose as starting position (joint-space constraint),
// does fk to get current flange position, 
// imposes assumed orientation (down),
// interpolates in Cartesian space (optionally, w/ speed arg?)
// solves IK options
// plans a path using stagecoach planner
 
//aborts if: not reachable along path
//       OR: joint-space optimal soln has large jumps

//else: stuffs a trajectory w/ optimal joint-space path;
//      sends this trajectory as a goal message to interpolator action server
 //     returns "success" if interpolator declares success (else "abort")

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../baxter_traj_streamer/action/traj.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (traj) and appended name (Action)
// If you write a new client of the server in this package, you will need to include baxter_traj_streamer in your package.xml,
// and include the header file below
#include<baxter_traj_streamer/trajAction.h>
#include<cartesian_moves/cartTrajAction.h>
#include <baxter_kinematics/baxter_kinematics.h>
//include the following if/when want to plan a joint-space path and execute it
#include <joint_space_planner.h>

#include <string.h>
#include <stdio.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>      /* printf, scanf, puts, NULL */
//#include <stdlib.h>     /* srand, rand */
//#include <time.h>       /* time */
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

//Eigen::Vector3d g_p_start;
//Eigen::Vector3d g_p_end;

    Baxter_IK_solver baxter_IK_solver; // instantiate an IK solver
    Baxter_fwd_solver baxter_fwd_solver; //instantiate a forward-kinematics solver
    
//global pointer to action client
actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> *ac_ptr_; 

Baxter_traj_streamer *baxter_traj_streamer_ptr; //pointer to a traj streamer, so can use globally


Vectorq7x1 q_vec_right_arm; // global arm state
vector<string> g_all_jnt_names; 
vector<int> right_arm_joint_indices;

void map_right_arm_joint_indices(vector<string> joint_names) {
 //vector<string> joint_names = joint_state->name;
    vector<string> rt_limb_jnt_names;
    
            
    right_arm_joint_indices.clear();
    int index;
    int n_jnts = joint_names.size();
    cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;
    std::string j_s0_name ("right_s0");
    rt_limb_jnt_names.push_back(j_s0_name);
    std::string j_s1_name ("right_s1");
   rt_limb_jnt_names.push_back(j_s1_name);    
    std::string j_e0_name ("right_e0");
   rt_limb_jnt_names.push_back(j_e0_name);    
    std::string j_e1_name ("right_e1");
   rt_limb_jnt_names.push_back(j_e1_name);    
    std::string j_w0_name ("right_w0");
   rt_limb_jnt_names.push_back(j_w0_name);    
    std::string j_w1_name ("right_w1");   
   rt_limb_jnt_names.push_back(j_w1_name);    
    std::string j_w2_name ("right_w2");  
   rt_limb_jnt_names.push_back(j_w2_name); 

   for (int j=0;j<7;j++) {
       j_name = rt_limb_jnt_names[j];
       for (int i=0;i<n_jnts;i++) {
        if (j_name.compare(joint_names[i])==0) {
            index = i;
            right_arm_joint_indices.push_back(index);
            break;
        }
    }
   }   
   cout<<"indices of right-arm joints: "<<endl;
   for (int i=0;i<7;i++) {
       cout<<right_arm_joint_indices[i]<<", ";
   }
   cout<<endl;
}

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    if (right_arm_joint_indices.size()<1) {
       //g_all_jnt_names = js_msg.name;
       map_right_arm_joint_indices(js_msg.name);
    }
    // copy right-arm angles to global vec
    for (int i=0;i<7;i++)
    {
        // should do this better; manually remap from joint_states indices to right-arm joint angles
        q_vec_right_arm[i] = js_msg.position[right_arm_joint_indices[i]]; //w2         
    }
    
}    

//specify start and end poses of right-arm flange w/rt torso.  Only orientation of end pose will be considered; orientation of start pose is ignored
bool cartesian_path_planner(Eigen::Affine3d a_flange_start,Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_flange_des;
    Eigen::Matrix3d R_des = a_flange_end.linear();
    a_flange_des.linear() = R_des;

    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;
    Eigen::Vector3d p_des,dp_vec,del_p,p_start,p_end;
    p_start = a_flange_start.translation();
    p_end = a_flange_end.translation();
    del_p = p_end-p_start;
    double dp_scaler = 0.05;
    nsteps = round(del_p.norm()/dp_scaler);
    if (nsteps<1) nsteps=1;
    dp_vec = del_p/nsteps;
    nsteps++; //account for pose at step 0


    std::vector<Vectorq7x1> q_solns;
    p_des = p_start;

    for (int istep=0;istep<nsteps;istep++) 
    {
            a_flange_des.translation() = p_des;
            cout<<"trying: "<<p_des.transpose()<<endl;
            nsolns = baxter_IK_solver.ik_solve_approx_wrt_torso(a_flange_des, q_solns);
            std::cout<<"nsolns = "<<nsolns<<endl;
            single_layer_nodes.clear();
            if (nsolns>0) {
                single_layer_nodes.resize(nsolns);
                for (int isoln = 0; isoln < nsolns; isoln++) {
                    // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                    node = q_solns[isoln];
                    single_layer_nodes[isoln] = node;
                }

                path_options.push_back(single_layer_nodes);
            }
            p_des += dp_vec;
    }

    //plan a path through the options:
    int nlayers = path_options.size();
    if (nlayers < 1) {
        ROS_WARN("no viable options: quitting");
        return false; // give up if no options
    }
    //std::vector<Eigen::VectorXd> optimal_path;
    optimal_path.resize(nlayers);
    double trip_cost;
    // set joint penalty weights for computing optimal path in joint space
    Eigen::VectorXd weights;
    weights.resize(VECTOR_DIM);
    for (int i = 0; i < VECTOR_DIM; i++) {
        weights(i) = 1.0;
    }

    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, weights);
        cout << "recovering the solution..." << endl;
        jsp.get_soln(optimal_path);
        trip_cost = jsp.get_trip_cost();
    }

    //now, jsp is deleted, but optimal_path lives on:
    cout << "resulting solution path: " << endl;
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
    }
    cout << "soln min cost: " << trip_cost << endl;
    return true;
}

// alt version: specify start as a q_vec, and goal as a Cartesian pose (w/rt torso)
bool cartesian_path_planner(Vectorq7x1 q_start,Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_flange_des,a_flange_start;
    Eigen::Matrix3d R_des = a_flange_end.linear();
    a_flange_start = baxter_fwd_solver.fwd_kin_flange_wrt_torso_solve(q_start);
    a_flange_start.linear() = R_des; // override the orientation component--require point down    
    a_flange_des.linear() = R_des;

    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;
    Eigen::Vector3d p_des,dp_vec,del_p,p_start,p_end;
    p_start = a_flange_start.translation();
    p_end = a_flange_end.translation();
    del_p = p_end-p_start;
    double dp_scaler = 0.05;
    nsteps = round(del_p.norm()/dp_scaler);
    if (nsteps<1) nsteps=1;
    dp_vec = del_p/nsteps;
    nsteps++; //account for pose at step 0

    //coerce path to start from provided q_start;
    single_layer_nodes.clear();
    node = q_start;
    single_layer_nodes.push_back(node);
    path_options.push_back(single_layer_nodes);   

    std::vector<Vectorq7x1> q_solns;
    p_des = p_start;

    for (int istep=1;istep<nsteps;istep++) 
    {
            p_des += dp_vec;
            a_flange_des.translation() = p_des;
            cout<<"trying: "<<p_des.transpose()<<endl;
            nsolns = baxter_IK_solver.ik_solve_approx_wrt_torso(a_flange_des, q_solns);
            std::cout<<"nsolns = "<<nsolns<<endl;
            single_layer_nodes.clear();
            if (nsolns>0) {
                single_layer_nodes.resize(nsolns);
                for (int isoln = 0; isoln < nsolns; isoln++) {
                    // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                    node = q_solns[isoln];
                    single_layer_nodes[isoln] = node;
                }

                path_options.push_back(single_layer_nodes);
            }
            p_des += dp_vec;
    }

    //plan a path through the options:
    int nlayers = path_options.size();
    if (nlayers < 1) {
        ROS_WARN("no viable options: quitting");
        return false; // give up if no options
    }
    //std::vector<Eigen::VectorXd> optimal_path;
    optimal_path.resize(nlayers);
    double trip_cost;
    // set joint penalty weights for computing optimal path in joint space
    Eigen::VectorXd weights;
    weights.resize(VECTOR_DIM);
    for (int i = 0; i < VECTOR_DIM; i++) {
        weights(i) = 1.0;
    }

    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, weights);
        cout << "recovering the solution..." << endl;
        jsp.get_soln(optimal_path);
        trip_cost = jsp.get_trip_cost();
    }

    //now, jsp is deleted, but optimal_path lives on:
    cout << "resulting solution path: " << endl;
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
    }
    cout << "soln min cost: " << trip_cost << endl;
    return true;
}

// alt version: look up current q_vec and head to specified goal as a Cartesian pose (w/rt torso)
bool cartesian_path_planner(Eigen::Affine3d a_flange_end, std::vector<Eigen::VectorXd> &optimal_path) {
    Vectorq7x1 q_start;
    q_start = q_vec_right_arm; // populated by callback from robot joint states
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_flange_des,a_flange_start;
    Eigen::Matrix3d R_des = a_flange_end.linear();
    a_flange_start = baxter_fwd_solver.fwd_kin_flange_wrt_torso_solve(q_start);
    a_flange_start.linear() = R_des; // override the orientation component--require point down    
    a_flange_des.linear() = R_des;
    
    Eigen::Vector3d p_des,dp_vec,del_p,p_start,p_end;    
    p_start = a_flange_start.translation();
    p_end = a_flange_end.translation();    
    cout<<"starting tool-flange origin: "<<p_start.transpose()<<endl;
    cout<<"goal tool-flange origin: "<<p_end.transpose()<<endl;

    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;


    del_p = p_end-p_start;
    double dp_scalar = 0.05; //want to base this on speed, really
    nsteps = round(del_p.norm()/dp_scalar);
    if (nsteps<1) nsteps=1;
    cout<<"distance: "<<del_p.norm()<<"; nsteps: "<<nsteps<<" at step size = "<<dp_scalar<<endl;
    dp_vec = del_p/nsteps;
    cout<<"dp_vec: "<<dp_vec.transpose()<<endl;
    nsteps++; //account for pose at step 0

    //coerce path to start from provided q_start;
    single_layer_nodes.clear();
    node = q_start;
    single_layer_nodes.push_back(node);
    path_options.push_back(single_layer_nodes);   

    std::vector<Vectorq7x1> q_solns;
    p_des = p_start;

    for (int istep=1;istep<nsteps;istep++) 
    {
            p_des += dp_vec;
            a_flange_des.translation() = p_des;
            cout<<"trying: "<<p_des.transpose()<<endl;
            nsolns = baxter_IK_solver.ik_solve_approx_wrt_torso(a_flange_des, q_solns);
            std::cout<<"nsolns = "<<nsolns<<endl;
            single_layer_nodes.clear();
            if (nsolns>0) {
                single_layer_nodes.resize(nsolns);
                for (int isoln = 0; isoln < nsolns; isoln++) {
                    // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                    node = q_solns[isoln];
                    single_layer_nodes[isoln] = node;
                }

                path_options.push_back(single_layer_nodes);
            }
    }

    //plan a path through the options:
    int nlayers = path_options.size();
    if (nlayers < 1) {
        ROS_WARN("no viable options: quitting");
        return false; // give up if no options
    }
    //std::vector<Eigen::VectorXd> optimal_path;
    optimal_path.resize(nlayers);
    double trip_cost;
    // set joint penalty weights for computing optimal path in joint space
    Eigen::VectorXd weights;
    weights.resize(VECTOR_DIM);
    for (int i = 0; i < VECTOR_DIM; i++) {
        weights(i) = 1.0;
    }

    // compute min-cost path, using Stagecoach algorithm
    cout << "instantiating a JointSpacePlanner:" << endl;
    { //limit the scope of jsp here:
        JointSpacePlanner jsp(path_options, weights);
        cout << "recovering the solution..." << endl;
        jsp.get_soln(optimal_path);
        trip_cost = jsp.get_trip_cost();
    }

    //now, jsp is deleted, but optimal_path lives on:
    cout << "resulting solution path: " << endl;
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        cout << "ilayer: " << ilayer << " node: " << optimal_path[ilayer].transpose() << endl;
    }
    cout << "soln min cost: " << trip_cost << endl;
    return true;
}


// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
}


class cartTrajActionServer {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in baxter_traj_streamer/action/traj.action
    // the type "trajAction" is auto-generated from our name "traj" and generic name "Action"
    actionlib::SimpleActionServer<cartesian_moves::cartTrajAction> as_;
    
    

    // here are some message types to communicate with our client(s)
    cartesian_moves::cartTrajGoal goal_; // goal message, received from client
    cartesian_moves::cartTrajResult result_; // put results here, to be sent back to the client when done w/ goal
    cartesian_moves::cartTrajFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    //Eigen::Vector3d p;
    Eigen::Vector3d n_des, t_des, b_des;
    
    Eigen::Affine3d a_tool_start_,a_tool_end_;
    Eigen::Affine3d a_flange_start_,a_flange_end_;    
    Eigen::Matrix3d R_des;
    std::vector<Eigen::VectorXd> optimal_path_;

public:
    cartTrajActionServer(); //define the body of the constructor outside of class definition

    ~cartTrajActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<cartesian_moves::cartTrajAction>::GoalConstPtr& goal);
};

cartTrajActionServer::cartTrajActionServer() :
as_(nh_, "cartTrajActionServer", boost::bind(&cartTrajActionServer::executeCB, this, _1), false)
// in the above initialization, we name the server "cartTrajActionServer"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of cartTrajActionServer...");
   // fixed orientation: tool flange points down, with x-axis forward
    b_des << 0, 0, -1;
    n_des << 1, 0, 0;
    t_des = b_des.cross(n_des);


    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;
    

    as_.start(); //start the server running
}


void cartTrajActionServer::executeCB(const actionlib::SimpleActionServer<cartesian_moves::cartTrajAction>::GoalConstPtr& goal) {
    Eigen::Affine3d a_flange_end;
    std::vector<Eigen::VectorXd> optimal_path;    
    a_flange_end.linear() = R_des;
    Eigen::Vector3d p_end;
    p_end[0] = goal->x;
    p_end[1] = goal->y;    
    p_end[2] = goal->z;
    cout<<"received flange goal pt: "<<p_end.transpose()<<endl;
    double speed = goal->speed;
    a_flange_end.translation() = p_end;
    
    cartesian_path_planner(a_flange_end, optimal_path);
    
    ROS_INFO("stuffing traj: " ); 
    trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 
    
    baxter_traj_streamer_ptr->stuff_trajectory(optimal_path, des_trajectory); 
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
    baxter_traj_streamer::trajGoal trajGoal; 
    trajGoal.trajectory = des_trajectory;    
    //action_client.sendGoal(trajGoal); // simple example--send goal, but do not specify callbacks
    ac_ptr_->sendGoal(trajGoal,&doneCb); // we could also name additional callback functions here, if desired
    //    action_client.sendGoal(trajGoal, &doneCb, &activeCb, &feedbackCb); //e.g., like this  
    bool finished_before_timeout = ac_ptr_->waitForResult(ros::Duration(15.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for traj_id %d",goal->traj_id);
            return;
        }
        else {
            ROS_INFO("finished before timeout");
        }   
    
    
    result_.traj_id = goal->traj_id;
    as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message 
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "baxter_cartesian_move");
    ros::NodeHandle nh; //standard ros node handle    
    
    ros::Subscriber joint_state_sub = nh.subscribe("robot/joint_states", 1, jointStatesCb); //subscribe to joint-state values
    // wait for valid data:
    q_vec_right_arm[0] = 1000;
    cout<<"waiting for valid joint_state message..."<<endl;
   while(q_vec_right_arm[0]>100) 
   {
        //ROS_INFO("waiting for valid state data...");
        cout<<q_vec_right_arm.transpose()<<endl;
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    cout<<"got current pose: "<<q_vec_right_arm.transpose()<<endl;   
    
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    baxter_traj_streamer_ptr =   &baxter_traj_streamer;  
    
    //     actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> action_client("trajActionServer", true);
    std::string action_name("/trajActionServer"); // name of the behavior server
    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(action_name, true); // instantiate the client, called "ac"
    // for "action client"
    actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> ac(action_name, true);
    ac_ptr_ = &ac; // global pointer, so callbacks can access the action client
    ROS_INFO_STREAM("Starting action_client of server: " << action_name);

    unsigned int attempts = 0;
    while (ros::ok() && !ac.waitForServer(ros::Duration(5.0)) && ++attempts < 3)
        ROS_INFO_STREAM("Waiting for Action Server to come up");

    if (!ac.isServerConnected()) {
        ROS_ERROR_STREAM("Action client not connected; giving up");
        return 0;
    }
    ROS_INFO("connected to joint-trajectory action server");
    
      ROS_INFO("instantiating the cartesian trajectory action server: ");
    cartTrajActionServer cartTraj_as; // create an instance of the class "cartTrajActionServer"  

    ROS_INFO("ready to receive/execute cartesian goals");  
   
    ros::spin(); // stop here;    

    return 0;
}

