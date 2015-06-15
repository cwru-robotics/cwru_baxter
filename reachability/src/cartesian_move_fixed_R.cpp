// cartesian_move_fixed_R.cpp
// wsn, June 2015; 
// start of cartesion-move functions;
// command gripper to point down, plan path from p_start to p_end (flange origin w/rt torso frame)
// compute optimal path (if reachable)


#include <baxter_kinematics/baxter_kinematics.h>
//include the following if/when want to plan a joint-space path and execute it
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <joint_space_planner.h>

#include <string.h>
#include <stdio.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>      /* printf, scanf, puts, NULL */
//#include <stdlib.h>     /* srand, rand */
//#include <time.h>       /* time */
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

Eigen::Vector3d g_p_start;
Eigen::Vector3d g_p_end;

void reversePathFnc(std::vector<Eigen::VectorXd> fwd_path, std::vector<Eigen::VectorXd> &rvrs_path) {
    int npts = fwd_path.size();
    rvrs_path.clear();
    for (int i = npts - 1; i >= 0; i--) {
        rvrs_path.push_back(fwd_path[i]);
    }
}
    Baxter_IK_solver baxter_IK_solver; // instantiate an IK solver
    Baxter_fwd_solver baxter_fwd_solver; //instantiate a forward-kinematics solver
    
//specify start and end poses w/rt torso.  Only orientation of end pose will be considered; orientation of start pose is ignored
bool cartesian_path_planner(Eigen::Affine3d a_tool_start,Eigen::Affine3d a_tool_end, std::vector<Eigen::VectorXd> &optimal_path) {
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;
    Eigen::Affine3d a_tool_des;
    Eigen::Matrix3d R_des = a_tool_end.linear();
    a_tool_des.linear() = R_des;

    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;
    Eigen::Vector3d p_des,dp_vec,del_p;
    del_p = g_p_end-g_p_start;
    double dp_scaler = 0.05;
    nsteps = round(del_p.norm()/dp_scaler);
    if (nsteps<1) nsteps=1;
    dp_vec = del_p/nsteps;
    nsteps++; //account for pose at step 0


    std::vector<Vectorq7x1> q_solns;
    p_des = g_p_start;

    for (int istep=0;istep<nsteps;istep++) 
    {
            a_tool_des.translation() = p_des;
            cout<<"trying: "<<p_des.transpose()<<endl;
            nsolns = baxter_IK_solver.ik_solve_approx_wrt_torso(a_tool_des, q_solns);
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
    
int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_cartesian_move");
    ros::NodeHandle nh; //standard ros node handle    

    Eigen::Vector3d p;
    Eigen::Vector3d n_des, t_des, b_des;
    Vectorq7x1 q_in;
    q_in << 0, 0, 0, 0, 0, 0, 0;
    g_p_start<<0.4, -0.6, 0.4;
    g_p_end<<0.5, 0, 0.4;
    
    Eigen::Affine3d a_tool_start,a_tool_end;
    //Baxter_fwd_solver baxter_fwd_solver; //instantiate a forward-kinematics solver
    //Baxter_IK_solver baxter_IK_solver; // instantiate an IK solver
    ROS_INFO("main: instantiating an object of type Baxter_traj_streamer");
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    cwru_srv::simple_bool_service_message srv;
    ros::ServiceClient traj_interp_stat_client = nh.serviceClient<cwru_srv::simple_bool_service_message>("trajInterpStatusSvc");

    // fixed orientation: tool flange points down, with x-axis forward
    b_des << 0, 0, -1;
    n_des << 1, 0, 0;
    t_des = b_des.cross(n_des);

    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;

    //Eigen::Affine3d a_tool_des; // expressed in torso frame
    a_tool_start.linear() = R_des;
    a_tool_start.translation()= g_p_start;
    a_tool_end.linear() = R_des;
    a_tool_end.translation() = g_p_end;
    std::vector<Eigen::VectorXd> optimal_path;
    /*
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;

    a_tool_des.linear() = R_des;

    int nsolns;
    bool reachable_proposition;
    int nsteps = 0;
    Eigen::Vector3d p_des,dp_vec,del_p;
    del_p = g_p_end-g_p_start;
    double dp_scaler = 0.05;
    nsteps = round(del_p.norm()/dp_scaler);
    if (nsteps<1) nsteps=1;
    dp_vec = del_p/nsteps;
    nsteps++; //account for pose at step 0


    std::vector<Vectorq7x1> q_solns;
    p_des = g_p_start;

    for (int istep=0;istep<nsteps;istep++) 
    {
            a_tool_des.translation() = p_des;
            cout<<"trying: "<<p_des.transpose()<<endl;
            nsolns = baxter_IK_solver.ik_solve_approx_wrt_torso(a_tool_des, q_solns);
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
        return 0; // give up if no options
    }
    std::vector<Eigen::VectorXd> optimal_path, rvrs_path;
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
    //return 0;
*/
    cartesian_path_planner(a_tool_start,a_tool_end, optimal_path); // try the fnc

    //convert above path into a trajectory:
    trajectory_msgs::JointTrajectory new_trajectory; // an empty trajectory
    cout << "stuffing traj from path: " << endl;
    baxter_traj_streamer.stuff_trajectory(optimal_path, new_trajectory); //convert from vector of 7dof poses to trajectory message

    int ans;
    cout<<"enter 1 to execute trajectory; 0 to quit: ";
    cin>>ans;
    if (ans!=1) return 0;
    //prepare to execute the above trajectory
    //make sure robot is listening; sends first command equal to current joint angles, repeating for 10 iterations over 1 second
    ROS_INFO("warming up the command listener...");
    for (int i = 0; i < 10; i++) {
        baxter_traj_streamer.pub_right_arm_trajectory_init();
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }


    // send trajectory:
    bool working_on_traj = true;
    while (working_on_traj) { // wait for interpolator to declare it is ready
            traj_interp_stat_client.call(srv); // communicate w/ trajectory interpolator node status service
            working_on_traj = srv.response.resp;
            cout << "working_on_traj = " <<working_on_traj<< endl;
            ros::spinOnce();
        }    
    while (ros::ok()) { //while ROS is okay...
        cout << "sending traj..." << endl;
        baxter_traj_streamer.pub_right_arm_trajectory(new_trajectory); //add timing and publish the traj message
        while (!working_on_traj) { //wait for a confirmation that trajectory has been received
            traj_interp_stat_client.call(srv); // communicate w/ trajectory interpolator node status service
            working_on_traj = srv.response.resp;
            cout << "no ack of new traj yet..." << endl;
            ros::spinOnce();
            ros::Duration(0.1).sleep(); //Wait dt
        }
        cout << "new traj acknowledged; quitting" << endl;
        //done sending this one trajectory
        return 0;
    }

    return 0;
}
