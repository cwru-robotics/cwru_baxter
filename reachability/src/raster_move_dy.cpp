// raster_move_dy.cpp
// wsn, June 2015; use for test of Baxter reachability
// command gripper to point down, fixed value of x (w/rt torso) and sweep arm along y
// use range of values predicted reachable, via reachability_from_above: 

//e.g., successful test w/:
//     point hand down, hand x-axis forward, and sweep the tool-flange origin as:
//z_des = 0.0
//x_des = 0.4;
//y_min = -0.9;
//y_max = 0.3;
// above looks good...maybe could go slightly closer to body, but not much
// seems to do a good job of passing through wrist-bend singularity


#include <baxter_kinematics/baxter_kinematics.h>
//include the following if/when want to plan a joint-space path and execute it
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <joint_space_planner.h>

#include <string.h>
#include <stdio.h>      /* printf, scanf, puts, NULL */
//#include <stdlib.h>     /* srand, rand */
//#include <time.h>       /* time */
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

void reversePathFnc(std::vector<Eigen::VectorXd> fwd_path, std::vector<Eigen::VectorXd> &rvrs_path) {
    int npts = fwd_path.size();
    rvrs_path.clear();
    for (int i = npts - 1; i >= 0; i--) {
        rvrs_path.push_back(fwd_path[i]);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_raster_dy");
    ros::NodeHandle nh; //standard ros node handle    

    Eigen::Vector3d p;
    Eigen::Vector3d n_des, t_des, b_des;
    Vectorq7x1 q_in;
    q_in << 0, 0, 0, 0, 0, 0, 0;

    Baxter_fwd_solver baxter_fwd_solver; //instantiate a forward-kinematics solver
    Baxter_IK_solver baxter_IK_solver; // instantiate an IK solver
    ROS_INFO("main: instantiating an object of type Baxter_traj_streamer");
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    cwru_srv::simple_bool_service_message srv;
    ros::ServiceClient traj_interp_stat_client = nh.serviceClient<cwru_srv::simple_bool_service_message>("trajInterpStatusSvc");


    b_des << 0, 0, -1;
    n_des << 1, 0, 0;
    t_des = b_des.cross(n_des);

    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;

    Eigen::Affine3d a_tool_des; // expressed in torso frame


    std::vector<std::vector<Eigen::VectorXd> > path_options;
    path_options.clear();
    std::vector<Eigen::VectorXd> single_layer_nodes;
    Eigen::VectorXd node;

    a_tool_des.linear() = R_des;
    //a_tool_des.translation() << x_des,0,0;
    double z_des = 0.0;

    p[2] = z_des;

    int nsolns;
    bool reachable_proposition;
    int nx = 0;
    int ny = 0;
    //set search range and resolution: only vary y value   
    double x_min = 0.4;
    double x_max = 0.4;
    double y_min = -0.9;
    double y_max = 0.3;
    double dx_des = 0.1;
    double dy_des = 0.1;

    //find how many values nx,ny in search range
    //for (double x_des = x_min; x_des<x_max; x_des+=dx_des) nx++;
    nx = 1;
    for (double y_des = y_min; y_des < y_max; y_des += dy_des) ny++;
    cout << "discretized search into nx = " << nx << ", ny= " << ny << endl;
    int nsolns_matrix[nx][ny];

    std::vector<Vectorq7x1> q_solns;
    int ix = 0;
    int iy = 0;

    //for (double x_des = x_min; x_des<x_max; x_des+=dx_des) 

    double x_des = x_min;
    {
        std::cout << std::endl;
        std::cout << "x=" << round(x_des * 10) << "  ";

        for (double y_des = y_min; y_des < y_max; y_des += dy_des) {
            p[0] = x_des;
            p[1] = y_des;
            a_tool_des.translation() = p;
            nsolns = baxter_IK_solver.ik_solve_approx_wrt_torso(a_tool_des, q_solns);
            //std::cout<<nsolns;
            nsolns_matrix[ix][iy] = nsolns;
            iy++;
            single_layer_nodes.clear();
            single_layer_nodes.resize(nsolns);
            for (int isoln = 0; isoln < nsolns; isoln++) {
                // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                node = q_solns[isoln];
                single_layer_nodes[isoln] = node;
            }

            path_options.push_back(single_layer_nodes);
            cout << "y = " << y_des << "; pushed " << nsolns << " solutions onto path_options" << endl;
        }
        ix++;
        iy = 0;
    }
    //display the matrix:
    cout << endl;
    cout << "x/y: ";
    int xval, yval;
    ix = 0;
    for (int iy = ny - 1; iy >= 0; iy--) {
        yval = round((y_min + iy * dy_des)*100);
        if (abs(yval) < 100) cout << " ";
        if (abs(yval) < 10) cout << " ";
        if (yval>-1) cout << " ";
        cout << yval;
    }
    cout << endl;

    //for (int ix=nx-1;ix>=0;ix--) 
    {
        // print out the x value as a column...in cm
        //xval = round((x_min+ix*dx_des)*100);
        xval = round(x_min * 100);
        if (abs(xval) < 100) cout << " ";
        if (abs(xval) < 10) cout << " ";
        if (xval>-1) cout << " ";
        cout << xval << "  ";
        for (iy = ny - 1; iy >= 0; iy--) {
            nsolns = nsolns_matrix[ix][iy];
            if (nsolns < 10)
                cout << " ";
            if (nsolns < 100)
                cout << " ";
            cout << nsolns << " ";
        }
        cout << endl;
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

    cout << "defining a reverse path" << endl;
    reversePathFnc(optimal_path, rvrs_path);
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        cout << "ilayer: " << ilayer << " node: " << rvrs_path[ilayer].transpose() << endl;
    }


    //convert above forward and reverse paths into trajectories:
    trajectory_msgs::JointTrajectory new_trajectory, rvrs_trajectory; // an empty trajectory
    cout << "stuffing traj for forward path: " << endl;
    baxter_traj_streamer.stuff_trajectory(optimal_path, new_trajectory); //convert from vector of 7dof poses to trajectory message
    cout << "stuffing traj for reverse path: " << endl;
    baxter_traj_streamer.stuff_trajectory(rvrs_path, rvrs_trajectory); //convert from vector of 7dof poses to trajectory message


    //prepare to execute the above trajectories
    //make sure robot is listening; sends first command equal to current joint angles, repeating for 10 iterations over 1 second
    ROS_INFO("warming up the command listener...");
    for (int i = 0; i < 10; i++) {
        baxter_traj_streamer.pub_right_arm_trajectory_init();
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }


    // go into a loop to send trajectories, forward and reverse:
    cout << "starting loop for planned arm trajectories..."<<endl;
            bool working_on_traj = false;
    while (ros::ok()) { //while ROS is okay...
        cout << "sending fwd traj..." << endl;
        baxter_traj_streamer.pub_right_arm_trajectory(new_trajectory); //add timing and publish the traj message
        while (!working_on_traj) {
            traj_interp_stat_client.call(srv); // communicate w/ trajectory interpolator node status service
            working_on_traj = srv.response.resp;
            cout << "no ack of new traj yet..." << endl;
            ros::spinOnce();
        }
        cout << "new traj acknowledged" << endl;
        for (int i = 0; i < 200; i++) { // this will time out after 20 sec
            ros::Duration(0.1).sleep(); //Wait dt
            traj_interp_stat_client.call(srv); // communicate w/ trajectory interpolator node status service
            working_on_traj = srv.response.resp;
            ros::spinOnce();
            if (!working_on_traj) {
                cout << "svc says done w/ traj..." << endl;
                break;
            }
        }
        //now go back again...

        cout << "sending reverse traj: " << endl;
        baxter_traj_streamer.pub_right_arm_trajectory(rvrs_trajectory); //add timing and publish the traj message
        working_on_traj = false;
        while (!working_on_traj) { // wait for ack
            traj_interp_stat_client.call(srv); // communicate w/ trajectory interpolator node status service
            working_on_traj = srv.response.resp;
            cout << "no ack of new traj yet..." << endl;
            ros::spinOnce();
        }
        cout << "new traj acknowledged" << endl;
        //working_on_traj=true;
        for (int i = 0; i < 200; i++) { // this will time out after 20 sec
            ros::Duration(0.1).sleep(); //Wait dt
            traj_interp_stat_client.call(srv); // communicate w/ trajectory interpolator node status service
            working_on_traj = srv.response.resp;
            ros::spinOnce();
            if (!working_on_traj) {
                cout << "svc says done w/ traj..." << endl;
                break;
            }
        }

    }

    return 0;
}
