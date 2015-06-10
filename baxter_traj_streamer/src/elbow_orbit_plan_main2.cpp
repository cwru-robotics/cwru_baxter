// wsn, June, 2015
// test main for joint-space planner, organized as a class
// compute a path consisting of elbow orbit, indexed by qs0
// somewhat unusual--expect only 1 to 4 soln options per "layer"
#include <joint_space_planner.h>
#include <baxter_kinematics/baxter_kinematics.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
//#include <baxter_kinematics.h>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


#define VECTOR_DIM 7 // e.g., a 7-dof vector
#define NLAYERS 100  // e.g., the number of points on a path, for which to generate IK solutions
#define MAX_OPTIONS_PER_LAYER 4000 //this can get LARGE; num IK solutions for a given task pose
#define MAX_VEC_COMPONENT 10.0 // generate random vecs with elements between +/-10
#define MIN_VEC_COMPONENT -10.0

void reversePathFnc(std::vector<Eigen::VectorXd> fwd_path, std::vector<Eigen::VectorXd> &rvrs_path) {
    int npts = fwd_path.size();
    rvrs_path.clear();
    for (int i = npts - 1; i >= 0; i--) {
        rvrs_path.push_back(fwd_path[i]);
    }
}


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
    Eigen::VectorXd weights;
    //Eigen::VectorXd vec2;
    //Eigen::VectorXd vec3; 
    //Eigen::VectorXd arg1,arg2;   
    ros::init(argc, argv, "elbow_orbit_plan_main"); //standard ros init with node name string
    ros::NodeHandle nh; //standard ros node handle    
    Baxter_fwd_solver baxter_fwd_solver; //instantiate a forward-kinematics solver
    Baxter_IK_solver baxter_IK_solver; // instantiate an IK solver

    //pass in nh, so constructor can set up subscribers, publishers and services
    Baxter_traj_streamer baxter_traj_streamer(&nh);
    cwru_srv::simple_bool_service_message srv;

    ros::ServiceClient traj_interp_stat_client = nh.serviceClient<cwru_srv::simple_bool_service_message>("trajInterpStatusSvc");


    int nlayers;
    std::vector<std::vector<Eigen::VectorXd> > path_options;
    // create a viable hand pose from q_test:
    Vectorq7x1 q_test;
    std::vector<Eigen::VectorXd> layer;
    Eigen::VectorXd node;
    q_test << 0, 0.5, 0.5, 1, 1, 1, 1; //some initial pose to define viable hand frame from which to do elbow orbit

    ros::Subscriber joint_state_sub = nh.subscribe("robot/joint_states", 1, jointStatesCb); //subscribe to joint-state values
    // wait for valid data:
    q_vec_right_arm[0] = 1000;
    while (true) //q_vec_right_arm[0]>100) 
   {
        //ROS_INFO("waiting for valid state data...");
        cout<<q_vec_right_arm.transpose()<<endl;
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    q_test = q_vec_right_arm; //use this pose as start of plan for elbow orbit
    cout<<"plan from current pose: "<<q_test.transpose()<<endl;


    Eigen::Affine3d desired_hand_pose_wrt_torso = baxter_fwd_solver.fwd_kin_solve_wrt_torso(q_test);
    nlayers = baxter_IK_solver.ik_solve_approx_elbow_orbit_from_flange_pose_wrt_torso(desired_hand_pose_wrt_torso, path_options);
    nlayers = path_options.size();
    cout << "nlayers: " << nlayers << endl;
    cout << "path options:" << endl;
    for (int ilayer = 0; ilayer < nlayers; ilayer++) {
        layer = path_options[ilayer];
        cout << "layer " << ilayer << endl;
        for (int inode = 0; inode < layer.size(); inode++) {
            node = layer[inode];
            cout << node.transpose() << endl;
        }
    }


    if (nlayers < 1) return 0; // give up if no options
    std::vector<Eigen::VectorXd> optimal_path, rvrs_path;
    optimal_path.resize(nlayers);
    double trip_cost;
    weights.resize(VECTOR_DIM);
    for (int i = 0; i < VECTOR_DIM; i++) {
        weights(i) = 1.0;
    }



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



    trajectory_msgs::JointTrajectory new_trajectory, rvrs_trajectory; // an empty trajectory
    cout<<"stuffing traj for forward path: "<<endl;
    baxter_traj_streamer.stuff_trajectory(optimal_path, new_trajectory); //convert from vector of 7dof poses to trajectory message
    cout<<"stuffing traj for reverse path: "<<endl;
    baxter_traj_streamer.stuff_trajectory(rvrs_path, rvrs_trajectory); //convert from vector of 7dof poses to trajectory message

/*
    int n_traj_pts = new_trajectory.points.size();
    cout<<"n_traj_pts = "<<n_traj_pts<<endl;
    Vectorq7x1 q_vec_test;
    trajectory_msgs::JointTrajectoryPoint traj_pt;
    for (int i=0;i<n_traj_pts;i++) {
        traj_pt = new_trajectory.points[i];
        cout<<"pt "<<i<<" time = "<<traj_pt.time_from_start.toSec()<<endl;
        for (int ijnt=0;ijnt<7;ijnt++) {
            cout<<"qcmd jnt "<<ijnt<<" = "<< traj_pt.positions[ijnt]<<endl;
            //q_vec_test[ijnt] = traj_pt.positions[ijnt];
        }
     cout<<"traj pt: "<<q_vec_test.transpose()<<endl;
}
    ros::Duration(2).sleep();
   return 0;
*/
    ROS_INFO("warming up the command listener...");
    for (int i = 0; i < 10; i++) {
        baxter_traj_streamer.pub_right_arm_trajectory_init();
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }


    bool working_on_traj = false;
    while (ros::ok()) { //while ROS is okay...
        cout << "sending fwd traj..." << endl;
        baxter_traj_streamer.pub_right_arm_trajectory(new_trajectory); //add timing and publish the traj message
        while(!working_on_traj) {
           traj_interp_stat_client.call(srv); // communicate w/ trajectory interpolator node status service
            working_on_traj = srv.response.resp;
            cout<<"no ack of new traj yet..."<<endl;
            ros::spinOnce();
        }
        cout<<"new traj acknowledged"<<endl;
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
        working_on_traj= false;
        while(!working_on_traj) { // wait for ack
           traj_interp_stat_client.call(srv); // communicate w/ trajectory interpolator node status service
            working_on_traj = srv.response.resp;
            cout<<"no ack of new traj yet..."<<endl;
            ros::spinOnce();
        }
         cout<<"new traj acknowledged"<<endl;
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

}
