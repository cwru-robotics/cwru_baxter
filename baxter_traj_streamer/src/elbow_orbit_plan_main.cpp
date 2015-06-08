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

void reversePathFnc(std::vector<Eigen::VectorXd> fwd_path,std::vector<Eigen::VectorXd> &rvrs_path) {
    int npts = fwd_path.size();
    rvrs_path.clear();
    for (int i= npts-1; i>=0;i--) {
        rvrs_path.push_back(fwd_path[i]);
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
     Baxter_IK_solver baxter_IK_solver;  // instantiate an IK solver

     //pass in nh, so constructor can set up subscribers, publishers and services
     Baxter_traj_streamer baxter_traj_streamer(&nh); 
     
     int nlayers;
        std::vector<std::vector<Eigen::VectorXd> > path_options;  
     // create a viable hand pose from q_test:
     Vectorq7x1 q_test;
     std::vector<Eigen::VectorXd> layer;
     Eigen::VectorXd node;
     q_test<<0,0.5,0.5,1,1,1,1;  //some initial pose to define viable hand frame from which to do elbow orbit
     
     Eigen::Affine3d desired_hand_pose_wrt_torso = baxter_fwd_solver.fwd_kin_solve_wrt_torso(q_test);
     nlayers =  baxter_IK_solver.ik_solve_approx_elbow_orbit_from_flange_pose_wrt_torso(desired_hand_pose_wrt_torso,path_options);
     nlayers = path_options.size();
     cout<<"nlayers: " <<nlayers<<endl;     
     cout<<"path options:"<<endl;
     for (int ilayer =0;ilayer<nlayers;ilayer++) {
            layer = path_options[ilayer];
            cout<<"layer "<<ilayer<<endl;
            for (int inode=0;inode<layer.size();inode++) {
                node = layer[inode];
                cout<<node.transpose()<<endl;
            }
     }
         
     
     if (nlayers<1) return 0; // give up if no options
     std::vector<Eigen::VectorXd> optimal_path,rvrs_path;
    optimal_path.resize(nlayers);    
        double trip_cost;
    weights.resize(VECTOR_DIM);
    for (int i=0;i<VECTOR_DIM;i++) {
        weights(i) = 1.0;
    }
    

    
     cout<<"instantiating a JointSpacePlanner:"<<endl;
     { //limit the scope of jsp here:
       JointSpacePlanner jsp (path_options,weights);
       cout<<"recovering the solution..."<<endl;
       jsp.get_soln(optimal_path);
       trip_cost= jsp.get_trip_cost();

     }

     //now, jsp is deleted, but optimal_path lives on:
     cout<<"resulting solution path: "<<endl;
     for (int ilayer=0;ilayer<nlayers;ilayer++) {
         cout<<"ilayer: "<<ilayer<<" node: "<<optimal_path[ilayer].transpose()<<endl;
     }
    cout<<"soln min cost: "<<trip_cost<<endl;
    //return 0;
      
    cout<<"defining a reverse path"<<endl;
    reversePathFnc(optimal_path,rvrs_path);
    ROS_INFO("warming up the command listener...");
  for (int i=0;i<10;i++) {
    baxter_traj_streamer.pub_right_arm_trajectory_init();  
    ros::spinOnce();
    ros::Duration(0.1).sleep();     
  }
  trajectory_msgs::JointTrajectory new_trajectory,rvrs_trajectory; // an empty trajectory
  baxter_traj_streamer.stuff_trajectory(optimal_path, new_trajectory); //convert from vector of 7dof poses to trajectory message
  baxter_traj_streamer.stuff_trajectory(rvrs_path, rvrs_trajectory); //convert from vector of 7dof poses to trajectory message

  while(ros::ok()){ //while ROS is okay...
  baxter_traj_streamer.pub_right_arm_trajectory(new_trajectory); //add timing and publish the traj message
  for (int i=0;i<100;i++) {
      ros::Duration(0.07).sleep(); //Wait dt
      ros::spinOnce();
  }
  //now go back again...
  baxter_traj_streamer.pub_right_arm_trajectory(rvrs_trajectory); //add timing and publish the traj message
  for (int i=0;i<100;i++) {
      ros::Duration(0.07).sleep(); //Wait dt
      ros::spinOnce();
  }  
      
  } 

}
