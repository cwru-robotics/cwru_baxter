// cart_path_planner_lib header
// wsn, June, 2015
// a library of arm-motion planning functions

#include<ros/ros.h>

//#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <baxter_kinematics/baxter_kinematics.h>
//include the following if/when want to plan a joint-space path and execute it
#include <joint_space_planner.h>
#include <Eigen/Eigen>
//#include <baxter_core_msgs/JointCommand.h>
//#include <trajectory_msgs/JointTrajectory.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>
//#include <sensor_msgs/JointState.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <iostream>

using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

class CartTrajPlanner {
private:

    //ros::NodeHandle nh_; // if need a node handle, get one upon instantiation
    

    Eigen::Vector3d n_des_, t_des_, b_des_;
    
    Eigen::Affine3d a_tool_start_,a_tool_end_;
    Eigen::Matrix3d R_gripper_down_;
    std::vector<Eigen::VectorXd> optimal_path_;
    
    Baxter_IK_solver baxter_IK_solver_; // instantiate an IK solver
    Baxter_fwd_solver baxter_fwd_solver_; //instantiate a forward-kinematics solver    

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

