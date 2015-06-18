// cartesian_move_svc_client_test.cpp
// wsn, June 2015; use to test arm_motion_interface
// send requests via service calls;
// likely extend this to actionserver/actionclient
#include<ros/ros.h>
#include<std_msgs/Float32.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Bool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <cartesian_moves/arm_motion_interface_defs.h>     
#include <cwru_srv/arm_nav_service_message.h>

typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "cart_move_svc_client_test");
    ros::NodeHandle nh; //standard ros node handle    

    Eigen::Vector3d p;
    Eigen::Vector3d n_des, t_des, b_des;
    Vectorq7x1 q_in;
    cwru_srv::arm_nav_service_message  arm_nav_srv_msg;

    q_in << 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6; // test values

    ROS_INFO("setting up a service client of cartMoveSvc");
    ros::ServiceClient arm_nav_svc_client = nh.serviceClient<cwru_srv::arm_nav_service_message>("cartMoveSvc");


    b_des << 0, 0, -1;
    n_des << 1, 0, 0;
    t_des = b_des.cross(n_des);

    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;

    Eigen::Affine3d a_tool_des; // expressed in torso frame

    a_tool_des.linear() = R_des;
    p<<0.4,0,0.5; // example, hard-coded goal
    a_tool_des.translation() = p;

    // populate a service request message:
    arm_nav_srv_msg.request.cmd_mode = TEST_MODE;
    arm_nav_svc_client.call(arm_nav_srv_msg);
    cout<<"response.rtn_code: "<<arm_nav_srv_msg.response.rtn_code<<endl;
    

    return 0;
}
