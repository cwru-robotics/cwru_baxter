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
    arm_nav_srv_msg.request.cmd_mode = ARM_TEST_MODE;
    //populate the service message with value for q_vec_start
    arm_nav_srv_msg.request.q_vec_start.clear();
    for (int i=0;i<7;i++) {
        arm_nav_srv_msg.request.q_vec_start.push_back(q_in[i]);
    }
    arm_nav_svc_client.call(arm_nav_srv_msg);
    
    
    // request plan from current pose to pre-pose: this will set start and end j-space poses
    arm_nav_srv_msg.request.cmd_mode = ARM_PLAN_PATH_CURRENT_TO_PRE_POSE;
    cout<<"sending request PLAN_PATH_CURRENT_TO_PRE_POSE"<<endl;
    arm_nav_svc_client.call(arm_nav_srv_msg);   
    int status = ARM_SERVER_IS_BUSY;
    cout<<"waiting for not busy..."<<endl;
    while (status != ARM_SERVER_NOT_BUSY) {
        arm_nav_srv_msg.request.cmd_mode = ARM_IS_SERVER_BUSY_QUERY;
        arm_nav_svc_client.call(arm_nav_srv_msg); 
        status = arm_nav_srv_msg.response.rtn_code;
        cout<<"status code: "<<status<<endl;
    }
    
    
    //try a query: are internal values of q_start and q_end used by motion interface?
    cout<<"sending q-data inquiry: "<<endl;
    arm_nav_srv_msg.request.cmd_mode = ARM_GET_Q_DATA;
    arm_nav_svc_client.call(arm_nav_srv_msg);
    //interpret the response data:
    cout<<"response.rtn_code: "<<arm_nav_srv_msg.response.rtn_code<<endl;
    int nsize; 
    nsize = arm_nav_srv_msg.response.q_vec_start.size();
    if (nsize>0) {
           cout<<"response.q_start:";
        for (int i=0;i<nsize;i++) cout<<" "<<arm_nav_srv_msg.response.q_vec_start[i]<<",";
        cout<<endl;
    }
    nsize = arm_nav_srv_msg.response.q_vec_end.size();
    if (nsize>0) {
           cout<<"response.q_end:";
        for (int i=0;i<nsize;i++) cout<<" "<<arm_nav_srv_msg.response.q_vec_start[i]<<",";
        cout<<endl;   
    }
    
        // populate a service request message: plan a relative cartesian move
    arm_nav_srv_msg.request.cmd_mode = ARM_DESCEND_20CM;
    //populate the service message with value for q_vec_start
    arm_nav_srv_msg.request.q_vec_start.clear();
    arm_nav_svc_client.call(arm_nav_srv_msg);
    
    // when ready, tell the interface to execute the path:
     arm_nav_srv_msg.request.cmd_mode = ARM_DESCEND_20CM;

   status = ARM_SERVER_IS_BUSY;
    cout<<"waiting for not busy..."<<endl;
    while (status != ARM_SERVER_NOT_BUSY) {
        arm_nav_srv_msg.request.cmd_mode = ARM_IS_SERVER_BUSY_QUERY;
        arm_nav_svc_client.call(arm_nav_srv_msg); 
        status = arm_nav_srv_msg.response.rtn_code;
        cout<<"status code: "<<status<<endl;
    }     
    arm_nav_srv_msg.request.cmd_mode = ARM_EXECUTE_PLANNED_PATH;
    arm_nav_svc_client.call(arm_nav_srv_msg);
   status = ARM_SERVER_IS_BUSY;
    cout<<"waiting for not busy..."<<endl;
    while (status != ARM_SERVER_NOT_BUSY) {
        arm_nav_srv_msg.request.cmd_mode = ARM_IS_SERVER_BUSY_QUERY;
        arm_nav_svc_client.call(arm_nav_srv_msg); 
        status = arm_nav_srv_msg.response.rtn_code;
        cout<<"status code: "<<status<<endl;
    }     
    

    return 0;
}
