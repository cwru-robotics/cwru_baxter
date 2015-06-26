// coordinator.cpp
// wsn, June 2015;
// uses service calls to communicate with:
// pcl_perception, reachability, cartesian_arm_interface, interactive_marker_node
// send requests and get info via service calls;
// also, provide a service that responds to simple integer command codes (for use w/ GUI)

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
//#include <Eigen/Eigen>  
#include <cwru_srv/IM_node_service_message.h>
#include <cwru_srv/simple_float_service_message.h>
#include <cwru_srv/simple_int_service_message.h>
#include <cwru_srv/arm_nav_service_message.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>

//#include <cwru_srv/>
//command codes for interactive marker: get/set pose
const int GET_CURRENT_MARKER_POSE = 0;
const int SET_NEW_MARKER_POSE = 1;



const double H_CYLINDER = 0.24; // estimated height of cylinder

using namespace std;

//incoming command codes to this service:
const int DO_NOTHING = 0;
const int DISPLAY_REACHABILITY_AT_MARKER_HEIGHT = 1;
const int COMPUTE_MOVE_ARM_TO_PRE_POSE = 2;
const int MOVE_ARM_TO_APPROACH_POSE = 3;
const int MOVE_ARM_TO_GRASP_POSE = 4;
const int MOVE_ARM_DEPART = 5;
const int MOVE_ARM_TO_MARKER = 6;
const int MOVE_MARKER_TO_GRASP_POSE = 7;
const int RQST_DO_PLANNED_PATH=8;
const int RQST_DESCEND_20CM=9;
const int RQST_ASCEND_20CM=10;

//service codes to send to arm interface:
const int TEST_MODE =0;
const int GO_TO_PREDFINED_PRE_POSE=1;
const int DESCEND_20CM=2;
const int DEPART_20CM=3;

const int IS_SERVER_BUSY_QUERY = 1;
const int PLAN_PATH_QSTART_TO_ADES = 4;
const int PLAN_PATH_QSTART_TO_QGOAL = 5;
const int PLAN_PATH_ASTART_TO_QGOAL = 6;
const int PLAN_PATH_CURRENT_TO_PRE_POSE=7;

const int GET_Q_DATA = 8;

const int EXECUTE_PLANNED_PATH = 9;


//responses...
const int STATUS_UNDEFINED=0;
const int RECEIVED_AND_INITIATED_RQST=1;
const int REQUEST_REJECTED_ALREADY_BUSY=2;
const int SERVER_NOT_BUSY=3;
const int SERVER_IS_BUSY=4;
const int RECEIVED_AND_COMPLETED_RQST=5;



bool g_trigger = false;
int g_coordinator_mode = DO_NOTHING;

tf::TransformListener *g_tfListener_ptr; //pointer to a global transform listener
   ros::ServiceClient g_pcl_getframe_svc_client; //global service to talk to pcl


//use this service to set processing modes interactively
// this is a very simple interface--expects an integer code in, and nothing gets returned;
// interact w/ dumb GUI

bool coordinatorService(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response) {
    ROS_INFO("coordinator service callback activated");
    response.resp = true; // boring, but valid response info
    g_coordinator_mode = request.req;
    g_trigger = true; //signal that we received a request; trigger a response
    cout << "Mode set to: " << g_coordinator_mode << endl;
    return true;
}

geometry_msgs::PoseStamped get_model_pose_wrt_torso() {
geometry_msgs::PoseStamped poseStamped;
    cwru_srv::IM_node_service_message pcl_getframe_msg;
                    ROS_INFO("requesting model pose from pcl_perception: ");
                    pcl_getframe_msg.request.cmd_mode = 1; // not really used yet...service always assumes you want the model frame, in kinect_pc_frame coords
                    bool status = g_pcl_getframe_svc_client.call(pcl_getframe_msg);
                    geometry_msgs::PoseStamped pose_from_pcl, pose_wrt_torso;
                    pose_from_pcl = pcl_getframe_msg.response.poseStamped_IM_current;
                    ROS_INFO("got current model pose: x,y,z = %f, %f, %f",
                            pose_from_pcl.pose.position.x,
                            pose_from_pcl.pose.position.y,
                            pose_from_pcl.pose.position.z);
                    ROS_INFO("quaternion is: %f, %f, %f, %f",
                            pose_from_pcl.pose.orientation.x,
                            pose_from_pcl.pose.orientation.y,
                            pose_from_pcl.pose.orientation.z,
                            pose_from_pcl.pose.orientation.w);

                    // need to transform this into torso coords:
                    /*
                    tf::Vector3 pos = tf_kinect_wrt_torso.getOrigin();
                    tf::Quaternion tf_quaternion = tf_kinect_wrt_torso.getRotation();

                    quaternion.x = tf_quaternion.x();
                    quaternion.y = tf_quaternion.y();
                    quaternion.z = tf_quaternion.z();
                    quaternion.w = tf_quaternion.w();
                    ROS_INFO("transform components: ");
                    ROS_INFO("x,y, z = %f, %f, %f", pos[0], pos[1], pos[2]);
                    ROS_INFO("q x,y,z,w: %f %f %f %f", quaternion.x, quaternion.y, quaternion.z, quaternion.w);
                     * */
                    g_tfListener_ptr->transformPose("torso", pose_from_pcl, pose_wrt_torso);
                    ROS_INFO("model pose w/rt torso: ");
                    ROS_INFO("origin: %f, %f, %f", pose_wrt_torso.pose.position.x, pose_wrt_torso.pose.position.y, pose_wrt_torso.pose.position.z);
                    ROS_INFO("orientation: %f, %f, %f, %f", pose_wrt_torso.pose.orientation.x, pose_wrt_torso.pose.orientation.y, pose_wrt_torso.pose.orientation.z, pose_wrt_torso.pose.orientation.w);
                    return pose_wrt_torso;
    
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "coordinator");
    ros::NodeHandle nh; //standard ros node handle   
    ros::Rate rate(2);
    bool status;
    cwru_srv::IM_node_service_message IM_6dof_srv_msg;
    cwru_srv::IM_node_service_message pcl_getframe_msg;
    cwru_srv::simple_float_service_message reachability_msg;
    cwru_srv::arm_nav_service_message arm_nav_msg;

    geometry_msgs::Pose pose;
    geometry_msgs::PoseStamped poseStamped;
    geometry_msgs::PoseStamped pose_from_pcl, model_pose_wrt_torso;

    tf::TransformListener tfListener;
    g_tfListener_ptr = &tfListener;
    
    tf::StampedTransform tf_kinect_wrt_torso;
    // wait to start receiving valid tf transforms between map and odom:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect and torso...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("kinect_pc_frame", "torso", ros::Time(0), tf_kinect_wrt_torso);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good");

    //communicate with node interactive_marker_node interactive_marker_node
    ROS_INFO("setting up a service client of rt_hand_marker");
    ros::ServiceClient IM_6dof_svc_client = nh.serviceClient<cwru_srv::IM_node_service_message>("IM6DofSvc");
    //ros::ServiceClient pcl_getframe_svc_client = nh.serviceClient<cwru_srv::IM_node_service_message>("pcl_getframe_svc");
    g_pcl_getframe_svc_client= nh.serviceClient<cwru_srv::IM_node_service_message>("pcl_getframe_svc");
    //talk to the reachability node:
    ros::ServiceClient reachability_svc_client = nh.serviceClient<cwru_srv::simple_float_service_message>("compute_reachability_svc");
    // talk to arm interface:
    ros::ServiceClient arm_interface_svc_client = nh.serviceClient<cwru_srv::arm_nav_service_message>("cartMoveSvc");
    
    
    
    
    ros::ServiceServer service = nh.advertiseService("coordinator_svc", coordinatorService);


    
    geometry_msgs::Quaternion quaternion;
    while (ros::ok()) {
        if (g_trigger) {
            g_trigger = false; // reset the trigger

            switch (g_coordinator_mode) { // what we do here depends on our mode; mode is settable via a service
                case DO_NOTHING:
                    ROS_INFO("case DO_NOTHING; doing nothing!");
                    break;
                case MOVE_MARKER_TO_GRASP_POSE:
                    ROS_INFO("case MOVE_MARKER_TO_GRASP_POSE");                    
                    model_pose_wrt_torso = get_model_pose_wrt_torso();

                    // put the marker at the top of the can:
                    pose = model_pose_wrt_torso.pose;
                    pose.position.z += H_CYLINDER;
                    poseStamped.pose = pose;
                    poseStamped.header.stamp = ros::Time::now();
                    IM_6dof_srv_msg.request.cmd_mode = SET_NEW_MARKER_POSE;
                    IM_6dof_srv_msg.request.poseStamped_IM_desired = poseStamped;
                    ROS_INFO("placing IM at top of model");
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);
                    /*
                    ROS_INFO("return status: %d", status);
                    ROS_INFO("got current marker pose: x,y,z = %f, %f, %f",
                            IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.x,
                            IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.y,
                            IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.z);*/
                    break;
                case DISPLAY_REACHABILITY_AT_MARKER_HEIGHT:
                    IM_6dof_srv_msg.request.cmd_mode = GET_CURRENT_MARKER_POSE;
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);
                    
                    ROS_INFO("case DISPLAY_REACHABILITY_AT_MARKER_HEIGHT");
                    // compute/display reachability at marker height:
                    reachability_msg.request.request_float32 = pose.position.z;
                    reachability_svc_client.call(reachability_msg);                    
                    
                    break;
                case COMPUTE_MOVE_ARM_TO_PRE_POSE:
                    ROS_INFO("case COMPUTE_MOVE_ARM_TO_PRE_POSE");
                    arm_nav_msg.request.cmd_mode= PLAN_PATH_CURRENT_TO_PRE_POSE;
                    arm_interface_svc_client.call(arm_nav_msg);
                    
                    break;
                case RQST_DO_PLANNED_PATH:
                                        ROS_INFO("case RQST_DO_PLANNED_PATH");
                    arm_nav_msg.request.cmd_mode= EXECUTE_PLANNED_PATH;     
                    arm_interface_svc_client.call(arm_nav_msg);
                    
                    break;
                case RQST_DESCEND_20CM:
                                        ROS_INFO("case RQST_DESCEND_20CM");
                    arm_nav_msg.request.cmd_mode= DESCEND_20CM;     
                    arm_interface_svc_client.call(arm_nav_msg);                    
                    break;
                 case RQST_ASCEND_20CM:
                                         ROS_INFO("case RQST_ASCEND_20CM");
                    arm_nav_msg.request.cmd_mode= DEPART_20CM;     
                    arm_interface_svc_client.call(arm_nav_msg);                    
                    break;
                    
                case MOVE_ARM_TO_APPROACH_POSE:
                    ROS_INFO("case MOVE_ARM_TO_APPROACH_POSE; doing nothing!");
                    break;
                case MOVE_ARM_TO_GRASP_POSE:
                    ROS_INFO("case MOVE_ARM_TO_GRASP_POSE; doing nothing!");
                    break;
                case MOVE_ARM_DEPART:
                    ROS_INFO("case MOVE_ARM_DEPART; doing nothing!");
                    break;
                case MOVE_ARM_TO_MARKER:
                    ROS_INFO("I should move arm to marker pose");
                    IM_6dof_srv_msg.request.cmd_mode = GET_CURRENT_MARKER_POSE;
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);
                    ROS_INFO("got current marker pose: x,y,z = %f, %f, %f",
                            IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.x,
                            IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.y,
                            IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.z);


                    break;
                    /**/

                default:
                    ROS_WARN("this mode is not implemented");

            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
