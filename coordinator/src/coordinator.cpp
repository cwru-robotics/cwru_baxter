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
#include <std_msgs/Float64.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cwru_srv/IM_node_service_message.h>
#include <cwru_srv/simple_float_service_message.h>
#include <cwru_srv/simple_int_service_message.h>
#include <cwru_srv/arm_nav_service_message.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>

#include "object_grasp_frame_computer.cpp"  // shortcut for now--move this to a library in future

const double H_CYLINDER = 0.12; // estimated height of cylinder; reconcile this w/ perception module const

using namespace std;

//#include <cwru_srv/>
//command codes for interactive marker: get/set pose
cwru_srv::IM_node_service_message g_pcl_getframe_msg;
    cwru_srv::IM_node_service_message IM_6dof_srv_msg;
const int IM_GET_CURRENT_MARKER_POSE=0;
const int IM_SET_NEW_MARKER_POSE= 1;

//command codes for reachability node--only 1 action at present, so only need to provide z-height value
cwru_srv::simple_float_service_message reachability_msg;

cwru_srv::simple_int_service_message g_pcl_perception_msg;


//incoming command codes for this coordinator service:
const int RQST_DO_NOTHING = 0;
const int RQST_DISPLAY_REACHABILITY_AT_MARKER_HEIGHT = 1;
const int RQST_COMPUTE_MOVE_ARM_TO_PRE_POSE = 2;
const int RQST_COMPUTE_MOVE_ARM_TO_APPROACH_POSE = 3;
const int RQST_COMPUTE_MOVE_ARM_TO_GRASP_POSE = 4;
const int RQST_COMPUTE_MOVE_ARM_DEPART = 5;
const int RQST_COMPUTE_MOVE_ARM_TO_MARKER = 6;
const int RQST_MOVE_MARKER_TO_GRASP_POSE = 7;
const int RQST_EXECUTE_PLANNED_PATH=8;
const int RQST_DESCEND_20CM=9;
const int RQST_ASCEND_20CM=10;

const int RQST_COMPUTE_MOVE_ARM_JSPACE_CURRENT_TO_PRE_POSE = 11;
const int RQST_PREVIEW_TRAJECTORY=12;

const int RQST_GRAB_COKE_CAN_FROM_ABOVE=13;
const int RQST_GRAB_GAZEBO_BEER_CAN_FROM_ABOVE=14;
const int RQST_MOVE_MARKER_TO_MODEL_FRAME = 15;
const int RQST_MOVE_MARKER_TO_HAND_POSE = 16;

//service codes to send to arm interface: these are in cartesian_moves/arm_motion_interface_defs.h
cwru_srv::arm_nav_service_message arm_nav_msg;
const int ARM_TEST_MODE =0;
//queries: 
const int ARM_IS_SERVER_BUSY_QUERY = 1;
const int ARM_QUERY_IS_PATH_VALID = 2;
const int ARM_GET_Q_DATA = 3;




//requests for motion plans:
const int ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE=20; //plan paths from current arm pose
const int ARM_PLAN_PATH_CURRENT_TO_PRE_POSE=21;


const int ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE=22;
const int ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL=23;

const int ARM_PLAN_PATH_QSTART_TO_QGOAL = 25;
const int ARM_PLAN_PATH_QSTART_TO_ADES = 24; //specify start and end, j-space or hand pose

const int ARM_PLAN_PATH_ASTART_TO_QGOAL = 26;

// request to preview plan:
const int ARM_DISPLAY_TRAJECTORY = 50;

//MOVE command!
const int ARM_EXECUTE_PLANNED_PATH = 100;
const int ARM_DESCEND_20CM=101;
const int ARM_DEPART_20CM=102;

//response codes...
const int ARM_STATUS_UNDEFINED=0;
const int ARM_RECEIVED_AND_INITIATED_RQST=1;
const int ARM_REQUEST_REJECTED_ALREADY_BUSY=2;
const int ARM_SERVER_NOT_BUSY=3;
const int ARM_SERVER_IS_BUSY=4;
const int ARM_RECEIVED_AND_COMPLETED_RQST=5;
const int ARM_PATH_IS_VALID=6;
const int ARM_PATH_NOT_VALID=7;

const int PCL_IDENTIFY_PLANE = 0;
const int PCL_FIND_PNTS_ABOVE_PLANE = 1;
const int PCL_COMPUTE_CYLINDRICAL_FIT_ERR_INIT = 2;
const int PCL_COMPUTE_CYLINDRICAL_FIT_ERR_ITERATE = 3;
const int PCL_MAKE_CAN_CLOUD = 4;
const int PCL_FIND_ON_TABLE = 5;
const int PCL_TAKE_SNAPSHOT = 6;
const int PCL_COMPUTE_Z_BED_OF_NAILS = 7;
const int PCL_FIND_COKE_FRAME = 8;
const int PCL_FIND_GAZEBO_BEER_FRAME = 9;
const int PCL_SORT_POINTS_BY_HORIZ_SLABS = 10;
const int PCL_FIND_FLOOR = 11;
const int PCL_FIND_TABLE = 12;
const int PCL_INQUIRE_STATUS = 13;
const int PCL_FIND_GAZEBO_BEER_GRASP_FROM_ABOVE_FRAME = 14;
const int PCL_FIND_COKE_CAN_GRASP_FROM_ABOVE_FRAME = 15;

const int PCL_STATUS_BUSY = 1;
const int PCL_STATUS_IDLE = 0;

const int PCL_FRAME_SVC_GET_MODEL_FRAME_WRT_KINECT = 1;
const int PCL_FRAME_SVC_GET_MODEL_FRAME_WRT_TORSO = 2;
const int PCL_FRAME_SVC_GET_GRASP_FRAME_WRT_MODEL = 3;
const int PCL_FRAME_SVC_GET_GRASP_FRAME_WRT_TORSO = 4;


bool g_trigger = false;
int g_coordinator_mode = RQST_DO_NOTHING;

tf::TransformListener *g_tfListener_ptr; //pointer to a global transform listener
ros::ServiceClient g_pcl_getframe_svc_client; //global service to talk to pcl
ros::ServiceClient g_pcl_perception_svc_client; //2nd PCL service
ros::ServiceClient g_arm_interface_svc_client;

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


//convert a tf to an affine transform;
// note that this loses the header, including frame_id and parent frame
Eigen::Affine3f affine3f_from_poseStamped(geometry_msgs::PoseStamped poseStamped) {
    Eigen::Affine3f affine_result;
    Eigen::Vector3f origin_vec;
    origin_vec[0] = poseStamped.pose.position.x;
    origin_vec[1] = poseStamped.pose.position.y;
    origin_vec[2] = poseStamped.pose.position.z;
    affine_result.translation() = origin_vec;

    Eigen::Quaternionf quat;
    quat.x() = poseStamped.pose.orientation.x;
    quat.y() = poseStamped.pose.orientation.y;
    quat.z() = poseStamped.pose.orientation.z;
    quat.w() = poseStamped.pose.orientation.w;
    Eigen::Matrix3f R(quat);
    affine_result.linear() = R;
    return affine_result;
}

 geometry_msgs::Pose transformEigenAffine3fToPose(Eigen::Affine3f e) {
    Eigen::Vector3f Oe;
    Eigen::Matrix3f Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaternionf q(Re); // convert rotation matrix Re to a quaternion, q
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    return pose;
}


//given a transform, convert this to an affine3f
void transformTFToEigen(const tf::Transform &t, Eigen::Affine3f &e) {
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i];
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j];
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
}


Eigen::Affine3f get_grasp_transform() {
    g_pcl_getframe_msg.request.cmd_mode = PCL_FRAME_SVC_GET_GRASP_FRAME_WRT_MODEL; // modes 1 or 2: frame in kinect coords (1) or torso (2)
    bool status = g_pcl_getframe_svc_client.call(g_pcl_getframe_msg);
    geometry_msgs::PoseStamped pose_grasp_wrt_model;
    pose_grasp_wrt_model = g_pcl_getframe_msg.response.poseStamped_IM_current;// name is misnomer--really just re-using this srv type
    Eigen::Affine3f grasp_wrt_model_frame = affine3f_from_poseStamped(pose_grasp_wrt_model);
}


geometry_msgs::PoseStamped get_model_pose_wrt_torso(int cmd_mode) {
    geometry_msgs::PoseStamped poseStamped;

    ROS_INFO("requesting pose from pcl_perception: ");
    g_pcl_getframe_msg.request.cmd_mode = cmd_mode; // modes 1 or 2: frame in kinect coords (1) or torso (2)
    bool status = g_pcl_getframe_svc_client.call(g_pcl_getframe_msg);
    if (status!=true) ROS_WARN("getframe service client failure!");
    geometry_msgs::PoseStamped pose_from_pcl, pose_wrt_torso;
    pose_from_pcl = g_pcl_getframe_msg.response.poseStamped_IM_current;
    ROS_INFO("got current model pose: x,y,z = %f, %f, %f",
            pose_from_pcl.pose.position.x,
            pose_from_pcl.pose.position.y,
            pose_from_pcl.pose.position.z);
    ROS_INFO("quaternion is: %f, %f, %f, %f",
            pose_from_pcl.pose.orientation.x,
            pose_from_pcl.pose.orientation.y,
            pose_from_pcl.pose.orientation.z,
            pose_from_pcl.pose.orientation.w);

    // transform this into torso coords:
    g_tfListener_ptr->transformPose("torso", pose_from_pcl, pose_wrt_torso);
    ROS_INFO("pose w/rt torso: ");
    ROS_INFO("origin: %f, %f, %f", pose_wrt_torso.pose.position.x, pose_wrt_torso.pose.position.y, pose_wrt_torso.pose.position.z);
    ROS_INFO("orientation: %f, %f, %f, %f", pose_wrt_torso.pose.orientation.x, pose_wrt_torso.pose.orientation.y, pose_wrt_torso.pose.orientation.z, pose_wrt_torso.pose.orientation.w);
    return pose_wrt_torso;
}

Eigen::Affine3f  get_yale_gripper_affine_wrt_torso() {
    geometry_msgs::PoseStamped poseStamped;

    
    ROS_INFO("getting gripper pose: ");
    //g_pcl_getframe_msg.request.cmd_mode = cmd_mode; // modes 1 or 2: frame in kinect coords (1) or torso (2)
    //bool status = g_pcl_getframe_svc_client.call(g_pcl_getframe_msg);
    //if (status!=true) ROS_WARN("getframe service client failure!");
    geometry_msgs::PoseStamped pose_wrt_torso;
    tf::StampedTransform tf_right_hand_wrt_torso;
    Eigen::Affine3f yale_gripper_affine_wrt_torso;
    
    bool tferr = true;
    ROS_INFO("waiting for tf between yale gripper frame and torso...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            //g_tfListener_ptr->lookupTransform("yale_gripper_frame", "torso", ros::Time(0), tf_right_hand_wrt_torso);
            g_tfListener_ptr->lookupTransform("torso", "yale_gripper_frame", ros::Time(0), tf_right_hand_wrt_torso);

        } catch (tf::TransformException &exception) {
            ROS_WARN("%s", exception.what());
            tferr = true;
            ros::Duration(0.1).sleep(); // sleep briefly
            ros::spinOnce();
        }
    }
    ROS_INFO("got gripper tf");
    transformTFToEigen(tf_right_hand_wrt_torso, yale_gripper_affine_wrt_torso);
    return yale_gripper_affine_wrt_torso;
}

bool arm_server_busy_wait_done() {
    int status= ARM_SERVER_IS_BUSY;
    int nwaits=0;
    cout<<"waiting on arm server: ";
    while (status != ARM_SERVER_NOT_BUSY) {
        arm_nav_msg.request.cmd_mode = ARM_IS_SERVER_BUSY_QUERY;
        g_arm_interface_svc_client.call(arm_nav_msg); 
        status = arm_nav_msg.response.rtn_code;
        cout<<".";
        nwaits++;
        //cout<<"status code: "<<status<<endl;
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        if (nwaits>50) {
            ROS_WARN("coord: giving up waiting on arm interface");
            return false;
        }
    }
    cout<<"done"<<endl;
    return true;
}

bool perception_busy_wait_done() {
    int status= PCL_STATUS_BUSY;
    int nwaits=0;
    cout<<"waiting on perception service: ";
    while (status != PCL_STATUS_IDLE) {
        g_pcl_perception_msg.request.req = PCL_INQUIRE_STATUS;
        g_pcl_perception_svc_client.call(g_pcl_perception_msg);        
        status = g_pcl_perception_msg.response.resp;
        cout<<".";
        nwaits++;
        //cout<<"status code: "<<status<<endl;
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        if (nwaits>50) {
            ROS_WARN("coord: giving up waiting on perception interface");
            return false;
        }
    }
    cout<<"done"<<endl;
    return true;
}

//utility fnc to convert from a rotation matrix to a geometry_msgs::Quaternion
geometry_msgs::Quaternion quaternion_from_R(Eigen::Matrix3d R) {
    geometry_msgs::Quaternion quat_msg;
    Eigen::Quaterniond  eigen_quat(R);
    quat_msg.x = eigen_quat.x();
    quat_msg.y = eigen_quat.y();
    quat_msg.z = eigen_quat.z();
    quat_msg.w = eigen_quat.w();   
    return quat_msg;
}

/*
 G_MODULE_EXPORT void open_gripper_cb(GtkButton *open_gripper, gpointer data) 
{
  printf("opening gripper\n");
  system("rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- -4.0"); //angle pi rot of motor--pretty much fully open fingers
}

G_MODULE_EXPORT void close_gripper_cb(GtkButton *close_gripper, gpointer data) 
{
  printf("closing gripper\n");
  system("rostopic pub -1 /tilt_controller/command std_msgs/Float64 -- -1.1"); // angle 90deg rot of motor; should be OK to pick up can
}*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "coordinator");
    ros::NodeHandle nh; //standard ros node handle   
    ros::Rate rate(2);
    bool status;

    geometry_msgs::Pose pose;
    geometry_msgs::PoseStamped poseStamped;
    geometry_msgs::PoseStamped pose_from_pcl, model_pose_wrt_torso,grasp_pose_wrt_model, grasp_pose_wrt_torso, approach_pose_wrt_torso;
    geometry_msgs::Pose yale_gripper_pose_wrt_torso; 

    Eigen::Affine3f A_grasp_wrt_model;
    Eigen::Vector3d n_des, t_des, b_des;
    b_des << 0, 0, -1;
    n_des << 1, 0, 0;
    t_des = b_des.cross(n_des);

    Eigen::Matrix3d R_flange_down;
    R_flange_down.col(0) = n_des;
    R_flange_down.col(1) = t_des;
    R_flange_down.col(2) = b_des;
    
    geometry_msgs::Quaternion quat_gripper_down = quaternion_from_R(R_flange_down);
    
    //define a pre-pose in this node; note--may not be same as pre-pose in arm interface node
    Eigen::Matrix<double, 7, 1> coord_pre_pose;
    //coord_pre_pose<< -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, -2.87179;   
    coord_pre_pose<< -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0; 
      
    tf::TransformListener tfListener;
    g_tfListener_ptr = &tfListener;
    
    ros::Publisher gripper_intfc = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 1);
    std_msgs::Float64 gripper_open;
    gripper_open.data = -4.0; //-4 rad opens fingers ~ maximally
    std_msgs::Float64 gripper_close_can;
    gripper_close_can.data = -1.1; //tune for grasp of Coke can
    gripper_intfc.publish(gripper_open); // publish the value--of type Float64-- 
    //tf::StampedTransform tf_kinect_wrt_torso;
    tf::StampedTransform tf_right_hand_wrt_torso; 
    
    Eigen::Affine3f yale_gripper_affine_wrt_torso;

    // wait to start receiving valid tf transforms 
    

    bool tferr = true;
    ROS_INFO("waiting for tf between yale gripper frame and torso...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            g_tfListener_ptr->lookupTransform("yale_gripper_frame", "torso", ros::Time(0), tf_right_hand_wrt_torso);
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
    g_pcl_perception_svc_client= nh.serviceClient<cwru_srv::simple_int_service_message>("pcl_perception_svc");    
    //talk to the reachability node:
    ros::ServiceClient reachability_svc_client = nh.serviceClient<cwru_srv::simple_float_service_message>("compute_reachability_svc");
    // talk to arm interface:
    g_arm_interface_svc_client= nh.serviceClient<cwru_srv::arm_nav_service_message>("cartMoveSvc");
    
    ros::ServiceServer service = nh.advertiseService("coordinator_svc", coordinatorService);
    double des_z_height=0.0;
    
    std::vector<Eigen::Affine3d> grasp_xforms; //vector to hold object grasp frame options    
    Object_grasp_frame_computer object_grasp_frame_computer; // create a grasp-frame computer object

    bool wait_done=true;
    geometry_msgs::Quaternion quaternion;
    while (ros::ok()) {
        if (g_trigger) {
            g_trigger = false; // reset the trigger

            switch (g_coordinator_mode) { // what we do here depends on our mode; mode is settable via a service
                case RQST_DO_NOTHING:
                    ROS_INFO("case DO_NOTHING; doing nothing!");
                    break;
                case RQST_MOVE_MARKER_TO_GRASP_POSE:
                    ROS_INFO("case MOVE_MARKER_TO_GRASP_POSE");                    
                    model_pose_wrt_torso = get_model_pose_wrt_torso(1); // mode 1--model frame in Kinect coords

                    // put the marker origin at the top of the can:
                    pose = model_pose_wrt_torso.pose;
                    pose.position.z += H_CYLINDER;
                    pose.orientation = quat_gripper_down; // coerce orientation to point down
                    
                    poseStamped.pose = pose;
                    poseStamped.header.stamp = ros::Time::now();
                    poseStamped.header.frame_id= "torso";
                    IM_6dof_srv_msg.request.cmd_mode = IM_SET_NEW_MARKER_POSE;
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
                    
                case RQST_MOVE_MARKER_TO_HAND_POSE:
                    ROS_INFO("case RQST_MOVE_MARKER_TO_HAND_POSE");  
                    yale_gripper_affine_wrt_torso  =get_yale_gripper_affine_wrt_torso(); 
                    //obtain current right gripper frame
                    yale_gripper_pose_wrt_torso =  transformEigenAffine3fToPose(yale_gripper_affine_wrt_torso);

                    // put the marker origin coincident with current gripper frame;
                    //pose = model_pose_wrt_torso.pose;
                    //pose.position.z += H_CYLINDER;
                    //pose.orientation = quat_gripper_down; // coerce orientation to point down
                    
                    poseStamped.pose = yale_gripper_pose_wrt_torso;
                    poseStamped.header.stamp = ros::Time::now();
                    poseStamped.header.frame_id= "torso";
                    
                    
                    IM_6dof_srv_msg.request.cmd_mode = IM_SET_NEW_MARKER_POSE;
                    IM_6dof_srv_msg.request.poseStamped_IM_desired = poseStamped;
                    ROS_INFO("placing IM at coincident with gripper frame");
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);                    
                    break;
                    
                case RQST_DISPLAY_REACHABILITY_AT_MARKER_HEIGHT:
                    IM_6dof_srv_msg.request.cmd_mode = IM_GET_CURRENT_MARKER_POSE;
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);
                    des_z_height = IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.z;
                    
                    ROS_INFO("case DISPLAY_REACHABILITY_AT_MARKER_HEIGHT: z_des = %f",des_z_height);
                    // compute/display reachability at marker height:
                    reachability_msg.request.request_float32 = des_z_height;
                    reachability_svc_client.call(reachability_msg);  
                    //arm_server_busy_wait_done(); // no...don't wait on arm server!  wait on reachability
                    
                    break;
                case RQST_COMPUTE_MOVE_ARM_TO_PRE_POSE:
                    ROS_INFO("case COMPUTE_MOVE_ARM_TO_PRE_POSE");
                    arm_nav_msg.request.cmd_mode= ARM_PLAN_PATH_CURRENT_TO_PRE_POSE;
                    g_arm_interface_svc_client.call(arm_nav_msg); //need error checking here
                    arm_server_busy_wait_done();
                    //let's see if resulting plan was successful:
                    arm_nav_msg.request.cmd_mode= ARM_QUERY_IS_PATH_VALID; 
                    g_arm_interface_svc_client.call(arm_nav_msg);
                    if (arm_nav_msg.response.rtn_code==ARM_PATH_IS_VALID) {
                        ROS_INFO("computed a valid path");
                    }
                    break;
                case RQST_COMPUTE_MOVE_ARM_JSPACE_CURRENT_TO_PRE_POSE:
                     ROS_INFO("case RQST_COMPUTE_MOVE_ARM_JSPACE_CURRENT_TO_PRE_POSE");
                    arm_nav_msg.request.cmd_mode= ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL; //ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE;
                    //need to fill in goal:
                    arm_nav_msg.request.q_vec_end.resize(7);
                    for (int i=0;i<7;i++ )  {
                        arm_nav_msg.request.q_vec_end[i] = coord_pre_pose[i];
                    }
                    g_arm_interface_svc_client.call(arm_nav_msg); //need error checking here
                    arm_server_busy_wait_done();
                    //let's see if resulting plan was successful:
                    arm_nav_msg.request.cmd_mode= ARM_QUERY_IS_PATH_VALID; 
                    g_arm_interface_svc_client.call(arm_nav_msg);
                    if (arm_nav_msg.response.rtn_code==ARM_PATH_IS_VALID) {
                        ROS_INFO("computed a valid path");
                        arm_nav_msg.request.cmd_mode= ARM_EXECUTE_PLANNED_PATH;     
                        g_arm_interface_svc_client.call(arm_nav_msg);
                        arm_server_busy_wait_done();
                    }                   
                    break;
                case  RQST_PREVIEW_TRAJECTORY:
                     ROS_INFO("case RQST_PREVIEW_TRAJECTORY");
                    arm_nav_msg.request.cmd_mode= ARM_DISPLAY_TRAJECTORY; //request a trajectory preview
                    g_arm_interface_svc_client.call(arm_nav_msg); //need error checking here
                    arm_server_busy_wait_done();
                    break;
                    
                case RQST_EXECUTE_PLANNED_PATH:
                    ROS_INFO("case RQST_EXECUTE_PLANNED_PATH");
                    arm_nav_msg.request.cmd_mode= ARM_EXECUTE_PLANNED_PATH;     
                    g_arm_interface_svc_client.call(arm_nav_msg);
                    arm_server_busy_wait_done();
                    break;
                case RQST_DESCEND_20CM:
                    ROS_INFO("case RQST_DESCEND_20CM");
                    arm_nav_msg.request.cmd_mode= ARM_DESCEND_20CM;     
                    g_arm_interface_svc_client.call(arm_nav_msg);  
                    arm_server_busy_wait_done();
                    if (arm_nav_msg.response.rtn_code==ARM_PATH_IS_VALID) {
                        ROS_INFO("computed a valid path");
                        ROS_INFO("requesting execution of planned path");
                        arm_nav_msg.request.cmd_mode= ARM_EXECUTE_PLANNED_PATH;     
                        g_arm_interface_svc_client.call(arm_nav_msg);
                        arm_server_busy_wait_done();
                    }                    
                    else {
                        ROS_WARN("did not compute a valid path to grasp pose");
                        break;        
                    }                     
                    break;
                 case RQST_ASCEND_20CM:
                    ROS_INFO("case RQST_ASCEND_20CM");
                    arm_nav_msg.request.cmd_mode= ARM_DEPART_20CM;     
                    g_arm_interface_svc_client.call(arm_nav_msg);   
                    arm_server_busy_wait_done();
                    if (arm_nav_msg.response.rtn_code==ARM_PATH_IS_VALID) {
                        ROS_INFO("computed a valid path");
                        ROS_INFO("requesting execution of planned path");
                        arm_nav_msg.request.cmd_mode= ARM_EXECUTE_PLANNED_PATH;     
                        g_arm_interface_svc_client.call(arm_nav_msg);
                        arm_server_busy_wait_done();
                    }                    
                    else {
                        ROS_WARN("did not compute a valid path to grasp pose");
                        break;        
                    }     
                    break;
                    
                case RQST_COMPUTE_MOVE_ARM_TO_APPROACH_POSE:
                    ROS_INFO("case RQST_COMPUTE_MOVE_ARM_TO_APPROACH_POSE; doing nothing!");
                    break;
                case RQST_COMPUTE_MOVE_ARM_TO_GRASP_POSE:
                    ROS_INFO("case RQST_COMPUTE_MOVE_ARM_TO_GRASP_POSE; doing nothing!");
                    arm_server_busy_wait_done();
                    break;
                case RQST_COMPUTE_MOVE_ARM_DEPART:
                    ROS_INFO("case RQST_COMPUTE_MOVE_ARM_DEPART; doing nothing!");
                    arm_server_busy_wait_done();
                    break;
                case RQST_COMPUTE_MOVE_ARM_TO_MARKER: //misnomer--this is a request to compute a plan
                    ROS_INFO("RQST_COMPUTE_MOVE_ARM_TO_MARKER: planning move to marker pose");
                    IM_6dof_srv_msg.request.cmd_mode = IM_GET_CURRENT_MARKER_POSE;
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);
                    ROS_INFO("got current marker pose: x,y,z = %f, %f, %f",
                            IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.x,
                            IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.y,
                            IM_6dof_srv_msg.response.poseStamped_IM_current.pose.position.z);
                    //fill in IM goal; arm interface will get starting pose from sensor vals
                    arm_nav_msg.request.poseStamped_goal = IM_6dof_srv_msg.response.poseStamped_IM_current;
                    
                    arm_nav_msg.request.cmd_mode= ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE;  

                    g_arm_interface_svc_client.call(arm_nav_msg);           
                    wait_done=arm_server_busy_wait_done(); 
                    //let's see if resulting plan was successful:
                    arm_nav_msg.request.cmd_mode= ARM_QUERY_IS_PATH_VALID; 
                    g_arm_interface_svc_client.call(arm_nav_msg);
                    if (arm_nav_msg.response.rtn_code==ARM_PATH_IS_VALID) {
                        ROS_INFO("computed a valid path");
                        ROS_INFO("requesting execution of planned path");
                        arm_nav_msg.request.cmd_mode= ARM_EXECUTE_PLANNED_PATH;     
                        g_arm_interface_svc_client.call(arm_nav_msg);
                        arm_server_busy_wait_done();                        
                    }
                    else
                        ROS_WARN("did not compute a valid path");
                    break;
                    
                    /**/

                case RQST_GRAB_COKE_CAN_FROM_ABOVE:
                    ROS_INFO("rqst take snapshot");
                    g_pcl_perception_msg.request.req = PCL_TAKE_SNAPSHOT; 
                    g_pcl_perception_svc_client.call(g_pcl_perception_msg);
                    perception_busy_wait_done();
                    ROS_INFO("rqst find floor");
                    g_pcl_perception_msg.request.req = PCL_FIND_FLOOR;  
                    g_pcl_perception_svc_client.call(g_pcl_perception_msg);
                    perception_busy_wait_done();
                    ROS_INFO("rqst find table");
                    g_pcl_perception_msg.request.req = PCL_FIND_TABLE;  
                    g_pcl_perception_svc_client.call(g_pcl_perception_msg);
                    perception_busy_wait_done();
                    ROS_INFO("rqst find coke can frame");
                    g_pcl_perception_msg.request.req = PCL_FIND_COKE_FRAME;   
                    g_pcl_perception_svc_client.call(g_pcl_perception_msg);
                    perception_busy_wait_done();
                    
                    ROS_INFO("requesting model frame from perception service");                    
                    pose_from_pcl = get_model_pose_wrt_torso(PCL_FRAME_SVC_GET_MODEL_FRAME_WRT_TORSO); // mode 2--model frame in base coords
                    // put the marker coincident with the model frame:
                    pose_from_pcl.header.stamp = ros::Time::now();
                    ROS_INFO("requesting set new marker pose coincident w/ model frame");
                    IM_6dof_srv_msg.request.cmd_mode = IM_SET_NEW_MARKER_POSE;
                    IM_6dof_srv_msg.request.poseStamped_IM_desired = pose_from_pcl;
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);    
                   /* */
                    g_pcl_perception_msg.request.req = PCL_FIND_COKE_CAN_GRASP_FROM_ABOVE_FRAME;
                    g_pcl_perception_svc_client.call(g_pcl_perception_msg);
                    perception_busy_wait_done();
                    grasp_pose_wrt_torso = get_model_pose_wrt_torso(PCL_FRAME_SVC_GET_GRASP_FRAME_WRT_TORSO);
                    approach_pose_wrt_torso = grasp_pose_wrt_torso;
                    approach_pose_wrt_torso.pose.position.z += 0.05; // add vertical offset for approach from above
                    // move marker here:
                    ROS_INFO("requesting set new marker pose coincident w/ grasp frame");
                    IM_6dof_srv_msg.request.cmd_mode = IM_SET_NEW_MARKER_POSE;
                    IM_6dof_srv_msg.request.poseStamped_IM_desired = grasp_pose_wrt_torso;
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);                        
                    /**/
                    ROS_INFO("requesting set new marker to approach pose: ");
                    IM_6dof_srv_msg.request.cmd_mode = IM_SET_NEW_MARKER_POSE;
                    IM_6dof_srv_msg.request.poseStamped_IM_desired = approach_pose_wrt_torso;
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);                     
                    //grasp_pose_wrt_model = get_grasp_transform; // mode 2 is model frame w/rt torso; rtn geometry_msgs::PoseStamped
                    
                    arm_nav_msg.request.poseStamped_goal = approach_pose_wrt_torso;
                    arm_nav_msg.request.cmd_mode= ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE;  
                    g_arm_interface_svc_client.call(arm_nav_msg);           
                    wait_done=arm_server_busy_wait_done(); 
                    //let's see if resulting plan was successful:
                    arm_nav_msg.request.cmd_mode= ARM_QUERY_IS_PATH_VALID; 
                    g_arm_interface_svc_client.call(arm_nav_msg);
                    if (arm_nav_msg.response.rtn_code==ARM_PATH_IS_VALID) {
                        ROS_INFO("computed a valid path");
                        ROS_INFO("requesting execution of planned path");
                        arm_nav_msg.request.cmd_mode= ARM_EXECUTE_PLANNED_PATH;     
                        g_arm_interface_svc_client.call(arm_nav_msg);
                        arm_server_busy_wait_done();
                    }
                    else {
                        ROS_WARN("did not compute a valid path");
                        break;        
                    }
                    
                    // move to grasp pose:
                    arm_nav_msg.request.poseStamped_goal = grasp_pose_wrt_torso;
                    arm_nav_msg.request.cmd_mode= ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE;  
                    g_arm_interface_svc_client.call(arm_nav_msg);           
                    wait_done=arm_server_busy_wait_done(); 
                    //let's see if resulting plan was successful:
                    arm_nav_msg.request.cmd_mode= ARM_QUERY_IS_PATH_VALID; 
                    g_arm_interface_svc_client.call(arm_nav_msg);
                    if (arm_nav_msg.response.rtn_code==ARM_PATH_IS_VALID) {
                        ROS_INFO("computed a valid path");
                        ROS_INFO("requesting execution of planned path");
                        arm_nav_msg.request.cmd_mode= ARM_EXECUTE_PLANNED_PATH;     
                        g_arm_interface_svc_client.call(arm_nav_msg);
                        arm_server_busy_wait_done();
                    }                    
                    else {
                        ROS_WARN("did not compute a valid path to grasp pose");
                        break;        
                    }     
                    gripper_intfc.publish(gripper_close_can);

                    break;
                    
                case RQST_MOVE_MARKER_TO_MODEL_FRAME:
                    ROS_INFO("requesting model frame from perception service");                    
                    pose_from_pcl = get_model_pose_wrt_torso(PCL_FRAME_SVC_GET_MODEL_FRAME_WRT_TORSO); // mode 2--model frame in base coords
                    // put the marker coincident with the model frame:
                    pose_from_pcl.header.stamp = ros::Time::now();
                    ROS_INFO("requesting set new marker pose coincident w/ model frame");
                    IM_6dof_srv_msg.request.cmd_mode = IM_SET_NEW_MARKER_POSE;
                    IM_6dof_srv_msg.request.poseStamped_IM_desired = pose_from_pcl;
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);                        
                    break;

                case RQST_GRAB_GAZEBO_BEER_CAN_FROM_ABOVE:
                    ROS_INFO("rqst take snapshot");
                    g_pcl_perception_msg.request.req = PCL_TAKE_SNAPSHOT; 
                    g_pcl_perception_svc_client.call(g_pcl_perception_msg);
                    perception_busy_wait_done();
                    ROS_INFO("rqst find floor");
                    g_pcl_perception_msg.request.req = PCL_FIND_FLOOR;  
                    g_pcl_perception_svc_client.call(g_pcl_perception_msg);
                    perception_busy_wait_done();
                    ROS_INFO("rqst find table");
                    g_pcl_perception_msg.request.req = PCL_FIND_TABLE;  
                    g_pcl_perception_svc_client.call(g_pcl_perception_msg);
                    perception_busy_wait_done();
                    ROS_INFO("rqst find coke can frame");
                    g_pcl_perception_msg.request.req = PCL_FIND_GAZEBO_BEER_FRAME;   
                    g_pcl_perception_svc_client.call(g_pcl_perception_msg);
                    perception_busy_wait_done();
                    ROS_INFO("requesting model frame from perception service");                    
                    pose_from_pcl = get_model_pose_wrt_torso(PCL_FRAME_SVC_GET_MODEL_FRAME_WRT_TORSO); // mode 2--model frame in base coords
                    // put the marker coincident with the model frame:
                    pose_from_pcl.header.stamp = ros::Time::now();
                    ROS_INFO("requesting set new marker pose coincident w/ model frame");
                    IM_6dof_srv_msg.request.cmd_mode = IM_SET_NEW_MARKER_POSE;
                    IM_6dof_srv_msg.request.poseStamped_IM_desired = pose_from_pcl;
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);    
                    
                    g_pcl_perception_msg.request.req = PCL_FIND_GAZEBO_BEER_GRASP_FROM_ABOVE_FRAME;
                    g_pcl_perception_svc_client.call(g_pcl_perception_msg);
                    perception_busy_wait_done();
                    grasp_pose_wrt_torso = get_model_pose_wrt_torso(PCL_FRAME_SVC_GET_GRASP_FRAME_WRT_TORSO);
                    // move marker here:
                    ROS_INFO("requesting set new marker pose coincident w/ grasp frame");
                    IM_6dof_srv_msg.request.cmd_mode = IM_SET_NEW_MARKER_POSE;
                    IM_6dof_srv_msg.request.poseStamped_IM_desired = grasp_pose_wrt_torso;
                    status = IM_6dof_svc_client.call(IM_6dof_srv_msg);                        
                    
                    //grasp_pose_wrt_model = get_grasp_transform; // mode 2 is model frame w/rt torso; rtn geometry_msgs::PoseStamped
                     
                    break;
                    
                default:
                    ROS_WARN("this mode is not implemented");

            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
