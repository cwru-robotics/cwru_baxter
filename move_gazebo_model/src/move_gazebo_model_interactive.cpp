//use ROS service to move objects in Gazebo
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SetModelState.h>
#include <cwru_srv/simple_bool_service_message.h>
#include <math.h>


// use service: gazebo/set_model_state
// w/ message: gazebo_msgs/SetModelState
//
// first run: rosrun example_ROS_service example_ROS_service
// then start this node:  rosrun example_ROS_service example_ROS_client

// sequence ctr, upper left, upper right, lower right, lower left, center, zoom
// rotations:  
/*
const int npts = 16;
double xvals[npts] = {-.135,-0.005,-0.005,-0.135,-0.067, -0.067,
     -0.067,-0.067,-0.067,-0.067,-0.067,-0.067,-0.067,-0.067,-0.067,-0.067};
double yvals[npts] = {-0.02, -0.02, -0.1, -0.1, -0.06,-0.06,
     -0.06,-0.06,-0.06,-0.06,-0.06,-0.06,-0.06,-0.06,-0.06,-0.06};
double zvals[npts] = {0.50, 0.50,0.50,0.50,0.50,0.68,
     0.50,0.50,0.50,0.50,0.50,0.50,0.50,0.50,0.50,0.50};
double qxvals[npts] = {0,0,0,0,0,0,  0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3, -0.4, 0, 0};
double qyvals[npts] = {0,0,0,0,0,0,  0,0,0,0,0,0,0,0, 0.1, 0.2};
double qzvals[npts] = {0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0.1};
double qwvals[npts] = {1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1};
int ipose=0;
bool g_trigger=true;
*/

#include <ros/ros.h>
#include <example_ros_service/example_server_msg.h> // this message type is defined in the current package
#include <iostream>
#include <string>
using namespace std; 

bool g_trigger=false;

geometry_msgs::Quaternion  convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

bool callback(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response)
{
    ROS_INFO("callback activated");
    g_trigger=true;
    response.resp=true;
    
  return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_gazebo_model");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceServer service = n.advertiseService("trigger_service", callback);
    gazebo_msgs::SetModelState set_model_state_srv;
    gazebo_msgs::ModelState des_model_state;
    geometry_msgs::Twist twist;
     twist.linear.x = 0.0;
     twist.linear.y = 0.0;
     twist.linear.z = 0.0;
     twist.angular.x = 0.0;
     twist.angular.y = 0.0;
     twist.angular.z = 0.0;

    geometry_msgs::Pose pose;
    geometry_msgs::Quaternion quat;
    cout<<"enter model name: ";
    char name[32];
    cin>>name;
    double x,y,z;
    cout<<"enter x: ";
    cin>>x;
 
    cout<<"enter y: "; 
    cin>>y;
    cout<<"enter z: ";
    cin>>z;
   //double x = -0.067;
    //double y = -0.06;
    //double z = 0.002;
    double qx=0.0;
    double qy=0.0;
     pose.position.x = x;
     pose.position.y = y;
     pose.position.z = z;
     quat.x = qx;
     quat.y = qy;
     quat.z = 0.0;
     quat.w = 1.0;
     pose.orientation= quat;

    des_model_state.model_name = name; //"checkerboard";//"wood_cube_10cm";//"beer";
    des_model_state.pose = pose;
    des_model_state.twist = twist;
    des_model_state.reference_frame = "world";
    double qz,qw,theta_yaw;
    theta_yaw=0;
    qz = 0.0;
    qw = 1.0;
    int nsteps = 360;
    int ans;

    double dphi = (M_PI*2.0)/nsteps;

   while(ros::ok()) {
        set_model_state_srv.request.model_state = des_model_state;
        if (client.call(set_model_state_srv)) {
            if (set_model_state_srv.response.success) {
		//ROS_INFO("set model state reports success");
               
            } else {
                ROS_INFO("set model state reports failure");
            }

        } else {
            ROS_ERROR("Failed to call service ");
            return 1;
        }
    ros::spinOnce();
    ros::Duration(0.01).sleep();
 
    }

  
    return 0;
}
