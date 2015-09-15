//use ROS service to move objects in Gazebo
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SetModelState.h>
#include <math.h>


// use service: gazebo/set_model_state
// w/ message: gazebo_msgs/SetModelState
//
// first run: rosrun example_ROS_service example_ROS_service
// then start this node:  rosrun example_ROS_service example_ROS_client



#include <ros/ros.h>
#include <example_ros_service/example_server_msg.h> // this message type is defined in the current package
#include <iostream>
#include <string>
using namespace std;

geometry_msgs::Quaternion  convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_gazebo_model");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
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
     pose.position.x = 0.5;
     pose.position.y = 0.0;
     pose.position.z = 0.775;
     quat.x = 0.0;
     quat.y = 0.0;
     quat.z = 0.0;
     quat.w = 1.0;
     pose.orientation= quat;
    cout<<"enter model name: ";
    char name[32];
    cin>>name;
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
   for (int i=0;i<nsteps;i++) {
        //cout<<"you entered "<<in_name<<endl;
        //des_model_state.pose.position.x+=0.001;

	quat = convertPlanarPhi2Quaternion(theta_yaw);
        qz = quat.z;
        qw = quat.w;
	cout<<"qz, qw= "<<qz<<", "<<qw<<endl;
        des_model_state.pose.orientation=quat;
        set_model_state_srv.request.model_state = des_model_state;
        if (client.call(set_model_state_srv)) {
            if (set_model_state_srv.response.success) {
		ROS_INFO("set model state reports success");
               
            } else {
                ROS_INFO("set model state reports failure");
            }

        } else {
            ROS_ERROR("Failed to call service ");
            return 1;
        }
    ros::Duration(0.1).sleep();
    cout<<"enter 1: ";
    cin>>ans;
        theta_yaw+= dphi;
    }
    return 0;
}
