//use ROS service to move objects in Gazebo
// put 1mm_bar at pose suitable for initial grasp
// for rosrun playfile_reader playfile_cameraspace test_cameraspace_peg_handoff.csp

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
// #include <example_ros_service/example_server_msg.h>  //(removed) this message type is defined in the current package
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
    double x = -0.017; //-0.118;
    double y = 0.016; //0.022;
    double z = 0.74;
    double qx=0.0;
    double qy=0.0;
    double qz = 0.0;
    double qw = 1.0;    
     pose.position.x = x;
     pose.position.y = y;
     pose.position.z = z;
     quat.x = qx;
     quat.y = qy;
     quat.z = 0.0;
     quat.w = 1.0;
     pose.orientation= quat;
    //cout<<"enter model name: ";
    char name[]="1mm_bar";
    //cin>>name;
    des_model_state.model_name = name; //"checkerboard";//"wood_cube_10cm";//"beer";
    des_model_state.pose = pose;
    des_model_state.twist = twist;
    des_model_state.reference_frame = "world";


    
    while(ros::ok()) 
  {
        //cout<<"you entered "<<in_name<<endl;
        //des_model_state.pose.position.x+=0.001;


        // overwrite:
        double norm = sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
        quat.x = qx/norm;
        quat.y = qy/norm;
        quat.z = qz/norm;
        quat.w = qw/norm;
	//cout<<"qx, qy, qz, qw= "<<qx<<", "<<qy<<", "<<qz<<", "<<qw<<endl;
	pose.orientation= quat;
        //des_model_state.pose.orientation=quat;
     	pose.position.x = x;
     	pose.position.y = y;
     	pose.position.z = z;
    	des_model_state.pose = pose;
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
    ros::Duration(0.01).sleep();
    //try x = -0.0067, y= -0.06, z = 0.1 and change rot's
    //cout<<"enter x: ";
    //cin >>x;
    //cout<<"enter y: ";
    //cin>>y;
    //cout<<"enter z: ";
    //cin>>z;
    }
    return 0;
}
