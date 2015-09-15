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
    double x = -0.067;
    double y = -0.06;
    double z = 0.002;
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

   while(ipose<npts) {
        //cout<<"you entered "<<in_name<<endl;
        //des_model_state.pose.position.x+=0.001;

	//quat = convertPlanarPhi2Quaternion(theta_yaw);
	qx = qxvals[ipose];
	qy = qyvals[ipose];
	qz = qzvals[ipose];
	qw = qwvals[ipose];
        double norm = sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
        quat.x = qx/norm;
        quat.y = qy/norm;
        quat.z = qz/norm;
        quat.w = qw/norm;
        if (g_trigger) {
           ipose++;
           if (ipose>npts) break;
           cout<<"pose number "<<ipose<<endl;
	   cout<<"qx, qy, qz, qw= "<<qx<<", "<<qy<<", "<<qz<<", "<<qw<<endl;
           cout<<"x,y,z = "<<xvals[ipose]<<", "<<yvals[ipose]<<", "<<zvals[ipose]<<endl;
	   pose.orientation= quat;
           //des_model_state.pose.orientation=quat;
     	   pose.position.x = xvals[ipose];
     	   pose.position.y = yvals[ipose];
     	   pose.position.z = zvals[ipose];
    	   des_model_state.pose = pose;
           g_trigger=false;
	}
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
    ros::Duration(0.1).sleep();
    //char ans_char[10];
    //cout<<"enter : ";
    //cin >>ans_char;
 
    }
   while(true) {
        //cout<<"you entered "<<in_name<<endl;
        //des_model_state.pose.position.x+=0.001;

	//quat = convertPlanarPhi2Quaternion(theta_yaw);
	qx = qxvals[0]+ ( (rand()%100)/100.0)-0.5; //rand in range -0.5 to +0.5
	qy = qyvals[0]+ ( (rand()%100)/100.0)-0.5;
	qz = qzvals[0]+ ( (rand()%100)/100.0)-0.5;
	qw = qwvals[0]+ ( (rand()%100)/100.0)-0.5;

	x = xvals[4]+ 0.15*((rand()%100)/100.0-0.5); //rand in range -0.075 to +0.075
	y = yvals[4]+ 0.15*((rand()%100)/100.0-0.5); //rand in range -0.075 to +0.075
	z = zvals[4]+ 0.15*((rand()%100)/100.0-0.5); //rand in range -0.075 to +0.075

        double norm = sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
	qx/=norm;
	qy/=norm;
	qz/=norm;
	qw/=norm;
        quat.x = qx/norm;
        quat.y = qy/norm;
        quat.z = qz/norm;
        quat.w = qw/norm;

           cout<<"pose number "<<ipose<<endl;
	   cout<<"qx, qy, qz, qw= "<<qx<<", "<<qy<<", "<<qz<<", "<<qw<<endl;
           cout<<"x,y,z = "<<x<<", "<<y<<", "<<z<<endl;
	   pose.orientation= quat;
           //des_model_state.pose.orientation=quat;
     	   pose.position.x = x;
     	   pose.position.y = y;
     	   pose.position.z = z;
    	   des_model_state.pose = pose;
      for (int i=0;i<100;i++) {
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
       ros::Duration(0.05).sleep();
     }
 
    }
    return 0;
}
