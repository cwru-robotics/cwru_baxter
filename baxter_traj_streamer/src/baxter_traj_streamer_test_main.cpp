// wsn pgm to test baxter_traj_streamer class
#include <baxter_traj_streamer/baxter_traj_streamer.h>

using namespace std;

int main(int argc, char** argv){
  double dt = 0.01; // time step

  Vectorq7x1 q_cmd,q_cmd2,q_cmd3;
  

  trajectory_msgs::JointTrajectory new_trajectory; // an empty trajectory
  
  //create a dummy example of sequence of joint angles
  q_cmd<<0,0,0,0,0,0,0;
  q_cmd2<< -0.5, -1.20,  0.5, 0.7,  0.92, 1.23, -0.5;
  q_cmd3 << -0.5, -1.20,  0.5, 0.7,  -0.92, 1.23, -0.5;  
  std::vector<Vectorq7x1> q_solns; 
  q_solns.push_back(q_cmd);
  q_solns.push_back(q_cmd2);  
  q_solns.push_back(q_cmd3); 
  
  ros::init(argc, argv, "Baxter_traj_streamer"); //standard ros init with node name string
  ros::NodeHandle nh; //standard ros node handle


  ROS_INFO("main: instantiating an object of type Baxter_traj_streamer");
  Baxter_traj_streamer baxter_traj_streamer(&nh);  //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
  ROS_INFO("make sure we have valid joint-state data: ");
  Vectorq7x1 q_snapshot;
  q_snapshot<<1000,1000,1000,1000,1000,1000,1000;
  while (q_snapshot[0]>10.0) {
    q_snapshot  = baxter_traj_streamer.get_qvec_right_arm();
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("warming up the command listener...");
  for (int i=0;i<10;i++) {
    baxter_traj_streamer.pub_right_arm_trajectory_init();  
    ros::spinOnce();
    ros::Duration(0.1).sleep();     
  }
  
  while(ros::ok()){ //while ROS is okay...
  baxter_traj_streamer.stuff_trajectory(q_solns, new_trajectory); //convert from vector of 7dof poses to trajectory message
  baxter_traj_streamer.pub_right_arm_trajectory(new_trajectory); //add timing and publish the traj message
  
  //baxter_traj_streamer.cmd_pose_right(q_cmd);
     ros::Duration(2.0).sleep(); //Wait dt
     ros::spinOnce();
  } 
  
  return 0;
}
