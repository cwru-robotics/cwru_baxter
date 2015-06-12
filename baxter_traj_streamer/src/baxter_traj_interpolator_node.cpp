// wsn pgm to receive Baxter trajectories and interpolate them smoothly
// as commands to Baxter;
// right arm only, at present; June 1, 2015
#include <baxter_traj_streamer/baxter_traj_streamer.h>

using namespace std;

trajectory_msgs::JointTrajectory new_trajectory; // global var to receive new traj's;
bool got_new_trajectory = false;
bool working_on_trajectory = false;
baxter_core_msgs::JointCommand right_cmd,left_cmd;
ros::Publisher joint_cmd_pub_right;
double dt_traj = 0.01; // time step
//complement of this:
// right_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("right_arm_joint_path_command", 1);  
void trajectoryCb(const trajectory_msgs::JointTrajectory& tj_msg) {
    // copy trajectory to global var:
    new_trajectory = tj_msg; // does this work?
    // insist that a traj have at least 2 pts
    if (new_trajectory.points.size()>1) got_new_trajectory = true;
    cout<<"Cb received traj w/ npts = "<<new_trajectory.points.size()<<endl;
        trajectory_msgs::JointTrajectoryPoint trajectory_point0;
        trajectory_point0 = new_trajectory.points[0];  
        trajectory_point0 =  tj_msg.points[0];   
        cout<<tj_msg.points[0].positions.size()<<" =  tj_msg.points[0].positions.size()"<<endl;
        cout<<"size of positions[]: "<<trajectory_point0.positions.size()<<endl;
        cout<<"1st pt: ";
          for (int i=0;i<7;i++) { //copy from traj point to 7x1 vector
            cout<<trajectory_point0.positions[i]<<", ";           
          }
          cout<<endl;

} 

void cmd_pose_right(Vectorq7x1 qvec ) {
    //member var right_cmd_ already has joint names populated
    for (int i=0;i<7;i++) {
        right_cmd.command[i]=qvec[i];
    }
    joint_cmd_pub_right.publish(right_cmd);
}

bool trajInterpStatusSvc(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    //ROS_INFO("responding to service request: status of trajectory interpolator");
    response.resp = working_on_trajectory; // return status of "working on trajectory"
    return true;
}

// this is the interesting func: compute new qvec
// update isegment and qvec according to traj_clock; 
//if traj_clock>= final_time, use exact end coords and set "working_on_trajectory" to false   
bool  update_trajectory(double traj_clock,trajectory_msgs::JointTrajectory trajectory,Vectorq7x1 qvec_prev, int &isegment,Vectorq7x1 &qvec_new){
    trajectory_msgs::JointTrajectoryPoint trajectory_point_from,trajectory_point_to;    
    Vectorq7x1 qvec,qvec_to,delta_qvec,dqvec;
    int nsegs = trajectory.points.size()-1;
    double t_subgoal;
    //cout<<"traj_clock = "<<traj_clock<<endl;
    if (isegment<nsegs) {
        trajectory_point_to = trajectory.points[isegment+1];
        t_subgoal = trajectory_point_to.time_from_start.toSec(); 
        //cout<<"iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
    }
    else {
        cout<<"reached end of last segment"<<endl;
         trajectory_point_to = trajectory.points[nsegs];
         t_subgoal = trajectory_point_to.time_from_start.toSec();  
            for (int i=0;i<7;i++) {
                 qvec_new[i] = trajectory_point_to.positions[i]; 
            }   
         cout<<"final time: "<<t_subgoal<<endl;
            return false;        
    }
    
    //cout<<"iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
    while ((t_subgoal< traj_clock)&&(isegment<nsegs)) {
        //cout<<"loop: iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
        isegment++;
        if (isegment>nsegs-1) {
            //last point
            trajectory_point_to = trajectory.points[nsegs];
            for (int i=0;i<7;i++) {
                 qvec_new[i] = trajectory_point_to.positions[i]; 
            }        
            cout<<"iseg>nsegs"<<endl;
            return false;
        }

       trajectory_point_to = trajectory.points[isegment+1];
       t_subgoal = trajectory_point_to.time_from_start.toSec();
    }
    //cout<<"t_subgoal = "<<t_subgoal<<endl;
    //here if have a valid segment:
    for (int i=0;i<7;i++) {
           qvec_to[i] = trajectory_point_to.positions[i]; 
    }
     delta_qvec = qvec_to - qvec_prev; //this far to go until next node;
    double delta_time = t_subgoal-traj_clock;
    if (delta_time<dt_traj) delta_time= dt_traj;
    dqvec = delta_qvec*dt_traj/delta_time;
    qvec_new = qvec_prev + dqvec;
    return true; 
}

int main(int argc, char** argv){

  double traj_clock, dt_segment,dq_segment,delta_q_segment,traj_final_time;
  int isegment;
  trajectory_msgs::JointTrajectoryPoint trajectory_point0;

  Vectorq7x1 qvec,qvec0,qvec_prev,qvec_new;
    ros::init(argc, argv, "baxter_kinematics_test_main");
    ros::NodeHandle nh;
    
    ros::Subscriber traj_sub = nh.subscribe("right_arm_joint_path_command", 1, trajectoryCb); 
    //publisher is global
    joint_cmd_pub_right = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1); 
    
    ROS_INFO("Initializing Services");
    ros::ServiceServer statusService = nh.advertiseService("trajInterpStatusSvc",trajInterpStatusSvc); 
    
    //initializations:
    left_cmd.mode = 1; // set the command modes to "position"
    right_cmd.mode = 1;

// define the joint angles 0-6 to be right arm, from shoulder out to wrist;
  right_cmd.names.push_back("right_s0");
  right_cmd.names.push_back("right_s1");
  right_cmd.names.push_back("right_e0");
  right_cmd.names.push_back("right_e1");
  right_cmd.names.push_back("right_w0");
  right_cmd.names.push_back("right_w1");
  right_cmd.names.push_back("right_w2");
// same order for left arm
  left_cmd.names.push_back("left_s0");
  left_cmd.names.push_back("left_s1");
  left_cmd.names.push_back("left_e0");
  left_cmd.names.push_back("left_e1");
  left_cmd.names.push_back("left_w0");
  left_cmd.names.push_back("left_w1");
  left_cmd.names.push_back("left_w2");
  // do push-backs to establish desired vector size with valid joint angles
  for (int i=0;i<7;i++) {
     right_cmd.command.push_back(0.0); // start commanding 0 angle for right-arm 7 joints
     left_cmd.command.push_back(0.0); // start commanding 0 angle for left-arm 7 joints
   } 
  
  ROS_INFO("ready to receive/execute trajectories");
  //main loop:
    while (ros::ok()) {
      if ((!working_on_trajectory)&&got_new_trajectory) {
          working_on_trajectory=true;
          got_new_trajectory=false;
          traj_clock=0.0; // initialize clock for trajectory;
          isegment=0;
          trajectory_point0 = new_trajectory.points[0];         
          for (int i=0;i<7;i++) { //copy from traj point to 7x1 vector
            qvec0[i] = trajectory_point0.positions[i];           
          }
          cmd_pose_right(qvec0);  //populate and send out first command  
          qvec_prev = qvec0;
          cout<<"start pt: "<< qvec0.transpose()<<endl;
      }
      //working_on_trajectory = false;

      if (working_on_trajectory) {
          traj_clock+=dt_traj;
          // update isegment and qvec according to traj_clock; 
          //if traj_clock>= final_time, use exact end coords and set "working_on_trajectory" to false          
          working_on_trajectory = update_trajectory(traj_clock,new_trajectory,qvec_prev,isegment,qvec_new);
          cmd_pose_right(qvec_new); // use qvec to populate object and send it to robot
          qvec_prev = qvec_new;
          cout<<"traj_clock: "<<traj_clock<<"; vec:"<<qvec_new.transpose()<<endl;
          if (!working_on_trajectory)
              cout<<"completed execution of a trajectory"<<endl;
      }
    ros::spinOnce();
    ros::Duration(dt_traj).sleep();
  }    
}
