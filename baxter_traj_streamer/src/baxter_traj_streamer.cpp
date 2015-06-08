// wsn test pgm to command joint values to Baxter
// accept keyboard commands
//"include" path--should just be <baxter_kinematics/baxter_kinematics.h>, at least for modules outside this package
#include <baxter_traj_streamer/baxter_traj_streamer.h>

// odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
Baxter_traj_streamer::Baxter_traj_streamer(ros::NodeHandle* nodehandle){
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();    
  left_cmd_.mode = 1; // set the command modes to "position"
  right_cmd_.mode = 1;

// define the joint angles 0-6 to be right arm, from shoulder out to wrist;
  right_cmd_.names.push_back("right_s0");
  right_cmd_.names.push_back("right_s1");
  right_cmd_.names.push_back("right_e0");
  right_cmd_.names.push_back("right_e1");
  right_cmd_.names.push_back("right_w0");
  right_cmd_.names.push_back("right_w1");
  right_cmd_.names.push_back("right_w2");
// same order for left arm
  left_cmd_.names.push_back("left_s0");
  left_cmd_.names.push_back("left_s1");
  left_cmd_.names.push_back("left_e0");
  left_cmd_.names.push_back("left_e1");
  left_cmd_.names.push_back("left_w0");
  left_cmd_.names.push_back("left_w1");
  left_cmd_.names.push_back("left_w2");

  // do push-backs to establish desired vector size with valid joint angles
  for (int i=0;i<7;i++) {
     right_cmd_.command.push_back(0.0); // start commanding 0 angle for right-arm 7 joints
     left_cmd_.command.push_back(0.0); // start commanding 0 angle for left-arm 7 joints
   }    
  
    qdot_max_vec<<q0dotmax,q1dotmax,q2dotmax,q3dotmax,q4dotmax,q5dotmax,q6dotmax;
}

//member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void Baxter_traj_streamer::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    joint_state_sub_ = nh_.subscribe("robot/joint_states", 1, &Baxter_traj_streamer::jointStatesCb,this);  
    // add more subscribers here, as needed
}

//member helper function to set up publishers;
void Baxter_traj_streamer::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    joint_cmd_pub_right_ = nh_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1, true); 
    joint_cmd_pub_left_  = nh_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1, true);
    right_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("right_arm_joint_path_command", 1); 
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}

void Baxter_traj_streamer::jointStatesCb(const sensor_msgs::JointState& js_msg) {
    // copy right-arm angles to member vec
    for (int i=0;i<7;i++)
    {
        // should do this better; manually remap from joint_states indices to right-arm joint angles
        q_vec_right_arm[6] = js_msg.position[14]; //w2
        q_vec_right_arm[5] = js_msg.position[13]; //w1
        q_vec_right_arm[4] = js_msg.position[12]; //w0
        q_vec_right_arm[3] = js_msg.position[9]; //e1
        q_vec_right_arm[2] = js_msg.position[8]; //e0  
        q_vec_right_arm[1] = js_msg.position[11]; //s1
        q_vec_right_arm[0] = js_msg.position[10]; //s0           
    }
}  

Vectorq7x1 Baxter_traj_streamer::get_qvec_right_arm() {
    return q_vec_right_arm;
}

double Baxter_traj_streamer::transition_time(Vectorq7x1 dqvec) {
    double t_max = fabs(dqvec[0])/qdot_max_vec[0];
    cout<<"qdot max: "<<qdot_max_vec.transpose()<<endl;
    double ti;
    for (int i=1;i<7;i++) {
        ti = fabs(dqvec[i])/qdot_max_vec[i];
        if (ti>t_max) t_max= ti;
    }
    return t_max;
}

double Baxter_traj_streamer::transition_time(Eigen::VectorXd dqvec) {
    double t_max = fabs(dqvec[0])/qdot_max_vec[0];
    cout<<"qdot max: "<<qdot_max_vec.transpose()<<endl;
    double ti;
    for (int i=1;i<7;i++) {
        ti = fabs(dqvec[i])/qdot_max_vec[i];
        if (ti>t_max) t_max= ti;
    }
    return t_max;
}

//command robot to move to "qvec" using a trajectory message, sent via ROS-I
void Baxter_traj_streamer::stuff_trajectory( std::vector<Vectorq7x1> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
    
    trajectory_point1.positions.clear(); 
    trajectory_point2.positions.clear(); 
    
    for (int i=0;i<7;i++) { //pre-size these vectors, so can access w/ indices
        trajectory_point1.positions.push_back(0.0);
        trajectory_point2.positions.push_back(0.0);
    }
    new_trajectory.points.clear();
    new_trajectory.joint_names.clear(); 
    
    new_trajectory.joint_names.push_back("right_s0");
    new_trajectory.joint_names.push_back("right_s1");
    new_trajectory.joint_names.push_back("right_e0");
    new_trajectory.joint_names.push_back("right_e1");
    new_trajectory.joint_names.push_back("right_w0");
    new_trajectory.joint_names.push_back("right_w1");
    new_trajectory.joint_names.push_back("right_w2");

    new_trajectory.header.stamp = ros::Time::now();  
    Vectorq7x1 q_start,q_end,dqvec;
    double del_time;
    double net_time=0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];    
    for (int i=0;i<7;i++) { //pre-size these vectors, so can access w/ indices
        trajectory_point1.positions[i]=q_start[i];
    }    
    trajectory_point1.time_from_start =    ros::Duration(net_time); 
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    //add the rest of the points from qvecs
    for (int iq=1;iq<qvecs.size();iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end-q_start;
        cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);

        net_time+= del_time;
        ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i=0;i<7;i++) { //pre-size these vectors, so can access w/ indices
            trajectory_point1.positions[i]=q_end[i];
        }    
        trajectory_point1.time_from_start =    ros::Duration(net_time); 
        new_trajectory.points.push_back(trajectory_point1);        
    }

}

// this version uses vector of VectorXd for input
void Baxter_traj_streamer::stuff_trajectory( std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
    
    trajectory_point1.positions.clear(); 
    

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear(); 
    
    new_trajectory.joint_names.push_back("right_s0");
    new_trajectory.joint_names.push_back("right_s1");
    new_trajectory.joint_names.push_back("right_e0");
    new_trajectory.joint_names.push_back("right_e1");
    new_trajectory.joint_names.push_back("right_w0");
    new_trajectory.joint_names.push_back("right_w1");
    new_trajectory.joint_names.push_back("right_w2");

    new_trajectory.header.stamp = ros::Time::now();  
    Eigen::VectorXd q_start,q_end,dqvec;
    double del_time;
    double net_time=0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];    
    //trajectory_point1.positions = qvecs[0];
 
    trajectory_point1.time_from_start =    ros::Duration(net_time); 
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    //add the rest of the points from qvecs
    for (int i=0;i<7;i++) { //pre-size these vectors, so can access w/ indices
        trajectory_point1.positions.push_back(0.0);
    }    
    for (int iq=1;iq<qvecs.size();iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end-q_start;
        cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);

        net_time+= del_time;
        ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i=0;i<7;i++) { //pre-size these vectors, so can access w/ indices
            trajectory_point1.positions[i]=q_end[i];
        }   
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start =    ros::Duration(net_time); 
        new_trajectory.points.push_back(trajectory_point1);        
    }        
        
}    

// command a single pose: cmd_pose_right(Vectorq7x1 qvec );
void Baxter_traj_streamer::cmd_pose_right(Vectorq7x1 qvec ) {
    //member var right_cmd_ already has joint names populated
    for (int i=0;i<7;i++) {
        right_cmd_.command[i]=qvec[i];
    }
    joint_cmd_pub_right_.publish(right_cmd_);
}

//publish a trajectory; to be processed by a separate interpolating joint commander
void Baxter_traj_streamer::pub_right_arm_trajectory(trajectory_msgs::JointTrajectory &new_trajectory) {
    right_traj_pub_.publish(new_trajectory);
    cout<<"publishing trajectory with npts = "<<new_trajectory.points.size()<<endl;
}

// get current arm joint angles, and send some trajectories with these angles, just to get listener warmed up
void Baxter_traj_streamer::pub_right_arm_trajectory_init() {
    std::vector<Vectorq7x1> qvecs;
    trajectory_msgs::JointTrajectory new_trajectory;
    Vectorq7x1 q_snapshot = q_vec_right_arm;
    qvecs.push_back(q_vec_right_arm);
    qvecs.push_back(q_vec_right_arm);    
    stuff_trajectory(qvecs, new_trajectory);    
    right_traj_pub_.publish(new_trajectory);
    cout<<"publishing trajectory with npts = "<<new_trajectory.points.size()<<endl;
}