// get_and_save_r_arm_pose: 
// wsn, April, 2016
// uses traj_interpolator_as to get r_arm joint angles; move merry's arm; enter "1" to save to disk, enter "0" to quit the pgm
// saves joint angles to file "merry_r_arm_angs.txt"

#include<ros/ros.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <iostream>
#include <fstream>

using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector


int main(int argc, char** argv) {
        ros::init(argc, argv, "traj_action_client_node"); // name this node 
        ros::NodeHandle nh; //standard ros node handle        
        int g_count = 0;
                int ans;
    Vectorq7x1 q_pre_pose;
    //q_in << 0, 0, 0, 0, 0, 0, 0;  
    q_pre_pose<< -0.907528, -0.111813,   2.06622,    1.8737,    -1.295,   2.00164,  -2.87179;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;
       
  ofstream outfile;
  outfile.open ("merry_r_arm_angs.txt");

        cout<<"instantiating a traj streamer"<<endl; // enter 1:";
        //cin>>ans;
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    cout<<"warming up callbacks..."<<endl;
    for (int i=0;i<100;i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }
    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm();  
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;    
    q_in_vecxd = q_vec_right_arm; // start from here;

    int trigger=2;
    int npts = 0;
    while (trigger>0) {
        cout<<"enter 1 for a snapshot, 0 to finish: ";
        cin>>trigger;
        if (trigger==1) {//take snapshot
            trigger=2; // reset trigger
            npts++;
    for (int i=0;i<10;i++) {

        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }
    cout<<"getting current right-arm pose:"<<endl;
    q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm();  
    cout<<"r_arm state:"<<q_vec_right_arm.transpose()<<endl;    
    q_in_vecxd = q_vec_right_arm; // start from here;    
    //save to disk:
      //outfile << q_in_vecxd.transpose()<<endl;
    outfile << q_in_vecxd[0]<<", "   << q_in_vecxd[1]<<", "<< q_in_vecxd[2]<<", "<< q_in_vecxd[3]<<", "<< q_in_vecxd[4]<<", "<< q_in_vecxd[05]<<", "<< q_in_vecxd[6]<<", "<<npts<<endl;
        }
    }
        //}
  outfile.close();
    return 0;
}

