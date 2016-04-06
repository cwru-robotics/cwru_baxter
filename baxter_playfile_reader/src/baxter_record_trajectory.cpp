// baxter_record_trajectory: 
// wsn, April, 2016
// get Merry ready; start this program; enter 1 to start recording
// saves trajectory to file "merry_r_arm_traj.jsp"

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

    Vectorq7x1 q_pre_pose;
    //q_in << 0, 0, 0, 0, 0, 0, 0;  
    q_pre_pose << -0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, -2.87179;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;
    double dt_samp = 0.2; // sample at 5Hz
    double dt_spin = 0.01;
    double dt_inc = 0.0;
    double arrival_time = 0.0;

    ofstream outfile;
    outfile.open("merry_r_arm_traj.jsp");

    cout << "instantiating a traj streamer" << endl; // enter 1:";
    //cin>>ans;
    Baxter_traj_streamer baxter_traj_streamer(&nh); //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    cout << "warming up callbacks..." << endl;
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }
    cout << "getting current right-arm pose:" << endl;
    q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm();
    cout << "r_arm state:" << q_vec_right_arm.transpose() << endl;
    q_in_vecxd = q_vec_right_arm; // start from here;

    //double time = 0.0;
    int ans;
    cout << "enter 1 to start capturing, then move right arm in desired trajectory: ";
    cin>>ans;
    while (ros::ok()) {

        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(dt_spin).sleep();
        dt_inc += dt_spin;
        if (dt_inc >= dt_samp) {
            arrival_time += dt_samp;
            dt_inc = 0.0;




            cout << "getting current right-arm pose:" << endl;
            q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm();
            cout << "r_arm state:" << q_vec_right_arm.transpose() << endl;
            q_in_vecxd = q_vec_right_arm; // start from here;    
            //save to disk:
            //outfile << q_in_vecxd.transpose()<<endl;
            outfile << q_in_vecxd[0] << ", " << q_in_vecxd[1] << ", " << q_in_vecxd[2] << ", " << q_in_vecxd[3] << ", " << q_in_vecxd[4] << ", " << q_in_vecxd[05] << ", " << q_in_vecxd[6] << ", " << arrival_time << endl;
        }
    }
    //}
    outfile.close();
    return 0;
}

