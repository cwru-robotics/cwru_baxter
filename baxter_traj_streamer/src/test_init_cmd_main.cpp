// wsn, June, 2015
// test main to check joint-command initializations--and test mapping joint indices correctly

#include <joint_space_planner.h>
#include <baxter_kinematics/baxter_kinematics.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <string.h>
//#include <baxter_kinematics.h>

#include <stdio.h>      /* printf, scanf, puts, NULL */




int main(int argc, char **argv) {
    Eigen::VectorXd weights;
    //Eigen::VectorXd vec2;
    //Eigen::VectorXd vec3; 
    //Eigen::VectorXd arg1,arg2;   
    ros::init(argc, argv, "test_init_cmd_main"); //standard ros init with node name string
    ros::NodeHandle nh; //standard ros node handle    

    
    //pass in nh, so constructor can set up subscribers, publishers and services
    Baxter_traj_streamer baxter_traj_streamer(&nh);
    cwru_srv::simple_bool_service_message srv;

    ros::ServiceClient traj_interp_stat_client = nh.serviceClient<cwru_srv::simple_bool_service_message>("trajInterpStatusSvc");

    ROS_INFO("warming up the command listener...");
    for (int i = 0; i < 10; i++) { //send traj 10x
        cout << "invoking init..." << endl;
        baxter_traj_streamer.pub_right_arm_trajectory_init();   
    }
    Vectorq7x1 r_arm_jnt_vals;
    while(ros::ok()) {
        r_arm_jnt_vals =  baxter_traj_streamer.get_qvec_right_arm();
        cout<<"r arm jnts: "<<r_arm_jnt_vals.transpose()<<endl;
        for (int i=0;i<5;i++) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }

    return 0;

}
