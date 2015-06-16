// cart_path_planner_lib: 
// wsn, June, 2015
// a library of arm-motion planning functions
#include <cartesian_moves/cart_path_planner_lib.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "test_cart_path_planner_lib");
    ros::NodeHandle nh; //standard ros node handle    
 
      ROS_INFO("instantiating a cartesian planner object: ");
    
    CartTrajPlanner cartTrajPlanner;
    
    ROS_INFO("instantiated the planner");  
    
    Vectorq7x1 q_pre_pose;
    Eigen::Vector3d p_des;
    p_des<<0.5, 0, 0.4; //0.4, -0.6, 0.4;
    Eigen::Affine3d a_tool_start,a_tool_end;   
    Eigen::Matrix3d R_gripper_down = cartTrajPlanner.get_R_gripper_down();
    a_tool_end.translation() = p_des;
    cout<<"p_des: "<<p_des.transpose()<<endl;
    cout<<"R_des: "<<endl;
    cout<<R_gripper_down<<endl;
    
    q_pre_pose<< -0.907528, -0.111813,   2.06622,    1.8737,    -1.295,   2.00164,  -2.87179;
    std::vector<Eigen::VectorXd> optimal_path;
    bool valid_path;
    
    valid_path = cartTrajPlanner.cartesian_path_planner_wrist(q_pre_pose,a_tool_end,optimal_path);
    int npts = optimal_path.size();
    cout<<"path npts = "<<npts<<endl;
    for (int i=0;i<npts;i++) {
        cout<<optimal_path[i].transpose()<<endl;
    }
    //ros::spin(); // stop here;    

    return 0;
}
