// baxter_kinematics_test_jacobian_main.cpp
// wsn, May 2015
// test function for baxter_kinematics library
// this version subscribes to actual Baxter joint angles (joint_states) and
// uses home-brew fwd_kin to compute shoulder, elbow, wrist and tool-flange origins
// Does perturbations to see if J3x3 is OK (using only s1, q_humerus, q_elbow)



#include <baxter_kinematics/baxter_kinematics.h>
 #include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

using namespace std;

 Vectorq7x1 q_vec_right_arm,q_in;
void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    // copy right-arm angles to global vec
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
    
int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_kinematics_test_jacobian_main");
    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub = nh.subscribe("robot/joint_states", 1, jointStatesCb);
    q_vec_right_arm<< 0,0,0,0,0,0,0;
    q_in << 0,0,0,0,0,0,0;
    
    Baxter_fwd_solver baxter_fwd_solver; //instantiate a forward-kinematics solver
    Baxter_IK_solver baxter_IK_solver;  // instantiate an IK solver
    
    Eigen::Affine3d A_fwd_DH;
    Eigen::Matrix4d A_wrist1,A_elbow,A_shoulder,A_flange;    
    Eigen::Matrix3d R_hand,Jw3x3,Jw3x3_inv;
    

    Eigen::Matrix3d R_flange_wrt_right_arm_mount;
    Eigen::Vector3d wrist_pt_wrt_right_arm_frame1,w_perturbed,dw,dq,Jdq,w_approx;
    Eigen::Vector3d w_err;
    Eigen::Vector3d dq123;
    Vectorq7x1 q_snapshot,q_perturbed;
    double eps1=0.001;
    double eps2= 0.001;
    double eps3 = 0.001;
    int ans;
         dq(0) = eps1;
        dq(1) = eps2;
        dq(2) = eps3;
    Eigen::Affine3d affine_flange;   
    Eigen::Affine3d affine_flange_perturbed;  
    std::vector<Vectorq7x1> q_solns_123;  
    Vectorq7x1 approx_soln_123, precise_soln_123;
    bool reachable;
    while (ros::ok()) 
    {
        cout<<"enter 1 to initiate: "; //pause here to allow user to move joint angles to desired test pose
        cin>>ans;
        for (int i=0;i<10;i++) { // spin to get current joint angles
            ros::spinOnce();
            ros::Duration(0.1).sleep(); 
        }     
        q_snapshot = q_vec_right_arm;
        q_perturbed = q_snapshot;
        q_perturbed(1)+= eps1;
        q_perturbed(2)+= eps2;
        q_perturbed(3)+= eps3;
        dq(0) = eps1;
        dq(1) = eps2;
        dq(2) = eps3;
        
        cout<<"qvec right arm: "<<endl;
        cout<< q_snapshot.transpose() <<endl;
        affine_flange = baxter_fwd_solver.fwd_kin_flange_wrt_r_arm_mount_solve(q_snapshot); //fwd_kin_solve 

       // wrist_pt_wrt_right_arm_frame1 = baxter_IK_solver.wrist_frame1_from_tool_wrt_rarm_mount(affine_flange,q_snapshot);
        wrist_pt_wrt_right_arm_frame1 = baxter_fwd_solver.get_wrist_coords_wrt_frame1(q_snapshot);
        cout<< "w w/rt frame1: "<<wrist_pt_wrt_right_arm_frame1.transpose()<<endl;
        
        cout<<"qvec perturbed: "<<endl;
        cout<< q_perturbed.transpose() <<endl;
        
        affine_flange_perturbed = baxter_fwd_solver.fwd_kin_flange_wrt_r_arm_mount_solve(q_perturbed); //fwd_kin_solve

        w_perturbed = baxter_fwd_solver.get_wrist_coords_wrt_frame1(q_perturbed);
        cout<< "perturbed w w/rt frame1: "<<w_perturbed.transpose()<<endl;   
        
        dw = w_perturbed - wrist_pt_wrt_right_arm_frame1;
        cout<<"dw: "<<dw.transpose()<<endl;
        
        Jw3x3 = baxter_fwd_solver.get_wrist_Jacobian_3x3(q_snapshot[1], q_snapshot[2], q_snapshot[3], q_snapshot[4]);
        cout<<"Jw3x3: "<<endl;
        cout<<Jw3x3<<endl;
        Jdq = Jw3x3*dq;
        cout<<"Jdq = "<<Jdq.transpose()<<endl;
        
        
        double q_s0 = q_snapshot[0];
        double w_err_norm;
        reachable= baxter_IK_solver.compute_q123_solns(affine_flange, q_s0, q_solns_123);
        if (reachable) {
          approx_soln_123 = q_solns_123[0]; //first approx; pick the first soln: 
          cout<<"TRYING NEW FNC precise_soln_q123"<<endl;
   
          w_err_norm=baxter_IK_solver.precise_soln_q123(affine_flange,approx_soln_123, precise_soln_123);
          cout<<"precise soln yielded w_err_norm = "<<w_err_norm<<endl;
 /*               
          Jw3x3 = baxter_fwd_solver.get_wrist_Jacobian_3x3(approx_soln_123[1], approx_soln_123[2], approx_soln_123[3], approx_soln_123[4]);          
        cout<<"Jw3x3: "<<endl;
        cout<<Jw3x3<<endl;
        cout<<"det = "<<Jw3x3.determinant()<<endl;
        Jw3x3_inv =  Jw3x3.inverse();
        cout<<"Jw3x3_inv: "<<endl;
        cout<<Jw3x3_inv<<endl;        
          
        int jiter=0;
        double w_err_norm = 1.0;
        while ((jiter<MAX_JINV_ITERS)&&(w_err_norm>W_ERR_TOL))
        {
            // iterate w/ Jacobian to find more precise soln
            w_approx = baxter_fwd_solver.get_wrist_coords_wrt_frame1(approx_soln_123);
            w_err = wrist_pt_wrt_right_arm_frame1-w_approx;
            w_err_norm = w_err.norm();
            cout<<"iter "<<jiter<<"; w_err_norm = "<<w_err_norm<< "; w_err =  "<<w_err.transpose()<<endl;
            dq123 = Jw3x3_inv*w_err;
            if (dq123.norm()> DQ_ITER_MAX) {
                dq123/=DQ_ITER_MAX; //protect against numerical instability
            }
            cout<<"dq123: "<<dq123.transpose()<<endl;
            for (int i=1;i<4;i++) {
                approx_soln_123[i]+=dq123[i-1];
            }
            jiter++;
          }
  * */ 
        }
    

    ros::Duration(2).sleep(); //sleep for 2 seconds
    ros::spinOnce();

    }
    return 0;
}
