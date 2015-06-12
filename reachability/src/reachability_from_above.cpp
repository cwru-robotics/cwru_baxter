// reachability_from_above.cpp
// wsn, June 2015; Baxter reachability, modified from ABB IRB120 reachability 
// compute reachability, w/ z_tool_des = [0,0,-1] = -z_axis w/rt torso
// w/ robot mounted as is, z_base points up
// Fix the value of z, --> const height; scan over x and y


#include <baxter_kinematics/baxter_kinematics.h>
//include the following if/when want to plan a joint-space path and execute it
//#include <baxter_traj_streamer/baxter_traj_streamer.h>
//#include <joint_space_planner.h>

#include <string.h>
#include <stdio.h>      /* printf, scanf, puts, NULL */
//#include <stdlib.h>     /* srand, rand */
//#include <time.h>       /* time */
using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_reachability_from_above");
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    Vectorq7x1 q_in;
    q_in<<0,0,0,0,0,0,0;

    Baxter_fwd_solver baxter_fwd_solver; //instantiate a forward-kinematics solver
    Baxter_IK_solver baxter_IK_solver; // instantiate an IK solver

    b_des<<0,0,-1;
    n_des<<1,0,0;
    t_des = b_des.cross(n_des);
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;
    
    /*
    //need tool transform to convert between DH and URDF frames
    Eigen::Affine3d a_tool;
    a_tool.linear() =R_des;
    a_tool.translation() << 0.3,
            0.0,
            0.3;

    
    Eigen::Affine3d A_fwd_DH = irb120_fwd_solver.fwd_kin_solve(q_in); //fwd_kin_solve
    // rotate DH frame6 to reconcile with URDF frame7:
    //Eigen::Affine3d A_fwd_URDF = A_fwd_DH*a_tool;
    std::cout << "q_in: " << q_in.transpose() << std::endl;
    std::cout << "A rot: " << std::endl;
    std::cout << A_fwd_DH.linear() << std::endl;
    std::cout << "A origin: " << A_fwd_DH.translation().transpose() << std::endl;
            
    int nsolns = ik_solver.ik_solve(A_fwd_DH);
    std::vector<Vectorq6x1> q6dof_solns;
    Vectorq6x1 qsoln;
    ik_solver.get_solns(q6dof_solns);
    std::cout<<"IK solns: "<<std::endl;
    for (int i=0;i<nsolns;i++) {
       std::cout<<(q6dof_solns[i]).transpose();
    }
    */
    Eigen::Affine3d a_tool_des; // expressed in torso frame
    
    a_tool_des.linear() = R_des;
    //a_tool_des.translation() << x_des,0,0;
    double z_des;
    while (ros::ok()) {
        std::cout<<std::endl<<"enter z_des: ";
        std::cin>>z_des;

    p[2]=z_des;


    std::cout << "====  Baxter kinematics solver ====" << std::endl;
    int ans = 1;
    int nsolns;
    bool reachable_proposition;
    int nx = 0;
    int ny = 0;    
    //set search range and resolution:
    double x_min = -1.0;
    double x_max = 1.0;
    double y_min = -1.0;
    double y_max = 1.0;
    double dx_des = 0.1;
    double dy_des = 0.1;
    
    //find how many values nx,ny in search range
    for (double x_des = x_min; x_des<x_max; x_des+=dx_des) nx++;
    for (double y_des=y_min; y_des<y_max; y_des+= dy_des) ny++;
    cout<<"discretized search into nx = "<<nx<<", ny= "<<ny<<endl;
    int nsolns_matrix[nx][ny];
        
    std::vector<Vectorq7x1> q_solns;  
    int ix=0;
    int iy=0;

    for (double x_des = x_min; x_des<x_max; x_des+=dx_des) {
        std::cout<<std::endl;
        std::cout<<"x="<< round(x_des*10)<<"  ";
       
       for (double y_des=y_min; y_des<y_max; y_des+= dy_des) {
           p[0]=x_des;
           p[1] = y_des;
            a_tool_des.translation()=p;
            nsolns = baxter_IK_solver.ik_solve_approx_wrt_torso(a_tool_des,q_solns);
            std::cout<<nsolns;
            nsolns_matrix[ix][iy]=nsolns;
            iy++;
            
        }
        ix++;
        iy=0;
    }
    //display the matrix:
    cout<<endl;
    cout<<"x/y: ";
    int xval,yval;    
    for (iy=ny-1;iy>=0;iy--) {
        yval = round((y_min+iy*dy_des)*100);
        if (abs(yval)<100) cout<<" ";
        if (abs(yval)<10)  cout<<" ";
        if (yval>-1)  cout<<" ";
        cout<<yval;        
    }
    cout<<endl;

    for (ix=nx-1;ix>=0;ix--) {
        // print out the x value as a column...in cm
        xval = round((x_min+ix*dx_des)*100);
        if (abs(xval)<100) cout<<" ";
        if (abs(xval)<10)  cout<<" ";
        if (xval>-1)  cout<<" ";
        cout<<xval<<"  ";
        for (iy=ny-1;iy>=0;iy--) {
            nsolns = nsolns_matrix[ix][iy];
            if (nsolns<10) 
                cout<<" ";
            if (nsolns<100) 
                cout<<" ";       
            cout<<nsolns<<" ";
        }
        cout<<endl;
    }
    }
    return 0;
}
