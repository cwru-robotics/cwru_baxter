// raster_move_dy.cpp
// wsn, June 2015; use for test of Baxter reachability
// command gripper to point down, fixed value of x (w/rt torso) and sweep arm along y
// use range of values predicted reachable, via reachability_from_above: 


#include <baxter_kinematics/baxter_kinematics.h>
//include the following if/when want to plan a joint-space path and execute it
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <joint_space_planner.h>

#include <string.h>
#include <stdio.h>      /* printf, scanf, puts, NULL */
//#include <stdlib.h>     /* srand, rand */
//#include <time.h>       /* time */
using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "baxter_raster_dy");
    ros::NodeHandle nh; //standard ros node handle    
    
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    Vectorq7x1 q_in;
    q_in<<0,0,0,0,0,0,0;

    Baxter_fwd_solver baxter_fwd_solver; //instantiate a forward-kinematics solver
    Baxter_IK_solver baxter_IK_solver; // instantiate an IK solver
    ROS_INFO("main: instantiating an object of type Baxter_traj_streamer");
    Baxter_traj_streamer baxter_traj_streamer(&nh);  //instantiate a Baxter_traj_streamer object and pass in pointer to nodehandle for constructor to use  

    b_des<<0,0,-1;
    n_des<<1,0,0;
    t_des = b_des.cross(n_des);
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;
    
    Eigen::Affine3d a_tool_des; // expressed in torso frame
    
    int nlayers;
    std::vector<std::vector<Eigen::VectorXd> > path_options;    
        path_options.clear();
    std::vector<Eigen::VectorXd>  single_layer_nodes;  
    Eigen::VectorXd  node; 

    a_tool_des.linear() = R_des;
    //a_tool_des.translation() << x_des,0,0;
    double z_des = 0.0;

    p[2]=z_des;

    int nsolns;
    bool reachable_proposition;
    int nx = 0;
    int ny = 0;    
    //set search range and resolution: only vary y value   
    double x_min = 0.4;
    double x_max = 0.4;
    double y_min = -0.9;
    double y_max = 0.3;
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
            single_layer_nodes.clear();
            single_layer_nodes.resize(nsolns);
            for (int isoln=0;isoln<nsolns;isoln++) {
                // this is annoying: can't treat std::vector<Vectorq7x1> same as std::vector<Eigen::VectorXd> 
                node = q_solns[isoln];
                single_layer_nodes[isoln] = node;
            }

            path_options.push_back(single_layer_nodes); 
            cout<<"y = "<<y_des<<"; pushed "<<nsolns<<"solutions onto path_options"<<endl;
        }
        ix++;
        iy=0;
    }
    //display the matrix:
    cout<<endl;
    cout<<"x/y: ";
    int xval,yval;    
    for (int iy=ny-1;iy>=0;iy--) {
        yval = round((y_min+iy*dy_des)*100);
        if (abs(yval)<100) cout<<" ";
        if (abs(yval)<10)  cout<<" ";
        if (yval>-1)  cout<<" ";
        cout<<yval;        
    }
    cout<<endl;

    for (int ix=nx-1;ix>=0;ix--) {
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

    return 0;
}
