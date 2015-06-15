// reachability_from_above_v2.cpp: modified reachability from above to display rviz markers
// wsn, June 2015; Baxter reachability, modified from ABB IRB120 reachability 
// compute reachability, w/ z_tool_des = [0,0,-1] = -z_axis w/rt torso
// w/ robot mounted as is, z_base points up
// Fix the value of z, --> const height; scan over x and y


#include <baxter_kinematics/baxter_kinematics.h>
#include <visualization_msgs/Marker.h>
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
    ros::NodeHandle nh;
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
    
    //marker stuff...
   ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "reachability_from_above_marker", 0 );            
    visualization_msgs::Marker marker;  // instantiate a marker object
    geometry_msgs::Point point;  // points will be used to specify where the markers go
    marker.header.frame_id = "/torso"; //base_link"; // select the reference frame 
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    // use SPHERE if you only want a single marker
    marker.type = visualization_msgs::Marker::SPHERE_LIST; //SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    // if just using a single marker, specify the coordinates here, like this:

    //marker.pose.position.x = 0.4;  
    //marker.pose.position.y = -0.4;
    //marker.pose.position.z = 0;
    //ROS_INFO("x,y,z = %f %f, %f",marker.pose.position.x,marker.pose.position.y, marker.pose.position.z);    
    // otherwise, for a list of markers, put their coordinates in the "points" array, as below
    
    //whether a single marker or list of markers, need to specify marker properties
    // these will all be the same for SPHERE_LIST
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    Eigen::Affine3d a_tool_des; // expressed in torso frame
    
    a_tool_des.linear() = R_des;
    //a_tool_des.translation() << x_des,0,0;
    double z_des;
    while (ros::ok()) {
        std::cout<<std::endl<<"enter z_des: ";
        std::cin>>z_des;
        marker.points.clear();
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
            //cout<<"trying: "<<p[0]<<","<<p[1]<<","<<p[2]<<":  "<<endl;
            nsolns = baxter_IK_solver.ik_solve_approx_wrt_torso(a_tool_des,q_solns);
            //std::cout<<"nsolns= "<<nsolns<<endl;
            /*
            if (nsolns>0) {
                int ans;
                cout<<"enter 1:";
                cin>>ans;
            }
             * */
            nsolns_matrix[ix][iy]=nsolns;
            iy++;
            if (nsolns>0) {
                point.x = x_des;
                point.y = y_des;
                point.z = z_des;
                marker.points.push_back(point);
                vis_pub.publish( marker );
            }
            
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
    
    for (int k=0; k<10; k++) {
        vis_pub.publish( marker );     
        ros::Duration(0.01).sleep();
        //ROS_INFO("publishing...");
        ros::spinOnce();
    }    
    }
    return 0;
}
