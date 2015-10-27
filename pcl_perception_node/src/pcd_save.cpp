// pcd_save.cpp: wsn, October, 2015
// example code to acquire a pointcloud from a topic, and save a snapshot to disk
// as a PCD file.  No transformation is done, so the data is with respect to the sensor frame

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <geometry_msgs/PointStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
//#include <pcl/common/impl/centroid.hpp>


using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;

//a pointer to a pointcloud object.  By making this global, "main" could access it, if desired
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclKinect(new PointCloud<pcl::PointXYZ>);

bool g_got_cloud = false; // cue to "main"; callback will inform main when a pointcloud is received and saved


void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {    
    cout<<"callback from kinect pointcloud pub"<<endl;
    pcl::fromROSMsg(*cloud, *g_pclKinect); //g_pclKinect is global--so "main" could access the pointcloud as well, if desired
                                            // better would be to make this a class member
    ROS_INFO("kinectCB %d * %d points", (int) g_pclKinect->width, (int) g_pclKinect->height);
    pcl::io::savePCDFileASCII ("snapshot.pcd", *g_pclKinect);
    ROS_INFO("saved PCD image consisting of %d data points to snapshot.pcd",(int) g_pclKinect->points.size ()); 
    
    g_got_cloud=true; //cue to "main" that callback received and saved a pointcloud

}

int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "pcd_save");
    ros::NodeHandle nh;
    ros::Rate rate(2);
 
    // Subscribers
    cout<<"subscribing to kinect data"<<endl;

    ros::Subscriber getPCLPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1, kinectCB);

    cout<<"starting main loop"<<endl;
    
    while (ros::ok()) {       
        if (g_got_cloud) {              
                return 0; // or bail out now
        }
        else 
            cout<<"waiting for got_cloud..."<<endl;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}