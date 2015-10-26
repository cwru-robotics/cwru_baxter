// process_pcd_save.cpp: wsn, April, 2015
// example code to acquire a pointcloud from a topic, and save a snapshot to disk
// as a PCD file.

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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
//#include <pcl/common/impl/centroid.hpp>


using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;

pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclKinect(new PointCloud<pcl::PointXYZ>);

bool g_got_cloud = false; 
bool g_saved_file = false;

tf::TransformListener *g_tfListenerPtr; //create a TransformListener to listen for tf's and assemble them

//here is a function that transforms a cloud of points into an alternative frame;
//supply a cloud of points and supply an Eigen::Affine3f; will populate the outputCloud
void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Affine3f A,  PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = inputCloud->width;
    outputCloud->height = inputCloud->height;
    int npts = inputCloud->points.size();
    cout << "transforming npts = " << npts << endl;
    outputCloud->points.resize(npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = A * inputCloud->points[i].getVector3fMap(); 
    }    
}

// a utility fnc to convert tf::Transform type into an Eigen::Affine3f
// "f" is needed for use with point clouds, since "floats" not "doubles" are more practical, considering
// the expected (limited) resolution of the sensor, as well as the large size of point clouds
Eigen::Affine3f transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3f e;
    // treat the Eigen::Affine as a 4x4 matrix:
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i]; //copy the origin from tf to Eigen
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j]; //and copy 3x3 rotation matrix
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}


void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    tf::StampedTransform tf_sensor_frame_to_torso_frame; //need objects of this type to hold tf's
    Eigen::Affine3f  A;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>); // holder for processed point clouds
    
    
    cout<<"callback from kinect pointcloud pub"<<endl;
    pcl::fromROSMsg(*cloud, *g_pclKinect); //g_pclKinect is global--so "main" could access the pointcloud as well, if desired
                                            // better--make this a class member
    ROS_INFO("kinectCB %d * %d points", (int) g_pclKinect->width, (int) g_pclKinect->height);
    pcl::io::savePCDFileASCII ("snapshot.pcd", *g_pclKinect);
    ROS_INFO("saved PCD image consisting of %d data points",(int) g_pclKinect->points.size ()); 
    
    ROS_INFO("get current transform from sensor frame to torso frame: ");
    g_tfListenerPtr->lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
    ROS_INFO("convert to Affine: ");
    A= transformTFToEigen(tf_sensor_frame_to_torso_frame);
    ROS_INFO("resulting Affine: rotation, translation");
    cout<<A.linear()<<endl;
    cout<<"translation: "<<A.translation().transpose()<<endl;
    ROS_INFO("transform pointcloud to world frame");
    transform_cloud(g_pclKinect, A, cloud_transformed); // transform the entire point cloud  
    ROS_INFO("saving transformed cloud as snapshot_wrt_torso.pcd");
    pcl::io::savePCDFileASCII ("snapshot_wrt_torso.pcd", *cloud_transformed);
    
    g_got_cloud=true; //cue to "main" that callback received and saved a pointcloud
    geometry_msgs::TransformStamped transform;
}

int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "pcd_save");
    ros::NodeHandle nh;
    ros::Rate rate(2);
    int ans;
    tf::StampedTransform tf_sensor_frame_to_torso_frame; //need objects of this type to hold tf's
    tf::TransformListener  tf_listener;
    g_tfListenerPtr = &tf_listener; // let's make the tf_listener global, via a pointer, so callback can use it.
                                    // better would be to make it a class member
    //let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
   bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and world...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll

    
    // Subscribers
    cout<<"subscribing to kinect data"<<endl;
    cout<<"enter 1 to continue: ";
    cin>>ans;
    ros::Subscriber getPCLPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1, kinectCB);

    cout<<"starting main loop"<<endl;
    
    while (ros::ok()) {       
        if (g_got_cloud) {
                //pcl::io::savePCDFileASCII ("snapshot.pcd", *g_pclKinect);
                //ROS_INFO("saved PCD image consisting of %d data points",(int) g_pclKinect->points.size ());
                //std::cerr << "Saved " << g_pclKinect->points.size () << " data points to snapshot.pcd." << std::endl;
                //std::cout<<"frame: "<<g_pclKinect->header.frame_id<<std::endl;                
                return 0; // or bail out now
        }
        else 
            cout<<"waiting for got_cloud..."<<endl;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}