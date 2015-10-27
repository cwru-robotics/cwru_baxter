// cwru_pcl_utils.h header file; doxygen comments follow //
/// wsn; Oct, 2015.  
/// Include this file in "cwru_pcl_utils.cpp", and in any main that uses this library.
///This class provides example functions using the Point Cloud Library to operate
/// on point-cloud data

#ifndef CWRU_PCL_UTILS_H_
#define CWRU_PCL_UTILS_H_

#include<ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <sensor_msgs/PointCloud2.h> //useful ROS message types
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_listener.h>  // transform listener headers
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>  //point-cloud library headers; likely don't need all these
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

using namespace std;  //just to avoid requiring std::, Eigen:: ...
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;

// define a class, including a constructor, member variables and member functions
class CwruPclUtils
{
public:
    CwruPclUtils(ros::NodeHandle* nodehandle); //constructor

     // insert doxygen documentation of member fncs;  run "doxywizard" to create documentation

     // this fnc is a copy of plane-fitter from exmaple_ros_library
     /**provide an array of 3-D points (in columns), and this function will use and eigen-vector approach to find the best-fit plane
     * It returns the plane's normal vector and the plane's (signed) distance from the origin.
     * @param points_array input: points_array is a matrix of 3-D points to be plane-fitted; coordinates are in columns
     * @param plane_normal output: this function will compute components of the plane normal here
     * @param plane_dist output: scalar (signed) distance of the plane from the origin
     */
    void fit_points_to_plane(Eigen::MatrixXd points_array, 
        Eigen::Vector3d &plane_normal, 
        double &plane_dist); 

// 
// 
// 
    /**a utility fnc to convert tf::Transform type into an Eigen::Affine3f
     * Affine type "f" is needed for use with point clouds, since "floats" not "doubles" are more practical, considering
     * the expected (limited) resolution of the sensor, as well as the large size of point clouds
     * @param t  [in] provide a transform, e.g. per:
     *     g_tfListenerPtr->lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
     * @return an Eigen Affine object, A, such that point_in_new_frame = A*point_in_original_frame
     */    
    Eigen::Affine3f transformTFToEigen(const tf::Transform &t);
    void transform_kinect_cloud(Eigen::Affine3f A);
    void reset_got_kinect_cloud() {got_kinect_cloud_= false;};
    bool get_got_kinect_cloud() { return got_kinect_cloud_; };
    void save_kinect_snapshot() {    pcl::io::savePCDFileASCII ("kinect_snapshot.pcd", *pclKinect_ptr_);};
    void save_transformed_kinect_snapshot() { pcl::io::savePCDFileASCII ("xformed_kinect_snapshot.pcd", *pclTransformed_ptr_);};

private:
    ros::NodeHandle nh_; 
    // some objects to support subscriber, service, and publisher
    ros::Subscriber pointcloud_subscriber_; //use this to subscribe to a pointcloud topic
    //ros::ServiceServer minimal_service_; //maybe want these later
    //ros::Publisher  minimal_publisher_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_ptr_; //(new PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformed_ptr_;
    
    bool got_kinect_cloud_;
    // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    //void initializePublishers();
    //void initializeServices();
    
    void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud); //prototype for callback fnc
    
    //prototype for example service
    //bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response);

}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
