// pcl_perception_node.cpp: wsn, June, 2015
// example code to acquire a pointcloud from disk, then perform various processing steps interactively.
// processing is invoked by point-cloud selections in rviz, as well as "mode" settings via a service
// e.g.:  rosservice call process_mode 0 induces processing in mode zero (plane fitting)

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
//#include <tf_eigen.h>
//#include <tf_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>

#include <pcl/PCLPointCloud2.h>

#include <cwru_srv/simple_int_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/IM_node_service_message.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

#include <pcl_conversions/pcl_conversions.h>

//clutching at straws here to find problem w/ pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;


//#include <pcl_perception_node/pcl_perception_defs.h>
/**/
 //define some processing modes; set these interactively via service
const int PCL_IDENTIFY_PLANE = 0;
const int PCL_FIND_PNTS_ABOVE_PLANE = 1;
const int PCL_COMPUTE_CYLINDRICAL_FIT_ERR_INIT = 2;
const int PCL_COMPUTE_CYLINDRICAL_FIT_ERR_ITERATE = 3;
const int PCL_MAKE_CAN_CLOUD = 4;
const int PCL_FIND_ON_TABLE = 5;
const int PCL_TAKE_SNAPSHOT = 6;

const double Z_EPS = 0.01; //choose a tolerance for plane fitting, e.g. 1cm
const double R_EPS = 0.05; // choose a tolerance for cylinder-fit outliers

const double R_CYLINDER = 0.055; //estimated from ruler tool...example to fit a cylinder of this radius to data
const double H_CYLINDER = 0.24; // estimated height of cylinder
 /**/

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud; // can use this for short-hand

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io; 

Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);
void computeRsqd(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, Eigen::Vector3f centroid, std::vector<float> &rsqd_vec);
Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::vector<int>iselect);
void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Matrix3f R_xform, PointCloud<pcl::PointXYZ>::Ptr outputCloud);
void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Matrix3f R_xform, Eigen::Vector3f offset, PointCloud<pcl::PointXYZ>::Ptr outputCloud);
void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Affine3f A,  PointCloud<pcl::PointXYZ>::Ptr outputCloud);
void filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, vector<int> &indices);
void filter_cloud_above_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_threshold, vector<int> &indices);

void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud);
void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> &indices, PointCloud<pcl::PointXYZ>::Ptr outputCloud);

void find_plane(Eigen::Vector4f plane_params, std::vector<int> &indices_z_eps);

void process_patch(std::vector<int> &iselect_filtered, Eigen::Vector3f &centroidEvec3f, Eigen::Vector4f &plane_params);
void compute_radial_error(PointCloud<pcl::PointXYZ>::Ptr inputCloud, std::vector<int> indices, double r, Eigen::Vector3f center, double &E, double &dEdCx, double &dEdCy);
void make_can_cloud(PointCloud<pcl::PointXYZ>::Ptr canCloud, double r_can,double h_can);

//next line causes a seg-fault!! even if you don't use ne
// SEEMS TO BE DUE TO C++11 FLAGS--DO NOT COMPILE W/ C++11 !!
//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
// a bunch of pointcloud holders, all global

//pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclKinect(new PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_from_disk(new pcl::PointCloud<pcl::PointXYZ>); //this one is read from PCD file on disk
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_out(new pcl::PointCloud<pcl::PointXYZ>); // holder for processed point clouds
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>); // holder for processed point clouds
pcl::PointCloud<pcl::PointXYZ>::Ptr g_display_cloud(new pcl::PointCloud<pcl::PointXYZ>); // this cloud gets published--viewable in rviz
//PointCloud<pcl::PointXYZ>::Ptr canCloud
        
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclSelect(new pcl::PointCloud<pcl::PointXYZ>); // holds published points, per Rviz tool
pcl::PointCloud<pcl::PointXYZ>::Ptr g_canCloud(new pcl::PointCloud<pcl::PointXYZ>); // holds model for a can
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclKinect(new pcl::PointCloud<pcl::PointXYZ>); //this one is from kinect stream

sensor_msgs::PointCloud2::Ptr g_pcl2_display_cloud (new sensor_msgs::PointCloud2);
// PointXYZRGB would be colorized

Eigen::Matrix3f g_R_transform; // a matrix useful for rotating the data

Eigen::Vector3f g_cylinder_origin; // origin of model for cylinder registration

int g_pcl_process_mode = PCL_TAKE_SNAPSHOT; // mode--set by service
bool g_trigger = true; // a trigger, to tell "main" to process points in the currently selected mode
bool g_processed_patch = false; // a state--to let us know if a default set of plane_parameters exists
bool g_take_snapshot = true; // let kinect CB save data ONLY when desired

//more globals--to share info on planes and patches
Eigen::Vector4f g_plane_params;
Eigen::Vector3f g_patch_centroid;
Eigen::Vector3f g_plane_normal;
Eigen::Vector3f g_plane_origin;
Eigen::Vector3f g_patch_origin;
Eigen::Affine3f g_A_plane,g_A_model;
double g_z_plane_nom;
std::vector<int> g_indices_of_plane; //indices of patch that do not contain outliers 





/**/

// process patch: filter selected points to remove outliers;
// then compute the centroid and the plane parameters of the filtered points
// return these values in centroidEvec3f and plane_params

void process_patch(std::vector<int> &iselect_filtered, Eigen::Vector3f &centroidEvec3f, Eigen::Vector4f &plane_params) {
    ROS_INFO("PROCESSING THE PATCH: ");
    int npts = g_pclSelect->width * g_pclSelect->height;
    cout<<"frame_id of g_pclSelect: "<<g_pclSelect->header.frame_id<<endl;
    cout<<("(this seems to be a lie; presumably this is actually kinect_pc_frame)");
    centroidEvec3f = computeCentroid(g_pclSelect); // compute the centroid of this point cloud (selected patch)

    std::vector<float> rsqd_vec;
    computeRsqd(g_pclSelect, centroidEvec3f, rsqd_vec);
    ROS_INFO("computing rsqd vec: ");
    float variance = 0.0;
    for (int i = 0; i < rsqd_vec.size(); i++) {
        variance += rsqd_vec[i];
        //cout<<rsqd_vec[i]<<", ";      
    }
    cout << endl;
    variance /= ((float) npts);
      
    // now, eliminate any outliers; actually, keep only points withing 1 std;
    for (int i = 0; i < npts; i++) {
        if (rsqd_vec[i] < variance) {
            iselect_filtered.push_back(i); // choosey: retain only points within 1 std dev
        }
    }
    cout << "npts = " << npts << endl;
    cout << "npts of filtered patch: " << iselect_filtered.size() << endl;
    cout << "variance = " << variance << endl;
    cout << "std_dev: " << sqrt(variance) << endl;
    centroidEvec3f = computeCentroid(g_pclSelect, iselect_filtered);
    cout << "refined centroid:    " << centroidEvec3f.transpose() << endl;

    // next line causes a core dump, if compile w/ C++11 !!
    //  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    NormalEstimation<PointXYZ, Normal> n; // object to compute the normal to a set of points

    float curvature;

    n.computePointNormal(*g_pclSelect, iselect_filtered, plane_params, curvature); // find plane params for filtered patch
    // any surface viewed w/ z_optical pointing "out" from camera must have a surface normal with z-component that is negative w/rt camera
    if (plane_params[2]>0.0) {
        //need to negate the surface normal, as well as (signed) distance of plane from base origin
        for (int i=0;i<4;i++)
            plane_params[i]*= -1.0;
    }
    std::cout << "plane_params, filtered patch: " << plane_params.transpose() << std::endl;
    /*
    std::cout<< "here are z-values of preserved points: "<<endl;
    int j;
    Eigen::Vector3f sel_pt;
    for (int i=0;i<iselect_filtered.size();i++) {
        j = iselect_filtered[i];
        sel_pt= g_pclSelect->points[j].getVector3fMap();   
        cout<<sel_pt.transpose()<<endl;
    }
    */
}


// this function operates on the global cloud pointer g_pclKinect;
// g_cloud_transformed contains a cloud rotated s.t. identified plane has normal (0,0,1), 
//  indices_z_eps contain the indices of the points on the identified plane;
//  g_display_cloud is a reduced version of g_pclKinect, w/ only the planar points (expressed in original frame)

/**/
void find_plane(Eigen::Vector4f plane_params, std::vector<int> &indices_z_eps) {
    float curvature;
    std::vector<int> iselect_all;
    NormalEstimation<PointXYZ, Normal> n; // object to compute the normal to a set of points 

   //Eigen::Vector3f plane_normal;
    Eigen::Vector3f x_dir;
    for (int i = 0; i < 3; i++) g_plane_normal[i] = plane_params[i]; //for selected patch, get plane normal from point-cloud processing result

    x_dir << 1, 0, 0; // keep x-axis the same...nominally
    x_dir = x_dir - g_plane_normal * (g_plane_normal.dot(x_dir)); // force x-dir to be orthogonal to z-dir
    x_dir /= x_dir.norm(); // want this to be unit length as well
    //populate g_R_transform with the direction vectors of the plane frame, with respect to the kinect frame
    g_R_transform.col(0) = x_dir;
    g_R_transform.col(2) = g_plane_normal; // want the z-axis to be the plane normal    
    g_R_transform.col(1) = g_R_transform.col(2).cross(g_R_transform.col(0)); //make y-axis consistent right-hand triad
    // let's define an origin on this plane as well.  The patch centroid should do
    g_plane_origin = g_patch_centroid;
    // define the transform s.t. A*pt_wrt_plane coords = pt_wrt_sensor_coords
    g_A_plane.linear()= g_R_transform;
    g_A_plane.translation() = g_plane_origin;
    cout<<"Defining plane coord sys: origin: "<<g_plane_origin.transpose()<<endl;
    cout<<"Orientation: "<<endl;
    cout<<g_R_transform<<endl;
    
    // use the following to transform kinect points into the plane frame; could do translation as well, but not done here
    //Eigen::Matrix3f R_transpose = g_R_transform.transpose();

    g_z_plane_nom = plane_params[3]; // distance of plane from sensor origin--same as distance measured along plane normal--so should be negative
    cout<<"nom plane dist from sensor origin: "<<g_z_plane_nom<<endl;
    // after rotating the points to align with the plane of the selected patch, all z-values should be approximately the same,
    // = z_plane_nom
    double z_eps = Z_EPS; // choose a tolerance for plane inclusion +/- z; 1cm??

    //transform the ENTIRE point cloud:
    //transform_cloud(g_pclKinect, R_transpose, g_cloud_transformed); // rotate the entire point cloud
    transform_cloud(g_pclKinect, g_A_plane.inverse(), g_cloud_transformed); // transform the entire point cloud    
    // g_cloud_transformed is now expressed in the frame of the selected plane;
    // let's extract all of the points (i.e., name the indices of these points) for which the z value corresponds to the chosen plane,
    // within tolerance z_eps
    //filter_cloud_z(g_cloud_transformed, g_z_plane_nom, z_eps, indices_z_eps);
    filter_cloud_z(g_cloud_transformed, 0.0, z_eps, indices_z_eps); //transform --> z-values of points on plane should be 0.0
    // point indices of interest are in indices_z_eps; use this to extract this subset from the parent cloud to create a new cloud
    copy_cloud(g_pclKinect, indices_z_eps, g_display_cloud); //g_display_cloud is being published regularly by "main"

}


// given a point cloud, compute the centroid. Mostly useful for small patches, since centroid of the whole cloud is not too useful
//no selection vector provided, --> use all the points
Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud) {
    Eigen::Vector3f centroid;
    centroid << 0, 0, 0;

    int size = pcl_cloud->width * pcl_cloud->height;
    std::cout << "frame: " << pcl_cloud->header.frame_id << std::endl;
    for (size_t i = 0; i != size; ++i) {
        centroid += pcl_cloud->points[i].getVector3fMap();
    }
    if (size > 0) {
        centroid /= ((float) size);
    }
    return centroid;
}

// second version: 
// will operate only on the points listed by index in iselect
Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, std::vector<int>iselect) {
    Eigen::Vector3f centroid;
    centroid << 0, 0, 0;
    int nselect = iselect.size();
    for (int i = 0; i < nselect; i++) {
        centroid += pcl_cloud->points[iselect[i]].getVector3fMap();
    }
    if (nselect > 0) {
        centroid /= ((float) nselect);
    }
    return centroid;
}


// compute the distance-squared of each point from the provided centroid
// presumably useful for outlier removal filtering

void computeRsqd(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, Eigen::Vector3f centroid, std::vector<float> &rsqd_vec) {
    Eigen::Vector3f evec3f;
    int npts = pcl_cloud->points.size();
    rsqd_vec.clear();
    rsqd_vec.resize(npts);
    for (int i = 0; i < npts; i++) {
        evec3f = pcl_cloud->points[i].getVector3fMap();
        evec3f -= centroid;
        rsqd_vec[i] = evec3f.dot(evec3f);
    }
}

// given an input cloud, rotate ALL points using matrix R_xform, and put the result in outputCloud
// This should be generalized for an affine transform (translation plus rotation)

void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Matrix3f R_xform, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    // copy over the header info from the inputCloud.
    // NOTE: copying the 
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = inputCloud->width;
    outputCloud->height = inputCloud->height;
    int npts = inputCloud->points.size();
    cout << "transforming npts = " << npts << endl;
    outputCloud->points.resize(npts);
    Eigen::Vector3f pt;
    for (int i = 0; i < npts; ++i) {
        pt = R_xform * inputCloud->points[i].getVector3fMap();
        //cout<<"transformed pt: "<<pt.transpose()<<endl;
        outputCloud->points[i].getVector3fMap() = pt; //R_xform * inputCloud->points[i].getVector3fMap ();
    }
}

void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Matrix3f R_xform, Eigen::Vector3f offset, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    // copy over the header info from the inputCloud.
    // NOTE: copying the 
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = inputCloud->width;
    outputCloud->height = inputCloud->height;
    int npts = inputCloud->points.size();
    cout << "transforming npts = " << npts << endl;
    outputCloud->points.resize(npts);
    Eigen::Vector3f pt;
    for (int i = 0; i < npts; ++i) {
        pt = R_xform * inputCloud->points[i].getVector3fMap() + offset; // + 
        //cout<<"transformed pt: "<<pt.transpose()<<endl;
        outputCloud->points[i].getVector3fMap() = pt; //R_xform * inputCloud->points[i].getVector3fMap ();
    }
}

//for this version, provide an affine transform:
void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Affine3f A,  PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = inputCloud->width;
    outputCloud->height = inputCloud->height;
    int npts = inputCloud->points.size();
    cout << "transforming npts = " << npts << endl;
    outputCloud->points.resize(npts);
    Eigen::Vector3f pt;
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = A * inputCloud->points[i].getVector3fMap(); // + 
        //cout<<"transformed pt: "<<pt.transpose()<<endl;
        //outputCloud->points[i].getVector3fMap() = pt; //R_xform * inputCloud->points[i].getVector3fMap ();
    }    
}

//function to copy ALL of the cloud points
void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    int npts = inputCloud->points.size(); //how many points to extract?
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = npts;
    outputCloud->height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud->points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = inputCloud->points[i].getVector3fMap();
    }
}

//make a new point cloud, extracted from inputCloud using only points listed  in "indices"
void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> &indices, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    int npts = indices.size(); //how many points to extract?
    outputCloud->header = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width = npts;
    outputCloud->height = 1;

    cout << "copying cloud w/ npts =" << npts << endl;
    outputCloud->points.resize(npts);
    for (int i = 0; i < npts; ++i) {
        outputCloud->points[i].getVector3fMap() = inputCloud->points[indices[i]].getVector3fMap();
    }
}

/* * */

//given a cloud, identify which points are within z_eps of z_nom; put these point indices in "indices"
// If the cloud has been rotated such that we expect a plane with normals (0,0,1) and offset z_nom,
// this function will find the points within tolerance z_eps of such a plane
void filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, vector<int> &indices) {
    int npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    double dz;
    int ans;
    for (int i = 0; i < npts; ++i) {
        pt = inputCloud->points[i].getVector3fMap();
        //cout<<"pt: "<<pt.transpose()<<endl;
        dz = pt[2] - z_nom;
        if (fabs(dz) < z_eps) {
            indices.push_back(i);
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = indices.size();
    cout << " number of points in range = " << n_extracted << endl;
}

//given a cloud, identify which points are ABOVE z_threshold; put these point indices in "indices"
// this can be useful, e.g., for finding objects "on" a table
void filter_cloud_above_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_threshold, vector<int> &indices) {
    int npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    //double dz;
    int ans;
    for (int i = 0; i < npts; ++i) {
        pt = inputCloud->points[i].getVector3fMap();
        //cout<<"pt: "<<pt.transpose()<<endl;
        //dz = pt[2] - z_threshold;
        if (pt[2] > z_threshold) {
            indices.push_back(i);
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = indices.size();
    cout << " number of points extracted = " << n_extracted << endl;
}


// this is a pretty specific function, but useful for illustrating how to create a point cloud that
// can be visualized in rviz.
// create a cloud to visualize a can of radius r, height h.
// assume origin of bottom of can is 0,0,0, and z-axis points up;
// resulting cloud of can model is contained both in arg (canCloud) and g_canCloud
void make_can_cloud(PointCloud<pcl::PointXYZ>::Ptr canCloud, double r_can, double h_can) {
    double theta,h;
    Eigen::Vector3f pt;
    int npts=0;
    ROS_INFO("making can cloud");
    //count the points:
    for (theta=0;theta<2.0*M_PI;theta+=0.3)
        for (h=0;h<h_can;h+= 0.01)  
            npts++;
    canCloud->points.resize(npts);

    int i=0;
    for (theta=0;theta<2.0*M_PI;theta+=0.3)
        for (h=0;h<h_can;h+= 0.01) {
            
            pt[0] = r_can*cos(theta);
            pt[1] = r_can*sin(theta);
            pt[2] = h;
            canCloud->points[i].getVector3fMap() = pt;
            i++;
        }

    canCloud->header.frame_id = "kinect_pc_frame"; //base"; //vs /base_link?
    //canCloud->header.stamp = ros::Time::now();
    canCloud->is_dense = true;
    canCloud->width = npts;
    canCloud->height = 1;

    copy_cloud(canCloud,g_canCloud);
    ROS_INFO("done making/copying canCloud");
    //optionally, rotate the cylinder to make its axis parallel to the most recently defined normal axis
    //transform_cloud(g_canCloud, g_R_transform, canCloud);    
}

//COMPUTE_RADIAL_ERROR:  NOT COMPLETE; RUNS, BUT HAS NOT BEEN TESTED, SO PROBABLY HAS BUGS
//try to fit a cylinder to points in inputCloud, assuming cylinder axis is vertical (0,0,1).  
//given radius of model and center coords, cx, cy, return the fit error and derivatives dE/dCx and dE/dCy
// eliminate points that are too far from expectation
// define the error as: r_i = sqrt(c_vec - p_i); err_i = (r_i - r)^2; E = sum_i err_i
//inputCloud should be rotated appropriately, so expected cylinder has major axis along z
void compute_radial_error(PointCloud<pcl::PointXYZ>::Ptr inputCloud, std::vector<int> indices, double r, Eigen::Vector3f center, double &E, double &dEdCx, double &dEdCy) {
    std::vector<int> copy_indices;
    int npts = indices.size();
    copy_indices.resize(npts);
    
    cout << " number of initial points = " << npts << endl;     
    for (int i=0;i<npts;i++) copy_indices[i]=indices[i];
    indices.clear(); // we will re-populate this with inliers
    double sum_sqd_err=0.0;
    double r_i,err_i,r_sqd_err;
    dEdCx=0.0;
    dEdCy=0.0;
    E=0.0;
    double dx,dy;
    Eigen::Vector3f pt;
    //indices.clear();
    //double dz;
    int ans;
    for (int i = 0; i < npts; ++i) {
        pt = inputCloud->points[copy_indices[i]].getVector3fMap();
        dx = center[0] - pt[0];
        dy = center[1] - pt[1];
        r_i = sqrt(dx*dx+dy*dy);
        r_sqd_err = (r_i - r)*(r_i-r);
        err_i = sqrt(r_sqd_err);
        if (err_i<R_EPS) {
            // include this point as an inlier:
            indices.push_back(copy_indices[i]);
            sum_sqd_err += r_sqd_err;
            dEdCx += (r_i-r)*(dx)/r_i;
            dEdCy += (r_i-r)*(dy)/r_i;            
        }       
    }
    int n_extracted = indices.size();
    cout << " number of inliers = " << n_extracted << endl;   
    E = 0.5*sum_sqd_err/n_extracted;
    dEdCx/= n_extracted;
    dEdCy/= n_extracted;

    cout<<"rms radial error: "<<sqrt(2.0*E)<<endl;
    cout<<"E = "<<E<<endl;    
    cout<<"dE/dCx = "<<dEdCx<<"; dEdCy = "<<dEdCy<<endl;
}
/**/

//use this service to set processing modes interactively
bool modeService(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response) {
    ROS_INFO("mode select service callback activated");
    response.resp = true; // boring, but valid response info
    g_pcl_process_mode = request.req;
    g_trigger = true; //signal that we received a request; trigger a response
    cout << "Mode set to: " << g_pcl_process_mode << endl;
    return true;
}

bool getFrameService(cwru_srv::IM_node_service_messageRequest& request, cwru_srv::IM_node_service_messageResponse& response ) {
    ROS_INFO("pcl_perception_node: received request for frame");
    //if pay attention to input codes, could choose what frame to return
    // by default, return the frame of the model (can)
    cout<<"model frame origin (w/rt sensor frame)"<<g_A_model.translation().transpose()<<endl;
    cout<<"model frame orientation: "<<endl;
    cout<<g_A_model.linear()<<endl;
    geometry_msgs::Pose pose;
    geometry_msgs::PoseStamped poseStamped;
    //tf::poseEigenToMsg 	(g_A_model,pose);
    
    Eigen::Quaternionf quat(g_A_model.linear());
    Eigen::Vector3f origin(g_A_model.translation());
    //origin = g_A_model.translation();    
    //quat = g_A_model.linear();
    ROS_INFO("quaternion: %f, %f, %f, %f", quat.x(),quat.y(),quat.z(),quat.w());
    //ROS_INFO("origin...");
    cout<<"origin: "<<origin.transpose()<<endl; 
    //cout<<"quaternion: "<<quat<<endl;
    poseStamped.pose.orientation.x =  quat.x();
    poseStamped.pose.orientation.y =  quat.y();    
    poseStamped.pose.orientation.z =  quat.z();
    poseStamped.pose.orientation.w =  quat.w();

    poseStamped.pose.position.x = origin[0];
    poseStamped.pose.position.y = origin[1];
    poseStamped.pose.position.z = origin[2]; 
    poseStamped.header.stamp = ros::Time().now();
    poseStamped.header.frame_id = "kinect_pc_frame";
    // need to convert this frame to a poseStamped and put into response
    response.poseStamped_IM_current = poseStamped;
}

// this callback wakes up when a new "selected Points" message arrives
void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect(new PointCloud<pcl::PointXYZ>);    
    pcl::fromROSMsg(*cloud, *pclKinect); 
    g_pclSelect = pclKinect;    
    //pcl::fromROSMsg(*cloud, *g_pclSelect);
    ROS_INFO("RECEIVED NEW PATCH w/  %d * %d points", g_pclSelect->width, g_pclSelect->height);
    //ROS_INFO("frame id is: %s",cloud->header.frame_id);
    //cout << "header frame: " << cloud->header.frame_id << endl;
    int npts = g_pclSelect->width * g_pclSelect->height;
    std::vector<int> iselect_filtered; //indices of patch that do not contain outliers

    //find centroid and plane params
    process_patch(iselect_filtered, g_patch_centroid, g_plane_params); // operate on selected points to remove outliers and

    g_processed_patch = true; // update our states to note that we have processed a patch, and thus have valid plane info
}

// this callback wakes up when a new kinect pointcloud arrives
void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect(new PointCloud<pcl::PointXYZ>);    
    pcl::fromROSMsg(*cloud, *pclKinect);   

    //pcl::fromROSMsg(*cloud, *g_pclKinect);
    if (g_take_snapshot) {
        g_pclKinect = pclKinect; 
        ROS_INFO("RECEIVED NEW KINECT POINTCLOUD w/  %d * %d points", g_pclKinect->width, g_pclKinect->height);
        //ROS_INFO("frame id is: %s",cloud->header.frame_id);
        cout << "header frame: " << g_pclKinect->header.frame_id << endl;
        int npts = g_pclKinect->width * g_pclKinect->height;
        ROS_INFO("npts = %d",npts);
 
    }
    g_take_snapshot=false; //don't do this again until requested
}

int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "pcl_perception_node");
    ros::NodeHandle nh;
    ros::Rate rate(2);
    int ans;
    
    // Subscribers
    cout<<"subscribing to kinect depth points"<<endl;    
    // Subscribers
    // use the following, if have "live" streaming from a Kinect
    ros::Subscriber getPCLPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1, kinectCB);
    
    cout<<"subscribing to selected points"<<endl;  
    
    // subscribe to "selected_points", which is published by Rviz tool
    ros::Subscriber selectedPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1, selectCB);

    // have rviz display both of these topics
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_perception_display", 1);
    ros::Publisher pubModel = nh.advertise<sensor_msgs::PointCloud2> ("/model_display", 1);    
    ros::Publisher pubPcdCloud = nh.advertise<sensor_msgs::PointCloud2> ("/kinect_pointcloud", 1);

    // service used to interactively change processing modes
    ros::ServiceServer service = nh.advertiseService("pcl_perception_svc", modeService);
    ros::ServiceServer frame_service = nh.advertiseService("pcl_getframe_svc", getFrameService);
    std::vector<int> indices_pts_above_plane;

/*
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *g_cloud_from_disk) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
            << g_cloud_from_disk->width * g_cloud_from_disk->height
            << " data points from test_pcd.pcd  " << std::endl;

    g_cloud_from_disk->header.frame_id = "world"; //kinect_pc_frame"; // why is this necessary?  Did PCD delete the reference frame id?
   */ 
    
    
    double z_threshold=0.0;
    double E;
    double dEdCx=0.0;
    double dEdCy=0.0;
    double dEnorm;
    float cx,cy;  

    Eigen::Vector3f can_center_wrt_plane;
    Eigen::Vector3f height_delta_wrt_sensor_frame;
    Eigen::Affine3f A_plane_to_sensor;
    //cout<<"enter 1 to start loop: ";
    //cin>>ans;
            
    while (ros::ok()) {
        if (g_trigger) {
            g_trigger = false; // reset the trigger

            switch (g_pcl_process_mode) { // what we do here depends on our mode; mode is settable via a service
                case PCL_TAKE_SNAPSHOT:
                    ROS_INFO("case TAKE_SNAPSHOT:");
                    g_take_snapshot=true;
                    //TEST TEST TEST:
                    //g_trigger = true; 
                    //g_pcl_process_mode=MAKE_CAN_CLOUD;
                    
                    break;
                case PCL_MAKE_CAN_CLOUD:
                    ROS_INFO("creating a can cloud");
                    make_can_cloud(g_display_cloud, R_CYLINDER, H_CYLINDER);
                    break;
                    /**/
                case PCL_IDENTIFY_PLANE:
                    ROS_INFO("MODE 0: identifying plane based on patch selection...");
                    find_plane(g_plane_params, g_indices_of_plane); // results in g_display_cloud (in orig frame), as well as 
                    //g_cloud_transformed (rotated version of original cloud); g_indices_of_plane indicate points on the plane
                    break;
                case PCL_FIND_PNTS_ABOVE_PLANE:
                    ROS_INFO("filtering for points above identified plane");
                    // w/ affine transform, z-coord of points on plane (in plane frame) should be ~0
                    z_threshold = 0.0+Z_EPS; //g_plane_params[3] + Z_EPS;
                    ROS_INFO("filtering for points above %f ", z_threshold);

                    filter_cloud_above_z(g_cloud_transformed, z_threshold, indices_pts_above_plane);
                    //extract these points--but in original, non-rotated frame; useful for display
                    copy_cloud(g_pclKinect, indices_pts_above_plane, g_display_cloud);
                    break;
                case PCL_COMPUTE_CYLINDRICAL_FIT_ERR_INIT:
                    //double cx,cy;
                    // base coords on centroid of most recent patch, and offset normal to this patch by radius of cylinder
                    ROS_INFO("creating a can cloud");
                    make_can_cloud(g_display_cloud, R_CYLINDER, H_CYLINDER);
                    
                    for (int i=0;i<3;i++) {
                        g_cylinder_origin[i] = g_patch_centroid[i]-R_CYLINDER*g_plane_params[i]; // TEST TEST TEST -R_CYLINDER*g_plane_params[i];
                    }
                    
                    // fix the z-height, based on plane height: watch out for signs
                    cout<<"g_z_plane_nom: "<<g_z_plane_nom<<endl;
                    cout<<"g_cylinder_origin: "<<g_cylinder_origin.transpose()<<endl;
                    cout<<"g_plane_normal: "<<g_plane_normal.transpose()<<endl;
                    cout<<"g_plane_origin: "<<g_plane_origin.transpose()<<endl;
                    //height_delta_wrt_sensor_frame = (g_z_plane_nom - g_plane_origin).dot(g_plane_normal)*g_plane_normal;
                    //cout<<"height_delta_wrt_sensor_frame: "<<height_delta_wrt_sensor_frame.transpose()<<endl;
                    //redo:
                    height_delta_wrt_sensor_frame = (g_cylinder_origin - g_plane_origin).dot(g_plane_normal)*g_plane_normal;
                    cout<<"height_delta_wrt_sensor_frame: "<<height_delta_wrt_sensor_frame.transpose()<<endl;                    
                    g_cylinder_origin -= height_delta_wrt_sensor_frame; // - (g_z_plane_nom - g_plane_normal.dot(g_cylinder_origin))*g_plane_normal;

                    // now, cast this into the rotated coordinate frame:
                    can_center_wrt_plane = g_A_plane.inverse()*g_cylinder_origin; //g_R_transform.transpose()*can_center_wrt_plane;

                    cout<<"initial guess for cylinder fit: "<<endl;
                    cout<<" attempting fit at c = "<<can_center_wrt_plane.transpose()<<endl;                

                    compute_radial_error(g_cloud_transformed,indices_pts_above_plane,R_CYLINDER,can_center_wrt_plane,E,dEdCx,dEdCy);
                    cout<<"E: "<<E<<"; dEdCx: "<<dEdCx<<"; dEdCy: "<<dEdCy<<endl;
                    cout<<"R_xform: "<<endl;
                    cout<<g_R_transform<<endl;
                    A_plane_to_sensor.linear() = g_R_transform;
                    A_plane_to_sensor.translation() = g_cylinder_origin;
                    g_A_model = A_plane_to_sensor;
                    transform_cloud(g_canCloud, A_plane_to_sensor, g_display_cloud);
 
                    break;
                case PCL_COMPUTE_CYLINDRICAL_FIT_ERR_ITERATE:         
                        cout<<"current cx,cy = "<<can_center_wrt_plane[0]<<", "<<can_center_wrt_plane[1]<<endl;
                        //cout<<"enter new cx: ";
                        //cin>>cx;
                        //scale dEdC:
                        dEnorm = sqrt(dEdCx*dEdCx + dEdCy*dEdCy);
                        
                        can_center_wrt_plane[0]-= 0.001*dEdCx/dEnorm; //move along negative gradient by 1mm
                        can_center_wrt_plane[1]-= 0.001*dEdCy/dEnorm;
                    
                    ROS_INFO("attempting to fit points to cylinder, radius %f, cx = %f, cy = %f",R_CYLINDER,can_center_wrt_plane[0],can_center_wrt_plane[1]);
                    compute_radial_error(g_cloud_transformed,indices_pts_above_plane,R_CYLINDER,can_center_wrt_plane,E,dEdCx,dEdCy);
                    cout<<"E: "<<E<<"; dEdCx: "<<dEdCx<<"; dEdCy: "<<dEdCy<<endl;
                    //cout<<"R_xform: "<<g_R_transform<<endl;
                    g_cylinder_origin=    g_A_plane*can_center_wrt_plane; 
                    A_plane_to_sensor.translation() = g_cylinder_origin;
                    g_A_model = A_plane_to_sensor;
                    transform_cloud(g_canCloud, A_plane_to_sensor, g_display_cloud);
                    //g_canCloud
                  
                    break;
                    


                case PCL_FIND_ON_TABLE:
                    ROS_INFO("filtering for objects on most recently defined plane: not implemented yet");

                    break;
                    
                default:
                    ROS_WARN("this mode is not implemented");

            }
        }

        // these are broken--need new conversions?
        //void pcl::toROSMsg (const pcl::PointCloud< PointT > &cloud,  sensor_msgs::PointCloud2 & msg
        //pubPcdCloud.publish(g_pclKinect); //keep displaying the original scene
        //convert pcl::PointCloud<pcl::PointXYZ>::Ptr to sensor_msgs::PointCloud2
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>)
        //sensor_msgs::PointCloud2::Ptr final_cloud (new sensor_msgs::PointCloud2);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr g_display_cloud(new pcl::PointCloud<pcl::PointXYZ>); // this cloud gets published--viewable in rviz
        //pcl::toROSMsg (*cloud_p, *final_cloud);

        pcl::toROSMsg(*g_display_cloud,*g_pcl2_display_cloud);
        g_pcl2_display_cloud->header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
        pubCloud.publish(g_pcl2_display_cloud); //and also display whatever we choose to put in here
        
        pcl::toROSMsg(*g_canCloud,*g_pcl2_display_cloud);
        g_pcl2_display_cloud->header.stamp = ros::Time::now(); //update the time stamp, so rviz does not complain        
        pubModel.publish(g_pcl2_display_cloud);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}