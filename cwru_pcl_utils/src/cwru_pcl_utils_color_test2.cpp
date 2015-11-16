//cwru_pcl_utils_color_test.cpp
// example accessing Kinect colored pointclouds

// run this w/:
// roslaunch cwru_baxter_sim baxter_world.launch
// roslaunch cwru_baxter_sim kinect_xform.launch
// rosrun cwru_pcl_utils cwru_pcl_utils_color_test

#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "cwru_pcl_utils_color_test"); //node name
    ros::NodeHandle nh;
    CwruPclUtils cwru_pcl_utils(&nh);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("kinect_clr_snapshot.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file kinect_clr_snapshot.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  


  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << (int) cloud->points[i].r
              << " "    << (int) cloud->points[i].g
              << " "    << (int) cloud->points[i].b << std::endl;

  return (0);


}

