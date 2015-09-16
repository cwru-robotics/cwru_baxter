//intent: receive two image streams, grab pairs, reset their time stamps
// and republish both.  This is to "spoof" the stereo process, which insists on
// images having nearly identical time stamps

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//static const std::string OPENCV_WINDOW = "Image window";

class ImageSyncher
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_left_;
  image_transport::Subscriber image_sub_right_;  
  image_transport::Publisher image_pub_left_;
  image_transport::Publisher image_pub_right_;  

  sensor_msgs::Image img_left_,img_right_;
   void imageLeftCb(const sensor_msgs::ImageConstPtr& msg)
  {
         //img_left_
      int datasize = msg->data.size();
      ROS_INFO("data size: %d",datasize);
      int cpy_datasize = img_left_.data.size();
      if (cpy_datasize!= datasize) {
          img_left_.data.resize(datasize);
          ROS_INFO("resizing");
      }
      img_left_ = *msg; // does this work? want to copy the data, but msg is a pointer
      got_new_image_left_=true; 
  }
   void imageRightCb(const sensor_msgs::ImageConstPtr& msg)
  {
      int datasize = msg->data.size();
      ROS_INFO("data size: %d",datasize);
      int cpy_datasize = img_right_.data.size();
      if (cpy_datasize!= datasize) {
          img_right_.data.resize(datasize);
          ROS_INFO("resizing");
      }       
      img_right_ = *msg; // does this work? want to copy the data, but msg is a pointer
      got_new_image_right_=true; 
  }
   
public:

  ImageSyncher(ros::NodeHandle &nh):nh_(nh),it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_left_ = it_.subscribe("/davinci/left_camera/image_raw", 1, 
      &ImageSyncher::imageLeftCb, this);
    image_pub_left_ = it_.advertise("/davinci/left_camera/image_raw_synced", 1);
    
    image_sub_right_ = it_.subscribe("/davinci/right_camera/image_raw", 1, 
      &ImageSyncher::imageRightCb, this);
    image_pub_right_ = it_.advertise("/davinci/right_camera/image_raw_synced", 1);
    
    got_new_image_left_=false;
    got_new_image_right_=false;
    //sensor_msgs::Image img1_,img2_; // copies of received images

    //cv::namedWindow(OPENCV_WINDOW);
  }

    void pub_both_images() {
        ros::Time tnow= ros::Time::now();
        img_left_.header.stamp = tnow; // reset the time stamps to be identical--a lie, but hopefully lets stereo work
        img_right_.header.stamp = tnow;
        image_pub_left_.publish(img_left_);
        got_new_image_left_=false;   
        image_pub_right_.publish(img_right_);
        got_new_image_right_=false;          
    }
  bool got_new_image_left_;
  bool got_new_image_right_;    

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  ros::NodeHandle nh;
    ros::Rate ratetimer(1);
  ImageSyncher is(nh);
  while(ros::ok())
  {
    ros::spinOnce();
    if (is.got_new_image_left_&&is.got_new_image_right_)
        is.pub_both_images();
    
    ratetimer.sleep();
  }
  return 0;
}
