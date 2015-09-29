//get an image, search for red pixels;
// republish on new topic...and print out result
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
using namespace std;

int g_redthresh;

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:

    ImageConverter()
    : it_(nh_) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/davinci/left_camera/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix = 0;
        int isum = 0;
        int jsum = 0;
        int redval,blueval,greenval,testval;
        cv::Vec3b rgbpix;
        //image.at<uchar>(j,i)= 255;
        /**/
        for (int i = 0; i < cv_ptr->image.cols; i++)
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j,i); //[j][i];
                redval = rgbpix[2];
                blueval = rgbpix[0]+1;
                greenval = rgbpix[1]+1;
                testval = redval/(blueval+greenval);
                //redval = (int) cv_ptr->image.at<cv::Vec3b>(j, i)[0]; 
                //cout<<"image("<<j<<","<<i<<")[0] = "<<redval<<endl;
                if (testval > g_redthresh) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                    npix++;
                    isum += i;
                    jsum += j;
                }
                else {
                    /*
                    for (int j = 0; j < cv_ptr->image.rows; j++) {
                        cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 128;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                    }
                     * */
                }
            }
         /* */
        cout << "npix: " << npix << endl;
        if (npix>0) {
            cout << "i_avg: " << isum / npix << endl;
            cout << "j_avg: " << jsum / npix << endl;
        }
        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());

    }
};

    int main(int argc, char** argv) {
        ros::init(argc, argv, "image_converter");
        ImageConverter ic;
        cout<<"enter red threshold: (0-255) ";
        cin>>g_redthresh;
        ros::spin();
        return 0;
    }
