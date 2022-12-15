#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.hpp>

#include <memory>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
// #include <sensor_msgs/msg/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/aruco.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using std::placeholders::_1;

class Detection : public rclcpp::Node
{
  public:
    Detection()
    : Node("image_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&Detection::topic_callback, this, _1)); 

    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {     
   cv_bridge::CvImagePtr cv_ptr;

      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // cv::Mat img = cv::imread("/home/vignesh/Downloads/aruco_marker.png");
    cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    int i=0;
    int count[25];
   std::vector<std::vector<cv::Point2f>> corners;
   std::vector<int> ids;
   cv::aruco::detectMarkers(cv_ptr,dict,corners,ids);
   if(ids.size()>0)
   cv::aruco::drawDetectedMarkers(cv_ptr,corners,ids);
  // cv::namedWindow("out",0);
    cv::resize(cv_ptr, cv_ptr, cv::Size(cv_ptr.cols/3, cv_ptr.rows/3)); 
    //cv::namedWindow( "Display frame", cv::WindowFlags::WINDOW_AUTOSIZE);
   cv::imshow("out",cv_ptr);
   
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Detection>());
  rclcpp::shutdown();
  return 0;
}