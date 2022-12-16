// MIT License

// Copyright (c) 2022 Guru Nandhan A D P

// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

/**
 * @file image_converter.cpp
 * @author Guru Nandhan A D P(guruadp@umd.edu), Dhanush Babu Allam (dallam@umd.edu), Vignesh RR (vignesh31794@umd.edu)
 * @brief This detects the aruco marker
 * @version 0.1
 * @date 2022-12-15
 *
 * @copyright Copyright (c) 2022
 *
 */
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