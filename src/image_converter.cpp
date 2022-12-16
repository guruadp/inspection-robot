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
 * @author Guru Nandhan A D P(guruadp@umd.edu), Dhanush Babu Allam
 * (dallam@umd.edu), Vignesh RR (rr94@umd.edu)
 * @brief This detects the aruco marker
 * @version 0.1
 * @date 2022-12-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <chrono>
#include <functional>
// #include <image_transport/image_transport.hpp>
#include <memory>
#include <string>

#include "rclcpp/logger.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#include <opencv2/opencv.hpp>

// #include <sensor_msgs/msg/image_encodings.h>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;
/**
 * @brief The Detection class is used to subscribe to sensor message and then
 * publish the report
 *
 */
class Detection : public rclcpp::Node {
 public:
  /**
   * @brief Construction of a new Detection object (node)
   *
   */
  Detection() : Node("image_subscriber") {
    // subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    // "/camera/image_raw", 10, std::bind(&Detection::topic_callback, this,
    // _1));

    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&Detection::timer_callback, this));
  }

 private:
  /**
   * @brief The function is used to process the image obtained from the
   * turtlebot
   *
   * @param msg : image obtained from the sensor_msgs topic capture by the
   * turtlebot
   */
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
    // cv_bridge::CvImagePtr cv_ptr;
    // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // cv::Mat img = cv::imread("/home/vignesh/Downloads/aruco_marker.png");
    // cv::Ptr<cv::aruco::Dictionary> dict =
    //    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
   // int i = 0;
    //int count[25];
    //std::vector<std::vector<cv::Point2f>> corners;
    //std::vector<int> ids;
    // cv::aruco::detectMarkers(cv_ptr,dict,corners,ids);
    //  if(ids.size()>0)
    //  cv::aruco::drawDetectedMarkers(cv_ptr,corners,ids);
    // // cv::namedWindow("out",0);
    //   cv::resize(cv_ptr, cv_ptr, cv::Size(cv_ptr.cols/3, cv_ptr.rows/3));
    //   //cv::namedWindow( "Display frame", cv::WindowFlags::WINDOW_AUTOSIZE);
    //  cv::imshow("out",cv_ptr);
  }
  /**
   * @brief function to publish the report of aruco id detected
   *
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    // int i=10;
    message.data = "Publishing the Aruco Marker ID : ";

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Able to update message data ");
    RCLCPP_INFO(this->get_logger(), "Publishing the message : '%s'",
                message.data);

    publisher_->publish(message);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Detection>());
  rclcpp::shutdown();
  return 0;
}
