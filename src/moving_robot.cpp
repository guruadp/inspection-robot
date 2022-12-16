// MIT License

// Copyright (c) 2022 Guru Nandhan A D P, Dhanus Babu Allaam, Vignesh RR

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
 * @file moving_robot.cpp
 * @author Guru Nandhan A D P(guruadp@umd.edu)
 * @brief this makes the robot to follow a circular path
 * @version 0.1
 * @date 2022-12-07
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MovingRobot : public rclcpp::Node {
 public:
  MovingRobot() : Node("move_turtlebot"), count_(0) {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MovingRobot::timer_callback, this));
  }

 private:
  void timer_callback() {
    int i = 1;
    auto message = geometry_msgs::msg::Twist();
   // message.linear.x=0.0;
    while (i<=2){
      message.linear.x = 0.1;
      // message.angular.z = 0.3;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f.2' and %f.2",
                  message.linear.x, message.angular.z);
      publisher_->publish(message);
      i++;
    }
    message.linear.x = 0.0;
    publisher_->publish(message);

  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovingRobot>());
  rclcpp::shutdown();
  return 0;
}
