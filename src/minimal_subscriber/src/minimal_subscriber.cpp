#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/fill_image.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1; 
class MinimalSubscriber : public rclcpp::Node

{ 
  rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr sub_pos_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_moving_cam_;

  public:

    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      sub_pos_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
      "position", 10, std::bind(&MinimalSubscriber::pos_callback, this, _1));

      sub_moving_cam_ = this->create_subscription<sensor_msgs::msg::Image>(
      "moving_camera_output", 10, std::bind(&MinimalSubscriber::moving_cam_callback, this, _1));
    }

 

  private:

    void pos_callback(const asdfr_interfaces::msg::Point2::SharedPtr pos)
    {
      RCLCPP_INFO(this->get_logger(), "Received setpoint: [%f, %f]", pos->x, pos->y);
    }

    void moving_cam_callback(const sensor_msgs::msg::Image::SharedPtr img)

    {

      // Convert the image message to an OpenCV Mat object
      cv::Mat image = cv_bridge::toCvCopy(img,"mono8")->image;
      
      // Resize image
      cv::Mat resized_image;
      cv::resize(image, resized_image, cv::Size(image.cols * 2, image.rows * 2));

      // show image
      cv::imshow("im_jiwy", image);
      cv::waitKey(1);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}