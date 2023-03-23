#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // include the new message type
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

class LightPositionIndicator : public rclcpp::Node
{
public:
  LightPositionIndicator() : Node("light_position_indicator")
  {
    // Create a ROS2 parameter for the brightness threshold with a default value of 100
    this->declare_parameter<double>("brightness_threshold", 100.0);

    // Subscribe to the input image topic
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", 10, std::bind(&LightPositionIndicator::imageCallback, this, std::placeholders::_1));

    // Advertise the output topic
    light_pub_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the image message to a cv::Mat object
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Apply a threshold on the brightness of each pixel
    double brightness_threshold = this->get_parameter("brightness_threshold").as_double();
    cv::Mat binary_image;
    cv::threshold(cv_ptr->image, binary_image, brightness_threshold, 255, cv::THRESH_BINARY);

    // Compute the center of gravity of the white pixels
    cv::Moments moments = cv::moments(binary_image, true);
    double x = moments.m10 / moments.m00 / cv_ptr->image.cols;
    double y = moments.m01 / moments.m00 / cv_ptr->image.rows;

    // Publish the position of the center of gravity as a geometry_msgs/Point message
    auto light_msg = asdfr_interfaces::msg::Point2();
    light_msg.x = x;
    light_msg.y = y;
    light_pub_->publish(light_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr light_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LightPositionIndicator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
