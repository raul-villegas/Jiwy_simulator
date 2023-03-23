#include "rclcpp/rclcpp.hpp"
#include "asdfr_interfaces/msg/point2.hpp"

class PointPublisherNode : public rclcpp::Node
{
public:
  PointPublisherNode()
  : Node("point_publisher")
  {
    // Set up publisher
    pub_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);

    // Set up timer for publishing
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&PointPublisherNode::publish_point, this));
  }

private:
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  float x_ = 0.0;
  float y_ = 0.0;

  void publish_point()
  {
    // Create and populate message
    auto msg = asdfr_interfaces::msg::Point2();
    msg.x = x_;
    msg.y = y_;

    // Publish message
    pub_->publish(msg);

    // Update x and y values
    x_ += 1.0;
    y_ += 0.5;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
