#include "rclcpp/rclcpp.hpp"
#include "asdfr_interfaces/msg/point2.hpp"

class LightFollower : public rclcpp::Node
{
public:
  LightFollower() : Node("light_follower")
  {
    // Subscribe to the light position topic
    light_sub_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
      "/setpoint", 10, std::bind(&LightFollower::lightCallback, this, std::placeholders::_1));

    // Advertise the position topic
    position_pub_ = this->create_publisher<asdfr_interfaces::msg::Point2>("position", 1);
  }

private:
  void lightCallback(const asdfr_interfaces::msg::Point2::SharedPtr msg)
  {
    // Do something with the light position
    RCLCPP_INFO(this->get_logger(), "Received light position (%f, %f)", msg->x, msg->y);

    // Calculate the new Jiwy position based on the light position
    double x = msg->x * 2.0; // Scale the light x-coordinate
    double y = msg->y * 2.0; // Scale the light y-coordinate

    // Send the new Jiwy position to the simulator
    std::string command = "position " + std::to_string(x) + " " + std::to_string(y);
    system(command.c_str());

    // Publish the Jiwy position on the position topic
      asdfr_interfaces::msg::Point2 position_msg;
      position_msg.x = x;
      position_msg.y = y;

    // Publish the light position on the position topic
    //  position_pub_->publish(*msg);
  }

  rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr light_sub_;
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr position_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LightFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
