#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/fill_image.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief Return minimum of a and b
 * @param a 
 * @param b 
 * @return int 
 */
int min_i(int a, int b)
{
  return (a<b)?a:b;
}

/**
 * @brief First-order dynamic system for the orientation of the camera, including end-stops
 */
class DynamicsSimulation
{
public:
  DynamicsSimulation(rclcpp::Node *parent_node, double time_step)
  : parent_node_{parent_node}
  {
    create_parameters();
    reset();
    time_step_ = time_step;
  }
  
  /**
   * @brief Reset the camera angle to (0,0)
   */
  void reset()
  {
    x_ = 0;
    y_ = 0;
    x_des_ = 0;
    y_des_ = 0;
  }

  /**
   * @brief Perform one time step; computing the new state.
   * 
   */
  void step()
  {
    double x_limit_rad = get_x_limit();
    double y_limit_rad = get_y_limit();
    double tau = parent_node_->get_parameter("tau_s").as_double();

    // Compute velocity 
    double dx = (x_des_ - x_) / tau;
    double dy = (y_des_ - y_) / tau;
    // Integrate (forward euler)
    x_ = x_ + dx * time_step_;
    y_ = y_ + dy * time_step_;
    // Apply endstops
    if (x_ < -x_limit_rad) x_ = -x_limit_rad;
    if (x_ >  x_limit_rad) x_ =  x_limit_rad;
    if (y_ < -y_limit_rad) y_ = -y_limit_rad;
    if (y_ >  y_limit_rad) y_ =  y_limit_rad;
  }

  void create_parameters()
  {
    // The parent node is needed to link the paramters to.
    parent_node_->declare_parameter<double>("x_limit_rad", 0.8);
    parent_node_->declare_parameter<double>("y_limit_rad", 0.6);
    parent_node_->declare_parameter<double>("tau_s", 0.3);
  }

  double get_x() { return x_; }
  double get_y() { return y_; }
  double get_x_limit() { return parent_node_->get_parameter("x_limit_rad").as_double(); }
  double get_y_limit() { return parent_node_->get_parameter("y_limit_rad").as_double(); }
    
  void set_x_des(const double x) { x_des_ = x; }
  void set_y_des(const double y) { y_des_ = y; }
  void set_des(const double x, const double y) { x_des_ = x; y_des_ = y; }
  
private:
  rclcpp::Node *parent_node_;
  double x_, y_; // Position in rad
  double x_des_, y_des_; // Desired positions
  double time_step_; // Approximate time step in seconds
};

class JiwySimulator : public rclcpp::Node
{
  public:
    JiwySimulator(double time_step)
    : Node("jiwy_simulator"),
      dynamics_simulation_{this, time_step}
    {
      create_topics();
      dynamics_timer_ = this->create_wall_timer(std::chrono::duration<double>(time_step), std::bind(&JiwySimulator::dynamics_timer_callback, this));
    }

  private:
    // Topics
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr webcam_input_topic_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr moving_camera_output_topic_;
    rclcpp::Subscription<asdfr_interfaces::msg::Point2>::SharedPtr setpoint_topic_;
    rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr position_topic_;

    // Dynamics simulation timer (plus position output)
    rclcpp::TimerBase::SharedPtr dynamics_timer_;
    DynamicsSimulation dynamics_simulation_;

    // Container for building the output image as a message type.
    sensor_msgs::msg::Image output_image_;
      
    /**
     * @brief Create all topics for this node
     * 
     */
    void create_topics()
    {
      RCLCPP_INFO(this->get_logger(), "Creating topics...");
      webcam_input_topic_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image", 10, std::bind(&JiwySimulator::webcam_topic_callback, this, _1));
      moving_camera_output_topic_ = this->create_publisher<sensor_msgs::msg::Image>("moving_camera_output", 1);
      setpoint_topic_ = this->create_subscription<asdfr_interfaces::msg::Point2>(
        "setpoint", 10, std::bind(&JiwySimulator::setpoint_topic_callback, this, _1));
      position_topic_ = this->create_publisher<asdfr_interfaces::msg::Point2>("position",1);
    }

    void webcam_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
      // RCLCPP_INFO(this->get_logger(), "Received webcam frame");

      // Compute which pixel from the original image should be in the center of the output image.
      // Given the camera pan angle x, the range from -x_limit ... +x_limit (in radians) maps linearly to pixels 0...(width-1);
      // the same for tilt angle y, but then upside down -> larger y gives lower pixel value
      const double x = dynamics_simulation_.get_x();
      const double y = dynamics_simulation_.get_y();
      const double x_limit = dynamics_simulation_.get_x_limit();
      const double y_limit = dynamics_simulation_.get_y_limit();

      int center_pixel_x = (int)floor( (x - (-x_limit)) / (2*x_limit) * msg->width); // This can also output msg->width itself, which should be corrected to msg->width-1
      if (center_pixel_x>=(int)msg->width) center_pixel_x = msg->width;

      int center_pixel_y = (int)floor( (-y - (-y_limit)) / (2*y_limit) * msg->height); // This can also output msg->height itself, which should be corrected to msg->height-1
      if (center_pixel_y>=(int)msg->height) center_pixel_y = msg->height;

      CreateSubimage(msg, center_pixel_x, center_pixel_y);
      moving_camera_output_topic_->publish(output_image_);
    }

    /**
     * @brief Fill the output_image_ member with a sub-image of the input image msg.
     * @param msg 
     * @param center_pixel_x 
     * @param center_pixel_y 
     */
    void CreateSubimage(const sensor_msgs::msg::Image::SharedPtr msg, const int center_pixel_x, const int center_pixel_y)
    {
      // Parameters (you're allowed to change these)
      const int width_divider= 2; // The output image's width is (int)(msg->width / width_divider).
      const int height_divider = 2;

      // Parameters for the Image message type
      const int width = msg->width / width_divider;
      const int height =  msg->height / height_divider;
      const int n_bytes_per_channel = sensor_msgs::image_encodings::bitDepth(msg->encoding) / 8;
      const int n_channels = sensor_msgs::image_encodings::numChannels(msg->encoding);
      const int n_bytes_per_pixel = n_bytes_per_channel * n_channels;
      const int step = width * n_bytes_per_pixel; // Size of one row in bytes
      uint8_t *data = new uint8_t[step*height]; // Temporary container for the image data.
          // For now, we allocate this dynamically in the function each time (this is costly, but easy to program), 
          // so that, if someone changes the input image size during run time, nothing breaks. A more efficient idea
          // is to allocate it as a member of the class and resize only when needed (which is probably never).
          // In fact, if we make sure that output_image is of the right size, we can directly update the data in there...
          // But that's for later.

     
      // The msg-index of the leftmost pixel in the output image (output_image_[0])  is: -w_output/2 + center_pixel)x
      // Compute which pixel from the orignal image should be in the leftmost column of the output image
      // Same for height`;  // integer division
      const int leftmost_pixel_x = center_pixel_x - width/2;  // integer division
      const int topmost_pixel_y = center_pixel_y - height/2; // integer division
      
      // Fill the image with image data
      memset(data, (uint8_t)0, step*height); // Make the whole image black we'll fill the part with data in it later.
      for (int row=0; row<height; row++)
      {
        // For each row of the output image
        int input_row_index = topmost_pixel_y + row;
        if (input_row_index>=0 && input_row_index < (int)msg->height)
        {
          // Then there is image data for this row, so process it. If not, just leave it; the rest of the image is already black.
          // Now find out which part of the row data to copy. We have two possibilities:
          // 1. leftmost_pixel_x < 0. This means that there should be some black on the left part of the output image. We start copying to a positive output-image-index
          // 2. leftmost_pixel_x >=0. Then the leftmost row of the output image has image data (i.e., not black). We _might_ have some blackness on the right part of the
          //    output image. In this case, we start copying to the first pixel (and, depending on black on the right or not, we copy more or less bytes)
          if (leftmost_pixel_x<0)
          {
            // In this case, the first pixel to write to is 0-leftmost_pixel_x; we fill it starting the lefmost pixel of the input image.
            const int n_pixels_to_copy = (width +leftmost_pixel_x); // leftmost_pixel_x is negative!
            if (n_pixels_to_copy>0)
              memcpy(&data[(step * row) + n_bytes_per_pixel * (0 - leftmost_pixel_x)], &msg->data[msg->step * input_row_index + 0], n_pixels_to_copy * n_bytes_per_pixel);
          } else
          {
            // In this case, the first pixel to write is 0; we fill it starting somewhere in the middle of the input image. Number of pixels to copy
            // depends: if there is black on the rigth part of the output image, we copy until the end of the row in the input image; otherwise we copy the full width of the output image.
            const int n_pixels_to_copy = min_i ( msg->width - leftmost_pixel_x, width);
            if (n_pixels_to_copy >0)
              memcpy(&data[(step * row) + 0], &msg->data[(msg->step * input_row_index) + n_bytes_per_pixel * leftmost_pixel_x], n_pixels_to_copy * n_bytes_per_pixel);
          }
        }
      }

      // Build the image message
      sensor_msgs::fillImage(output_image_, msg->encoding, height, width, step, data);
      delete data;
    }

    void setpoint_topic_callback(const asdfr_interfaces::msg::Point2::SharedPtr msg) 
    {
      RCLCPP_INFO(this->get_logger(), "Received setpoint: [%f, %f]", msg->x, msg->y);
      dynamics_simulation_.set_des(msg->x, msg->y);
    }

    void dynamics_timer_callback()
    {
      // Do dynamics integration step
      dynamics_simulation_.step();

      // Output the actual position
      asdfr_interfaces::msg::Point2 pos;
      pos.x = dynamics_simulation_.get_x();
      pos.y = dynamics_simulation_.get_y();
      position_topic_->publish(pos);
      // RCLCPP_INFO(this->get_logger(), "Dynamics timer!   [x,y] = [%f,%f]", pos.x, pos.y);
    }

};

int main(int argc, char * argv[])
{
  printf("Jiwy Simulator Node\n-------------------\n");
  rclcpp::init(argc, argv);
  auto jiwy_simulator = std::make_shared<JiwySimulator>(0.01);
  rclcpp::spin(jiwy_simulator);
  rclcpp::shutdown();
  return 0;
}
