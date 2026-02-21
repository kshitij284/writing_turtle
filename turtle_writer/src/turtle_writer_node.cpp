#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"

class TurtleWriterNode : public rclcpp::Node
{
    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client_;
    
    void sub_callback(const turtlesim::msg::Pose & msg)
    {
      RCLCPP_INFO(this->get_logger(),"The current pose is x: %.2f, y: %.2f, theta: %.2f, a_vel: %.2f, l_vel: %.2f",msg.x, msg.y, msg.theta, msg.angular_velocity, msg.linear_velocity);
    }

    public:
    TurtleWriterNode():Node("turtle_writer")
    {
      this->declare_parameter("name","HI");
      std::string name = this->get_parameter("name").as_string();
      RCLCPP_INFO(this->get_logger(), "Will draw: %s", name.c_str());

      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",10);
      subscription_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose",10,[this](const turtlesim::msg::Pose & msg){this->sub_callback(msg);});
      client_ = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleWriterNode>());
  rclcpp::shutdown();
  return 0;
}
