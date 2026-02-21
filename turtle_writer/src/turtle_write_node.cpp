#include "rclcpp/rclcpp.hpp"

class TurtleWriterNode : public rclcpp::Node
{
    public:
    TurtleWriterNode():Node("turtle_writer")
    {
      this->declare_parameter("name","HI");
      std::string name = this->get_parameter("name").as_string();
      RCLCPP_INFO(this->get_logger(), "Will draw: %s", name.c_str());
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrutleWriterNode>());
  rclcpp::shutdown();
  return 0;
}
