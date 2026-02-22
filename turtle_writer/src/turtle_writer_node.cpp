#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"

using namespace std::chrono_literals;

class TurtleWriterNode : public rclcpp::Node
{
    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client_;
    turtlesim::msg::Pose current_pose_;
    geometry_msgs::msg::Twist cmd_vel_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;  

    void sub_callback(const turtlesim::msg::Pose & msg) { current_pose_ = msg; }

    public:
    void set_executor(rclcpp::executors::SingleThreadedExecutor::SharedPtr exec)  
    {
        executor_ = exec;
    }

    TurtleWriterNode():Node("turtle_writer")
    {
        this->declare_parameter("name", "HI"); 
        std::string name = this->get_parameter("name").as_string();

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleWriterNode::sub_callback, this, std::placeholders::_1));
        client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for set_pen service...");
        }

    }

    void move(float distance)
    {
        float start_x = current_pose_.x;  
        float start_y = current_pose_.y;  

        cmd_vel_.linear.x = 1.0;  
        cmd_vel_.angular.z = 0.0;  

        while (true)
        {
            publisher_->publish(cmd_vel_);
            executor_->spin_some();

            float dx = current_pose_.x - start_x;  
            float dy = current_pose_.y - start_y;  
            float distance_traveled = std::sqrt(dx*dx + dy*dy);

            if (distance_traveled >= distance) { break; }
        }

        cmd_vel_.linear.x = 0.0;  
        publisher_->publish(cmd_vel_);
    }

    void turn(float angle)
    {
        float start_angle = current_pose_.theta;  

        cmd_vel_.angular.z = 1.0;  
        cmd_vel_.linear.x = 0.0;  

        while (true)
        {
            publisher_->publish(cmd_vel_);
            executor_->spin_some();  

            float angle_turned = std::abs(current_pose_.theta - start_angle);  
            if (angle_turned >= std::abs(angle)) { break; }
        }

        cmd_vel_.angular.z = 0.0; 
        publisher_->publish(cmd_vel_);
    }

    void set_pen(bool on)
    {
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>(); 
        request->off = on ? 0 : 1;
        client_->async_send_request(request);
        rclcpp::sleep_for(std::chrono::milliseconds(100)); 
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleWriterNode>();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();  

    executor->add_node(node);
    node->set_executor(executor);

    rclcpp::sleep_for(std::chrono::milliseconds(1000)); 

    node->move(2.0);
    node->turn(3.14159/2.0);
    node->move(2.0);

    executor->spin();
    rclcpp::shutdown();

    return 0;
}
