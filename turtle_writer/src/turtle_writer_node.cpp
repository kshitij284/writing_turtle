#include <chrono>
#include <memory>
#include <cmath>
#include <thread>
#include <string>
#include <vector>
#include <map>

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
    bool pose_received_ = false;

    void sub_callback(const turtlesim::msg::Pose & msg)
    {
        current_pose_ = msg;
        pose_received_ = true;
    }

    public:
    TurtleWriterNode() : Node("turtle_writer")
    {
        this->declare_parameter("name", "HI");
        std::string name = this->get_parameter("name").as_string();
        RCLCPP_INFO(this->get_logger(), "Will draw: %s", name.c_str());

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleWriterNode::sub_callback, this, std::placeholders::_1));
        client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for set_pen service...");
        }
    }

    void wait_for_pose()
    {
        while (!pose_received_) {
            rclcpp::sleep_for(std::chrono::milliseconds(50));
        }
        RCLCPP_INFO(this->get_logger(), "Pose ready: x=%.3f y=%.3f", current_pose_.x, current_pose_.y);
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
            rclcpp::sleep_for(std::chrono::milliseconds(10));

            float dx = current_pose_.x - start_x;
            float dy = current_pose_.y - start_y;
            float distance_traveled = std::sqrt(dx*dx + dy*dy);

            if (distance_traveled >= distance) { break; }
        }

        cmd_vel_.linear.x = 0.0;
        publisher_->publish(cmd_vel_);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    void turn(float angle)
    {
        float total_turned = 0.0;
        float prev_theta = current_pose_.theta;

        cmd_vel_.angular.z = angle > 0 ? 1.0 : -1.0;
        cmd_vel_.linear.x = 0.0;

        float target = std::abs(angle);
        float buffer = 0.11;

        while (true)
        {
            publisher_->publish(cmd_vel_);
            rclcpp::sleep_for(std::chrono::milliseconds(10));

            float current_theta = current_pose_.theta;
            float delta = current_theta - prev_theta;

            if (delta > M_PI)  delta -= 2.0 * M_PI;
            if (delta < -M_PI) delta += 2.0 * M_PI;

            total_turned += delta;
            prev_theta = current_theta;

            if (std::abs(total_turned) >= (target - buffer)) { break; }
        }

        cmd_vel_.angular.z = 0.0;
        publisher_->publish(cmd_vel_);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    void turn_to(float target_theta)
    {
        float delta = target_theta - current_pose_.theta;

        while (delta > M_PI)  delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;

        if (std::abs(delta) > 0.05) {
            turn(delta);
        }
    }

    void set_pen(bool on)
    {
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->off = on ? 0 : 1;
        request->r = 255;
        request->g = 255;
        request->b = 255;
        request->width = 2;
        client_->async_send_request(request);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    void go_to(float x, float y)
    {

        float dx = x - current_pose_.x;
        float dy = y - current_pose_.y;
        float distance = std::sqrt(dx*dx + dy*dy);
        float angle = std::atan2(dy, dx);

        turn_to(angle);
        move(distance);

    }

    void draw_H()
    {
	float start_x = 2.0;
	float start_y = 2.0;
	
	set_pen(false);	
	go_to(start_x, start_y);
        
	std::vector<std::vector<float>> H = {{2,5,1},{2,3.5,0},{4,3.5,1},{4,5,0},{4,2,1}};
	
	for(size_t i = 0; i< H.size() ; i++)
	{
	  float x = H[i][0];
	  float y = H[i][1];

	  float pen = H[i][2];
	  
	  if(pen == 1){set_pen(true);}
	  else{set_pen(false);}

	  	 
	  float scale = 1.0;
	  go_to( x*scale, y*scale);

	}
    }

    void draw_word(const std::string & name, const std::map<char, std::vector<std::vector<float>>> & letters)
    {
      float origin_x = 2.0;
      float origin_y = 2.0;

      set_pen(false);
      go_to(origin_x, origin_y);
      float scale = 0.5;
      for (char c : name)
      {
        std::vector<std::vector<float>> character = letters.at(c);
        for (size_t i = 0; i < character.size(); i++)
        {
            float x   = character[i][0];
            float y   = character[i][1];
            float pen = character[i][2];

            if (pen == 1) { set_pen(true); }
            else          { set_pen(false); }

            go_to(origin_x + x * scale, origin_y + y * scale);
        }

        float space = scale * 2.5;
        origin_x += space;

        set_pen(false);
        go_to(origin_x, origin_y);
      }
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleWriterNode>();
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    executor->add_node(node);



    std::map<char, std::vector<std::vector<float>>> letters = {
      {'H', {{0,3,1},{0,1.5,0},{2,1.5,1},{2,3,0},{2,0,1}}},
      {'E', {{2,0,1},{0,0,0},{0,3,1},{2,3,1},{0,1.5,0},{2,1.5,1}}},
      {'L', {{2,0,1},{0,0,0},{0,3,1}}},
      {'O', {{0,3,1},{2,3,1},{2,0,1},{0,0,1},{0,3,1}}}
      };
    
    std::thread drawing_thread([&node, &letters]() {
        node->wait_for_pose();
        node->draw_word("HELLO",letters);
    });

    executor->spin();

    drawing_thread.join();
    rclcpp::shutdown();
    return 0;
}
