#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class SignalGeneratorNode : public rclcpp::Node
{
public:
    SignalGeneratorNode() : Node("signal_generator_node")
    {
        
        sin_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/sin_wave", 10);
        square_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/square_wave", 10);
        timer_ = this->create_wall_timer(1ms, std::bind(&SignalGeneratorNode::publish_signals, this));
        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Signal Generator Node started (1000Hz publish rate)");
    }

private:
    void publish_signals()
    {
        //计算当前运行时间
        auto current_time = this->now();
        double t = (current_time - start_time_).seconds();
        
        std_msgs::msg::Float64 sin_msg;
        sin_msg.data = sin(20*M_PI*t);  
        
        std_msgs::msg::Float64 square_msg;
        double square_period = 1.0;  
        if (fmod(t, square_period) < square_period / 2)
        {
            square_msg.data = 1.0;  //高电平
        }
        else
        {
            square_msg.data = -1.0;  //低电平
        }
        
        //发布信号
        sin_publisher_->publish(sin_msg);
        square_publisher_->publish(square_msg);
    }

    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sin_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr square_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
    //初始化ROS2
    rclcpp::init(argc, argv);
    //运行节点
    rclcpp::spin(std::make_shared<SignalGeneratorNode>());
    //关闭ROS2
    rclcpp::shutdown();
    return 0;
}