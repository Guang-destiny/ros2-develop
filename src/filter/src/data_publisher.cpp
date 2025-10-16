#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class Datapublisher : public rclcpp::Node
{
public:
    Datapublisher() : Node("data_publisher"),
    noise_distribution_(0.0, 0.1)
    {
        
        data_generator_ = this->create_publisher<std_msgs::msg::Float64>("/data_generator", 10);
        timer_ = this->create_wall_timer(2ms, std::bind(&Datapublisher::publish_signals, this));
        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(),"publisher:start");
    }

private:
    void publish_signals()
    {
        //计算当前运行时间
        auto current_time = this->now();
        double t = (current_time - start_time_).seconds();
        
        
        double sin_msg = std::sin(2*M_PI*t);  
        double noise = noise_distribution_(generator_);
        
        //信号是由一段正弦波和一些噪声叠加而成
        std_msgs::msg::Float64 data_msg;
        data_msg.data=sin_msg+noise;

        //发布信号
        data_generator_->publish(data_msg);
        
    }

    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr data_generator_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    std::default_random_engine generator_;
    std::normal_distribution<double> noise_distribution_;
};

int main(int argc, char * argv[])
{
    //初始化ROS2
    rclcpp::init(argc, argv);
    //运行节点
    rclcpp::spin(std::make_shared<Datapublisher>());
    //关闭ROS2
    rclcpp::shutdown();
    return 0;
}
