#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <mutex>  //用于线程安全的变量访问

class SignalProcessorNode : public rclcpp::Node
{
public:
    SignalProcessorNode() : Node("signal_processor_node")
    {
        sin_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sin_wave", 10,
            std::bind(&SignalProcessorNode::sin_callback, this, std::placeholders::_1));
        
        square_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/square_wave", 10,
            std::bind(&SignalProcessorNode::square_callback, this, std::placeholders::_1));
        
        //发布处理后的信号
        processed_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/processed_wave", 10);
        
        RCLCPP_INFO(this->get_logger(), "Signal Processor Node started");
    }

private:
  
    void sin_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sin_cache_ = msg->data;
    }

    
    void square_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        square_cache_ = msg->data;
        
        //信号处理
        std_msgs::msg::Float64 processed_msg;
        if (sin_cache_ * square_cache_ >= 0)
        {
            processed_msg.data = sin_cache_;
        }
        else
        {
            processed_msg.data = 0.0;
        }
        
        // 发布处理后的信号
        processed_publisher_->publish(processed_msg);
    }
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sin_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr square_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr processed_publisher_;
    
    double sin_cache_ = 0.0;    //正弦信号缓存
    double square_cache_ = 0.0; //方波信号缓存
    std::mutex mutex_;          //保护缓存变量的互斥锁
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
