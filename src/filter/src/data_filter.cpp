#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <deque>
#include <algorithm>

class Datafilter : public rclcpp::Node
{
public:
    Datafilter() : Node("data_filter")
    {
        data_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/data_generator", 10,
            std::bind(&Datafilter::callback, this, std::placeholders::_1));
        
        Median_filter_ = this->create_publisher<std_msgs::msg::Float64>("/median_filter", 10);
        LowPass_filter_ = this->create_publisher<std_msgs::msg::Float64>("/lowpass_filter", 10);

        RCLCPP_INFO(this->get_logger(), "Data_filter Node started");
    }

private:
  
 
    void callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        
        double new_value = msg->data;

        //中值滤波
        data_buffer_.push_back(new_value);
        if (data_buffer_.size() > window_size_) {
            data_buffer_.pop_front();
        }
        if (data_buffer_.size() == window_size_) {
            std::vector<double> temp(data_buffer_.begin(), data_buffer_.end());
            std::sort(temp.begin(), temp.end());
            double median = temp[window_size_ / 2];

            std_msgs::msg::Float64 median_msg;
            median_msg.data = median;
            Median_filter_->publish(median_msg);
        }


        //低通滤波
        double alpha = 0.1; //低通滤波系数
        filtered_value_ = alpha * new_value + (1 - alpha) * filtered_value_;

        std_msgs::msg::Float64 lowpass_msg;
        lowpass_msg.data = filtered_value_;
        LowPass_filter_->publish(lowpass_msg);
    }
    
    

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr data_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr Median_filter_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr LowPass_filter_;
    
    std::deque<double> data_buffer_;
    const size_t window_size_ = 15;  //最近5个样本
    double filtered_value_ = 0.0;   //上一次低通滤波结果
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Datafilter>());
    rclcpp::shutdown();
    return 0;
}