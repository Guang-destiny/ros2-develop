#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <deque>
#include <algorithm>

class MotorPID : public rclcpp::Node
{
public:
    MotorPID()
        : Node("motor_pid_controller")
    {
        //PID参数
        kp_ = 0.5;
        ki_ = 0.00;
        kd_ = 0.0001;

        target_velocity_ = 500.0;  //目标转速(rad/s)

        velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/simulated_motor_velocity", 10,
            std::bind(&MotorPID::velocity_callback, this, std::placeholders::_1));

        control_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/motor_simulator_control_input", 10);

        RCLCPP_INFO(this->get_logger(), "Motor_PID  start");
    }

private:
    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
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

            median_msg.data = median;

        }
        //中值滤波
        filtered_value_ = alpha_ * median_msg.data + (1 - alpha_) * filtered_value_;
        lowpass_msg.data = filtered_value_;
        
        double current_velocity = lowpass_msg.data;

        //计算误差
        double error = target_velocity_ - current_velocity;

        double current_time = this->now().seconds();
        double dt = current_time - last_time_;
        if (dt <= 0.0) dt = 1e-3;  //防止除0

        integral_ += error * dt;
        double derivative = (error - last_error_) / dt;

        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        //限制输出范围（防止过冲）
        output = std::clamp(output, -3.0, 3.0);

        
        std_msgs::msg::Float64 control_msg;
        control_msg.data = output;
        control_pub_->publish(control_msg);

        last_error_ = error;
        last_time_ = current_time;

    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;

    double kp_, ki_, kd_;
    double target_velocity_;
    double last_error_ = 0.0;
    double integral_ = 0.0;
    double last_time_ = 0.0;
    double alpha_ = 0.1;
    double filtered_value_ = 0.0;   //上一次低通滤波结果
    std_msgs::msg::Float64 median_msg;
    std_msgs::msg::Float64 lowpass_msg;
    const size_t window_size_ = 5;  //最近5个样本
    std::deque<double> data_buffer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorPID>());
    rclcpp::shutdown();
    return 0;
}
