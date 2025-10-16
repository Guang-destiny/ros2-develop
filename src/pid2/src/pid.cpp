#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <deque>
#include <algorithm>

class MotorPID : public rclcpp::Node
{
public:
    MotorPID()
        : Node("pid_controller")
    {
        //PID参数
        kp_speed = 0.5;
        ki_speed = 0.00;
        kd_speed = 0.0001;
        kp=0.5;
        ki=0.00;
        kd=0.0001;

        target_speed = 300.0;  //目标转速(rad/s)

        velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/simulated_motor_velocity", 10,
            std::bind(&MotorPID::velocity_callback, this, std::placeholders::_1));

        control_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/motor_simulator_control_input", 10);

        RCLCPP_INFO(this->get_logger(), "PID2 start");
    }

private:
    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
    //滤波部分
        double new_value = msg->data;
        //中值滤波
        filtered_value_ = alpha_ * new_value + (1 - alpha_) * filtered_value_;
        lowpass_msg.data = filtered_value_;
        
        //中值滤波
        data_buffer_.push_back(lowpass_msg.data);
        if (data_buffer_.size() > window_size_) {
            data_buffer_.pop_front();
        }
        if (data_buffer_.size() == window_size_) {
            std::vector<double> temp(data_buffer_.begin(), data_buffer_.end());
            std::sort(temp.begin(), temp.end());
            double median = temp[window_size_ / 2];

            median_msg.data = median;
        }


    //时间相关
        current_time = this->now().seconds();
        dt = current_time - last_time;
        if (dt <= 0.0) dt = 1e-3;  //防止除0
    //PID外环    
        current_speed = median_msg.data;
        //计算误差
        error_speed = target_speed - current_speed;
        integral_speed += error_speed * dt;
        derivative_speed = (error_speed - last_error_speed) / dt;
        target = kp_speed * error_speed + ki_speed * integral_speed + kd_speed * derivative_speed;
        //限制输出范围（防止过冲）
        target = std::clamp(target, -3.0, 3.0);
    //PID内环    
        current=last_output;
        error=target-current;
        integral +=error * dt;
        derivative = (error - last_error) / dt;
        output = kp * error + ki * integral + kd * derivative;
    //PID结束，发布信息    
        std_msgs::msg::Float64 control_msg;
        control_msg.data = output;
        control_pub_->publish(control_msg);



    //最后的数据转换
        last_error_speed = error_speed;
        last_error=error;
        last_time = current_time;
        last_output=output;
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;


    //PID外环参数
    double kp_speed, ki_speed, kd_speed;
    double target_speed;
    double current_speed;
    double last_error_speed = 0.0;
    double integral_speed = 0.0;
    double error_speed=0;
    double derivative_speed=0;
    //PID内环参数
    double kp,ki,kd;
    double target=0.0;
    double error=0.0;
    double last_error=0.0;
    double last_output=0;
    double output=0;
    double current;
    double derivative=0;
    //内外环通用参数
    double last_time = 0.0;
    double dt=0;
    double current_time=0;
    
    double integral=0.0;
    
    //滤波参数
    double alpha_ = 0.05;
    double filtered_value_ = 0.0;   //上一次低通滤波结果
    std_msgs::msg::Float64 median_msg;
    std_msgs::msg::Float64 lowpass_msg;
    const size_t window_size_ = 9;  //最近5个样本
    std::deque<double> data_buffer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorPID>());
    rclcpp::shutdown();
    return 0;
}

      