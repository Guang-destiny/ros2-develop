#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <deque>
#include <algorithm>
#include <cmath>
class MotorPID : public rclcpp::Node
{
public:
    MotorPID()
        : Node("pid_controller")
    {
        //PID参数
        kp_angle = 1;
        ki_angle = 0.00;
        kd_angle = 0.0001;
        kp_speed=0.3;
        ki_speed=0.0;
        kd_speed=0.001;

        target_angle = 3;  

        velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/simulated_motor_angle", 10,
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
        


    //时间相关
        current_time = this->now().seconds();
        dt = current_time - last_time;
        if (dt <= 0.0) dt = 1e-3;  //防止除0
    //PID外环   
        double update_interval = 0.02; // 每20ms更新一次外环
        static double last_outer_time = 0.0;

        if (current_time - last_outer_time >= update_interval){
        current_angle = lowpass_msg.data;
        //计算误差
        error_angle = std::atan2(std::sin(target_angle - current_angle),
                         std::cos(target_angle - current_angle));
        integral_angle += error_angle * dt;
        double derivative_angle_raw = (error_angle - last_error_angle) / dt;

        derivative_angle = 0.9 * derivative_angle + 0.1 * derivative_angle_raw;
        target_speed = kp_angle * error_angle + ki_angle * integral_angle + kd_angle * derivative_angle;
        //限制输出范围（防止过冲）
        target_speed = std::clamp(target_speed, -3.0, 3.0);
        }
    //PID内环    
        double raw_speed = (filtered_value_ - last_angle) / dt;
        current_speed = 0.8 * current_speed + 0.2 * raw_speed;
        error_speed=target_speed-current_speed;
        integral_speed +=error_speed * dt;
        derivative_speed = (error_speed - last_error_speed) / dt;
        output_speed = kp_speed * error_speed + ki_speed * integral_speed + kd_speed * derivative_speed;
        output_speed = std::clamp(output_speed, -1.0, 1.0);
    //PID结束，发布信息    
        std_msgs::msg::Float64 control_msg;
        control_msg.data = output_speed;
        control_pub_->publish(control_msg);



    //最后的数据转换
        last_error_angle = error_angle;
        last_error_speed=error_speed;
        last_time = current_time;
        last_output_speed=output_speed;
        last_angle=current_angle;
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;


    //PID外环参数
    double kp_angle, ki_angle, kd_angle;
    double target_angle;
    double current_angle;
    double last_angle=0;
    double last_error_angle = 0.0;
    double integral_angle = 0.0;
    double error_angle=0;
    double derivative_angle=0;
    //PID内环参数
    double kp_speed,ki_speed,kd_speed;
    double target_speed=0.0;
    double error_speed=0.0;
    double last_error_speed=0.0;
    double last_output_speed=0;
    double output_speed=0;
    double current_speed;
    double derivative_speed=0;
    double integral_speed=0.0;
    //内外环通用参数
    double last_time = 0.0;
    double dt=0;
    double current_time=0;
    
    double i=0;
    
    //滤波参数
    double alpha_ = 0.1;
    double filtered_value_ = 0.0;   //上一次低通滤波结果
    std_msgs::msg::Float64 median_msg;
    std_msgs::msg::Float64 lowpass_msg;
    const size_t window_size_ = 15;  
    std::deque<double> data_buffer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorPID>());
    rclcpp::shutdown();
    return 0;
}



      