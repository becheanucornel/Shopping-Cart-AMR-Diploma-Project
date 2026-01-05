#include <chrono>
#include <gpiod.hpp>
#include <fstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MotorController : public rclcpp::Node
{
    public:
    MotorController() : Node("motor_controller_node")
    {
        try
        {
            // Initialize PWM channels
            setup_pwm(0);  // Motor 1 Forwards
            setup_pwm(1);  // Motor 1 Backwards
            setup_pwm(2);  // Motor 2 Forwards
            setup_pwm(3);  // Motor 2 Backwards
            
            RCLCPP_INFO(this->get_logger(), "PWM initialized successfully");
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Initialization failed: %s", e.what());
        }
        
        cmd_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&MotorController::cmd_callback, this, std::placeholders::_1));    
    }

    ~MotorController()
    {
        cleanup_pwm(0);
        cleanup_pwm(1);
        cleanup_pwm(2);
        cleanup_pwm(3);
    }

    private:
    void setup_pwm(int pwm_id)
    {
        std::ofstream export_file("/sys/class/pwm/pwmchip0/export");
        export_file << pwm_id;
        export_file.close();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        std::string path = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(pwm_id) + "/period";
        std::ofstream period_file(path);
        period_file << 40000;  // 25kHz
        period_file.close();
    }

    void cleanup_pwm(int pwm_id)
    {
        std::ofstream unexport_file("/sys/class/pwm/pwmchip0/unexport");
        unexport_file << pwm_id;
        unexport_file.close();
    }

    void set_pwm_speed(int pwm_id, double speed)
    {
        int duty = static_cast<int>(std::abs(speed) * 40000);
        duty = std::max(0, std::min(40000, duty));
        
        std::string path = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(pwm_id) + "/duty_cycle";
        std::ofstream duty_file(path);
        duty_file << duty;
        duty_file.close();
        
        path = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(pwm_id) + "/enable";
        std::ofstream enable_file(path);
        enable_file << (duty > 0 ? 1 : 0);
        enable_file.close();
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear = msg->linear.x;
        double angular = msg->angular.z;
        double wheel_base = 0.37;
        
        // Differential drive kinematics
        double left_speed = linear - (angular * wheel_base / 2.0);
        double right_speed = linear + (angular * wheel_base / 2.0);
        
        // Speeds
        left_speed = std::max(-1.0, std::min(1.0, left_speed));
        right_speed = std::max(-1.0, std::min(1.0, right_speed));
        
        // Motor 1 (left): PWM0 = forwards, PWM1 = backwards
        if (left_speed > 0)
        {
            set_pwm_speed(0, left_speed);
            set_pwm_speed(1, 0);
        }
        else if (left_speed < 0)
        {
            set_pwm_speed(0, 0);
            set_pwm_speed(1, -left_speed);
        }
        else
        {
            set_pwm_speed(0, 0);
            set_pwm_speed(1, 0);
        }
        
        // Motor 2 (right): PWM2 = forwards, PWM3 = backwards
        if (right_speed > 0)
        {
            set_pwm_speed(2, right_speed);
            set_pwm_speed(3, 0);
        }
        else if (right_speed < 0)
        {
            set_pwm_speed(2, 0);
            set_pwm_speed(3, -right_speed);
        }
        else
        {
            set_pwm_speed(2, 0);
            set_pwm_speed(3, 0);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    
    return 0;
}