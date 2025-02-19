#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include <SDL2/SDL.h>

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using std::placeholders::_1;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("arm_test"),
          shoulderMotor(0, "can0"), // Initialize TalonSRX with ID 0 on CAN bus "can0"
          elbowMotor(1, "can0") // Assume elbowMotor has a different ID (1)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray &msg)
    {
        if (msg.data.size() < 3)
        {
            RCLCPP_WARN(this->get_logger(), "Received data is too short!");
            return;
        }

        double shoulder_speed = msg.data[1]; // SDL_CONTROLLER_AXIS_LEFTY
        double elbow_speed = msg.data[2];    // SDL_CONTROLLER_AXIS_LEFTY

        shoulderMotor.Set(ControlMode::PercentOutput, shoulder_speed);
        elbowMotor.Set(ControlMode::PercentOutput, elbow_speed);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;

    // Declare TalonSRX objects as class members
    TalonSRX shoulderMotor;
    TalonSRX elbowMotor;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
