#include "ctre/phoenix6/TalonFX.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <functional> // Include this for std::bind

using namespace ctre::phoenix6;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("arm_test"),
          elbowMotor(1, "can0"),
          shoulderMotor(0, "can0"),
          shoulderOut(0),
          elbowOut(0)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

        configs::TalonFXConfiguration fx_cfg{};

        /* the left motor is CCW+ */
        fx_cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
        elbowMotor.GetConfigurator().Apply(fx_cfg);
        shoulderMotor.GetConfigurator().Apply(fx_cfg);
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
        std::cout << "shoulderSpeed: " << shoulder_speed << ", "
                  << " ElbowSpeed: " << elbow_speed << '\n';

        shoulderOut.Output = shoulder_speed;
        elbowOut.Output = elbow_speed;

        elbowMotor.SetControl(elbowOut);
        shoulderMotor.SetControl(shoulderOut);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;

    // Move these to class members
    hardware::TalonFX elbowMotor;
    hardware::TalonFX shoulderMotor;
    controls::DutyCycleOut shoulderOut;
    controls::DutyCycleOut elbowOut;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
