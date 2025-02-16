#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "ctre/phoenix6/TalonFX.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("arm_test")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "arm", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

        static constexpr char const *CANBUS_NAME = "*";

        /* Only one left and one right motor */
        shoulderMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(0, CANBUS_NAME);
        elbowMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(1, CANBUS_NAME);

        /* control requests */
        shoulderOut = std::make_unique<ctre::phoenix6::controls::DutyCycleOut>(0);
        elbowOut = std::make_unique<ctre::phoenix6::controls::DutyCycleOut>(0);

        ctre::phoenix6::configs::TalonFXConfiguration fx_cfg{};

        /* Shoulder motor is CCW+ */
        fx_cfg.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
        shoulderMotor->GetConfigurator().Apply(fx_cfg);

        /* Elbow motor is CW+ */
        fx_cfg.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
        elbowMotor->GetConfigurator().Apply(fx_cfg);
    }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray &msg) const
    {
        if (msg.data.size() < 3)
        {
            RCLCPP_WARN(this->get_logger(), "Received data is too short!");
            return;
        }

        double shoulder_speed = msg.data[1]; // SDL_CONTROLLER_AXIS_LEFTY
        double elbow_speed = msg.data[2];    // SDL_CONTROLLER_AXIS_LEFTY

        shoulderOut->Output = shoulder_speed;
        elbowOut->Output = elbow_speed;

        shoulderMotor->SetControl(*shoulderOut);
        elbowMotor->SetControl(*elbowOut);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> shoulderMotor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> elbowMotor;
    std::unique_ptr<ctre::phoenix6::controls::DutyCycleOut> shoulderOut;
    std::unique_ptr<ctre::phoenix6::controls::DutyCycleOut> elbowOut;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}

