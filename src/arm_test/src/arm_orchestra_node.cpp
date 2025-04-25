#include "ctre/phoenix6/TalonFX.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include <functional> // Include this for std::bind4
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp" // for FeedEnable
#include <ctre/phoenix6/Orchestra.hpp> // for music!

using namespace ctre::phoenix6;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("arm_tone"),
          elbowMotor(1, "can0"),
          shoulderMotor(0, "can0"),
          m_orchestra("onwisc.chrp")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "buttons", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
        
	m_orchestra.AddInstrument(elbowMotor);
	m_orchestra.AddInstrument(shoulderMotor);
        // skip all the boring configuration stuff, surely it isn't important
    }

private:
    void topic_callback(const std_msgs::msg::Int16MultiArray msg)
    {
        // play the tone on a motor
        if(msg.data[5]==1 && !m_orchestra.IsPlaying()){
            m_orchestra.Play();
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;

    hardware::TalonFX elbowMotor;
    hardware::TalonFX shoulderMotor;
    Orchestra m_orchestra;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
