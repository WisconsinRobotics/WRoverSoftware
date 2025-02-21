#include "ctre/phoenix6/TalonFX.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <functional> // Include this for std::bind4
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp" // for FeedEnable
#include <ctre/phoenix6/controls/MusicTone.hpp> // for music! I'm opting not to use orchestra because i don't want to deal with chirp files

using namespace ctre::phoenix6;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("arm_tone"),
          elbowMotor(1, "can0"),
          shoulderMotor(0, "can0"),
          shoulderOut(0),
          elbowOut(0),
    {
        subscription_ = this->create_subscription<std_msgs::msg::Int16>(
            "tone_freq", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
        
        // skip all the boring configuration stuff, surely it isn't important
    }

private:
    void topic_callback(const std_msgs::msg::Int16 msg)
    {
        // play the tone on a motor
        elbowMotor.SetControl(elbowOut.WithAudioFrequency(msg))
    }

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;

    hardware::TalonFX elbowMotor;
    hardware::TalonFX shoulderMotor;
    controls::MusicTone shoulderOut;
    controls::MusicTone elbowOut;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
