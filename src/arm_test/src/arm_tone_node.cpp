#include "ctre/phoenix6/TalonFX.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
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
          elbowMotor(5, "can0"),
	  elbowOut(elbowOut.WithAudioFrequency(units::frequency::hertz_t(static_cast<double>(0.0)))),
    	  tone_heard(0.0)
    {
	   
        timer_shoulder = this->create_wall_timer(
            10ms, std::bind(&MinimalSubscriber::timer_callback, this)); // Reduced delay for smoother control
	    
        elbowOut.WithUpdateFreqHz(units::frequency::hertz_t(static_cast<double>(20.0))),
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "tone_freq", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
        
        // skip all the boring configuration stuff, surely it isn't important
    }

private:
    void topic_callback(const std_msgs::msg::Float32 msg)
    {
	std::cout << "Tone heard: " << msg.data << std::endl;
    	tone_heard = msg.data;
    }

    void timer_callback()
    {
        ctre::phoenix::unmanaged::FeedEnable(10);
        auto freq = units::frequency::hertz_t(static_cast<double>(tone_heard));
    	elbowMotor.SetControl(elbowOut.WithAudioFrequency(freq));

	//std::cout << "Vel: " << elbowMotor.GetVelocity() << std::endl;
        //std::cout << "Pos: " << elbowMotor.GetPosition() << std::endl;

    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_shoulder;
    hardware::TalonFX elbowMotor;
    controls::MusicTone elbowOut;
    double tone_heard;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
