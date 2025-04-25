#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "std_msgs/msg/float64.hpp"


using namespace ctre::phoenix6;
using namespace std::chrono_literals;


class CompassDataPublisher : public rclcpp::Node {
    public:
        CompassDataPublisher()
        :
            Node("compass_data"),
            pigeon2imu(10, "can0")
        {
            publisher_ = this->create_publisher<std_msgs::msg::Float64>("compass_data_topic", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&CompassDataPublisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::Float64(); // this is potentially wrong
            message.data = pigeon2imu.GetAngle();
            publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
        hardware::Pigeon2 pigeon2imu;
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompassDataPublisher>());
    rclcpp::shutdown();
    return 0;
}