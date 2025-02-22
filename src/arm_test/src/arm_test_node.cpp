#include "ctre/phoenix6/TalonFX.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <functional> // Include this for std::bind4
#include "ctre/phoenix6/unmanaged/Unmanaged.hpp" // for FeedEnable

using namespace ctre::phoenix6;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("arm_test"),
          elbowMotor(4, "can0"),
          shoulderMotor(5, "can0"),
          shoulderOut(units::angular_velocity::turns_per_second_t(0)),
          elbowOut(units::angular_velocity::turns_per_second_t(0)),
          shoulder_speed(0), // Initialize speeds to 0
          elbow_speed(0)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
        
        ctre::phoenix::unmanaged::SetTransmitEnable(true);
        // fx_cfg is the default config with inverted output
        configs::TalonFXConfiguration fx_cfg{};

        /* the left motor is CCW+ */
        fx_cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
        elbowMotor.GetConfigurator().Apply(fx_cfg);
        fx_cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
        shoulderMotor.GetConfigurator().Apply(fx_cfg);

        // Corrected class name
        timer_shoulder = this->create_wall_timer(
            10ms, std::bind(&MinimalSubscriber::timer_callback_shoulder, this)); // Reduced delay for smoother control
        

        configs::TalonFXConfiguration cfg{};

        /* Configure gear ratio */
        configs::FeedbackConfigs &fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1; // 12.8 rotor rotations per mechanism rotation
        
        /* Configure Motion Magic */
        configs::MotionMagicConfigs &mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = units::angular_velocity::turns_per_second_t(20).value(); // 5 (mechanism) rotations per second cruise
        mm.MotionMagicAcceleration = units::angular_acceleration::turns_per_second_squared_t(100).value(); // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        mm.MotionMagicJerk = units::angular_jerk::turns_per_second_cubed_t(100).value();

        configs::Slot0Configs &slot0 = cfg.Slot0;
        slot0.kS = 0.0; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = .01; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
        
        ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
        ctre::phoenix::StatusCode statusS = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
        // Configure elbow motor
        for (int i = 0; i < 5; ++i) {
            status = elbowMotor.GetConfigurator().Apply(cfg);
            if (status.IsOK()) break;
        }
        if (!status.IsOK()) {
            std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
        }

        // Configure shoulder motor
        for (int i = 0; i < 5; ++i) {
            statusS = shoulderMotor.GetConfigurator().Apply(cfg);
            if (statusS.IsOK()) break;
        }
        if (!statusS.IsOK()) {
            std::cout << "Could not configure device. Error: " << statusS.GetName() << std::endl;
        }
        
    }

private:

    void timer_callback_shoulder()
    {
        ctre::phoenix::unmanaged::FeedEnable(10);
        elbowMotor.SetControl(elbowOut.WithVelocity(elbow_speed * 1_tps).WithSlot(0));
        //std::cout << "Vel: " << shoulderMotor.GetVelocity() << std::endl;
        shoulderMotor.SetControl(shoulderOut.WithVelocity(shoulder_speed * 1_tps).WithSlot(0));
    }

    void topic_callback(const std_msgs::msg::Float32MultiArray &msg)
    {
        if (msg.data.size() < 3)
        {
            RCLCPP_WARN(this->get_logger(), "Received data is too short!");
            return;
        }

        // Store values as class members for periodic updates
        elbow_speed = static_cast<double>(msg.data[1]* 20_tps); // Go for plus/minus 1 rotations per second
        
        shoulder_speed = static_cast<double>(msg.data[0]* 20_tps); // Go for plus/minus 1 rotations per second
        
        //std::cout << "Received joystick input - Shoulder: " << shoulder_speed
                  //<< ", Elbow: " << elbow_speed << '\n';
    }

    rclcpp::TimerBase::SharedPtr timer_elbow;
    rclcpp::TimerBase::SharedPtr timer_shoulder;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;

    hardware::TalonFX elbowMotor;
    hardware::TalonFX shoulderMotor;
    controls::MotionMagicVelocityDutyCycle shoulderOut;
    controls::MotionMagicVelocityDutyCycle elbowOut;

    // Moved speeds to class members
    double shoulder_speed;
    double elbow_speed;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
