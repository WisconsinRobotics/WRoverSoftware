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
          elbowMotor(1, "can0"),
          shoulderMotor(0, "can0"),
          shoulderOut(0),
          elbowOut(units::angular_velocity::turns_per_second_t(0)),
          shoulder_speed(0), // Initialize speeds to 0
          elbow_speed(0)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

        configs::TalonFXConfiguration fx_cfg{};

        /* the left motor is CCW+ */
        fx_cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
        elbowMotor.GetConfigurator().Apply(fx_cfg);
        shoulderMotor.GetConfigurator().Apply(fx_cfg);

        // Corrected class name
        timer_ = this->create_wall_timer(
            10ms, std::bind(&MinimalSubscriber::timer_callback, this)); // Reduced delay for smoother control
        
        configs::TalonFXConfiguration cfg{};

        /* Configure gear ratio */
        configs::FeedbackConfigs &fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1; // 12.8 rotor rotations per mechanism rotation
        
        /* Configure Motion Magic */
        configs::MotionMagicConfigs &mm = cfg.MotionMagic;
        //mm.MotionMagicCruiseVelocity = 2_tps; // 5 (mechanism) rotations per second cruise
        //mm.MotionMagicAcceleration = 5_tr_per_s_sq; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        //mm.MotionMagicJerk = 100_tr_per_s_cu;
        
        configs::Slot0Configs &slot0 = cfg.Slot0;
        slot0.kS = 0.0; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = .3; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
        
        ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = elbowMotor.GetConfigurator().Apply(cfg);
            if (status.IsOK()) break;
        }
        if (!status.IsOK()) {
            std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
        }
    }

private:
    void timer_callback()
    {
        ctre::phoenix::unmanaged::FeedEnable(20);
        shoulderOut.Output = shoulder_speed;
        // Convert Output to double before printing
        std::cout << "Sending to motors - Shoulder: " << static_cast<double>(shoulderOut.Output)
                   << ", Elbow: " << static_cast<double>(elbow_speed) << '\n';
        std::cout << elbowMotor.SetControl(elbowOut.WithVelocity(units::angular_velocity::turns_per_second_t(elbow_speed)).WithSlot(0).WithFeedForward(units::voltage::volt_t(0))) << std::endl;
        std::cout << "Pos: " << elbowMotor.GetPosition() << std::endl;
        std::cout << "Vel: " << elbowMotor.GetVelocity() << std::endl;
    }

    void topic_callback(const std_msgs::msg::Float32MultiArray &msg)
    {
        if (msg.data.size() < 3)
        {
            RCLCPP_WARN(this->get_logger(), "Received data is too short!");
            return;
        }

        // Store values as class members for periodic updates
        shoulder_speed = msg.data[1];
        elbow_speed = static_cast<double>(msg.data[2]*5); // Go for plus/minus 1 rotations per second
        
        //std::cout << "Received joystick input - Shoulder: " << shoulder_speed
                  //<< ", Elbow: " << elbow_speed << '\n';
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;

    hardware::TalonFX elbowMotor;
    hardware::TalonFX shoulderMotor;
    controls::DutyCycleOut shoulderOut;
    controls::MotionMagicVelocityVoltage elbowOut;

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
