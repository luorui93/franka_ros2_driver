// Description: A node that subscribes to joint states and publishes joint impedance commands
#include <vector>
#include <array>
#include <mutex>
#include <cmath>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

class JointImpedanceTeleop : public rclcpp::Node
{
  public:
    JointImpedanceTeleop()
    : Node("joint_impedance_teleop")
    {
      subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/franka/joint_states", 1, std::bind(&JointImpedanceTeleop::state_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_effort_controller/commands", 1);
      timer_ = this->create_wall_timer(
        1ms, std::bind(&JointImpedanceTeleop::publish_command, this));
      message_.data.resize(7);
    }
    ~JointImpedanceTeleop()
    {
      is_state_initialized_ = false;
      std::fill(message_.data.begin(), message_.data.end(), 0.0);
      for (int i = 0; i < 7; i++)
      {
        publisher_->publish(message_);
        RCLCPP_INFO(this->get_logger(), "message: %f", message_.data[0]);
      }
      RCLCPP_INFO(this->get_logger(), "Shutting down");
      subscriber_.reset();
      publisher_.reset();
    }
  private:
    //TODO: Make this a realtime safe publisher using realtime tool
    void publish_command()
    {
      if (!is_state_initialized_)
      {
        return;
      }
      // RCLCPP_INFO(this->get_logger(), "Publishing command");
      std::array<double, 7> tau_d_calculated;
      auto time = this->get_clock()->now() - start_time_;
      auto delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time.seconds()));
      q_d_ = init_q_;
      q_d_.at(6) += delta_angle;
      q_d_.at(5) += delta_angle;

      const double kAlpha = 0.99;

      read_mutex_.lock();
      for (size_t i = 0; i < 7; i++)
      {
        dq_filtered_[i] = (1 - kAlpha) * dq_filtered_[i] + kAlpha * dq_[i];
        tau_d_calculated[i] = Kp_[i] * (q_d_[i] - q_[i]) - Kd_[i] * dq_filtered_[i];
      }
      read_mutex_.unlock();


      std::copy(tau_d_calculated.begin(), tau_d_calculated.end(), message_.data.begin());
      publisher_->publish(message_);

    }

    // TODO: Make this lock free using realtime tools
    void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::copy(msg->position.begin(), msg->position.end(), q_.begin());
      std::copy(msg->velocity.begin(), msg->velocity.end(), dq_.begin());
      std::copy(msg->effort.begin(), msg->effort.end(), tau_J_.begin());
      if (!is_state_initialized_)
      {
        init_q_ = q_;
        q_d_ = q_;
        start_time_ = this->get_clock()->now();
        is_state_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Initialized state");
      }
    }
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_state_initialized_ = false;

    // variables storing the robot state
    std::array<double, 7> q_;
    std::array<double, 7> q_d_;
    std::array<double, 7> dq_filtered_;
    std::array<double, 7> init_q_;
    std::array<double, 7> dq_;
    std::array<double, 7> tau_J_;
    rclcpp::Time start_time_;
    std_msgs::msg::Float64MultiArray message_;

    std::mutex read_mutex_;

    // variables for impedance gains
    const std::array<double, 7> Kp_ = {24, 24, 24, 24, 10, 6, 2};
    const std::array<double, 7> Kd_ = {2, 2, 2, 1, 1, 1, 0.5};

};  

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointImpedanceTeleop>());
  rclcpp::shutdown();
  return 0;
}
