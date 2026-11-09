#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "control_msgs/msg/speed_scaling_factor.hpp"

namespace pinchy_hardware
{
struct Calibration
{
  int id, zero, direction, minimum, maximum;
  double lower, upper, max_velocity;
  double to_radians(int ticks, int feedback_tolerance = 0) const;
  int to_ticks(double radians) const;
  void validate() const;
};
class CheckedPort;
class FeetechSystem : public hardware_interface::SystemInterface
{
public:
  FeetechSystem();
  ~FeetechSystem() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  void sample(bool require_torque);
  void verify_zero_hold();
  void release() noexcept;
  void announce(const std::string & message);
  hardware_interface::return_type fault(const std::exception &);
  std::unique_ptr<CheckedPort> port_;
  std::vector<Calibration> calibration_;
  std::vector<double> positions_, commands_, previous_commands_, voltages_, temperatures_;
  std::vector<int> ticks_, goal_ticks_, hot_counts_;
  std::shared_ptr<rclcpp::Node> safety_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_status_;
  rclcpp::Publisher<control_msgs::msg::SpeedScalingFactor>::SharedPtr pause_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr held_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> safety_executor_;
  std::vector<double> held_commands_;
  int healthy_cycles_ = 0;
  std::string device_;
  int timeout_ms_ = 20, speed_ = 100, acceleration_ = 5, torque_limit_ = 300;
  double max_feedback_age_ = 0.25;
  bool active_ = false, faulted_ = false, adopt_zero_hold_ = false;
  std::chrono::steady_clock::time_point last_feedback_, last_write_;
};
}  // namespace pinchy_hardware
