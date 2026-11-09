#include "pinchy_hardware/feetech_system.hpp"
#include "SMS_STS.h"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <set>
#include <stdexcept>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pinchy_hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
static constexpr double radians_per_tick = 6.283185307179586 / 4096.0;
static const auto logger = rclcpp::get_logger("pinchy_hardware");
static void require(bool condition, const std::string & message)
{ if (!condition) throw std::runtime_error(message); }
static int integer(const std::string & text)
{
  size_t consumed = 0; int value = std::stoi(text, &consumed);
  require(consumed == text.size(), "Expected an integer, got: " + text); return value;
}
static double real(const std::string & text)
{
  size_t consumed = 0; double value = std::stod(text, &consumed);
  require(consumed == text.size(), "Expected a number, got: " + text); return value;
}

double Calibration::to_radians(int ticks, int feedback_tolerance) const
{
  require(feedback_tolerance >= 0 && feedback_tolerance <= 10 && ticks >= 0 && ticks <= 4095 &&
          ticks >= minimum - feedback_tolerance && ticks <= maximum + feedback_tolerance,
          "Measured ticks outside calibrated bounds");
  return direction * (ticks - zero) * radians_per_tick;
}
int Calibration::to_ticks(double radians) const
{
  require(std::isfinite(radians) && radians >= lower && radians <= upper,
          "Command outside calibrated joint bounds");
  const auto ticks = std::lround(zero + direction * radians / radians_per_tick);
  require(ticks >= minimum && ticks <= maximum, "Command crosses calibrated encoder bounds");
  return static_cast<int>(ticks);
}
void Calibration::validate() const
{
  require(id >= 1 && id <= 253 && (direction == 1 || direction == -1), "Invalid servo ID/direction");
  require(minimum >= 0 && maximum <= 4095 && minimum < maximum && zero >= minimum && zero <= maximum,
          "Invalid calibrated encoder range/zero; wrapping is unsupported");
  require(std::isfinite(lower) && std::isfinite(upper) && lower < upper &&
          std::isfinite(max_velocity) && max_velocity > 0 && max_velocity <= 1,
          "Invalid calibrated radian limits/velocity (maximum 1 rad/s)");
  to_ticks(lower); to_ticks(upper);
}

// Reuse the official protocol layer; replace only its Linux serial transport.
class CheckedPort : public SMS_STS
{
public:
  CheckedPort(const std::string & device, int timeout) : timeout_(timeout)
  {
    descriptor_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    require(descriptor_ >= 0, "Cannot open servo port: " + device);
    try {
      require(::flock(descriptor_, LOCK_EX | LOCK_NB) == 0, "Servo port is owned by another process");
      termios options{};
      require(tcgetattr(descriptor_, &options) == 0, "Cannot read serial attributes");
      cfmakeraw(&options);
      options.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB | CSIZE);
      options.c_cflag |= CS8 | CLOCAL | CREAD;
      cfsetispeed(&options, B1000000); cfsetospeed(&options, B1000000);
      require(tcsetattr(descriptor_, TCSANOW, &options) == 0, "Cannot configure 1 Mbaud port");
    } catch (...) { ::close(descriptor_); descriptor_ = -1; throw; }
  }
  ~CheckedPort() { if (descriptor_ >= 0) ::close(descriptor_); }
  int byte(int id, int address)
  { auto value = readByte(id, address); checked(value >= 0, id); return value; }
  int word(int id, int address)
  { auto value = readWord(id, address); checked(value >= 0, id); return value; }
  void set_byte(int id, int address, int value)
  { checked(writeByte(id, address, value) == 1, id); }
  void set_word(int id, int address, int value)
  { checked(writeWord(id, address, value) == 1, id); }
  void checked(bool okay, int id)
  {
    require(okay && getLastError() == 0 && getState() == 0,
            "ID " + std::to_string(id) + ": SDK communication/status fault (communication=" +
            std::to_string(getLastError()) + ", status=" + std::to_string(getState()) + ")");
  }
protected:
  int writeSCS(unsigned char * bytes, int size) override
  { pending_.insert(pending_.end(), bytes, bytes + size); return size; }
  int writeSCS(unsigned char byte) override { pending_.push_back(byte); return 1; }
  void rFlushSCS() override
  {
    pending_.clear();
    deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_);
    require(tcflush(descriptor_, TCIFLUSH) == 0, "Serial input flush failed");
  }
  bool ready(short events)
  {
    for (;;) {
      auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline_ - std::chrono::steady_clock::now()).count();
      if (remaining <= 0) return false;
      pollfd pfd{descriptor_, events, 0};
      int result = ::poll(&pfd, 1, static_cast<int>(remaining));
      if (result < 0 && errno == EINTR) continue;
      require(result >= 0 && !(pfd.revents & (POLLERR | POLLHUP | POLLNVAL)), "Serial port disconnected");
      return result > 0 && (pfd.revents & events);
    }
  }
  void wFlushSCS() override
  {
    size_t sent = 0;
    while (sent < pending_.size()) {
      require(ready(POLLOUT), "Serial write timeout");
      auto size = ::write(descriptor_, pending_.data() + sent, pending_.size() - sent);
      if (size < 0 && (errno == EINTR || errno == EAGAIN)) continue;
      require(size > 0, "Serial write failed"); sent += size;
    }
    pending_.clear();
  }
  int readSCS(unsigned char * bytes, int size) override
  {
    int received = 0;
    while (received < size && ready(POLLIN)) {
      auto count = ::read(descriptor_, bytes + received, size - received);
      if (count < 0 && (errno == EINTR || errno == EAGAIN)) continue;
      require(count > 0, "Serial read failed"); received += count;
    }
    return received;
  }
  int readSCS(unsigned char * bytes, int size, unsigned long) override
  { return readSCS(bytes, size); }
private:
  int descriptor_ = -1, timeout_;
  std::vector<unsigned char> pending_;
  std::chrono::steady_clock::time_point deadline_;
};

FeetechSystem::FeetechSystem() = default;
FeetechSystem::~FeetechSystem() { release(); }

CallbackReturn FeetechSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  try {
    const auto & p = info_.hardware_parameters;
    require(p.at("calibration_confirmed") == "true" && p.at("motion_enabled") == "true",
            "Hardware requires reviewed calibration and explicit motion_enabled=true");
    device_ = p.at("port"); require(!device_.empty(), "Missing serial device");
    const auto adoption = p.find("adopt_zero_hold");
    adopt_zero_hold_ = adoption != p.end() && adoption->second == "true";
    timeout_ms_ = integer(p.at("timeout_ms"));
    speed_ = integer(p.at("speed")); acceleration_ = integer(p.at("acceleration"));
    torque_limit_ = integer(p.at("torque_limit"));
    max_feedback_age_ = real(p.at("max_feedback_age"));
    require(timeout_ms_ >= 1 && timeout_ms_ <= 100 && speed_ >= 1 && speed_ <= 1000 &&
            acceleration_ >= 1 && acceleration_ <= 40 && torque_limit_ >= 1 && torque_limit_ <= 1000 &&
            std::isfinite(max_feedback_age_) && max_feedback_age_ >= 0.05 && max_feedback_age_ <= 1,
            "Invalid timing/speed/acceleration/torque settings");
    require(!info_.joints.empty() && info_.joints.size() <= 6, "Expected 1 to 6 joints");
    std::set<int> ids;
    for (const auto & joint : info_.joints) {
      require(joint.command_interfaces.size() == 1 && joint.command_interfaces[0].name == "position" &&
              joint.state_interfaces.size() == 1 && joint.state_interfaces[0].name == "position",
              "Only measured position state and position command interfaces are supported");
      const auto & j = joint.parameters;
      Calibration c{integer(j.at("servo_id")), integer(j.at("zero_tick")),
        integer(j.at("direction")), integer(j.at("min_tick")), integer(j.at("max_tick")),
        real(j.at("min_radians")), real(j.at("max_radians")), real(j.at("max_velocity"))};
      c.validate(); require(ids.insert(c.id).second, "Duplicate servo ID");
      calibration_.push_back(c);
    }
    size_t n = calibration_.size();
    positions_.resize(n); commands_.resize(n); previous_commands_.resize(n);
    ticks_.resize(n); goal_ticks_.resize(n); voltages_.resize(n); temperatures_.resize(n);
    hot_counts_.assign(n, 0);
    if (rclcpp::ok()) {
      auto options = rclcpp::NodeOptions().use_global_arguments(false).start_parameter_services(false);
      safety_node_ = std::make_shared<rclcpp::Node>("pinchy_hardware_safety", options);
      auto qos = rclcpp::QoS(1).transient_local().reliable();
      safety_status_ = safety_node_->create_publisher<std_msgs::msg::String>("/arm_safety/status", qos);
      held_publisher_ = safety_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_safety/held_positions", qos);
      reset_service_ = safety_node_->create_service<std_srvs::srv::Trigger>("/arm_safety/reset",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = false;
          response->message = "Recovery requires a safety hold, four healthy feedback cycles, and a replacement hold at the last accepted command";
          if (!active_ || !faulted_ || healthy_cycles_ < 4 || held_commands_.size() != commands_.size() ||
              std::chrono::duration<double>(std::chrono::steady_clock::now()-last_feedback_).count() > max_feedback_age_) return;
          for (size_t i=0; i<commands_.size(); ++i) {
            if (!std::isfinite(commands_[i]) || std::abs(commands_[i]-held_commands_[i]) > 1e-7 ||
                temperatures_[i] >= 60 || hot_counts_[i] != 0) return;
          }
          previous_commands_ = commands_;
          last_write_ = std::chrono::steady_clock::now();
          faulted_ = false;
          response->success = true; response->message = "Recovered to powered hold; old trajectory discarded";
          announce("READY");
        });
      safety_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      safety_executor_->add_node(safety_node_);

      pause_publisher_ = safety_node_->create_publisher<control_msgs::msg::SpeedScalingFactor>(
        "/arm_controller/speed_scaling_input", qos);
    }
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Configuration rejected: %s", e.what()); return CallbackReturn::ERROR;
  }
}
std::vector<hardware_interface::StateInterface> FeetechSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  for (size_t i = 0; i < positions_.size(); ++i)
    result.emplace_back(info_.joints[i].name, "position", &positions_[i]);
  return result;
}
std::vector<hardware_interface::CommandInterface> FeetechSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  for (size_t i = 0; i < commands_.size(); ++i)
    result.emplace_back(info_.joints[i].name, "position", &commands_[i]);
  return result;
}
void FeetechSystem::sample(bool require_torque)
{
  const auto sample_started = std::chrono::steady_clock::now();
  std::vector<double> measured(positions_.size());
  for (size_t i = 0; i < calibration_.size(); ++i) {
    auto id = calibration_[i].id;
    // One snapshot includes torque, actual goal and health/position feedback.
    // Six transactions per sweep instead of twelve reduces USB turnaround gaps.
    unsigned char data[26]{}; // Registers 40..65
    port_->checked(port_->Read(id, 40, data, 26) == 26, id);
    require(data[25] == 0, "ID " + std::to_string(id) + ": servo alarm register nonzero");
    ticks_[i] = data[16] | (data[17] << 8);
    goal_ticks_[i] = data[2] | (data[3] << 8);
    // Commands remain strictly bounded; measurements may differ by up to ten
    // encoder ticks because of servo deadband/quantization at a boundary goal.
    measured[i] = calibration_[i].to_radians(ticks_[i], 10);
    require(measured[i] >= calibration_[i].lower - 10*radians_per_tick - 1e-12 &&
            measured[i] <= calibration_[i].upper + 10*radians_per_tick + 1e-12,
            "Measured joint angle outside calibrated bounds");
    voltages_[i] = data[22] / 10.0; temperatures_[i] = data[23];
    require(voltages_[i] >= 10 && voltages_[i] <= 13, "Servo supply outside 10..13 V");
    if (temperatures_[i] >= 60) {
      const int first = data[23], direct = port_->byte(id, 63), block = port_->word(id, 62);
      const int repeated = block >> 8, status = port_->byte(id, 65);
      const bool cool = direct < 60 && repeated < 60 && std::abs(direct-repeated) <= 2 && status == 0;
      hot_counts_[i] = cool ? 0 : std::min(4, hot_counts_[i]+1);
      if (!faulted_) RCLCPP_WARN(logger, "ID %d temperature check %d/4: initial=%d C, direct=%d C, block=%d C, status=0x%02x; %s",
                  id, hot_counts_[i], first, direct, repeated, status,
                  cool ? "isolated high sample, rechecks cool" : "suspect reading");
      require(status == 0, "ID " + std::to_string(id) + ": servo alarm during thermal recheck");
      require(faulted_ || hot_counts_[i] < 4, "ID " + std::to_string(id) + ": four consecutive suspect temperature checks; initial=" +
              std::to_string(first) + ", direct=" + std::to_string(direct) + ", block=" +
              std::to_string(repeated) + ", status=" + std::to_string(status));
      require((block & 255) >= 100 && (block & 255) <= 130, "Voltage recheck outside 10..13 V");
      temperatures_[i] = direct;
    } else hot_counts_[i] = 0;
    require(data[0] == (require_torque ? 1 : 0), "Unexpected servo torque state");
  }
  require(std::chrono::duration<double>(std::chrono::steady_clock::now() - sample_started).count()
          <= max_feedback_age_, "Feedback sweep exceeded maximum age");
  std::copy(measured.begin(), measured.end(), positions_.begin());
  // Timestamp the oldest observation, not the end of the sequential sweep.
  last_feedback_ = sample_started;
}
CallbackReturn FeetechSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    port_ = std::make_unique<CheckedPort>(device_, timeout_ms_);
    for (const auto & c : calibration_) {
      require(port_->word(c.id, 3) == 777, "Expected STS3215 model 777");
      require(port_->byte(c.id, 5) == c.id && port_->byte(c.id, 33) == 0 && port_->byte(c.id, 30) == 1,
              "Require matching ID, position mode 0, resolution 1; configure with bench tools first");
    }
    sample(adopt_zero_hold_);
    if (adopt_zero_hold_) {
      verify_zero_hold();
      std::fill(commands_.begin(), commands_.end(), 0.0);
    } else {
      for (size_t i = 0; i < positions_.size(); ++i) calibration_[i].to_ticks(positions_[i]);
      commands_ = positions_;
    }
    previous_commands_ = commands_;
    faulted_ = false; return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    // No enable/write has occurred in configure. Leave existing external state alone.
    RCLCPP_ERROR(logger, "Configure failed: %s", e.what()); port_.reset(); return CallbackReturn::ERROR;
  }
}
void FeetechSystem::verify_zero_hold()
{
  for (size_t i = 0; i < calibration_.size(); ++i) {
    const auto & c = calibration_[i];
    c.to_ticks(0.0);
    require(std::abs(ticks_[i] - c.zero) <= 10 && port_->word(c.id, 42) == c.zero,
            "Zero-hold handoff requires each measured position near zero and goal at its calibrated zero");
    require(port_->word(c.id, 48) == torque_limit_, "Zero-hold handoff torque limit mismatch");
  }
}
CallbackReturn FeetechSystem::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    require(port_ && !faulted_, "Cleanup and reconfigure hardware after a fault");
    if (adopt_zero_hold_) {
      sample(true); verify_zero_hold();
      std::fill(commands_.begin(), commands_.end(), 0.0);
      previous_commands_ = commands_; last_write_ = std::chrono::steady_clock::now(); active_ = true;
      announce("READY");
      RCLCPP_INFO(logger, "Adopted six calibrated zero holds without disabling torque");
      return CallbackReturn::SUCCESS;
    }
    sample(false); commands_ = positions_; previous_commands_ = positions_;
    for (size_t i = 0; i < positions_.size(); ++i) calibration_[i].to_ticks(positions_[i]);
    for (size_t i = 0; i < calibration_.size(); ++i) {
      auto id = calibration_[i].id;
      port_->set_word(id, 48, torque_limit_);
      require(port_->word(id, 48) == torque_limit_, "Torque RAM limit readback mismatch");
      port_->checked(port_->WritePosEx(id, ticks_[i], speed_, acceleration_) == 1, id);
      require(port_->word(id, 42) == ticks_[i], "Startup hold goal readback mismatch");
    }
    for (const auto & c : calibration_) port_->set_byte(c.id, 40, 1);
    sample(true);
    for (size_t i = 0; i < positions_.size(); ++i)
      require(std::abs(positions_[i] - commands_[i]) <= 5 * radians_per_tick,
              "Startup hold moved by more than 5 ticks");
    last_write_ = std::chrono::steady_clock::now(); active_ = true;
    announce("READY");
    RCLCPP_INFO(logger, "Hardware active at measured pose; no automatic home movement");
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) { fault(e); return CallbackReturn::ERROR; }
}
void FeetechSystem::release() noexcept
{
  active_ = false;
  if (port_) for (const auto & c : calibration_) {
    try { port_->set_byte(c.id, 40, 0); }
    catch (const std::exception & e) { RCLCPP_ERROR(logger, "Release ID %d unconfirmed: %s", c.id, e.what()); }
  }
}
return_type FeetechSystem::fault(const std::exception & e)
{
  if (active_) {
    if (!faulted_) {
      healthy_cycles_ = 0;
      held_commands_ = previous_commands_;
      try {
        if (held_publisher_) { std_msgs::msg::Float64MultiArray held; held.data = held_commands_; held_publisher_->publish(held); }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(logger, "Could not publish held positions: %s", error.what());
      }
      faulted_ = true;
      RCLCPP_ERROR(logger, "Safety HOLD: %s. Movement latched off; leaving torque and existing servo goals unchanged. Correct the cause, then press Stop / hold to recover.", e.what());
      announce(std::string("HOLD: ")+e.what());
      if (pause_publisher_) {
        try {
          control_msgs::msg::SpeedScalingFactor pause; pause.factor = 0;
          pause_publisher_->publish(pause);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(logger, "Could not publish trajectory pause: %s; motor writes remain blocked", error.what());
        }
      }
    }
    healthy_cycles_ = 0;
    // Returning ERROR would make controller_manager deactivate the hardware
    // and release a gravity-loaded arm. Stay readable but never write new goals.
    return return_type::OK;
  }
  RCLCPP_ERROR(logger, "Hardware fault: %s. Releasing torque; recovery requires reconfiguration.", e.what());
  faulted_ = true; release(); return return_type::ERROR;
}
void FeetechSystem::announce(const std::string & message)
{
  try {
    if (safety_status_) { std_msgs::msg::String status; status.data = message; safety_status_->publish(status); }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Could not publish safety status: %s", error.what());
  }
}
CallbackReturn FeetechSystem::on_deactivate(const rclcpp_lifecycle::State &)
{ release(); return CallbackReturn::SUCCESS; }
CallbackReturn FeetechSystem::on_cleanup(const rclcpp_lifecycle::State &)
{ release(); port_.reset(); return CallbackReturn::SUCCESS; }
CallbackReturn FeetechSystem::on_error(const rclcpp_lifecycle::State &)
{ faulted_ = true; release(); port_.reset(); return CallbackReturn::SUCCESS; }
return_type FeetechSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) return faulted_ ? return_type::ERROR : return_type::OK;
  try {
    sample(true);
    bool healthy = true;
    for (size_t i=0; i<temperatures_.size(); ++i) healthy = healthy && temperatures_[i] < 60 && hot_counts_[i] == 0;
    healthy_cycles_ = healthy ? std::min(4, healthy_cycles_+1) : 0;
    if (safety_executor_) safety_executor_->spin_some(std::chrono::nanoseconds(0));
    return return_type::OK;
  } catch (const std::exception & e) {
    const auto result = fault(e);
    if (safety_executor_) safety_executor_->spin_some(std::chrono::nanoseconds(0));
    return result;
  }
}
return_type FeetechSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (active_ && faulted_) return return_type::OK; // Retain existing powered holds; no automatic recovery.
  if (!active_) return faulted_ ? return_type::ERROR : return_type::OK;
  try {
    const auto written_at = std::chrono::steady_clock::now();
    double age = std::chrono::duration<double>(written_at - last_feedback_).count();
    const double elapsed = std::chrono::duration<double>(written_at - last_write_).count();
    require(age <= max_feedback_age_, "Measured feedback is stale");
    require(std::isfinite(period.seconds()) && period.seconds() > 0 && period.seconds() <= max_feedback_age_,
            "Invalid or stalled control period");
    require(elapsed <= max_feedback_age_, "Actual hardware write interval stalled");
    // Controller sampling can be delayed by read/OS scheduling while the supplied
    // period remains nominal. Check against the real interval too, including
    // cycles with no changed encoder goal; idle time cannot accumulate credit.
    const double command_interval = std::max(period.seconds(), elapsed);
    std::vector<u8> ids, accelerations; std::vector<s16> goals; std::vector<u16> speeds;
    for (size_t i = 0; i < commands_.size(); ++i) {
      const auto & c = calibration_[i];
      auto target = c.to_ticks(commands_[i]);
      require(std::abs(commands_[i] - previous_commands_[i]) <=
              c.max_velocity * command_interval + 2 * radians_per_tick,
              "ID " + std::to_string(c.id) + " (" + info_.joints[i].name +
              "): command exceeds calibrated joint velocity/step limit; previous=" +
              std::to_string(previous_commands_[i]) + ", requested=" + std::to_string(commands_[i]) +
              " rad, period=" + std::to_string(period.seconds()) +
              " s, actual_interval=" + std::to_string(elapsed) +
              " s, max_velocity=" + std::to_string(c.max_velocity) + " rad/s");
      // Verify through readback: a dropped broadcast remains different next
      // cycle and is retried. Avoid reissuing an already-held encoder target.
      if (target != goal_ticks_[i]) {
        ids.push_back(c.id); goals.push_back(target); speeds.push_back(speed_); accelerations.push_back(acceleration_);
      }
    }
    if (!ids.empty()) port_->SyncWritePosEx(ids.data(), ids.size(), goals.data(), speeds.data(), accelerations.data());
    previous_commands_ = commands_;
    last_write_ = written_at;
    return return_type::OK;
  } catch (const std::exception & e) { return fault(e); }
}
}  // namespace pinchy_hardware
PLUGINLIB_EXPORT_CLASS(pinchy_hardware::FeetechSystem, hardware_interface::SystemInterface)
