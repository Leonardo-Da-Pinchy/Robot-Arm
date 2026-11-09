#include "pinchy_hardware/feetech_system.hpp"
#include <gtest/gtest.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <pty.h>
#include <thread>
#include <unistd.h>

using pinchy_hardware::Calibration;
using pinchy_hardware::FeetechSystem;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

// A pseudo-terminal emulates six servos. No test opens a physical USB device.
class Bus
{
public:
  std::string path;
  std::atomic<int> corrupt{0}, syncs{0}, eeprom_writes{0}, torque_off_writes{0};
  std::atomic<int> temperature_spike_id{0}, direct_temperature{-1}, read_requests{0};
  std::atomic<int> spike_temperature{65};
  std::atomic<bool> drop_next_sync{false};
  Bus()
  {
    char name[128]; EXPECT_EQ(openpty(&master_, &slave_, name, nullptr, nullptr), 0); path = name;
    for (int id = 1; id <= 6; ++id) {
      auto & r = regs_[id]; r.fill(0); r[3] = 9; r[4] = 3; r[5] = id;
      r[30] = 1; r[55] = 1; r[62] = 124; r[63] = 33;
      put_word(r, 56, 2050 + id); put_word(r, 42, 3000); // intentionally stale saved goal
    }
    worker_ = std::thread([this] { run(); });
  }
  ~Bus() { stop_ = true; worker_.join(); close(master_); close(slave_); }
  int byte(int id, int address) { std::lock_guard<std::mutex> g(mutex_); return regs_[id][address]; }
  void set(int id, int address, int value)
  { std::lock_guard<std::mutex> g(mutex_); regs_[id][address] = value; }
private:
  using Registers = std::array<unsigned char, 256>;
  int master_, slave_; std::atomic<bool> stop_{false}; std::thread worker_; std::mutex mutex_;
  std::array<Registers, 7> regs_{};
  static void put_word(Registers & r, int address, int value)
  { r[address] = value & 255; r[address + 1] = value >> 8; }
  void store(int id, int address, const unsigned char * data, int count)
  {
    if (address < 40) ++eeprom_writes;
    if (address == 40 && count == 1 && data[0] == 0) ++torque_off_writes;
    auto & r = regs_[id]; std::copy(data, data + count, r.begin() + address);
    if (r[40] && ((address <= 42 && address + count > 43) || address == 40)) {
      r[56] = r[42]; r[57] = r[43];
    }
  }
  void reply(int id, const std::vector<unsigned char> & data, int mode)
  {
    if (mode == 4) return; // no reply
    std::vector<unsigned char> packet{255, 255, static_cast<unsigned char>(mode == 2 ? id + 1 : id),
      static_cast<unsigned char>(data.size() + (mode == 3 ? 3 : 2)),
      static_cast<unsigned char>(mode == 5 ? 4 : 0)};
    packet.insert(packet.end(), data.begin(), data.end());
    unsigned char sum = 0; for (size_t i = 2; i < packet.size(); ++i) sum += packet[i];
    packet.push_back(static_cast<unsigned char>(~sum) ^ (mode == 1 ? 1 : 0));
    // Split frames to exercise partial serial reads.
    ::write(master_, packet.data(), 3);
    ::write(master_, packet.data() + 3, packet.size() - 3);
  }
  void run()
  {
    std::vector<unsigned char> buffer;
    while (!stop_) {
      pollfd p{master_, POLLIN, 0}; if (::poll(&p, 1, 10) <= 0) continue;
      unsigned char data[256]; auto n = ::read(master_, data, sizeof(data));
      if (n <= 0) continue; buffer.insert(buffer.end(), data, data + n);
      while (buffer.size() >= 4) {
        if (buffer[0] != 255 || buffer[1] != 255) { buffer.erase(buffer.begin()); continue; }
        size_t size = buffer[3] + 4; if (buffer.size() < size) break;
        std::vector<unsigned char> packet(buffer.begin(), buffer.begin() + size);
        buffer.erase(buffer.begin(), buffer.begin() + size);
        std::lock_guard<std::mutex> g(mutex_);
        int id = packet[2], instruction = packet[4];
        if (id == 254 && instruction == 0x83) {
          if (drop_next_sync.exchange(false)) { ++syncs; continue; }
          int address = packet[5], count = packet[6];
          for (size_t i = 7; i + count < size - 1; i += count + 1)
            store(packet[i], address, &packet[i + 1], count);
          ++syncs; continue;
        }
        if (id < 1 || id > 6) continue;
        int mode = corrupt.exchange(0);
        if (instruction == 2) {
          ++read_requests;
          int address = packet[5], count = packet[6];
          std::vector<unsigned char> payload(regs_[id].begin() + address, regs_[id].begin() + address + count);
          if (address <= 63 && address + count > 63 && id == temperature_spike_id.load()) {
            payload[63-address] = spike_temperature.load(); temperature_spike_id = 0;
          } else if (address == 63 && count == 1 && direct_temperature >= 0) {
            payload[0] = direct_temperature.load();
          }
          reply(id, payload, mode);
        } else if (instruction == 3) {
          store(id, packet[5], &packet[6], size - 7); reply(id, {}, mode);
        }
      }
    }
  }
};

hardware_interface::HardwareInfo info(const std::string & path)
{
  hardware_interface::HardwareInfo h; h.name = "TestArm"; h.type = "system";
  h.hardware_parameters = {{"port", path}, {"calibration_confirmed", "true"}, {"motion_enabled", "true"},
    {"timeout_ms", "20"}, {"speed", "100"}, {"acceleration", "5"}, {"torque_limit", "300"},
    {"max_feedback_age", "0.25"}};
  for (int i = 1; i <= 6; ++i) {
    hardware_interface::ComponentInfo j; j.name = "joint_" + std::to_string(i); j.type = "joint";
    hardware_interface::InterfaceInfo position; position.name = "position";
    j.command_interfaces.push_back(position); j.state_interfaces.push_back(position);
    j.parameters = {{"servo_id", std::to_string(i)}, {"zero_tick", "2048"}, {"direction", "1"},
      {"min_tick", "1024"}, {"max_tick", "3072"}, {"min_radians", "-1"},
      {"max_radians", "1"}, {"max_velocity", "0.3"}};
    h.joints.push_back(j);
  }
  return h;
}
const rclcpp_lifecycle::State state;
const rclcpp::Time now(0);
const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.05);

TEST(Calibration, DirectionRoundTripAndLimits)
{
  for (int direction : {-1, 1}) {
    Calibration c{1, 2048, direction, 1024, 3072, -1, 1, 0.3}; c.validate();
    for (double q : {-1.0, -0.5, 0.0, 0.5, 1.0}) EXPECT_NEAR(c.to_radians(c.to_ticks(q)), q, 0.0008);
    EXPECT_THROW(c.to_ticks(NAN), std::runtime_error);
    EXPECT_THROW(c.to_ticks(1.01), std::runtime_error);
    EXPECT_THROW(c.to_radians(0), std::runtime_error);
  }
  Calibration bad{1, 2048, 0, 1024, 3072, -1, 1, 0.3}; EXPECT_THROW(bad.validate(), std::runtime_error);
}
TEST(Hardware, ExplicitCalibrationAndUniqueIDs)
{
  auto h = info("/does/not/exist"); h.hardware_parameters["calibration_confirmed"] = "false";
  FeetechSystem first; EXPECT_EQ(first.on_init(h), CallbackReturn::ERROR);
  h = info("/does/not/exist"); h.hardware_parameters["motion_enabled"] = "false";
  FeetechSystem second; EXPECT_EQ(second.on_init(h), CallbackReturn::ERROR);
  h = info("/does/not/exist"); h.joints[1].parameters["servo_id"] = "1";
  FeetechSystem third; EXPECT_EQ(third.on_init(h), CallbackReturn::ERROR);
}
TEST(Hardware, StartupAtActualPositionSyncAndRelease)
{
  Bus bus; FeetechSystem hw;
  auto h = info(bus.path); h.hardware_parameters["torque_limit"] = "1000";
  ASSERT_EQ(hw.on_init(h), CallbackReturn::SUCCESS);
  auto states = hw.export_state_interfaces(); auto commands = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  for (int i = 1; i <= 6; ++i) EXPECT_EQ(bus.byte(i, 40), 0);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  for (int i = 1; i <= 6; ++i) EXPECT_EQ(bus.byte(i, 48) + 256 * bus.byte(i, 49), 1000);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(states[i].get_optional().value(), (i + 3) * 6.283185307179586 / 4096, 1e-9);
    EXPECT_TRUE(commands[i].set_value(states[i].get_optional().value() + 0.005));
  }
  ASSERT_EQ(hw.write(now, period), return_type::OK);
  ASSERT_EQ(hw.read(now, period), return_type::OK);
  EXPECT_GE(bus.syncs, 1); EXPECT_EQ(bus.eeprom_writes, 0);
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(states[i].get_optional().value(), commands[i].get_optional().value(), 0.0008);
  EXPECT_EQ(hw.on_deactivate(state), CallbackReturn::SUCCESS);
  for (int i = 1; i <= 6; ++i) EXPECT_EQ(bus.byte(i, 40), 0);
}
TEST(Hardware, PortOwnershipAndReadOnlyConfigureFailure)
{
  Bus bus; FeetechSystem first, second;
  ASSERT_EQ(first.on_init(info(bus.path)), CallbackReturn::SUCCESS);
  ASSERT_EQ(second.on_init(info(bus.path)), CallbackReturn::SUCCESS);
  ASSERT_EQ(first.on_configure(state), CallbackReturn::SUCCESS);
  EXPECT_EQ(second.on_configure(state), CallbackReturn::ERROR);
  EXPECT_EQ(bus.syncs, 0); EXPECT_EQ(bus.eeprom_writes, 0);
}
TEST(Hardware, FeedbackSweepUsesSixTransactionsAndHeldGoalsAreNotResent)
{
  Bus bus; FeetechSystem hw;
  ASSERT_EQ(hw.on_init(info(bus.path)), CallbackReturn::SUCCESS);
  auto commands = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  const int before = bus.read_requests;
  ASSERT_EQ(hw.read(now, period), return_type::OK);
  EXPECT_EQ(bus.read_requests-before, 6);
  ASSERT_EQ(hw.write(now, period), return_type::OK); EXPECT_EQ(bus.syncs, 0);
  ASSERT_TRUE(commands[0].set_value(commands[0].get_optional().value()+0.0002));
  ASSERT_EQ(hw.write(now, period), return_type::OK); EXPECT_EQ(bus.syncs, 0);
  ASSERT_TRUE(commands[0].set_value(commands[0].get_optional().value()+0.005));
  ASSERT_EQ(hw.write(now, period), return_type::OK);
  ASSERT_EQ(hw.read(now, period), return_type::OK); EXPECT_EQ(bus.syncs, 1);
  ASSERT_EQ(hw.write(now, period), return_type::OK); EXPECT_EQ(bus.syncs, 1);
}
TEST(Hardware, DroppedGroupedGoalIsRetriedUntilReadbackConfirmsIt)
{
  Bus bus; FeetechSystem hw;
  ASSERT_EQ(hw.on_init(info(bus.path)), CallbackReturn::SUCCESS);
  auto commands = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  ASSERT_TRUE(commands[0].set_value(commands[0].get_optional().value()+0.005));
  bus.drop_next_sync = true;
  ASSERT_EQ(hw.write(now, period), return_type::OK);
  ASSERT_EQ(hw.read(now, period), return_type::OK); EXPECT_EQ(bus.syncs, 1);
  ASSERT_EQ(hw.write(now, period), return_type::OK);
  ASSERT_EQ(hw.read(now, period), return_type::OK); EXPECT_EQ(bus.syncs, 2);
  ASSERT_EQ(hw.write(now, period), return_type::OK); EXPECT_EQ(bus.syncs, 2);
}
TEST(Hardware, IsolatedTemperatureSpikeNeedsTwoAgreeingCoolRechecks)
{
  Bus bus; FeetechSystem hw;
  ASSERT_EQ(hw.on_init(info(bus.path)), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  bus.temperature_spike_id = 6;
  ASSERT_EQ(hw.read(now, period), return_type::OK);
  for (int id = 1; id <= 6; ++id) EXPECT_EQ(bus.byte(id, 40), 1);
  bus.temperature_spike_id = 6; bus.direct_temperature = 40; // block is still 33
  EXPECT_EQ(hw.read(now, period), return_type::OK);
  for (int id = 1; id <= 6; ++id) EXPECT_EQ(bus.byte(id, 40), 1);
}
TEST(Hardware, FourHotCyclesLatchHoldWithoutTorqueReleaseOrAutomaticRecovery)
{
  Bus bus; FeetechSystem hw;
  ASSERT_EQ(hw.on_init(info(bus.path)), CallbackReturn::SUCCESS);
  auto commands = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  bus.set(6, 63, 70);
  for (int cycle = 0; cycle < 3; ++cycle) ASSERT_EQ(hw.read(now, period), return_type::OK);
  ASSERT_TRUE(commands[0].set_value(commands[0].get_optional().value()+0.005));
  ASSERT_EQ(hw.write(now, period), return_type::OK);
  ASSERT_EQ(hw.read(now, period), return_type::OK); // fourth: latch
  const int sent = bus.syncs;
  ASSERT_TRUE(commands[0].set_value(commands[0].get_optional().value()+0.005));
  ASSERT_EQ(hw.write(now, period), return_type::OK);
  EXPECT_EQ(bus.syncs, sent);
  EXPECT_EQ(bus.torque_off_writes, 0);
  for (int id = 1; id <= 6; ++id) EXPECT_EQ(bus.byte(id, 40), 1);
  bus.set(6, 63, 33);
  ASSERT_EQ(hw.read(now, period), return_type::OK);
  ASSERT_EQ(hw.write(now, period), return_type::OK);
  EXPECT_EQ(bus.syncs, sent); // cooling does not resume commands
  EXPECT_EQ(hw.on_deactivate(state), CallbackReturn::SUCCESS); // explicit shutdown still releases
  for (int id = 1; id <= 6; ++id) EXPECT_EQ(bus.byte(id, 40), 0);
}
TEST(Hardware, CoolSampleResetsConsecutiveHighCounter)
{
  Bus bus; FeetechSystem hw;
  ASSERT_EQ(hw.on_init(info(bus.path)), CallbackReturn::SUCCESS);
  auto commands = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  for (int repetition = 0; repetition < 2; ++repetition) {
    bus.set(6, 63, 70);
    for (int cycle = 0; cycle < 3; ++cycle) ASSERT_EQ(hw.read(now, period), return_type::OK);
    bus.set(6, 63, 36); ASSERT_EQ(hw.read(now, period), return_type::OK);
  }
  ASSERT_TRUE(commands[0].set_value(commands[0].get_optional().value()+0.005));
  ASSERT_EQ(hw.write(now, period), return_type::OK);
  ASSERT_EQ(hw.read(now, period), return_type::OK);
  EXPECT_EQ(bus.syncs, 1); EXPECT_EQ(bus.torque_off_writes, 0);
}
TEST(Hardware, FourConflicting706036ChecksLatchHoldOnFourthCycle)
{
  Bus bus; FeetechSystem hw;
  ASSERT_EQ(hw.on_init(info(bus.path)), CallbackReturn::SUCCESS);
  auto commands = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  bus.set(6, 63, 36); bus.direct_temperature = 70; bus.spike_temperature = 70;
  for (int cycle = 0; cycle < 4; ++cycle) {
    bus.temperature_spike_id = 6;
    ASSERT_EQ(hw.read(now, period), return_type::OK);
    ASSERT_TRUE(commands[0].set_value(commands[0].get_optional().value()+0.0016));
    ASSERT_EQ(hw.write(now, period), return_type::OK);
    const int expected = std::min(cycle+1, 3);
    // Grouped writes have no acknowledgement; wait for the emulator worker.
    for (int wait = 0; bus.syncs < expected && wait < 100; ++wait)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    EXPECT_EQ(bus.syncs, expected);
    EXPECT_EQ(bus.torque_off_writes, 0);
  }
}
class PacketFault : public testing::TestWithParam<int> {};
TEST_P(PacketFault, LatchMovementAndPreserveTorque)
{
  Bus bus; FeetechSystem hw;
  ASSERT_EQ(hw.on_init(info(bus.path)), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  bus.corrupt = GetParam();
  EXPECT_EQ(hw.read(now, period), return_type::OK);
  EXPECT_EQ(hw.write(now, period), return_type::OK);
  for (int i = 1; i <= 6; ++i) EXPECT_EQ(bus.byte(i, 40), 1);
  EXPECT_EQ(bus.torque_off_writes, 0); EXPECT_EQ(bus.syncs, 0);
  EXPECT_EQ(hw.on_activate(state), CallbackReturn::ERROR);
}
INSTANTIATE_TEST_SUITE_P(Checks, PacketFault, testing::Values(1, 2, 3, 4, 5));
class HealthFault : public testing::TestWithParam<std::pair<int, int>> {};
TEST_P(HealthFault, RejectUnsafeFeedback)
{
  Bus bus; FeetechSystem hw;
  ASSERT_EQ(hw.on_init(info(bus.path)), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  auto [address, value] = GetParam(); bus.set(3, address, value);
  EXPECT_EQ(hw.read(now, period), return_type::OK);
  EXPECT_EQ(hw.write(now, period), return_type::OK);
  for (int i = 1; i <= 6; ++i) EXPECT_EQ(bus.byte(i, 40), (address == 40 && i == 3) ? 0 : 1);
  EXPECT_EQ(bus.torque_off_writes, 0); EXPECT_EQ(bus.syncs, 0);
}
INSTANTIATE_TEST_SUITE_P(Checks, HealthFault, testing::Values(
  std::make_pair(62, 90), std::make_pair(62, 140),
  std::make_pair(65, 4), std::make_pair(40, 0), std::make_pair(57, 0)));
TEST(Hardware, RejectJumpAndNaNBeforeGroupWrite)
{
  for (double command : {0.5, std::nan(""), 2.0}) {
    Bus bus; FeetechSystem hw;
    ASSERT_EQ(hw.on_init(info(bus.path)), CallbackReturn::SUCCESS);
    auto commands = hw.export_command_interfaces();
    ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
    EXPECT_TRUE(commands[0].set_value(command));
    EXPECT_EQ(hw.write(now, period), return_type::OK); EXPECT_EQ(bus.syncs, 0);
    for (int i = 1; i <= 6; ++i) EXPECT_EQ(bus.byte(i, 40), 1);
  }
}
TEST(Hardware, StaleFeedbackStopsWithoutWrite)
{
  Bus bus; FeetechSystem hw; auto h = info(bus.path); h.hardware_parameters["max_feedback_age"] = "0.05";
  ASSERT_EQ(hw.on_init(h), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(std::chrono::milliseconds(65));
  EXPECT_EQ(hw.write(now, period), return_type::OK); EXPECT_EQ(bus.syncs, 0);
  EXPECT_EQ(bus.torque_off_writes, 0);
}
TEST(Hardware, SpeedGuardUsesActualWriteIntervalAfterSchedulingDelay)
{
  Bus bus; FeetechSystem hw;
  ASSERT_EQ(hw.on_init(info(bus.path)), CallbackReturn::SUCCESS);
  auto commands = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  auto nominal = rclcpp::Duration::from_seconds(0.02);
  ASSERT_EQ(hw.write(now, nominal), return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(45));
  ASSERT_EQ(hw.read(now, nominal), return_type::OK);
  ASSERT_TRUE(commands[0].set_value(commands[0].get_optional().value()+0.012));
  ASSERT_EQ(hw.write(now, nominal), return_type::OK);
  ASSERT_EQ(hw.read(now, nominal), return_type::OK);
  EXPECT_EQ(bus.syncs, 1); EXPECT_EQ(bus.torque_off_writes, 0);
}

TEST(Calibration, FeedbackAllowanceDoesNotExpandCommands)
{
  Calibration c{1, 2048, 1, 2048, 3072, 0, 1, 0.3}; c.validate();
  EXPECT_NEAR(c.to_radians(2045, 5), -3*6.283185307179586/4096, 1e-12);
  EXPECT_THROW(c.to_radians(2042, 5), std::runtime_error);
  EXPECT_THROW(c.to_ticks(-0.001), std::runtime_error);
}

TEST(Hardware, AdoptPoweredZeroHoldsWithoutTorqueGap)
{
  Bus bus; FeetechSystem hw; auto h = info(bus.path);
  h.hardware_parameters["adopt_zero_hold"] = "true";
  for (auto & joint : h.joints) {
    joint.parameters["min_tick"] = "2048"; joint.parameters["min_radians"] = "0";
  }
  for (int id = 1; id <= 6; ++id) {
    bus.set(id, 40, 1); bus.set(id, 42, 0); bus.set(id, 43, 8);
    bus.set(id, 48, 44); bus.set(id, 49, 1); // RAM torque limit 300
    bus.set(id, 56, 247); bus.set(id, 57, 7); // measured 2039: 9 ticks of settling error
  }
  ASSERT_EQ(hw.on_init(h), CallbackReturn::SUCCESS);
  auto commands = hw.export_command_interfaces();
  ASSERT_EQ(hw.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(state), CallbackReturn::SUCCESS);
  for (const auto & command : commands) EXPECT_DOUBLE_EQ(command.get_optional().value(), 0);
  EXPECT_EQ(bus.torque_off_writes, 0);
  EXPECT_EQ(hw.write(now, period), return_type::OK);
  EXPECT_EQ(hw.read(now, period), return_type::OK);
  EXPECT_EQ(hw.on_deactivate(state), CallbackReturn::SUCCESS);
  EXPECT_EQ(bus.torque_off_writes, 6);
}

TEST(Hardware, AdoptRejectsUnknownGoalsWithoutDisablingExternalHolds)
{
  Bus bus; FeetechSystem hw; auto h = info(bus.path);
  h.hardware_parameters["adopt_zero_hold"] = "true";
  for (int id = 1; id <= 6; ++id) {
    bus.set(id, 40, 1); bus.set(id, 56, 0); bus.set(id, 57, 8);
    // The existing stale goal of 3000 is deliberately not a verified zero hold.
  }
  ASSERT_EQ(hw.on_init(h), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw.on_configure(state), CallbackReturn::ERROR);
  EXPECT_EQ(bus.torque_off_writes, 0);
  EXPECT_EQ(bus.syncs, 0);
}

TEST(Hardware, AdoptRejectsTorqueOffBeforeAnyGoalWrite)
{
  Bus bus; FeetechSystem hw; auto h = info(bus.path);
  h.hardware_parameters["adopt_zero_hold"] = "true";
  ASSERT_EQ(hw.on_init(h), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw.on_configure(state), CallbackReturn::ERROR);
  EXPECT_EQ(bus.torque_off_writes, 0); EXPECT_EQ(bus.syncs, 0);
}
