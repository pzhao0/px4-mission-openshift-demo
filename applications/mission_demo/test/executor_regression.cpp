// Exercises the production executor and real interface library without a live PX4.
#include "MissionExecutor.hpp"
#include "MissionModes.hpp"
#include <src/components/registration.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
using Status = px4_msgs::msg::VehicleStatus;
using Command = px4_msgs::msg::VehicleCommand;
using Completed = px4_msgs::msg::ModeCompleted;
using Result = px4_ros2::Result;

void require(bool condition, const std::string& message)
{
  if (!condition) throw std::runtime_error(message);
}

class RegistrationStub : public Registration {
 public:
  RegistrationStub(rclcpp::Node& node, uint8_t mode, uint8_t executor = 1)
      : Registration(node), mode_(mode), executor_(executor) {}
  bool doRegister(const RegistrationSettings& settings) override
  {
    setRegistrationDetails(1, mode_, executor_, settings.name);
    return true;
  }
  void doUnregister() override {}
 private:
  uint8_t mode_;
  uint8_t executor_;
};

class ModeStub : public px4_ros2::ModeBase {
 public:
  ModeStub(rclcpp::Node& node, const std::string& name, uint8_t id) : ModeBase(node, name)
  {
    setpoint_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    setSkipMessageCompatibilityCheck();
    setSkipSetpointCheck();
    overrideRegistration(std::make_shared<RegistrationStub>(node, id));
  }
 private:
  std::shared_ptr<px4_ros2::TrajectorySetpointType> setpoint_;
};

class ExecutorProbe : public MissionExecutor {
 public:
  ExecutorProbe(ModeStub& owned, ModeStub& landing, DemoSession& session,
                Operation operation = Operation::Demo, uint8_t mode_id = 100, uint8_t executor_id = 1)
      : MissionExecutor(owned, landing, session, operation)
  {
    setSkipMessageCompatibilityCheck();
    overrideRegistration(std::make_shared<RegistrationStub>(node(), operation == Operation::Demo ? mode_id : 102,
                                                          operation == Operation::Demo ? executor_id : 2));
  }
  Result sendCommandSync(uint32_t command, float p1, float, float, float, float, float,
                         float p7) override
  {
    commands.push_back({command, p1, p7});
    return command == Command::VEHICLE_CMD_NAV_TAKEOFF ? takeoff_result : Result::Success;
  }
  struct SentCommand { uint32_t command; float param1; float altitude; };
  std::vector<SentCommand> commands;
  Result takeoff_result{Result::Success};
  Result selectReturnMode() override { ++selections; return selection_result; }
  unsigned selections{};
  Result selection_result{Result::Success};
};

class Fixture {
 public:
  Fixture() : node(std::make_shared<rclcpp::Node>("executor_regression")),
      session(*node), owned(*node, "waypoints", 100), landing(*node, "landing", 101),
      return_mode(*node, "return", 102), executor(owned, landing, session),
      returning(return_mode, landing, session, MissionExecutor::Operation::Return)
  {
    status_pub = node->create_publisher<Status>(
        "/fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<Status>(), 1);
    completed_pub = node->create_publisher<Completed>(
        "/fmu/out/mode_completed" + px4_ros2::getMessageNameVersion<Completed>(), 1);
    logs_sub = node->create_subscription<rcl_interfaces::msg::Log>("/rosout", 100,
        [this](const rcl_interfaces::msg::Log& log) { logs.push_back(log.msg); });
    position_pub = node->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>(), 1);
    land_pub = node->create_publisher<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLandDetected>(), 1);
    require(landing.doRegister(), "dummy landing mode registration failed");
    require(executor.doRegister(), "executor registration failed");
    require(returning.doRegister(), "return executor registration failed");
    session.setModeIds(100, 102);
    pump(150ms);
  }
  void pump(std::chrono::milliseconds duration = 100ms)
  {
    auto until = std::chrono::steady_clock::now() + duration;
    do { rclcpp::spin_some(node); std::this_thread::sleep_for(1ms); }
    while (std::chrono::steady_clock::now() < until);
  }
  void status(uint8_t mode, bool armed = true, uint8_t in_charge = 1)
  {
    Status msg{};
    msg.nav_state = mode;
    msg.arming_state = armed ? Status::ARMING_STATE_ARMED : Status::ARMING_STATE_DISARMED;
    msg.executor_in_charge = in_charge;
    for (int i = 0; i < 3; ++i) { status_pub->publish(msg); pump(20ms); }
  }
  void start()
  {
    telemetry();
    status(100, false, 0);
    status(100);
    require(!executor.commands.empty(), "fake armed status did not activate executor");
  }
  void telemetry(float x = 0, float y = 0, float z = 0, uint8_t resets = 0)
  {
    px4_msgs::msg::VehicleLocalPosition position{};
    position.x = x; position.y = y; position.z = z;
    position.xy_valid = position.z_valid = position.v_xy_valid = position.v_z_valid = true;
    position.xy_reset_counter = resets;
    px4_msgs::msg::VehicleLandDetected land{};
    land.landed = true;
    position_pub->publish(position); land_pub->publish(land); pump(30ms);
  }
  void complete(uint8_t mode, Result result = Result::Success, uint8_t in_charge = 1)
  {
    status(mode, true, in_charge);
    Completed msg{};
    msg.nav_state = mode;
    msg.result = static_cast<uint8_t>(result);
    completed_pub->publish(msg);
    pump();
  }
  bool commanded(uint8_t mode) const
  {
    return std::any_of(executor.commands.begin(), executor.commands.end(), [mode](const auto& c) {
      return c.command == Command::VEHICLE_CMD_SET_NAV_STATE && c.param1 == mode;
    });
  }
  bool reportedSuccess()
  {
    pump();
    return std::any_of(logs.begin(), logs.end(), [](const auto& line) {
      return line.find("All states complete") != std::string::npos;
    });
  }
  void reachDisarmWait()
  {
    start();
    complete(Status::NAVIGATION_STATE_AUTO_TAKEOFF);
    require(commanded(100), "successful takeoff did not start waypoints");
    complete(100);
    require(commanded(101), "successful waypoints did not start landing");
    complete(101);
  }
  std::shared_ptr<rclcpp::Node> node;
  DemoSession session;
  ModeStub owned, landing, return_mode;
  ExecutorProbe executor, returning;
  rclcpp::Publisher<Status>::SharedPtr status_pub;
  rclcpp::Publisher<Completed>::SharedPtr completed_pub;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr logs_sub;
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_pub;
  std::vector<std::string> logs;
};

int main(int argc, char** argv)
{
  // Never allow this fake autopilot to share the live application's domain.
  require(std::string(std::getenv("ROS_DOMAIN_ID") ? std::getenv("ROS_DOMAIN_ID") : "") == "101",
          "run with ROS_DOMAIN_ID=101 (isolated regression domain)");
  rclcpp::init(argc, argv);
  int failures = 0;
  auto test = [&](const char* name, auto body) {
    try { body(); std::cout << "PASS " << name << '\n'; }
    catch (const std::exception& e) { ++failures; std::cerr << "FAIL " << name << ": " << e.what() << '\n'; }
  };
  test("takeoff uses PX4 relative-height default", [] {
    Fixture f; f.start();
    require(std::isnan(f.executor.commands.front().altitude),
            "takeoff supplied a finite AMSL altitude instead of using MIS_TAKEOFF_ALT");
  });
  test("rejected takeoff cannot start waypoints", [] {
    Fixture f; f.executor.takeoff_result = Result::Rejected; f.start();
    require(!f.commanded(100) && !f.commanded(101), "rejected takeoff advanced the mission");
    require(f.commanded(Status::NAVIGATION_STATE_AUTO_LOITER), "failure did not request Hold");
  });
  test("failed waypoints cannot start precision landing", [] {
    Fixture f; f.start(); f.complete(Status::NAVIGATION_STATE_AUTO_TAKEOFF);
    f.complete(100, Result::ModeFailureOther);
    require(!f.commanded(101), "waypoint failure advanced to precision landing");
    require(f.commanded(Status::NAVIGATION_STATE_AUTO_LOITER), "failure did not request Hold");
    require(!f.reportedSuccess(), "failure reported successful completion");
  });
  test("operator cancellation sends no replacement commands", [] {
    Fixture f; f.start(); auto count = f.executor.commands.size();
    f.status(Status::NAVIGATION_STATE_AUTO_LOITER, true, 0);
    require(f.executor.commands.size() == count, "cancelled takeoff scheduled more commands");
    require(!f.reportedSuccess(), "cancellation reported successful completion");
  });
  test("successful sequence finishes only after disarm", [] {
    Fixture f; f.reachDisarmWait();
    require(!f.reportedSuccess(), "mission reported success before disarm");
    f.status(101, false, 0);
    require(f.reportedSuccess(), "successful landing and disarm did not report completion");
  });
  test("cancellation during disarm wait is not success", [] {
    Fixture f; f.reachDisarmWait(); f.status(Status::NAVIGATION_STATE_AUTO_LOITER, true, 0);
    require(!f.reportedSuccess(), "cancelled disarm wait reported completion");
  });
  test("return request cannot select a mode before a demo or while armed", [] {
    Fixture f;
    require(!f.returning.prepareReturn().first, "return accepted without saved start");
    f.reachDisarmWait(); f.telemetry(-4, 8);
    require(!f.returning.prepareReturn().first, "return accepted while armed");
    require(f.returning.selections == 0, "rejected return sent a mode-selection command");
  });
  test("two complete demo and return cycles preserve the original start", [] {
    Fixture f;
    for (int cycle = 0; cycle < 2; ++cycle) {
      f.reachDisarmWait(); f.telemetry(-4, 8); f.status(101, false, 0);
      require(f.session.currentPhase() == "demo_complete", "demo did not finish after disarm");
      require(f.returning.prepareReturn().first, "valid return preparation rejected");
      require(f.returning.commands.empty() || cycle > 0, "return preparation issued flight commands before arming");
      f.status(102, false, 0);
      f.status(102, true, 2);
      require(!f.returning.commands.empty(), "arming return mode did not request takeoff");
      require(std::isnan(f.returning.commands.back().altitude), "return takeoff used finite AMSL altitude");
      f.complete(Status::NAVIGATION_STATE_AUTO_TAKEOFF, Result::Success, 2);
      require(f.session.currentPhase() == "return_transit", "return takeoff did not start return transit");
      f.telemetry(0, 0, -2);
      f.complete(102, Result::Success, 2);
      require(std::any_of(f.returning.commands.begin(), f.returning.commands.end(), [](const auto& command) {
        return command.command == Command::VEHICLE_CMD_SET_NAV_STATE && command.param1 == Status::NAVIGATION_STATE_AUTO_LAND;
      }), "return did not request landing at the start");
      f.complete(Status::NAVIGATION_STATE_AUTO_LAND, Result::Success, 2);
      f.telemetry(); f.status(Status::NAVIGATION_STATE_AUTO_LAND, false, 0);
      require(f.session.currentPhase() == "ready", "return did not finish ready for another demo");
      require(f.session.start().norm() < .01f, "return overwrote the original start");
    }
    require(f.session.statusJson().find("\"completed_runs\":2") != std::string::npos, "did not record two demo flights");
    require(f.session.statusJson().find("\"completed_returns\":2") != std::string::npos, "did not record two return flights");
  });
  test("both registered demos select their own route and share landing and return", [] {
    Fixture f;
    ModeStub survey(*f.node, "SignSurvey", 103);
    ExecutorProbe survey_exec(survey, f.landing, f.session, MissionExecutor::Operation::Demo, 103, 3);
    require(survey_exec.doRegister(), "survey registration failed");
    f.session.setModeIds(100, 102, 103);
    require(f.session.statusJson().find("\"waypoint_mode_id\":100,\"survey_mode_id\":103") != std::string::npos,
            "missing named demo IDs");
    f.telemetry();
    f.status(103, false, 0);
    f.status(103, true, 3);
    require(!survey_exec.commands.empty() && f.executor.commands.empty(), "wrong executor took off");
    f.complete(Status::NAVIGATION_STATE_AUTO_TAKEOFF, Result::Success, 3);
    require(f.session.surveying() && f.session.currentPhase() == "search_searching", "wrong survey route");
    f.complete(103, Result::Success, 3);
    require(f.session.currentPhase() == "demo_landing", "survey did not share marker landing");
    f.complete(101, Result::Success, 3);
    f.telemetry(-4, 8);
    f.status(101, false, 0);
    require(f.returning.prepareReturn().first, "return after survey rejected");
    f.status(102, false, 0);
    f.status(102, true, 2);
    f.complete(Status::NAVIGATION_STATE_AUTO_TAKEOFF, Result::Success, 2);
    f.complete(102, Result::Success, 2);
    f.complete(Status::NAVIGATION_STATE_AUTO_LAND, Result::Success, 2);
    f.telemetry();
    f.status(Status::NAVIGATION_STATE_AUTO_LAND, false, 0);
    require(f.session.currentPhase() == "ready", "shared return failed");
    f.reachDisarmWait();
    require(!f.session.surveying() && f.session.statusJson().find("\"survey\":") == std::string::npos,
            "previous survey leaked into waypoint flight");
  });
  test("cancelled return preparation is rejected when position origin changes", [] {
    Fixture f; f.reachDisarmWait(); f.telemetry(-4, 8); f.status(101, false, 0);
    f.telemetry(-4, 8, 0, 1);
    require(!f.returning.prepareReturn().first, "return accepted after local coordinate reset");
    require(f.returning.selections == 0, "invalid-origin return selected a mode");
  });
  test("stale telemetry cannot prepare a return", [] {
    Fixture f; f.reachDisarmWait(); f.telemetry(-4, 8); f.status(101, false, 0);
    f.pump(3100ms);
    require(!f.returning.prepareReturn().first, "return accepted with stale telemetry");
  });
  rclcpp::shutdown();
  return failures ? 1 : 0;
}
