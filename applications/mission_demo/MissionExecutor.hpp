#pragma once

#include "DemoSession.hpp"
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/utils/vehicle_command_sender.hpp>
#include <std_srvs/srv/trigger.hpp>

// Demo and recovery share this sequence, each with a distinct selectable mode.
class MissionExecutor : public px4_ros2::ModeExecutorBase {
 public:
  enum class Operation { Demo, Return };
  MissionExecutor(px4_ros2::ModeBase& owned_mode, px4_ros2::ModeBase& landing_mode,
                  DemoSession& session, Operation operation = Operation::Demo);
  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;
  std::pair<bool, std::string> prepareReturn();

 protected:
  virtual px4_ros2::Result selectReturnMode();

 private:
  enum class State { Takeoff, Waypoints, PrecisionLand, Land, WaitUntilDisarmed };
  void switchToState(State state, px4_ros2::Result previous_result);
  void stopMission(px4_ros2::Result result);
  px4_ros2::ModeBase& _landing_mode;
  DemoSession& _session;
  Operation _operation;
  State _state{State::Takeoff};
  bool _mission_active{false};
  px4_ros2::VehicleCommandSender _mode_selector;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _return_service;
};
