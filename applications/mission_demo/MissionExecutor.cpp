#include "MissionExecutor.hpp"

using Result = px4_ros2::Result;

MissionExecutor::MissionExecutor(px4_ros2::ModeBase& owned_mode, px4_ros2::ModeBase& landing_mode,
                                 DemoSession& session, Operation operation)
    : ModeExecutorBase(Settings{}, owned_mode), _landing_mode(landing_mode),
      _session(session), _operation(operation), _mode_selector(node(), "")
{
  if (_operation == Operation::Return) {
    _return_service = node().create_service<std_srvs::srv::Trigger>("/mission/prepare_return",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          const auto [success, message] = prepareReturn();
          response->success = success;
          response->message = message;
        });
  }
}

std::pair<bool, std::string> MissionExecutor::prepareReturn()
{
  std::string reason;
  if (_operation != Operation::Return || !_session.canPrepareReturn(reason)) return {false, reason};
  const auto result = selectReturnMode();
  if (result != Result::Success) return {false, std::string("Mode selection failed: ") + resultToString(result)};
  _session.returnPrepared();
  return {true, "ReturnToStart selected. Arm in QGroundControl to fly back."};
}

Result MissionExecutor::selectReturnMode()
{
  // Selecting a mode while disarmed is a user action. Executor commands are
  // accepted only from the executor currently in charge of an armed flight.
  px4_msgs::msg::VehicleCommand command{};
  command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
  command.param1 = ownedMode().id();
  command.source_system = 255;
  command.source_component = 191;
  command.target_system = 1;
  command.target_component = 1;
  command.from_external = true;
  return _mode_selector.sendCommandSync(command);
}

void MissionExecutor::onActivate()
{
  std::string reason;
  const bool allowed = _operation == Operation::Demo ? _session.beginDemo(ownedMode().id(), reason) : _session.beginReturn(reason);
  _mission_active = true;
  if (!allowed) {
    RCLCPP_WARN(node().get_logger(), "%s", reason.c_str());
    stopMission(Result::Rejected);
    return;
  }
  switchToState(State::Takeoff, Result::Success);
}

void MissionExecutor::onDeactivate(DeactivateReason reason)
{
  if (_mission_active) {
    _session.stop(reason == DeactivateReason::FailsafeActivated ? "Flight stopped by failsafe." : "Flight cancelled by operator.");
  }
  _mission_active = false;
}

void MissionExecutor::switchToState(State state, Result previous_result)
{
  if (!_mission_active) return;
  if (previous_result != Result::Success) { stopMission(previous_result); return; }
  _state = state;
  switch (state) {
    case State::Takeoff:
      // Unset AMSL altitude uses current altitude + MIS_TAKEOFF_ALT.
      takeoff([this](Result result) { switchToState(State::Waypoints, result); });
      break;
    case State::Waypoints:
      if (_operation == Operation::Return) {
        _session.phase("return_transit", "Flying back to the saved original start.");
      } else if (_session.surveying()) {
        _session.phase("search_searching", "Starting the stop-sign survey.");
      } else {
        _session.phase("demo_waypoints", "Flying the demo waypoint route.");
      }
      scheduleMode(ownedMode().id(), [this](Result result) {
        switchToState(_operation == Operation::Demo ? State::PrecisionLand : State::Land, result);
      });
      break;
    case State::PrecisionLand:
      _session.phase("demo_landing", "Searching for the marker, aligning and landing.");
      scheduleMode(_landing_mode.id(), [this](Result result) { switchToState(State::WaitUntilDisarmed, result); });
      break;
    case State::Land:
      _session.phase("return_landing", "Landing at the original start.");
      land([this](Result result) { switchToState(State::WaitUntilDisarmed, result); });
      break;
    case State::WaitUntilDisarmed:
      waitUntilDisarmed([this](Result result) {
        if (!_mission_active) return;
        // Disarm cancels library waits before onDeactivate; an armed takeover
        // must never be counted as completion.
        if (!isArmed() && (result == Result::Success || result == Result::Deactivated)) {
          _mission_active = false;
          if (_operation == Operation::Demo) _session.demoComplete();
          else _session.returnComplete();
          RCLCPP_INFO(node().get_logger(), "All states complete (Success)");
        } else stopMission(result);
      });
      break;
  }
}

void MissionExecutor::stopMission(Result result)
{
  _mission_active = false;
  _session.stop(std::string("Flight stopped: ") + resultToString(result) + ". Use QGroundControl to recover.");
  // Preserve the operator/failsafe mode during cancellation.
  if (result == Result::Deactivated || !isInCharge() || !isArmed()) return;
  scheduleMode(px4_ros2::ModeBase::kModeIDLoiter, [this](Result hold_result) {
    if (hold_result != Result::Success && hold_result != Result::Deactivated)
      RCLCPP_ERROR(node().get_logger(), "Hold failed: %s", resultToString(hold_result));
  });
}
