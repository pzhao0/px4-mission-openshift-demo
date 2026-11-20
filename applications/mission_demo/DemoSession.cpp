#include "DemoSession.hpp"

#include <cmath>
#include <iomanip>
#include <px4_ros2/utils/message_version.hpp>
#include <sstream>

using namespace std::chrono_literals;

DemoSession::DemoSession(rclcpp::Node& node) : _node(node)
{
  using Position = px4_msgs::msg::VehicleLocalPosition;
  using Status = px4_msgs::msg::VehicleStatus;
  using Land = px4_msgs::msg::VehicleLandDetected;
  _position_sub = node.create_subscription<Position>(
      "/fmu/out/vehicle_local_position" + px4_ros2::getMessageNameVersion<Position>(),
      rclcpp::SensorDataQoS(), [this](const Position& msg) {
        if (_start && (msg.xy_reset_counter != _xy_resets || msg.z_reset_counter != _z_resets)) {
          _origin_changed = true;
        }
        _position = msg;
        _position_time = Clock::now();
      });
  _status_sub = node.create_subscription<Status>(
      "/fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<Status>(),
      rclcpp::SensorDataQoS(), [this](const Status& msg) {
        _status = msg;
        _status_time = Clock::now();
        if (_phase == "return_awaiting_arm" && msg.nav_state != _return_mode && Clock::now() - _prepared_time > 1s) {
          phase("demo_complete", "Return selection cancelled. Select Return to start to prepare it again.");
        }
      });
  _land_sub = node.create_subscription<Land>(
      "/fmu/out/vehicle_land_detected" + px4_ros2::getMessageNameVersion<Land>(),
      rclcpp::SensorDataQoS(), [this](const Land& msg) {
        _land = msg;
        _land_time = Clock::now();
      });
  _publisher = node.create_publisher<std_msgs::msg::String>("/mission/demo_status", 1);
  _timer = node.create_wall_timer(500ms, [this] { publish(); });
}

bool DemoSession::telemetryFresh() const
{
  const auto now = Clock::now();
  return now - _position_time < 3s && now - _status_time < 3s && now - _land_time < 3s &&
         _position.xy_valid && _position.z_valid && _position.v_xy_valid && _position.v_z_valid &&
         position().allFinite() && velocity().allFinite() && std::isfinite(_position.heading);
}

bool DemoSession::startValid() const { return _start.has_value() && !_origin_changed; }

bool DemoSession::atStart() const
{
  return startValid() && telemetryFresh() && (position().head<2>() - start().head<2>()).norm() < .75f &&
         std::abs(_position.z - start().z()) < .5f;
}

bool DemoSession::beginDemo(std::string& reason)
{
  if (!telemetryFresh() || !_land.landed) {
    reason = "Demo start requires fresh valid position and a landed vehicle.";
    return false;
  }
  if (!_start) {
    _start = position();
    _start_heading = _position.heading;
    _xy_resets = _position.xy_reset_counter;
    _z_resets = _position.z_reset_counter;
  }
  if (!atStart()) {
    reason = "Return to the saved start before starting another demo.";
    return false;
  }
  phase("demo_takeoff", "Taking off for the waypoint and marker-landing demo.");
  return true;
}

bool DemoSession::canPrepareReturn(std::string& reason) const
{
  if (!startValid()) reason = "No valid saved start. Complete a demo from the starting position first.";
  else if (!telemetryFresh()) reason = "Waiting for fresh valid vehicle telemetry.";
  else if (_status.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED || !_land.landed)
    reason = "Wait until the vehicle has landed and disarmed.";
  else if (_phase != "demo_complete" && _phase != "stopped") reason = "Return is available after a demo finishes or stops.";
  else if (atStart()) reason = "The drone is already at the saved start.";
  else { reason = "Ready to select ReturnToStart. Arm separately in QGroundControl."; return true; }
  return false;
}

bool DemoSession::beginReturn(std::string& reason)
{
  if (!startValid() || !telemetryFresh() || !_land.landed ||
      (_phase != "demo_complete" && _phase != "stopped" && _phase != "return_awaiting_arm")) {
    reason = "Return requires a saved start, fresh telemetry and a completed or stopped demo on the ground.";
    return false;
  }
  phase("return_takeoff", "Taking off to return to the original start.");
  return true;
}

void DemoSession::returnPrepared()
{
  _prepared_time = Clock::now();
  phase("return_awaiting_arm", "ReturnToStart selected. Arm in QGroundControl to fly back; this page does not arm the drone.");
}

void DemoSession::demoComplete()
{
  ++_completed_runs;
  phase("demo_complete", "Demo complete. Select Return to start, then arm in QGroundControl.");
}

void DemoSession::returnComplete()
{
  if (!atStart()) {
    stop("Return landed outside the start tolerance. Check position before proceeding.");
    return;
  }
  ++_completed_returns;
  phase("ready", "Ready for the next run. Select DemoWaypoints and arm in QGroundControl.");
}

void DemoSession::stop(const std::string& reason) { phase("stopped", reason); }

void DemoSession::phase(const std::string& phase_name, const std::string& message)
{
  _phase = phase_name;
  _message = message;
  RCLCPP_INFO(_node.get_logger(), "Demo phase: %s — %s", _phase.c_str(), _message.c_str());
  publish();
}

std::string DemoSession::statusJson() const
{
  std::string reason;
  const bool can_return = canPrepareReturn(reason);
  std::ostringstream out;
  out << std::boolalpha << std::setprecision(6)
      << "{\"phase\":" << std::quoted(_phase) << ",\"message\":" << std::quoted(_message)
      << ",\"can_return\":" << can_return << ",\"return_reason\":" << std::quoted(reason)
      << ",\"telemetry_fresh\":" << telemetryFresh() << ",\"at_start\":" << atStart()
      << ",\"armed\":" << (_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
      << ",\"completed_runs\":" << _completed_runs << ",\"completed_returns\":" << _completed_returns
      << ",\"demo_mode_id\":" << unsigned(_demo_mode) << ",\"return_mode_id\":" << unsigned(_return_mode)
      << ",\"start_ned\":";
  if (startValid()) out << '[' << start().x() << ',' << start().y() << ',' << start().z() << ']';
  else out << "null";
  out << ",\"distance_to_start_metres\":";
  if (startValid() && telemetryFresh()) out << (position().head<2>() - start().head<2>()).norm();
  else out << "null";
  return out.str() + '}';
}

void DemoSession::publish()
{
  std_msgs::msg::String msg;
  msg.data = statusJson();
  _publisher->publish(msg);
}
