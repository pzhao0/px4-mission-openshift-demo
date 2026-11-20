#include "ReturnToStart.hpp"

#include <algorithm>

ReturnToStart::ReturnToStart(rclcpp::Node& node, DemoSession& session)
    : ModeBase(node, Settings{"ReturnToStart"}), _session(session)
{
  _setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
  modeRequirements().local_position = true;
  modeRequirements().local_alt = true;
}

void ReturnToStart::onActivate()
{
  _settled = _elapsed = 0;
  _finished = false;
  if (_session.startValid() && _session.telemetryFresh()) {
    _target = _session.start();
    // Stay at the completed takeoff height while returning horizontally.
    _target.z() = std::min(_session.position().z(), _session.start().z() - 2.f);
  }
}

void ReturnToStart::checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter& reporter)
{
  if (!_session.startValid() || !_session.telemetryFresh()) {
    reporter.armingCheckFailureExt(px4_ros2::events::ID("mission_demo_return_start_unavailable"),
        px4_ros2::events::Log::Error, "Return requires a saved start and fresh vehicle telemetry");
  }
}

void ReturnToStart::updateSetpoint(float dt_s)
{
  if (_finished) return;
  if (!_session.startValid() || !_session.telemetryFresh()) {
    _finished = true;
    completed(px4_ros2::Result::ModeFailureOther);
    return;
  }
  _setpoint->update(px4_ros2::TrajectorySetpoint{}.withPosition(_target).withYaw(_session.startHeading()));
  _elapsed += std::clamp(dt_s, 0.f, .5f);
  if ((_session.position() - _target).norm() < .35f && _session.velocity().norm() < .3f)
    _settled += std::clamp(dt_s, 0.f, .5f);
  else _settled = 0;
  if (_settled >= 1.f || _elapsed > 60.f) {
    _finished = true;
    completed(_settled >= 1.f ? px4_ros2::Result::Success : px4_ros2::Result::Timeout);
  }
}
