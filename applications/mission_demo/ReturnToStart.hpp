#pragma once

#include "DemoSession.hpp"
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

class ReturnToStart : public px4_ros2::ModeBase {
 public:
  ReturnToStart(rclcpp::Node& node, DemoSession& session);
  void onActivate() override;
  void updateSetpoint(float dt_s) override;
  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter& reporter) override;

 private:
  DemoSession& _session;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _setpoint;
  Eigen::Vector3f _target{};
  float _settled{}, _elapsed{};
  bool _finished{};
};
