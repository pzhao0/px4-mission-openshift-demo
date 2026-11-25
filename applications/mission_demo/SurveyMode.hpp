#pragma once

#include "DemoSession.hpp"
#include "SurveyLanes.hpp"
#include <px4_ros2/odometry/attitude.hpp>
#include <mission_demo/msg/survey_observation.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

class SurveyMode : public px4_ros2::ModeBase {
 public:
  SurveyMode(rclcpp::Node& node, DemoSession& session);
  void onActivate() override;
  void updateSetpoint(float dt_s) override;

 private:
  using Observation = mission_demo::msg::SurveyObservation;
  enum class State { LaneEntry, Pass, Align, Recovery };

  void observe(const Observation& observation);
  void checkCoverage(int64_t now);
  void logPass(const char* outcome, int64_t now) const;
  void returnToPad(int64_t now, bool spotted);
  void updateAlignment(const Observation& observation, int64_t stamp);

  rclcpp::Node& _node;
  DemoSession& _session;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _setpoint;
  rclcpp::Subscription<Observation>::SharedPtr _observations;
  State _state{State::LaneEntry};
  int _pass{};
  std::shared_ptr<px4_ros2::OdometryAttitude> _attitude;
  Eigen::Vector3f _sign_hold{};
  int64_t _align_start{}, _last_face{}, _retry_after{};
  std::optional<int64_t> _hold_start;

  int64_t _pass_start{}, _last_frame{}, _last_fresh{}, _max_gap{};
  unsigned _fresh_frames{};
  bool _gap_active{}, _coverage_incomplete{};
};
