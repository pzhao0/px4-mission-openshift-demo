#pragma once

#include <Eigen/Eigen>
#include <chrono>
#include <optional>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Shared flight-session state. The original start is captured once, before the
// first demo takeoff, and remains fixed across all subsequent arming cycles.
class DemoSession {
 public:
  explicit DemoSession(rclcpp::Node& node);
  bool beginDemo(uint8_t mode, std::string& reason);
  bool beginReturn(std::string& reason);
  bool canPrepareReturn(std::string& reason) const;
  void returnPrepared();
  void demoComplete();
  void returnComplete();
  void stop(const std::string& reason);
  void phase(const std::string& phase, const std::string& message);
  bool telemetryFresh() const;
  bool surveyPositionFresh(int64_t now) const;
  int64_t positionSimTime() const { return _position_sim_time; }
  void setSurveyPassStart(int64_t stamp) { _survey_pass_start = stamp; }
  void setHoldFrame(int64_t stamp) { _hold_frame = stamp; }
  void setSightingFrame(int64_t stamp) { _sighting_frame = stamp; }
  void setDemoSummary(const std::string& summary) { _demo_summary = summary; }
  bool surveying() const { return _survey_run != 0; }
  bool startValid() const;
  bool atStart() const;
  Eigen::Vector3f start() const { return *_start; }
  Eigen::Vector3f position() const { return {_position.x, _position.y, _position.z}; }
  Eigen::Vector3f velocity() const { return {_position.vx, _position.vy, _position.vz}; }
  float startHeading() const { return _start_heading; }
  const std::string& currentPhase() const { return _phase; }
  void setModeIds(uint8_t waypoints, uint8_t returning, std::optional<uint8_t> survey = std::nullopt);
  std::string statusJson() const;

 private:
  using Clock = std::chrono::steady_clock;
  void publish();
  rclcpp::Node& _node;
  px4_msgs::msg::VehicleLocalPosition _position{};
  px4_msgs::msg::VehicleStatus _status{};
  px4_msgs::msg::VehicleLandDetected _land{};
  Clock::time_point _position_time{}, _status_time{}, _land_time{};
  Clock::time_point _prepared_time{};
  int64_t _position_sim_time{}, _survey_run{}, _survey_pass_start{}, _sighting_frame{}, _hold_frame{};
  std::string demoChoices() const;
  std::string _demo_summary;
  std::optional<Eigen::Vector3f> _start;
  float _start_heading{};
  uint8_t _xy_resets{}, _z_resets{};
  bool _origin_changed{false};
  std::string _phase{"ready"};
  std::string _message{"Select DemoWaypoints and arm in QGroundControl to begin."};
  unsigned _completed_runs{}, _completed_returns{};
  uint8_t _waypoint_mode{255}, _return_mode{255};
  std::optional<uint8_t> _survey_mode;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _position_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _timer;
};
