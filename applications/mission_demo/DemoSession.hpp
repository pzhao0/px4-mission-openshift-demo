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
  bool beginDemo(std::string& reason);
  bool beginReturn(std::string& reason);
  bool canPrepareReturn(std::string& reason) const;
  void returnPrepared();
  void demoComplete();
  void returnComplete();
  void stop(const std::string& reason);
  void phase(const std::string& phase, const std::string& message);
  bool telemetryFresh() const;
  bool startValid() const;
  bool atStart() const;
  Eigen::Vector3f start() const { return *_start; }
  Eigen::Vector3f position() const { return {_position.x, _position.y, _position.z}; }
  Eigen::Vector3f velocity() const { return {_position.vx, _position.vy, _position.vz}; }
  float startHeading() const { return _start_heading; }
  const std::string& currentPhase() const { return _phase; }
  void setModeIds(uint8_t demo, uint8_t returning) { _demo_mode = demo; _return_mode = returning; }
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
  std::optional<Eigen::Vector3f> _start;
  float _start_heading{};
  uint8_t _xy_resets{}, _z_resets{};
  bool _origin_changed{false};
  std::string _phase{"ready"};
  std::string _message{"Select DemoWaypoints and arm in QGroundControl to begin."};
  unsigned _completed_runs{}, _completed_returns{};
  uint8_t _demo_mode{255}, _return_mode{255};
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _position_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_sub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _timer;
};
