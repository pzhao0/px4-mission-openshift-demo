#include "SurveyMode.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <numbers>

namespace {
const std::string kModeName = "SignSurvey";
constexpr int64_t kMaxImageAge = 500'000'000;
constexpr int64_t kMaxTelemetryGap = 250'000'000;
constexpr int64_t kCoverageGap = 1'000'000'000;
constexpr int64_t kHold = 2'000'000'000;
constexpr float kSettled = .25f;
constexpr float kYaw = 75.f * std::numbers::pi_v<float> / 180.f;
const Eigen::Vector3f kRecovery{-4.f, 8.f, -3.f};

Eigen::Vector3f ned(const std::array<float, 3>& point)
{
  return {point[0], point[1], point[2]};
}
}  // namespace

SurveyMode::SurveyMode(rclcpp::Node& node, DemoSession& session)
    : ModeBase(node, kModeName), _node(node), _session(session),
      _setpoint(std::make_shared<px4_ros2::TrajectorySetpointType>(*this))
{
  _attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
  _observations = node.create_subscription<Observation>(
      "/mission/survey_observation", rclcpp::SensorDataQoS().keep_last(1),
      [this](const Observation& observation) { observe(observation); });
}

void SurveyMode::onActivate()
{
  _state = State::LaneEntry;
  _pass = 0;
  _retry_after = 0;
  _hold_start.reset();
  _last_frame = 0;
  _coverage_incomplete = false;
  _session.phase("search_searching", "Flying to survey pass 1 of 6.");
}

void SurveyMode::observe(const Observation& observation)
{
  if (!isActive() || (_state != State::Pass && _state != State::Align)) return;
  const auto now = _node.now().nanoseconds();
  const auto stamp = rclcpp::Time(observation.stamp).nanoseconds();
  if (stamp < _pass_start || stamp <= _last_frame || stamp > now || now - stamp > kMaxImageAge) return;

  _last_frame = stamp;
  if (_state == State::Align) {
    updateAlignment(observation, stamp);
    return;
  }
  checkCoverage(now);
  _last_fresh = now;
  ++_fresh_frames;
  if (_gap_active) {
    _gap_active = false;
    _session.phase("search_searching", "Detector updates resumed. Survey coverage is incomplete.");
  }

  if (now < _retry_after || !observation.sign_detected || !observation.face_valid ||
      !_attitude->lastValid() || !_session.surveyPositionFresh(now)) return;
  _sign_hold = _session.position();
  _sign_hold.z() = survey::kDown;
  _align_start = now;
  _last_face = 0;
  _state = State::Align;
  _session.setSightingFrame(stamp);
  _session.setSurveyPassStart(0);
  RCLCPP_INFO(_node.get_logger(), "SURVEY sign_spotted sim_ns=%" PRId64 " frame_ns=%" PRId64
              " pass=%d confidence=%.3f", now, stamp, _pass+1, observation.confidence);
  _session.phase("search_spotted", "Aligning the downward camera over the inspection sign.");
  updateAlignment(observation, stamp);
}

void SurveyMode::checkCoverage(int64_t now)
{
  const auto gap = now - _last_fresh;
  _max_gap = std::max(_max_gap, gap);
  if (gap < kCoverageGap || _gap_active) return;
  _gap_active = true;
  _coverage_incomplete = true;
  RCLCPP_WARN(_node.get_logger(), "SURVEY detector_gap sim_ns=%" PRId64 " pass=%d gap_ns=%" PRId64,
              now, _pass + 1, gap);
  _session.phase("search_searching", "Waiting for fresh detector results. Survey coverage is incomplete.");
}

void SurveyMode::logPass(const char* outcome, int64_t now) const
{
  RCLCPP_INFO(_node.get_logger(),
      "SURVEY pass_end sim_ns=%" PRId64 " pass=%d fresh=%u max_gap_ns=%" PRId64 " outcome=%s",
      now, _pass + 1, _fresh_frames, _max_gap, outcome);
}

void SurveyMode::returnToPad(int64_t now, bool spotted)
{
  _state = State::Recovery;
  const std::string summary = _coverage_incomplete ? "Survey coverage incomplete." :
      spotted ? "Stop-sign hold complete." : "All six passes complete. No confirmed sign hold.";
  _session.setDemoSummary(summary);
  RCLCPP_INFO(_node.get_logger(), "SURVEY returning sim_ns=%" PRId64 " spotted=%d coverage_complete=%d",
              now, spotted, !_coverage_incomplete);
  _session.phase("search_returning", summary + " Flying to the recovery pad.");
}

void SurveyMode::updateAlignment(const Observation& observation, int64_t stamp)
{
  const auto now = _node.now().nanoseconds();
  bool centered = false;
  if (observation.sign_detected && observation.face_valid &&
      _session.surveyPositionFresh(now) && _attitude->lastValid()) {
    const auto& p = observation.box_midpoint_on_sign_m;
    // The inspection face is stationary on level ground. Reject poses inconsistent
    // with that plane before using the live box midpoint on the inspection face.
    const Eigen::Vector3f body(-p[1], p[0], p[2]);
    const Eigen::Vector3f sign = _session.position() + _attitude->attitude() * body;
    if (sign.allFinite() && std::abs(sign.z()) < 1.5f &&
        sign.x() >= survey::kNorthStart - survey::kLaneSpacing &&
        sign.x() <= survey::laneNorth(survey::kPasses) &&
        sign.y() >= survey::kEastStart - 1.f && sign.y() <= survey::kEastEnd + 1.f) {
      const float weight = _last_face ? .25f : 1.f;
      _sign_hold.head<2>() += weight * (sign.head<2>() - _sign_hold.head<2>());
      centered = std::abs(observation.box_offset[0]) <= .0625f &&
                 std::abs(observation.box_offset[1]) <= .0625f &&
                 std::abs(_session.position().z() - survey::kDown) < kSettled &&
                 _session.velocity().norm() < kSettled;
      if (stamp - _last_face > kMaxTelemetryGap) {
        _hold_start.reset();
        _session.setHoldFrame(0);
      }
      _last_face = stamp;
    }
  }
  if (!centered) {
    if (_hold_start) {
      _hold_start.reset();
      _session.phase("search_spotted", "Checking the stop sign again before holding.");
    }
    return;
  }
  if (!_hold_start) {
    _hold_start = stamp;
    _session.setHoldFrame(stamp);
    RCLCPP_INFO(_node.get_logger(), "SURVEY hold_start sim_ns=%" PRId64, stamp);
    _session.phase("search_investigating", "Stop sign centered. Holding for two simulation seconds.");
  } else if (stamp - *_hold_start >= kHold) {
    RCLCPP_INFO(_node.get_logger(), "SURVEY hold_complete frame_ns=%" PRId64 " start_ns=%" PRId64,
                stamp, *_hold_start);
    logPass("sign_hold", now);
    returnToPad(now, true);
  }
}

void SurveyMode::updateSetpoint([[maybe_unused]] float dt_s)
{
  const auto now = _node.now().nanoseconds();
  Eigen::Vector3f target;
  switch (_state) {
    case State::LaneEntry: target = ned(survey::endpoint(_pass, true)); break;
    case State::Pass: target = ned(survey::endpoint(_pass, false)); break;
    case State::Align: target = _sign_hold; break;
    case State::Recovery: target = kRecovery; break;
  }
  _setpoint->update(px4_ros2::TrajectorySetpoint().withPosition(target).withYaw(kYaw));

  if (_state == State::Align) {
    if (now - std::max(_last_face, _align_start) > 2'000'000'000LL ||
        now - _align_start > 20'000'000'000LL) {
      _hold_start.reset();
      _state = State::Pass;
      _retry_after = now + 3'000'000'000LL;
      _last_fresh = now;
      _session.setSurveyPassStart(now);
      _session.phase("search_searching", "Sign view lost. Resuming the interrupted pass.");
    }
    return;
  }
  if (_state == State::Pass) checkCoverage(now);
  if (!_session.surveyPositionFresh(now) || (_session.position() - target).norm() >= .5f) return;

  switch (_state) {
    case State::LaneEntry:
      _state = State::Pass;
      _pass_start = _last_fresh = now;
      _session.setSurveyPassStart(now);
      _fresh_frames = _max_gap = 0;
      _gap_active = false;
      RCLCPP_INFO(_node.get_logger(), "SURVEY pass_start sim_ns=%" PRId64 " pass=%d", now, _pass + 1);
      _session.phase("search_searching", "Searching survey pass " + std::to_string(_pass + 1) + " of 6." +
          (_coverage_incomplete ? " Survey coverage is incomplete." : ""));
      break;
    case State::Pass:
      if (_fresh_frames == 0) _coverage_incomplete = true;
      _session.setSurveyPassStart(0);
      logPass("complete", now);
      if (_pass + 1 == survey::kPasses) returnToPad(now, false);
      else { ++_pass; _state = State::LaneEntry; }
      break;
    case State::Recovery:
      completed(px4_ros2::Result::Success);
      break;
    case State::Align:
      break;
  }
}
