#include "MissionExecutor.hpp"
#include "MissionModes.hpp"
#include "ReturnToStart.hpp"
#include "SurveyMode.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mission_demo");
  DemoSession session(*node);
  const bool survey_enabled = node->declare_parameter("survey_enabled", false);
  if (survey_enabled && !node->get_parameter("use_sim_time").as_bool()) {
    RCLCPP_ERROR(node->get_logger(), "SignSurvey requires simulation time");
    rclcpp::shutdown();
    return 1;
  }
  DemoWaypoints waypoints(*node);
  DemoPrecisionLand landing(*node);
  ReturnToStart return_mode(*node, session);
  MissionExecutor demonstration(waypoints, landing, session);
  MissionExecutor returning(return_mode, landing, session, MissionExecutor::Operation::Return);
  std::unique_ptr<SurveyMode> survey;
  std::unique_ptr<MissionExecutor> survey_executor;
  if (survey_enabled) {
    survey = std::make_unique<SurveyMode>(*node, session);
    survey_executor = std::make_unique<MissionExecutor>(*survey, landing, session);
  }
  if (!demonstration.doRegister() || !landing.doRegister() || !returning.doRegister() ||
      (survey_executor && !survey_executor->doRegister())) {
    RCLCPP_ERROR(node->get_logger(), "Mission demo registration failed");
    rclcpp::shutdown();
    return 1;
  }
  session.setModeIds(waypoints.id(), return_mode.id(), survey ? std::optional{survey->id()} : std::nullopt);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
