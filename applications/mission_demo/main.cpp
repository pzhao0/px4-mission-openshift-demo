#include "MissionExecutor.hpp"
#include "MissionModes.hpp"
#include "ReturnToStart.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mission_demo");
  DemoSession session(*node);
  DemoWaypoints waypoints(*node);
  DemoPrecisionLand landing(*node);
  ReturnToStart return_mode(*node, session);
  MissionExecutor demonstration(waypoints, landing, session);
  MissionExecutor returning(return_mode, landing, session, MissionExecutor::Operation::Return);
  if (!demonstration.doRegister() || !landing.doRegister() || !returning.doRegister()) {
    RCLCPP_ERROR(node->get_logger(), "Mission demo registration failed");
    rclcpp::shutdown();
    return 1;
  }
  session.setModeIds(waypoints.id(), return_mode.id());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
