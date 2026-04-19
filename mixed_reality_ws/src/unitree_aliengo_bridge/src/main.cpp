#include "unitree_aliengo_bridge/aliengo_bridge_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace unitree_aliengo_bridge
{

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AliengoBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

}  // namespace unitree_aliengo_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_aliengo_bridge::AliengoBridgeNode)
