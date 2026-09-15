#include <gtest/gtest.h>

#include "controller_interface/hierarchical_controller_interface.hpp"

namespace
{
class TestHierarchicalController : public controller_interface::HierarchicalControllerInterface
{
public:
  controller_interface::return_type update_state(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    ++state_calls;
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_command(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    ++command_calls;
    return controller_interface::return_type::OK;
  }

  int state_calls = 0;
  int command_calls = 0;
};

TEST(HierarchicalControllerInterface, ExposesTwoPhases)
{
  TestHierarchicalController controller;
  EXPECT_EQ(
    controller.update_state(rclcpp::Time{}, rclcpp::Duration{}),
    controller_interface::return_type::OK);
  EXPECT_EQ(
    controller.update_command(rclcpp::Time{}, rclcpp::Duration{}),
    controller_interface::return_type::OK);
  EXPECT_EQ(controller.state_calls, 1);
  EXPECT_EQ(controller.command_calls, 1);
}
}  // namespace
