#include <gtest/gtest.h>

#include "controller_manager/hierarchical_controller_executor.hpp"

namespace
{
class HierarchicalStub : public controller_interface::ControllerInterfaceBase,
                         public controller_interface::HierarchicalControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }
  bool is_chainable() const override { return false; }
  std::vector<hardware_interface::CommandInterface> export_reference_interfaces() override
  {
    return {};
  }
  bool set_chained_mode(bool) override { return false; }
  bool is_in_chained_mode() const override { return false; }
  controller_interface::return_type update(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return controller_interface::return_type::ERROR;
  }
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

TEST(HierarchicalControllerExecutor, DispatchesBothDirections)
{
  auto root = std::make_shared<HierarchicalStub>();
  auto child = std::make_shared<HierarchicalStub>();
  controller_manager::HierarchicalControllerExecutor executor;
  executor.set_controllers({root, child});
  executor.set_plan(controller_manager::build_controller_hierarchy(
    {{"root", ""}, {"child", "root"}}));
  EXPECT_EQ(
    executor.update(rclcpp::Time{}, rclcpp::Duration{}),
    controller_interface::return_type::OK);
  EXPECT_EQ(root->state_calls, 1);
  EXPECT_EQ(root->command_calls, 1);
  EXPECT_EQ(child->state_calls, 1);
  EXPECT_EQ(child->command_calls, 1);
}
}  // namespace
