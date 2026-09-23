#include <gtest/gtest.h>

#include "controller_manager/hierarchy.hpp"

namespace
{
using controller_manager::ControllerHierarchyNode;

TEST(ControllerHierarchy, BuildsBidirectionalPlans)
{
  const std::vector<ControllerHierarchyNode> nodes{
    {"base", ""}, {"left_module", "base"}, {"left_joint", "left_module"},
    {"right_module", "base"}, {"right_joint", "right_module"}};
  const auto plan = controller_manager::build_controller_hierarchy(nodes);
  ASSERT_EQ(plan.preorder, (std::vector<std::size_t>{0, 1, 2, 3, 4}));
  ASSERT_EQ(plan.postorder, (std::vector<std::size_t>{2, 1, 4, 3, 0}));
}

TEST(ControllerHierarchy, RejectsMultipleRoots)
{
  EXPECT_THROW(
    controller_manager::build_controller_hierarchy(
      std::vector<ControllerHierarchyNode>{{"a", ""}, {"b", ""}}),
    std::invalid_argument);
}

TEST(ControllerHierarchy, RejectsCycles)
{
  EXPECT_THROW(
    controller_manager::build_controller_hierarchy(
      std::vector<ControllerHierarchyNode>{{"a", ""}, {"b", "c"}, {"c", "b"}}),
    std::invalid_argument);
}

TEST(ControllerHierarchy, ExecutesChildrenThenParents)
{
  const auto plan = controller_manager::build_controller_hierarchy(
    std::vector<ControllerHierarchyNode>{{"root", ""}, {"child", "root"}});
  std::vector<std::size_t> state;
  std::vector<std::size_t> command;
  controller_manager::execute_controller_hierarchy(
    plan, [&](const auto node) { state.push_back(node); },
    [&](const auto node) { command.push_back(node); });
  EXPECT_EQ(state, (std::vector<std::size_t>{1, 0}));
  EXPECT_EQ(command, (std::vector<std::size_t>{0, 1}));
}
}  // namespace
