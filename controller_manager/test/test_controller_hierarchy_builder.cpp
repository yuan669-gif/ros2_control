#include <gtest/gtest.h>

#include "controller_manager/controller_hierarchy_builder.hpp"

TEST(ControllerHierarchyBuilder, InfersNestedControllersFromJointAnchors)
{
  const std::vector<std::string> links{"base", "arm", "wrist"};
  const std::vector<controller_manager::UrdfJointEdge> joints{
    {"arm_joint", "base", "arm"}, {"wrist_joint", "arm", "wrist"}};
  const std::vector<controller_manager::ControllerResourceBinding> bindings{
    {"arm_controller", {"arm_joint"}, ""},
    {"wrist_controller", {"wrist_joint"}, ""}};
  const auto nodes = controller_manager::build_controller_hierarchy_from_urdf(
    links, joints, bindings);
  ASSERT_EQ(nodes.size(), 2u);
  EXPECT_EQ(nodes[0].parent, "");
  EXPECT_EQ(nodes[1].parent, "arm_controller");
}

TEST(ControllerHierarchyBuilder, SupportsCompositeAnchorWithoutHardwareClaim)
{
  const std::vector<std::string> links{"base", "wheel"};
  const std::vector<controller_manager::UrdfJointEdge> joints{
    {"wheel_joint", "base", "wheel"}};
  const std::vector<controller_manager::ControllerResourceBinding> bindings{
    {"base_controller", {}, "base"}, {"wheel_controller", {"wheel_joint"}, ""}};
  const auto plan = controller_manager::build_controller_plan_from_urdf(links, joints, bindings);
  EXPECT_EQ(plan.preorder, (std::vector<std::size_t>{0, 1}));
  EXPECT_EQ(plan.postorder, (std::vector<std::size_t>{1, 0}));
}

TEST(ControllerHierarchyBuilder, RejectsUnknownJoint)
{
  EXPECT_THROW(
    controller_manager::build_controller_hierarchy_from_urdf(
      {"base"}, {}, {{"controller", {"missing_joint"}, ""}}),
    std::invalid_argument);
}
