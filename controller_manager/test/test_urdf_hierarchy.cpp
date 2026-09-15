#include <gtest/gtest.h>

#include "controller_manager/urdf_hierarchy.hpp"

TEST(UrdfHierarchy, DerivesPlansFromLinkJointTree)
{
  const auto result = controller_manager::build_hierarchy_from_urdf(
    {"base", "left", "right"},
    {{"left_joint", "base", "left"}, {"right_joint", "base", "right"}});
  ASSERT_EQ(result.plan.preorder, (std::vector<std::size_t>{0, 1, 2}));
  ASSERT_EQ(result.plan.postorder, (std::vector<std::size_t>{1, 2, 0}));
}

TEST(UrdfHierarchy, RejectsMultipleRoots)
{
  EXPECT_THROW(
    controller_manager::build_hierarchy_from_urdf({"a", "b"}, {}), std::invalid_argument);
}
