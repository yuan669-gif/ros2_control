// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Corollary 1 (doc/FORMAL_MODEL.md) on REAL ros2_control code, not on a port.
//
// A bidirectional pair: the parent claims `child/ref` (reference edge) AND `child/state`
// (state edge). Corollary 1 predicts that any order respecting the reference edge must leave the
// state edge stale, and that the manager's single-pass ordering therefore always puts the
// reference producer first - independently of registration order.
//
// Humble cannot actually export a controller state interface (that is Jazzy+), so the state edge
// is expressed only as a claimed name: this test checks the ORDERING behaviour of the real
// `controller_sorting` comparator, which is exactly where the conflict lives.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "test_staged_controller/test_staged_controller.hpp"

namespace
{
using TestStagedController = test_staged_controller::TestStagedController;
using Return = controller_interface::return_type;

constexpr char kParent[] = "ord_parent";
constexpr char kChild[] = "ord_child";
constexpr char kType[] = "ordering_test";

controller_interface::InterfaceConfiguration individual(const std::vector<std::string> & names)
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names = names;
  return cfg;
}

std::vector<std::string> LoadedNames(const controller_manager::ControllerManager & cm)
{
  std::vector<std::string> names;
  for (const auto & spec : cm.get_loaded_controllers()) {names.push_back(spec.info.name);}
  return names;
}

class TestUpstreamOrdering : public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  void Build(bool parent_registered_first)
  {
    parent_ = std::make_shared<TestStagedController>();
    child_ = std::make_shared<TestStagedController>();

    // Parent: produces the child's reference (reference edge) and consumes the child's state.
    parent_->set_reference_interface_names({"src"});
    parent_->set_command_interface_configuration(individual({std::string(kChild) + "/ref"}));
    parent_->set_state_interface_configuration(individual({std::string(kChild) + "/state"}));

    // Child: exports the reference and drives one hardware command interface.
    child_->set_reference_interface_names({"ref"});
    child_->set_actuator_ports({"joint2/velocity"});
    child_->set_command_interface_configuration(individual({"joint2/velocity"}));
    child_->set_state_interface_configuration(individual({}));

    if (parent_registered_first)
    {
      cm_->add_controller(parent_, kParent, kType);
      cm_->add_controller(child_, kChild, kType);
    }
    else
    {
      cm_->add_controller(child_, kChild, kType);
      cm_->add_controller(parent_, kParent, kType);
    }

    // Configure the child first so its reference interface is exported before the parent is sorted.
    ASSERT_EQ(Return::OK, cm_->configure_controller(kChild));
    ASSERT_EQ(Return::OK, cm_->configure_controller(kParent));
  }

  std::shared_ptr<TestStagedController> parent_, child_;
};

/// Registration order must not change the resulting order: the reference edge wins, so the state
/// edge is the one that ends up stale.
TEST_F(TestUpstreamOrdering, reference_wins_regardless_of_registration_order)
{
  Build(true);
  const auto names = LoadedNames(*cm_);
  ASSERT_EQ(2u, names.size());
  EXPECT_EQ(kParent, names[0]);
  EXPECT_EQ(kChild, names[1]);
}

TEST_F(TestUpstreamOrdering, reference_wins_even_when_child_is_registered_first)
{
  Build(false);
  const auto names = LoadedNames(*cm_);
  ASSERT_EQ(2u, names.size());
  EXPECT_EQ(kParent, names[0]) << "the parent must precede the child, so the state edge is the "
                                  "stale one (Corollary 1)";
  EXPECT_EQ(kChild, names[1]);
}

}  // namespace
