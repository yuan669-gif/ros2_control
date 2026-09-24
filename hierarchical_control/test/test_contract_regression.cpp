// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Regression tests for contract gaps found in review doc/REVIEW_HUMBLE_WORK_2026-09-23.md.
//
// Each test here was written to FAIL against the reviewed revision b1bf616, so that a passing
// result means the gap was actually closed and not merely re-described. The gaps covered:
//
//   R3  a controller that returns OK without writing one of its declared outputs keeps the
//       PREVIOUS cycle's value, and the kernel re-stamps it as current -> stale data presented as
//       fresh, with no diagnostic.
//   R4  committing several leaves one at a time leaves the group's internal committed state
//       partially updated when a later sink fails.
//   R6  a parent writing two ports of the SAME child is rejected as "multiple writers", because
//       the check tests whether the consumer already has a parent rather than whether that
//       individual port already has a writer.
//
// These are deliberately separate from test_execution_group.cpp, which covers the intended
// behaviour; this file covers the behaviour that must NOT be possible.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hierarchical_control/staged_execution_group.hpp"
#include "hierarchical_control/topology_binding.hpp"
#include "hierarchical_control/typed_ports.hpp"
#include "test_controller_stub.hpp"

namespace hc = hierarchical_control;
using Return = controller_interface::return_type;

namespace
{
/// Node stub whose writes can be suppressed, so "returned OK but produced nothing" is testable.
class PartialNode : public hc::StagedControllerInterface
{
public:
  PartialNode(std::string name, double value) : name_(std::move(name)), value_(value) {}

  PartialNode & as_root()
  {
    is_root_ = true;
    source_ = std::make_unique<Source>(this);
    return *this;
  }
  PartialNode & as_leaf(std::size_t actuator_count = 1)
  {
    is_leaf_ = true;
    actuator_names_.assign(actuator_count, "actuator");
    sink_ = std::make_unique<Sink>(this);
    return *this;
  }

  void set_state_port_count(std::size_t n) { state_port_count_ = n; }

  std::vector<std::string> staged_state_ports() const override
  {
    std::vector<std::string> out;
    for (std::size_t i = 0; i < state_port_count_; ++i) {out.push_back("state" + std::to_string(i));}
    return out;
  }
  std::vector<std::string> staged_actuator_ports() const override {return actuator_names_;}

  hc::StagedCommandSink * staged_command_sink() noexcept override
  {
    return is_leaf_ ? sink_.get() : nullptr;
  }
  hc::StagedReferenceSource * staged_reference_source() noexcept override
  {
    return is_root_ ? source_.get() : nullptr;
  }

  Return update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hc::StagedContext &,
    const hc::StagedInputView & children, hc::StagedValueWriter state) noexcept override
  {
    ++state_calls;
    // R3 knob: return OK while writing nothing at all.
    if (skip_state_write) {return Return::OK;}
    // R3 knob: write only the first element of several.
    const std::size_t limit = write_only_first_state ? std::min<std::size_t>(1, state.size())
                                                     : state.size();
    const double sum = children.size() > 0 && children[0].size() > 0 ? children[0][0] : value_;
    for (std::size_t i = 0; i < limit; ++i) {state[i] = sum + static_cast<double>(i) * 0.5;}
    return Return::OK;
  }

  Return update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hc::StagedContext &,
    const hc::StagedValueView &, const hc::StagedValueView &,
    const hc::StagedReferenceWriter &, hc::StagedValueWriter actuators) noexcept override
  {
    ++command_calls;
    if (skip_actuator_write) {return Return::OK;}
    const double produced =
      (committed_value_override >= 0.0) ? committed_value_override : value_;
    for (std::size_t i = 0; i < actuators.size(); ++i) {actuators[i] = produced;}
    return Return::OK;
  }

  bool skip_state_write = false;
  bool write_only_first_state = false;
  bool skip_actuator_write = false;
  bool fail_commit = false;
  double committed_value_override = -1.0;
  int state_calls = 0;
  int command_calls = 0;
  std::vector<double> committed;

private:
  class Sink : public hc::StagedCommandSink
  {
  public:
    explicit Sink(PartialNode * owner) : owner_(owner) {}
    bool commit(const double * values, std::size_t size) noexcept override
    {
      ++owner_->commit_calls;
      if (owner_->fail_commit) {return false;}
      owner_->committed.assign(values, values + size);
      return true;
    }
    int commit_calls = 0;
  private:
    PartialNode * owner_;
  };

  class Source : public hc::StagedReferenceSource
  {
  public:
    explicit Source(PartialNode * owner) : owner_(owner) {}
    bool read(std::uint64_t, std::int64_t, double * values, std::size_t size) noexcept override
    {
      for (std::size_t i = 0; i < size; ++i) {values[i] = owner_->reference_value;}
      return true;
    }
  private:
    PartialNode * owner_;
  };

  std::string name_;
  double value_;
  std::size_t state_port_count_ = 1;
  bool is_leaf_ = false;
  bool is_root_ = false;
  std::vector<std::string> actuator_names_;
  std::unique_ptr<Sink> sink_;
  std::unique_ptr<Source> source_;

public:
  double reference_value = 1.0;
  int commit_calls = 0;
};

/// leaf <- module <- root (single chain).
struct Chain
{
  PartialNode root{"root", 10.0};
  PartialNode module{"module", 20.0};
  PartialNode leaf{"leaf", 30.0};
  std::shared_ptr<hc::StagedExecutionGroup> group;

  hc::StagedExecutionGroup::Spec spec()
  {
    return {{"root", "module", "leaf"}, {&root, &module, &leaf}, {"", "root", "module"}};
  }
};
}  // namespace

/// R3: a node that returns OK without writing its declared state ports must NOT have the previous
/// cycle's value silently accepted as this cycle's output.
///
/// This is the exact reviewed scenario: one successful cycle establishes a value, then the node
/// stops writing while still reporting success. Before the fix the group kept committing and the
/// stale value was re-stamped as current.
TEST(ContractRegression, r3_missing_state_write_is_not_accepted_as_fresh)
{
  Chain chain;
  chain.root.as_root();
  chain.leaf.as_leaf();
  chain.group = hc::StagedExecutionGroup::create_library(chain.spec());

  // Cycle 1: module writes normally, so a committed value exists.
  const auto good = chain.group->run_ns(1000, 1);
  ASSERT_EQ(hc::StagedStatus::committed, good.status);

  // Cycle 2: module returns OK but writes nothing.
  chain.module.skip_state_write = true;
  const auto missing = chain.group->run_ns(2000, 1);
  EXPECT_NE(hc::StagedStatus::committed, missing.status)
    << "R3: an unwritten output must make the cycle fail, not be re-stamped as fresh";

  // And it must recover once the node writes again.
  chain.module.skip_state_write = false;
  const auto recovered = chain.group->run_ns(3000, 1);
  EXPECT_EQ(hc::StagedStatus::committed, recovered.status);
}

/// R3 (partial): writing only some of several declared state ports must also be detected.
TEST(ContractRegression, r3_partial_state_write_is_detected)
{
  Chain chain;
  chain.root.as_root();
  chain.module.set_state_port_count(3);
  chain.module.write_only_first_state = true;  // writes 1 of 3 ports
  chain.leaf.as_leaf();
  chain.group = hc::StagedExecutionGroup::create_library(chain.spec());

  const auto result = chain.group->run_ns(1000, 1);
  EXPECT_NE(hc::StagedStatus::committed, result.status)
    << "R3: a partially written output group must not be treated as a complete one";
}

/// R3 (actuator): a leaf that returns OK without writing its actuator ports must not commit.
TEST(ContractRegression, r3_missing_actuator_write_is_not_committed)
{
  Chain chain;
  chain.root.as_root();
  chain.module.skip_state_write = false;
  chain.leaf.as_leaf();
  chain.leaf.skip_actuator_write = true;
  chain.group = hc::StagedExecutionGroup::create_library(chain.spec());

  // An unwritten actuator port makes the cycle fail. It is reported as state_failed because the
  // state stage of this chain runs first and the failure propagates before commit is reached; the
  // important property is that the cycle does NOT commit and the sink is never called.
  const auto result = chain.group->run_ns(1000, 1);
  EXPECT_NE(hc::StagedStatus::committed, result.status)
    << "R3: an unwritten actuator port must not be committed";
  EXPECT_TRUE(chain.leaf.committed.empty())
    << "R3: the sink must not be called when nothing was produced";
}

/// R4: with two leaves, a failure on the SECOND sink must not leave the group's internal
/// committed view carrying the FIRST leaf's new value.
///
/// `committed_cycle()` does not advance either way (the loop returns before reaching it), so this
/// test inspects `committed_actuators()` — the group's own published command image. Before the fix
/// leaf_a's slot was updated while leaf_b's was not, i.e. the internal image described a mix of two
/// different cycles.
TEST(ContractRegression, r4_late_sink_failure_leaves_internal_view_consistent)
{
  PartialNode root{"root", 10.0};
  PartialNode leaf_a{"leaf_a", 1.0};
  PartialNode leaf_b{"leaf_b", 2.0};
  root.as_root();
  leaf_a.as_leaf();
  leaf_b.as_leaf();

  const hc::StagedExecutionGroup::Spec spec{
    {"root", "leaf_a", "leaf_b"}, {&root, &leaf_a, &leaf_b}, {"", "root", "root"}};
  auto group = hc::StagedExecutionGroup::create_library(spec);

  // One clean cycle establishes a committed image.
  const auto good = group->run_ns(1000, 1);
  ASSERT_EQ(hc::StagedStatus::committed, good.status);
  const std::uint64_t cycle_after_good = group->committed_cycle();
  const std::vector<double> image_after_good = group->committed_actuators();
  ASSERT_EQ(2u, image_after_good.size());

  // Change both leaves' outputs, and make ONLY the second sink fail.
  leaf_a.committed_value_override = 111.0;
  leaf_b.committed_value_override = 222.0;
  leaf_b.fail_commit = true;

  const auto bad = group->run_ns(2000, 1);
  EXPECT_EQ(hc::StagedStatus::command_failed, bad.status);

  // The group's internal image must still describe the LAST FULLY COMMITTED cycle.
  EXPECT_EQ(cycle_after_good, group->committed_cycle())
    << "R4: committed_cycle must not advance on a partially committed group";
  EXPECT_EQ(image_after_good, group->committed_actuators())
    << "R4: a late sink failure must not leave the internal command image half-updated";
}

/// R6: one parent providing TWO ports to the same child is a single writer, not a conflict.
///
/// Direction convention, taken from the staged kernel's own contract (`StagedGroupMember` in
/// `staged_execution_group.hpp`) and confirmed end-to-end by
/// `controller_manager/test/test_staged_execution_group.cpp`: the controller that CLAIMS
/// "<owner>/port" is the reference *producer*, hence the PARENT, and the prefix owner is the CHILD
/// that reads it. (The claimant writes the claimed command interface.) The rule lives in
/// `derive_parents_from_claimed_interfaces` (hierarchy.hpp), testable without a full
/// ControllerInterfaceBase stub.
///
/// NOTE: an earlier revision of this test asserted the OPPOSITE direction; that inverted the
/// hierarchy and broke `test_staged_execution_group` / `test_hierarchy_comparison`.
TEST(ContractRegression, r6_one_parent_may_provide_two_ports_to_the_same_child)
{
  // `chassis` writes both ports; `wheel` reads them. This is an ordinary 2-D reference.
  const std::vector<std::string> names{"wheel", "chassis"};
  const std::vector<std::vector<std::string>> claimed{{}, {"wheel/x", "wheel/y"}};

  std::vector<std::string> parents;
  ASSERT_NO_THROW(parents = hc::derive_parents_from_claimed_interfaces(names, claimed))
    << "R6: two ports from ONE parent to one child must be accepted";
  ASSERT_EQ(2u, parents.size());
  EXPECT_EQ("chassis", parents[0]) << "the claimant is the parent of the port owner";
  EXPECT_EQ("", parents[1]) << "the claimant is the root here";
}

/// One parent taking references from two different children is ordinary fan-out, not a conflict.
TEST(ContractRegression, r6_one_parent_with_two_children_is_consistent)
{
  const std::vector<std::string> names{"p", "c1", "c2"};
  const std::vector<std::vector<std::string>> claimed{{"c1/x", "c2/y"}, {}, {}};

  std::vector<std::string> parents;
  ASSERT_NO_THROW(parents = hc::derive_parents_from_claimed_interfaces(names, claimed))
    << "one controller may take references from two different controllers";
  ASSERT_EQ(3u, parents.size());
  EXPECT_EQ("", parents[0]) << "the single claimant is the root";
  EXPECT_EQ("p", parents[1]);
  EXPECT_EQ("p", parents[2]);
}

/// A child fed by two different parents is rejected: a tree node has exactly one parent.
TEST(ContractRegression, r6_child_with_two_parents_is_rejected)
{
  // Two claimants take different ports of the SAME owner, so `wheel` would have two parents.
  const std::vector<std::string> names{"wheel", "c1", "c2"};
  const std::vector<std::vector<std::string>> claimed{{}, {"wheel/x"}, {"wheel/y"}};

  EXPECT_THROW(
    hc::derive_parents_from_claimed_interfaces(names, claimed), std::invalid_argument);
}

/// Two claimants taking the SAME port is two writers for one port and is rejected.
TEST(ContractRegression, r6_port_with_two_writers_is_rejected)
{
  const std::vector<std::string> names{"wheel", "c1", "c2"};
  const std::vector<std::vector<std::string>> claimed{{}, {"wheel/x"}, {"wheel/x"}};

  EXPECT_THROW(
    hc::derive_parents_from_claimed_interfaces(names, claimed), std::invalid_argument);
}

/// One claimant listing the SAME port twice is a duplicate claim and is rejected.
TEST(ContractRegression, r6_duplicate_claim_by_one_claimant_is_rejected)
{
  const std::vector<std::string> names{"wheel", "c1"};
  const std::vector<std::vector<std::string>> claimed{{}, {"wheel/x", "wheel/x"}};

  EXPECT_THROW(
    hc::derive_parents_from_claimed_interfaces(names, claimed), std::invalid_argument);
}

/// A node claiming its own port is not a reference into the group; it is its own hardware
/// interface and must not make the node its own parent.
TEST(ContractRegression, r6_self_owned_port_is_treated_as_hardware)
{
  const std::vector<std::string> names{"only"};
  const std::vector<std::vector<std::string>> claimed{{"only/velocity", "joint1/velocity"}};

  std::vector<std::string> parents;
  ASSERT_NO_THROW(parents = hc::derive_parents_from_claimed_interfaces(names, claimed));
  ASSERT_EQ(1u, parents.size());
  EXPECT_EQ("", parents[0]) << "self-owned and foreign ports are both ordinary hardware ports";
}

/// Ports whose owner is not part of the group are ordinary hardware interfaces and are ignored.
TEST(ContractRegression, r6_foreign_owner_is_ignored)
{
  const std::vector<std::string> names{"only"};
  const std::vector<std::vector<std::string>> claimed{{"joint1/velocity", "hardware/thing"}};

  std::vector<std::string> parents;
  ASSERT_NO_THROW(parents = hc::derive_parents_from_claimed_interfaces(names, claimed));
  ASSERT_EQ(1u, parents.size());
  EXPECT_EQ("", parents[0]) << "a single node with only hardware interfaces is a root";
}


/// ---------------------------------------------------------------------------------------------
/// R5: instance pointers must survive the binding even when a controller inherits the staged
/// interface at a NON-ZERO offset.
///
/// The reviewed revision erased instances to `void*` and recovered them with a cast back to the
/// staged interface. For a class that inherits both ControllerInterfaceBase and
/// StagedControllerInterface, that second base usually sits at an offset, and the round trip loses
/// the adjustment. This test measures the offset first, so it cannot pass vacuously on a layout
/// where the two pointers happen to coincide.
/// ---------------------------------------------------------------------------------------------
namespace
{
namespace tp = hierarchical_control::typed_ports;
namespace dm = hierarchical_control::dimensions;
namespace st = hierarchical_control::static_topology;
namespace tc = hierarchical_control::topology_contract;

struct mi_owner_n
{
  static constexpr auto value = st::NameOf("mi_owner/state");
};
using mi_owner_port = tc::Port<mi_owner_n, dm::Position>;
struct mi_node_n
{
  static constexpr auto value = st::NameOf("mi_owner");
};
using mi_node = st::Root<mi_node_n>;
using mi_contract = tc::Contract<tc::PortList<>, tc::PortList<mi_owner_port>>;

/// A controller whose StagedControllerInterface base is NOT the first base.
class MultiInheritController : public hierarchical_control_test::MinimalController,
                               public hierarchical_control::StagedControllerInterface
{
public:
  MultiInheritController() : MinimalController("mi_owner") {}

  std::vector<std::string> staged_state_ports() const override {return {"mi_owner/state"};}

  Return update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hc::StagedContext &,
    const hc::StagedInputView &, hc::StagedValueWriter state) noexcept override
  {
    for (std::size_t i = 0; i < state.size(); ++i) {state[i] = 7.0;}
    return Return::OK;
  }

  Return update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hc::StagedContext &,
    const hc::StagedValueView &, const hc::StagedValueView &, const hc::StagedReferenceWriter &,
    hc::StagedValueWriter) noexcept override
  {
    return Return::OK;
  }
};
}  // namespace

TEST(ContractRegression, r5_non_zero_base_offset_survives_the_binding)
{
  MultiInheritController controller;

  auto * as_controller =
    static_cast<controller_interface::ControllerInterfaceBase *>(&controller);
  auto * as_staged = static_cast<hc::StagedControllerInterface *>(&controller);
  if (as_controller == static_cast<void *>(as_staged))
  {
    GTEST_SKIP() << "this layout has no base offset, so it cannot exercise R5";
  }

  const auto binding = tc::make_leaf<mi_node, mi_contract>(&controller);
  const auto rows = tc::build_spec_rows(binding);
  ASSERT_EQ(1u, rows.names.size());

  // The stored pointer must be the ControllerInterfaceBase subobject, not the object address.
  EXPECT_EQ(as_controller, rows.instances[0])
    << "the binding must store the adjusted base pointer";

  // And the kernel must recover a *working* staged interface from it: a cast that ignored the
  // offset would land on the wrong subobject and this run would not reach the controller.
  auto group = hierarchical_control::topology_binding::create_library_group(binding);
  ASSERT_NE(nullptr, group);
  const auto result = group->run_ns(0, 1000);
  EXPECT_EQ(hc::StagedStatus::committed, result.status);
}
