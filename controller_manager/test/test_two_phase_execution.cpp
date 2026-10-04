// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Manager-level tests for the opt-in two-phase execution path.
//
// One controller implementation (`TwoPhaseExampleController`) is run by the REAL ControllerManager in
// two modes over the same ordered controller list:
//
//   native single pass (default) : one `update()` per controller, list walked FORWARD (parents first)
//   two-phase (opt-in)           : `update_phase()` walked BACKWARD, then `handle_phase()` FORWARD
//
// Because the controller's algorithm is an exact power-of-two filter, the two modes produce different
// NUMBERS, not just different call counts: a step of 1.0 at the leaf leaves the root at 0.0 (native:
// the root ran before the child) or 0.125 after three levels (two-phase: the child published first).
// The assertions below are therefore exact and cannot pass for the wrong execution order.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_test_common.hpp"
#include "two_phase_example_controller/two_phase_example_controller.hpp"

namespace
{
using Controller = two_phase_example_controller::TwoPhaseExampleController;
using Return = controller_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

constexpr char kLeaf[] = "tp_leaf";
constexpr char kMid[] = "tp_mid";
constexpr char kRoot[] = "tp_root";
constexpr char kLeft[] = "tp_left";
constexpr char kRight[] = "tp_right";
constexpr char kSolo[] = "tp_solo";
constexpr char kLegacy[] = "tp_legacy";
constexpr char kType[] = "two_phase_example";

/// `is_controller_active()` lives in the manager's own anonymous namespace, so the test reads the
/// lifecycle state directly, exactly as the other manager tests do.
bool ControllerIsActive(const controller_interface::ControllerInterfaceBase & controller)
{
  return controller.get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

/// A chainable controller that does NOT implement `TwoPhaseControllerInterface`: it stands for the
/// native/plugin mode. It exports the same two reference channels, so it can be placed on the other
/// end of an edge from a two-phase controller -- which is exactly the cross-mode case the admission
/// must refuse.
class LegacySourceController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  }

  CallbackReturn on_init() override
  {
    reference_interfaces_.assign(2, 0.0);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    reference_interfaces_.assign(2, 0.0);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return CallbackReturn::SUCCESS;
  }

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.emplace_back(get_node()->get_name(), "target", &reference_interfaces_[0]);
    interfaces.emplace_back(get_node()->get_name(), "estimate", &reference_interfaces_[1]);
    return interfaces;
  }

  controller_interface::return_type update_reference_from_subscribers() override
  {
    return Return::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return Return::OK;
  }
};

struct ChainValues
{
  double leaf = -1.0;
  double mid = -1.0;
  double root = -1.0;
  double leaf_command = 0.0;
};

class TestTwoPhaseExecution : public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  /// `ControllerManagerFixture::TearDownTestCase()` calls `rclcpp::shutdown()`, which segfaults at
  /// process exit in this environment for EVERY binary that uses the fixture -- verified against the
  /// untouched `test_load_controller` and `test_release_interfaces`. Hiding it keeps THIS suite's
  /// ctest verdict meaningful (the process is about to exit, so the context is not needed); it is a
  /// harness property, not a property of anything under test.
  static void TearDownTestCase() {}

  /// Drive `switch_controller()` on another thread while pumping the control loop, which is how the
  /// manager's own handshake completes (see controller_manager_test_common.hpp).
  void SwitchNow(const std::vector<std::string> & start, const std::vector<std::string> & stop)
  {
    auto future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_.get(), start,
      stop, STRICT, true, rclcpp::Duration(0, 0));
    ASSERT_EQ(std::future_status::timeout, future.wait_for(std::chrono::milliseconds(50)));
    for (int i = 0;
         i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cm_->update(TIME, PERIOD);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_EQ(std::future_status::ready, future.wait_for(std::chrono::milliseconds(0)));
    EXPECT_EQ(Return::OK, future.get());
  }

  /// One control cycle through the real read/update/write sequence.
  void Cycle(int count)
  {
    for (int i = 0; i < count; ++i)
    {
      cm_->read(TIME, PERIOD);
      ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
      cm_->write(TIME, PERIOD);
    }
  }

  std::shared_ptr<Controller> MakeNode(
    const std::string & name, std::vector<std::string> commands, std::vector<std::string> states,
    std::vector<std::string> children)
  {
    auto controller = std::make_shared<Controller>();
    controller->set_command_interface_names(std::move(commands));
    controller->set_state_interface_names(std::move(states));
    controller->set_children(std::move(children));
    EXPECT_NE(nullptr, cm_->add_controller(controller, name, kType));
    return controller;
  }

  /// Configure `leaf -> mid -> root`, leaving it INACTIVE. `leaf_rate` of 0 means "follow the
  /// manager"; any other value is set on the leaf's node before it is configured (the manager reads
  /// `update_rate` during configure).
  void BuildChain(unsigned int leaf_rate = 0)
  {
    leaf_ = MakeNode(kLeaf, {"joint2/velocity"}, {"joint2/position"}, {});
    mid_ = MakeNode(kMid, {}, {}, {kLeaf});
    root_ = MakeNode(kRoot, {}, {}, {kMid});

    if (leaf_rate != 0)
    {
      leaf_->get_node()->set_parameter({"update_rate", static_cast<int>(leaf_rate)});
    }

    ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
    ASSERT_EQ(Return::OK, cm_->configure_controller(kMid));
    ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));
    ASSERT_TRUE(leaf_->set_chained_mode(true));
    ASSERT_TRUE(mid_->set_chained_mode(true));
  }

  void ActivateChain()
  {
    SwitchNow({kLeaf}, {});
    SwitchNow({kMid}, {});
    SwitchNow({kRoot}, {});
  }

  /// Build, (optionally) enable two-phase execution, activate, settle at 0, then apply ONE step of
  /// 1.0 on the leaf's hardware state and return the values at the END of that cycle.
  ChainValues RunStep(bool two_phase)
  {
    BuildChain();
    EXPECT_EQ(Return::OK, cm_->set_two_phase_execution(two_phase));
    EXPECT_EQ(two_phase, cm_->two_phase_execution());
    ActivateChain();
    Cycle(5);
    leaf_->set_hardware_state(1.0);
    cm_->read(TIME, PERIOD);
    EXPECT_EQ(Return::OK, cm_->update(TIME, PERIOD));
    cm_->write(TIME, PERIOD);
    return {leaf_->estimate(), mid_->estimate(), root_->estimate(), leaf_->command()};
  }

  /// The order the manager's own sort produced, as names, so a test can say WHICH order it assumes.
  std::vector<std::string> ControllerOrder() const
  {
    std::vector<std::string> names;
    for (const auto & controller : cm_->get_loaded_controllers())
    {
      names.push_back(controller.info.name);
    }
    return names;
  }

  std::shared_ptr<Controller> leaf_, mid_, root_;
  std::shared_ptr<Controller> left_, right_, solo_, first_, second_;
};

// ---------------------------------------------------------------------------------------------
// 1. The default is the upstream single pass, unchanged.
// ---------------------------------------------------------------------------------------------

TEST_F(TestTwoPhaseExecution, the_feature_is_off_by_default)
{
  BuildChain();
  EXPECT_FALSE(cm_->two_phase_execution());
  EXPECT_TRUE(cm_->two_phase_rejected_controllers().empty())
    << "a coherent chain has nothing to reject; the mode being off is a separate fact";
}

TEST_F(TestTwoPhaseExecution, native_single_pass_never_calls_a_two_phase_stage)
{
  RunStep(false);
  EXPECT_GT(leaf_->native_update_calls(), 0);
  EXPECT_GT(mid_->native_update_calls(), 0);
  EXPECT_GT(root_->native_update_calls(), 0);
  EXPECT_EQ(0, leaf_->update_phase_calls());
  EXPECT_EQ(0, leaf_->handle_phase_calls());
  EXPECT_EQ(0, root_->update_phase_calls());
}

// ---------------------------------------------------------------------------------------------
// 2. The defect: with one pass, the parent cannot see the child's state in the same cycle.
// ---------------------------------------------------------------------------------------------

TEST_F(TestTwoPhaseExecution, single_pass_lags_by_one_cycle_at_every_level)
{
  const auto values = RunStep(false);
  // The leaf is the source, so it is never late; every level above it is, by exactly one cycle.
  EXPECT_DOUBLE_EQ(1.0, values.leaf);
  EXPECT_DOUBLE_EQ(0.0, values.mid) << "the parent ran before its child";
  EXPECT_DOUBLE_EQ(0.0, values.root);
}

TEST_F(TestTwoPhaseExecution, single_pass_command_uses_the_previous_cycles_reference)
{
  const auto values = RunStep(false);
  // target was 0 in this cycle for the leaf, so command = 0 - 1 = -1: the parent's fresh command has
  // not reached the leaf yet either.
  EXPECT_DOUBLE_EQ(-1.0, values.leaf_command);
}

// ---------------------------------------------------------------------------------------------
// 3. Two-phase: both directions are fresh in the SAME cycle.
// ---------------------------------------------------------------------------------------------

TEST_F(TestTwoPhaseExecution, enabling_is_accepted_for_a_coherent_chain)
{
  BuildChain();
  ASSERT_EQ(std::vector<std::string>({kRoot, kMid, kLeaf}), ControllerOrder())
    << "the manager sorts a cascade parents-first; the passes rely on it";
  EXPECT_TRUE(cm_->two_phase_rejected_controllers().empty());
  EXPECT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  EXPECT_TRUE(cm_->two_phase_execution());
}

TEST_F(TestTwoPhaseExecution, two_pass_gives_the_parent_the_same_cycle_child_state)
{
  const auto values = RunStep(true);
  EXPECT_DOUBLE_EQ(1.0, values.leaf);
  EXPECT_DOUBLE_EQ(0.5, values.mid) << "mid ingested the leaf's estimate from THIS cycle";
  EXPECT_DOUBLE_EQ(0.25, values.root) << "root ingested mid's estimate from THIS cycle";
}

TEST_F(TestTwoPhaseExecution, two_pass_gives_the_child_the_same_cycle_parent_reference)
{
  const auto values = RunStep(true);
  // root command = 0 - 0.25 = -0.25; mid command = -0.25 - 0.5 = -0.75;
  // leaf command = -0.75 - 1.0 = -1.75. Only an in-cycle forward pass produces that value.
  EXPECT_DOUBLE_EQ(-1.75, values.leaf_command);
}

TEST_F(TestTwoPhaseExecution, each_stage_runs_exactly_once_per_cycle)
{
  BuildChain();
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  ActivateChain();
  const auto leaf_updates = leaf_->update_phase_calls();
  const auto leaf_handles = leaf_->handle_phase_calls();
  const auto mid_updates = mid_->update_phase_calls();
  const auto mid_handles = mid_->handle_phase_calls();
  const auto root_updates = root_->update_phase_calls();
  const auto root_handles = root_->handle_phase_calls();
  Cycle(7);
  EXPECT_EQ(leaf_updates + 7, leaf_->update_phase_calls());
  EXPECT_EQ(leaf_handles + 7, leaf_->handle_phase_calls());
  EXPECT_EQ(mid_updates + 7, mid_->update_phase_calls());
  EXPECT_EQ(mid_handles + 7, mid_->handle_phase_calls());
  EXPECT_EQ(root_updates + 7, root_->update_phase_calls());
  EXPECT_EQ(root_handles + 7, root_->handle_phase_calls());
}

TEST_F(TestTwoPhaseExecution, a_two_phase_member_is_never_handed_to_the_native_loop)
{
  BuildChain();
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  ActivateChain();
  Cycle(4);
  // A switch makes the passes pause for a few cycles (membership may be changing). A member must
  // NOT be moved back to the fused single-pass path in that window.
  SwitchNow({}, {kRoot});
  SwitchNow({kRoot}, {});
  Cycle(4);
  EXPECT_EQ(0, leaf_->native_update_calls());
  EXPECT_EQ(0, mid_->native_update_calls());
  EXPECT_EQ(0, root_->native_update_calls());
}

// ---------------------------------------------------------------------------------------------
// 4. Failure containment: no command stage on a cycle whose state stage failed.
// ---------------------------------------------------------------------------------------------

TEST_F(TestTwoPhaseExecution, a_failed_state_stage_skips_the_whole_command_stage)
{
  BuildChain();
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  ActivateChain();
  Cycle(3);

  const auto root_handles_before = root_->handle_phase_calls();
  const auto mid_handles_before = mid_->handle_phase_calls();
  const auto leaf_handles_before = leaf_->handle_phase_calls();

  mid_->fail_next_update_phase();
  cm_->read(TIME, PERIOD);
  EXPECT_EQ(Return::ERROR, cm_->update(TIME, PERIOD));
  cm_->write(TIME, PERIOD);

  // The state pass is skipped for the rest of the cycle, so the command pass must not run at all.
  EXPECT_EQ(root_handles_before, root_->handle_phase_calls());
  EXPECT_EQ(mid_handles_before, mid_->handle_phase_calls());
  EXPECT_EQ(leaf_handles_before, leaf_->handle_phase_calls());

  // And the next cycle recovers.
  Cycle(1);
  EXPECT_EQ(root_handles_before + 1, root_->handle_phase_calls());
}

// ---------------------------------------------------------------------------------------------
// 5. Mode lifecycle and observability.
// ---------------------------------------------------------------------------------------------

TEST_F(TestTwoPhaseExecution, disable_returns_to_the_native_path)
{
  BuildChain();
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  ActivateChain();
  Cycle(2);
  const auto native_before = leaf_->native_update_calls();

  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(false));
  EXPECT_FALSE(cm_->two_phase_execution());
  Cycle(2);
  EXPECT_EQ(native_before + 2, leaf_->native_update_calls());
}

TEST_F(TestTwoPhaseExecution, the_execution_generation_advances_on_each_mode_change)
{
  BuildChain();
  const auto initial = cm_->execution_generation();
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  const auto enabled = cm_->execution_generation();
  EXPECT_GT(enabled, initial);
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(false));
  EXPECT_GT(cm_->execution_generation(), enabled);
}

TEST_F(TestTwoPhaseExecution, rejections_can_be_asked_for_before_enabling)
{
  auto legacy = std::make_shared<LegacySourceController>();
  ASSERT_NE(nullptr, cm_->add_controller(legacy, kLegacy, "legacy_source"));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLegacy));
  root_ = MakeNode(kRoot, {"joint2/velocity"}, {}, {kLegacy});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));

  EXPECT_FALSE(cm_->two_phase_execution());
  const auto rejections = cm_->two_phase_rejected_controllers();
  EXPECT_FALSE(rejections.empty());
  bool mentions_legacy = false;
  for (const auto & rejection : rejections)
  {
    mentions_legacy |= rejection.reason.find(kLegacy) != std::string::npos;
  }
  EXPECT_TRUE(mentions_legacy);
}

// ---------------------------------------------------------------------------------------------
// 6. Admission: the configurations that must be refused.
// ---------------------------------------------------------------------------------------------

TEST_F(TestTwoPhaseExecution, a_cross_mode_edge_is_refused)
{
  auto legacy = std::make_shared<LegacySourceController>();
  ASSERT_NE(nullptr, cm_->add_controller(legacy, kLegacy, "legacy_source"));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLegacy));
  root_ = MakeNode(kRoot, {"joint2/velocity"}, {}, {kLegacy});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true));
  EXPECT_FALSE(cm_->two_phase_execution()) << "a refused request must not flip the mode";

  bool mentions_crossing = false;
  for (const auto & rejection : cm_->two_phase_rejected_controllers())
  {
    mentions_crossing |= rejection.reason.find("two-phase/native boundary") != std::string::npos;
  }
  EXPECT_TRUE(mentions_crossing);
}

TEST_F(TestTwoPhaseExecution, an_order_that_inverts_an_edge_is_refused)
{
  // A child that claims NO command interface is sorted AHEAD of its parent by upstream
  // `controller_sorting()`. The passes do not re-sort, so both would walk the edge the same (wrong)
  // way and the parent would silently compute from the previous cycle.
  auto source = MakeNode(kLeaf, {}, {}, {});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
  root_ = MakeNode(kRoot, {"joint2/velocity"}, {}, {kLeaf});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));

  ASSERT_EQ(std::vector<std::string>({kLeaf, kRoot}), ControllerOrder())
    << "this test needs the inverted order upstream produces for a source-only child";

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true));
  EXPECT_FALSE(cm_->two_phase_execution());
  bool mentions_order = false;
  for (const auto & rejection : cm_->two_phase_rejected_controllers())
  {
    mentions_order |= rejection.reason.find("ordered AFTER its child") != std::string::npos;
  }
  EXPECT_TRUE(mentions_order);
}

TEST_F(TestTwoPhaseExecution, one_object_under_two_names_is_refused)
{
  auto shared = MakeNode(kSolo, {}, {}, {});
  ASSERT_NE(nullptr, cm_->add_controller(shared, "tp_solo_dup", kType))
    << "upstream accepts a second name for the same instance; the mode must catch it";

  // No configure() is needed for this verdict: it is purely a property of the controller LIST.
  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true));
  EXPECT_FALSE(cm_->two_phase_execution());
  EXPECT_EQ(2u, cm_->two_phase_rejected_controllers().size())
    << "both names of the shared object must be reported";
}

TEST_F(TestTwoPhaseExecution, a_rate_that_does_not_divide_the_manager_rate_is_refused)
{
  ASSERT_NE(0u, cm_->get_update_rate() % 3u)
    << "this test needs a rate the manager cannot hit exactly";
  ASSERT_GT(cm_->get_update_rate(), 3u);

  solo_ = MakeNode(kSolo, {}, {}, {});
  solo_->get_node()->set_parameter({"update_rate", 3});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kSolo));

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true));
  EXPECT_FALSE(cm_->two_phase_execution());
  bool mentions_rate = false;
  for (const auto & rejection : cm_->two_phase_rejected_controllers())
  {
    mentions_rate |= rejection.reason.find("exact divisor") != std::string::npos;
  }
  EXPECT_TRUE(mentions_rate);
}

TEST_F(TestTwoPhaseExecution, a_lower_rate_member_joins_its_own_bucket)
{
  const auto half_rate = cm_->get_update_rate() / 2;
  ASSERT_GE(half_rate, 1u);

  solo_ = MakeNode(kSolo, {}, {}, {});
  solo_->get_node()->set_parameter({"update_rate", static_cast<int>(half_rate)});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kSolo));
  ASSERT_EQ(static_cast<unsigned int>(half_rate), solo_->get_update_rate());

  ASSERT_TRUE(cm_->two_phase_rejected_controllers().empty());
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  SwitchNow({kSolo}, {});

  const auto before_updates = solo_->update_phase_calls();
  const auto before_handles = solo_->handle_phase_calls();
  Cycle(6);
  // factor 2: due on every second cycle, and its period is the BUCKET's, not the manager's. The
  // deltas are exact whatever phase the manager's cycle counter was in.
  EXPECT_EQ(before_updates + 3, solo_->update_phase_calls());
  EXPECT_EQ(before_handles + 3, solo_->handle_phase_calls());
  EXPECT_EQ(20000000, solo_->last_period_ns());
}

TEST_F(TestTwoPhaseExecution, a_cross_bucket_edge_is_refused_by_the_default_budget_and_quantified)
{
  const auto half_rate = cm_->get_update_rate() / 2;
  ASSERT_GE(half_rate, 1u);

  // mid runs at the manager's rate (factor 1) and claims the leaf's interfaces, while the leaf runs
  // at half rate (factor 2): state lag 2, reference lag 0, so the default budget of 0 refuses it.
  BuildChain(half_rate);
  ASSERT_EQ(static_cast<unsigned int>(half_rate), leaf_->get_update_rate());

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true));
  EXPECT_FALSE(cm_->two_phase_execution());
  bool quantified = false;
  for (const auto & rejection : cm_->two_phase_rejected_controllers())
  {
    quantified |= rejection.reason.find("worst 2 cycle(s)") != std::string::npos;
  }
  EXPECT_TRUE(quantified) << "the refusal must quantify the staleness it refuses";

  // The SAME configuration is admissible once the budget covers the lag it just reported.
  EXPECT_EQ(Return::OK, cm_->set_two_phase_execution(true, 2));
  EXPECT_TRUE(cm_->two_phase_execution());
  EXPECT_EQ(2u, cm_->two_phase_max_lag_cycles());
}

TEST_F(TestTwoPhaseExecution, a_deactivated_member_is_skipped_while_others_continue)
{
  // Two INDEPENDENT members, because upstream refuses to deactivate a chained child whose parent is
  // still active -- so "member deactivated, parent running" is not a state the manager can reach.
  first_ = MakeNode("tp_a", {"joint2/velocity"}, {"joint2/position"}, {});
  second_ = MakeNode("tp_b", {"joint3/velocity"}, {"joint3/position"}, {});
  ASSERT_EQ(Return::OK, cm_->configure_controller("tp_a"));
  ASSERT_EQ(Return::OK, cm_->configure_controller("tp_b"));
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  SwitchNow({"tp_a"}, {});
  SwitchNow({"tp_b"}, {});
  Cycle(2);

  // Snapshot AFTER the switch: the switch itself pumps the control loop, and `tp_b` runs in it.
  SwitchNow({}, {"tp_a"});
  EXPECT_FALSE(ControllerIsActive(*first_));
  const auto first_before = first_->update_phase_calls();
  const auto second_before = second_->update_phase_calls();
  Cycle(2);
  EXPECT_EQ(first_before, first_->update_phase_calls()) << "an inactive member must not run";
  EXPECT_EQ(second_before + 2, second_->update_phase_calls());
}

// ---------------------------------------------------------------------------------------------
// 7. Installation while the loop is running.
// ---------------------------------------------------------------------------------------------

TEST_F(TestTwoPhaseExecution, installing_while_a_cycle_is_in_flight_is_refused)
{
  BuildChain();
  // Install the path while the loop is idle (the supported configuration point) ...
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  ActivateChain();
  Cycle(1);

  // ... then hold the control loop INSIDE a cycle and try to re-install from this thread.
  leaf_->hold_update_phase(true);
  std::thread loop(
    [this]()
    {
      cm_->read(TIME, PERIOD);
      cm_->update(TIME, PERIOD);
      cm_->write(TIME, PERIOD);
    });
  for (int i = 0; i < 5000 && !leaf_->update_phase_entered(); ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(leaf_->update_phase_entered()) << "the control loop never entered the state pass";
  EXPECT_TRUE(cm_->control_loop_busy());

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true))
    << "installing a path while a cycle is in flight must be refused";

  leaf_->hold_update_phase(false);
  loop.join();
  EXPECT_FALSE(cm_->control_loop_busy());

  // Removing the path is still allowed while busy: a running cycle holds its own snapshot.
  EXPECT_EQ(Return::OK, cm_->set_two_phase_execution(false));
}

// ---------------------------------------------------------------------------------------------
// 8. A branching tree.
// ---------------------------------------------------------------------------------------------

TEST_F(TestTwoPhaseExecution, a_branching_tree_propagates_within_one_cycle)
{
  left_ = MakeNode(kLeft, {"joint2/velocity"}, {"joint2/position"}, {});
  right_ = MakeNode(kRight, {"joint3/velocity"}, {"joint3/position"}, {});
  root_ = MakeNode(kRoot, {}, {}, {kLeft, kRight});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeft));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRight));
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true));
  SwitchNow({kLeft}, {});
  SwitchNow({kRight}, {});
  SwitchNow({kRoot}, {});
  Cycle(4);

  left_->set_hardware_state(1.0);
  right_->set_hardware_state(1.0);
  cm_->read(TIME, PERIOD);
  EXPECT_EQ(Return::OK, cm_->update(TIME, PERIOD));
  cm_->write(TIME, PERIOD);

  // Both leaves publish 1.0 in the backward pass, so the root aggregates a SAME-CYCLE mean of 1.0:
  // estimate = 0.5 * 0 + 0.5 * 1.0 = 0.5. A single parents-first pass would leave it at 0.
  EXPECT_DOUBLE_EQ(1.0, left_->estimate());
  EXPECT_DOUBLE_EQ(1.0, right_->estimate());
  EXPECT_DOUBLE_EQ(0.5, root_->estimate());
}

// ---------------------------------------------------------------------------------------------
// 9. Rate buckets: the exact lag reported by the admission, measured in the REAL manager.
//
// `doc/CROSS_RATE_BOUND.md` derives the worst-case staleness of a two-phase edge from the two rate
// buckets; the values are ATTAINED, not loose bounds. The tests below read the age of the value each
// end CONSUMED (the controller's cycle-stamp instrument), so they compare the derivation against the
// real ControllerManager rather than against a model of it. That comparison is the prerequisite for
// admitting cross-bucket edges at all.
// ---------------------------------------------------------------------------------------------

/// The age, in manager cycles, of the value each end of one edge consumed, worst case over a window.
struct MeasuredLag
{
  unsigned int state = 0;      ///< child published -> parent ingested
  unsigned int reference = 0;  ///< parent wrote     -> child consumed
};

TEST_F(TestTwoPhaseExecution, the_exact_lag_of_every_edge_is_reported_before_enabling)
{
  // parent rate 50 (factor 2), child rate 100 (factor 1): the parent is slower.
  leaf_ = MakeNode(kLeaf, {"joint2/velocity"}, {"joint2/position"}, {});
  leaf_->get_node()->set_parameter({"update_rate", 100});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
  root_ = MakeNode(kRoot, {}, {}, {kLeaf});
  root_->get_node()->set_parameter({"update_rate", 50});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));

  // Reported whether or not the mode is on, so a configuration can be judged before enabling.
  EXPECT_FALSE(cm_->two_phase_execution());
  const auto lags = cm_->two_phase_edge_lags();
  ASSERT_EQ(1u, lags.size());
  EXPECT_EQ(kRoot, lags[0].parent);
  EXPECT_EQ(kLeaf, lags[0].child);
  EXPECT_EQ(2u, lags[0].parent_factor);
  EXPECT_EQ(1u, lags[0].child_factor);
  // f_P = 2, f_C = 1, g = 1: state f_C - g = 0 (the faster child is served same-cycle),
  //                          reference f_P = 2 (the child's bucket runs first in the command pass).
  EXPECT_EQ(0u, lags[0].state_lag_cycles);
  EXPECT_EQ(2u, lags[0].reference_lag_cycles);
  EXPECT_EQ(2u, lags[0].worst_lag_cycles);
}

TEST_F(TestTwoPhaseExecution, the_default_budget_reproduces_the_strict_same_bucket_rule)
{
  leaf_ = MakeNode(kLeaf, {"joint2/velocity"}, {"joint2/position"}, {});
  leaf_->get_node()->set_parameter({"update_rate", 100});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
  root_ = MakeNode(kRoot, {}, {}, {kLeaf});
  root_->get_node()->set_parameter({"update_rate", 50});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));

  EXPECT_EQ(0u, cm_->two_phase_max_lag_cycles()) << "the default budget is zero";
  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true))
    << "factor 2 vs factor 1 is not same-cycle in both directions, so the default budget refuses it";
  EXPECT_FALSE(cm_->two_phase_execution());

  // The refusal must state the EXACT lag it computed, not just "different buckets".
  bool mentions_lag = false;
  for (const auto & rejection : cm_->two_phase_rejected_controllers())
  {
    mentions_lag |= rejection.reason.find("worst 2 cycle(s)") != std::string::npos;
  }
  EXPECT_TRUE(mentions_lag) << "the rejection must quantify the staleness it refuses";
}

TEST_F(TestTwoPhaseExecution, raising_the_budget_admits_the_edge_it_quantifies)
{
  leaf_ = MakeNode(kLeaf, {"joint2/velocity"}, {"joint2/position"}, {});
  leaf_->get_node()->set_parameter({"update_rate", 100});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
  root_ = MakeNode(kRoot, {}, {}, {kLeaf});
  root_->get_node()->set_parameter({"update_rate", 50});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));

  // Budget 1 is still short of the computed worst case of 2 ...
  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true, 1));
  // ... and 2 admits it.
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true, 2));
  EXPECT_TRUE(cm_->two_phase_execution());
  EXPECT_EQ(2u, cm_->two_phase_max_lag_cycles());
}

TEST_F(TestTwoPhaseExecution, the_measured_lag_matches_the_declared_bound_for_a_slower_parent)
{
  // f_P = 2 (parent rate 50), f_C = 1 (child rate 100).
  constexpr unsigned int kBudget = 2;
  leaf_ = MakeNode(kLeaf, {"joint2/velocity"}, {"joint2/position"}, {});
  leaf_->get_node()->set_parameter({"update_rate", 100});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
  root_ = MakeNode(kRoot, {}, {}, {kLeaf});
  root_->get_node()->set_parameter({"update_rate", 50});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));

  leaf_->set_cycle_stamp_mode(true);
  root_->set_cycle_stamp_mode(true);
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true, kBudget));
  SwitchNow({kLeaf}, {});
  SwitchNow({kRoot}, {});

  unsigned int max_state = 0;
  unsigned int max_reference = 0;
  // One GLOBAL stamp per cycle, given to every node: a node only advances on the cycles its own
  // bucket is due, so a per-node counter is not a common time base.
  for (std::int64_t stamp = 1; stamp <= 24; ++stamp)
  {
    leaf_->set_cycle_stamp(stamp);
    root_->set_cycle_stamp(stamp);
    const auto root_ingests_before = root_->update_phase_calls();
    const auto leaf_consumes_before = leaf_->handle_phase_calls();
    cm_->read(TIME, PERIOD);
    ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
    cm_->write(TIME, PERIOD);
    if (stamp <= 4) {continue;}  // warm up: until both producers have stamped
    // The age of a consumed value is only defined on the cycles the CONSUMER actually ran; sampling
    // on an idle cycle measures "time since it last ran" instead, which is not staleness.
    if (root_->update_phase_calls() != root_ingests_before)
    {
      max_state = std::max(
        max_state,
        static_cast<unsigned int>(
          stamp - static_cast<std::int64_t>(root_->last_child_estimate_seen())));
    }
    if (leaf_->handle_phase_calls() != leaf_consumes_before)
    {
      max_reference = std::max(
        max_reference,
        static_cast<unsigned int>(
          stamp - static_cast<std::int64_t>(leaf_->last_target_seen())));
    }
  }

  // The declared bound is what the admission computed; the measurement is what the manager did.
  EXPECT_LE(max_state, kBudget);
  EXPECT_LE(max_reference, kBudget);
  // ATTAINED, not merely bounded: the faster child is served same-cycle, and the slower parent's
  // reference reaches it two cycles later on the cycles where the child's bucket runs first.
  EXPECT_EQ(0u, max_state);
  EXPECT_EQ(2u, max_reference);
}

TEST_F(TestTwoPhaseExecution, the_measured_lag_matches_the_declared_bound_for_a_slower_child)
{
  // The mirror configuration: f_P = 1 (parent every cycle), f_C = 2 (child at half rate).
  constexpr unsigned int kBudget = 2;
  leaf_ = MakeNode(kLeaf, {"joint2/velocity"}, {"joint2/position"}, {});
  leaf_->get_node()->set_parameter({"update_rate", 50});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
  root_ = MakeNode(kRoot, {}, {}, {kLeaf});
  root_->get_node()->set_parameter({"update_rate", 100});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));

  const auto lags = cm_->two_phase_edge_lags();
  ASSERT_EQ(1u, lags.size());
  EXPECT_EQ(2u, lags[0].state_lag_cycles) << "a slower child's state is one child period old";
  EXPECT_EQ(0u, lags[0].reference_lag_cycles) << "the parent writes every cycle, before the child";
  EXPECT_EQ(2u, lags[0].worst_lag_cycles);

  leaf_->set_cycle_stamp_mode(true);
  root_->set_cycle_stamp_mode(true);
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true, kBudget));
  SwitchNow({kLeaf}, {});
  SwitchNow({kRoot}, {});

  unsigned int max_state = 0;
  unsigned int max_reference = 0;
  // One GLOBAL stamp per cycle, given to every node: a node only advances on the cycles its own
  // bucket is due, so a per-node counter is not a common time base.
  for (std::int64_t stamp = 1; stamp <= 24; ++stamp)
  {
    leaf_->set_cycle_stamp(stamp);
    root_->set_cycle_stamp(stamp);
    const auto root_ingests_before = root_->update_phase_calls();
    const auto leaf_consumes_before = leaf_->handle_phase_calls();
    cm_->read(TIME, PERIOD);
    ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
    cm_->write(TIME, PERIOD);
    if (stamp <= 4) {continue;}  // warm up: until both producers have stamped
    // The age of a consumed value is only defined on the cycles the CONSUMER actually ran; sampling
    // on an idle cycle measures "time since it last ran" instead, which is not staleness.
    if (root_->update_phase_calls() != root_ingests_before)
    {
      max_state = std::max(
        max_state,
        static_cast<unsigned int>(
          stamp - static_cast<std::int64_t>(root_->last_child_estimate_seen())));
    }
    if (leaf_->handle_phase_calls() != leaf_consumes_before)
    {
      max_reference = std::max(
        max_reference,
        static_cast<unsigned int>(
          stamp - static_cast<std::int64_t>(leaf_->last_target_seen())));
    }
  }

  EXPECT_LE(max_state, kBudget);
  EXPECT_LE(max_reference, kBudget);
  EXPECT_EQ(2u, max_state);
  EXPECT_EQ(0u, max_reference);
}

TEST_F(TestTwoPhaseExecution, a_non_harmonic_edge_needs_its_exact_worst_lag)
{
  // f_P = 2 (parent rate 50), f_C = 5 (child rate 20), g = 1:
  //   state = f_C = 5 (the parent bucket runs first, so even a coincident cycle costs 5),
  //   reference = f_P - g = 1.
  leaf_ = MakeNode(kLeaf, {"joint2/velocity"}, {"joint2/position"}, {});
  leaf_->get_node()->set_parameter({"update_rate", 20});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kLeaf));
  root_ = MakeNode(kRoot, {}, {}, {kLeaf});
  root_->get_node()->set_parameter({"update_rate", 50});
  ASSERT_EQ(Return::OK, cm_->configure_controller(kRoot));

  const auto lags = cm_->two_phase_edge_lags();
  ASSERT_EQ(1u, lags.size());
  EXPECT_EQ(5u, lags[0].state_lag_cycles);
  EXPECT_EQ(1u, lags[0].reference_lag_cycles);
  EXPECT_EQ(5u, lags[0].worst_lag_cycles);

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true, 4))
    << "a budget below the computed worst case must be refused";
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true, 5));
  EXPECT_EQ(5u, cm_->two_phase_max_lag_cycles());
}

}  // namespace
