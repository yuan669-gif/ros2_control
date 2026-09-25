// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "test_composite_controller/test_composite_controller.hpp"
#include "test_composite_library/generic_composite_controller.hpp"
#include "test_composite_library/typed_fork_composite_controller.hpp"
#include "test_staged_controller/test_staged_controller.hpp"

// GCC cannot see that this malloc-based replacement pair is consistent and reports
// -Wmismatched-new-delete from the inlined call sites. Disable it for this translation unit.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif

namespace
{
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using Return = controller_interface::return_type;
using TestStagedController = test_staged_controller::TestStagedController;
using TestCompositeController = test_composite_controller::TestCompositeController;
using GenericCompositeController = test_composite_library::GenericCompositeController;
using TypedForkCompositeController = test_composite_library::TypedForkCompositeController;
using CompositeNodeSpec = test_composite_library::CompositeNodeSpec;

const auto kTime = rclcpp::Time(0);
const auto kPeriod = rclcpp::Duration::from_seconds(0.01);
const auto kStrict = controller_manager_msgs::srv::SwitchController::Request::STRICT;

constexpr char kRoot[] = "cmp_root";
constexpr char kModule[] = "cmp_module";
constexpr char kLeaf[] = "cmp_leaf";
constexpr char kComposite[] = "cmp_composite";
constexpr char kChainedType[] = "chainable";
constexpr char kCompositeType[] = "composite";
constexpr double kStateOffset = 3.0;

InterfaceConfiguration individual(const std::vector<std::string> & names)
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;
  cfg.names = names;
  return cfg;
}

double ReferenceFor(int cycle) {return 1000.0 + 10.0 * static_cast<double>(cycle);}

/// Analytic same-algorithm result: reference - 7 * (hardware state + offset).
double Expected(double reference) {return reference - 7.0 * kStateOffset;}

// ---------------------------------------------------------------------------------------------
// Dynamic-allocation counting, enabled only around the measured update() call.
//
// `g_count_only_current_thread` narrows the count to `g_counted_thread`, so a probe can measure
// the control-loop thread while another thread (switch_controller, a driver) allocates freely.
// ---------------------------------------------------------------------------------------------
std::atomic<bool> g_count_allocations{false};
std::atomic<std::size_t> g_allocation_count{0};
std::atomic<bool> g_count_only_current_thread{false};
std::thread::id g_counted_thread;

/// Restores the default (count every thread) even when a test aborts early.
struct AllocationCounterGuard
{
  AllocationCounterGuard() = default;
  ~AllocationCounterGuard()
  {
    g_count_allocations.store(false, std::memory_order_relaxed);
    g_count_only_current_thread.store(false, std::memory_order_relaxed);
  }
  AllocationCounterGuard(const AllocationCounterGuard &) = delete;
  AllocationCounterGuard & operator=(const AllocationCounterGuard &) = delete;
};

void CountAllocation() noexcept
{
  if (!g_count_allocations.load(std::memory_order_relaxed)) {return;}
  if (
    g_count_only_current_thread.load(std::memory_order_relaxed) &&
    std::this_thread::get_id() != g_counted_thread)
  {
    return;
  }
  g_allocation_count.fetch_add(1, std::memory_order_relaxed);
}
}  // namespace

// GCC cannot see that this malloc-based replacement pair is consistent and emits
// -Wmismatched-new-delete on the sized delete. Silence the false positive locally.
void * operator new(std::size_t size)
{
  CountAllocation();
  void * pointer = std::malloc(size == 0 ? 1 : size);
  if (pointer == nullptr) {throw std::bad_alloc();}
  return pointer;
}

void * operator new[](std::size_t size) {return ::operator new(size);}

void * operator new(std::size_t size, const std::nothrow_t &) noexcept
{
  CountAllocation();
  return std::malloc(size == 0 ? 1 : size);
}

void * operator new[](std::size_t size, const std::nothrow_t & tag) noexcept
{
  return ::operator new(size, tag);
}

void * operator new(std::size_t size, std::align_val_t alignment)
{
  CountAllocation();
  void * pointer = nullptr;
  const auto align = static_cast<std::size_t>(alignment);
  const auto realign = align < sizeof(void *) ? sizeof(void *) : align;
  if (posix_memalign(&pointer, realign, size == 0 ? 1 : size))
  {
    throw std::bad_alloc();
  }
  return pointer;
}

void * operator new[](std::size_t size, std::align_val_t alignment)
{
  return ::operator new(size, alignment);
}

void operator delete(void * pointer) noexcept {std::free(pointer);}
void operator delete[](void * pointer) noexcept {std::free(pointer);}
void operator delete(void * pointer, std::size_t) noexcept {std::free(pointer);}
void operator delete[](void * pointer, std::size_t) noexcept {std::free(pointer);}
void operator delete(void * pointer, const std::nothrow_t &) noexcept {std::free(pointer);}
void operator delete[](void * pointer, const std::nothrow_t &) noexcept {std::free(pointer);}
void operator delete(void * pointer, std::align_val_t) noexcept {std::free(pointer);}
void operator delete[](void * pointer, std::align_val_t) noexcept {std::free(pointer);}
void operator delete(void * pointer, std::size_t, std::align_val_t) noexcept {std::free(pointer);}
void operator delete[](void * pointer, std::size_t, std::align_val_t) noexcept {std::free(pointer);}

namespace
{

enum class Mode { chained, composite, staged };

struct Metrics
{
  std::vector<double> commands;
  std::vector<double> update_us;
  std::size_t allocations = 0;
  int update_errors = 0;
  double last_update_us = 0.0;
  std::string label;
};

void SwitchNow(
  const std::shared_ptr<controller_manager::ControllerManager> & cm,
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  auto future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm.get(), start,
    stop, kStrict, true, rclcpp::Duration(0, 0));
  for (int i = 0;
       i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cm->update(kTime, kPeriod);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    throw std::runtime_error("switch did not complete");
  }
  if (future.get() != Return::OK) {throw std::runtime_error("switch failed");}
}

struct Group
{
  std::shared_ptr<controller_manager::ControllerManager> cm;
  std::shared_ptr<TestStagedController> root, module, leaf, second_leaf;
  std::vector<std::shared_ptr<TestStagedController>> extra_leaves;
  std::shared_ptr<TestCompositeController> composite;
  std::shared_ptr<GenericCompositeController> generic;
};

std::shared_ptr<controller_manager::ControllerManager> MakeManager(
  const std::shared_ptr<rclcpp::Executor> & executor, const std::string & name)
{
  auto resource_manager = std::make_unique<hardware_interface::ResourceManager>(
    ros2_control_test_assets::minimal_robot_urdf, true, true);
  return std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), executor, name, "");
}

void ConfigureController(
  const std::shared_ptr<controller_manager::ControllerManager> & cm, const std::string & name)
{
  if (cm->configure_controller(name) != Return::OK)
  {
    throw std::runtime_error("configure failed: " + name);
  }
}

/// Native Humble chaining: each node reads the raw hardware snapshot itself and derives its own
/// local estimate; execution is a single parent-to-child pass with no group commit.
Group SetupChained(const std::shared_ptr<rclcpp::Executor> & executor, const std::string & name)
{
  Group group;
  group.cm = MakeManager(executor, name);
  group.root = std::make_shared<TestStagedController>();
  group.module = std::make_shared<TestStagedController>();
  group.leaf = std::make_shared<TestStagedController>();

  const std::vector<std::string> hardware_state{"joint2/position"};

  group.root->set_native_mode(true, 4.0, kStateOffset);
  group.root->set_reference_interface_names({"command"});
  group.root->set_command_interface_configuration(individual({std::string(kModule) + "/target"}));
  group.root->set_state_interface_configuration(individual(hardware_state));

  group.module->set_native_mode(true, 2.0, kStateOffset);
  group.module->set_reference_interface_names({"target"});
  group.module->set_command_interface_configuration(individual({std::string(kLeaf) + "/target"}));
  group.module->set_state_interface_configuration(individual(hardware_state));

  group.leaf->set_native_mode(true, 1.0, kStateOffset);
  group.leaf->set_actuator_ports({"joint2/velocity"});
  group.leaf->set_reference_interface_names({"target"});
  group.leaf->set_command_interface_configuration(individual({"joint2/velocity"}));
  group.leaf->set_state_interface_configuration(individual(hardware_state));

  group.cm->add_controller(group.root, kRoot, kChainedType);
  group.cm->add_controller(group.module, kModule, kChainedType);
  group.cm->add_controller(group.leaf, kLeaf, kChainedType);
  ConfigureController(group.cm, kLeaf);
  if (group.cm->configure_controller(kModule) != Return::OK)
  {
    throw std::runtime_error("configure module");
  }
  ConfigureController(group.cm, kRoot);
  group.leaf->set_chained_mode(true);
  group.module->set_chained_mode(true);
  SwitchNow(group.cm, {kLeaf}, {});
  SwitchNow(group.cm, {kModule}, {});
  SwitchNow(group.cm, {kRoot}, {});
  return group;
}

/// One handwritten composite plugin with the same internal two-phase algorithm.
Group SetupComposite(const std::shared_ptr<rclcpp::Executor> & executor, const std::string & name)
{
  Group group;
  group.cm = MakeManager(executor, name);
  group.composite = std::make_shared<TestCompositeController>();
  group.composite->set_command_interface_configuration(individual({"joint2/velocity"}));
  group.composite->set_state_interface_configuration(individual({"joint2/position"}));
  group.composite->set_state_offset(kStateOffset);
  group.cm->add_controller(group.composite, kComposite, kCompositeType);
  if (group.cm->configure_controller(kComposite) != Return::OK)
  {
    throw std::runtime_error("configure composite");
  }
  SwitchNow(group.cm, {kComposite}, {});
  return group;
}

/// Proposed staged execution group: three controllers, postorder state, preorder command, commit.
Group SetupStaged(const std::shared_ptr<rclcpp::Executor> & executor, const std::string & name)
{
  Group group;
  group.cm = MakeManager(executor, name);
  group.root = std::make_shared<TestStagedController>();
  group.module = std::make_shared<TestStagedController>();
  group.leaf = std::make_shared<TestStagedController>();

  group.root->set_reference_interface_names({"command"});
  group.root->set_command_interface_configuration(individual({std::string(kModule) + "/target"}));
  group.root->set_state_interface_configuration(individual({}));

  group.module->set_reference_interface_names({"target"});
  group.module->set_command_interface_configuration(individual({std::string(kLeaf) + "/target"}));
  group.module->set_state_interface_configuration(individual({}));

  group.leaf->set_actuator_ports({"joint2/velocity"});
  group.leaf->set_reference_interface_names({"target"});
  group.leaf->set_command_interface_configuration(individual({"joint2/velocity"}));
  group.leaf->set_state_interface_configuration(individual({"joint2/position"}));
  group.leaf->set_state_offset(kStateOffset);

  group.cm->add_controller(group.root, kRoot, kChainedType);
  group.cm->add_controller(group.module, kModule, kChainedType);
  group.cm->add_controller(group.leaf, kLeaf, kChainedType);
  ConfigureController(group.cm, kLeaf);
  if (group.cm->configure_controller(kModule) != Return::OK)
  {
    throw std::runtime_error("configure module");
  }
  ConfigureController(group.cm, kRoot);
  group.leaf->set_chained_mode(true);
  group.module->set_chained_mode(true);
  if (group.cm->set_staged_execution_group({kRoot, kModule, kLeaf}) != Return::OK)
  {
    throw std::runtime_error("install staged group");
  }
  SwitchNow(group.cm, {kLeaf}, {});
  SwitchNow(group.cm, {kModule}, {});
  SwitchNow(group.cm, {kRoot}, {});
  return group;
}

double ObservedCommand(const Group & group, Mode mode)
{
  if (mode == Mode::composite) {return group.composite->command_interface_value();}
  if (mode == Mode::chained) {return group.leaf->command_interface_value();}
  return group.leaf->committed_value();
}

Metrics RunGroup(
  Mode mode, const std::shared_ptr<rclcpp::Executor> & executor, int cycles, int fail_cycle = -1,
  const std::string & label = "")
{
  Group group;
  switch (mode)
  {
    case Mode::chained:
      group = SetupChained(executor, "cmp_chained_cm");
      break;
    case Mode::composite:
      group = SetupComposite(executor, "cmp_composite_cm");
      break;
    case Mode::staged:
      group = SetupStaged(executor, "cmp_staged_cm");
      break;
  }

  Metrics metrics;
  metrics.label = label;
  for (int cycle = 1; cycle <= cycles; ++cycle)
  {
    const double reference = ReferenceFor(cycle);
    if (mode == Mode::composite)
    {
      group.composite->set_external_reference(reference);
    }
    else
    {
      group.root->set_external_reference(reference);
    }
    const bool failing = (cycle == fail_cycle);
    if (failing)
    {
      if (mode == Mode::composite)
      {
        group.composite->set_fail_command(true);
      }
      else
      {
        group.module->set_fail_command(true);
      }
    }

    group.cm->read(kTime, kPeriod);
    g_allocation_count.store(0, std::memory_order_relaxed);
    g_count_allocations.store(true, std::memory_order_relaxed);
    const auto start = std::chrono::steady_clock::now();
    const auto result = group.cm->update(kTime, kPeriod);
    const auto finish = std::chrono::steady_clock::now();
    g_count_allocations.store(false, std::memory_order_relaxed);
    metrics.allocations += g_allocation_count.load(std::memory_order_relaxed);
    group.cm->write(kTime, kPeriod);

    if (failing)
    {
      if (mode == Mode::composite)
      {
        group.composite->set_fail_command(false);
      }
      else
      {
        group.module->set_fail_command(false);
      }
    }
    if (result != Return::OK) {++metrics.update_errors;}
    metrics.commands.push_back(ObservedCommand(group, mode));
    metrics.last_update_us =
      std::chrono::duration<double, std::micro>(finish - start).count();
    metrics.update_us.push_back(metrics.last_update_us);
  }

  if (group.cm->staged_execution_group()) {group.cm->clear_staged_execution_group();}
  return metrics;
}

class HierarchyFairComparison : public ::testing::Test
{
public:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
  static void TearDownTestCase() {rclcpp::shutdown();}

  void SetUp() override {executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();}

  std::shared_ptr<rclcpp::Executor> executor_;
};

TEST_F(HierarchyFairComparison, same_algorithm_outputs_match_across_implementations)
{
  const int cycles = 20;
  const auto chained = RunGroup(Mode::chained, executor_, cycles, -1, "native chaining");
  const auto composite = RunGroup(Mode::composite, executor_, cycles, -1, "composite plugin");
  const auto staged = RunGroup(Mode::staged, executor_, cycles, -1, "staged group");

  ASSERT_EQ(static_cast<std::size_t>(cycles), chained.commands.size());
  ASSERT_EQ(chained.commands.size(), composite.commands.size());
  ASSERT_EQ(chained.commands.size(), staged.commands.size());

  for (int cycle = 1; cycle <= cycles; ++cycle)
  {
    const auto index = static_cast<std::size_t>(cycle - 1);
    const double expected = Expected(ReferenceFor(cycle));
    EXPECT_DOUBLE_EQ(expected, chained.commands[index]) << "native chaining, cycle " << cycle;
    EXPECT_DOUBLE_EQ(expected, composite.commands[index]) << "composite, cycle " << cycle;
    EXPECT_DOUBLE_EQ(expected, staged.commands[index]) << "staged, cycle " << cycle;
    EXPECT_DOUBLE_EQ(chained.commands[index], composite.commands[index]);
    EXPECT_DOUBLE_EQ(chained.commands[index], staged.commands[index]);
  }

  // No failures injected, so every cycle must succeed in every implementation.
  EXPECT_EQ(0, chained.update_errors);
  EXPECT_EQ(0, composite.update_errors);
  EXPECT_EQ(0, staged.update_errors);
}

TEST_F(HierarchyFairComparison, reported_overhead_and_allocations)
{
  const int cycles = 200;
  const auto chained = RunGroup(Mode::chained, executor_, cycles, -1, "native chaining");
  const auto composite = RunGroup(Mode::composite, executor_, cycles, -1, "composite plugin");
  const auto staged = RunGroup(Mode::staged, executor_, cycles, -1, "staged group");

  // These counts measure a whole `ControllerManager::update()` call, including Humble's existing
  // per-cycle `for (auto loaded_controller : rt_controller_list)` copy of every ControllerSpec.
  // They are reported, not asserted: the manager-level cost is common to all three designs.
  RecordProperty("chained_allocations", static_cast<int>(chained.allocations / cycles));
  RecordProperty("composite_allocations", static_cast<int>(composite.allocations / cycles));
  RecordProperty("staged_allocations", static_cast<int>(staged.allocations / cycles));
  RecordProperty("chained_last_update_us", static_cast<int>(chained.last_update_us));
  RecordProperty("composite_last_update_us", static_cast<int>(composite.last_update_us));
  RecordProperty("staged_last_update_us", static_cast<int>(staged.last_update_us));
  RecordProperty("chained_update_errors", chained.update_errors);
  RecordProperty("composite_update_errors", composite.update_errors);
  RecordProperty("staged_update_errors", staged.update_errors);

  // Isolate the execution group's own real-time path by calling run() directly, so the manager's
  // pre-existing ControllerSpec copies are excluded. Topology, bindings and storage were all
  // resolved during configuration, so this path must not allocate.
  auto group = SetupStaged(executor_, "cmp_group_alloc_cm");
  group.root->set_record_diagnostics(false);
  group.module->set_record_diagnostics(false);
  group.leaf->set_record_diagnostics(false);
  group.root->set_external_reference(1.0);
  for (int i = 0; i < 5; ++i)
  {
    group.cm->staged_execution_group()->run(kTime, kPeriod);
  }
  g_allocation_count.store(0, std::memory_order_relaxed);
  g_count_allocations.store(true, std::memory_order_relaxed);
  for (int i = 0; i < 100; ++i)
  {
    const auto result = group.cm->staged_execution_group()->run(kTime, kPeriod);
    ASSERT_EQ(controller_manager::StagedStatus::committed, result.status);
  }
  g_count_allocations.store(false, std::memory_order_relaxed);
  const auto group_allocations = g_allocation_count.load(std::memory_order_relaxed);

  // Control: the same measurement window with no group call, to expose background-thread noise.
  g_allocation_count.store(0, std::memory_order_relaxed);
  g_count_allocations.store(true, std::memory_order_relaxed);
  for (int i = 0; i < 100; ++i)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  g_count_allocations.store(false, std::memory_order_relaxed);
  const auto control_allocations = g_allocation_count.load(std::memory_order_relaxed);

  std::cout << "[comparison] allocations per update(): chained="
            << chained.allocations / static_cast<std::size_t>(cycles)
            << " composite=" << composite.allocations / static_cast<std::size_t>(cycles)
            << " staged=" << staged.allocations / static_cast<std::size_t>(cycles) << '\n';
  for (const auto * metrics : {&chained, &composite, &staged})
  {
    auto samples = metrics->update_us;
    std::sort(samples.begin(), samples.end());
    std::cout << "[comparison] update() us over " << cycles << " cycles [" << metrics->label
              << "]: min=" << samples.front() << " median=" << samples[samples.size() / 2]
              << " max=" << samples.back() << '\n';
  }
  std::cout << "[comparison] StagedExecutionGroup::run allocations per 100 calls="
            << group_allocations << " (control=" << control_allocations << ")\n";

  RecordProperty(
    "staged_group_run_allocations_per_100_cycles", static_cast<int>(group_allocations));
  RecordProperty("control_allocations_per_100_iterations", static_cast<int>(control_allocations));
  EXPECT_EQ(0u, group_allocations) << "StagedExecutionGroup::run must not allocate";
  group.cm->clear_staged_execution_group();
}

// ---------------------------------------------------------------------------------------------
// Fork topology (the POV chassis shape): root -> {leaf_a, leaf_b, ...}.
//
// Adding a leaf to this topology is exactly one more `LeafSpec` entry below: both the staged
// group and native Humble chaining derive the new reference edge from the root's claimed
// interfaces, and the leaf controller class itself is reused unchanged.
// ---------------------------------------------------------------------------------------------
constexpr char kLeafB[] = "cmp_leaf_b";
constexpr char kLeafC[] = "cmp_leaf_c";
constexpr double kOffsetA = 3.0;
constexpr double kOffsetB = 5.0;
constexpr double kOffsetC = 7.0;

/// One fork leaf. An empty `state_interface` means the leaf's local estimate is only its offset.
struct LeafSpec
{
  std::string name;
  std::string state_interface;
  std::string command_interface;
  double offset = 0.0;
};

/// Baseline topology: one leaf per available TestSystem velocity command.
std::vector<LeafSpec> TwoLeafFork()
{
  return {
    {kLeaf, "joint2/position", "joint2/velocity", kOffsetA},
    {kLeafB, "joint3/position", "joint3/velocity", kOffsetB}};
}

/// The change under measurement: one extra LeafSpec entry, nothing else.
std::vector<LeafSpec> ThreeLeafFork()
{
  return {
    {kLeaf, "joint2/position", "joint2/velocity", kOffsetA},
    {kLeafB, "joint3/position", "joint3/velocity", kOffsetB},
    {kLeafC, "", "joint2/max_acceleration", kOffsetC}};
}

Group SetupChainedForkN(
  const std::shared_ptr<rclcpp::Executor> & executor, const std::string & name,
  const std::vector<LeafSpec> & leaves)
{
  Group group;
  group.cm = MakeManager(executor, name);
  group.root = std::make_shared<TestStagedController>();

  // Native chaining baseline: the root re-derives every child estimate from the same hardware
  // snapshot, so its local bias is the sum of the leaves' offsets.
  std::vector<std::string> claims, root_states;
  double root_bias = 0.0;
  for (const auto & leaf : leaves)
  {
    claims.push_back(leaf.name + "/target");
    if (!leaf.state_interface.empty()) {root_states.push_back(leaf.state_interface);}
    root_bias += leaf.offset;
  }
  group.root->set_native_mode(true, 2.0, root_bias);
  group.root->set_reference_interface_names({"command"});
  group.root->set_command_interface_configuration(individual(claims));
  group.root->set_state_interface_configuration(individual(root_states));
  group.cm->add_controller(group.root, kRoot, kChainedType);
  ConfigureController(group.cm, kRoot);

  std::vector<std::shared_ptr<TestStagedController>> leaf_controllers;
  for (const auto & leaf : leaves)
  {
    auto controller = std::make_shared<TestStagedController>();
    std::vector<std::string> states;
    if (!leaf.state_interface.empty()) {states.push_back(leaf.state_interface);}
    controller->set_native_mode(true, 1.0, leaf.offset);
    controller->set_actuator_ports({leaf.command_interface});
    controller->set_reference_interface_names({"target"});
    controller->set_command_interface_configuration(individual({leaf.command_interface}));
    controller->set_state_interface_configuration(individual(states));
    group.cm->add_controller(controller, leaf.name, kChainedType);
    ConfigureController(group.cm, leaf.name);
    controller->set_chained_mode(true);
    leaf_controllers.push_back(controller);
  }

  // Activate leaves before the root so the root can claim their reference interfaces.
  for (const auto & leaf : leaves) {SwitchNow(group.cm, {leaf.name}, {});}
  SwitchNow(group.cm, {kRoot}, {});
  group.leaf = leaf_controllers.front();
  if (leaf_controllers.size() > 1) {group.second_leaf = leaf_controllers[1];}
  group.extra_leaves = leaf_controllers;
  return group;
}

Group SetupStagedForkN(
  const std::shared_ptr<rclcpp::Executor> & executor, const std::string & name,
  const std::vector<LeafSpec> & leaves)
{
  Group group;
  group.cm = MakeManager(executor, name);
  group.root = std::make_shared<TestStagedController>();

  std::vector<std::string> reference_claims;
  for (const auto & leaf : leaves) {reference_claims.push_back(leaf.name + "/target");}
  group.root->set_reference_interface_names({"command"});
  group.root->set_command_interface_configuration(individual(reference_claims));
  group.root->set_state_interface_configuration(individual({}));
  group.cm->add_controller(group.root, kRoot, kChainedType);

  std::vector<std::shared_ptr<TestStagedController>> leaf_controllers;
  for (const auto & leaf : leaves)
  {
    auto controller = std::make_shared<TestStagedController>();
    std::vector<std::string> states;
    if (!leaf.state_interface.empty()) {states.push_back(leaf.state_interface);}
    controller->set_actuator_ports({leaf.command_interface});
    controller->set_reference_interface_names({"target"});
    controller->set_command_interface_configuration(individual({leaf.command_interface}));
    controller->set_state_interface_configuration(individual(states));
    controller->set_state_offset(leaf.offset);
    group.cm->add_controller(controller, leaf.name, kChainedType);
    ConfigureController(group.cm, leaf.name);
    controller->set_chained_mode(true);
    leaf_controllers.push_back(controller);
  }
  ConfigureController(group.cm, kRoot);

  std::vector<std::string> members{kRoot};
  for (const auto & leaf : leaves) {members.push_back(leaf.name);}
  if (group.cm->set_staged_execution_group(members) != Return::OK)
  {
    throw std::runtime_error("install staged fork");
  }
  for (const auto & leaf : leaves) {SwitchNow(group.cm, {leaf.name}, {});}
  SwitchNow(group.cm, {kRoot}, {});
  group.leaf = leaf_controllers.front();
  if (leaf_controllers.size() > 1) {group.second_leaf = leaf_controllers[1];}
  group.extra_leaves = leaf_controllers;
  return group;
}

Group SetupChainedFork(const std::shared_ptr<rclcpp::Executor> & executor, const std::string & name)
{
  return SetupChainedForkN(executor, name, TwoLeafFork());
}

Group SetupStagedFork(const std::shared_ptr<rclcpp::Executor> & executor, const std::string & name)
{
  return SetupStagedForkN(executor, name, TwoLeafFork());
}

struct ForkMetrics
{
  std::vector<double> command_a;
  std::vector<double> command_b;
  int update_errors = 0;
};

ForkMetrics RunForkFault(Mode mode, const std::shared_ptr<rclcpp::Executor> & executor, int cycles)
{
  if (mode != Mode::staged && mode != Mode::chained)
  {
    throw std::invalid_argument("fork comparison only covers chained and staged modes");
  }
  Group group = (mode == Mode::staged) ? SetupStagedFork(executor, "cmp_fork_staged_cm")
                                       : SetupChainedFork(executor, "cmp_fork_chained_cm");
  ForkMetrics metrics;
  const int fail_cycle = 5;
  for (int cycle = 1; cycle <= cycles; ++cycle)
  {
    group.root->set_external_reference(ReferenceFor(cycle));
    const bool failing = (cycle == fail_cycle);
    if (failing) {group.leaf->set_fail_command(true);}

    group.cm->read(kTime, kPeriod);
    const auto result = group.cm->update(kTime, kPeriod);

    if (failing) {group.leaf->set_fail_command(false);}
    if (result != Return::OK) {++metrics.update_errors;}
    const double a = (mode == Mode::staged) ? group.leaf->committed_value()
                                            : group.leaf->command_interface_value();
    const double b = (mode == Mode::staged) ? group.second_leaf->committed_value()
                                            : group.second_leaf->command_interface_value();
    metrics.command_a.push_back(a);
    metrics.command_b.push_back(b);
  }
  if (group.cm->staged_execution_group()) {group.cm->clear_staged_execution_group();}
  return metrics;
}

/// Two-level fork: S_root = 2 * (S_a + S_b), C_root = R - S_root, C_x = C_root - S_x.
double ForkExpectedA(double reference) {return reference - 2.0 * (kOffsetA + kOffsetB) - kOffsetA;}
double ForkExpectedB(double reference) {return reference - 2.0 * (kOffsetA + kOffsetB) - kOffsetB;}

TEST_F(HierarchyFairComparison, sibling_partial_commit_in_native_chaining_versus_staged_group)
{
  const int cycles = 9;
  const auto chained = RunForkFault(Mode::chained, executor_, cycles);
  const auto staged = RunForkFault(Mode::staged, executor_, cycles);
  const auto failing = static_cast<std::size_t>(4);  // cycle 5, zero-based

  // Both implementations agree on every healthy cycle with the analytic result.
  for (int cycle = 1; cycle <= cycles; ++cycle)
  {
    if (cycle == 5) {continue;}
    const auto index = static_cast<std::size_t>(cycle - 1);
    EXPECT_DOUBLE_EQ(ForkExpectedA(ReferenceFor(cycle)), chained.command_a[index]);
    EXPECT_DOUBLE_EQ(ForkExpectedB(ReferenceFor(cycle)), chained.command_b[index]);
    EXPECT_DOUBLE_EQ(ForkExpectedA(ReferenceFor(cycle)), staged.command_a[index]);
    EXPECT_DOUBLE_EQ(ForkExpectedB(ReferenceFor(cycle)), staged.command_b[index]);
  }
  EXPECT_EQ(1, chained.update_errors);
  EXPECT_EQ(1, staged.update_errors);

  // Staged group: leaf_a fails inside the command phase, so no command sink is called at all and
  // both hardware buffers keep the previous cycle's values. The two siblings stay consistent.
  EXPECT_DOUBLE_EQ(staged.command_a[failing - 1], staged.command_a[failing]);
  EXPECT_DOUBLE_EQ(staged.command_b[failing - 1], staged.command_b[failing]);

  // Native chaining: leaf_a fails, but the manager only logs the error and still runs leaf_b in
  // the same cycle. joint3/velocity therefore receives a new command while joint2/velocity keeps
  // the old one - exactly the mixed sibling state the POV-chassis case must avoid.
  EXPECT_DOUBLE_EQ(chained.command_a[failing - 1], chained.command_a[failing]);
  EXPECT_DOUBLE_EQ(ForkExpectedB(ReferenceFor(5)), chained.command_b[failing]);
  EXPECT_NE(chained.command_b[failing - 1], chained.command_b[failing]);

  // Both recover on the next cycle.
  EXPECT_DOUBLE_EQ(ForkExpectedA(ReferenceFor(6)), staged.command_a[failing + 1]);
  EXPECT_DOUBLE_EQ(ForkExpectedB(ReferenceFor(6)), staged.command_b[failing + 1]);
  EXPECT_DOUBLE_EQ(ForkExpectedA(ReferenceFor(6)), chained.command_a[failing + 1]);
  EXPECT_DOUBLE_EQ(ForkExpectedB(ReferenceFor(6)), chained.command_b[failing + 1]);
}

/// Gate B evidence: the same application code runs a 2-leaf and a 3-leaf fork. The only difference
/// between the two topologies is one `LeafSpec` entry - no controller class change, no phase
/// schedule change, no new buffer or commit code. Both the staged group and native chaining reuse
/// the identical leaf controller for every leaf.
TEST_F(HierarchyFairComparison, adding_a_leaf_is_configuration_only)
{
  const int cycles = 6;
  for (const auto & leaves : {TwoLeafFork(), ThreeLeafFork()})
  {
    double offset_sum = 0.0;
    for (const auto & leaf : leaves) {offset_sum += leaf.offset;}

    for (const auto mode : {Mode::staged, Mode::chained})
    {
      Group group =
        (mode == Mode::staged)
          ? SetupStagedForkN(executor_, "cmp_wire_staged_cm", leaves)
          : SetupChainedForkN(executor_, "cmp_wire_chained_cm", leaves);

      for (int cycle = 1; cycle <= cycles; ++cycle)
      {
        const double reference = ReferenceFor(cycle);
        group.root->set_external_reference(reference);
        group.cm->read(kTime, kPeriod);
        ASSERT_EQ(Return::OK, group.cm->update(kTime, kPeriod));
        const double root_command = reference - 2.0 * offset_sum;
        for (std::size_t i = 0; i < leaves.size(); ++i)
        {
          const double expected = root_command - leaves[i].offset;
          const double observed = (mode == Mode::staged)
                                    ? group.extra_leaves[i]->committed_value()
                                    : group.extra_leaves[i]->command_interface_value();
          EXPECT_DOUBLE_EQ(expected, observed)
            << "leaves=" << leaves.size() << " leaf " << i << " cycle " << cycle;
        }
      }
      if (group.cm->staged_execution_group()) {group.cm->clear_staged_execution_group();}
    }
  }
}

/// Host the identical tree in one generic composite plugin that reuses the same kernel in library
/// mode. No chainable interfaces, no reference interfaces, no ControllerManager change.
Group SetupLibraryForkN(
  const std::shared_ptr<rclcpp::Executor> & executor, const std::string & name,
  const std::vector<LeafSpec> & leaves)
{
  Group group;
  group.cm = MakeManager(executor, name);
  group.generic = std::make_shared<GenericCompositeController>();

  std::vector<CompositeNodeSpec> specs;
  CompositeNodeSpec root;
  root.name = "lc_root";
  root.factor = 2.0;
  specs.push_back(root);
  for (const auto & leaf : leaves)
  {
    CompositeNodeSpec spec;
    spec.name = std::string("lc_") + leaf.name;
    spec.parent = "lc_root";
    if (!leaf.state_interface.empty()) {spec.state_interfaces.push_back(leaf.state_interface);}
    spec.command_interfaces.push_back(leaf.command_interface);
    spec.factor = 1.0;
    spec.offset = leaf.offset;
    specs.push_back(spec);
  }
  group.generic->set_nodes(std::move(specs));
  group.cm->add_controller(group.generic, "cmp_generic", "generic_composite");
  ConfigureController(group.cm, "cmp_generic");
  SwitchNow(group.cm, {"cmp_generic"}, {});
  return group;
}

/// Gate B: the generic composite library host produces the same output as the manager-integrated
/// staged group for 2-leaf and 3-leaf forks, and its control path is allocation-free from the very
/// first cycle (the kernel is built during activation, not lazily inside `update()`).
TEST_F(HierarchyFairComparison, library_host_matches_staged_group)
{
  for (const auto & leaves : {TwoLeafFork(), ThreeLeafFork()})
  {
    double offset_sum = 0.0;
    for (const auto & leaf : leaves) {offset_sum += leaf.offset;}

    auto staged_group = SetupStagedForkN(executor_, "cmp_lib_staged_cm", leaves);
    auto library_group = SetupLibraryForkN(executor_, "cmp_lib_library_cm", leaves);

    // The FIRST update after activation must already be allocation-free. Before the kernel was
    // built in `on_activate()`, this cycle was where the lazy build ran, with its `reserve`,
    // `make_shared` and node construction on the control path.
    g_allocation_count.store(0, std::memory_order_relaxed);
    g_count_allocations.store(true, std::memory_order_relaxed);
    ASSERT_EQ(Return::OK, library_group.generic->update(kTime, kPeriod));
    g_count_allocations.store(false, std::memory_order_relaxed);
    const auto first_cycle_allocations = g_allocation_count.load(std::memory_order_relaxed);
    std::cout << "[comparison] generic composite first post-activation update allocations="
              << first_cycle_allocations << " (leaves=" << leaves.size() << ")\n";
    EXPECT_EQ(0u, first_cycle_allocations)
      << "the kernel must be built during activation, so the first control cycle cannot allocate";

    for (int cycle = 1; cycle <= 6; ++cycle)
    {
      const double reference = ReferenceFor(cycle);
      staged_group.root->set_external_reference(reference);
      library_group.generic->set_external_reference(reference);
      staged_group.cm->read(kTime, kPeriod);
      library_group.cm->read(kTime, kPeriod);
      ASSERT_EQ(Return::OK, staged_group.cm->update(kTime, kPeriod));
      ASSERT_EQ(Return::OK, library_group.cm->update(kTime, kPeriod));
      for (std::size_t i = 0; i < leaves.size(); ++i)
      {
        const double expected = reference - 2.0 * offset_sum - leaves[i].offset;
        EXPECT_DOUBLE_EQ(expected, staged_group.extra_leaves[i]->committed_value())
          << "staged, leaves=" << leaves.size() << " leaf " << i;
        EXPECT_DOUBLE_EQ(expected, library_group.generic->command_interface_value(i))
          << "library, leaves=" << leaves.size() << " leaf " << i;
      }
    }
    EXPECT_GE(library_group.generic->build_allocations, 1);

    // The steady-state update path must not allocate either.
    g_allocation_count.store(0, std::memory_order_relaxed);
    g_count_allocations.store(true, std::memory_order_relaxed);
    for (int i = 0; i < 100; ++i)
    {
      ASSERT_EQ(Return::OK, library_group.generic->update(kTime, kPeriod));
    }
    g_count_allocations.store(false, std::memory_order_relaxed);
    const auto library_allocations = g_allocation_count.load(std::memory_order_relaxed);
    std::cout << "[comparison] generic composite library update allocations per 100 calls="
              << library_allocations << " (leaves=" << leaves.size() << ")\n";
    EXPECT_EQ(0u, library_allocations);

    staged_group.cm->clear_staged_execution_group();
  }
}

/// R8: the two-phase membership set must be built off the real-time path.
///
/// The reviewed revision set a "dirty" flag inside the switch and rebuilt the entry vector at the
/// top of the NEXT real-time cycle (`reserve` / `push_back` / `dynamic_cast` / `sort` plus the old
/// vector's destructor). That is a per-configuration allocation inside `update()`, and it is
/// invisible to a steady-state allocation measurement.
///
/// The probe is deliberately run with NO controller active: the per-cycle allocation count then
/// contains no per-controller component, so the baseline is flat and a membership rebuild shows up
/// as a clearly larger cycle. `switch_controller()` still republishes membership while every member
/// is inactive, which is exactly what makes this window meaningful.
///
/// The count is restricted to the control-loop thread, so the concurrent `switch_controller()`
/// thread (which legitimately allocates: async launch, controller-list copy, service bookkeeping)
/// can neither mask nor fake the result. Counting starts on the first cycle AFTER the switch has
/// been applied -- the cycle the reviewed revision rebuilt in.
TEST_F(HierarchyFairComparison, post_switch_two_phase_cycles_do_not_rebuild_membership)
{
  constexpr int kBaselineCycles = 30;
  constexpr int kPostSwitchCycles = 10;

  auto cm = MakeManager(executor_, "cmp_r8_cm");
  auto root = std::make_shared<TestStagedController>();
  auto mid = std::make_shared<TestStagedController>();
  auto leaf = std::make_shared<TestStagedController>();

  // root claims mid/target, mid claims leaf/target: root is the top of the command chain, which
  // also exercises the parent-derivation direction that the staged kernel documents.
  root->set_reference_interface_names({"command"});
  root->set_command_interface_configuration(individual({std::string(kModule) + "/target"}));
  root->set_state_interface_configuration(individual({}));
  mid->set_reference_interface_names({"target"});
  mid->set_command_interface_configuration(individual({std::string(kLeaf) + "/target"}));
  mid->set_state_interface_configuration(individual({}));
  leaf->set_reference_interface_names({"target"});
  leaf->set_actuator_ports({"joint2/velocity"});
  leaf->set_command_interface_configuration(individual({"joint2/velocity"}));
  leaf->set_state_interface_configuration(individual({}));
  for (auto * controller : {root.get(), mid.get(), leaf.get()})
  {
    controller->set_two_phase_legacy(true);
  }

  cm->add_controller(root, kRoot, kChainedType);
  cm->add_controller(mid, kModule, kChainedType);
  cm->add_controller(leaf, kLeaf, kChainedType);
  ConfigureController(cm, kLeaf);
  ConfigureController(cm, kModule);
  ConfigureController(cm, kRoot);
  ASSERT_EQ(Return::OK, cm->set_two_phase_execution(true));

  // Everything stays INACTIVE: the per-cycle allocation count is then the fixed control-loop
  // overhead only, with no per-active-controller component to hide a rebuild behind.
  AllocationCounterGuard guard;
  std::array<std::size_t, kBaselineCycles> per_cycle{};
  std::size_t previous = 0;
  g_counted_thread = std::this_thread::get_id();
  g_count_only_current_thread.store(true, std::memory_order_relaxed);
  g_allocation_count.store(0, std::memory_order_relaxed);
  g_count_allocations.store(true, std::memory_order_relaxed);
  for (int i = 0; i < kBaselineCycles; ++i)
  {
    ASSERT_EQ(Return::OK, cm->update(kTime, kPeriod));
    const auto now = g_allocation_count.load(std::memory_order_relaxed);
    per_cycle[static_cast<std::size_t>(i)] = now - previous;
    previous = now;
  }
  g_count_allocations.store(false, std::memory_order_relaxed);

  // The very first update() of a freshly configured manager is a warm-up cycle; after it the idle
  // loop must be perfectly flat, otherwise this probe could not attribute a change to anything.
  for (int i = 2; i < kBaselineCycles; ++i)
  {
    ASSERT_EQ(per_cycle[1], per_cycle[static_cast<std::size_t>(i)])
      << "the idle control loop must have a flat per-cycle allocation count for this probe to "
         "mean anything (cycle " << i << ")";
  }

  // Activate and then deactivate `leaf`, with counting disabled. The deactivation leaves the group
  // empty again, so the window below is directly comparable with the baseline; and because it is a
  // switch, the reviewed revision left its entry set dirty for the very next control cycle.
  const auto run_switch = [&](const std::vector<std::string> & activate,
                              const std::vector<std::string> & deactivate, int expected_state)
  {
    auto future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm.get(),
      activate, deactivate, kStrict, true, rclcpp::Duration(0, 0));
    for (int i = 0;
         i < 4000 &&
         future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cm->update(kTime, kPeriod);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_EQ(std::future_status::ready, future.wait_for(std::chrono::seconds(30)));
    EXPECT_EQ(Return::OK, future.get());
    EXPECT_EQ(expected_state, leaf->get_state().id());
  };
  run_switch({kLeaf}, {}, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Start the deactivation but stop as soon as it has been applied: the whole update() that applied
  // it has returned by then, so do_switch is already false and the next cycle is the first
  // post-switch cycle.
  auto future = std::async(
    std::launch::async, &controller_manager::ControllerManager::switch_controller, cm.get(),
    std::vector<std::string>{}, std::vector<std::string>{kLeaf}, kStrict, true,
    rclcpp::Duration(0, 0));
  for (int i = 0; i < 4000; ++i)
  {
    cm->update(kTime, kPeriod);
    if (leaf->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, leaf->get_state().id())
    << "the deactivation switch never completed";

  // Count from the first post-switch cycle. The loop must keep running until switch_controller()
  // returns, because that thread waits (inside `switch_updated_list`) for the control loop to move
  // off the old controller list.
  std::array<std::size_t, 4096> post_cycle{};
  std::size_t cycles_counted = 0;
  g_allocation_count.store(0, std::memory_order_relaxed);
  previous = 0;
  g_count_allocations.store(true, std::memory_order_relaxed);
  for (int i = 0;
       i < 4000 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    ASSERT_EQ(Return::OK, cm->update(kTime, kPeriod));
    const auto now = g_allocation_count.load(std::memory_order_relaxed);
    post_cycle[cycles_counted] = now - previous;
    previous = now;
    ++cycles_counted;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  for (int i = 0; i < kPostSwitchCycles; ++i)
  {
    ASSERT_EQ(Return::OK, cm->update(kTime, kPeriod));
    const auto now = g_allocation_count.load(std::memory_order_relaxed);
    post_cycle[cycles_counted] = now - previous;
    previous = now;
    ++cycles_counted;
  }
  g_count_allocations.store(false, std::memory_order_relaxed);

  ASSERT_EQ(std::future_status::ready, future.wait_for(std::chrono::seconds(30)));
  ASSERT_EQ(Return::OK, future.get());
  ASSERT_GT(cycles_counted, 1u);

  std::cout << "[comparison] idle baseline per cycle: first=" << per_cycle[0]
            << " steady=" << per_cycle[1] << " | post-switch profile:";
  for (std::size_t i = 0; i < cycles_counted; ++i) {std::cout << " " << post_cycle[i];}
  std::cout << "\n";

  // The cycle that applied the switch is not counted, so the first counted cycle is the one the
  // reviewed revision rebuilt its membership in.
  //
  // MEASURED LIMITATION, stated so nobody over-reads this probe: the settled idle loop already
  // spends ~18 allocations per cycle in upstream code, while rebuilding a 3-entry membership vector
  // costs only ~2 (`make_shared` + `reserve`). The ambient count therefore dwarfs the signal, and no
  // assertion here can prove the absence of the rebuild. What is asserted is:
  //   * the settled cycles after the switch cost EXACTLY one idle cycle -- this does catch a
  //     rebuild (or any other allocation) that runs every cycle;
  //   * the first post-switch cycle carries at most the one-off reconfiguration allocation the
  //     probe actually measures (+1 here), which is below the >=2 allocations the reviewed
  //     `make_shared` + `reserve` rebuild needed.
  // The structural guarantee is in the code: `update()` contains no call that republishes
  // membership, so there is nothing left on the control path to rebuild.
  EXPECT_LE(post_cycle[0], per_cycle[1] + 1u)
    << "a surplus on the first post-switch control cycle is the signature of rebuilding membership "
       "inside update()";
  for (std::size_t i = 1; i < cycles_counted; ++i)
  {
    EXPECT_EQ(per_cycle[1], post_cycle[i])
      << "settled idle cycles after a switch must cost exactly one idle cycle (cycle " << i << ")";
  }
}

}  // namespace


/// The typed tree example hosted by an ordinary composite plugin, through the LIBRARY path.
///
/// This is the last piece of review recommendation 5. `SetupLibraryForkN` already hosts the same
/// two-leaf fork from RUNTIME specs (`CompositeNodeSpec`: parent names as strings, port names
/// hand-written on the node, kernel built from a `Spec`). Here the same fork is declared ONCE as
/// types -- `typed_fork::root_ports` / `a_ports` / `b_ports` inside the plugin -- and hosted through
/// `topology_binding::create_library_group`, the checked entry, which:
///   * requires each child to declare the parent it is nested under (compile time),
///   * requires the parent's per-child reference/state lists to be the concatenation, in child order,
///     of the children's declarations, with matching physical dimensions (compile time),
///   * verifies the RUNTIME port strings of every node in the tree against that declaration,
///   * then builds the plan and the group.
///
/// The comparison is exact and per cycle: the typed host must produce the same leaf commands as the
/// data-driven host and as the closed-form expectation, because the algorithm is identical and only
/// the SOURCE of the topology differs.
TEST_F(HierarchyFairComparison, typed_declaration_hosted_by_a_composite_plugin_matches_the_spec_host)
{
  constexpr int kCycles = 6;
  const double offset_sum = kOffsetA + kOffsetB;

  // Host 1: the runtime-spec composite (existing baseline).
  Group spec_host = SetupLibraryForkN(executor_, "cmp_typed_spec_cm", TwoLeafFork());

  // Host 2: the same fork declared as types, in its own plugin, on its own manager.
  Group typed_host;
  typed_host.cm = MakeManager(executor_, "cmp_typed_decl_cm");
  auto typed = std::make_shared<TypedForkCompositeController>();
  typed->set_external_reference(0.0);
  typed_host.cm->add_controller(typed, "cmp_typed", "typed_fork");
  ConfigureController(typed_host.cm, "cmp_typed");
  SwitchNow(typed_host.cm, {"cmp_typed"}, {});

  // The plan really came from the declaration: three nodes, parent before children.
  const auto names = typed->plan_node_names();
  ASSERT_EQ(3u, names.size());
  EXPECT_EQ("typed_root", names[0]);
  EXPECT_EQ("typed_a", names[1]);
  EXPECT_EQ("typed_b", names[2]);

  // The first post-activation cycle must be allocation-free: the kernel (including the checked
  // binding and the plan) is built in `on_activate()`, never on the control path.
  g_allocation_count.store(0, std::memory_order_relaxed);
  g_count_allocations.store(true, std::memory_order_relaxed);
  ASSERT_EQ(Return::OK, typed->update(kTime, kPeriod));
  g_count_allocations.store(false, std::memory_order_relaxed);
  const auto first_cycle_allocations = g_allocation_count.load(std::memory_order_relaxed);
  std::cout << "[comparison] typed fork first post-activation update allocations="
            << first_cycle_allocations << "\n";
  EXPECT_EQ(0u, first_cycle_allocations);

  // Counts are captured AFTER activation: `SwitchNow` pumps cycles, and the plugin is already active
  // in the cycle that applies the switch, so absolute counts are not a property of this test.
  std::vector<int> state_before(3);
  std::vector<int> command_before(3);
  for (std::size_t node = 0; node < 3; ++node)
  {
    state_before[node] = typed->state_calls(node);
    command_before[node] = typed->command_calls(node);
  }
  const auto commits_before = typed->commit_calls;
  for (int cycle = 1; cycle <= kCycles; ++cycle)
  {
    const double reference = ReferenceFor(cycle);
    typed->set_external_reference(reference);
    spec_host.generic->set_external_reference(reference);

    ASSERT_EQ(Return::OK, typed->update(kTime, kPeriod));
    ASSERT_EQ(Return::OK, spec_host.generic->update(kTime, kPeriod));

    const double expected_a = ForkExpectedA(reference);
    const double expected_b = ForkExpectedB(reference);
    EXPECT_DOUBLE_EQ(expected_a, typed->command_interface_value(0))
      << "typed host, cycle " << cycle;
    EXPECT_DOUBLE_EQ(expected_b, typed->command_interface_value(1))
      << "typed host, cycle " << cycle;
    // ... and identical to the data-driven host on the same input.
    EXPECT_DOUBLE_EQ(spec_host.generic->command_interface_value(0), typed->command_interface_value(0));
    EXPECT_DOUBLE_EQ(spec_host.generic->command_interface_value(1), typed->command_interface_value(1));
  }

  // One state stage and one command stage per node per cycle, for ALL THREE nodes of the tree, and
  // one commit per leaf per cycle.
  for (std::size_t node = 0; node < 3; ++node)
  {
    EXPECT_EQ(state_before[node] + kCycles, typed->state_calls(node))
      << "node " << names[node] << " ran a wrong number of state stages";
    EXPECT_EQ(command_before[node] + kCycles, typed->command_calls(node))
      << "node " << names[node] << " ran a wrong number of command stages";
  }
  EXPECT_EQ(commits_before + 2 * kCycles, typed->commit_calls)
    << "the two leaves must commit once each per cycle";
  (void)offset_sum;
}
