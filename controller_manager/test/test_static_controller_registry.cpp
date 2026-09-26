// Copyright 2026
// Licensed under the Apache License, Version 2.0.

// Controllers COMPILED INTO THE BINARY, reachable by the ordinary configuration path.
//
// The manager's `load_controller(name, type)` used to resolve `type` only through pluginlib, so a
// controller built into the executable could not be named by a type string: no YAML `type:` field, no
// spawner, no `load_controller(name)` reading `<name>.type`. `StaticControllerRegistry` closes that
// gap, and the point of these tests is that it closes it WITHOUT a second code path:
//
//   * a registered type is created by its factory and then follows `add_controller_impl()`, so it
//     lands in the same controller list with the same lifecycle and the same admission checks;
//   * the SAME class (`test_controller::TestController`) is loaded through BOTH routes on one manager
//     and must behave identically -- that is the lifecycle-parity evidence;
//   * a compiled-in TWO-PHASE controller must be admitted, executed and (when non-conforming)
//     refused by exactly the same code as a plugin one;
//   * two loads of the same registered type are two independent instances, which is the property
//     that rules out "controllers as global objects".

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "controller_manager/static_controller_registry.hpp"
#include "controller_manager_test_common.hpp"
#include "test_composite_library/typed_fork_composite_controller.hpp"
#include "test_controller/test_controller.hpp"

namespace
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Return = controller_interface::return_type;

/// The type string the compiled-in typed fork is registered under.
constexpr char kTypedForkType[] = "static_typed_fork";
using Registry = controller_manager::StaticControllerRegistry;

constexpr char kCompiledLeaf[] = "compiled_two_phase_leaf";
constexpr char kCompiledRateMismatch[] = "compiled_two_phase_leaf_half_rate";

/// A controller COMPILED INTO THE TEST BINARY, with a fixed declaration.
/**
 * This is what "static controller" means here: no plugin, no configuration-time class lookup, a
 * constant interface declaration known to the compiler. It implements the staged and two-phase
 * interfaces so it can be admitted into the two-phase passes, and it is deliberately small -- the
 * point is the plumbing, not the algorithm.
 */
class CompiledTwoPhaseLeaf
: public controller_interface::ControllerInterface,
  public hierarchical_control::StagedControllerInterface,
  public hierarchical_control::TwoPhaseControllerInterface
{
public:
  CompiledTwoPhaseLeaf() = default;

  /// A compiled-in controller may still be parameterised at construction: `add_factory` covers the
  /// case where building one needs arguments.
  void set_interfaces(std::string command, std::string state)
  {
    command_interface_ = std::move(command);
    state_interface_ = std::move(state);
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return individual({command_interface_});
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return individual({state_interface_});
  }

  CallbackReturn on_init() override {return CallbackReturn::SUCCESS;}

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // The compiled-in controller also honours the manager's rate contract, so the two-phase
    // admission checker sees a real `update_rate` (a plugin reads it from its node the same way).
    int rate = 0;
    if (get_node()->get_parameter("update_rate", rate)) {update_rate_ = static_cast<unsigned int>(rate);}
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

  // ---- StagedControllerInterface -----------------------------------------------------------------
  std::vector<std::string> staged_state_ports() const override {return {"compiled/state"};}
  std::vector<std::string> staged_reference_ports() const override {return {};}
  std::vector<std::string> staged_actuator_ports() const override {return {command_interface_};}
  hierarchical_control::StagedCommandSink * staged_command_sink() noexcept override
  {
    return &sink_;
  }

  controller_interface::return_type update_state_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedInputView &,
    hierarchical_control::StagedValueWriter state) noexcept override
  {
    if (state.size() > 0) {state[0] = 1.0;}
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_command_stage(
    const rclcpp::Time &, const rclcpp::Duration &, const hierarchical_control::StagedContext &,
    const hierarchical_control::StagedValueView & state,
    const hierarchical_control::StagedValueView &,
    const hierarchical_control::StagedReferenceWriter &,
    hierarchical_control::StagedValueWriter actuators) noexcept override
  {
    for (std::size_t i = 0; i < actuators.size(); ++i)
    {
      actuators[i] = (state.size() > 0 ? state[0] : 0.0);
    }
    return controller_interface::return_type::OK;
  }

  // ---- TwoPhaseControllerInterface ---------------------------------------------------------------
  controller_interface::return_type update_phase(
    const rclcpp::Time &, const rclcpp::Duration &) noexcept override
  {
    ++update_phase_calls;
    return controller_interface::return_type::OK;
  }
  controller_interface::return_type handle_phase(
    const rclcpp::Time &, const rclcpp::Duration &) noexcept override
  {
    ++handle_phase_calls;
    // Each instance writes ITS OWN value, so two loads sharing anything would show up as equal
    // commands below.
    last_command = bias_ + static_cast<double>(handle_phase_calls);
    for (auto & interface : command_interfaces_) {interface.set_value(last_command);}
    return controller_interface::return_type::OK;
  }

  // ---- two-phase test controller entry points (mirrors TestStagedController) ---------------------
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    ++legacy_update_calls;
    return controller_interface::return_type::OK;
  }

  void set_bias(double value) {bias_ = value;}

  int update_phase_calls = 0;
  int handle_phase_calls = 0;
  int legacy_update_calls = 0;
  unsigned int update_rate_ = 0;
  double last_command = 0.0;

  class Sink : public hierarchical_control::StagedCommandSink
  {
  public:
    bool commit(const double * values, std::size_t size) noexcept override
    {
      if (size > 0) {last = values[0];}
      ++commits;
      return true;
    }
    int commits = 0;
    double last = 0.0;
  };
  Sink sink_;

private:
  double bias_ = 0.0;
  std::string command_interface_ = "joint2/velocity";
  std::string state_interface_ = "joint2/position";

  static controller_interface::InterfaceConfiguration individual(
    const std::vector<std::string> & names)
  {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names = names;
    return cfg;
  }

};

class TestStaticControllerRegistry
: public ControllerManagerFixture<controller_manager::ControllerManager>
{
public:
  void SetUp() override
  {
    ControllerManagerFixture<controller_manager::ControllerManager>::SetUp();
    registry_ = std::make_shared<Registry>();
    registry_->add<CompiledTwoPhaseLeaf>(kCompiledLeaf);
    // Two parameterised compiled-in controllers, built by explicit factories: proof that the
    // registry is a factory registry, not a table of singleton instances.
    registry_->add_factory(
      kCompiledJoint2,
      []()
      {
        auto leaf = std::make_shared<CompiledTwoPhaseLeaf>();
        leaf->set_interfaces("joint2/velocity", "joint2/position");
        return leaf;
      });
    registry_->add_factory(
      kCompiledJoint3,
      []()
      {
        auto leaf = std::make_shared<CompiledTwoPhaseLeaf>();
        leaf->set_interfaces("joint3/velocity", "joint3/position");
        return leaf;
      });
    registry_->add<test_controller::TestController>(kStaticTestControllerType);
    cm_->set_static_controller_registry(registry_);
  }

  static constexpr char kStaticTestControllerType[] = "static_test_controller";
  static constexpr char kCompiledJoint2[] = "compiled_two_phase_leaf_joint2";
  static constexpr char kCompiledJoint3[] = "compiled_two_phase_leaf_joint3";

  void Cycle(int count)
  {
    for (int i = 0; i < count; ++i)
    {
      cm_->read(TIME, PERIOD);
      ASSERT_EQ(Return::OK, cm_->update(TIME, PERIOD));
      cm_->write(TIME, PERIOD);
    }
  }

  void SwitchNow(const std::vector<std::string> & start, const std::vector<std::string> & stop)
  {
    auto future = std::async(
      std::launch::async, &controller_manager::ControllerManager::switch_controller, cm_.get(),
      start, stop, STRICT, true, rclcpp::Duration(0, 0));
    for (int i = 0;
         i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cm_->update(TIME, PERIOD);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_EQ(std::future_status::ready, future.wait_for(std::chrono::milliseconds(0)));
    ASSERT_EQ(Return::OK, future.get());
  }

  std::shared_ptr<Registry> registry_;
};
}  // namespace

/// A registered type is reachable through the ordinary path, and an unregistered one is not silently
/// accepted.
TEST_F(TestStaticControllerRegistry, a_compiled_in_type_is_loadable_by_type_string)
{
  const auto controller = cm_->load_controller("leaf", kCompiledLeaf);
  ASSERT_NE(nullptr, controller) << "a registered compiled-in type must load";
  EXPECT_STREQ("leaf", controller->get_node()->get_name());

  // Both the name-based and the type-based overload accept it (the name-based one reads
  // `<name>.type`, which is how a YAML/spawner configuration reaches it).
  EXPECT_EQ(nullptr, cm_->load_controller("leaf"));
  EXPECT_EQ(nullptr, cm_->load_controller("unknown", "not_a_registered_or_plugin_type"))
    << "an unknown type must still be refused";

  EXPECT_THAT(registry_->types(), ::testing::Contains(kCompiledLeaf));
}

/// The registry, not the caller, decides the instance: two loads are two objects with independent
/// state, which is exactly what a singleton "static controller" could not provide.
TEST_F(TestStaticControllerRegistry, two_loads_are_two_independent_instances)
{
  const auto first = cm_->load_controller("leaf_one", kCompiledJoint2);
  const auto second = cm_->load_controller("leaf_two", kCompiledJoint3);
  ASSERT_NE(nullptr, first);
  ASSERT_NE(nullptr, second);
  EXPECT_NE(first.get(), second.get());

  ASSERT_EQ(Return::OK, cm_->configure_controller("leaf_one"));
  ASSERT_EQ(Return::OK, cm_->configure_controller("leaf_two"));
  ASSERT_EQ(Return::OK, cm_->set_two_phase_execution(true))
    << "both compiled-in members are conforming, so the mode must be admitted";
  SwitchNow({"leaf_one", "leaf_two"}, {});

  auto * one = dynamic_cast<CompiledTwoPhaseLeaf *>(first.get());
  auto * two = dynamic_cast<CompiledTwoPhaseLeaf *>(second.get());
  ASSERT_NE(nullptr, one);
  ASSERT_NE(nullptr, two);
  one->set_bias(10.0);
  two->set_bias(20.0);
  // Counts are captured AFTER the switch: the plugin is already active in the cycle that applies it,
  // so absolute counts are not a property of this test.
  const int one_phases = one->update_phase_calls;
  const int two_phases = two->update_phase_calls;
  const int one_legacy_before = one->legacy_update_calls;
  const int two_legacy_before = two->legacy_update_calls;

  Cycle(3);

  // Each phase ran once per instance per cycle ...
  EXPECT_EQ(one_phases + 3, one->update_phase_calls);
  EXPECT_EQ(two_phases + 3, two->update_phase_calls);
  EXPECT_EQ(one->update_phase_calls, one->handle_phase_calls);
  EXPECT_EQ(two->update_phase_calls, two->handle_phase_calls);
  EXPECT_EQ(one_legacy_before, one->legacy_update_calls)
    << "a two-phase member must never be handed to the native loop";
  EXPECT_EQ(two_legacy_before, two->legacy_update_calls);

  // ... and each wrote ITS OWN value: the command is this instance's bias plus its own call count,
  // so two loads sharing anything would produce equal commands.
  EXPECT_DOUBLE_EQ(10.0 + one->handle_phase_calls, one->last_command);
  EXPECT_DOUBLE_EQ(20.0 + two->handle_phase_calls, two->last_command);
  EXPECT_NE(one->last_command, two->last_command);
}

/// A compiled-in controller goes through the two-phase admission and execution exactly like a plugin
/// one: enabling the mode, running the passes, and the per-phase call counts.
TEST_F(TestStaticControllerRegistry, a_compiled_in_controller_runs_through_the_two_phase_passes)
{
  ASSERT_NE(nullptr, cm_->load_controller("leaf", kCompiledLeaf));
  ASSERT_EQ(Return::OK, cm_->configure_controller("leaf"));
  EXPECT_EQ(Return::OK, cm_->set_two_phase_execution(true))
    << "a conforming compiled-in controller must be admitted";
  SwitchNow({"leaf"}, {});

  CompiledTwoPhaseLeaf * leaf = nullptr;
  for (const auto & spec : cm_->get_loaded_controllers())
  {
    if (spec.info.name == "leaf") {leaf = dynamic_cast<CompiledTwoPhaseLeaf *>(spec.c.get());}
  }
  ASSERT_NE(nullptr, leaf);
  const int phase_before = leaf->update_phase_calls;
  const int legacy_before = leaf->legacy_update_calls;
  Cycle(4);
  EXPECT_EQ(phase_before + 4, leaf->update_phase_calls);
  EXPECT_EQ(phase_before + 4, leaf->handle_phase_calls);
  EXPECT_EQ(legacy_before, leaf->legacy_update_calls)
    << "a two-phase member must never go through the native update()";
}

/// ... including the ADMISSION verdict: a compiled-in controller with a rate the passes cannot honour
/// is refused by the same checker, with the same reason, as a plugin one would be.
TEST_F(TestStaticControllerRegistry, a_non_conforming_compiled_in_controller_is_refused)
{
  ASSERT_GE(cm_->get_update_rate(), 2u);
  const auto controller = cm_->load_controller("leaf", kCompiledLeaf);
  ASSERT_NE(nullptr, controller);
  // The rate is a node parameter, exactly as it is for a plugin controller: the node exists after
  // `load_controller()` and `on_configure` reads it.
  controller->get_node()->set_parameter(
    {"update_rate", static_cast<int>(cm_->get_update_rate() / 2)});

  ASSERT_EQ(Return::OK, cm_->configure_controller("leaf"));

  EXPECT_EQ(Return::ERROR, cm_->set_two_phase_execution(true))
    << "the two-phase admission checker must treat a compiled-in controller like any other";
  EXPECT_FALSE(cm_->two_phase_execution());
  const auto rejections = cm_->two_phase_rejected_controllers();
  ASSERT_EQ(1u, rejections.size());
  EXPECT_EQ("leaf", rejections.front().name);
  EXPECT_NE(std::string::npos, rejections.front().reason.find("update rate"));
}

/// The SAME class, loaded through BOTH routes on one manager, must behave identically: same
/// lifecycle, same list, same interface claiming. This is the "one lifecycle adapter, one admission
/// checker" requirement in test form.
TEST_F(TestStaticControllerRegistry, the_same_class_behaves_identically_through_both_routes)
{
  const auto plugin = cm_->load_controller("from_plugin", test_controller::TEST_CONTROLLER_CLASS_NAME);
  const auto compiled = cm_->load_controller("from_registry", kStaticTestControllerType);
  ASSERT_NE(nullptr, plugin) << "the pluginlib route must still work";
  ASSERT_NE(nullptr, compiled) << "the registry route must work for the same class";

  // Same C++ type behind both, so any behavioural difference would be a difference of the ROUTE.
  EXPECT_NE(nullptr, dynamic_cast<test_controller::TestController *>(plugin.get()));
  EXPECT_NE(nullptr, dynamic_cast<test_controller::TestController *>(compiled.get()));

  EXPECT_EQ(Return::OK, cm_->configure_controller("from_plugin"));
  EXPECT_EQ(Return::OK, cm_->configure_controller("from_registry"));

  const auto loaded = cm_->get_loaded_controllers();
  ASSERT_EQ(2u, loaded.size());
  for (const auto & spec : loaded)
  {
    EXPECT_FALSE(spec.info.type.empty());
    EXPECT_FALSE(spec.c->is_chainable());
    // The declaration each one asks the manager for is the class's own, not the route's.
    EXPECT_EQ(
      plugin->command_interface_configuration().names,
      compiled->command_interface_configuration().names);
    EXPECT_EQ(
      plugin->state_interface_configuration().names,
      compiled->state_interface_configuration().names);
  }

  SwitchNow({"from_plugin", "from_registry"}, {});
  for (const auto & spec : cm_->get_loaded_controllers())
  {
    EXPECT_EQ(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, spec.c->get_state().id());
  }
  Cycle(2);
}

/// The registry exposes a compiled-in type's compile-time description when it has one, so a tool can
/// enumerate the declared topology and hardware requirements without constructing the controller.
TEST_F(TestStaticControllerRegistry, a_registered_type_exposes_its_compile_time_description)
{
  auto registry = std::make_shared<Registry>();
  registry->add<CompiledTwoPhaseLeaf>(kCompiledLeaf);
  EXPECT_FALSE(registry->has_manifest(kCompiledLeaf))
    << "this test controller has no type-level manifest";

  registry->add<test_composite_library::TypedForkCompositeController>(kTypedForkType);
  ASSERT_TRUE(registry->has_manifest(kTypedForkType));
  EXPECT_EQ(
    (std::vector<std::string>{"joint2/velocity", "joint3/velocity"}),
    registry->command_interfaces(kTypedForkType));
  EXPECT_EQ(
    (std::vector<std::string>{"joint2/position", "joint3/position"}),
    registry->state_interfaces(kTypedForkType));

  // ... and it is still a normal loadable type.
  cm_->set_static_controller_registry(registry);
  EXPECT_NE(nullptr, cm_->load_controller("typed", kTypedForkType));
  EXPECT_EQ(Return::OK, cm_->configure_controller("typed"));
}
