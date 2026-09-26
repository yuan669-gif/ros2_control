// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_

#include <atomic>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/controller_interface_base.hpp"

#include "controller_manager/static_controller_registry.hpp"
#include "controller_manager/controller_spec.hpp"
#include "controller_manager/staged_execution_group.hpp"
#include "hierarchical_control/two_phase_controller_interface.hpp"
#include "controller_manager/visibility_control.h"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/list_controller_types.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "controller_manager_msgs/srv/list_hardware_interfaces.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/reload_controller_libraries.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/resource_manager.hpp"

#include "pluginlib/class_loader.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace controller_manager
{
using ControllersListIterator = std::vector<controller_manager::ControllerSpec>::const_iterator;

CONTROLLER_MANAGER_PUBLIC rclcpp::NodeOptions get_cm_node_options();

class ControllerManager : public rclcpp::Node
{
public:
  static constexpr bool kWaitForAllResources = false;
  static constexpr auto kInfiniteTimeout = 0;

  CONTROLLER_MANAGER_PUBLIC
  ControllerManager(
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & namespace_ = "",
    const rclcpp::NodeOptions & options = get_cm_node_options());

  CONTROLLER_MANAGER_PUBLIC
  ControllerManager(
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & namespace_ = "",
    const rclcpp::NodeOptions & options = get_cm_node_options());

  CONTROLLER_MANAGER_PUBLIC
  virtual ~ControllerManager();

  /// Shutdown all controllers in the controller manager.
  /**
   * \return true if all controllers are successfully shutdown, false otherwise.
   */
  CONTROLLER_MANAGER_PUBLIC
  bool shutdown_controllers();

  CONTROLLER_MANAGER_PUBLIC
  void robot_description_callback(const std_msgs::msg::String & msg);

  CONTROLLER_MANAGER_PUBLIC
  void init_resource_manager(const std::string & robot_description);

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceBaseSharedPtr load_controller(
    const std::string & controller_name, const std::string & controller_type);

  /// Install the registry of controllers compiled into this binary.
  /**
   * `load_controller()` consults it BEFORE pluginlib, and a type it finds is created by its factory
   * and then follows the identical path (`add_controller_impl()`), so lifecycle, interface claiming
   * and every admission check are shared with pluginlib controllers. Installing a registry is
   * optional; without one the behaviour is exactly as before.
   *
   * REGISTRATION TIMING (review item P2-2). The registry is a `std::map` read by `load_controller()`
   * and a `shared_ptr` read by the load path, so "register at start-up" can not stay a comment: this
   * call REFUSES to install or replace a registry once this manager has attempted to load a
   * controller, and the first `load_controller()` freezes the installed registry so `add()`/
   * `add_factory()` throw instead of mutating the map under a concurrent lookup. The rule is
   * therefore: register every compiled-in type, then load. Unloading a controller does not re-open
   * the type set; sealing is one-way and the sealed state is exactly the state lookups are safe in.
   *
   * \return true when the registry was installed, false when the call was refused (logged), in which
   * case the previously installed registry is unchanged.
   */
  CONTROLLER_MANAGER_PUBLIC
  bool set_static_controller_registry(StaticControllerRegistry::SharedPtr registry);

  CONTROLLER_MANAGER_PUBLIC
  std::shared_ptr<StaticControllerRegistry> static_controller_registry() const;

  /// Convenience: register one compiled-in type, installing a registry on first use.
  /**
   * \return false when the registry is already sealed (a controller has been loaded, see
   * `set_static_controller_registry()`); the call is then logged and nothing is registered. A
   * duplicate or empty type string still throws `std::invalid_argument`, exactly like
   * `StaticControllerRegistry::add()`: that is a start-up programming error, not a timing decision.
   */
  template <typename ControllerT>
  bool register_static_controller_type(const std::string & type)
  {
    if (!static_controller_registry_)
    {
      static_controller_registry_ = std::make_shared<StaticControllerRegistry>();
    }
    if (static_controller_registry_->frozen())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Refusing to register compiled-in type '%s': the static controller registry is sealed "
        "because this manager has already loaded a controller. Register every compiled-in type "
        "before the first load_controller() call.",
        type.c_str());
      return false;
    }
    static_controller_registry_->add<ControllerT>(type);
    return true;
  }

  /// load_controller loads a controller by name, the type must be defined in the parameter server.
  /**
   * \param[in] controller_name as a string.
   * \return controller
   * \see Documentation in controller_manager_msgs/LoadController.srv
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceBaseSharedPtr load_controller(
    const std::string & controller_name);

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type unload_controller(const std::string & controller_name);

  CONTROLLER_MANAGER_PUBLIC
  std::vector<ControllerSpec> get_loaded_controllers() const;

  /// Install an opt-in staged execution group from already loaded and configured controllers.
  /**
   * The referenced controllers must be inactive. Topology resolution, string handling,
   * `dynamic_cast` and storage allocation all happen here, outside the real-time loop, and the
   * plan may not change while the group is active. Group members are executed by the two-phase
   * staged contract instead of the native `update()` path; no controller is ever called by both
   * paths in the same cycle.
   *
   * PARTIAL MEMBERSHIP POLICY (review item P1-2), stated rather than implied:
   *   * installing a group REQUIRES atomic activation (`set_atomic_activation(true)`), because a
   *     group is a whole-tree path and only the rollback keeps a FAILED multi-controller switch from
   *     leaving it half-activated. The install is refused with ERROR while atomic activation is off;
   *   * partial membership can still exist legitimately: Humble's lifecycle activates members one at
   *     a time, and any member may be deactivated individually. In that state the group is INERT --
   *     `run()` returns `StagedStatus::inactive`, no member's `update()` is called and no command is
   *     committed. The active members keep their interface claims but publish nothing;
   *   * because "three ACTIVE controllers that drive nothing" must not be silent, the switch that
   *     produced the partial state logs a warning naming the missing members (`switch_controller()`
   *     is the non-real-time thread, so the report does not allocate in the control loop).
   *
   * \param[in] controller_names members of the group, in any declaration order.
   * \param[in] max_age_ns maximum accepted age of a state sample relative to the cycle.
   * \return OK when the plan was built and installed, ERROR otherwise (the reason is logged).
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type set_staged_execution_group(
    const std::vector<std::string> & controller_names, std::int64_t max_age_ns = 0);

  /// Remove the staged group; required before unloading any of its members.
  ///
  /// Same configuration constraint as `set_two_phase_execution`: clearing a group is always accepted
  /// (it only removes a path, and a running cycle holds its own generation), while INSTALLING one is
  /// refused with an error report while a control cycle is in flight.
  CONTROLLER_MANAGER_PUBLIC
  void clear_staged_execution_group();

  CONTROLLER_MANAGER_PUBLIC
  std::shared_ptr<StagedExecutionGroup> staged_execution_group() const;

  /// Opt-in FineMote-style execution: controllers implementing TwoPhaseControllerInterface are run
  /// as two passes over the SAME ordered controller list (reverse for `update_phase`, forward for
  /// `handle_phase`) instead of the native single-pass loop. Legacy and two-phase controllers may
  /// be mixed but their relative order is not guaranteed.
  ///
  /// Enabling is refused, and nothing is changed, when any controller that implements
  /// TwoPhaseControllerInterface cannot be run by this path. The native loop rate-gates controllers
  /// on their own update_rate; the two-phase passes always run every cycle, so a controller whose
  /// update_rate differs from the manager's would silently be called off-rate. The two paths also
  /// own disjoint controller sets, so a controller that is already a staged group member is
  /// rejected rather than executed twice per cycle.
  ///
  /// \return OK when the flag was applied, ERROR when the request was rejected (the offending
  /// controller and the reason are logged).
  ///
  /// MODE, MEMBERS AND PLAN ARE ONE GENERATION (review item D): the flag, the admitted member set and
  /// the staged group are built into one immutable snapshot and published with a single atomic store,
  /// so no cycle can observe "enabled but no entries", or a new group with the previous members.
  ///
  /// WHAT IS STILL NOT ONE GENERATION, and why an in-flight cycle is refused: the controller LIST.
  /// It is upstream's double-buffered publication with its own handshake, and the admission decision
  /// this call makes (which controllers may join, and which are active) is taken against that list.
  /// Installing or EXTENDING an execution path while a cycle is in flight could therefore admit a
  /// set that the concurrently changing list no longer matches. The call is refused with ERROR while
  /// `control_loop_busy()` is true.
  ///
  /// REMOVING a path (`set_two_phase_execution(false)`, `clear_staged_execution_group()`) stays
  /// allowed: a cycle already running holds its own generation, so it finishes with the state it
  /// started from, and the next cycle sees the smaller one. Only additions are refused.
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type set_two_phase_execution(bool enabled);

  /// True while a control cycle is inside `update()` on another thread.
  /**
   * The supported configuration rule, now enforced rather than documented: installing an execution
   * path (this call with `true`, or `set_staged_execution_group`) while this returns true is refused,
   * because the admission decision would race with the controller-list publication. Removing a path
   * is always accepted.
   */
  CONTROLLER_MANAGER_PUBLIC
  bool control_loop_busy() const noexcept;

  CONTROLLER_MANAGER_PUBLIC
  bool two_phase_execution() const;

  /// All-or-nothing activation of a switch's activate set (our extension; default OFF).
  /**
   * ROS 2 Humble activates a request SET best-effort: each controller is claimed and activated on its
   * own, and one that cannot be activated is skipped while the others stay active. That is upstream's
   * documented behaviour and its own tests rely on it (a spawner that starts controllers one at a
   * time must not deactivate the ones already running when a later one fails).
   *
   * A SCHEDULED tree has a stronger requirement: a partially activated tree is not a tree -- the
   * parent-before-child order the execution plan guarantees does not hold for a subset, and the
   * hardware sees commands from half a cascade. With atomic activation ON, a switch that activates
   * several controllers at once undoes the ones IT activated when any of them fails: they are
   * deactivated, their interfaces are released and the hardware mode switch is reported as failed,
   * so the caller observes "nothing activated" instead of a partial tree.
   *
   * Scope, stated exactly: only the controllers activated BY THE FAILING SWITCH are undone.
   * Controllers that were already active before it keep running (deactivating them would be a much
   * larger action than the request), and configuration/unload steps are unaffected.
   *
   * INSTALLING A STAGED EXECUTION GROUP REQUIRES THIS (review item P1-2). A group is a whole-tree
   * path, and the only way a FAILED multi-controller switch cannot leave that tree half-activated is
   * this rollback. The requirement is refused-not-silently-ignored: `set_staged_execution_group()`
   * returns ERROR while this is off, and disabling it is refused while a group is installed. This is
   * our own API, so requiring the opt-in costs no upstream compatibility.
   *
   * \return whether the requested state was applied. `false` means the call was rejected (see
   * `atomic_activation()` for the state that is actually in force).
   */
  CONTROLLER_MANAGER_PUBLIC
  bool set_atomic_activation(bool enabled);

  CONTROLLER_MANAGER_PUBLIC
  bool atomic_activation() const;

  /// Identifier of the currently published execution generation (mode + members + staged group).
  /**
   * It changes exactly when one of those changes, and each publication is a single atomic store, so
   * a client can tell "the same configuration state" from "a new one" without reading three fields.
   */
  CONTROLLER_MANAGER_PUBLIC
  std::uint64_t execution_generation() const noexcept;

  /// One controller that implements `TwoPhaseControllerInterface` but is NOT executing through the
  /// two-phase passes, with the reason it was excluded.
  struct TwoPhaseRejection
  {
    std::string name;
    std::string reason;
  };

  /// The controllers that implement the interface but cannot join the two-phase path, in list order.
  /**
   * Exposes what the rebuild would otherwise only log. A controller that is silently excluded keeps
   * running through the native single-pass loop, which means the parent-before-child order it relies
   * on is NOT the one its neighbours see, so a caller has to be able to observe it (review item E).
   *
   * The verdict is computed on demand from the CURRENT controller list and does NOT depend on the
   * flag: it answers "as the configuration stands, which controllers could not join the two-phase
   * path", so a caller can ask before enabling. Empty means every implementing controller was
   * admitted (or none implements the interface).
   *
   * Returns BY VALUE: the set is derived from the controller list, which the non-real-time thread
   * replaces whenever membership changes, so a reference could race with that replacement. Call it
   * from the non-real-time thread.
   */
  CONTROLLER_MANAGER_PUBLIC
  std::vector<TwoPhaseRejection> two_phase_rejected_controllers() const;

  template <
    typename T, typename std::enable_if<
                  std::is_convertible<T *, controller_interface::ControllerInterfaceBase *>::value,
                  T>::type * = nullptr>
  controller_interface::ControllerInterfaceBaseSharedPtr add_controller(
    std::shared_ptr<T> controller, const std::string & controller_name,
    const std::string & controller_type)
  {
    ControllerSpec controller_spec;
    controller_spec.c = controller;
    controller_spec.info.name = controller_name;
    controller_spec.info.type = controller_type;
    return add_controller_impl(controller_spec);
  }

  /// configure_controller Configure controller by name calling their "configure" method.
  /**
   * \param[in] controller_name as a string.
   * \return configure controller response
   * \see Documentation in controller_manager_msgs/ConfigureController.srv
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type configure_controller(const std::string & controller_name);

  /// switch_controller Stops some controllers and start others.
  /**
   * \param[in] start_controllers is a list of controllers to start
   * \param[in] stop_controllers is a list of controllers to stop
   * \param[in] set level of strictness (BEST_EFFORT or STRICT)
   * \see Documentation in controller_manager_msgs/SwitchController.srv
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type switch_controller(
    const std::vector<std::string> & start_controllers,
    const std::vector<std::string> & stop_controllers, int strictness,
    bool activate_asap = kWaitForAllResources,
    const rclcpp::Duration & timeout = rclcpp::Duration::from_nanoseconds(kInfiniteTimeout));

  /// Read values to state interfaces.
  /**
   * Read current values from hardware to state interfaces.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in]  time    The time at the start of this control loop iteration
   * \param[in]  period  The measured period of the last control loop iteration
   */
  CONTROLLER_MANAGER_PUBLIC
  void read(const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Run update on controllers
  /**
   * Call update of all controllers.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in]  time    The time at the start of this control loop iteration
   * \param[in]  period  The measured period of the last control loop iteration
   */
  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Write values from command interfaces.
  /**
   * Write values from command interface into hardware.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in]  time    The time at the start of this control loop iteration
   * \param[in]  period  The measured period of the last control loop iteration
   */
  CONTROLLER_MANAGER_PUBLIC
  void write(const rclcpp::Time & time, const rclcpp::Duration & period);

  /// Deterministic (real-time safe) callback group, e.g., update function.
  /**
   * Deterministic (real-time safe) callback group for the update function. Default behavior
   * is read hardware, update controller and finally write new values to the hardware.
   */
  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // rclcpp::CallbackGroup::SharedPtr deterministic_callback_group_;

  // Per controller update rate support
  CONTROLLER_MANAGER_PUBLIC
  unsigned int get_update_rate() const;

protected:
  CONTROLLER_MANAGER_PUBLIC
  void init_services();

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::ControllerInterfaceBaseSharedPtr add_controller_impl(
    const ControllerSpec & controller);

  CONTROLLER_MANAGER_PUBLIC
  void manage_switch();

  CONTROLLER_MANAGER_PUBLIC
  void deactivate_controllers();

  /**
   * Switch chained mode for all the controllers with respect to the following cases:
   * - a preceding controller is getting activated --> switch controller to chained mode;
   * - all preceding controllers are deactivated --> switch controller from chained mode.
   *
   * \param[in] chained_mode_switch_list list of controller to switch chained mode.
   * \param[in] to_chained_mode flag if controller should be switched *to* or *from* chained mode.
   */
  CONTROLLER_MANAGER_PUBLIC
  void switch_chained_mode(
    const std::vector<std::string> & chained_mode_switch_list, bool to_chained_mode);

  /// What one activation pass did, so that the caller can undo it when activation is atomic.
  /**
   * The outcome records the SIDE EFFECTS of the pass, not only which controllers became active.
   * Deactivating a controller does not undo a hardware command-mode switch or the publication of a
   * chainable controller's reference interfaces, so both are recorded here and undone explicitly by
   * `rollback_activated_controllers()`. Without that, a rollback could report success while the
   * hardware still sat in a mode that this pass had requested -- a half-done rollback is exactly the
   * state atomic activation exists to prevent.
   */
  struct ActivationOutcome
  {
    bool any_failure = false;
    /// Names activated BY THIS PASS, in the order they became active.
    std::vector<std::string> activated;
    /// Command interfaces claimed by the controllers in `activated` (union, no duplicates).
    /**
     * This pass switched the hardware INTO these interfaces; the rollback has to switch it back OUT
     * of them. Interface names, not indices: the same physical port can be re-mapped by a later
     * controller list, so an index would not survive the switch it describes.
     */
    std::vector<std::string> activated_command_interfaces;
    /// Chainable controllers in `activated` whose reference interfaces this pass published.
    std::vector<std::string> activated_chainable;
    /// Whether a rollback ran at all (for tests and for an honest final report).
    bool rollback_performed = false;
    /// Whether any step of the rollback failed, i.e. the system may still be half-undone.
    bool rollback_failed = false;
  };

  /// Deactivate the controllers this pass activated and undo the side effects it had on hardware.
  /**
   * Runs only for atomic activation (`set_atomic_activation(true)`). Controllers are undone in
   * reverse activation order, so a child that lent the parent's reference interfaces is released
   * before its parent. `outcome.rollback_*` is filled in, and a failed step is logged as a rollback
   * failure rather than as a failed activation: the two need different reactions.
   */
  void rollback_activated_controllers(ActivationOutcome & outcome);

  ActivationOutcome activate_controllers();

  CONTROLLER_MANAGER_PUBLIC
  ActivationOutcome activate_controllers_asap();

  CONTROLLER_MANAGER_PUBLIC
  void list_controllers_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void list_hardware_interfaces_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void load_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::LoadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::LoadController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void configure_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void reload_controller_libraries_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void switch_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void unload_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::UnloadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::UnloadController::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void list_controller_types_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void list_hardware_components_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Response> response);

  CONTROLLER_MANAGER_PUBLIC
  void set_hardware_component_state_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Response> response);

  // Per controller update rate support
  unsigned int update_loop_counter_ = 0;
  unsigned int update_rate_ = 100;
  std::vector<std::vector<std::string>> chained_controllers_configuration_;

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;
  /// Controllers compiled into this binary, addressable by type string (optional).
  std::shared_ptr<StaticControllerRegistry> static_controller_registry_;

private:
  std::vector<std::string> get_controller_names();
  std::pair<std::string, std::string> split_command_interface(
    const std::string & command_interface);
  void subscribe_to_robot_description_topic();

  /**
   * Clear request lists used when switching controllers. The lists are shared between "callback"
   * and "control loop" threads.
   */
  void clear_requests();

  /**
   * If a controller is deactivated all following controllers (if any exist) should be switched
   * 'from' the chained mode.
   *
   * \param[in] controllers list with controllers.
   */
  void propagate_deactivation_of_chained_mode(const std::vector<ControllerSpec> & controllers);

  /// Check if all the following controllers will be in active state and in the chained mode
  /// after controllers' switch.
  /**
   * Check recursively that all following controllers of the @controller_it
   * - are already active,
   * - will not be deactivated,
   * - or will be activated.
   * The following controllers are added to the request to switch in the chained mode or removed
   * from the request to switch from the chained mode.
   *
   * For each controller the whole chain of following controllers is checked.
   *
   * NOTE: The automatically adding of following controller into starting list is not implemented
   * yet.
   *
   * \param[in] controllers list with controllers.
   * \param[in] strictness if value is equal "MANIPULATE_CONTROLLERS_CHAIN" then all following
   * controllers will be automatically added to the activate request list if they are not in the
   * deactivate request.
   * \param[in] controller_it iterator to the controller for which the following controllers are
   * checked.
   *
   * \returns return_type::OK if all following controllers pass the checks, otherwise
   * return_type::ERROR.
   */
  controller_interface::return_type check_following_controllers_for_activate(
    const std::vector<ControllerSpec> & controllers, int strictness,
    const ControllersListIterator controller_it);

  /// Check if all the preceding controllers will be in inactive state after controllers' switch.
  /**
   * Check that all preceding controllers of the @controller_it
   * - are inactive,
   * - will be deactivated,
   * - and will not be activated.
   *
   * NOTE: The automatically adding of preceding controllers into stopping list is not implemented
   * yet.
   *
   * \param[in] controllers list with controllers.
   * \param[in] strictness if value is equal "MANIPULATE_CONTROLLERS_CHAIN" then all preceding
   * controllers will be automatically added to the deactivate request list.
   * \param[in] controller_it iterator to the controller for which the preceding controllers are
   * checked.
   *
   * \returns return_type::OK if all preceding controllers pass the checks, otherwise
   * return_type::ERROR.
   */
  controller_interface::return_type check_preceeding_controllers_for_deactivate(
    const std::vector<ControllerSpec> & controllers, int strictness,
    const ControllersListIterator controller_it);

  /// A method to be used in the std::sort method to sort the controllers to be able to
  /// execute them in a proper order
  /**
   * Compares the controllers ctrl_a and ctrl_b and then returns which comes first in the sequence
   *
   *  @note The following conditions needs to be handled while ordering the controller list
   *  1. The controllers that do not use any state or command interfaces are updated first
   *  2. The controllers that use only the state system interfaces only are updated next
   *  3. The controllers that use any of an another controller's reference interface are updated
   * before the preceding controller
   *  4. The controllers that use the controller's estimated interfaces are updated after the
   * preceding controller
   *  5. The controllers that only use the hardware command interfaces are updated last
   *  6. All inactive controllers go at the end of the list
   *
   * \param[in] controllers list of controllers to compare their names to interface's prefix.
   *
   * @return true, if ctrl_a needs to execute first, else false
   */
  bool controller_sorting(
    const ControllerSpec & ctrl_a, const ControllerSpec & ctrl_b,
    const std::vector<controller_manager::ControllerSpec> & controllers);

  /**
   * @brief determine_controller_node_options - A method that retrieves the controller defined node
   * options and adapts them, based on if there is a params file to be loaded or the use_sim_time
   * needs to be set
   * @param controller - controller info
   * @return The node options that will be set to the controller LifeCycleNode
   */
  rclcpp::NodeOptions determine_controller_node_options(const ControllerSpec & controller) const;

  std::shared_ptr<rclcpp::Executor> executor_;

  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ControllerInterface>> loader_;
  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ChainableControllerInterface>>
    chainable_loader_;

  /// Best effort (non real-time safe) callback group, e.g., service callbacks.
  /**
   * Best effort (non real-time safe) callback group for callbacks that can possibly break
   * real-time requirements, for example, service callbacks.
   */
  rclcpp::CallbackGroup::SharedPtr best_effort_callback_group_;

  /**
   * The RTControllerListWrapper class wraps a double-buffered list of controllers
   * to avoid needing to lock the real-time thread when switching controllers in
   * the non-real-time thread.
   *
   * There's always an "updated" list and an "outdated" one
   * There's always an "used by rt" list and an "unused by rt" list
   *
   * The updated state changes on the switch_updated_list()
   * The rt usage state changes on the update_and_get_used_by_rt_list()
   */
  class RTControllerListWrapper
  {
    // *INDENT-OFF*
  public:
    // *INDENT-ON*
    /// update_and_get_used_by_rt_list Makes the "updated" list the "used by rt" list
    /**
     * \warning Should only be called by the RT thread, no one should modify the
     * updated list while it's being used
     * \return reference to the updated list
     */
    std::vector<ControllerSpec> & update_and_get_used_by_rt_list();

    /**
     * get_unused_list Waits until the "outdated" and "unused by rt"
     * lists match and returns a reference to it
     * This referenced list can be modified safely until switch_updated_controller_list()
     * is called, at this point the RT thread may start using it at any time
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by
     * rt list
     */
    std::vector<ControllerSpec> & get_unused_list(
      const std::lock_guard<std::recursive_mutex> & guard);

    /// get_updated_list Returns a const reference to the most updated list.
    /**
     * \warning May or may not being used by the realtime thread, read-only reference for safety
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by
     * rt list
     */
    const std::vector<ControllerSpec> & get_updated_list(
      const std::lock_guard<std::recursive_mutex> & guard) const;

    /**
     * switch_updated_list Switches the "updated" and "outdated" lists, and waits
     *  until the RT thread is using the new "updated" list.
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by
     * rt list
     */
    void switch_updated_list(const std::lock_guard<std::recursive_mutex> & guard);

    // Mutex protecting the controllers list
    // must be acquired before using any list other than the "used by rt"
    mutable std::recursive_mutex controllers_lock_;

    // *INDENT-OFF*
  private:
    // *INDENT-ON*
    /// get_other_list get the list not pointed by index
    /**
     * \param[in] index int
     */
    int get_other_list(int index) const;

    void wait_until_rt_not_using(
      int index, std::chrono::microseconds sleep_delay = std::chrono::microseconds(200)) const;

    std::vector<ControllerSpec> controllers_lists_[2];
    /// The index of the controller list with the most updated information.
    /**
     * ATOMIC, and with release/acquire on the publish/observe pair: the idle thread fills the unused
     * list and then publishes it by storing this index, and the control loop reads the index before
     * reading the list, so the list's contents must be visible before the index is. The plain `int`
     * version is reported as a data race by ThreadSanitizer (review item D: the controller-list
     * publication was never covered by the old harness).
     */
    std::atomic<int> updated_controllers_index_{0};
    /// The index of the controllers list being used in the real-time thread. Atomic for the same
    /// reason: the control loop stores it, and `switch_updated_list()` polls it from the idle thread.
    std::atomic<int> used_by_realtime_controllers_index_{-1};
  };

  std::unique_ptr<rclcpp::PreShutdownCallbackHandle> preshutdown_cb_handle_{nullptr};
  RTControllerListWrapper rt_controllers_wrapper_;
  /// Opt-in two-phase (FineMote Update/Handle) execution. The membership vector is rebuilt only
  /// outside the control loop and published atomically; the passes do one pointer lookup per
  /// controller and never allocate.
  static constexpr std::size_t no_two_phase = std::numeric_limits<std::size_t>::max();
  struct TwoPhaseEntry
  {
    const controller_interface::ControllerInterfaceBase * base;
    hierarchical_control::TwoPhaseControllerInterface * instance;
  };
  /// Why a controller that implements TwoPhaseControllerInterface may not join the two-phase path.
  enum class TwoPhaseAdmission
  {
    accepted,
    /// Already a member of the installed staged execution group, which would execute it too.
    already_staged,
    /// Has a nonzero update_rate that the two-phase passes cannot honour.
    unsupported_update_rate,
    /// Claims a reference interface owned by a loaded controller that is NOT a two-phase member (or
    /// is such a controller claimed by a non-member). The two ends would then be ordered by
    /// DIFFERENT schedules -- pass 2 runs after the native loop -- so the edge silently degrades to
    /// the previous cycle. Review item E rejects the whole admission instead.
    cross_mode_dependency,
    /// The controller manager's controller list puts a reference edge's two ends in the wrong
    /// order, so `update_phase` (backward walk) and `handle_phase` (forward walk) would both visit
    /// the parent before the child (or the child before the parent) and the edge would silently use
    /// the previous cycle's value. Measured case: upstream `controller_sorting()` places a chainable
    /// controller that claims NO command interface BEFORE the parent that claims its reference.
    unschedulable_order,
    /// Two names in the controller list refer to ONE controller object. `add_controller()` only
    /// rejects duplicate NAMES, so this is accepted upstream, and then a pass advances that single
    /// object once per name in the same cycle (measured: 2 `update_phase` + 2 `handle_phase` calls
    /// per cycle). The staged/library path is protected by the kernel's own instance check; the
    /// two-phase path does not go through the kernel, so it checks here.
    duplicate_instance
  };
  /// Opt-in all-or-nothing activation (our extension, default false: upstream activates a request
  /// set best-effort and its tests rely on that). Read on the real-time thread, written by setters.
  std::atomic<bool> atomic_activation_{false};
  /// Control cycles currently inside `update()` (0 or 1 in practice). Written by the control loop,
  /// read by the configuration setters to refuse installing a path mid-cycle.
  std::atomic<int> cycles_in_flight_{0};

  /// RAII marker so every `return` inside `update()` clears the in-flight count.
  class CycleGuard
  {
  public:
    explicit CycleGuard(std::atomic<int> & counter) noexcept : counter_(counter)
    {
      counter_.fetch_add(1, std::memory_order_relaxed);
    }
    ~CycleGuard() {counter_.fetch_sub(1, std::memory_order_relaxed);}
    CycleGuard(const CycleGuard &) = delete;
    CycleGuard & operator=(const CycleGuard &) = delete;

  private:
    std::atomic<int> & counter_;
  };
  /// Read by `update()` every cycle and written by the non-real-time setter, so it is atomic.
  /// An earlier revision used a plain bool, which is a data race on its own -- making the entry
  /// SNAPSHOT atomic does not make the flag that gates it atomic (review item D).
  std::atomic<bool> two_phase_enabled_{false};
  /// Immutable once published, so a reader in `update()` may dereference it without locking.
  /// `mutable` because the atomic accessors take a non-const pointer.
  mutable std::shared_ptr<const std::vector<TwoPhaseEntry>> two_phase_entries_;
  /// Human-readable reason, with the offending owner name where one is relevant.
  std::string two_phase_admission_reason(TwoPhaseAdmission admission, const std::string & detail)
    const;
  TwoPhaseAdmission two_phase_admission(
    const ControllerSpec & controller,
    const std::shared_ptr<StagedExecutionGroup> & staged_for_admission) const noexcept;
  /// The command interfaces a controller claims, for the scheduling checks.
  /**
   * For an ACTIVE controller this is the set it actually holds a loan for
   * (`ControllerSpec::info::claimed_interfaces`, kept up to date by `switch_controller()`), because
   * that is what it really writes; its declaration is used only when the snapshot is empty. Reading
   * the declaration for an active controller would invent edges, since a controller cannot claim an
   * interface that was imported after its activation -- a controller declaring `ALL` is the visible
   * case. For a configured-but-INACTIVE controller the declaration is the right answer: it is what
   * the controller would claim once activated, and that is the prospect the checks have to judge.
   *
   * Reading the declaration is only defined once the controller is configured (some controllers
   * throw before that), so an unconfigured controller reports nothing.
   */
  std::vector<std::string> claimed_command_interfaces(const ControllerSpec & controller) const;
  /// Every controller that implements the interface but is not admitted, in list order.
  /**
   * `active_mask` restricts the verdict to the controllers that will actually run: index i is judged
   * only when `(*active_mask)[i] != 0`. `nullptr` judges the WHOLE list, which is what the enable and
   * configure paths want, because the flag describes the mode the configured set is in.
   *
   * A mask (rather than a "currently active" flag) is what lets `switch_controller()` judge the
   * PROSPECTIVE active set before it applies anything, so a switch that would create a scheduling
   * violation is refused instead of activating the controllers and then reporting an error.
   */
  std::vector<TwoPhaseRejection> two_phase_rejections(
    const std::vector<ControllerSpec> & controllers, const std::vector<char> * active_mask,
    const std::shared_ptr<StagedExecutionGroup> & staged_for_admission) const;
  /// The mask accepted by `two_phase_rejections`: 1 where `is_controller_active()` holds.
  std::vector<char> controller_active_mask(const std::vector<ControllerSpec> & controllers) const;
  /// ONE immutable snapshot of everything `update()` needs to know about HOW to execute.
  /**
   * Review item D's second half. The two-phase flag, the two-phase member set and the staged group
   * used to be three separately published values with three atomic loads per cycle, so a cycle could
   * observe a mixture: the flag from one configuration state and the member set from another (for
   * example "enabled" while the entries were still the previous, or empty, set). They are now built
   * into one object and published with ONE atomic store, so every cycle sees a coherent generation.
   *
   * What is deliberately NOT in here: the controller LIST. It is upstream's own double-buffered
   * publication (`RTControllerListWrapper`) with its own sleep-based handshake; folding it in would
   * mean replacing that mechanism. `controllers_version` records which list the snapshot was built
   * for, so a mismatch is at least observable in diagnostics.
   *
   * `mutable` because the atomic accessors take a non-const pointer.
   */
  struct ExecutionGeneration
  {
    bool two_phase_enabled = false;
    std::shared_ptr<const std::vector<TwoPhaseEntry>> two_phase_entries;
    std::shared_ptr<StagedExecutionGroup> staged_group;
    /// Bumped on every publication; lets a caller (and a test) name the state it saw.
    std::uint64_t id = 0;
  };
  mutable std::shared_ptr<const ExecutionGeneration> generation_;

  /// Publish a complete new generation with ONE atomic store.
  /**
   * Every publisher states the WHOLE new state, so there is no window in which one field has been
   * updated and another has not. Called only from the non-real-time thread.
   */
  void publish_generation(
    bool two_phase_enabled, std::shared_ptr<const std::vector<TwoPhaseEntry>> entries,
    std::shared_ptr<StagedExecutionGroup> staged_group);
  /// The published generation (never null after construction).
  std::shared_ptr<const ExecutionGeneration> current_generation() const noexcept;

  /// The published entry set, BY VALUE: the caller must own the snapshot for as long as it uses it.
  /**
   * An earlier revision returned a reference into a snapshot held only by a local `shared_ptr`, so
   * a concurrent republish could destroy the vector while the returned reference was still in use
   * (review item D). Returning the `shared_ptr` makes the caller share ownership and is the only
   * shape that is safe against a concurrent retire.
   */
  std::shared_ptr<const std::vector<TwoPhaseEntry>> two_phase_entries() const noexcept;
  /// Build the two-phase member set for a given staged group. Never locks; non-real-time only.
  std::shared_ptr<const std::vector<TwoPhaseEntry>> build_two_phase_entries(
    const std::vector<ControllerSpec> & controllers, bool admission_enabled,
    const std::shared_ptr<StagedExecutionGroup> & staged_for_admission) const;
  /// Build membership from a controller list the caller already owns, then publish a new generation.
  /// Never locks: it is only called from the non-real-time thread, which must not contend with
  /// switch_controller() while that holds the controllers lock waiting for the control loop.
  void rebuild_two_phase_entries(const std::vector<ControllerSpec> & controllers);
  /// Same, but with admission applied regardless of the current flag.
  /**
   * `set_two_phase_execution(true)` needs this: the entry set must be published BEFORE the flag
   * becomes true, otherwise a control cycle can observe "enabled but no entries" and hand a member
   * to the native loop for that cycle.
   */
  void rebuild_two_phase_entries(
    const std::vector<ControllerSpec> & controllers, bool admission_enabled);
  void refresh_two_phase_controllers();
  std::size_t two_phase_index(
    const std::vector<TwoPhaseEntry> & entries,
    const controller_interface::ControllerInterfaceBase * controller) const noexcept;
  /// mutex copied from ROS1 Control, protects service callbacks
  /// not needed if we're guaranteed that the callbacks don't come from multiple threads
  std::mutex services_lock_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllerTypes>::SharedPtr
    list_controller_types_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ConfigureController>::SharedPtr
    configure_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ReloadControllerLibraries>::SharedPtr
    reload_controller_libraries_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::UnloadController>::SharedPtr
    unload_controller_service_;

  rclcpp::Service<controller_manager_msgs::srv::ListHardwareComponents>::SharedPtr
    list_hardware_components_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListHardwareInterfaces>::SharedPtr
    list_hardware_interfaces_service_;
  rclcpp::Service<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr
    set_hardware_component_state_service_;

  std::vector<std::string> activate_request_, deactivate_request_;
  std::vector<std::string> to_chained_mode_request_, from_chained_mode_request_;
  std::vector<std::string> activate_command_interface_request_,
    deactivate_command_interface_request_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;

  struct SwitchParams
  {
    void reset()
    {
      do_switch.store(false, std::memory_order_relaxed);
      started.store(false, std::memory_order_relaxed);
      strictness.store(0, std::memory_order_relaxed);
      activate_asap.store(false, std::memory_order_relaxed);
    }

    /// Cross-thread handshake flags, ATOMIC because they are written by the idle thread
    /// (`switch_controller()`) and read by the control loop (`update()` / `manage_switch()`).
    /// An instrumented build of this file under ThreadSanitizer reports the plain versions as a data
    /// race, which is exactly the publication protocol review item D was about; the two-phase flag
    /// was made atomic earlier, and these are the same class of field.
    ///
    /// `do_switch` carries the handshake with release/acquire, so the options below it are ordered
    /// without needing atomics of their own; they are atomic anyway so that any reader (a log, a
    /// status query) cannot race either.
    std::atomic<bool> do_switch{false};
    std::atomic<bool> started{false};

    // Switch options
    std::atomic<int> strictness{0};
    std::atomic<bool> activate_asap{false};
    std::chrono::nanoseconds timeout{0};

    // conditional variable and mutex to wait for the switch to complete
    std::condition_variable cv;
    std::mutex mutex;
  };

  SwitchParams switch_params_;
};

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
