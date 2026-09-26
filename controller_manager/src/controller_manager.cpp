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

#include "controller_manager/controller_manager.hpp"

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "controller_manager_msgs/msg/hardware_component_state.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rcl/arguments.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace  // utility
{
static constexpr const char * kControllerInterfaceNamespace = "controller_interface";
static constexpr const char * kControllerInterfaceClassName =
  "controller_interface::ControllerInterface";
static constexpr const char * kChainableControllerInterfaceClassName =
  "controller_interface::ChainableControllerInterface";

// Changed services history QoS to keep all so we don't lose any client service calls
static const rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

inline bool is_controller_unconfigured(
  const controller_interface::ControllerInterfaceBase & controller)
{
  return controller.get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
}

inline bool is_controller_inactive(const controller_interface::ControllerInterfaceBase & controller)
{
  return controller.get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

inline bool is_controller_inactive(
  const controller_interface::ControllerInterfaceBaseSharedPtr & controller)
{
  return is_controller_inactive(*controller);
}

inline bool is_controller_active(const controller_interface::ControllerInterfaceBase & controller)
{
  return controller.get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

inline bool is_controller_active(
  const controller_interface::ControllerInterfaceBaseSharedPtr & controller)
{
  return is_controller_active(*controller);
}

bool controller_name_compare(const controller_manager::ControllerSpec & a, const std::string & name)
{
  return a.info.name == name;
}

/// Checks if a command interface belongs to a controller based on its prefix.
/**
 * A command interface can be provided by a controller in which case is called "reference"
 * interface.
 * This means that the @interface_name starts with the name of a controller.
 *
 * \param[in] interface_name to be found in the map.
 * \param[in] controllers list of controllers to compare their names to interface's prefix.
 * \param[out] following_controller_it iterator to the following controller that reference interface
 * @interface_name belongs to.
 * \return true if interface has a controller name as prefix, false otherwise.
 */
bool command_interface_is_reference_interface_of_controller(
  const std::string interface_name,
  const std::vector<controller_manager::ControllerSpec> & controllers,
  controller_manager::ControllersListIterator & following_controller_it)
{
  auto split_pos = interface_name.find_first_of('/');
  if (split_pos == std::string::npos)  // '/' exist in the string (should be always false)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("ControllerManager::utils"),
      "Character '/', was not find in the interface name '%s'. This should never happen. "
      "Stop the controller manager immediately and restart it.",
      interface_name.c_str());
    throw std::runtime_error("Mismatched interface name. See the FATAL message above.");
  }

  auto interface_prefix = interface_name.substr(0, split_pos);
  following_controller_it = std::find_if(
    controllers.begin(), controllers.end(),
    std::bind(controller_name_compare, std::placeholders::_1, interface_prefix));

  RCLCPP_DEBUG(
    rclcpp::get_logger("ControllerManager::utils"),
    "Deduced interface prefix '%s' - searching for the controller with the same name.",
    interface_prefix.c_str());

  if (following_controller_it == controllers.end())
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ControllerManager::utils"),
      "Required command interface '%s' with prefix '%s' is not reference interface.",
      interface_name.c_str(), interface_prefix.c_str());

    return false;
  }
  return true;
}

/**
 * A method to retrieve the names of all it's following controllers given a controller name
 * For instance, for the following case
 * A -> B -> C -> D
 * When called with B, returns C and D
 * NOTE: A -> B signifies that the controller A is utilizing the reference interfaces exported from
 * the controller B (or) the controller B is utilizing the expected interfaces exported from the
 * controller A
 *
 * @param controller_name - Name of the controller for checking the tree
 * \param[in] controllers list of controllers to compare their names to interface's prefix.
 * @return list of controllers that are following the given controller in a chain. If none, return
 * empty.
 */
std::vector<std::string> get_following_controller_names(
  const std::string controller_name,
  const std::vector<controller_manager::ControllerSpec> & controllers)
{
  std::vector<std::string> following_controllers;
  auto controller_it = std::find_if(
    controllers.begin(), controllers.end(),
    std::bind(controller_name_compare, std::placeholders::_1, controller_name));
  if (controller_it == controllers.end())
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ControllerManager::utils"),
      "Required controller : '%s' is not found in the controller list ", controller_name.c_str());

    return following_controllers;
  }
  // If the controller is not configured, return empty
  if (!(is_controller_active(controller_it->c) || is_controller_inactive(controller_it->c)))
  {
    return following_controllers;
  }
  const auto cmd_itfs = controller_it->c->command_interface_configuration().names;
  for (const auto & itf : cmd_itfs)
  {
    controller_manager::ControllersListIterator ctrl_it;
    if (command_interface_is_reference_interface_of_controller(itf, controllers, ctrl_it))
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("ControllerManager::utils"),
        "The interface is a reference interface of controller : %s", ctrl_it->info.name.c_str());
      following_controllers.push_back(ctrl_it->info.name);
      const std::vector<std::string> ctrl_names =
        get_following_controller_names(ctrl_it->info.name, controllers);
      for (const std::string & controller : ctrl_names)
      {
        if (
          std::find(following_controllers.begin(), following_controllers.end(), controller) ==
          following_controllers.end())
        {
          // Only add to the list if it doesn't exist
          following_controllers.push_back(controller);
        }
      }
    }
  }
  return following_controllers;
}

/**
 * A method to retrieve the names of all it's preceding controllers given a controller name
 * For instance, for the following case
 * A -> B -> C -> D
 * When called with C, returns A and B
 * NOTE: A -> B signifies that the controller A is utilizing the reference interfaces exported from
 * the controller B (or) the controller B is utilizing the expected interfaces exported from the
 * controller A
 *
 * @param controller_name - Name of the controller for checking the tree
 * \param[in] controllers list of controllers to compare their names to interface's prefix.
 * @return list of controllers that are preceding the given controller in a chain. If none, return
 * empty.
 */
std::vector<std::string> get_preceding_controller_names(
  const std::string controller_name,
  const std::vector<controller_manager::ControllerSpec> & controllers)
{
  std::vector<std::string> preceding_controllers;
  auto controller_it = std::find_if(
    controllers.begin(), controllers.end(),
    std::bind(controller_name_compare, std::placeholders::_1, controller_name));
  if (controller_it == controllers.end())
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ControllerManager::utils"),
      "Required controller : '%s' is not found in the controller list ", controller_name.c_str());
    return preceding_controllers;
  }
  for (const auto & ctrl : controllers)
  {
    // If the controller is not configured, then continue
    if (!(is_controller_active(ctrl.c) || is_controller_inactive(ctrl.c)))
    {
      continue;
    }
    auto cmd_itfs = ctrl.c->command_interface_configuration().names;
    for (const auto & itf : cmd_itfs)
    {
      auto split_pos = itf.find_first_of('/');
      if ((split_pos != std::string::npos) && (itf.substr(0, split_pos) == controller_name))
      {
        preceding_controllers.push_back(ctrl.info.name);
        auto ctrl_names = get_preceding_controller_names(ctrl.info.name, controllers);
        for (const std::string & controller : ctrl_names)
        {
          if (
            std::find(preceding_controllers.begin(), preceding_controllers.end(), controller) ==
            preceding_controllers.end())
          {
            // Only add to the list if it doesn't exist
            preceding_controllers.push_back(controller);
          }
        }
      }
    }
  }
  return preceding_controllers;
}

}  // namespace

namespace controller_manager
{
rclcpp::NodeOptions get_cm_node_options()
{
  rclcpp::NodeOptions node_options;
  // Required for getting types of controllers to be loaded via service call
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  return node_options;
}

ControllerManager::ControllerManager(
  std::shared_ptr<rclcpp::Executor> executor, const std::string & manager_node_name,
  const std::string & namespace_, const rclcpp::NodeOptions & options)
: rclcpp::Node(manager_node_name, namespace_, options),
  resource_manager_(std::make_unique<hardware_interface::ResourceManager>()),
  executor_(executor),
  loader_(
    std::make_shared<pluginlib::ClassLoader<controller_interface::ControllerInterface>>(
      kControllerInterfaceNamespace, kControllerInterfaceClassName)),
  chainable_loader_(
    std::make_shared<pluginlib::ClassLoader<controller_interface::ChainableControllerInterface>>(
      kControllerInterfaceNamespace, kChainableControllerInterfaceClassName))
{
  if (!get_parameter("update_rate", update_rate_))
  {
    RCLCPP_WARN(get_logger(), "'update_rate' parameter not set, using default value.");
  }

  // Opt-in FineMote-style execution, configurable from the controller_manager YAML section.
  // Membership is resolved lazily after the first switch, so controllers may be loaded later.
  bool two_phase = false;
  if (get_parameter("two_phase_execution", two_phase))
  {
    publish_generation(two_phase, nullptr, nullptr);
    RCLCPP_INFO(get_logger(), "Two-phase execution requested by parameter: %s",
      two_phase ? "enabled" : "disabled");
  }

  // Our extension; see set_atomic_activation(). Off by default, which keeps upstream semantics.
  bool atomic_activation = false;
  if (get_parameter("atomic_activation", atomic_activation))
  {
    atomic_activation_.store(atomic_activation, std::memory_order_relaxed);
    RCLCPP_INFO(
      get_logger(), "Atomic activation requested by parameter: %s",
      atomic_activation ? "enabled" : "disabled");
  }

  std::string robot_description = "";
  get_parameter("robot_description", robot_description);
  if (robot_description.empty())
  {
    subscribe_to_robot_description_topic();
  }
  else
  {
    RCLCPP_WARN(
      get_logger(),
      "[Deprecated] Passing the robot description parameter directly to the control_manager node "
      "is deprecated. Use '~/robot_description' topic from 'robot_state_publisher' instead.");
    init_resource_manager(robot_description);
    init_services();
  }
}

ControllerManager::ControllerManager(
  std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
  std::shared_ptr<rclcpp::Executor> executor, const std::string & manager_node_name,
  const std::string & namespace_, const rclcpp::NodeOptions & options)
: rclcpp::Node(manager_node_name, namespace_, options),
  resource_manager_(std::move(resource_manager)),
  executor_(executor),
  loader_(
    std::make_shared<pluginlib::ClassLoader<controller_interface::ControllerInterface>>(
      kControllerInterfaceNamespace, kControllerInterfaceClassName)),
  chainable_loader_(
    std::make_shared<pluginlib::ClassLoader<controller_interface::ChainableControllerInterface>>(
      kControllerInterfaceNamespace, kChainableControllerInterfaceClassName))
{
  if (!get_parameter("update_rate", update_rate_))
  {
    RCLCPP_WARN(get_logger(), "'update_rate' parameter not set, using default value.");
  }

  // Opt-in FineMote-style execution, configurable from the controller_manager YAML section.
  // Membership is resolved lazily after the first switch, so controllers may be loaded later.
  bool two_phase = false;
  if (get_parameter("two_phase_execution", two_phase))
  {
    publish_generation(two_phase, nullptr, nullptr);
    RCLCPP_INFO(get_logger(), "Two-phase execution requested by parameter: %s",
      two_phase ? "enabled" : "disabled");
  }

  // Our extension; see set_atomic_activation(). Off by default, which keeps upstream semantics.
  bool atomic_activation = false;
  if (get_parameter("atomic_activation", atomic_activation))
  {
    atomic_activation_.store(atomic_activation, std::memory_order_relaxed);
    RCLCPP_INFO(
      get_logger(), "Atomic activation requested by parameter: %s",
      atomic_activation ? "enabled" : "disabled");
  }

  if (!resource_manager_->is_urdf_already_loaded())
  {
    subscribe_to_robot_description_topic();
  }
  else
  {
    init_services();
  }
}

ControllerManager::~ControllerManager()
{
  if (preshutdown_cb_handle_)
  {
    rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();
    context->remove_pre_shutdown_callback(*(preshutdown_cb_handle_.get()));
    preshutdown_cb_handle_.reset();
  }
}

bool ControllerManager::shutdown_controllers()
{
  RCLCPP_INFO(get_logger(), "Shutting down all controllers in the controller manager.");
  // Shutdown all controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  std::vector<ControllerSpec> controllers_list = rt_controllers_wrapper_.get_updated_list(guard);
  bool ctrls_shutdown_status = true;
  for (auto & controller : controllers_list)
  {
    if (is_controller_active(controller.c))
    {
      RCLCPP_INFO(
        get_logger(), "Deactivating controller '%s'", controller.c->get_node()->get_name());
      controller.c->get_node()->deactivate();
      controller.c->release_interfaces();
    }
    if (is_controller_inactive(*controller.c) || is_controller_unconfigured(*controller.c))
    {
      RCLCPP_INFO(
        get_logger(), "Shutting down controller '%s'", controller.c->get_node()->get_name());
      controller.c->get_node()->shutdown();
    }
    ctrls_shutdown_status &=
      (controller.c->get_node()->get_current_state().id() ==
       lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
    executor_->remove_node(controller.c->get_node()->get_node_base_interface());
  }
  return ctrls_shutdown_status;
}

void ControllerManager::subscribe_to_robot_description_topic()
{
  // set QoS to transient local to get messages that have already been published
  // (if robot state publisher starts before controller manager)
  RCLCPP_INFO(
    get_logger(), "Subscribing to '~/robot_description' topic for robot description file.");
  robot_description_subscription_ = create_subscription<std_msgs::msg::String>(
    "~/robot_description", rclcpp::QoS(1).transient_local(),
    std::bind(&ControllerManager::robot_description_callback, this, std::placeholders::_1));
}

void ControllerManager::robot_description_callback(const std_msgs::msg::String & robot_description)
{
  RCLCPP_INFO(get_logger(), "Received robot description file.");
  RCLCPP_DEBUG(
    get_logger(), "'Content of robot description file: %s", robot_description.data.c_str());
  // TODO(mamueluth): errors should probably be caught since we don't want controller_manager node
  // to die if a non valid urdf is passed. However, should maybe be fine tuned.
  try
  {
    if (resource_manager_->is_urdf_already_loaded())
    {
      RCLCPP_WARN(
        get_logger(),
        "ResourceManager has already loaded an urdf file. Ignoring attempt to reload a robot "
        "description file.");
      return;
    }
    init_resource_manager(robot_description.data.c_str());
    init_services();
  }
  catch (std::runtime_error & e)
  {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "The published robot description file (urdf) seems not to be genuine. The following error "
      "was caught:"
        << e.what());
  }
}

void ControllerManager::init_resource_manager(const std::string & robot_description)
{
  // TODO(destogl): manage this when there is an error - CM should not die because URDF is wrong...
  resource_manager_->load_urdf(robot_description);

  // Get all components and if they are not defined in parameters activate them automatically
  auto components_to_activate = resource_manager_->get_components_status();

  using lifecycle_msgs::msg::State;

  auto set_components_to_state =
    [&](const std::string & parameter_name, rclcpp_lifecycle::State state)
  {
    std::vector<std::string> components_to_set = std::vector<std::string>({});
    if (get_parameter(parameter_name, components_to_set))
    {
      for (const auto & component : components_to_set)
      {
        if (component.empty())
        {
          continue;
        }
        if (components_to_activate.find(component) == components_to_activate.end())
        {
          RCLCPP_WARN(
            get_logger(), "Hardware component '%s' is unknown, therefore not set in '%s' state.",
            component.c_str(), state.label().c_str());
        }
        else
        {
          RCLCPP_INFO(
            get_logger(), "Setting component '%s' to '%s' state.", component.c_str(),
            state.label().c_str());
          if (
            resource_manager_->set_component_state(component, state) ==
            hardware_interface::return_type::ERROR)
          {
            throw std::runtime_error(
              "Failed to set the initial state of the component : " + component + " to " +
              state.label());
          }
          components_to_activate.erase(component);
        }
      }
    }
  };

  // unconfigured (loaded only)
  set_components_to_state(
    "hardware_components_initial_state.unconfigured",
    rclcpp_lifecycle::State(
      State::PRIMARY_STATE_UNCONFIGURED, hardware_interface::lifecycle_state_names::UNCONFIGURED));

  // inactive (configured)
  // BEGIN: Keep old functionality on for backwards compatibility
  std::vector<std::string> configure_components_on_start = std::vector<std::string>({});
  get_parameter("configure_components_on_start", configure_components_on_start);
  if (!configure_components_on_start.empty())
  {
    RCLCPP_WARN(
      get_logger(),
      "[Deprecated]: Parameter 'configure_components_on_start' is deprecated. "
      "Use 'hardware_interface_state_after_start.inactive' instead, to set component's initial "
      "state to 'inactive'. Don't use this parameters in combination with the new "
      "'hardware_interface_state_after_start' parameter structure.");
    set_components_to_state(
      "configure_components_on_start",
      rclcpp_lifecycle::State(
        State::PRIMARY_STATE_INACTIVE, hardware_interface::lifecycle_state_names::INACTIVE));
  }
  // END: Keep old functionality on humble backwards compatibility (Remove at the end of 2023)
  else
  {
    set_components_to_state(
      "hardware_components_initial_state.inactive",
      rclcpp_lifecycle::State(
        State::PRIMARY_STATE_INACTIVE, hardware_interface::lifecycle_state_names::INACTIVE));
  }

  // BEGIN: Keep old functionality on for backwards compatibility
  std::vector<std::string> activate_components_on_start = std::vector<std::string>({});
  get_parameter("activate_components_on_start", activate_components_on_start);
  rclcpp_lifecycle::State active_state(
    State::PRIMARY_STATE_ACTIVE, hardware_interface::lifecycle_state_names::ACTIVE);
  if (!activate_components_on_start.empty())
  {
    RCLCPP_WARN(
      get_logger(),
      "[Deprecated]: Parameter 'activate_components_on_start' is deprecated. "
      "Components are activated per default. Don't use this parameters in combination with the new "
      "'hardware_components_initial_state' parameter structure.");
    for (const auto & component : activate_components_on_start)
    {
      resource_manager_->set_component_state(component, active_state);
    }
  }
  // END: Keep old functionality on humble for backwards compatibility (Remove at the end of 2023)
  else
  {
    // activate all other components
    for (const auto & [component, state] : components_to_activate)
    {
      if (
        resource_manager_->set_component_state(component, active_state) ==
        hardware_interface::return_type::ERROR)
      {
        throw std::runtime_error(
          "Failed to set the initial state of the component : " + component + " to " +
          active_state.label());
      }
    }
  }
}

void ControllerManager::init_services()
{
  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // deterministic_callback_group_ = create_callback_group(
  //   rclcpp::CallbackGroupType::MutuallyExclusive);
  best_effort_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  using namespace std::placeholders;
  list_controllers_service_ = create_service<controller_manager_msgs::srv::ListControllers>(
    "~/list_controllers", std::bind(&ControllerManager::list_controllers_srv_cb, this, _1, _2),
    rmw_qos_profile_services_hist_keep_all, best_effort_callback_group_);
  list_controller_types_service_ =
    create_service<controller_manager_msgs::srv::ListControllerTypes>(
      "~/list_controller_types",
      std::bind(&ControllerManager::list_controller_types_srv_cb, this, _1, _2),
      rmw_qos_profile_services_hist_keep_all, best_effort_callback_group_);
  load_controller_service_ = create_service<controller_manager_msgs::srv::LoadController>(
    "~/load_controller", std::bind(&ControllerManager::load_controller_service_cb, this, _1, _2),
    rmw_qos_profile_services_hist_keep_all, best_effort_callback_group_);
  configure_controller_service_ = create_service<controller_manager_msgs::srv::ConfigureController>(
    "~/configure_controller",
    std::bind(&ControllerManager::configure_controller_service_cb, this, _1, _2),
    rmw_qos_profile_services_hist_keep_all, best_effort_callback_group_);
  reload_controller_libraries_service_ =
    create_service<controller_manager_msgs::srv::ReloadControllerLibraries>(
      "~/reload_controller_libraries",
      std::bind(&ControllerManager::reload_controller_libraries_service_cb, this, _1, _2),
      rmw_qos_profile_services_hist_keep_all, best_effort_callback_group_);
  switch_controller_service_ = create_service<controller_manager_msgs::srv::SwitchController>(
    "~/switch_controller",
    std::bind(&ControllerManager::switch_controller_service_cb, this, _1, _2),
    rmw_qos_profile_services_hist_keep_all, best_effort_callback_group_);
  unload_controller_service_ = create_service<controller_manager_msgs::srv::UnloadController>(
    "~/unload_controller",
    std::bind(&ControllerManager::unload_controller_service_cb, this, _1, _2),
    rmw_qos_profile_services_hist_keep_all, best_effort_callback_group_);
  list_hardware_components_service_ =
    create_service<controller_manager_msgs::srv::ListHardwareComponents>(
      "~/list_hardware_components",
      std::bind(&ControllerManager::list_hardware_components_srv_cb, this, _1, _2),
      rmw_qos_profile_services_hist_keep_all, best_effort_callback_group_);
  list_hardware_interfaces_service_ =
    create_service<controller_manager_msgs::srv::ListHardwareInterfaces>(
      "~/list_hardware_interfaces",
      std::bind(&ControllerManager::list_hardware_interfaces_srv_cb, this, _1, _2),
      rmw_qos_profile_services_hist_keep_all, best_effort_callback_group_);
  set_hardware_component_state_service_ =
    create_service<controller_manager_msgs::srv::SetHardwareComponentState>(
      "~/set_hardware_component_state",
      std::bind(&ControllerManager::set_hardware_component_state_srv_cb, this, _1, _2),
      rmw_qos_profile_services_hist_keep_all, best_effort_callback_group_);

  // Add on_shutdown callback to stop the controller manager
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();
  preshutdown_cb_handle_ =
    std::make_unique<rclcpp::PreShutdownCallbackHandle>(context->add_pre_shutdown_callback(
      [this]()
      {
        RCLCPP_INFO(get_logger(), "Shutdown request received....");
        if (this->get_node_base_interface()->get_associated_with_executor_atomic().load())
        {
          executor_->remove_node(this->get_node_base_interface());
        }
        executor_->cancel();
        if (!this->shutdown_controllers())
        {
          RCLCPP_ERROR(get_logger(), "Failed shutting down the controllers.");
        }
        if (!resource_manager_->shutdown_components())
        {
          RCLCPP_ERROR(get_logger(), "Failed shutting down hardware components.");
        }
        RCLCPP_INFO(get_logger(), "Shutting down the controller manager.");
      }));
}

controller_interface::ControllerInterfaceBaseSharedPtr ControllerManager::load_controller(
  const std::string & controller_name, const std::string & controller_type)
{
  RCLCPP_INFO(get_logger(), "Loading controller '%s'", controller_name.c_str());

  // P2-2: seal the type set BEFORE the first lookup. Loading is what shares the registry with the
  // control path, so a registration that raced this call must fail loudly (add() throws) instead of
  // mutating the map under the lookup below. One-way: unloading does not re-open it.
  if (static_controller_registry_ && !static_controller_registry_->frozen())
  {
    static_controller_registry_->freeze();
  }

  controller_interface::ControllerInterfaceBaseSharedPtr controller;

  // A type compiled into the binary is created by its factory and then follows the SAME path as a
  // plugin (`add_controller_impl` below): same controller list, same lifecycle, same interface
  // claiming, same admission checks. The registry is consulted first so a compiled-in type can
  // override a plugin with the same name deliberately, and so a configuration that names it works
  // without changing anything else.
  const bool compiled_in =
    static_controller_registry_ && static_controller_registry_->has(controller_type);
  if (compiled_in)
  {
    RCLCPP_INFO(
      get_logger(), "Controller '%s' (type '%s') comes from the compiled-in registry.",
      controller_name.c_str(), controller_type.c_str());
    controller = static_controller_registry_->create(controller_type);
    if (!controller)
    {
      RCLCPP_ERROR(
        get_logger(), "The compiled-in factory for type '%s' returned no controller.",
        controller_type.c_str());
      return nullptr;
    }
  }
  else
  {
    if (
      !loader_->isClassAvailable(controller_type) &&
      !chainable_loader_->isClassAvailable(controller_type))
    {
      RCLCPP_ERROR(
        get_logger(), "Loader for controller '%s' (type '%s') not found.", controller_name.c_str(),
        controller_type.c_str());
      RCLCPP_INFO(get_logger(), "Available plugin classes:");
      for (const auto & available_class : loader_->getDeclaredClasses())
      {
        RCLCPP_INFO(get_logger(), "  %s", available_class.c_str());
      }
      for (const auto & available_class : chainable_loader_->getDeclaredClasses())
      {
        RCLCPP_INFO(get_logger(), "  %s", available_class.c_str());
      }
      if (static_controller_registry_)
      {
        RCLCPP_INFO(get_logger(), "Available compiled-in types:");
        for (const auto & available_type : static_controller_registry_->types())
        {
          RCLCPP_INFO(get_logger(), "  %s", available_type.c_str());
        }
      }
      return nullptr;
    }
    RCLCPP_DEBUG(get_logger(), "Loader for controller '%s' found.", controller_name.c_str());

    try
    {
      if (loader_->isClassAvailable(controller_type))
      {
        controller = loader_->createSharedInstance(controller_type);
      }
      if (chainable_loader_->isClassAvailable(controller_type))
      {
        controller = chainable_loader_->createSharedInstance(controller_type);
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        get_logger(), "Caught exception while loading the controller '%s' of plugin type '%s':\n%s",
        controller_name.c_str(), controller_type.c_str(), e.what());
      return nullptr;
    }
    catch (...)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Caught unknown exception while loading the controller '%s' of plugin type '%s'",
        controller_name.c_str(), controller_type.c_str());
      throw;
    }
  }

  ControllerSpec controller_spec;
  controller_spec.c = controller;
  controller_spec.info.name = controller_name;
  controller_spec.info.type = controller_type;

  // We have to fetch the parameters_file at the time of loading the controller, because this way we
  // can load them at the creation of the LifeCycleNode and this helps in using the features such as
  // read_only params, dynamic maps lists etc
  // Now check if the parameters_file parameter exist
  const std::string param_name = controller_name + ".params_file";
  controller_spec.info.parameters_files.clear();

  // get_parameter checks if parameter has been declared/set
  rclcpp::Parameter params_files_parameter;
  if (get_parameter(param_name, params_files_parameter))
  {
    if (params_files_parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
      controller_spec.info.parameters_files = params_files_parameter.as_string_array();
    }
    else if (params_files_parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    {
      controller_spec.info.parameters_files.push_back(params_files_parameter.as_string());
    }
    else
    {
      RCLCPP_ERROR(
        get_logger(),
        "The 'params_file' param needs to be a string or a string array for '%s', but it is of "
        "type %s",
        controller_name.c_str(), params_files_parameter.get_type_name().c_str());
    }
  }

  return add_controller_impl(controller_spec);
}

controller_interface::ControllerInterfaceBaseSharedPtr ControllerManager::load_controller(
  const std::string & controller_name)
{
  const std::string param_name = controller_name + ".type";
  std::string controller_type;

  // We cannot declare the parameters for the controllers that will be loaded in the future,
  // because they are plugins and we cannot be aware of all of them.
  // So when we're told to load a controller by name, we need to declare the parameter if
  // we haven't done so, and then read it.

  // Check if parameter has been declared
  if (!has_parameter(param_name))
  {
    declare_parameter(param_name, rclcpp::ParameterType::PARAMETER_STRING);
  }
  if (!get_parameter(param_name, controller_type))
  {
    RCLCPP_ERROR(
      get_logger(), "The 'type' param was not defined for '%s'.", controller_name.c_str());
    return nullptr;
  }
  return load_controller(controller_name, controller_type);
}

controller_interface::return_type ControllerManager::unload_controller(
  const std::string & controller_name)
{
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  std::vector<ControllerSpec> & to = rt_controllers_wrapper_.get_unused_list(guard);
  const std::vector<ControllerSpec> & from = rt_controllers_wrapper_.get_updated_list(guard);

  // Transfers the active controllers over, skipping the one to be removed and the active ones.
  to = from;

  auto found_it = std::find_if(
    to.begin(), to.end(),
    std::bind(controller_name_compare, std::placeholders::_1, controller_name));
  if (found_it == to.end())
  {
    // Fails if we could not remove the controllers
    to.clear();
    RCLCPP_ERROR(
      get_logger(),
      "Could not unload controller with name '%s' because no controller with this name exists",
      controller_name.c_str());
    return controller_interface::return_type::ERROR;
  }

  auto & controller = *found_it;

  if (is_controller_active(*controller.c))
  {
    to.clear();
    RCLCPP_ERROR(
      get_logger(), "Could not unload controller with name '%s' because it is still active",
      controller_name.c_str());
    return controller_interface::return_type::ERROR;
  }

  RCLCPP_DEBUG(get_logger(), "Cleanup controller");
  // TODO(destogl): remove reference interface if chainable; i.e., add a separate method for
  // cleaning-up controllers?
  if (is_controller_inactive(*controller.c))
  {
    RCLCPP_DEBUG(
      get_logger(), "Controller '%s' is cleaned-up before unloading!", controller_name.c_str());
    // TODO(destogl): remove reference interface if chainable; i.e., add a separate method for
    // cleaning-up controllers?
    const auto new_state = controller.c->get_node()->cleanup();
    if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
      RCLCPP_WARN(
        get_logger(), "Failed to clean-up the controller '%s' before unloading!",
        controller_name.c_str());
    }
  }
  executor_->remove_node(controller.c->get_node()->get_node_base_interface());
  to.erase(found_it);

  // Destroys the old controllers list when the realtime thread is finished with it.
  RCLCPP_DEBUG(get_logger(), "Realtime switches over to new controller list");
  rt_controllers_wrapper_.switch_updated_list(guard);
  std::vector<ControllerSpec> & new_unused_list = rt_controllers_wrapper_.get_unused_list(guard);
  RCLCPP_DEBUG(get_logger(), "Destruct controller");
  new_unused_list.clear();
  RCLCPP_DEBUG(get_logger(), "Destruct controller finished");
  // Membership changed: republish the two-phase entry set from this idle thread.
  rebuild_two_phase_entries(rt_controllers_wrapper_.get_updated_list(guard));

  RCLCPP_DEBUG(get_logger(), "Successfully unloaded controller '%s'", controller_name.c_str());
  return controller_interface::return_type::OK;
}

std::vector<ControllerSpec> ControllerManager::get_loaded_controllers() const
{
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  return rt_controllers_wrapper_.get_updated_list(guard);
}

controller_interface::return_type ControllerManager::configure_controller(
  const std::string & controller_name)
{
  RCLCPP_INFO(get_logger(), "Configuring controller '%s'", controller_name.c_str());

  const auto & controllers = get_loaded_controllers();

  auto found_it = std::find_if(
    controllers.begin(), controllers.end(),
    std::bind(controller_name_compare, std::placeholders::_1, controller_name));

  if (found_it == controllers.end())
  {
    RCLCPP_ERROR(
      get_logger(),
      "Could not configure controller with name '%s' because no controller with this name exists",
      controller_name.c_str());
    return controller_interface::return_type::ERROR;
  }
  auto controller = found_it->c;

  auto state = controller->get_state();
  if (
    state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
    state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    RCLCPP_ERROR(
      get_logger(), "Controller '%s' can not be configured from '%s' state.",
      controller_name.c_str(), state.label().c_str());
    return controller_interface::return_type::ERROR;
  }

  auto new_state = controller->get_state();
  if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_DEBUG(
      get_logger(), "Controller '%s' is cleaned-up before configuring", controller_name.c_str());
    // TODO(destogl): remove reference interface if chainable; i.e., add a separate method for
    // cleaning-up controllers?
    new_state = controller->get_node()->cleanup();
    if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
      RCLCPP_ERROR(
        get_logger(), "Controller '%s' can not be cleaned-up before configuring",
        controller_name.c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  new_state = controller->configure();
  if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      get_logger(), "After configuring, controller '%s' is in state '%s' , expected inactive.",
      controller_name.c_str(), new_state.label().c_str());
    return controller_interface::return_type::ERROR;
  }

  const auto controller_update_rate = controller->get_update_rate();
  const auto cm_update_rate = get_update_rate();
  if (controller_update_rate > cm_update_rate)
  {
    RCLCPP_WARN(
      get_logger(),
      "The controller : %s update rate : %d Hz should be less than or equal to controller "
      "manager's update rate : %d Hz!. The controller will be updated at controller_manager's "
      "update rate.",
      controller_name.c_str(), controller_update_rate, cm_update_rate);
  }
  else if (controller_update_rate != 0 && cm_update_rate % controller_update_rate != 0)
  {
    // NOTE: The following computation is done to compute the approx controller update that can be
    // achieved w.r.t to the CM's update rate. This is done this way to take into account the
    // unsigned integer division.
    const auto act_ctrl_update_rate = cm_update_rate / (cm_update_rate / controller_update_rate);
    RCLCPP_WARN(
      get_logger(),
      "The controller : %s update rate : %d Hz is not a perfect divisor of the controller "
      "manager's update rate : %d Hz!. The controller will be updated with nearest divisor's "
      "update rate which is : %d Hz.",
      controller_name.c_str(), controller_update_rate, cm_update_rate, act_ctrl_update_rate);
  }

  // CHAINABLE CONTROLLERS: get reference interfaces from chainable controllers
  if (controller->is_chainable())
  {
    RCLCPP_DEBUG(
      get_logger(),
      "Controller '%s' is chainable. Interfaces are being exported to resource manager.",
      controller_name.c_str());
    auto interfaces = controller->export_reference_interfaces();
    if (interfaces.empty())
    {
      // TODO(destogl): Add test for this!
      RCLCPP_ERROR(
        get_logger(), "Controller '%s' is chainable, but does not export any reference interfaces.",
        controller_name.c_str());
      return controller_interface::return_type::ERROR;
    }
    resource_manager_->import_controller_reference_interfaces(controller_name, interfaces);
  }

  // Now let's reorder the controllers
  // lock controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  std::vector<ControllerSpec> & to = rt_controllers_wrapper_.get_unused_list(guard);
  const std::vector<ControllerSpec> & from = rt_controllers_wrapper_.get_updated_list(guard);

  // Copy all controllers from the 'from' list to the 'to' list
  to = from;

  // Reordering the controllers
  std::stable_sort(
    to.begin(), to.end(),
    std::bind(
      &ControllerManager::controller_sorting, this, std::placeholders::_1, std::placeholders::_2,
      to));

  RCLCPP_DEBUG(get_logger(), "Reordered controllers list is:");
  for (const auto & ctrl : to)
  {
    RCLCPP_DEBUG(this->get_logger(), "\t%s", ctrl.info.name.c_str());
  }

  // Item E: two-phase execution is a mode of the whole controller SET, and this is the operation
  // that admits a controller into it. A late-loaded controller that implements the interface but
  // cannot join the passes used to be logged and excluded, so the configuration looked successful
  // while the controller actually ran through the native loop -- with the wrong order relative to
  // its neighbours. Fail the configure instead, BEFORE the new list is published, so the caller
  // sees the misconfiguration at the last point where it can still react.
  if (current_generation()->two_phase_enabled)
  {
    const auto rejections =
      two_phase_rejections(to, nullptr, current_generation()->staged_group);
    if (!rejections.empty())
    {
      for (const auto & rejection : rejections)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Can not configure '%s' while two-phase execution is enabled: controller '%s' %s.",
          controller_name.c_str(), rejection.name.c_str(), rejection.reason.c_str());
      }
      return controller_interface::return_type::ERROR;
    }
  }

  // switch lists
  rt_controllers_wrapper_.switch_updated_list(guard);
  // clear unused list
  rt_controllers_wrapper_.get_unused_list(guard).clear();
  // Reordering invalidates the order the two passes walk, so republish the entry set as well.
  rebuild_two_phase_entries(rt_controllers_wrapper_.get_updated_list(guard));

  return controller_interface::return_type::OK;
}

void ControllerManager::clear_requests()
{
  switch_params_.do_switch.store(false, std::memory_order_release);
  deactivate_request_.clear();
  activate_request_.clear();
  // The pre-switch snapshot belongs to one switch request; the rollback is the only reader and it
  // runs while the pass is in flight.
  pre_switch_state_.clear();
  // Set these interfaces as unavailable when clearing requests to avoid leaving them in available
  // state without the controller being in active state
  for (const auto & controller_name : to_chained_mode_request_)
  {
    resource_manager_->make_controller_reference_interfaces_unavailable(controller_name);
  }
  to_chained_mode_request_.clear();
  from_chained_mode_request_.clear();
  activate_command_interface_request_.clear();
  deactivate_command_interface_request_.clear();
}

controller_interface::return_type ControllerManager::switch_controller(
  const std::vector<std::string> & activate_controllers,
  const std::vector<std::string> & deactivate_controllers, int strictness, bool activate_asap,
  const rclcpp::Duration & timeout)
{
  // reset the switch param internal variables
  switch_params_.reset();

  if (!deactivate_request_.empty() || !activate_request_.empty())
  {
    RCLCPP_FATAL(
      get_logger(),
      "The internal deactivate and activate request lists are not empty at the beginning of the "
      "switch_controller() call. This should never happen.");
    throw std::runtime_error("CM's internal state is not correct. See the FATAL message above.");
  }
  if (
    !deactivate_command_interface_request_.empty() || !activate_command_interface_request_.empty())
  {
    RCLCPP_FATAL(
      get_logger(),
      "The internal deactivate and activat requests command interface lists are not empty at the "
      "switch_controller() call. This should never happen.");
    throw std::runtime_error("CM's internal state is not correct. See the FATAL message above.");
  }
  if (!from_chained_mode_request_.empty() || !to_chained_mode_request_.empty())
  {
    RCLCPP_FATAL(
      get_logger(),
      "The internal 'from' and 'to' chained mode requests are not empty at the "
      "switch_controller() call. This should never happen.");
    throw std::runtime_error("CM's internal state is not correct. See the FATAL message above.");
  }
  if (strictness == 0)
  {
    RCLCPP_WARN(
      get_logger(),
      "Controller Manager: to switch controllers you need to specify a "
      "strictness level of controller_manager_msgs::SwitchController::STRICT "
      "(%d) or ::BEST_EFFORT (%d). Defaulting to ::BEST_EFFORT",
      controller_manager_msgs::srv::SwitchController::Request::STRICT,
      controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT);
    strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  }

  RCLCPP_DEBUG(get_logger(), "Switching controllers:");
  for (const auto & controller : activate_controllers)
  {
    RCLCPP_DEBUG(get_logger(), "- Activating controller '%s'", controller.c_str());
  }
  for (const auto & controller : deactivate_controllers)
  {
    RCLCPP_DEBUG(get_logger(), "- Deactivating controller '%s'", controller.c_str());
  }

  const auto list_controllers = [this, strictness](
                                  const std::vector<std::string> & controller_list,
                                  std::vector<std::string> & request_list,
                                  const std::string & action)
  {
    // lock controllers
    std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);

    // list all controllers to (de)activate
    for (const auto & controller : controller_list)
    {
      const auto & updated_controllers = rt_controllers_wrapper_.get_updated_list(guard);

      auto found_it = std::find_if(
        updated_controllers.begin(), updated_controllers.end(),
        std::bind(controller_name_compare, std::placeholders::_1, controller));

      if (found_it == updated_controllers.end())
      {
        RCLCPP_WARN(
          get_logger(),
          "Could not '%s' controller with name '%s' because no controller with this name exists",
          action.c_str(), controller.c_str());
        if (strictness == controller_manager_msgs::srv::SwitchController::Request::STRICT)
        {
          RCLCPP_ERROR(get_logger(), "Aborting, no controller is switched! ('STRICT' switch)");
          return controller_interface::return_type::ERROR;
        }
      }
      else
      {
        RCLCPP_DEBUG(
          get_logger(), "Found controller '%s' that needs to be %sed in list of controllers",
          controller.c_str(), action.c_str());
        request_list.push_back(controller);
      }
    }
    RCLCPP_DEBUG(
      get_logger(), "'%s' request vector has size %i", action.c_str(), (int)request_list.size());

    return controller_interface::return_type::OK;
  };

  // list all controllers to deactivate (check if all controllers exist)
  auto ret = list_controllers(deactivate_controllers, deactivate_request_, "deactivate");
  if (ret != controller_interface::return_type::OK)
  {
    deactivate_request_.clear();
    return ret;
  }

  // list all controllers to activate (check if all controllers exist)
  ret = list_controllers(activate_controllers, activate_request_, "activate");
  if (ret != controller_interface::return_type::OK)
  {
    deactivate_request_.clear();
    activate_request_.clear();
    return ret;
  }

  // lock controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);

  const std::vector<ControllerSpec> & controllers = rt_controllers_wrapper_.get_updated_list(guard);

  // Record what every controller looks like BEFORE this switch rewrites the request lists. The
  // rollback needs it to tell "this pass activated a controller" from "this pass restarted an
  // already-active controller only to change its chained mode", and to know which chained mode to go
  // back to. Taken here, before `propagate_deactivation_of_chained_mode()` and the restart loop below
  // add entries to the lists.
  pre_switch_state_.clear();
  pre_switch_state_.reserve(controllers.size());
  for (const auto & controller : controllers)
  {
    pre_switch_state_.push_back(
      PreSwitchState{
        controller.info.name, is_controller_active(*controller.c),
        controller.c->is_in_chained_mode()});
  }

  // if a preceding controller is deactivated, all first-level controllers should be switched 'from'
  // chained mode
  propagate_deactivation_of_chained_mode(controllers);

  // check if controllers should be switched 'to' chained mode when controllers are activated
  for (auto ctrl_it = activate_request_.begin(); ctrl_it != activate_request_.end(); ++ctrl_it)
  {
    auto controller_it = std::find_if(
      controllers.begin(), controllers.end(),
      std::bind(controller_name_compare, std::placeholders::_1, *ctrl_it));
    controller_interface::return_type status = controller_interface::return_type::OK;

    // if controller is not inactive then do not do any following-controllers checks
    if (!is_controller_inactive(controller_it->c))
    {
      RCLCPP_WARN(
        get_logger(),
        "Controller with name '%s' is not inactive so its following "
        "controllers do not have to be checked, because it cannot be activated.",
        controller_it->info.name.c_str());
      status = controller_interface::return_type::ERROR;
    }
    else
    {
      status = check_following_controllers_for_activate(controllers, strictness, controller_it);
    }

    if (status != controller_interface::return_type::OK)
    {
      RCLCPP_WARN(
        get_logger(),
        "Could not activate controller with name '%s'. Check above warnings for more details. "
        "Check the state of the controllers and their required interfaces using "
        "`ros2 control list_controllers -v` CLI to get more information.",
        (*ctrl_it).c_str());
      if (strictness == controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT)
      {
        // TODO(destogl): automatic manipulation of the chain:
        // || strictness ==
        //  controller_manager_msgs::srv::SwitchController::Request::MANIPULATE_CONTROLLERS_CHAIN);
        // remove controller that can not be activated from the activation request and step-back
        // iterator to correctly step to the next element in the list in the loop
        activate_request_.erase(ctrl_it);
        --ctrl_it;
      }
      if (strictness == controller_manager_msgs::srv::SwitchController::Request::STRICT)
      {
        RCLCPP_ERROR(get_logger(), "Aborting, no controller is switched! (::STRICT switch)");
        // reset all lists
        clear_requests();
        return controller_interface::return_type::ERROR;
      }
    }
  }

  // check if controllers should be deactivated if used in chained mode
  for (auto ctrl_it = deactivate_request_.begin(); ctrl_it != deactivate_request_.end(); ++ctrl_it)
  {
    auto controller_it = std::find_if(
      controllers.begin(), controllers.end(),
      std::bind(controller_name_compare, std::placeholders::_1, *ctrl_it));
    controller_interface::return_type status = controller_interface::return_type::OK;

    // if controller is not active then skip preceding-controllers checks
    if (!is_controller_active(controller_it->c))
    {
      RCLCPP_WARN(
        get_logger(), "Controller with name '%s' can not be deactivated since it is not active.",
        controller_it->info.name.c_str());
      status = controller_interface::return_type::ERROR;
    }
    else
    {
      status = check_preceeding_controllers_for_deactivate(controllers, strictness, controller_it);
    }

    if (status != controller_interface::return_type::OK)
    {
      RCLCPP_WARN(
        get_logger(),
        "Could not deactivate controller with name '%s'. Check above warnings for more details. "
        "Check the state of the controllers and their required interfaces using "
        "`ros2 control list_controllers -v` CLI to get more information.",
        (*ctrl_it).c_str());
      if (strictness == controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT)
      {
        // remove controller that can not be activated from the activation request and step-back
        // iterator to correctly step to the next element in the list in the loop
        deactivate_request_.erase(ctrl_it);
        --ctrl_it;
      }
      if (strictness == controller_manager_msgs::srv::SwitchController::Request::STRICT)
      {
        RCLCPP_ERROR(get_logger(), "Aborting, no controller is switched! (::STRICT switch)");
        // reset all lists
        clear_requests();
        return controller_interface::return_type::ERROR;
      }
    }
  }

  for (const auto & controller : controllers)
  {
    auto to_chained_mode_list_it = std::find(
      to_chained_mode_request_.begin(), to_chained_mode_request_.end(), controller.info.name);
    bool in_to_chained_mode_list = to_chained_mode_list_it != to_chained_mode_request_.end();

    auto from_chained_mode_list_it = std::find(
      from_chained_mode_request_.begin(), from_chained_mode_request_.end(), controller.info.name);
    bool in_from_chained_mode_list = from_chained_mode_list_it != from_chained_mode_request_.end();

    auto deactivate_list_it =
      std::find(deactivate_request_.begin(), deactivate_request_.end(), controller.info.name);
    bool in_deactivate_list = deactivate_list_it != deactivate_request_.end();

    const bool is_active = is_controller_active(*controller.c);
    const bool is_inactive = is_controller_inactive(*controller.c);

    // restart controllers that need to switch their 'chained mode' - add to (de)activate lists
    if (in_to_chained_mode_list || in_from_chained_mode_list)
    {
      if (is_active && !in_deactivate_list)
      {
        deactivate_request_.push_back(controller.info.name);
        activate_request_.push_back(controller.info.name);
      }
    }

    // get pointers to places in deactivate and activate lists ((de)activate lists have changed)
    deactivate_list_it =
      std::find(deactivate_request_.begin(), deactivate_request_.end(), controller.info.name);
    in_deactivate_list = deactivate_list_it != deactivate_request_.end();

    auto activate_list_it =
      std::find(activate_request_.begin(), activate_request_.end(), controller.info.name);
    bool in_activate_list = activate_list_it != activate_request_.end();

    auto handle_conflict = [&](const std::string & msg)
    {
      if (strictness == controller_manager_msgs::srv::SwitchController::Request::STRICT)
      {
        RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
        deactivate_request_.clear();
        deactivate_command_interface_request_.clear();
        activate_request_.clear();
        activate_command_interface_request_.clear();
        to_chained_mode_request_.clear();
        from_chained_mode_request_.clear();
        return controller_interface::return_type::ERROR;
      }
      RCLCPP_WARN(get_logger(), "%s", msg.c_str());
      return controller_interface::return_type::OK;
    };

    // check for double stop
    if (!is_active && in_deactivate_list)
    {
      auto conflict_status = handle_conflict(
        "Could not deactivate controller '" + controller.info.name + "' since it is not active");
      if (conflict_status != controller_interface::return_type::OK)
      {
        return conflict_status;
      }
      in_deactivate_list = false;
      deactivate_request_.erase(deactivate_list_it);
    }

    // check for doubled activation
    if (is_active && !in_deactivate_list && in_activate_list)
    {
      auto conflict_status = handle_conflict(
        "Could not activate controller '" + controller.info.name + "' since it is already active");
      if (conflict_status != controller_interface::return_type::OK)
      {
        return conflict_status;
      }
      in_activate_list = false;
      activate_request_.erase(activate_list_it);
    }

    // check for illegal activation of an unconfigured/finalized controller
    if (!is_inactive && !in_deactivate_list && in_activate_list)
    {
      auto conflict_status = handle_conflict(
        "Could not activate controller '" + controller.info.name +
        "' since it is not in inactive state");
      if (conflict_status != controller_interface::return_type::OK)
      {
        return conflict_status;
      }
      in_activate_list = false;
      activate_request_.erase(activate_list_it);
    }

    const auto extract_interfaces_for_controller =
      [this](const ControllerSpec ctrl, std::vector<std::string> & request_interface_list)
    {
      auto command_interface_config = ctrl.c->command_interface_configuration();
      std::vector<std::string> command_interface_names = {};
      if (command_interface_config.type == controller_interface::interface_configuration_type::ALL)
      {
        command_interface_names = resource_manager_->available_command_interfaces();
      }
      if (
        command_interface_config.type ==
        controller_interface::interface_configuration_type::INDIVIDUAL)
      {
        command_interface_names = command_interface_config.names;
      }
      request_interface_list.insert(
        request_interface_list.end(), command_interface_names.begin(),
        command_interface_names.end());
    };

    if (in_activate_list)
    {
      extract_interfaces_for_controller(controller, activate_command_interface_request_);
    }
    if (in_deactivate_list)
    {
      extract_interfaces_for_controller(controller, deactivate_command_interface_request_);
    }
  }

  if (activate_request_.empty() && deactivate_request_.empty())
  {
    RCLCPP_INFO(get_logger(), "Empty activate and deactivate list, not requesting switch");
    clear_requests();
    return controller_interface::return_type::OK;
  }

  // Item E, FIRST LINE: validate the PROSPECTIVE active set BEFORE the switch is requested.
  //
  // The controller list is not re-sorted by a switch, so a violation that was invisible while the
  // controllers were inactive only becomes real when they activate. Two ways that happens:
  //   * a reference edge appears between two controllers that were never active together before;
  //   * a controller whose declaration is "claim everything" gains a reference interface that was
  //     imported after the last snapshot of its claims was taken.
  // Judging here means such a switch is refused while nothing has happened yet: no lifecycle
  // transition, no published list, no republished membership. The check after the switch remains as
  // a second line for a controller whose claims only become visible once it is ACTIVE.
  if (current_generation()->two_phase_enabled)
  {
    std::vector<char> prospective = controller_active_mask(controllers);
    for (std::size_t i = 0; i < controllers.size(); ++i)
    {
      const bool will_activate =
        std::find(
          activate_request_.begin(), activate_request_.end(), controllers[i].info.name) !=
        activate_request_.end();
      const bool will_deactivate =
        std::find(
          deactivate_request_.begin(), deactivate_request_.end(), controllers[i].info.name) !=
        deactivate_request_.end();
      if (will_activate) {prospective[i] = 1;}        // activation wins a chained-mode restart
      else if (will_deactivate) {prospective[i] = 0;}
    }

    const auto rejections = two_phase_rejections(
      controllers, &prospective, current_generation()->staged_group);
    if (!rejections.empty())
    {
      for (const auto & rejection : rejections)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Refusing the switch before it is applied: controller '%s' %s.",
          rejection.name.c_str(), rejection.reason.c_str());
      }
      clear_requests();
      return controller_interface::return_type::ERROR;
    }
  }

  if (
    !activate_command_interface_request_.empty() || !deactivate_command_interface_request_.empty())
  {
    if (!resource_manager_->prepare_command_mode_switch(
          activate_command_interface_request_, deactivate_command_interface_request_))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Could not switch controllers since prepare command mode switch was rejected.");
      clear_requests();
      return controller_interface::return_type::ERROR;
    }
  }
  // start the atomic controller switching
  switch_params_.strictness.store(strictness, std::memory_order_relaxed);
  switch_params_.activate_asap.store(activate_asap, std::memory_order_relaxed);
  if (timeout == rclcpp::Duration{0, 0})
  {
    RCLCPP_INFO_ONCE(get_logger(), "Switch controller timeout is set to 0, using default 1s!");
    switch_params_.timeout = std::chrono::nanoseconds(1'000'000'000);
  }
  else
  {
    switch_params_.timeout = timeout.to_chrono<std::chrono::nanoseconds>();
  }
  // Release: everything above (strictness, activate_asap, timeout, the request lists) must be
  // visible to the control loop before it observes `do_switch`.
  switch_params_.do_switch.store(true, std::memory_order_release);
  // wait until switch is finished
  RCLCPP_DEBUG(get_logger(), "Requested atomic controller switch from realtime loop");
  std::unique_lock<std::mutex> switch_params_guard(switch_params_.mutex, std::defer_lock);
  if (!switch_params_.cv.wait_for(
        switch_params_guard, switch_params_.timeout,
        [this] { return !switch_params_.do_switch.load(std::memory_order_acquire); }))
  {
    RCLCPP_ERROR(
      get_logger(), "Switch controller timed out after %f seconds!",
      static_cast<double>(switch_params_.timeout.count()) / 1e9);
    clear_requests();
    return controller_interface::return_type::ERROR;
  }

  // copy the controllers spec from the used to the unused list
  std::vector<ControllerSpec> & to = rt_controllers_wrapper_.get_unused_list(guard);
  to = controllers;

  // update the claimed interface controller info
  auto switch_result = controller_interface::return_type::OK;
  for (auto & controller : to)
  {
    if (is_controller_active(controller.c))
    {
      auto command_interface_config = controller.c->command_interface_configuration();
      if (command_interface_config.type == controller_interface::interface_configuration_type::ALL)
      {
        controller.info.claimed_interfaces = resource_manager_->available_command_interfaces();
      }
      if (
        command_interface_config.type ==
        controller_interface::interface_configuration_type::INDIVIDUAL)
      {
        controller.info.claimed_interfaces = command_interface_config.names;
      }
    }
    else
    {
      controller.info.claimed_interfaces.clear();
    }
    if (
      std::find(activate_request_.begin(), activate_request_.end(), controller.info.name) !=
      activate_request_.end())
    {
      if (!is_controller_active(controller.c))
      {
        RCLCPP_ERROR(
          get_logger(), "Could not activate controller : '%s'", controller.info.name.c_str());
        switch_result = controller_interface::return_type::ERROR;
      }
    }
    /// @note The following is the case of the real controllers that are deactivated and doesn't
    /// include the chained controllers that are deactivated and activated
    if (
      std::find(deactivate_request_.begin(), deactivate_request_.end(), controller.info.name) !=
        deactivate_request_.end() &&
      std::find(activate_request_.begin(), activate_request_.end(), controller.info.name) ==
        activate_request_.end())
    {
      if (is_controller_active(controller.c))
      {
        RCLCPP_ERROR(
          get_logger(), "Could not deactivate controller : '%s'", controller.info.name.c_str());
        switch_result = controller_interface::return_type::ERROR;
      }
    }
  }

  // P1-2, the report half of the policy: partial membership of a staged group is legitimate
  // (members activate one at a time) but it is INERT -- the group runs nothing. Saying so here, on
  // the switch thread, keeps the state "controller reports ACTIVE but publishes nothing" from being
  // silent, without allocating in the control loop.
  // A COPY of the shared_ptr (not a reference into the generation): the generation handle returned by
  // `current_generation()` is a temporary, so a reference to one of its members would dangle in the
  // body below.
  if (const auto staged = current_generation()->staged_group)
  {
    staged->refresh_member_active_state();
    if (!staged->members_active())
    {
      std::string missing;
      for (const auto & name : staged->member_names())
      {
        const auto it = std::find_if(
          to.begin(), to.end(),
          std::bind(controller_name_compare, std::placeholders::_1, name));
        if (it != to.end() && is_controller_active(*it->c)) {continue;}
        if (!missing.empty()) {missing += ", ";}
        missing += name;
      }
      RCLCPP_WARN(
        get_logger(),
        "Staged execution group is not fully active (inactive member(s): %s). The group is inert "
        "until every member is active: no member's update() is called and no command is committed, "
        "while the members that ARE active keep their interface claims.",
        missing.c_str());
    }
  }

  // Item E, SECOND LINE: the pre-flight below already refuses every scheduling violation known
  // before the switch is requested. This judges the state the switch actually produced, so it also
  // covers a controller whose claims only become visible once it is ACTIVE. By this point the
  // lifecycle transitions have already happened (the control loop applied the switch), so a failure
  // here reports the misconfiguration rather than undoing it -- the same shape as the "could not
  // activate" failures above.
  if (
    switch_result == controller_interface::return_type::OK &&
    current_generation()->two_phase_enabled)
  {
    const auto active_now = controller_active_mask(to);
    const auto rejections = two_phase_rejections(
      to, &active_now, current_generation()->staged_group);
    if (!rejections.empty())
    {
      for (const auto & rejection : rejections)
      {
        RCLCPP_ERROR(
          get_logger(), "Two-phase execution can not schedule controller '%s': %s.",
          rejection.name.c_str(), rejection.reason.c_str());
      }
      switch_result = controller_interface::return_type::ERROR;
    }
  }

  // switch lists
  rt_controllers_wrapper_.switch_updated_list(guard);
  // clear unused list
  rt_controllers_wrapper_.get_unused_list(guard).clear();
  // A switch changes who is active and which controllers exist; republish the two-phase entry set
  // here on the idle thread, so update() never has to rebuild it.
  rebuild_two_phase_entries(rt_controllers_wrapper_.get_updated_list(guard));

  clear_requests();

  RCLCPP_DEBUG_EXPRESSION(
    get_logger(), switch_result == controller_interface::return_type::OK,
    "Successfully switched controllers");
  return switch_result;
}

controller_interface::ControllerInterfaceBaseSharedPtr ControllerManager::add_controller_impl(
  const ControllerSpec & controller)
{
  // lock controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);

  // P2-2: a controller entering the list seals the compiled-in type set, exactly as a load does. The
  // direct add path does not pass through load_controller(), so this is not redundant.
  if (static_controller_registry_ && !static_controller_registry_->frozen())
  {
    static_controller_registry_->freeze();
  }

  std::vector<ControllerSpec> & to = rt_controllers_wrapper_.get_unused_list(guard);
  const std::vector<ControllerSpec> & from = rt_controllers_wrapper_.get_updated_list(guard);

  // Copy all controllers from the 'from' list to the 'to' list
  to = from;

  auto found_it = std::find_if(
    to.begin(), to.end(),
    std::bind(controller_name_compare, std::placeholders::_1, controller.info.name));
  // Checks that we're not duplicating controllers
  if (found_it != to.end())
  {
    to.clear();
    RCLCPP_ERROR(
      get_logger(), "A controller named '%s' was already loaded inside the controller manager",
      controller.info.name.c_str());
    return nullptr;
  }

  const rclcpp::NodeOptions controller_node_options = determine_controller_node_options(controller);
  if (
    controller.c->init(controller.info.name, get_namespace(), controller_node_options) ==
    controller_interface::return_type::ERROR)
  {
    to.clear();
    RCLCPP_ERROR(
      get_logger(), "Could not initialize the controller named '%s'", controller.info.name.c_str());
    return nullptr;
  }

  executor_->add_node(controller.c->get_node()->get_node_base_interface());
  to.emplace_back(controller);

  // Destroys the old controllers list when the realtime thread is finished with it.
  RCLCPP_DEBUG(get_logger(), "Realtime switches over to new controller list");
  rt_controllers_wrapper_.switch_updated_list(guard);
  RCLCPP_DEBUG(get_logger(), "Destruct controller");
  std::vector<ControllerSpec> & new_unused_list = rt_controllers_wrapper_.get_unused_list(guard);
  new_unused_list.clear();
  RCLCPP_DEBUG(get_logger(), "Destruct controller finished");
  // Membership changed: republish the two-phase entry set from this idle thread.
  rebuild_two_phase_entries(rt_controllers_wrapper_.get_updated_list(guard));

  return to.back().c;
}

void ControllerManager::manage_switch()
{
  std::unique_lock<std::mutex> guard(switch_params_.mutex, std::try_to_lock);
  if (!guard.owns_lock())
  {
    RCLCPP_DEBUG(get_logger(), "Unable to lock switch mutex. Retrying in next cycle.");
    return;
  }
  // Ask hardware interfaces to change mode
  if (!resource_manager_->perform_command_mode_switch(
        activate_command_interface_request_, deactivate_command_interface_request_))
  {
    RCLCPP_ERROR(get_logger(), "Error while performing mode switch.");
  }

  deactivate_controllers();

  switch_chained_mode(to_chained_mode_request_, true);
  switch_chained_mode(from_chained_mode_request_, false);

  // activate controllers once the switch is fully complete
  ActivationOutcome outcome;
  if (!switch_params_.activate_asap.load(std::memory_order_relaxed))
  {
    outcome = activate_controllers();
  }
  else
  {
    // activate controllers as soon as their required joints are done switching
    outcome = activate_controllers_asap();
  }
  if (outcome.any_failure && atomic_activation_.load(std::memory_order_relaxed))
  {
    // All-or-nothing: a partially activated tree is not a tree. Only the controllers THIS pass
    // activated are undone; ones that were already active keep running.
    rollback_activated_controllers(outcome);
    if (outcome.rollback_failed)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Atomic activation rollback was incomplete; the controller set may be partially "
        "activated. Recover by inspecting `ros2 control list_controllers -v` and switching the "
        "affected controllers explicitly.");
    }
  }

  // TODO(destogl): move here "do_switch = false"

  switch_params_.do_switch.store(false, std::memory_order_release);
  switch_params_.cv.notify_all();
}

void ControllerManager::rollback_activated_controllers(ActivationOutcome & outcome)
{
  if (outcome.activated.empty()) {return;}
  outcome.rollback_performed = true;
  std::vector<ControllerSpec> & rt_controller_list =
    rt_controllers_wrapper_.update_and_get_used_by_rt_list();

  RCLCPP_ERROR(
    get_logger(),
    "Atomic activation: %zu controller(s) were activated by this switch and at least one other "
    "failed, so the ones this pass started are being undone (nothing stays half-activated).",
    outcome.activated.size());

  // Two kinds of controllers are in `outcome.activated`, and "undo" means something different for
  // each. A controller that was ALREADY ACTIVE before this switch can only be here because upstream
  // restarted it to change its chained mode (`set_chained_mode()` is only allowed while inactive);
  // undoing it means ACTIVE again with the old chained mode. A controller that was INACTIVE is undone
  // by deactivating it. The distinction comes from the pre-switch snapshot, not from the request
  // lists, which this pass has rewritten.
  std::vector<std::string> restore_to_active;
  std::vector<std::string> switched_back_interfaces;
  // Reverse activation order: in a chain the children were activated last, so they are released
  // first and no child is left holding a reference interface of an already-deactivated parent.
  for (auto entry_it = outcome.activated.rbegin(); entry_it != outcome.activated.rend(); ++entry_it)
  {
    const auto & entry = *entry_it;
    const auto found_it = std::find_if(
      rt_controller_list.begin(), rt_controller_list.end(),
      std::bind(controller_name_compare, std::placeholders::_1, entry.name));
    if (found_it == rt_controller_list.end())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Atomic activation rollback: controller '%s' is not in the realtime controller list and "
        "can not be undone.",
        entry.name.c_str());
      outcome.rollback_failed = true;
      continue;
    }
    const auto * pre = pre_switch_state_of(entry.name);
    if (pre != nullptr && pre->active)
    {
      // Restarted, not newly started: it has to end ACTIVE again (handled below, after the chained
      // modes are back). Its interfaces and its loan must NOT be released.
      restore_to_active.push_back(entry.name);
      continue;
    }

    auto controller = found_it->c;
    if (is_controller_active(*controller))
    {
      const auto new_state = controller->get_node()->deactivate();
      controller->release_interfaces();
      if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      {
        RCLCPP_ERROR(
          get_logger(),
          "During the atomic-activation rollback, controller '%s' ended in state '%s', expected "
          "Inactive. The controller may still be running.",
          entry.name.c_str(), new_state.label().c_str());
        outcome.rollback_failed = true;
      }
    }

    // Undo the chainable-controller publication for the controllers that are NOT running any more: a
    // reference interface that stays available while its controller is INACTIVE is exactly the state
    // `clear_requests()` exists to avoid, and it would let a following controller claim a reference
    // into a controller that is not running.
    if (entry.chainable)
    {
      resource_manager_->make_controller_reference_interfaces_unavailable(entry.name);
    }
    // Only the interfaces of controllers that end up NOT active are switched back out: a restarted
    // controller stays started, so the hardware must keep its mode.
    for (const auto & interface_name : entry.command_interfaces)
    {
      if (
        std::find(
          switched_back_interfaces.begin(), switched_back_interfaces.end(), interface_name) ==
        switched_back_interfaces.end())
      {
        switched_back_interfaces.push_back(interface_name);
      }
    }
  }

  // A restart whose own activation failed never reached `outcome.activated`, but it was ACTIVE
  // before this switch, so it must be brought back too -- otherwise a failed switch silently stops a
  // controller that had nothing to do with the failure.
  for (const auto & controller_name : activate_request_)
  {
    const auto * pre = pre_switch_state_of(controller_name);
    if (pre == nullptr || !pre->active) {continue;}
    if (
      std::find(restore_to_active.begin(), restore_to_active.end(), controller_name) ==
      restore_to_active.end())
    {
      restore_to_active.push_back(controller_name);
    }
  }

  // Revert every chained-mode switch this pass made, including for controllers it did NOT activate:
  // a following controller can be moved to chained mode without being started, and `clear_requests()`
  // (upstream's own failure path) equally reverts the reference-interface bookkeeping. Controllers
  // that are ACTIVE still can not change their chained mode here; the restarts among them are
  // stopped for exactly that reason in the loop below.
  const auto revert_chained_mode = [this, &outcome, &rt_controller_list](
                                     const std::string & controller_name)
  {
    const auto * pre = pre_switch_state_of(controller_name);
    if (pre == nullptr) {return;}
    const auto found_it = std::find_if(
      rt_controller_list.begin(), rt_controller_list.end(),
      std::bind(controller_name_compare, std::placeholders::_1, controller_name));
    if (found_it == rt_controller_list.end())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Atomic activation rollback: can not restore the chained mode of unknown controller '%s'.",
        controller_name.c_str());
      outcome.rollback_failed = true;
      return;
    }
    const auto controller = found_it->c;
    if (controller->is_in_chained_mode() == pre->chained) {return;}
    if (is_controller_active(*controller)) {return;}
    switch_chained_mode({controller_name}, pre->chained);
  };
  for (const auto & controller_name : to_chained_mode_request_)
  {
    revert_chained_mode(controller_name);
  }
  for (const auto & controller_name : from_chained_mode_request_)
  {
    revert_chained_mode(controller_name);
  }

  // Bring the restarted controllers back. They are ACTIVE right now, and their chained mode is the
  // one this pass wanted -- so they are stopped once more, put back to the pre-switch chained mode,
  // and started again through the ordinary activation path (which re-claims the interfaces, restores
  // the lifecycle and, for a chainable controller, republishes its reference interfaces).
  if (!restore_to_active.empty())
  {
    std::vector<std::string> to_reactivate;
    for (const auto & controller_name : restore_to_active)
    {
      const auto * pre = pre_switch_state_of(controller_name);
      const auto found_it = std::find_if(
        rt_controller_list.begin(), rt_controller_list.end(),
        std::bind(controller_name_compare, std::placeholders::_1, controller_name));
      if (found_it == rt_controller_list.end() || pre == nullptr)
      {
        outcome.rollback_failed = true;
        continue;
      }
      const auto controller = found_it->c;
      if (is_controller_active(*controller))
      {
        if (controller->is_in_chained_mode() == pre->chained)
        {
          continue;  // already where it was: leave it running
        }
        const auto stopped = controller->get_node()->deactivate();
        controller->release_interfaces();
        if (stopped.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
          RCLCPP_ERROR(
            get_logger(),
            "Atomic activation rollback: controller '%s' could not be stopped to restore its "
            "chained mode and ended in state '%s'.",
            controller_name.c_str(), stopped.label().c_str());
          outcome.rollback_failed = true;
          continue;
        }
      }
      if (controller->is_in_chained_mode() != pre->chained)
      {
        switch_chained_mode({controller_name}, pre->chained);
      }
      to_reactivate.push_back(controller_name);
    }
    if (!to_reactivate.empty())
    {
      const auto restored = activate_controllers_for(to_reactivate);
      if (restored.any_failure)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Atomic activation rollback: %zu controller(s) that this switch had restarted to change "
          "their chained mode could not be brought back to ACTIVE. They were running before the "
          "switch and are now stopped.",
          to_reactivate.size());
        outcome.rollback_failed = true;
      }
    }
  }

  // Undo the hardware command-mode switch. The activation pass switched the hardware into the
  // interfaces of the controllers it activated (and, on failure, switched only the FAILED
  // controllers' interfaces back), so the interfaces of the controllers that end up INACTIVE are
  // still switched in. Deactivating a controller only releases our loan; a hardware in exclusive mode
  // would otherwise stay configured for a controller that no longer runs.
  if (!switched_back_interfaces.empty())
  {
    const bool prepared =
      resource_manager_->prepare_command_mode_switch({}, switched_back_interfaces);
    const bool performed =
      resource_manager_->perform_command_mode_switch({}, switched_back_interfaces);
    if (!prepared || !performed)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Atomic activation rollback: the hardware refused to switch %zu interface(s) back out of "
        "the rolled-back controllers' mode (prepare %s, perform %s). Those interfaces may still be "
        "configured for controllers that are no longer active.",
        switched_back_interfaces.size(), prepared ? "OK" : "FAILED", performed ? "OK" : "FAILED");
      outcome.rollback_failed = true;
    }
  }
}

void ControllerManager::deactivate_controllers()
{
  std::vector<ControllerSpec> & rt_controller_list =
    rt_controllers_wrapper_.update_and_get_used_by_rt_list();
  // stop controllers
  for (const auto & controller_name : deactivate_request_)
  {
    auto found_it = std::find_if(
      rt_controller_list.begin(), rt_controller_list.end(),
      std::bind(controller_name_compare, std::placeholders::_1, controller_name));
    if (found_it == rt_controller_list.end())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Got request to stop controller '%s' but it is not in the realtime controller list",
        controller_name.c_str());
      continue;
    }
    auto controller = found_it->c;
    if (is_controller_active(*controller))
    {
      const auto new_state = controller->get_node()->deactivate();
      controller->release_interfaces();
      if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      {
        RCLCPP_ERROR(
          get_logger(), "After deactivating, controller '%s' is in state '%s', expected Inactive",
          controller_name.c_str(), new_state.label().c_str());
      }
    }
  }
}

void ControllerManager::switch_chained_mode(
  const std::vector<std::string> & chained_mode_switch_list, bool to_chained_mode)
{
  std::vector<ControllerSpec> & rt_controller_list =
    rt_controllers_wrapper_.update_and_get_used_by_rt_list();

  for (const auto & controller_name : chained_mode_switch_list)
  {
    auto found_it = std::find_if(
      rt_controller_list.begin(), rt_controller_list.end(),
      std::bind(controller_name_compare, std::placeholders::_1, controller_name));
    if (found_it == rt_controller_list.end())
    {
      RCLCPP_FATAL(
        get_logger(),
        "Got request to turn %s chained mode for controller '%s', but controller is not in the "
        "realtime controller list. (This should never happen!)",
        (to_chained_mode ? "ON" : "OFF"), controller_name.c_str());
      continue;
    }
    auto controller = found_it->c;
    if (!is_controller_active(*controller))
    {
      if (controller->set_chained_mode(to_chained_mode))
      {
        if (to_chained_mode)
        {
          resource_manager_->make_controller_reference_interfaces_available(controller_name);
        }
        else
        {
          resource_manager_->make_controller_reference_interfaces_unavailable(controller_name);
        }
      }
      else if (!controller->set_chained_mode(to_chained_mode))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Got request to turn %s chained mode for controller '%s', but controller refused to do "
          "it! The control will probably not work as expected. Try to restart all controllers. "
          "If "
          "the error persist check controllers' individual configuration.",
          (to_chained_mode ? "ON" : "OFF"), controller_name.c_str());
      }
    }
    else
    {
      RCLCPP_FATAL(
        get_logger(),
        "Got request to turn %s chained mode for controller '%s', but this can not happen if "
        "controller is in '%s' state. (This should never happen!)",
        (to_chained_mode ? "ON" : "OFF"), controller_name.c_str(),
        hardware_interface::lifecycle_state_names::ACTIVE);
    }
  }
}

ControllerManager::ActivationOutcome ControllerManager::activate_controllers()
{
  auto outcome = activate_controllers_for(activate_request_);
  // All controllers activated, switching done
  switch_params_.do_switch.store(false, std::memory_order_release);
  return outcome;
}

ControllerManager::ActivationOutcome ControllerManager::activate_controllers_for(
  const std::vector<std::string> & names)
{
  ActivationOutcome outcome;
  std::vector<ControllerSpec> & rt_controller_list =
    rt_controllers_wrapper_.update_and_get_used_by_rt_list();
  std::vector<std::string> failed_controllers_command_interfaces;
  for (const auto & controller_name : names)
  {
    auto found_it = std::find_if(
      rt_controller_list.begin(), rt_controller_list.end(),
      std::bind(controller_name_compare, std::placeholders::_1, controller_name));
    if (found_it == rt_controller_list.end())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Got request to activate controller '%s' but it is not in the realtime controller list",
        controller_name.c_str());
      outcome.any_failure = true;
      continue;
    }
    auto controller = found_it->c;

    bool assignment_successful = true;
    // assign command interfaces to the controller
    auto command_interface_config = controller->command_interface_configuration();
    // default to controller_interface::configuration_type::NONE
    std::vector<std::string> command_interface_names = {};
    if (command_interface_config.type == controller_interface::interface_configuration_type::ALL)
    {
      command_interface_names = resource_manager_->available_command_interfaces();
    }
    if (
      command_interface_config.type ==
      controller_interface::interface_configuration_type::INDIVIDUAL)
    {
      command_interface_names = command_interface_config.names;
    }
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    command_loans.reserve(command_interface_names.size());
    for (const auto & command_interface : command_interface_names)
    {
      if (resource_manager_->command_interface_is_claimed(command_interface))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Resource conflict for controller '%s'. Command interface '%s' is already claimed.",
          controller_name.c_str(), command_interface.c_str());
        command_loans.clear();
        assignment_successful = false;
        break;
      }
      try
      {
        command_loans.emplace_back(resource_manager_->claim_command_interface(command_interface));
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(
          get_logger(), "Can't activate controller '%s': %s", controller_name.c_str(), e.what());
        command_loans.clear();
        assignment_successful = false;
        break;
      }
    }
    // something went wrong during command interfaces, go skip the controller
    if (!assignment_successful)
    {
      outcome.any_failure = true;
      continue;
    }

    // assign state interfaces to the controller
    auto state_interface_config = controller->state_interface_configuration();
    // default to controller_interface::configuration_type::NONE
    std::vector<std::string> state_interface_names = {};
    if (state_interface_config.type == controller_interface::interface_configuration_type::ALL)
    {
      state_interface_names = resource_manager_->available_state_interfaces();
    }
    if (
      state_interface_config.type == controller_interface::interface_configuration_type::INDIVIDUAL)
    {
      state_interface_names = state_interface_config.names;
    }
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    state_loans.reserve(state_interface_names.size());
    for (const auto & state_interface : state_interface_names)
    {
      try
      {
        state_loans.emplace_back(resource_manager_->claim_state_interface(state_interface));
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(
          get_logger(), "Can't activate controller '%s': %s", controller_name.c_str(), e.what());
        assignment_successful = false;
        break;
      }
    }
    // something went wrong during state interfaces, go skip the controller
    if (!assignment_successful)
    {
      outcome.any_failure = true;
      continue;
    }
    controller->assign_interfaces(std::move(command_loans), std::move(state_loans));

    const auto new_state = controller->get_node()->activate();
    if (new_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_ERROR(
        get_logger(),
        "After activation, controller '%s' is in state '%s' (%d), expected '%s' (%d). Releasing "
        "interfaces!",
        controller->get_node()->get_name(), new_state.label().c_str(), new_state.id(),
        hardware_interface::lifecycle_state_names::ACTIVE,
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
      controller->release_interfaces();
      outcome.any_failure = true;
      failed_controllers_command_interfaces.insert(
        failed_controllers_command_interfaces.end(), command_interface_names.begin(),
        command_interface_names.end());
      continue;
    }
    outcome.activated.push_back(ActivationOutcome::Activated{
      controller_name, command_interface_names, controller->is_chainable()});

    // if it is a chainable controller, make the reference interfaces available on activation
    if (controller->is_chainable())
    {
      resource_manager_->make_controller_reference_interfaces_available(controller_name);
    }
  }
  // Now prepare and perform the stop interface switching as this is needed for exclusive
  // interfaces
  if (
    !failed_controllers_command_interfaces.empty() &&
    (!resource_manager_->prepare_command_mode_switch({}, failed_controllers_command_interfaces) ||
     !resource_manager_->perform_command_mode_switch({}, failed_controllers_command_interfaces)))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Error switching back the interfaces in the hardware when the controller activation "
      "failed.");
  }
  return outcome;
}

ControllerManager::ActivationOutcome ControllerManager::activate_controllers_asap()
{
  //  https://github.com/ros-controls/ros2_control/issues/263
  return activate_controllers();
}

const ControllerManager::PreSwitchState * ControllerManager::pre_switch_state_of(
  const std::string & name) const
{
  const auto it = std::find_if(
    pre_switch_state_.begin(), pre_switch_state_.end(),
    [&name](const PreSwitchState & state) {return state.name == name;});
  return it == pre_switch_state_.end() ? nullptr : &*it;
}

void ControllerManager::list_controllers_srv_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request>,
  std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response)
{
  // lock services
  RCLCPP_DEBUG(get_logger(), "list controller service called");
  std::lock_guard<std::mutex> services_guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "list controller service locked");

  // lock controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  const std::vector<ControllerSpec> & controllers = rt_controllers_wrapper_.get_updated_list(guard);
  // create helper containers to create chained controller connections
  std::unordered_map<std::string, std::vector<std::string>> controller_chain_interface_map;
  std::unordered_map<std::string, std::set<std::string>> controller_chain_map;
  std::vector<size_t> chained_controller_indices;
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    controller_chain_map[controllers[i].info.name] = {};
  }

  response->controller.reserve(controllers.size());
  for (size_t i = 0; i < controllers.size(); ++i)
  {
    controller_manager_msgs::msg::ControllerState controller_state;

    controller_state.name = controllers[i].info.name;
    controller_state.type = controllers[i].info.type;
    controller_state.claimed_interfaces = controllers[i].info.claimed_interfaces;
    controller_state.state = controllers[i].c->get_state().label();
    controller_state.is_chainable = controllers[i].c->is_chainable();
    controller_state.is_chained = controllers[i].c->is_in_chained_mode();

    // Get information about interfaces if controller are in 'inactive' or 'active' state
    if (is_controller_active(controllers[i].c) || is_controller_inactive(controllers[i].c))
    {
      auto command_interface_config = controllers[i].c->command_interface_configuration();
      if (command_interface_config.type == controller_interface::interface_configuration_type::ALL)
      {
        controller_state.required_command_interfaces = resource_manager_->command_interface_keys();
      }
      else if (
        command_interface_config.type ==
        controller_interface::interface_configuration_type::INDIVIDUAL)
      {
        controller_state.required_command_interfaces = command_interface_config.names;
      }

      auto state_interface_config = controllers[i].c->state_interface_configuration();
      if (state_interface_config.type == controller_interface::interface_configuration_type::ALL)
      {
        controller_state.required_state_interfaces = resource_manager_->state_interface_keys();
      }
      else if (
        state_interface_config.type ==
        controller_interface::interface_configuration_type::INDIVIDUAL)
      {
        controller_state.required_state_interfaces = state_interface_config.names;
      }
      // check for chained interfaces
      for (const auto & interface : controller_state.required_command_interfaces)
      {
        auto prefix_interface_type_pair = split_command_interface(interface);
        auto prefix = prefix_interface_type_pair.first;
        auto interface_type = prefix_interface_type_pair.second;
        if (controller_chain_map.find(prefix) != controller_chain_map.end())
        {
          controller_chain_map[controller_state.name].insert(prefix);
          controller_chain_interface_map[controller_state.name].push_back(interface_type);
        }
      }
      // check reference interfaces only if controller is inactive or active
      if (controllers[i].c->is_chainable())
      {
        auto references =
          resource_manager_->get_controller_reference_interface_names(controllers[i].info.name);
        controller_state.reference_interfaces.reserve(references.size());
        for (const auto & reference : references)
        {
          const std::string prefix_name = controllers[i].c->get_node()->get_name();
          const std::string interface_name = reference.substr(prefix_name.size() + 1);
          controller_state.reference_interfaces.push_back(interface_name);
        }
      }
    }
    response->controller.push_back(controller_state);
    // keep track of controllers that are part of a chain
    if (
      !controller_chain_interface_map[controller_state.name].empty() ||
      controllers[i].c->is_chainable())
    {
      chained_controller_indices.push_back(i);
    }
  }

  // create chain connections for all controllers in a chain
  for (const auto & index : chained_controller_indices)
  {
    auto & controller_state = response->controller[index];
    auto chained_set = controller_chain_map[controller_state.name];
    for (const auto & chained_name : chained_set)
    {
      controller_manager_msgs::msg::ChainConnection connection;
      connection.name = chained_name;
      connection.reference_interfaces = controller_chain_interface_map[controller_state.name];
      controller_state.chain_connections.push_back(connection);
    }
  }

  RCLCPP_DEBUG(get_logger(), "list controller service finished");
}

void ControllerManager::list_controller_types_srv_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Request>,
  std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Response> response)
{
  // lock services
  RCLCPP_DEBUG(get_logger(), "list types service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "list types service locked");

  auto cur_types = loader_->getDeclaredClasses();
  for (const auto & cur_type : cur_types)
  {
    response->types.push_back(cur_type);
    response->base_classes.push_back(kControllerInterfaceClassName);
    RCLCPP_DEBUG(get_logger(), "%s", cur_type.c_str());
  }
  cur_types = chainable_loader_->getDeclaredClasses();
  for (const auto & cur_type : cur_types)
  {
    response->types.push_back(cur_type);
    response->base_classes.push_back(kChainableControllerInterfaceClassName);
    RCLCPP_DEBUG(get_logger(), "%s", cur_type.c_str());
  }

  RCLCPP_DEBUG(get_logger(), "list types service finished");
}

void ControllerManager::load_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::LoadController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::LoadController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(get_logger(), "loading service called for controller '%s' ", request->name.c_str());
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "loading service locked");

  response->ok = load_controller(request->name).get() != nullptr;

  RCLCPP_DEBUG(
    get_logger(), "loading service finished for controller '%s' ", request->name.c_str());
}

void ControllerManager::configure_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(
    get_logger(), "configuring service called for controller '%s' ", request->name.c_str());
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "configuring service locked");

  response->ok = configure_controller(request->name) == controller_interface::return_type::OK;

  RCLCPP_DEBUG(
    get_logger(), "configuring service finished for controller '%s' ", request->name.c_str());
}

void ControllerManager::reload_controller_libraries_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Response> response)
{
  // lock services
  RCLCPP_DEBUG(get_logger(), "reload libraries service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "reload libraries service locked");

  // only reload libraries if no controllers are active
  std::vector<std::string> loaded_controllers, active_controllers;
  loaded_controllers = get_controller_names();
  {
    // lock controllers
    std::lock_guard<std::recursive_mutex> ctrl_guard(rt_controllers_wrapper_.controllers_lock_);
    for (const auto & controller : rt_controllers_wrapper_.get_updated_list(ctrl_guard))
    {
      if (is_controller_active(*controller.c))
      {
        active_controllers.push_back(controller.info.name);
      }
    }
  }
  if (!active_controllers.empty() && !request->force_kill)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Controller manager: Cannot reload controller libraries because"
      " there are still %i active controllers",
      (int)active_controllers.size());
    response->ok = false;
    return;
  }

  // stop active controllers if requested
  if (!loaded_controllers.empty())
  {
    RCLCPP_INFO(get_logger(), "Controller manager: Stopping all active controllers");
    std::vector<std::string> empty;
    if (
      switch_controller(
        empty, active_controllers,
        controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT) !=
      controller_interface::return_type::OK)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Controller manager: Cannot reload controller libraries because failed to stop "
        "active controllers");
      response->ok = false;
      return;
    }
    for (const auto & controller : loaded_controllers)
    {
      if (unload_controller(controller) != controller_interface::return_type::OK)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Controller manager: Cannot reload controller libraries because "
          "failed to unload controller '%s'",
          controller.c_str());
        response->ok = false;
        return;
      }
    }
    loaded_controllers = get_controller_names();
  }
  assert(loaded_controllers.empty());

  // Force a reload on all the PluginLoaders (internally, this recreates the plugin loaders)
  loader_ = std::make_shared<pluginlib::ClassLoader<controller_interface::ControllerInterface>>(
    kControllerInterfaceNamespace, kControllerInterfaceClassName);
  chainable_loader_ =
    std::make_shared<pluginlib::ClassLoader<controller_interface::ChainableControllerInterface>>(
      kControllerInterfaceNamespace, kChainableControllerInterfaceClassName);
  RCLCPP_INFO(
    get_logger(), "Controller manager: reloaded controller libraries for '%s'",
    kControllerInterfaceNamespace);

  response->ok = true;

  RCLCPP_DEBUG(get_logger(), "reload libraries service finished");
}

void ControllerManager::switch_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(get_logger(), "switching service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "switching service locked");

  //   response->ok = switch_controller(
  //     request->activate_controllers, request->deactivate_controllers, request->strictness,
  //     request->activate_asap, request->timeout) == controller_interface::return_type::OK;
  // TODO(destogl): remove this after deprecated fields are removed from service and use the
  // commented three lines above
  // BEGIN: remove when deprecated removed
  auto activate_controllers = request->activate_controllers;
  auto deactivate_controllers = request->deactivate_controllers;

  if (!request->start_controllers.empty())
  {
    RCLCPP_WARN(
      get_logger(),
      "'start_controllers' field is deprecated, use 'activate_controllers' field instead!");
    activate_controllers.insert(
      activate_controllers.end(), request->start_controllers.begin(),
      request->start_controllers.end());
  }
  if (!request->stop_controllers.empty())
  {
    RCLCPP_WARN(
      get_logger(),
      "'stop_controllers' field is deprecated, use 'deactivate_controllers' field instead!");
    deactivate_controllers.insert(
      deactivate_controllers.end(), request->stop_controllers.begin(),
      request->stop_controllers.end());
  }

  auto activate_asap = request->activate_asap;
  if (request->start_asap)
  {
    RCLCPP_WARN(
      get_logger(), "'start_asap' field is deprecated, use 'activate_asap' field instead!");
    activate_asap = request->start_asap;
  }

  response->ok = switch_controller(
                   activate_controllers, deactivate_controllers, request->strictness, activate_asap,
                   request->timeout) == controller_interface::return_type::OK;
  // END: remove when deprecated removed

  RCLCPP_DEBUG(get_logger(), "switching service finished");
}

void ControllerManager::unload_controller_service_cb(
  const std::shared_ptr<controller_manager_msgs::srv::UnloadController::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::UnloadController::Response> response)
{
  // lock services
  RCLCPP_DEBUG(
    get_logger(), "unloading service called for controller '%s' ", request->name.c_str());
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "unloading service locked");

  response->ok = unload_controller(request->name) == controller_interface::return_type::OK;

  RCLCPP_DEBUG(
    get_logger(), "unloading service finished for controller '%s' ", request->name.c_str());
}

void ControllerManager::list_hardware_components_srv_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Request>,
  std::shared_ptr<controller_manager_msgs::srv::ListHardwareComponents::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "list hardware components service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "list hardware components service locked");

  auto hw_components_info = resource_manager_->get_components_status();

  response->component.reserve(hw_components_info.size());

  for (const auto & [component_name, component_info] : hw_components_info)
  {
    auto component = controller_manager_msgs::msg::HardwareComponentState();
    component.name = component_name;
    component.type = component_info.type;
    component.class_type = component_info.class_type;
    component.state.id = component_info.state.id();
    component.state.label = component_info.state.label();

    component.command_interfaces.reserve(component_info.command_interfaces.size());
    for (const auto & interface : component_info.command_interfaces)
    {
      controller_manager_msgs::msg::HardwareInterface hwi;
      hwi.name = interface;
      hwi.is_available = resource_manager_->command_interface_is_available(interface);
      hwi.is_claimed = resource_manager_->command_interface_is_claimed(interface);
      component.command_interfaces.push_back(hwi);
    }

    component.state_interfaces.reserve(component_info.state_interfaces.size());
    for (const auto & interface : component_info.state_interfaces)
    {
      controller_manager_msgs::msg::HardwareInterface hwi;
      hwi.name = interface;
      hwi.is_available = resource_manager_->state_interface_is_available(interface);
      hwi.is_claimed = false;
      component.state_interfaces.push_back(hwi);
    }

    response->component.push_back(component);
  }

  RCLCPP_DEBUG(get_logger(), "list hardware components service finished");
}

void ControllerManager::list_hardware_interfaces_srv_cb(
  const std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Request>,
  std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "list hardware interfaces service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "list hardware interfaces service locked");

  auto state_interface_names = resource_manager_->state_interface_keys();
  for (const auto & state_interface_name : state_interface_names)
  {
    controller_manager_msgs::msg::HardwareInterface hwi;
    hwi.name = state_interface_name;
    hwi.is_available = resource_manager_->state_interface_is_available(state_interface_name);
    hwi.is_claimed = false;
    response->state_interfaces.push_back(hwi);
  }
  auto command_interface_names = resource_manager_->command_interface_keys();
  for (const auto & command_interface_name : command_interface_names)
  {
    controller_manager_msgs::msg::HardwareInterface hwi;
    hwi.name = command_interface_name;
    hwi.is_available = resource_manager_->command_interface_is_available(command_interface_name);
    hwi.is_claimed = resource_manager_->command_interface_is_claimed(command_interface_name);
    response->command_interfaces.push_back(hwi);
  }

  RCLCPP_DEBUG(get_logger(), "list hardware interfaces service finished");
}

void ControllerManager::set_hardware_component_state_srv_cb(
  const std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Request> request,
  std::shared_ptr<controller_manager_msgs::srv::SetHardwareComponentState::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "set hardware component state service called");
  std::lock_guard<std::mutex> guard(services_lock_);
  RCLCPP_DEBUG(get_logger(), "set hardware component state service locked");

  RCLCPP_DEBUG(get_logger(), "set hardware component state '%s'", request->name.c_str());

  auto hw_components_info = resource_manager_->get_components_status();
  if (hw_components_info.find(request->name) != hw_components_info.end())
  {
    rclcpp_lifecycle::State target_state(
      request->target_state.id,
      // the ternary operator is needed because label in State constructor cannot be an empty string
      request->target_state.label.empty() ? "-" : request->target_state.label);
    response->ok =
      (resource_manager_->set_component_state(request->name, target_state) ==
       hardware_interface::return_type::OK);
    hw_components_info = resource_manager_->get_components_status();
    response->state.id = hw_components_info[request->name].state.id();
    response->state.label = hw_components_info[request->name].state.label();
  }
  else
  {
    RCLCPP_ERROR(
      get_logger(), "hardware component with name '%s' does not exist", request->name.c_str());
    response->ok = false;
  }

  RCLCPP_DEBUG(get_logger(), "set hardware component state service finished");
}

std::vector<std::string> ControllerManager::get_controller_names()
{
  std::vector<std::string> names;

  // lock controllers
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  for (const auto & controller : rt_controllers_wrapper_.get_updated_list(guard))
  {
    names.push_back(controller.info.name);
  }
  return names;
}

void ControllerManager::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  resource_manager_->read(time, period);
}

std::shared_ptr<const ControllerManager::ExecutionGeneration>
ControllerManager::current_generation() const noexcept
{
  auto generation = std::atomic_load(&generation_);
  if (!generation)
  {
    // Only reachable before the first publication (a configuration that never set the parameter).
    generation = std::make_shared<const ExecutionGeneration>();
  }
  return generation;
}

void ControllerManager::publish_generation(
  bool two_phase_enabled, std::shared_ptr<const std::vector<TwoPhaseEntry>> entries,
  std::shared_ptr<StagedExecutionGroup> staged_group)
{
  // Copy-then-store: the published object is never mutated afterwards, so a cycle that already
  // loaded it keeps a coherent view while a newer generation is built off to the side.
  const auto previous = current_generation();
  auto next = std::make_shared<ExecutionGeneration>();
  next->two_phase_enabled = two_phase_enabled;
  next->two_phase_entries = std::move(entries);
  next->staged_group = std::move(staged_group);
  next->id = previous->id + 1;
  std::atomic_store(&generation_, std::shared_ptr<const ExecutionGeneration>(std::move(next)));
}

std::uint64_t ControllerManager::execution_generation() const noexcept
{
  return current_generation()->id;
}

std::string ControllerManager::two_phase_admission_reason(
  TwoPhaseAdmission admission, const std::string & detail) const
{
  switch (admission)
  {
    case TwoPhaseAdmission::accepted:
      return "is accepted";
    case TwoPhaseAdmission::already_staged:
      return "is a member of the installed staged execution group, so it would be executed by both "
             "paths in the same cycle";
    case TwoPhaseAdmission::unsupported_update_rate:
      return "declares an update rate that differs from the controller manager's, which the "
             "two-phase passes cannot honour (they always run every cycle)";
    case TwoPhaseAdmission::cross_mode_dependency:
      return "takes part in a reference edge that crosses the two-phase/legacy boundary with '" +
             detail +
             "', whose end would be ordered by a different schedule (the two-phase command pass "
             "runs after the native loop), silently degrading that edge to the previous cycle";
    case TwoPhaseAdmission::unschedulable_order:
      return "is ordered AFTER its child '" + detail +
             "' in the controller manager's controller list, so the two-phase passes would walk that "
             "reference edge in the wrong direction (the backwards state pass would visit the parent "
             "first and the forwards command pass the child first), silently using the previous "
             "cycle's value. Upstream `controller_sorting()` places a chainable controller that "
             "claims NO command interface BEFORE its parent, which is the usual cause";
    case TwoPhaseAdmission::duplicate_instance:
      return "shares ONE controller object with '" + detail +
             "' (the manager only rejects duplicate names, so the same instance can be added under "
             "two names), and a pass would then advance that object once per name in the same cycle";
  }
  return "is rejected";
}

ControllerManager::TwoPhaseAdmission ControllerManager::two_phase_admission(
  const ControllerSpec & controller,
  const std::shared_ptr<StagedExecutionGroup> & staged_for_admission) const noexcept
{
  if (staged_for_admission && staged_for_admission->owns(controller.c.get()))
  {
    return TwoPhaseAdmission::already_staged;
  }

  // The native loop rate-gates a controller whose update_rate divides the manager's rate and
  // passes it a matching period; the two-phase passes have no such gate, so only "follow the
  // manager" (0) or an exact match is safe.
  const auto controller_update_rate = controller.c->get_update_rate();
  if (controller_update_rate != 0 && controller_update_rate != update_rate_)
  {
    return TwoPhaseAdmission::unsupported_update_rate;
  }
  return TwoPhaseAdmission::accepted;
}

std::vector<std::string> ControllerManager::claimed_command_interfaces(
  const ControllerSpec & controller) const
{
  // WHICH set is authoritative depends on the lifecycle state, because the two answer different
  // questions:
  //
  //   * ACTIVE   : which interfaces does this controller ACTUALLY write right now? It holds a loan
  //                for each one, and it cannot acquire a loan for an interface that was imported
  //                after it was activated. `ControllerSpec::info::claimed_interfaces`, refreshed by
  //                `switch_controller()` for every active controller, is that set. Reading the
  //                declaration instead would invent edges to interfaces the controller does not hold
  //                -- e.g. a controller declaring `ALL` would appear to write a reference interface
  //                that a later-configured controller exported, although it can never have claimed it.
  //   * INACTIVE : which interfaces WOULD it write once activated? The declaration answers that, and
  //                the snapshot is empty because deactivation clears it.
  //   * neither  : unconfigured, so nothing is known and the declaration may not even be callable.
  if (is_controller_active(controller.c) && !controller.info.claimed_interfaces.empty())
  {
    return controller.info.claimed_interfaces;
  }
  if (!is_controller_active(controller.c) && !is_controller_inactive(controller.c)) {return {};}

  const auto configuration = controller.c->command_interface_configuration();
  if (configuration.type == controller_interface::interface_configuration_type::ALL)
  {
    return resource_manager_->available_command_interfaces();
  }
  if (configuration.type == controller_interface::interface_configuration_type::INDIVIDUAL)
  {
    return configuration.names;
  }
  return {};
}

std::vector<char> ControllerManager::controller_active_mask(
  const std::vector<ControllerSpec> & controllers) const
{
  std::vector<char> mask(controllers.size(), 0);
  for (std::size_t i = 0; i < controllers.size(); ++i)
  {
    mask[i] = is_controller_active(controllers[i].c) ? 1 : 0;
  }
  return mask;
}

std::vector<ControllerManager::TwoPhaseRejection> ControllerManager::two_phase_rejections(
  const std::vector<ControllerSpec> & controllers, const std::vector<char> * active_mask,
  const std::shared_ptr<StagedExecutionGroup> & staged_for_admission) const
{
  // `judged(i)`: is controller i in the set this verdict is about?
  const auto judged = [&](const std::size_t i)
  {
    if (active_mask == nullptr) {return true;}
    return (*active_mask)[i] != 0;
  };
  // Which loaded controller owns a "<owner>/..." port, and whether that owner implements the
  // interface. An edge between a two-phase member and a non-member cannot be ordered by either
  // schedule, so BOTH ends of such an edge are reported -- excluding only the member would leave
  // the non-member silently writing a reference the member reads a cycle late.
  std::unordered_map<std::string, std::size_t> by_name;
  by_name.reserve(controllers.size());
  for (std::size_t i = 0; i < controllers.size(); ++i)
  {
    by_name.emplace(controllers[i].info.name, i);
  }

  std::vector<char> implements(controllers.size(), 0);
  for (std::size_t i = 0; i < controllers.size(); ++i)
  {
    implements[i] =
      dynamic_cast<hierarchical_control::TwoPhaseControllerInterface *>(controllers[i].c.get()) !=
      nullptr;
  }

  std::vector<TwoPhaseRejection> rejections;
  for (std::size_t i = 0; i < controllers.size(); ++i)
  {
    if (!implements[i]) {continue;}
    if (!judged(i)) {continue;}

    auto admission = two_phase_admission(controllers[i], staged_for_admission);
    std::string detail;
    if (admission == TwoPhaseAdmission::accepted)
    {
      for (const auto & port : claimed_command_interfaces(controllers[i]))
      {
        const auto split = port.find_first_of('/');
        if (split == std::string::npos) {continue;}
        const auto owner = port.substr(0, split);
        if (owner == controllers[i].info.name) {continue;}
        const auto owner_it = by_name.find(owner);
        if (owner_it == by_name.end()) {continue;}  // hardware or an unloaded controller
        if (!implements[owner_it->second])
        {
          admission = TwoPhaseAdmission::cross_mode_dependency;
          detail = owner;
          break;
        }
      }
    }
    if (admission == TwoPhaseAdmission::accepted) {continue;}
    rejections.push_back(
      TwoPhaseRejection{
        controllers[i].info.name, two_phase_admission_reason(admission, detail)});
  }

  // The mirror direction: a controller that does NOT implement the interface is a neighbour of one
  // that does, so it too is part of a cross-mode edge and cannot be scheduled consistently.
  for (std::size_t i = 0; i < controllers.size(); ++i)
  {
    if (implements[i]) {continue;}
    if (!judged(i)) {continue;}
    for (const auto & port : claimed_command_interfaces(controllers[i]))
    {
      const auto split = port.find_first_of('/');
      if (split == std::string::npos) {continue;}
      const auto owner = port.substr(0, split);
      const auto owner_it = by_name.find(owner);
      if (owner_it == by_name.end() || !implements[owner_it->second]) {continue;}
      rejections.push_back(
        TwoPhaseRejection{
          controllers[i].info.name,
          two_phase_admission_reason(TwoPhaseAdmission::cross_mode_dependency, owner)});
      break;
    }
  }

  // Two names for ONE controller object. `add_controller()` checks names only, so this configuration
  // is accepted upstream, and a pass would then advance that object once per name in the same cycle
  // -- the per-name call counts still look perfect while the controller's state advances twice
  // (measured: 6 update_phase and 6 handle_phase calls in 3 cycles for one object listed twice).
  {
    std::unordered_map<const controller_interface::ControllerInterfaceBase *, std::string> owners;
    owners.reserve(controllers.size());
    for (std::size_t i = 0; i < controllers.size(); ++i)
    {
      if (!implements[i]) {continue;}
      if (!judged(i)) {continue;}
      const auto inserted = owners.emplace(controllers[i].c.get(), controllers[i].info.name);
      if (!inserted.second)
      {
        // Both names are reported, so excluding one still leaves no half of the pair scheduled.
        rejections.push_back(
          TwoPhaseRejection{
            inserted.first->second,
            two_phase_admission_reason(
              TwoPhaseAdmission::duplicate_instance, controllers[i].info.name)});
        rejections.push_back(
          TwoPhaseRejection{
            controllers[i].info.name,
            two_phase_admission_reason(TwoPhaseAdmission::duplicate_instance, inserted.first->second)});
      }
    }
  }

  // ORDER VALIDATION. The passes do not sort the controllers themselves: they walk the manager's
  // own controller list, BACKWARD for the state pass and FORWARD for the command pass. That is only
  // correct if, for every reference edge, the parent (the claimant that writes "<child>/<port>")
  // appears BEFORE the child in that list -- then the backward walk reaches the child first and the
  // forward walk reaches the parent first. Upstream `controller_sorting()` normally produces that
  // order, but it places a chainable controller with NO command interface ahead of one that has
  // them, so a child that claims nothing lands before its own parent and BOTH passes run that edge
  // in the wrong direction (measured: the parent then reads the previous cycle's child estimate).
  // The staged group is immune because it derives its order from the declared edges; the two-phase
  // path has to verify the order it is handed. Only edges between two members are checked here;
  // cross-mode edges are already rejected above.
  {
    // Members = implementing controllers that passed the checks above.
    std::unordered_map<std::string, std::size_t> rejected_index;
    for (const auto & rejection : rejections)
    {
      const auto it = by_name.find(rejection.name);
      if (it != by_name.end()) {rejected_index.emplace(rejection.name, it->second);}
    }
    std::vector<char> is_member(controllers.size(), 0);
    for (std::size_t i = 0; i < controllers.size(); ++i)
    {
      is_member[i] = implements[i] && rejected_index.find(controllers[i].info.name) ==
                                       rejected_index.end();
      if (!judged(i)) {is_member[i] = 0;}
    }

    bool order_broken = false;
    for (std::size_t parent = 0; parent < controllers.size() && !order_broken; ++parent)
    {
      if (!is_member[parent]) {continue;}
      for (const auto & port : claimed_command_interfaces(controllers[parent]))
      {
        const auto split = port.find_first_of('/');
        if (split == std::string::npos) {continue;}
        const auto owner = port.substr(0, split);
        const auto child_it = by_name.find(owner);
        if (child_it == by_name.end() || !is_member[child_it->second]) {continue;}
        const std::size_t child = child_it->second;
        if (child == parent) {continue;}  // a controller's own port is not an edge
        if (parent < child) {continue;}   // the required order

        // Report BOTH ends, so a caller that can only exclude (the rebuild path) never keeps one end
        // of an edge the schedule cannot order.
        rejections.push_back(
          TwoPhaseRejection{
            controllers[parent].info.name,
            two_phase_admission_reason(TwoPhaseAdmission::unschedulable_order, owner)});
        rejections.push_back(
          TwoPhaseRejection{
            controllers[child].info.name,
            two_phase_admission_reason(TwoPhaseAdmission::unschedulable_order, owner)});
        order_broken = true;  // one report is enough; the caller fails the whole operation
        break;
      }
    }
  }

  // De-duplicate by name: a controller can take part in several rejected edges.
  std::vector<TwoPhaseRejection> unique;
  unique.reserve(rejections.size());
  std::unordered_map<std::string, bool> seen;
  for (auto & rejection : rejections)
  {
    if (seen.emplace(rejection.name, true).second) {unique.push_back(std::move(rejection));}
  }
  return unique;
}

std::vector<ControllerManager::TwoPhaseRejection>
ControllerManager::two_phase_rejected_controllers() const
{
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  return two_phase_rejections(
    rt_controllers_wrapper_.get_updated_list(guard), nullptr,
    current_generation()->staged_group);
}

std::shared_ptr<const std::vector<ControllerManager::TwoPhaseEntry>>
ControllerManager::two_phase_entries() const noexcept
{
  // By value: the caller keeps the snapshot alive. Returning a reference into a snapshot owned only
  // by this function's local `shared_ptr` would dangle as soon as another thread republishes.
  return current_generation()->two_phase_entries;
}

std::shared_ptr<const std::vector<ControllerManager::TwoPhaseEntry>>
ControllerManager::build_two_phase_entries(
  const std::vector<ControllerSpec> & controllers, bool admission_enabled,
  const std::shared_ptr<StagedExecutionGroup> & staged_for_admission) const
{
  auto entries = std::make_shared<std::vector<TwoPhaseEntry>>();
  if (!admission_enabled) {return entries;}

  entries->reserve(controllers.size());

  // Rejections are computed ONCE, up front, so the log and the published set cannot disagree: the
  // same verdict decides both. A rejected member is left out because the native loop rate-gates it
  // and the staged group, if any, owns it -- leaving it out is what makes "no controller runs twice
  // per cycle" true.
  const auto rejections = two_phase_rejections(controllers, nullptr, staged_for_admission);
  std::unordered_map<std::string, std::string> rejected_names;
  rejected_names.reserve(rejections.size());
  for (const auto & rejection : rejections)
  {
    rejected_names.emplace(rejection.name, rejection.reason);
  }

  std::size_t rejected_members = 0;
  for (const auto & controller : controllers)
  {
    auto * instance =
      dynamic_cast<hierarchical_control::TwoPhaseControllerInterface *>(controller.c.get());
    if (instance == nullptr) {continue;}

    if (rejected_names.find(controller.info.name) != rejected_names.end())
    {
      ++rejected_members;
      continue;
    }
    entries->push_back(TwoPhaseEntry{controller.c.get(), instance});
  }

  std::sort(
    entries->begin(), entries->end(),
    [](const TwoPhaseEntry & a, const TwoPhaseEntry & b)
    {
      return std::less<const controller_interface::ControllerInterfaceBase *>()(a.base, b.base);
    });

  // Non-real-time thread only, and only when something was actually excluded, so this cannot spam
  // the control loop. EVERY rejection is logged, not just the first: the caller has to be able to
  // see why the configuration is half-working (review item E). `two_phase_rejected_controllers()`
  // exposes the same information programmatically.
  if (rejected_members != 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Two-phase execution excluded %zu controller(s) that implement the interface. They keep "
      "running through the native single-pass loop, so their order relative to the two-phase passes "
      "is NOT the parent-before-child one.",
      rejected_members);
  }
  for (const auto & rejection : rejections)
  {
    RCLCPP_ERROR(
      get_logger(), "Two-phase execution: controller '%s' %s.", rejection.name.c_str(),
      rejection.reason.c_str());
  }
  return entries;
}

void ControllerManager::rebuild_two_phase_entries(
  const std::vector<ControllerSpec> & controllers)
{
  rebuild_two_phase_entries(controllers, current_generation()->two_phase_enabled);
}

void ControllerManager::rebuild_two_phase_entries(
  const std::vector<ControllerSpec> & controllers, bool admission_enabled)
{
  // ONE publication: entries for this controller list, with the mode and the staged group the
  // generation already has. When the feature is off there is nothing to change, so the configuration
  // path does not pay for a feature that is not in use (the earlier revision had to publish a null
  // snapshot here, which is exactly the kind of separate publication this generation removes).
  const auto generation = current_generation();
  if (!admission_enabled)
  {
    if (!generation->two_phase_enabled) {return;}
    publish_generation(false, nullptr, generation->staged_group);
    return;
  }
  publish_generation(
    true, build_two_phase_entries(controllers, true, generation->staged_group),
    generation->staged_group);
}

void ControllerManager::refresh_two_phase_controllers()
{
  // Called from the non-real-time thread (set_two_phase_execution): taking the lock is fine here.
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  const std::vector<ControllerSpec> & controllers = rt_controllers_wrapper_.get_updated_list(guard);
  rebuild_two_phase_entries(controllers);
}

std::size_t ControllerManager::two_phase_index(
  const std::vector<TwoPhaseEntry> & entries,
  const controller_interface::ControllerInterfaceBase * controller) const noexcept
{
  const auto it = std::lower_bound(
    entries.begin(), entries.end(), controller,
    [](const TwoPhaseEntry & entry, const controller_interface::ControllerInterfaceBase * value)
    {return std::less<const controller_interface::ControllerInterfaceBase *>()(entry.base, value);});
  if (it == entries.end() || it->base != controller) {return no_two_phase;}
  return static_cast<std::size_t>(std::distance(entries.begin(), it));
}

controller_interface::return_type ControllerManager::set_two_phase_execution(bool enabled)
{
  const auto generation = current_generation();
  if (!enabled)
  {
    // ONE store that both clears the mode and drops the member set: there is no window in which a
    // cycle could see "enabled" with members left over, or members without the mode.
    publish_generation(false, nullptr, generation->staged_group);
    RCLCPP_INFO(get_logger(), "Two-phase execution disabled.");
    return controller_interface::return_type::OK;
  }

  // Refuse the whole request rather than installing a half-working set: a silently excluded
  // controller would look enabled in the flag while still running through the native loop.
  {
    std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
    const std::vector<ControllerSpec> & controllers =
      rt_controllers_wrapper_.get_updated_list(guard);
    const auto rejections = two_phase_rejections(controllers, nullptr, generation->staged_group);
    if (!rejections.empty())
    {
      for (const auto & rejection : rejections)
      {
        RCLCPP_ERROR(
          get_logger(), "Can not enable two-phase execution: controller '%s' %s.",
          rejection.name.c_str(), rejection.reason.c_str());
      }
      return controller_interface::return_type::ERROR;
    }

    // Installing a path while a cycle is in flight is refused: the admission decision above was
    // taken against the controller list, whose publication is upstream's and is not part of the
    // generation. Removing a path (the `!enabled` branch) stays allowed.
    if (control_loop_busy())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Refusing to enable two-phase execution while a control cycle is in flight: the member "
        "admission decision is taken against the controller list, which is published separately "
        "from the execution generation. Retry while the control loop is stopped.");
      return controller_interface::return_type::ERROR;
    }

    // Mode and members in ONE store, so no cycle can observe "enabled but no entries" -- the window
    // the previous publish-then-set order could only narrow, not remove.
    publish_generation(
      true, build_two_phase_entries(controllers, true, generation->staged_group),
      generation->staged_group);
  }
  const auto entries = two_phase_entries();
  RCLCPP_INFO(
    get_logger(), "Two-phase execution enabled (%zu controller(s) implement the interface).",
    entries ? entries->size() : 0u);
  return controller_interface::return_type::OK;
}

bool ControllerManager::set_atomic_activation(bool enabled)
{
  // P1-2: a staged execution group is only safe to activate when a failed multi-controller switch
  // rolls back, so the requirement can not be undone behind the group's back.
  if (!enabled)
  {
    const auto group = current_generation()->staged_group;
    if (group)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Refusing to disable atomic activation while a staged execution group with %zu member(s) "
        "is installed: a group is a whole-tree path and a failed switch must not leave it "
        "half-activated. Clear the group first (clear_staged_execution_group()).",
        group->size());
      return false;
    }
  }
  atomic_activation_.store(enabled, std::memory_order_relaxed);
  RCLCPP_INFO(
    get_logger(), "Atomic activation %s.", enabled ? "enabled" : "disabled (upstream semantics)");
  return true;
}

bool ControllerManager::atomic_activation() const
{
  return atomic_activation_.load(std::memory_order_relaxed);
}

bool ControllerManager::control_loop_busy() const noexcept
{
  return cycles_in_flight_.load(std::memory_order_relaxed) != 0;
}

bool ControllerManager::two_phase_execution() const
{
  return current_generation()->two_phase_enabled;
}

bool ControllerManager::set_static_controller_registry(StaticControllerRegistry::SharedPtr registry)
{
  if (!registry)
  {
    RCLCPP_ERROR(get_logger(), "Refusing to install an empty static controller registry.");
    return false;
  }
  // P2-2: an installed registry that has been loaded from is sealed (see load_controller()). Swapping
  // it would silently change the set of loadable types while controllers are running, and would race
  // the shared_ptr against a concurrent load.
  if (static_controller_registry_ && static_controller_registry_->frozen())
  {
    RCLCPP_ERROR(
      get_logger(),
      "Refusing to replace the static controller registry: this manager has already attempted to "
      "load a controller, so the set of compiled-in types is sealed. Register and install every "
      "type before the first load_controller() call.");
    return false;
  }
  static_controller_registry_ = std::move(registry);
  RCLCPP_INFO(
    get_logger(), "Installed a static controller registry with %zu type(s).",
    static_controller_registry_->types().size());
  return true;
}

std::shared_ptr<StaticControllerRegistry> ControllerManager::static_controller_registry() const
{
  return static_controller_registry_;
}

controller_interface::return_type ControllerManager::set_staged_execution_group(
  const std::vector<std::string> & controller_names, std::int64_t max_age_ns)
{
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  const std::vector<ControllerSpec> & controllers = rt_controllers_wrapper_.get_updated_list(guard);

  // P1-2: a staged group executes as a whole, so it may only be installed together with the
  // all-or-nothing activation that keeps a FAILED switch from leaving it half-activated. Checked
  // first, before the members are even inspected, so the reason is unambiguous.
  if (!atomic_activation_.load(std::memory_order_relaxed))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Refusing to install a staged execution group while atomic activation is disabled: a group "
      "is executed as a whole, and without the rollback a failed switch would leave it partially "
      "activated. Call set_atomic_activation(true) first.");
    return controller_interface::return_type::ERROR;
  }

  std::vector<StagedGroupMember> members;
  members.reserve(controller_names.size());
  for (const auto & name : controller_names)
  {
    const auto it = std::find_if(
      controllers.begin(), controllers.end(),
      std::bind(controller_name_compare, std::placeholders::_1, name));
    if (it == controllers.end())
    {
      RCLCPP_ERROR(
        get_logger(), "Can not stage unknown controller '%s'.", name.c_str());
      return controller_interface::return_type::ERROR;
    }
    if (is_controller_active(it->c))
    {
      RCLCPP_ERROR(
        get_logger(), "Controller '%s' must be inactive to join the staged group.", name.c_str());
      return controller_interface::return_type::ERROR;
    }
    if (dynamic_cast<hierarchical_control::StagedControllerInterface *>(it->c.get()) == nullptr)
    {
      RCLCPP_ERROR(
        get_logger(), "Controller '%s' does not implement the staged controller interface.",
        name.c_str());
      return controller_interface::return_type::ERROR;
    }
    // The two-phase passes and the staged group must own disjoint controller sets, otherwise the
    // same controller would be executed twice in one cycle. The mirror-image check lives in
    // set_two_phase_execution(), so the two entry points can never both admit one controller.
    static const std::vector<TwoPhaseEntry> no_two_phase_entries;
    const auto current_entries = two_phase_entries();
    const auto & entries_for_check = current_entries ? *current_entries : no_two_phase_entries;
    if (
      current_generation()->two_phase_enabled &&
      two_phase_index(entries_for_check, it->c.get()) != no_two_phase)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Controller '%s' is executed by the two-phase passes, so it can not also join the staged "
        "execution group.",
        name.c_str());
      return controller_interface::return_type::ERROR;
    }
    // There is no staged equivalent of the native loop's per-controller rate gate: a group is run
    // once per cycle with the manager's period. Refuse a member that asks for a different rate.
    {
      const auto member_update_rate = it->c->get_update_rate();
      if (member_update_rate != 0 && member_update_rate != update_rate_)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Controller '%s' declares update rate %u Hz but the staged group is executed at the "
          "controller manager's %u Hz.",
          name.c_str(), member_update_rate, update_rate_);
        return controller_interface::return_type::ERROR;
      }
    }
    StagedGroupMember member;
    member.name = name;
    member.controller = it->c;
    // Claimed command interfaces are the topology source: a name prefixed by another member's name
    // is a reference interface produced by this member and consumed by that member.
    member.command_interfaces = it->c->command_interface_configuration().names;
    members.push_back(std::move(member));
  }

  // Same rule as set_two_phase_execution(true): installing a path is refused while a cycle is in
  // flight, because group membership feeds the two-phase admission decision, which is taken against
  // the separately published controller list. Clearing the group stays allowed.
  if (control_loop_busy())
  {
    RCLCPP_ERROR(
      get_logger(),
      "Refusing to install a staged execution group while a control cycle is in flight. Retry "
      "while the control loop is stopped, or clear the group instead (removal is always allowed).");
    return controller_interface::return_type::ERROR;
  }

  std::shared_ptr<StagedExecutionGroup> group;
  try
  {
    group = StagedExecutionGroup::create(members, max_age_ns);
  }
  catch (const std::invalid_argument & e)
  {
    RCLCPP_ERROR(get_logger(), "Rejected staged execution group: %s", e.what());
    return controller_interface::return_type::ERROR;
  }

  // Refresh the cached active flags BEFORE publishing: the published group is then never written
  // again by this thread, so a control cycle that already loaded it cannot race with a refresh
  // (review item D). Members are required to be inactive anyway; this only makes the cache exact.
  group->refresh_member_active_state();
  // Group membership feeds two-phase admission, so the group AND the entry set derived from it are
  // published as ONE generation: a cycle can never see the new group with the old member set (which
  // would let a controller be owned by both paths for one cycle).
  const auto generation = current_generation();
  publish_generation(
    generation->two_phase_enabled,
    build_two_phase_entries(controllers, generation->two_phase_enabled, group), std::move(group));
  RCLCPP_INFO(
    get_logger(), "Installed a staged execution group with %zu members and max age %ld ns.",
    members.size(), static_cast<long>(max_age_ns));
  return controller_interface::return_type::OK;
}

void ControllerManager::clear_staged_execution_group()
{
  // The group must be removed before any of its members is unloaded or goes active. The entry set
  // that admitted members *because* of the group is republished in the same generation.
  std::lock_guard<std::recursive_mutex> guard(rt_controllers_wrapper_.controllers_lock_);
  const auto controllers = rt_controllers_wrapper_.get_updated_list(guard);
  const auto generation = current_generation();
  publish_generation(
    generation->two_phase_enabled,
    build_two_phase_entries(controllers, generation->two_phase_enabled, nullptr), nullptr);
}

std::shared_ptr<StagedExecutionGroup> ControllerManager::staged_execution_group() const
{
  return current_generation()->staged_group;
}

controller_interface::return_type ControllerManager::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Marks a cycle in flight for the configuration setters: installing an execution path while a
  // cycle is running would take its admission decision against a controller list that is being
  // published concurrently (see set_two_phase_execution). RAII so every `return` below clears it.
  const CycleGuard cycle_guard(cycles_in_flight_);

  std::vector<ControllerSpec> & rt_controller_list =
    rt_controllers_wrapper_.update_and_get_used_by_rt_list();

  auto ret = controller_interface::return_type::OK;
  ++update_loop_counter_;
  update_loop_counter_ %= update_rate_;

  // ONE atomic load per cycle for the whole execution state (review item D): the mode, the
  // two-phase member set and the staged group come from the same immutable generation, so a cycle
  // can never mix "enabled" from one configuration state with the member set of another. The local
  // shared_ptr keeps the generation and everything it owns alive for the whole cycle.
  const auto generation = current_generation();
  static const std::vector<TwoPhaseEntry> no_entries;
  const auto & entries =
    generation->two_phase_entries ? *generation->two_phase_entries : no_entries;

  // Opt-in staged group: explicit state (postorder) and command (preorder) phases, one group
  // commit. Its members are skipped by the native loop below, so no controller runs twice.
  // The group is not executed while a switch is pending: membership may be changing.
  const auto & staged = generation->staged_group;
  if (staged && !switch_params_.do_switch.load(std::memory_order_acquire))
  {
    const auto staged_result = staged->run(time, period);
    switch (staged_result.status)
    {
      case StagedStatus::committed:
      case StagedStatus::inactive:
        break;
      default:
        RCLCPP_ERROR(
          get_logger(),
          "Staged execution group failed in cycle %llu at node %zu ('%s') (status %d, fault 0x%x).",
          static_cast<unsigned long long>(staged_result.cycle), staged_result.failed_node,
          staged->node_name(staged_result.failed_node).c_str(),
          static_cast<int>(staged_result.status), staged_result.fault_code);
        ret = controller_interface::return_type::ERROR;
        break;
    }
  }

  // Opt-in FineMote-style execution: two passes over the SAME ordered controller list.
  // Pass 1 ("Update") walks it BACKWARD so children publish before parents consume.
  // Skipped while a switch is pending, because membership may be changing.
  const bool run_two_phase =
    generation->two_phase_enabled && !entries.empty() &&
    !switch_params_.do_switch.load(std::memory_order_acquire);
  bool two_phase_state_failed = false;
  if (run_two_phase)
  {
    for (std::size_t slot = rt_controller_list.size(); slot-- > 0;)
    {
      auto & spec = rt_controller_list[slot];
      const auto index = two_phase_index(entries, spec.c.get());
      if (index == no_two_phase || !is_controller_active(*spec.c)) {continue;}
      if (
        entries[index].instance->update_phase(time, period) !=
        controller_interface::return_type::OK)
      {
        RCLCPP_ERROR(
          get_logger(),
          "Two-phase update stage failed for controller '%s'; this cycle computes no commands.",
          spec.info.name.c_str());
        ret = controller_interface::return_type::ERROR;
        // Containment, weakest form that is still honest: a controller that rejected its own state
        // must not then have its command stage run on that state, and the parents downstream of it
        // cannot be trusted either. The command pass is skipped for the WHOLE cycle instead of
        // partially applied. The command interfaces keep the previous cycle's values; there is no
        // rollback of anything a controller already wrote (see the header).
        two_phase_state_failed = true;
      }
    }
  }

  for (auto loaded_controller : rt_controller_list)
  {
    // TODO(v-lopez) we could cache this information
    // https://github.com/ros-controls/ros2_control/issues/153
    if (is_controller_active(*loaded_controller.c))
    {
      if (staged && staged->owns(loaded_controller.c.get()))
      {
        continue;
      }
      // Drive-by-one-path invariant: with two-phase execution ENABLED, a member is never executed
      // by the native single-pass loop -- not even while a switch is pending and the passes are
      // paused. Gating this on `run_two_phase` instead would silently switch such a controller back
      // to the fused single-pass semantics for the few cycles a switch takes, which is exactly the
      // scheduling behaviour the feature exists to replace (and would double-advance the state of a
      // controller that implements both entry points).
      if (
        generation->two_phase_enabled &&
        two_phase_index(entries, loaded_controller.c.get()) != no_two_phase)
      {
        continue;
      }
      const auto controller_update_rate = loaded_controller.c->get_update_rate();
      const auto controller_update_factor =
        (controller_update_rate == 0) || (controller_update_rate >= update_rate_)
          ? 1u
          : update_rate_ / controller_update_rate;

      bool controller_go = ((update_loop_counter_ % controller_update_factor) == 0);
      RCLCPP_DEBUG(
        get_logger(), "update_loop_counter: '%d ' controller_go: '%s ' controller_name: '%s '",
        update_loop_counter_, controller_go ? "True" : "False",
        loaded_controller.info.name.c_str());

      if (controller_go)
      {
        auto controller_ret = loaded_controller.c->update(
          time, (controller_update_factor != 1u)
                  ? rclcpp::Duration::from_seconds(1.0 / controller_update_rate)
                  : period);

        if (controller_ret != controller_interface::return_type::OK)
        {
          RCLCPP_ERROR(
            get_logger(), "The update call of the following controller returned an error: '%s'",
            loaded_controller.info.name.c_str());
          ret = controller_ret;
        }
      }
    }
  }

  // Pass 2 ("Handle") walks the same list FORWARD so parents publish references before children use.
  if (run_two_phase && two_phase_state_failed)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Two-phase command stage skipped for this cycle because a state stage failed; the command "
      "interfaces keep their previous values.");
  }
  else if (run_two_phase)
  {
    for (auto & spec : rt_controller_list)
    {
      const auto index = two_phase_index(entries, spec.c.get());
      if (index == no_two_phase || !is_controller_active(*spec.c)) {continue;}
      if (
        entries[index].instance->handle_phase(time, period) !=
        controller_interface::return_type::OK)
      {
        RCLCPP_ERROR(
          get_logger(), "Two-phase handle stage failed for controller '%s'.",
          spec.info.name.c_str());
        ret = controller_interface::return_type::ERROR;
      }
    }
  }

  // there are controllers to (de)activate
  if (switch_params_.do_switch.load(std::memory_order_acquire))
  {
    manage_switch();
    // Lifecycle state changed: refresh the staged group's cached active flag here instead of
    // querying it every cycle (Humble's get_current_state() allocates).
    if (staged)
    {
      staged->refresh_member_active_state();
    }
    // Membership is NOT rebuilt here: it belongs to the idle thread, which republishes it in
    // switch_controller() once the switch has been applied. The passes simply pause while a
    // switch is pending, so the control loop never allocates.
  }

  return ret;
}

void ControllerManager::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  resource_manager_->write(time, period);
}

std::vector<ControllerSpec> &
ControllerManager::RTControllerListWrapper::update_and_get_used_by_rt_list()
{
  const int published = updated_controllers_index_.load(std::memory_order_acquire);
  used_by_realtime_controllers_index_.store(published, std::memory_order_release);
  return controllers_lists_[published];
}

std::vector<ControllerSpec> & ControllerManager::RTControllerListWrapper::get_unused_list(
  const std::lock_guard<std::recursive_mutex> &)
{
  if (!controllers_lock_.try_lock())
  {
    throw std::runtime_error("controllers_lock_ not owned by thread");
  }
  controllers_lock_.unlock();
  // Get the index to the outdated controller list
  int free_controllers_list = get_other_list(updated_controllers_index_.load(std::memory_order_acquire));

  // Wait until the outdated controller list is not being used by the realtime thread
  wait_until_rt_not_using(free_controllers_list);
  return controllers_lists_[free_controllers_list];
}

const std::vector<ControllerSpec> & ControllerManager::RTControllerListWrapper::get_updated_list(
  const std::lock_guard<std::recursive_mutex> &) const
{
  if (!controllers_lock_.try_lock())
  {
    throw std::runtime_error("controllers_lock_ not owned by thread");
  }
  controllers_lock_.unlock();
  return controllers_lists_[updated_controllers_index_.load(std::memory_order_acquire)];
}

void ControllerManager::RTControllerListWrapper::switch_updated_list(
  const std::lock_guard<std::recursive_mutex> &)
{
  if (!controllers_lock_.try_lock())
  {
    throw std::runtime_error("controllers_lock_ not owned by thread");
  }
  controllers_lock_.unlock();
  int former_current_controllers_list_ = updated_controllers_index_.load(std::memory_order_acquire);
  updated_controllers_index_.store(
    get_other_list(former_current_controllers_list_), std::memory_order_release);
  wait_until_rt_not_using(former_current_controllers_list_);
}

int ControllerManager::RTControllerListWrapper::get_other_list(int index) const
{
  return (index + 1) % 2;
}

void ControllerManager::RTControllerListWrapper::wait_until_rt_not_using(
  int index, std::chrono::microseconds sleep_period) const
{
  while (used_by_realtime_controllers_index_.load(std::memory_order_acquire) == index)
  {
    if (!rclcpp::ok())
    {
      throw std::runtime_error("rclcpp interrupted");
    }
    std::this_thread::sleep_for(sleep_period);
  }
}

std::pair<std::string, std::string> ControllerManager::split_command_interface(
  const std::string & command_interface)
{
  auto index = command_interface.find('/');
  auto prefix = command_interface.substr(0, index);
  auto interface_type = command_interface.substr(index + 1, command_interface.size() - 1);
  return {prefix, interface_type};
}

unsigned int ControllerManager::get_update_rate() const { return update_rate_; }

void ControllerManager::propagate_deactivation_of_chained_mode(
  const std::vector<ControllerSpec> & controllers)
{
  for (const auto & controller : controllers)
  {
    // get pointers to places in deactivate and activate lists ((de)activate lists have changed)
    auto deactivate_list_it =
      std::find(deactivate_request_.begin(), deactivate_request_.end(), controller.info.name);

    if (deactivate_list_it != deactivate_request_.end())
    {
      // if controller is not active then skip adding following-controllers to "from" chained mode
      // request
      if (!is_controller_active(controller.c))
      {
        RCLCPP_DEBUG(
          get_logger(),
          "Controller with name '%s' can not be deactivated since is not active. "
          "The controller will be removed from the list later."
          "Skipping adding following controllers to 'from' chained mode request.",
          controller.info.name.c_str());
        break;
      }

      for (const auto & cmd_itf_name : controller.c->command_interface_configuration().names)
      {
        // controller that 'cmd_tf_name' belongs to
        ControllersListIterator following_ctrl_it;
        if (
          command_interface_is_reference_interface_of_controller(
            cmd_itf_name, controllers, following_ctrl_it))
        {
          // currently iterated "controller" is preceding controller --> add following controller
          // with matching interface name to "from" chained mode list (if not already in it)
          if (
            std::find(
              from_chained_mode_request_.begin(), from_chained_mode_request_.end(),
              following_ctrl_it->info.name) == from_chained_mode_request_.end())
          {
            from_chained_mode_request_.push_back(following_ctrl_it->info.name);
            RCLCPP_DEBUG(
              get_logger(), "Adding controller '%s' in 'from chained mode' request.",
              following_ctrl_it->info.name.c_str());
          }
        }
      }
    }
  }
}

controller_interface::return_type ControllerManager::check_following_controllers_for_activate(
  const std::vector<ControllerSpec> & controllers, int strictness,
  const ControllersListIterator controller_it)
{
  // we assume that the controller exists is checked in advance
  RCLCPP_DEBUG(
    get_logger(), "Checking following controllers of preceding controller with name '%s'.",
    controller_it->info.name.c_str());

  for (const auto & cmd_itf_name : controller_it->c->command_interface_configuration().names)
  {
    ControllersListIterator following_ctrl_it;
    // Check if interface if reference interface and following controller exist.
    if (!command_interface_is_reference_interface_of_controller(
          cmd_itf_name, controllers, following_ctrl_it))
    {
      continue;
    }
    // TODO(destogl): performance of this code could be optimized by adding additional lists with
    // controllers that cache if the check has failed and has succeeded. Then the following would be
    // done only once per controller, otherwise in complex scenarios the same controller is checked
    // multiple times

    // check that all following controllers exits, are either: activated, will be activated, or
    // will not be deactivated
    RCLCPP_DEBUG(
      get_logger(), "Checking following controller with name '%s'.",
      following_ctrl_it->info.name.c_str());

    // check if following controller is chainable
    if (!following_ctrl_it->c->is_chainable())
    {
      RCLCPP_WARN(
        get_logger(),
        "No reference interface '%s' exist, since the following controller with name '%s' "
        "is not chainable.",
        cmd_itf_name.c_str(), following_ctrl_it->info.name.c_str());
      return controller_interface::return_type::ERROR;
    }

    if (is_controller_active(following_ctrl_it->c))
    {
      // will following controller be deactivated?
      if (
        std::find(
          deactivate_request_.begin(), deactivate_request_.end(), following_ctrl_it->info.name) !=
        deactivate_request_.end())
      {
        RCLCPP_WARN(
          get_logger(), "The following controller with name '%s' will be deactivated.",
          following_ctrl_it->info.name.c_str());
        return controller_interface::return_type::ERROR;
      }
    }
    // check if following controller will not be activated
    else if (
      std::find(activate_request_.begin(), activate_request_.end(), following_ctrl_it->info.name) ==
      activate_request_.end())
    {
      RCLCPP_WARN(
        get_logger(),
        "The following controller with name '%s' is not active and will not be activated.",
        following_ctrl_it->info.name.c_str());
      return controller_interface::return_type::ERROR;
    }

    // Trigger recursion to check all the following controllers only if they are OK, add this
    // controller update chained mode requests
    if (
      check_following_controllers_for_activate(controllers, strictness, following_ctrl_it) ==
      controller_interface::return_type::ERROR)
    {
      return controller_interface::return_type::ERROR;
    }

    // TODO(destogl): this should be discussed how to it the best - just a placeholder for now
    // else if (strictness ==
    //  controller_manager_msgs::srv::SwitchController::Request::MANIPULATE_CONTROLLERS_CHAIN)
    // {
    // // insert to the begin of activate request list to be activated before preceding controller
    //   activate_request_.insert(activate_request_.begin(), following_ctrl_name);
    // }
    if (!following_ctrl_it->c->is_in_chained_mode())
    {
      auto found_it = std::find(
        to_chained_mode_request_.begin(), to_chained_mode_request_.end(),
        following_ctrl_it->info.name);
      if (found_it == to_chained_mode_request_.end())
      {
        to_chained_mode_request_.push_back(following_ctrl_it->info.name);
        // if it is a chainable controller, make the reference interfaces available on preactivation
        // (This is needed when you activate a couple of chainable controller altogether)
        resource_manager_->make_controller_reference_interfaces_available(
          following_ctrl_it->info.name);
        RCLCPP_DEBUG(
          get_logger(), "Adding controller '%s' in 'to chained mode' request.",
          following_ctrl_it->info.name.c_str());
      }
    }
    else
    {
      // Check if following controller is in 'from' chained mode list and remove it, if so
      auto found_it = std::find(
        from_chained_mode_request_.begin(), from_chained_mode_request_.end(),
        following_ctrl_it->info.name);
      if (found_it != from_chained_mode_request_.end())
      {
        from_chained_mode_request_.erase(found_it);
        RCLCPP_DEBUG(
          get_logger(),
          "Removing controller '%s' in 'from chained mode' request because it "
          "should stay in chained mode.",
          following_ctrl_it->info.name.c_str());
      }
    }
  }
  return controller_interface::return_type::OK;
};

controller_interface::return_type ControllerManager::check_preceeding_controllers_for_deactivate(
  const std::vector<ControllerSpec> & controllers, int /*strictness*/,
  const ControllersListIterator controller_it)
{
  // if not chainable no need for any checks
  if (!controller_it->c->is_chainable())
  {
    return controller_interface::return_type::OK;
  }

  if (!controller_it->c->is_in_chained_mode())
  {
    RCLCPP_DEBUG(
      get_logger(),
      "Controller with name '%s' is chainable but not in chained mode. "
      "No need to do any checks of preceding controllers when stopping it.",
      controller_it->info.name.c_str());
    return controller_interface::return_type::OK;
  }

  RCLCPP_DEBUG(
    get_logger(), "Checking preceding controller of following controller with name '%s'.",
    controller_it->info.name.c_str());

  for (const auto & ref_itf_name :
       resource_manager_->get_controller_reference_interface_names(controller_it->info.name))
  {
    std::vector<ControllersListIterator> preceding_controllers_using_ref_itf;

    // TODO(destogl): This data could be cached after configuring controller into a map for faster
    // access here
    for (auto preceding_ctrl_it = controllers.begin(); preceding_ctrl_it != controllers.end();
         ++preceding_ctrl_it)
    {
      const auto preceding_ctrl_cmd_itfs =
        preceding_ctrl_it->c->command_interface_configuration().names;

      // if controller is not preceding go the next one
      if (
        std::find(preceding_ctrl_cmd_itfs.begin(), preceding_ctrl_cmd_itfs.end(), ref_itf_name) ==
        preceding_ctrl_cmd_itfs.end())
      {
        continue;
      }

      // check if preceding controller will be activated
      if (
        is_controller_inactive(preceding_ctrl_it->c) &&
        std::find(
          activate_request_.begin(), activate_request_.end(), preceding_ctrl_it->info.name) !=
          activate_request_.end())
      {
        RCLCPP_WARN(
          get_logger(),
          "Could not deactivate controller with name '%s' because "
          "preceding controller with name '%s' will be activated. ",
          controller_it->info.name.c_str(), preceding_ctrl_it->info.name.c_str());
        return controller_interface::return_type::ERROR;
      }
      // check if preceding controller will not be deactivated
      else if (
        is_controller_active(preceding_ctrl_it->c) &&
        std::find(
          deactivate_request_.begin(), deactivate_request_.end(), preceding_ctrl_it->info.name) ==
          deactivate_request_.end())
      {
        RCLCPP_WARN(
          get_logger(),
          "Could not deactivate controller with name '%s' because "
          "preceding controller with name '%s' is active and will not be deactivated.",
          controller_it->info.name.c_str(), preceding_ctrl_it->info.name.c_str());
        return controller_interface::return_type::ERROR;
      }
      // TODO(destogl): this should be discussed how to it the best - just a placeholder for now
      // else if (
      //  strictness ==
      //  controller_manager_msgs::srv::SwitchController::Request::MANIPULATE_CONTROLLERS_CHAIN)
      // {
      // // insert to the begin of activate request list to be activated before preceding controller
      //   activate_request_.insert(activate_request_.begin(), preceding_ctrl_name);
      // }
    }
  }
  return controller_interface::return_type::OK;
}

bool ControllerManager::controller_sorting(
  const ControllerSpec & ctrl_a, const ControllerSpec & ctrl_b,
  const std::vector<controller_manager::ControllerSpec> & controllers)
{
  // If the neither of the controllers are configured, then return false
  if (!((is_controller_active(ctrl_a.c) || is_controller_inactive(ctrl_a.c)) &&
        (is_controller_active(ctrl_b.c) || is_controller_inactive(ctrl_b.c))))
  {
    if (is_controller_active(ctrl_a.c) || is_controller_inactive(ctrl_a.c))
    {
      return true;
    }
    return false;
  }

  const std::vector<std::string> cmd_itfs = ctrl_a.c->command_interface_configuration().names;
  const std::vector<std::string> state_itfs = ctrl_a.c->state_interface_configuration().names;
  if (cmd_itfs.empty() || !ctrl_a.c->is_chainable())
  {
    // The case of the controllers that don't have any command interfaces. For instance,
    // joint_state_broadcaster
    // If the controller b is also under the same condition, then maintain their initial order
    if (ctrl_b.c->command_interface_configuration().names.empty() || !ctrl_b.c->is_chainable())
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  else if (ctrl_b.c->command_interface_configuration().names.empty() || !ctrl_b.c->is_chainable())
  {
    // If only the controller b is a broadcaster or non chainable type , then swap the controllers
    return false;
  }
  else
  {
    auto following_ctrls = get_following_controller_names(ctrl_a.info.name, controllers);
    if (following_ctrls.empty())
    {
      return false;
    }
    // If the ctrl_b is any of the following controllers of ctrl_a, then place ctrl_a before ctrl_b
    if (
      std::find(following_ctrls.begin(), following_ctrls.end(), ctrl_b.info.name) !=
      following_ctrls.end())
    {
      return true;
    }
    else
    {
      auto ctrl_a_preceding_ctrls = get_preceding_controller_names(ctrl_a.info.name, controllers);
      // This is to check that the ctrl_b is in the preceding controllers list of ctrl_a - This
      // check is useful when there is a chained controller branching, but they belong to same
      // branch
      if (
        std::find(ctrl_a_preceding_ctrls.begin(), ctrl_a_preceding_ctrls.end(), ctrl_b.info.name) !=
        ctrl_a_preceding_ctrls.end())
      {
        return false;
      }

      // This is to handle the cases where, the parsed ctrl_a and ctrl_b are not directly related
      // but might have a common parent - happens in branched chained controller
      auto ctrl_b_preceding_ctrls = get_preceding_controller_names(ctrl_b.info.name, controllers);
      std::sort(ctrl_a_preceding_ctrls.begin(), ctrl_a_preceding_ctrls.end());
      std::sort(ctrl_b_preceding_ctrls.begin(), ctrl_b_preceding_ctrls.end());
      std::list<std::string> intersection;
      std::set_intersection(
        ctrl_a_preceding_ctrls.begin(), ctrl_a_preceding_ctrls.end(),
        ctrl_b_preceding_ctrls.begin(), ctrl_b_preceding_ctrls.end(),
        std::back_inserter(intersection));
      if (!intersection.empty())
      {
        // If there is an intersection, then there is a common parent controller for both ctrl_a and
        // ctrl_b
        return true;
      }

      // If there is no common parent, then they belong to 2 different sets
      auto following_ctrls_b = get_following_controller_names(ctrl_b.info.name, controllers);
      if (following_ctrls_b.empty())
      {
        return true;
      }
      auto find_first_element = [&](const auto & controllers_list) -> int64_t
      {
        auto it = std::find_if(
          controllers.begin(), controllers.end(),
          std::bind(controller_name_compare, std::placeholders::_1, controllers_list.back()));
        if (it != controllers.end())
        {
          return std::distance(controllers.begin(), it);
        }
        return 0;
      };
      const auto ctrl_a_chain_first_controller = find_first_element(following_ctrls);
      const auto ctrl_b_chain_first_controller = find_first_element(following_ctrls_b);
      if (ctrl_a_chain_first_controller < ctrl_b_chain_first_controller)
      {
        return true;
      }
    }

    // If the ctrl_a's state interface is the one exported by the ctrl_b then ctrl_b should be
    // in front of ctrl_a
    // TODO(saikishor): deal with the state interface chaining in the sorting algorithm
    auto state_it = std::find_if(
      state_itfs.begin(), state_itfs.end(),
      [ctrl_b](auto itf)
      {
        auto index = itf.find_first_of('/');
        return ((index != std::string::npos) && (itf.substr(0, index) == ctrl_b.info.name));
      });
    if (state_it != state_itfs.end())
    {
      return false;
    }

    // The rest of the cases, basically end up at the end of the list
    return false;
  }
};

rclcpp::NodeOptions ControllerManager::determine_controller_node_options(
  const ControllerSpec & controller) const
{
  auto check_for_element = [](const auto & list, const auto & element)
  { return std::find(list.begin(), list.end(), element) != list.end(); };

  rclcpp::NodeOptions controller_node_options =
    rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true);
  std::vector<std::string> node_options_arguments = controller_node_options.arguments();
  for (const auto & parameters_file : controller.info.parameters_files)
  {
    if (!check_for_element(node_options_arguments, RCL_ROS_ARGS_FLAG))
    {
      node_options_arguments.push_back(RCL_ROS_ARGS_FLAG);
    }
    node_options_arguments.push_back(RCL_PARAM_FILE_FLAG);
    node_options_arguments.push_back(parameters_file);
  }

  // ensure controller's `use_sim_time` parameter matches controller_manager's
  const rclcpp::Parameter use_sim_time = this->get_parameter("use_sim_time");
  if (use_sim_time.as_bool())
  {
    if (!check_for_element(node_options_arguments, RCL_ROS_ARGS_FLAG))
    {
      node_options_arguments.push_back(RCL_ROS_ARGS_FLAG);
    }
    node_options_arguments.push_back(RCL_PARAM_FLAG);
    node_options_arguments.push_back("use_sim_time:=true");
  }

  controller_node_options = controller_node_options.arguments(node_options_arguments);
  return controller_node_options;
}

}  // namespace controller_manager
