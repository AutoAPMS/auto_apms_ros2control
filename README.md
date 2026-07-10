<p align="center">
  <img width="50%" src="https://autoapms.github.io/auto-apms-guide/logo/autoapms_logo.svg">
</p>

# Using ROS 2 and AutoAPMS with ros2_control

This repository holds ROS 2 packages enabling behavior-based ROS 2 applications for the [ros2_control](https://control.ros.org) framework. This implementation adopts the [AutoAPMS](https://github.com/AutoAPMS/auto-apms) framework and provides behavior tree nodes covering all services offered by the controller manager.

## Packages

- **`auto_apms_ros2control`** — Behavior tree node plugins wrapping the `controller_manager_msgs` services:

  | Node | Service |
  | --- | --- |
  | `SwitchController` | `~/switch_controller` |
  | `LoadController` | `~/load_controller` |
  | `ConfigureController` | `~/configure_controller` |
  | `CleanupController` | `~/cleanup_controller` |
  | `UnloadController` | `~/unload_controller` |
  | `ReloadControllerLibraries` | `~/reload_controller_libraries` |
  | `ListControllers` | `~/list_controllers` |
  | `ListControllerTypes` | `~/list_controller_types` |
  | `ListHardwareComponents` | `~/list_hardware_components` |
  | `ListHardwareInterfaces` | `~/list_hardware_interfaces` |
  | `SetHardwareComponentState` | `~/set_hardware_component_state` |

  The default node manifest additionally provides the convenience registrations `ActivateController` and `DeactivateController` for switching a single controller by name.

## Usage in downstream packages

All nodes resolve the targeted controller manager from the `controller_manager` input port (default: `controller_manager`, following the standard ROS 2 name expansion rules — relative, private `~/...` and fully qualified `/...` names are all supported). The registrations are installed as the node manifest resource `auto_apms_ros2control::behavior_tree_nodes`, so downstream packages can either

1. reference the manifest directly when registering behavior trees:

   ```cmake
   auto_apms_behavior_tree_register_trees(
       "behavior/my_tree.xml"
       NODE_MANIFEST
       "auto_apms_ros2control::behavior_tree_nodes"
   )
   ```

2. or derive customized registrations in their own node manifest using the `parent` field:

   ```yaml
   ActivateMyController:
     parent: auto_apms_ros2control::behavior_tree_nodes.ActivateController
     topic: /my_robot/controller_manager/switch_controller
     port_default:
       controller: my_controller
   ```

## License

Licensed under the Apache License, Version 2.0.
