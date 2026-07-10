cmake_minimum_required(VERSION 3.12)
project(auto_apms_ros2control)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(auto_apms_behavior_tree_core REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

#
# ------------ Library --------------
#

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
    # Adding this for the generated node model header
    auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
)

#
# ------------ Behavior tree nodes --------------
#

add_library(${PROJECT_NAME}_behavior_tree_nodes SHARED
    "src/node/switch_controller.cpp"
    "src/node/controller_lifecycle.cpp"
    "src/node/reload_controller_libraries.cpp"
    "src/node/list_controllers.cpp"
    "src/node/list_controller_types.cpp"
    "src/node/list_hardware_components.cpp"
    "src/node/list_hardware_interfaces.cpp"
    "src/node/set_hardware_component_state.cpp"
)
target_link_libraries(${PROJECT_NAME}_behavior_tree_nodes PUBLIC
    auto_apms_behavior_tree_core::auto_apms_behavior_tree_core
    ${controller_manager_msgs_TARGETS}
    ${lifecycle_msgs_TARGETS}
)
auto_apms_behavior_tree_register_nodes(${PROJECT_NAME}_behavior_tree_nodes
    "${PROJECT_NAME}::SwitchController"
    "${PROJECT_NAME}::LoadController"
    "${PROJECT_NAME}::ConfigureController"
    "${PROJECT_NAME}::CleanupController"
    "${PROJECT_NAME}::UnloadController"
    "${PROJECT_NAME}::ReloadControllerLibraries"
    "${PROJECT_NAME}::ListControllers"
    "${PROJECT_NAME}::ListControllerTypes"
    "${PROJECT_NAME}::ListHardwareComponents"
    "${PROJECT_NAME}::ListHardwareInterfaces"
    "${PROJECT_NAME}::SetHardwareComponentState"
    NODE_MANIFEST
    "config/node_manifest.yaml.in"
    NODE_MANIFEST_ALIAS
    behavior_tree_nodes
    NODE_MODEL_HEADER_TARGET ${PROJECT_NAME}
)

#
# ------------ Packaging --------------
#

# Export library
install(
    TARGETS
    ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
    auto_apms_behavior_tree_core # Adding this for the generated node model header
)

# Plugin library
install(
    TARGETS
    ${PROJECT_NAME}_behavior_tree_nodes
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

#
# ------------ Testing --------------
#

if(BUILD_TESTING)
    find_package(ament_cmake_copyright REQUIRED)
    ament_copyright()
endif()

ament_package()
