set(subsystem environment_perception_controller)
set(library_name ${subsystem}_core)
set(executable_name ${subsystem})

add_library(${library_name} SHARED
  environment_perception_controller.cpp
)

target_link_libraries(${library_name} 
 ${dependency_libs}
)

add_executable(${executable_name}
  main.cpp
)

target_link_libraries(${executable_name} 
 ${dependency_libs}
 ${library_name}
)

rclcpp_components_register_nodes(${library_name} "subsystem_controllers::EnvironmentPerceptionControllerNode")

install(TARGETS ${library_name} ${executable_name}
DESTINATION lib/${PROJECT_NAME}
)
