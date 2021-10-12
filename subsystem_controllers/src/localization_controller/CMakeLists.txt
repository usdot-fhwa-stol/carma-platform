set(subsystem localization_controller)
set(library_name ${subsystem}_core)
set(executable_name ${subsystem})

add_library(${library_name} SHARED
  localization_controller.cpp
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

rclcpp_components_register_nodes(${library_name} "subsystem_controllers::LocalizationControllerNode")

install(TARGETS ${library_name} ${executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
