cmake_minimum_required(VERSION 3.5)
project(hsr_trajectory_publisher_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_AUTOMOC ON)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/head_trajectory_panel.cpp
  src/arm_trajectory_panel.cpp
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY resource # icons
  DESTINATION share/${PROJECT_NAME}
)

ament_package()