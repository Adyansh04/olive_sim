cmake_minimum_required(VERSION 3.8)
project(olive_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF/Xacro files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install materials
install(DIRECTORY
  materials
  DESTINATION share/${PROJECT_NAME}/
)

# Install models
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
