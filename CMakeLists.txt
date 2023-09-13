cmake_minimum_required(VERSION 3.8)
project(rr_estimator)

find_package(ament_cmake_auto REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/${PROJECT_NAME}_component.cpp)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "abu2023::RREstimator")
target_compile_options(${PROJECT_NAME}_component PUBLIC -Wall)

ament_auto_add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
target_link_libraries(${PROJECT_NAME}_component ${OpenCV_LIBS})

ament_auto_package()