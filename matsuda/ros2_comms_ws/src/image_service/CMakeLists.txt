cmake_minimum_required(VERSION 3.8)
project(image_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED libcurl)

# Create executable for image service server
add_executable(image_service_server src/image_service_server.cpp)

# Create executable for image service client
add_executable(image_service_client src/image_service_client.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(image_service_server rclcpp std_srvs sensor_msgs std_msgs)
ament_target_dependencies(image_service_client rclcpp std_srvs sensor_msgs std_msgs)

# Link OpenCV and curl libraries
target_link_libraries(image_service_server ${OpenCV_LIBS} ${CURL_LIBRARIES})
target_include_directories(image_service_server PRIVATE ${CURL_INCLUDE_DIRS})

# Install executables
install(TARGETS
  image_service_server
  image_service_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
