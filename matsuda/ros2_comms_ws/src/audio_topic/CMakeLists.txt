cmake_minimum_required(VERSION 3.8)
project(audio_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Find audio libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA REQUIRED alsa)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
pkg_check_modules(SNDFILE REQUIRED sndfile)
pkg_check_modules(FFTW REQUIRED fftw3)

# Create audio_publisher executable
add_executable(audio_publisher src/audio_publisher.cpp)
ament_target_dependencies(audio_publisher rclcpp std_msgs sensor_msgs)

# Link libraries for audio_publisher
target_link_libraries(audio_publisher 
  ${OpenCV_LIBS}
  ${ALSA_LIBRARIES}
  ${PORTAUDIO_LIBRARIES}
  ${SNDFILE_LIBRARIES}
  ${FFTW_LIBRARIES}
  -lpthread
)
target_include_directories(audio_publisher PRIVATE 
  ${ALSA_INCLUDE_DIRS}
  ${PORTAUDIO_INCLUDE_DIRS}
  ${SNDFILE_INCLUDE_DIRS}
  ${FFTW_INCLUDE_DIRS}
)

# Create audio_subscriber executable
add_executable(audio_subscriber src/audio_subscriber.cpp)
ament_target_dependencies(audio_subscriber rclcpp std_msgs sensor_msgs)

# Link libraries for audio_subscriber
target_link_libraries(audio_subscriber 
  ${OpenCV_LIBS}
  ${ALSA_LIBRARIES}
  ${PORTAUDIO_LIBRARIES}
  ${SNDFILE_LIBRARIES}
  ${FFTW_LIBRARIES}
  -lpthread
)
target_include_directories(audio_subscriber PRIVATE 
  ${ALSA_INCLUDE_DIRS}
  ${PORTAUDIO_INCLUDE_DIRS}
  ${SNDFILE_INCLUDE_DIRS}
  ${FFTW_INCLUDE_DIRS}
)

# Install executables
install(TARGETS
  audio_publisher
  audio_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if launch directory exists)
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
