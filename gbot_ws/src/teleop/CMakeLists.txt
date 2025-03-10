cmake_minimum_required(VERSION 3.8)
project(teleop)

# Set C++ standard before any targets are created
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(websocketpp REQUIRED)

# Remove or comment out the add_subdirectory line since we're using the installed version
# add_subdirectory(${CMAKE_SOURCE_DIR}/../../libdatachannel libdatachannel)

# Make sure we can find the built library
link_directories(${CMAKE_SOURCE_DIR}/../../install/libdatachannel/lib)

# Add include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/../../install/libdatachannel/include
  ${JSONCPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
)

# Create WebRTC Library
add_library(webrtc_lib
  src/webrtc.cpp
)

target_link_libraries(webrtc_lib
  PUBLIC  # Changed from PRIVATE to PUBLIC
  datachannel
  ${OPENSSL_LIBRARIES}
  Threads::Threads
  ${OpenCV_LIBS}
  dl
)

target_compile_definitions(webrtc_lib
  PRIVATE
  WEBRTC_POSIX
  WEBRTC_ARM64
)

target_include_directories(webrtc_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../../install/libdatachannel/include
    ${CMAKE_SOURCE_DIR}/../../install/libdatachannel/build/include
)

# Image subscriber executable
add_executable(image_subscriber
  src/image_subscriber.cpp
)
ament_target_dependencies(image_subscriber 
  rclcpp 
  std_msgs 
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)
target_link_libraries(image_subscriber
  webrtc_lib
)

# Modern way to include directories
target_include_directories(image_subscriber PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install the executable
install(TARGETS
  image_subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

ament_package()