cmake_minimum_required(VERSION 3.5)
project(usb_cam)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED COMPONENTS core imgproc cudaarithm cudawarping)
find_package(cv_bridge REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(avformat REQUIRED IMPORTED_TARGET libavformat)
pkg_check_modules(avdevice REQUIRED IMPORTED_TARGET libavdevice)
pkg_check_modules(avcodec REQUIRED IMPORTED_TARGET libavcodec)
pkg_check_modules(avutil REQUIRED IMPORTED_TARGET libavutil)
pkg_check_modules(swscale REQUIRED IMPORTED_TARGET libswscale)

## Build the USB camera library
## Do not use ament_auto here so as to not link to rclcpp
add_library(${PROJECT_NAME} SHARED
  src/usb_cam.cpp
  src/ffmpeg_capture.cpp
  src/format_mapping.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "include"
)

target_link_libraries(${PROJECT_NAME}
  PkgConfig::avformat
  PkgConfig::avdevice
  PkgConfig::avcodec
  PkgConfig::avutil
  PkgConfig::swscale
)

ament_export_libraries(${PROJECT_NAME})

## Declare a ROS 2 composible node as a library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/usb_cam_node.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES})

## Use node to generate an executable
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "usb_cam::UsbCamNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# To fix `DSO missing from command line` error with clang
target_link_libraries(${PROJECT_NAME}_node_exe stdc++)

if(SANITIZE)
  target_compile_options(${PROJECT_NAME} PUBLIC -fsanitize=address -fsanitize=leak)
  target_link_libraries(${PROJECT_NAME} -fsanitize=address -fsanitize=leak)
  target_compile_options(${PROJECT_NAME}_node PUBLIC -fsanitize=address -fsanitize=leak)
  target_link_libraries(${PROJECT_NAME}_node -fsanitize=address -fsanitize=leak)
  target_compile_options(${PROJECT_NAME}_node_exe PUBLIC -fsanitize=address -fsanitize=leak)
  target_link_libraries(${PROJECT_NAME}_node_exe -fsanitize=address -fsanitize=leak)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)
  ament_add_gtest(test_format_mapping
    test/test_format_mapping.cpp)
  target_link_libraries(test_format_mapping
    ${PROJECT_NAME} m stdc++)
  ament_add_gtest(test_timestamps
    test/test_timestamps.cpp)
  target_link_libraries(test_timestamps
    ${PROJECT_NAME} m stdc++)
  ament_add_gtest(test_controls
    test/test_controls.cpp)
  target_link_libraries(test_controls
    ${PROJECT_NAME} m stdc++)
  ament_add_gtest(test_ffmpeg_capture_file
    test/test_ffmpeg_capture_file.cpp)
  target_link_libraries(test_ffmpeg_capture_file
    ${PROJECT_NAME} m stdc++)
  ament_add_gtest(test_crashlog
    test/test_crashlog.cpp)
  target_link_libraries(test_crashlog
    ${PROJECT_NAME} m stdc++)
  ament_add_gtest(test_logging
    test/test_logging.cpp)
  target_link_libraries(test_logging
    ${PROJECT_NAME} m stdc++)
endif()

install(
  PROGRAMS scripts/show_image.py
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
