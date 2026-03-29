cmake_minimum_required(VERSION 3.8)
project(hik_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
# add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
ament_auto_find_build_dependencies()


# Custom OpenVINO paths (adjust these based on your installation)
set(OpenVINO_ROOT "/usr/local")  # Base directory
set(OpenVINO_INCLUDE_DIRS "${OpenVINO_ROOT}/runtime/include")
set(OpenVINO_LIB_DIR "${OpenVINO_ROOT}/runtime/lib/intel64")

# 验证头文件存在
if(NOT EXISTS "${OpenVINO_INCLUDE_DIRS}/openvino/openvino.hpp")
    message(FATAL_ERROR "OpenVINO headers not found at ${OpenVINO_INCLUDE_DIRS}")
endif()


# 查找所有必需的OpenVINO库
set(OpenVINO_LIBS_TO_FIND openvino openvino_c openvino_onnx_frontend)
set(OpenVINO_LIBRARIES "")
foreach(lib ${OpenVINO_LIBS_TO_FIND})
    find_library(OpenVINO_${lib}_LIB
        NAMES ${lib}
        PATHS ${OpenVINO_LIB_DIR}
        NO_DEFAULT_PATH
    )
    if(OpenVINO_${lib}_LIB)
        list(APPEND OpenVINO_LIBRARIES ${OpenVINO_${lib}_LIB})
        message(STATUS "Found ${lib}: ${OpenVINO_${lib}_LIB}")
    else()
        message(FATAL_ERROR "Could not find ${lib} in ${OpenVINO_LIB_DIR}")
    endif()
endforeach()

include_directories (
  include
  ${OpenVINO_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hik_camera_node.cpp
  src/openvino.cpp
  src/params.cpp
)

ament_target_dependencies(${PROJECT_NAME}
rclcpp 
sensor_msgs 
nav_msgs 
std_msgs 
geometry_msgs 
tf2 
tf2_ros 
tf2_geometry_msgs
OpenCV
image_transport
cv_bridge
)

target_include_directories(${PROJECT_NAME} PUBLIC hikSDK/include)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/amd64)
  install(
    DIRECTORY hikSDK/lib/amd64/
    DESTINATION lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/arm64)
  install(
    DIRECTORY hikSDK/lib/arm64/
    DESTINATION lib
  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME}
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
  ${OpenVINO_LIBRARIES} 
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN hik_camera::HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
