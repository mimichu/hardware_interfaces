cmake_minimum_required(VERSION 3.5)
project(hardware_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)

# Add paths for dependencies
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
set(OpenCV_DIR "/opt/homebrew/Cellar/opencv/4.10.0_11/lib/cmake/opencv4")
set(force_control_DIR "/usr/local/lib/cmake/force_control")

# Find required packages
find_package(Eigen3 3.4 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  thread
  program_options
)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

# find_package(FTXUI REQUIRED)
# target_link_libraries(your_target PRIVATE FTXUI::screen FTXUI::dom)


# Find RobotUtilities (from cpplibrary)
find_library(RUT Utilities HINTS 
    /usr/local/lib/RobotUtilities
    ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities
)
if(NOT RUT)
    message(FATAL_ERROR "RobotUtilities library not found. Please install cpplibrary first.")
endif()

# Find Force Controllers
find_library(FORCE_CONTROLLERS FORCE_CONTROLLERS HINTS 
    /usr/local/lib
    ${CMAKE_INSTALL_PREFIX}/lib
)
if(NOT FORCE_CONTROLLERS)
    message(FATAL_ERROR "FORCE_CONTROLLERS library not found. Please install force_control first.")
endif()

# Set platform-specific definitions
if(APPLE)
    add_definitions(-DMACOS)
endif()

include_directories(
  include/
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include  # For RobotUtilities and force_control headers
)

# Common library
add_library(HI_COMMON SHARED
  src/types.cpp
)

target_link_libraries(HI_COMMON
  yaml-cpp::yaml-cpp
)

# Hardware to build - platform specific
if(APPLE)
    set(HARDWARE_COMPONENTS
        hardware/ati_netft
        hardware/coinft
        hardware/ur_rtde
    )
else()
    set(HARDWARE_COMPONENTS
        hardware/ati_netft
        hardware/coinft
        hardware/realsense
        hardware/gopro
        hardware/ur_rtde
        hardware/robotiq_ft_modbus
    )
endif()

foreach(component ${HARDWARE_COMPONENTS})
    add_subdirectory(${component})
endforeach()

# Workcell to build
file(GLOB HARDWARE_INCLUDE_DIRS "hardware/*/include")
include_directories(${HARDWARE_INCLUDE_DIRS})
# add_subdirectory(workcell/table_top_manip)

# Application components
if(APPLE)
    set(APP_COMPONENTS
        applications/force_control_demo
        # applications/ft_calibration
    )
else()
    set(APP_COMPONENTS
        applications/force_control_demo
        # applications/ft_calibration
        # applications/manipulation_data_collection
    )
endif()

foreach(component ${APP_COMPONENTS})
    add_subdirectory(${component})
endforeach()

# Install common
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)

install(TARGETS HI_COMMON
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

# Test executable - platform specific components
# if(APPLE)
#     add_executable(test_hardware_interfaces
#       test/main.cc
#     )
#     target_include_directories(test_hardware_interfaces PRIVATE
#       hardware/ati_netft/include/
#       hardware/ur_rtde/include/
#     )
#     target_link_libraries(test_hardware_interfaces
#       ${Boost_LIBRARIES}
#       ${RUT}
#       ATI_NETFT
#       UR_RTDE
#       yaml-cpp::yaml-cpp
#       spdlog::spdlog
#       ${FORCE_CONTROLLERS}
#     )
# else()
#     add_executable(test_hardware_interfaces
#       test/main.cc
#     )
#     target_include_directories(test_hardware_interfaces PRIVATE
#       hardware/ati_netft/include/
#       hardware/realsense/include/
#       hardware/ur_rtde/include/
#       hardware/robotiq_ft_modbus/include/
#     )
#     target_link_libraries(test_hardware_interfaces
#       ${Boost_LIBRARIES}
#       ${RUT}
#       ATI_NETFT
#       REALSENSE
#       ROBOTIQ_FT_HW
#       yaml-cpp::yaml-cpp
#       spdlog::spdlog
#       ${FORCE_CONTROLLERS}
#     )
# endif()


