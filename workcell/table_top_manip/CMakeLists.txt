cmake_minimum_required(VERSION 3.5)
project(table_top_manip)

# Common packages needed for both platforms
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  thread
  program_options
)
find_package(yaml-cpp REQUIRED)

# Platform-specific packages
if(NOT APPLE)
    find_package(OpenCV REQUIRED)
    find_package(realsense2 REQUIRED)
endif()

# Find common libraries
find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)
find_library(FORCE_CONTROLLERS FORCE_CONTROLLERS HINTS ${CMAKE_INSTALL_PREFIX}/lib/)

include_directories(
  include/
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(ManipServer SHARED 
    src/manip_server.cc 
    src/manip_server_loops.cc
)

if(APPLE)
    target_link_libraries(ManipServer
        ATI_NETFT
        UR_RTDE
        HI_COMMON
        ${RUT}
        ${FORCE_CONTROLLERS}
        yaml-cpp::yaml-cpp
        ${Boost_LIBRARIES}
    )
    target_compile_definitions(ManipServer PRIVATE MACOS)
else()
    target_link_libraries(ManipServer
        REALSENSE
        ATI_NETFT
        GoPro
        ROBOTIQ_FT_HW
        RQSensorLinux
        UR_RTDE
        HI_COMMON
        ${RUT}
        ${FORCE_CONTROLLERS}
        ${realsense2_LIBRARY}
        ${OpenCV_LIBS}
        yaml-cpp::yaml-cpp
        ${Boost_LIBRARIES}
    )
endif()

# Installation
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS ManipServer
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

# Python bindings
pybind11_add_module(manip_server_pybind 
    src/manip_server_pybind.cc
)
target_link_libraries(manip_server_pybind PRIVATE
    ManipServer
)

# Python module settings
set_target_properties(manip_server_pybind PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python"
    PREFIX ""
    OUTPUT_NAME "manip_server_pybind"
)