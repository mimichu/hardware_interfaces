cmake_minimum_required(VERSION 3.5)
project(ft_calibration)

find_package(Boost REQUIRED)

include_directories(
  include/
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ft_calibration src/main.cc)

if(APPLE)
    target_link_libraries(ft_calibration
        HI_COMMON
        UR_RTDE
        ATI_NETFT
        ${RUT}
        ${FORCE_CONTROLLERS}
        yaml-cpp::yaml-cpp
    )
    target_compile_definitions(ft_calibration PRIVATE MACOS)
else()
    target_link_libraries(ft_calibration
        HI_COMMON
        UR_RTDE
        ROBOTIQ_FT_HW
        ATI_NETFT
        ${RUT}
        ${FORCE_CONTROLLERS}
        yaml-cpp::yaml-cpp
    )
endif()