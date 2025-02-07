cmake_minimum_required(VERSION 3.15)
project(coinft_class)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ONNXRUNTIME_ROOT "/opt/onnxruntime")

include_directories(include)
include_directories(${ONNXRUNTIME_ROOT}/include)
link_directories(${ONNXRUNTIME_ROOT}/lib)

# Check if Boost is already installed locally
find_package(Boost REQUIRED COMPONENTS system thread)

# Find Eigen
find_package(Eigen3 REQUIRED NO_MODULE)

# Add Eigen include directory
include_directories(${EIGEN3_INCLUDE_DIRS})

if(Boost_FOUND)
    message(STATUS "Using system-installed Boost.")
    include_directories(${Boost_INCLUDE_DIRS})
else()
    # Fall back to fetching Boost if it's not found
    message(WARNING "Boost not found locally. Fetching Boost...")
    include(FetchContent)
    FetchContent_Declare(
        boost
        URL https://github.com/boostorg/boost/releases/download/boost-1.87.0/boost-1.87.0-b2-nodocs.tar.gz
        URL_HASH SHA256=<INSERT_SHA256_HASH_HERE>
    )
    FetchContent_MakeAvailable(boost)
    include_directories(${boost_SOURCE_DIR})
endif()

# Add source files
set(SOURCES
    src/coinft_bus_interface.cpp
    src/CoinFTBus.cpp
)

# Add the executable
add_executable(coinft_bus_test ${SOURCES})

# Link Boost libraries
if(Boost_FOUND)
    target_link_libraries(coinft_bus_test ${Boost_LIBRARIES})
else()
    target_link_libraries(coinft_bus_test Boost::Boost)
endif()

find_library(ONNXRUNTIME_LIB onnxruntime HINTS ${ONNXRUNTIME_ROOT}/lib)
target_link_libraries(coinft_bus_test ${ONNXRUNTIME_LIB})
