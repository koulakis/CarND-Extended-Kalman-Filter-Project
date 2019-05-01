project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/tools.cpp src/fusion_ekf.cpp src/kalman_filter.cpp src/tools.h src/fusion_ekf.h src/kalman_filter.h)
set(main src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Link runTests with what we want to test and the GTest and pthread library
# This is a bit hacky since the excecutable which runs the test is being built every time the CMakeLists file is run
# Those two parts should be decoupled so that the code is independent of the tests
add_executable(ExecuteTests tests/run_all_tests.cpp ${sources})
target_link_libraries(ExecuteTests ${GTEST_LIBRARIES} pthread)


add_executable(ExtendedKF ${sources} ${main})

target_link_libraries(ExtendedKF z ssl uv uWS)