cmake_minimum_required(VERSION 3.15)
project(KCF-with-Kalman)

set(CMAKE_CXX_STANDARD 14)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Release)
set(CMAKE_C_FLAGS_RELEASE  "${CMAKE_C_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -pg -march=native")
set(CMAKE_CXX_FLAGS_RELEASE   "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG    "${CMAKE_CXX_FLAGS_DEBUG} -Wall -pg -march=native")

find_package(OpenCV  REQUIRED)
find_package(Boost  REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
add_subdirectory(src)

add_executable(KCF-with-Kalman ./src/main.cpp)

target_link_libraries(KCF-with-Kalman
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        process
        util
        )