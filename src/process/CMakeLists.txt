add_subdirectory(kalman)
add_subdirectory(kcf)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

if (NOT OPENCV_FOUND)
    find_package(OpenCV  REQUIRED)
endif ()

add_library(process SHARED process.cpp)
target_link_libraries(process
        ${OpenCV_LIBS}
        kcf
        kalman
        )