set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

if (NOT OPENCV_FOUND)
    find_package(OpenCV  REQUIRED)
endif ()

AUX_SOURCE_DIRECTORY(. DIR_KALMAN)
add_library(kalman SHARED ${DIR_KALMAN})
target_link_libraries(kalman
        ${OpenCV_LIBS}
        )