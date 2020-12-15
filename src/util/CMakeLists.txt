set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

if (NOT OPENCV_FOUND)
    find_package(OpenCV  REQUIRED)
endif ()

AUX_SOURCE_DIRECTORY(. DIR_UTIL)
add_library(util SHARED ${DIR_UTIL})
target_link_libraries(util
        ${OpenCV_LIBS}
        )