set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

if (NOT OPENCV_FOUND)
    find_package(OpenCV  REQUIRED)
endif ()

AUX_SOURCE_DIRECTORY(. DIR_KCF)
add_library(kcf SHARED ${DIR_KCF})
target_link_libraries(kcf
        ${OpenCV_LIBS}
        )