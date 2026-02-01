file(GLOB_RECURSE
    data_src 
        ${CMAKE_SOURCE_DIR}/src/data_manager/*.cpp
)

set(MVS_INCLUDE_DIR /opt/MVS/include)
set(MVS_LIB_DIR /opt/MVS/lib/aarch64)
include_directories(${MVS_INCLUDE_DIR})

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(${CMAKE_SOURCE_DIR}/libs/json json_binary_dir)
set(JSON_LIBS 
    nlohmann_json::nlohmann_json
    -lpthread
)

set(MVS_LIBS
    ${MVS_LIB_DIR}/libMvCameraControl.so
)


add_library(data_manager
    STATIC
        ${data_src}    
)

target_link_libraries(data_manager
    PUBLIC
        ${OpenCV_LIBS}
        ${OpenRM_LIBS}
        ${JSON_LIBS}
    ${MVS_LIBS}
)
