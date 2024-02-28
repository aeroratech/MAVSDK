cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(camera_definition)

add_executable(${PROJECT_NAME}
    camera_definition.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(${PROJECT_NAME}
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
else()
    add_compile_options(${PROJECT_NAME} PRIVATE -WX -W2)
endif()
