cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE "C:/src/vcpkg/scripts/buildsystems/vcpkg.cmake")

project(Dependencies)

enable_testing()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



find_package(OpenCV REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE ${OpenCV_LIBS})

message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")