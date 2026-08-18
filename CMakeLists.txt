cmake_minimum_required(VERSION 3.16)
project(raicom_multimodal_inspection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(Threads REQUIRED)

add_executable(raicom_multimodal_inspection
  src/raicom_multimodal_inspection.cpp
)

target_include_directories(raicom_multimodal_inspection PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(raicom_multimodal_inspection PRIVATE
  unitree_sdk2
  ${OpenCV_LIBS}
  Threads::Threads
)

target_compile_options(raicom_multimodal_inspection PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)
