cmake_minimum_required(VERSION 2.8.9)

project(lshaped_fitting_test)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

set(LIB_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/lshaped_fitting.cpp
)

set(LIB_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/lshaped_fitting.h
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/lshaped_fitting_test.cpp ${LIB_SOURCES} ${LIB_HEADERS})
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
) 