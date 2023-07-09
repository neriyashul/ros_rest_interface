cmake_minimum_required(VERSION 3.10)
project(ros_rest_interface)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  roslint
)

find_package(cpprestsdk REQUIRED)
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}_server
  CATKIN_DEPENDS
    roscpp
    DEPENDS
    cpprestsdk::cpprest
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_server
   src/web_server.cpp
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}
  src/ros_rest_relay_node.cpp
  src/point_controller.cpp
  src/ros_message_gateway.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} 
  cpprestsdk::cpprest
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_server
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if(${CATKIN_ENABLE_TESTING})
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_ros_rest_interface.cpp
)
target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()

roslint_cpp()