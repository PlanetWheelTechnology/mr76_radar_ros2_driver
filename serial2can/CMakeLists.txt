cmake_minimum_required(VERSION 2.8.3)
project(test)

if(${CMAKE_BUILD_TYPE} MATCHES  "Release")
  #set(CMAKE_BUILD_TYPE "Release") 
  set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -Wall")
  #set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -Wall")
  message(STATUS "Mode: Release")
  message(STATUS "optional:-std=c++11 -Wall")
else()
  #set(CMAKE_BUILD_TYPE "Debug") 
  set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
  #set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -O2 -Wall -Wextra -Wpedantic -g2 -ggdb")
  message(STATUS "Mode: Debug")
  #message(STATUS "optional:-std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/
)

# file(GLOB  CPP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
# file(GLOB  C_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.c)

add_executable( ${PROJECT_NAME}_node 
  ${CMAKE_CURRENT_SOURCE_DIR}/linux_demo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/log_module.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/serial_interface_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/serial2can_interface.cpp
)

# binary file link to library
target_link_libraries(${PROJECT_NAME}_node pthread)
