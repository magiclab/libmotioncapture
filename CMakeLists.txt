cmake_minimum_required(VERSION 2.8.3)
project(libmotioncapture)

find_package(catkin)

# Enable C++11
# This requires PCL to be compiled with C++11 enabled as well!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libmotioncapture
  CATKIN_DEPENDS
  DEPENDS
)

find_package(PCL REQUIRED)
set(VICON_SDK_DIR externalDependencies/vicon_sdk/)
set(NATNET_DIR externalDependencies/NatNetLinux/)
set(CDP_DIR externalDependencies/CDPParser/)

###########
## Build ##
###########

## Additional include folders
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${VICON_SDK_DIR}/include
  ${NATNET_DIR}/include
  ${CDP_DIR}/include
)

link_directories(
  ${VICON_SDK_DIR}/lib64
)

## Declare a cpp library
add_library(libmotioncapture
  src/motioncapture.cpp
  src/vicon.cpp
  src/optitrack.cpp
  src/cdp.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(libmotioncapture
  ${PCL_LIBRARIES}
  ViconDataStreamSDK_CPP
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS crazyflie crazyflie_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_crazyflie.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

