cmake_minimum_required(VERSION 2.8.3)
project(pioneer_simulator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
#add_message_files(FILES arrayData.msg gridData.msg Path.msg planData.msg stateInfo.msg vehInfo.msg)

# Generate services in the 'srv' folder
#add_service_files(FILES CommStatus.srv GetMyPlan.srv Removal.srv)
 #add_service_files(
   #FILES
  # AddTwoInts.srv
 #)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs tf message_runtime
)


###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(pioneerSimulator src/pioneerSimulator.cpp)
add_executable(veh_status src/veh_status.cpp)
add_executable(scanReader src/scanReader.cpp)
add_executable(tf_listen src/tf_listen.cpp)
add_executable(enterTruckMode src/enterTruckMode.cpp)
add_executable(local src/local.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(pioneerSimulator ${catkin_LIBRARIES})
target_link_libraries(veh_status ${catkin_LIBRARIES})
target_link_libraries(scanReader ${catkin_LIBRARIES})
target_link_libraries(tf_listen ${catkin_LIBRARIES})
target_link_libraries(enterTruckMode ${catkin_LIBRARIES})
target_link_libraries(local ${catkin_LIBRARIES})



#add_executable(add_two_ints_server src/add_two_ints_server.cpp)
#target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_server pioneer_simulator_generate_service_cpp)

#add_executable(add_two_ints_client src/add_two_ints_client.cpp)
#target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_client pioneer_simulator_generate_service_cpp)


# This prevents files being compiled before the necessary messages
# get generated!
#add_dependencies(agvController agv_control_gencpp)
#add_dependencies(pathPlanner agv_control_gencpp)
#add_dependencies(poseEstimator agv_control_gencpp)
#add_dependencies(motionSimSR agv_control_gencpp)
#add_dependencies(motionSimSR_multilayer agv_control_gencpp)

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
# install(TARGETS agv_control agv_control_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_agv_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
