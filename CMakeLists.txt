cmake_minimum_required(VERSION 2.8.3)
project(s3000_laser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  message_generation
  self_test
  diagnostic_msgs
  diagnostic_updater
)

add_service_files(
   FILES
   enable_disable.srv
)

generate_messages(
   DEPENDENCIES
   s3000_laser
)

catkin_package(

  INCLUDE_DIRS include

  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    tf
    message_generation  
    self_test
    diagnostic_msgs
    diagnostic_updater
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(s3000_laser_node src/s3000_laser.cc src/sicks3000.cc src/SerialDevice.cc)

add_dependencies(s3000_laser_node s3000_laser_generate_messages_cpp)

target_link_libraries(s3000_laser_node
   ${catkin_LIBRARIES}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS s3000_laser_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES
  launch/s3000_laser.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


