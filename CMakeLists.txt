cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_safety)

find_package( catkin REQUIRED COMPONENTS nodelet roscpp geometry_msgs turtlebot3_msgs tf )

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES turtlebot3_safety 
  CATKIN_DEPENDS nodelet roscpp geometry_msgs std_msgs turtlebot3_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable( turtlebot3_safety src/turtlebot3_safety.cpp )
target_link_libraries( turtlebot3_safety ${catkin_LIBRARIES}  )
add_dependencies( turtlebot3_safety turtlebot3_msgs_generate_messages_cpp )

install(TARGETS turtlebot3_safety
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES turtlebot3_safety.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

