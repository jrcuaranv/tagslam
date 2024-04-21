cmake_minimum_required(VERSION 3.0.2)
project(tagslam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-g -Wall -Wno-misleading-indentation -Wno-int-in-bool-context)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2_ros
)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(GTSAM REQUIRED QUIET)

catkin_package(
	INCLUDE_DIRS include
	#DEPENDS OpenCV
	)

include_directories(
  include
  ${GTSAM_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(optimizer
src/gtsam_optimizer.cpp
)
target_link_libraries(optimizer ${catkin_LIBRARIES} gtsam ${Boost_LIBRARIES})

add_executable(tagslam
src/tagslam_node.cpp
)
target_link_libraries(tagslam optimizer ${catkin_LIBRARIES} gtsam ${Boost_LIBRARIES})

install(TARGETS tagslam
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
data
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
FILES_MATCHING PATTERN "*.csv"
)

install(DIRECTORY
launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY
params
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
FILES_MATCHING PATTERN "*.yaml"
)

catkin_install_python(PROGRAMS
scripts/get_tags_data.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
