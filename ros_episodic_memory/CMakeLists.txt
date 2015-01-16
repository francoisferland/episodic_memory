cmake_minimum_required(VERSION 2.4.6)
SET(CMAKE_VERBOSE_MAKEFILE ON)
PROJECT(ros_episodic_memory)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

FIND_PACKAGE( Qt4 REQUIRED )
FIND_PACKAGE( UtiLite REQUIRED )
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include/ros_episodic_memory 
                     ${UTILITE_INCLUDE_DIRS})

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS} -fPIC)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

SET( ROS_EPISODIC_MEMORY_MOC_SRC
    ${PROJECT_SOURCE_DIR}/include/ros_episodic_memory/episodicMemoryWrapperROS.h
)

SET (ROS_EM_SRC_CXX
	src/ros_episodic_memory.cpp
	src/episodicMemoryWrapperROS.cpp

 )
 
SET(EPISODIC_MEMORY_PLANNER
    ${PROJECT_SOURCE_DIR}/src/episodic_memory_planner.cpp
)

QT4_WRAP_CPP(ROS_EM_SRC_CXX ${ROS_EPISODIC_MEMORY_MOC_SRC} )

rosbuild_add_executable(ros_episodic_memory ${ROS_EM_SRC_CXX})

rosbuild_add_executable(episodic_memory_planner ${EPISODIC_MEMORY_PLANNER}     
                            ${PROJECT_SOURCE_DIR}/src/episodic_memory_planner_node.cpp)

rosbuild_add_library(em_planner
    ${EPISODIC_MEMORY_PLANNER}
)

rosbuild_add_boost_directories()
rosbuild_link_boost(ros_episodic_memory thread)

target_link_libraries(ros_episodic_memory 		
                      ${UTILITE_LIBRARIES})

