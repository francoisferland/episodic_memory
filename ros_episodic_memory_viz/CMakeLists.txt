cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/libs)

FIND_PACKAGE( Qt4 REQUIRED QtCore QtGui)
FIND_PACKAGE( UtiLite REQUIRED )

INCLUDE_DIRECTORIES(${UTILITE_INCLUDE_DIRS})

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS} )

SET(ROS_EPISODIC_MEMORY_VIZ_CXX
    src/ros_networkvisualizer.cpp
)

SET(ROS_EPISODIC_MEMORY_VIZ_MOC_SRC
    ${PROJECT_SOURCE_DIR}/include/ros_networkvisualizer.h
)

QT4_WRAP_CPP(ROS_EPISODIC_MEMORY_VIZ_CXX ${ROS_EPISODIC_MEMORY_VIZ_MOC_SRC} )

rosbuild_add_executable(ros_episodic_memory_viz
		src/main.cpp
	    ${ROS_EPISODIC_MEMORY_VIZ_CXX} 
	    )
	    
SET(LIBRARIES 	
    ${UTILITE_LIBRARIES}
)

rosbuild_add_boost_directories()
rosbuild_link_boost(ros_episodic_memory_viz thread )     
TARGET_LINK_LIBRARIES(  ros_episodic_memory_viz	
                        ${LIBRARIES}	
                      )