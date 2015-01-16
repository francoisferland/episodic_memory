cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)
IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

project(episodicMemoryCore)

SET (episodicMemoryCore_VERSION_MAJOR 3)
SET (episodicMemoryCore_VERSION_MINOR 0)

rosbuild_init()

FIND_PACKAGE( Boost 1.46 COMPONENTS thread date_time REQUIRED )
FIND_PACKAGE( Qt4 REQUIRED )
FIND_PACKAGE( UtiLite REQUIRED )
FIND_LIBRARY( SQLITE_LIBRARY sqlite3 )

#IF(UtiLite_FOUND)

#ELSE()
#	MESSAGE(STATUS "[WARNING] UTILITE_INCLUDE_DIRS =" ${UTILITE_INCLUDE_DIRS})
#ENDIF()

SET(EPISODIC_MEMORY_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
SET(EPISODIC_MEMORY_DATABASE_PATH ${PROJECT_SOURCE_DIR}/database/episodicMemory.db)
SET(EPISODIC_MEMORY_TEST_DATABASE_PATH ${PROJECT_SOURCE_DIR}/database/episodicMemoryTest.db)

# configure a header file to pass some of the CMake settings
# to the source code
CONFIGURE_FILE (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${EPISODIC_MEMORY_INCLUDE_DIR}/config.h"
  )
  
add_subdirectory("src")
add_subdirectory("unit_test")
