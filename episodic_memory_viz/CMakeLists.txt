cmake_minimum_required(VERSION 2.8.3)
project(episodic_memory_viz)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  episodic_memory_core
  roscpp
)

FIND_PACKAGE( Boost 1.46 COMPONENTS thread date_time REQUIRED )
FIND_PACKAGE( Qt4 REQUIRED )
#FIND_PACKAGE( UtiLite REQUIRED )
FIND_LIBRARY( SQLITE_LIBRARY sqlite3 )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
  LIBRARIES EpisodicMemoryVizLib 
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(EPISODIC_MEMORY_VIZ_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )
SET( LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/libs )
SET ( EPISODIC_MEMORY_PATH ${PROJECT_SOURCE_DIR}/../episodic_memory_core )

INCLUDE_DIRECTORIES(
                    ${Boost_INCLUDE_DIR} 
                    ${UTILITE_INCLUDE_DIRS}
                    ${EPISODIC_MEMORY_VIZ_INCLUDE_DIR}
                    ${EPISODIC_MEMORY_PATH}/include
                    )
                    
link_directories(${PROJECT_BINARY_DIR}
                 ${Boost_LIBRARY_DIR}
                )   
                
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS} )

SET(EPISODIC_MEMORY_VIZ_SRC_CXX
    src/edge.cpp
    src/graphWidget.cpp
    src/networkvisualizer.cpp
    src/node.cpp
)

SET(EPISODIC_MEMORY_VIZ_MOC_SRC
    ${EPISODIC_MEMORY_VIZ_INCLUDE_DIR}/graphWidget.h
    ${EPISODIC_MEMORY_VIZ_INCLUDE_DIR}/networkvisualizer.h
    ${EPISODIC_MEMORY_VIZ_INCLUDE_DIR}/myApp.h
)

QT4_WRAP_CPP(EPISODIC_MEMORY_VIZ_SRC_CXX ${EPISODIC_MEMORY_VIZ_MOC_SRC} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

ADD_EXECUTABLE(EpisodicMemoryViz
		src/main.cpp
	    ${EPISODIC_MEMORY_VIZ_SRC_CXX} 
	    )	  
	      
SET(LIBRARIES 
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES} 
    ${UTILITE_LIBRARIES}
)
    
TARGET_LINK_LIBRARIES(EpisodicMemoryViz
                ${LIBRARIES}
)
              
ADD_LIBRARY(EpisodicMemoryVizLib SHARED
			    ${EPISODIC_MEMORY_VIZ_SRC_CXX} 
)    
TARGET_LINK_LIBRARIES(EpisodicMemoryVizLib
                ${LIBRARIES}
)
              
SET_TARGET_PROPERTIES(EpisodicMemoryVizLib 
      PROPERTIES OUTPUT_NAME EpisodicMemoryViz)
