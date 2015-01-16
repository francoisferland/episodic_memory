cmake_minimum_required(VERSION 2.8.3)
project(ros_episodic_memory)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  episodic_memory_core
  hbba_msgs
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
add_message_files(
  FILES
  anticipatedEvent.msg
  contextRepresentation.msg
  learningMode.msg
  newChannel.msg
  updatedWeight.msg
  categoryActivation.msg
  inputData.msg
  newCategory.msg
  recalledEpisode.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  setLearningRateParameter.srv
  setRelevanceParameter.srv
  setVigilanceParameter.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

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
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp
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
  ${PROJECT_SOURCE_DIR}/include/ros_episodic_memory 
  #${UTILITE_INCLUDE_DIRS}
)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS} -fPIC)

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

add_executable(ros_episodic_memory ${ROS_EM_SRC_CXX})
add_dependencies(ros_episodic_memory ros_episodic_memory_generate_messages_cpp)
add_dependencies(ros_episodic_memory hbba_msgs_generate_messages_cpp)

add_executable(episodic_memory_planner ${EPISODIC_MEMORY_PLANNER}     
                            ${PROJECT_SOURCE_DIR}/src/episodic_memory_planner_node.cpp)
add_dependencies(episodic_memory_planner ros_episodic_memory_generate_messages_cpp)
add_dependencies(episodic_memory_planner hbba_msgs_generate_messages_cpp)

add_library(em_planner
    ${EPISODIC_MEMORY_PLANNER}
)

target_link_libraries(episodic_memory_planner
                      EM_CORE
                      ${catkin_LIBRARIES}
)
target_link_libraries(ros_episodic_memory 		
                      EM_CORE
                      ${catkin_LIBRARIES}
)


## Declare a cpp library
# add_library(test
#   src/${PROJECT_NAME}/test.cpp
# )

## Declare a cpp executable
# add_executable(test_node src/test_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(test_node test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(test_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS test test_node
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

SET(EPISODIC_MEMORY_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
SET(EPISODIC_MEMORY_DATABASE_PATH ${PROJECT_SOURCE_DIR}/database/episodicMemory.db)
SET(EPISODIC_MEMORY_TEST_DATABASE_PATH ${PROJECT_SOURCE_DIR}/database/episodicMemoryTest.db)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

