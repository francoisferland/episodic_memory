###########################################################
# Library level unit tests
###########################################################

# Fixes
#   undefined reference to "pthread_getspecific"
# This appeared on 12.04 / Fuerte
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

catkin_add_gtest(test_parameter test_parameter.cpp)

catkin_add_gtest(test_associations test_associations.cpp)

catkin_add_gtest(test_iterator test_iterator.cpp)

catkin_add_gtest(test_information test_information.cpp)

catkin_add_gtest(test_egosphere test_egosphere.cpp)

catkin_add_gtest(test_people_detection test_people_detection_sample.cpp)

#target_link_libraries(test_people_detection ${PROJECT_NAME})

###########################################################
# Node level unit tests
###########################################################

#rosbuild_add_executable(test_location_topic test_location_topic.cpp)
#target_link_libraries(test_location_topic ${PROJECT_NAME})
#rosbuild_add_gtest_build_flags(test_location_topic)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test_location_topic.launch)

#rosbuild_add_executable(test_manager_register test_manager_register.cpp)
#rosbuild_add_gtest_build_flags(test_manager_register)
#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test_manager_register.launch)
