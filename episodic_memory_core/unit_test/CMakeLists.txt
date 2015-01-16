# Setup testing

FIND_PACKAGE(GTest REQUIRED)

enable_testing(true)
include_directories(${GTEST_INCLUDE_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

include_directories (
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR} 
    ${EPISODIC_MEMORY_INCLUDE_DIR}
    ${UTILITE_INCLUDE_DIRS}
    ) 

link_directories(
    ${PROJECT_BINARY_DIR}
    ${Boost_LIBRARY_DIR}
    )    
        
add_executable(test_init_model test_init_model.cpp)
target_link_libraries(test_init_model gtest gtest_main EM_CORE)
add_test(test_init_model test_init_model)

add_executable(test_resonance test_resonance.cpp)
target_link_libraries(test_resonance gtest gtest_main EM_CORE)
add_test(test_resonance test_resonance)
    
