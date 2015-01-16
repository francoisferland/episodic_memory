SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs )
SET( EXECUTABLE_OUTPUT_PATH         ${PROJECT_SOURCE_DIR}/bin  )

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${PROJECT_SOURCE_DIR} 
                    ${PROJECT_BINARY_DIR} 
                    ${Boost_INCLUDE_DIR}
                    ${UTILITE_INCLUDE_DIRS}
                    ${EPISODIC_MEMORY_INCLUDE_DIR}
                   )
                    
link_directories(${PROJECT_BINARY_DIR}
                 ${Boost_LIBRARY_DIR}
                )    
    
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS} )

IF(UNIX)
	ADD_DEFINITIONS( -fPIC)
ENDIF()

SET(EM_CORE_SRC_CXX

    categoryART.cpp
    channelART.cpp
    channelDao.cpp
    channelObj.cpp
    dao.cpp
    daoFactory.cpp
    episodeLayer.cpp
    episodicMemoryCore.cpp
    eventLayer.cpp
    inputDao.cpp
    inputLayer.cpp
    inputObj.cpp
    layerART.cpp
    memoryDecay.cpp
    patternRecognizerDao.cpp
    patternRecognizerObj.cpp
    weightDao.cpp
    weightObj.cpp
    
    )
    
#files which need to be moc'ed by qt
SET(QT_EM_CORE_HEADERS
    ${EPISODIC_MEMORY_INCLUDE_DIR}/layerART.h 
    ${EPISODIC_MEMORY_INCLUDE_DIR}/episodeLayer.h
    ${EPISODIC_MEMORY_INCLUDE_DIR}/episodicMemoryCore.h
    )

QT4_WRAP_CPP(EM_CORE_SRC_CXX ${QT_EM_CORE_HEADERS} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)


IF(UNIX)
    rosbuild_add_library (EM_CORE    SHARED                 
            ${EM_CORE_SRC_CXX}
            )
ELSE()
   add_library (EM_CORE                      
            ${EM_CORE_SRC_CXX}
            )         
ENDIF()

    
target_link_libraries ( EM_CORE
                        ${QT_LIBRARIES}
                        ${Boost_LIBRARIES}
                        ${SQLITE_LIBRARY}
                        ${UTILITE_LIBRARIES}
                        )
                        
INSTALL (TARGETS EM_CORE DESTINATION ${PROJECT_SOURCE_DIR}/libs)