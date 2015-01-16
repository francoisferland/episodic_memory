cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

FIND_PACKAGE( Boost 1.46 COMPONENTS thread date_time REQUIRED )
FIND_PACKAGE( Qt4 REQUIRED )
FIND_PACKAGE( UtiLite REQUIRED )

SET(EPISODIC_MEMORY_VIZ_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )
SET( LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/libs )
SET ( EPISODIC_MEMORY_PATH ${PROJECT_SOURCE_DIR}/../episodic_memory_core )

FIND_LIBRARY( EPISODIC_MEMORY_LIBRARY NAMES libEM_CORE.so PATHS ${EPISODIC_MEMORY_PATH}/libs)

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
    ${EPISODIC_MEMORY_LIBRARY}
    ${QT_LIBRARIES}
    ${Boost_LIBRARIES} 
    ${UTILITE_LIBRARIES}
    )
    
TARGET_LINK_LIBRARIES(  EpisodicMemoryViz
                ${LIBRARIES}
              )
              
ADD_LIBRARY(EpisodicMemoryVizLib SHARED
			${EPISODIC_MEMORY_VIZ_SRC_CXX} 
			)    
TARGET_LINK_LIBRARIES(  EpisodicMemoryVizLib
                ${LIBRARIES}
              )
              
SET_TARGET_PROPERTIES(EpisodicMemoryVizLib 
      PROPERTIES OUTPUT_NAME EpisodicMemoryViz)
              