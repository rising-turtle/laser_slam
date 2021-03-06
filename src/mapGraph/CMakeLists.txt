PROJECT(PFG)

# SET Include Directory

INCLUDE_DIRECTORIES(
	${G2O_DIR} ${EIGEN} ${CSPARSE} ${FLIRT_DIR} ${SSPARSE_DIR} 
	${CMAKE_SOURCE_DIR}/include 
	${CMAKE_CURRENT_SOURCE_DIR}/../zhpsm 
	${CMAKE_CURRENT_SOURCE_DIR}/../zhcsm
	${CMAKE_CURRENT_SOURCE_DIR}/../zhicp
	)

# SET Lib Directory
LINK_DIRECTORIES(${FLIRT_LIB} ${G2O_LIB} ${CSM_LIB} ${PROJECT_BINARY_DIR} ${CMAKE_BINARY_DIR}/lib ${MRPT_LIB})

# SET Libraries
SET(OTHER_LIBS zhpsm zhcsm zhicp)

# LIB_LIST
SET(LIB_LIST
	FlirterNode.cpp
	PMap.cpp
	line.cpp
	MapNode.cpp
	MapGraph.cpp
)

# SRC Files
SET(SRC_FILES test.cpp)

# OUTPUT LIB
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
ADD_LIBRARY(pfg SHARED ${LIB_LIST})

# Executable files 
ADD_EXECUTABLE(mapGraph ${SRC_FILES})
TARGET_LINK_LIBRARIES(mapGraph pfg ${OTHER_LIBS} ${SSPARSE_LIBS} ${G2O_LIBS} ${FLIRT_LIBS} ${CSM_LIBS} ${MRPT_LIBS})

####### test FlirterNode perfomance
ADD_EXECUTABLE(flirt test_flirt.cpp)
TARGET_LINK_LIBRARIES(flirt pfg ${OTHER_LIBS} ${SSPARSE_LIBS} ${G2O_LIBS} ${FLIRT_LIBS} ${CSM_LIBS} ${MRPT_LIBS})

####### expr1 
#ADD_EXECUTABLE(expr1 expr1.cpp)
#TARGET_LINK_LIBRARIES(expr1 pfg ${OTHER_LIBS} ${SSPARSE_LIBS} ${G2O_LIBS} ${FLIRT_LIBS} ${CSM_LIBS} ${MRPT_LIBS})



